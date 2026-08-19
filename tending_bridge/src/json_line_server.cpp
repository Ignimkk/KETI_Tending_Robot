#include "tending_bridge/json_line_server.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <utility>
#include <vector>

namespace tending_bridge
{

namespace
{
// 라인 하나의 상한. 프로토콜상 state 메시지가 수백 바이트이므로 여유 있게 잡되,
// 악의적/오동작 클라이언트가 무한히 버퍼를 키우지 못하게 막는다.
constexpr size_t kMaxLineBytes = 64 * 1024;

bool set_nonblocking(int fd)
{
  int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}
}  // namespace

JsonLineServer::JsonLineServer() = default;

JsonLineServer::~JsonLineServer()
{
  stop();
}

bool JsonLineServer::start(
  const std::string & bind_address, uint16_t port, int max_clients, std::string & err)
{
  if (running_.load()) {
    err = "이미 실행 중입니다.";
    return false;
  }
  max_clients_ = max_clients > 0 ? max_clients : 1;

  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    err = std::string("socket() 실패: ") + std::strerror(errno);
    return false;
  }

  int one = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = ::htons(port);
  if (::inet_pton(AF_INET, bind_address.c_str(), &addr.sin_addr) != 1) {
    err = "bind_address 형식 오류: " + bind_address;
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }

  if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    err = std::string("bind(") + bind_address + ":" + std::to_string(port) + ") 실패: " +
      std::strerror(errno);
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }

  if (::listen(listen_fd_, 4) < 0) {
    err = std::string("listen() 실패: ") + std::strerror(errno);
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }

  if (!set_nonblocking(listen_fd_) || ::pipe(wake_fds_) != 0 ||
    !set_nonblocking(wake_fds_[0]) || !set_nonblocking(wake_fds_[1]))
  {
    err = std::string("논블로킹 설정 실패: ") + std::strerror(errno);
    stop();
    return false;
  }

  running_.store(true);
  thread_ = std::thread(&JsonLineServer::run, this);
  return true;
}

void JsonLineServer::stop()
{
  const bool was_running = running_.exchange(false);
  if (was_running) {
    wake();  // poll 에서 즉시 빠져나오게
  }
  if (thread_.joinable()) {
    thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto & kv : clients_) {
      if (kv.second.fd >= 0) {
        ::close(kv.second.fd);
      }
    }
    clients_.clear();
  }

  if (listen_fd_ >= 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
  }
  for (int & fd : wake_fds_) {
    if (fd >= 0) {
      ::close(fd);
      fd = -1;
    }
  }
}

void JsonLineServer::wake()
{
  if (wake_fds_[1] >= 0) {
    const char b = 'x';
    ssize_t n = ::write(wake_fds_[1], &b, 1);
    (void)n;  // 파이프가 차 있으면 이미 깨어날 예정이므로 무시해도 안전
  }
}

void JsonLineServer::broadcast(const std::string & json_line)
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (clients_.empty()) {
      return;
    }
    for (auto & kv : clients_) {
      kv.second.tx += json_line;
      kv.second.tx += '\n';
    }
  }
  wake();
}

void JsonLineServer::send_to(ClientId id, const std::string & json_line)
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = clients_.find(id);
    if (it == clients_.end()) {
      return;
    }
    it->second.tx += json_line;
    it->second.tx += '\n';
  }
  wake();
}

size_t JsonLineServer::client_count()
{
  std::lock_guard<std::mutex> lock(mtx_);
  return clients_.size();
}

bool JsonLineServer::last_rx_elapsed(ClientId id, std::chrono::milliseconds & out)
{
  std::lock_guard<std::mutex> lock(mtx_);
  const auto it = clients_.find(id);
  if (it == clients_.end()) {
    return false;
  }
  out = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - it->second.last_rx);
  return true;
}

void JsonLineServer::log(int level, const std::string & msg)
{
  if (on_log_) {
    on_log_(level, msg);
  }
}

void JsonLineServer::run()
{
  std::vector<pollfd> pfds;
  std::vector<ClientId> ids;  // pfds[2..] 와 인덱스 대응

  while (running_.load()) {
    pfds.clear();
    ids.clear();

    pfds.push_back(pollfd{listen_fd_, POLLIN, 0});
    pfds.push_back(pollfd{wake_fds_[0], POLLIN, 0});

    {
      std::lock_guard<std::mutex> lock(mtx_);
      for (auto & kv : clients_) {
        short events = POLLIN;
        if (!kv.second.tx.empty()) {
          events |= POLLOUT;
        }
        pfds.push_back(pollfd{kv.second.fd, events, 0});
        ids.push_back(kv.first);
      }
    }

    // 100ms 상한: 데드맨 판정 주기를 위해 클라이언트가 조용해도 주기적으로 돌아온다.
    const int rc = ::poll(pfds.data(), pfds.size(), 100);
    if (rc < 0) {
      if (errno == EINTR) {
        continue;
      }
      log(2, std::string("poll() 실패: ") + std::strerror(errno));
      break;
    }
    if (!running_.load()) {
      break;
    }

    if (pfds[0].revents & POLLIN) {
      accept_new();
    }
    if (pfds[1].revents & POLLIN) {
      char drain[256];
      while (::read(wake_fds_[0], drain, sizeof(drain)) > 0) {}
    }

    for (size_t i = 0; i < ids.size(); ++i) {
      const pollfd & p = pfds[i + 2];
      if (p.revents == 0) {
        continue;
      }
      const ClientId id = ids[i];

      bool alive = !(p.revents & (POLLERR | POLLHUP | POLLNVAL));

      if (alive && (p.revents & POLLOUT)) {
        std::lock_guard<std::mutex> lock(mtx_);
        auto it = clients_.find(id);
        alive = (it != clients_.end()) && flush_tx(it->second);
      }

      if (alive && (p.revents & POLLIN)) {
        // handle_readable 은 on_line_ 콜백을 부르므로 뮤텍스를 잡은 채로 호출하지 않는다.
        // fd 만 미리 꺼내오고, 수신 버퍼 갱신 시에만 내부에서 다시 잠근다.
        int fd = -1;
        {
          std::lock_guard<std::mutex> lock(mtx_);
          auto it = clients_.find(id);
          if (it != clients_.end()) {
            fd = it->second.fd;
          }
        }
        alive = (fd >= 0) && handle_readable(id, fd);
      }

      if (!alive) {
        drop_client(id);
      }
    }
  }

  log(0, "TCP 서버 IO 스레드 종료");
}

void JsonLineServer::accept_new()
{
  while (true) {
    sockaddr_in peer{};
    socklen_t len = sizeof(peer);
    const int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr *>(&peer), &len);
    if (fd < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        log(1, std::string("accept() 실패: ") + std::strerror(errno));
      }
      return;
    }

    char ip[INET_ADDRSTRLEN] = {0};
    ::inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));
    const std::string peer_str = std::string(ip) + ":" + std::to_string(::ntohs(peer.sin_port));

    if (!allowed_client_ip_.empty() && allowed_client_ip_ != ip) {
      log(1, "허용되지 않은 클라이언트 IP — 거부: " + peer_str);
      ::close(fd);
      continue;
    }

    size_t count = 0;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      count = clients_.size();
    }
    if (static_cast<int>(count) >= max_clients_) {
      log(1, "클라이언트 수 상한(" + std::to_string(max_clients_) + ") 초과 — 거부: " + peer_str);
      ::close(fd);
      continue;
    }

    if (!set_nonblocking(fd)) {
      log(1, "클라이언트 논블로킹 설정 실패 — 거부: " + peer_str);
      ::close(fd);
      continue;
    }
    int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));  // 상태 스트림 지연 최소화

    ClientId id = 0;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      id = next_id_++;
      Client c;
      c.fd = fd;
      c.last_rx = std::chrono::steady_clock::now();
      clients_.emplace(id, std::move(c));
    }
    log(0, "클라이언트 연결: " + peer_str + " (id=" + std::to_string(id) + ")");
    if (on_connect_) {
      on_connect_(id);
    }
  }
}

bool JsonLineServer::handle_readable(ClientId id, int fd)
{
  char buf[4096];
  while (true) {
    const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
    if (n > 0) {
      std::vector<std::string> lines;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        auto it = clients_.find(id);
        if (it == clients_.end()) {
          return false;
        }
        Client & real = it->second;
        real.last_rx = std::chrono::steady_clock::now();
        real.rx.append(buf, static_cast<size_t>(n));

        size_t pos;
        while ((pos = real.rx.find('\n')) != std::string::npos) {
          std::string line = real.rx.substr(0, pos);
          real.rx.erase(0, pos + 1);
          if (!line.empty() && line.back() == '\r') {
            line.pop_back();  // CRLF 로 보내는 클라이언트 허용
          }
          if (!line.empty()) {
            lines.push_back(std::move(line));
          }
        }

        if (real.rx.size() > kMaxLineBytes) {
          log(1, "라인 길이 상한 초과 — 연결 종료 (id=" + std::to_string(id) + ")");
          return false;
        }
      }
      // 콜백은 뮤텍스 밖에서 — 핸들러가 send_to() 를 호출해도 교착되지 않게.
      for (const auto & line : lines) {
        if (on_line_) {
          on_line_(id, line);
        }
      }
      continue;
    }

    if (n == 0) {
      return false;  // 정상 종료(FIN)
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return true;   // 더 읽을 것 없음
    }
    if (errno == EINTR) {
      continue;
    }
    return false;
  }
}

bool JsonLineServer::flush_tx(Client & c)
{
  while (!c.tx.empty()) {
    const ssize_t n = ::send(c.fd, c.tx.data(), c.tx.size(), MSG_NOSIGNAL);
    if (n > 0) {
      c.tx.erase(0, static_cast<size_t>(n));
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return true;   // 소켓 버퍼 포화 — 다음 POLLOUT 에서 이어서
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

void JsonLineServer::drop_client(ClientId id)
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = clients_.find(id);
    if (it == clients_.end()) {
      return;
    }
    if (it->second.fd >= 0) {
      ::close(it->second.fd);
    }
    clients_.erase(it);
  }
  // 콜백은 반드시 락 밖에서 — 핸들러가 broadcast()/send_to() 를 불러도 교착되지 않게.
  log(0, "클라이언트 해제 (id=" + std::to_string(id) + ")");
  if (on_disconnect_) {
    on_disconnect_(id);
  }
}

}  // namespace tending_bridge
