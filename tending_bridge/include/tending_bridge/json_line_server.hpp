// JsonLineServer — 개행 구분 JSON 라인을 주고받는 경량 TCP 서버.
//
// doc/PROTOCOL.md §1 의 전송 계층 구현. ROS 에 의존하지 않는 순수 POSIX 소켓 계층으로,
// 자체 스레드에서 poll() 루프를 돌며 다음을 담당한다.
//   - accept / 클라이언트 수 제한
//   - 부분 수신 라인 누적 후 완전한 라인 단위로 콜백
//   - broadcast(전체) / send_to(특정 클라이언트) 송신 큐
//   - 클라이언트별 마지막 수신 시각 추적(데드맨 판정용, PROTOCOL §5.3)
//
// 스레드 규약: on_line/on_connect/on_disconnect 콜백은 모두 내부 IO 스레드에서 호출된다.
// 호출자는 콜백 안에서 오래 걸리는 작업을 하지 말고, ROS 처리로 넘길 것.

#ifndef TENDING_BRIDGE__JSON_LINE_SERVER_HPP_
#define TENDING_BRIDGE__JSON_LINE_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace tending_bridge
{

// 클라이언트 식별자(내부 fd 와 무관한 단조 증가 ID).
using ClientId = uint64_t;

class JsonLineServer
{
public:
  // 완전한 한 줄(개행 제외)을 수신했을 때.
  using LineHandler = std::function<void (ClientId, const std::string &)>;
  using ConnectHandler = std::function<void (ClientId)>;
  using DisconnectHandler = std::function<void (ClientId)>;
  // 내부 로그를 호출자(ROS 로거)로 흘려보내기 위한 싱크. level: 0=info,1=warn,2=error
  using LogHandler = std::function<void (int, const std::string &)>;

  JsonLineServer();
  ~JsonLineServer();

  JsonLineServer(const JsonLineServer &) = delete;
  JsonLineServer & operator=(const JsonLineServer &) = delete;

  void set_line_handler(LineHandler h) { on_line_ = std::move(h); }
  void set_connect_handler(ConnectHandler h) { on_connect_ = std::move(h); }
  void set_disconnect_handler(DisconnectHandler h) { on_disconnect_ = std::move(h); }
  void set_log_handler(LogHandler h) { on_log_ = std::move(h); }
  // 비어 있지 않으면 해당 IPv4 주소의 클라이언트만 허용한다.
  void set_allowed_client_ip(const std::string & ip) { allowed_client_ip_ = ip; }

  // 리스닝 시작 후 IO 스레드 기동. 실패 시 false 와 err 채움.
  bool start(const std::string & bind_address, uint16_t port, int max_clients, std::string & err);

  // IO 스레드 정지 및 모든 소켓 정리. 재진입 안전.
  void stop();

  // 모든 클라이언트에게 한 줄 송신(개행은 내부에서 붙인다).
  void broadcast(const std::string & json_line);

  // 특정 클라이언트에게 한 줄 송신. 대상이 없으면 무시.
  void send_to(ClientId id, const std::string & json_line);

  size_t client_count();

  // 특정 클라이언트의 마지막 수신 이후 경과 시간. 명령 소유자별 데드맨 판정에 사용한다.
  // 해당 클라이언트가 이미 끊겼으면 false 를 반환한다.
  bool last_rx_elapsed(ClientId id, std::chrono::milliseconds & out);

private:
  struct Client
  {
    int fd{-1};
    std::string rx;                 // 부분 라인 누적 버퍼
    std::string tx;                 // 미송신 잔여 바이트
    std::chrono::steady_clock::time_point last_rx;
  };

  void run();                       // IO 스레드 본체
  void accept_new();
  bool handle_readable(ClientId id, int fd);       // false 면 연결 종료
  bool flush_tx(Client & c);                       // false 면 연결 종료. mtx_ 보유 상태로 호출
  void drop_client(ClientId id);                   // 소켓 정리 + on_disconnect_ 발행(락 밖에서 호출)
  void log(int level, const std::string & msg);
  void wake();                      // 송신 큐에 넣은 뒤 poll 을 깨움

  int listen_fd_{-1};
  int wake_fds_[2]{-1, -1};         // self-pipe: 송신/종료를 poll 에 알림
  int max_clients_{2};
  std::string allowed_client_ip_;

  std::thread thread_;
  std::atomic<bool> running_{false};

  std::mutex mtx_;                  // clients_, next_id_ 보호
  std::map<ClientId, Client> clients_;
  ClientId next_id_{1};

  LineHandler on_line_;
  ConnectHandler on_connect_;
  DisconnectHandler on_disconnect_;
  LogHandler on_log_;
};

}  // namespace tending_bridge

#endif  // TENDING_BRIDGE__JSON_LINE_SERVER_HPP_
