#!/usr/bin/env python3
"""tending_bridge 프로토콜 검증용 목(mock) 클라이언트.

Windows GUI 없이 doc/PROTOCOL.md v1 왕복을 확인한다.
Windows 측 RosBridgeClient 가 해야 할 동작(1초 ping, 라인 프레이밍, id 매칭)을 그대로 흉내낸다.

사용 예:
    python3 mock_ui_client.py                        # 상태 스트림만 관찰
    python3 mock_ui_client.py --goto home            # 포즈 이동 1회
    python3 mock_ui_client.py --jog 6 --delta 30     # J6 를 +30deg 조그
    python3 mock_ui_client.py --run --views 4        # 검사 실행 후 result 대기
    python3 mock_ui_client.py --run --cancel-after 3 # 3초 뒤 취소
    python3 mock_ui_client.py --stop                 # 비상정지만 보내고 종료
    python3 mock_ui_client.py --no-ping --run        # 데드맨 동작 확인(ping 중단)
"""

import argparse
import json
import socket
import sys
import threading
import time
import uuid

SCHEMA_VERSION = 1


class MockClient:
    def __init__(self, host, port, send_ping=True, quiet_state=False):
        self.host = host
        self.port = port
        self.send_ping = send_ping
        self.quiet_state = quiet_state
        self.sock = None
        self.rx = b""
        self.running = False
        self.lock = threading.Lock()
        self.pending = {}          # cmd id -> cmd 이름
        self.results = {}          # cmd id -> result 메시지
        self.last_state = None
        self.state_count = 0

    # ---------- 연결 ----------
    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=5)
        self.sock.settimeout(0.5)
        self.running = True
        print(f"[연결] {self.host}:{self.port}")
        threading.Thread(target=self._rx_loop, daemon=True).start()
        if self.send_ping:
            threading.Thread(target=self._ping_loop, daemon=True).start()

    def close(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass

    # ---------- 송신 ----------
    def send_cmd(self, cmd, args=None):
        cmd_id = str(uuid.uuid4())[:8]
        msg = {
            "v": SCHEMA_VERSION,
            "type": "cmd",
            "id": cmd_id,
            "ts": int(time.time() * 1000),
            "cmd": cmd,
            "args": args or {},
        }
        line = json.dumps(msg, ensure_ascii=False) + "\n"
        with self.lock:
            self.pending[cmd_id] = cmd
            self.sock.sendall(line.encode("utf-8"))
        if cmd != "ping":
            print(f"[송신] {cmd} id={cmd_id} args={args or {}}")
        return cmd_id

    def _ping_loop(self):
        while self.running:
            try:
                self.send_cmd("ping")
            except OSError:
                return
            time.sleep(1.0)

    # ---------- 수신 ----------
    def _rx_loop(self):
        while self.running:
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                print("[연결] 서버가 연결을 종료했습니다.")
                self.running = False
                break

            self.rx += chunk
            while b"\n" in self.rx:
                raw, self.rx = self.rx.split(b"\n", 1)
                if not raw.strip():
                    continue
                try:
                    self._on_message(json.loads(raw.decode("utf-8")))
                except (ValueError, UnicodeDecodeError) as exc:
                    print(f"[오류] 파싱 실패: {exc} / {raw[:120]!r}")

    def _on_message(self, msg):
        mtype = msg.get("type")

        if mtype == "state":
            self.last_state = msg
            self.state_count += 1
            if not self.quiet_state:
                self._print_state(msg)

        elif mtype == "event":
            print(f"[이벤트] {msg.get('level','?').upper():5s} {msg.get('code')} — {msg.get('message')}")

        elif mtype == "ack":
            name = self.pending.get(msg.get("id"), "?")
            if name != "ping":
                mark = "OK" if msg.get("ok") else "거부"
                print(f"[ACK ] {name} → {mark} {msg.get('message','')}")

        elif mtype == "result":
            name = self.pending.get(msg.get("id"), "?")
            print(
                f"[결과] {name} → {msg.get('status')} success={msg.get('success')} "
                f"'{msg.get('message','')}'"
            )
            if msg.get("dataset_path"):
                print(f"       dataset={msg['dataset_path']} images={msg.get('image_count')}")
            self.results[msg.get("id")] = msg

    def _print_state(self, s):
        # 상태는 초당 10개가 오므로 1초에 1줄만 찍는다.
        if self.state_count % 10 != 1:
            return
        link = s.get("link", {})
        safety = s.get("safety", {})
        job = s.get("job", {})
        joints = " ".join(f"{v:7.2f}" for v in s.get("joint_deg", []))
        flags = []
        if safety.get("e_stop"):
            flags.append("E-STOP")
        if safety.get("robot_error"):
            flags.append(f"ERR({safety.get('error_code')})")
        if safety.get("safetyguard"):
            flags.append("GUARD")
        job_str = ""
        if job.get("active"):
            job_str = (f" | job={job.get('cmd')} {job.get('phase')} "
                       f"{job.get('progress', 0) * 100:.0f}% cap={job.get('captured')}")
        print(
            f"[상태] driver={link.get('driver')} sct={link.get('sct')} svr={link.get('svr')}"
            f" | J[{joints}]{job_str}"
            + (f" | {' '.join(flags)}" if flags else "")
        )

    def wait_result(self, cmd_id, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if cmd_id in self.results:
                return self.results[cmd_id]
            if not self.running:
                return None
            time.sleep(0.1)
        return None


def main():
    ap = argparse.ArgumentParser(description="tending_bridge 목 클라이언트")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5901)
    ap.add_argument("--run", action="store_true", help="run_inspection 실행")
    ap.add_argument("--views", type=int, default=4, help="검사 뷰 수")
    ap.add_argument("--machine", default="mc_a")
    ap.add_argument("--goto", metavar="POSE", help="명명 포즈로 이동 (home/init/approach/inspect/retreat)")
    ap.add_argument("--jog", type=int, metavar="JOINT", help="조그할 관절 번호 1..6")
    ap.add_argument("--delta", type=float, default=10.0, help="조그 증분 (deg)")
    ap.add_argument("--stop", action="store_true", help="비상정지 전송")
    ap.add_argument("--cancel-after", type=float, metavar="SEC", help="N초 뒤 실행 중 명령 취소")
    ap.add_argument("--timeout", type=float, default=180.0, help="result 대기 상한 (초)")
    ap.add_argument("--no-ping", action="store_true", help="ping 중단(데드맨 동작 확인용)")
    ap.add_argument("--quiet", action="store_true", help="state 출력 억제")
    args = ap.parse_args()

    client = MockClient(args.host, args.port, send_ping=not args.no_ping, quiet_state=args.quiet)
    try:
        client.connect()
    except OSError as exc:
        print(f"[오류] 연결 실패: {exc}")
        print("       tending_bridge 가 실행 중인지, 포트/방화벽 설정을 확인하세요.")
        return 1

    time.sleep(0.5)   # 연결 직후 스냅샷 state 수신 대기
    if client.last_state is None:
        print("[경고] 연결 직후 state 를 받지 못했습니다 (PROTOCOL §6 위반).")

    cmd_id = None
    try:
        if args.stop:
            cmd_id = client.send_cmd("stop")
            time.sleep(1.0)

        elif args.goto:
            cmd_id = client.send_cmd("goto_pose", {"pose_name": args.goto, "velocity_deg_s": 20.0})

        elif args.jog:
            cmd_id = client.send_cmd(
                "jog", {"joint": args.jog, "delta_deg": args.delta, "velocity_deg_s": 20.0})

        elif args.run:
            cmd_id = client.send_cmd("run_inspection", {
                "machine_id": args.machine,
                "scenario": 1,
                "angle_start_deg": 0.0,
                "angle_end_deg": 360.0,
                "num_views": args.views,
                "inspect_distance_mm": 100.0,
            })
            if args.cancel_after:
                def _cancel():
                    time.sleep(args.cancel_after)
                    client.send_cmd("cancel", {"target_id": cmd_id})
                threading.Thread(target=_cancel, daemon=True).start()

        if cmd_id and not args.stop:
            print(f"[대기] result 수신 대기 중 (최대 {args.timeout:.0f}초)...")
            res = client.wait_result(cmd_id, args.timeout)
            if res is None:
                print("[오류] result 를 받지 못했습니다 (타임아웃 또는 연결 종료).")
                return 2
        elif not cmd_id:
            print("[관찰] 상태 스트림만 출력합니다. Ctrl+C 로 종료.")
            while client.running:
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n[종료] 사용자 중단")
    finally:
        client.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
