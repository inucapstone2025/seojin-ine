import httpx
from pathlib import Path
from datetime import datetime
import zipfile
import io
import socket

PROJECT_ROOT = Path(__file__).parent.parent.parent
ESP32_TIMEOUT = 25

def capture_from_raspberry_pi_with_esp32(config, name="user", gender="U"):
    base = config["mode"]["pi_api_base"]
    token = config["mode"]["capture_token"]

    # ESP32 정보 config에서 가져오기
    esp32_host = config["camera"]["ip"]
    esp32_port = config["camera"]["port"]

    # 세션 저장 폴더
    now = datetime.now()
    date_str = now.strftime("%Y%m%d")
    time_str = now.strftime("%H%M%S")
    session_dir = PROJECT_ROOT / "data" / date_str / f"{name}"
    
    raw_dir = session_dir / "raw"
    filtered_dir = session_dir / "filtered"
    aligned_dir = session_dir / "aligned"
    mesh_dir = session_dir / "mesh"

    raw_dir.mkdir(parents=True, exist_ok=True)
    filtered_dir.mkdir(parents=True, exist_ok=True)
    aligned_dir.mkdir(parents=True, exist_ok=True)
    mesh_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] 세션 저장 경로: {raw_dir}")

    # ESP32 연결
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # s.settimeout(ESP32_TIMEOUT)
        print(f"[INFO] ESP32 연결 시도: {esp32_host}:{esp32_port}")
        s.connect((esp32_host, esp32_port))
    except Exception as e:
        print("❌ ESP32 연결 실패:", e)
        return None

    try:
        while True:
            try:
                buf = s.recv(1024)
            except socket.timeout:
                print("❌ ESP32 응답 타임아웃")
                break

            if not buf:
                continue

            msg = buf.decode(errors="ignore").strip()
            if msg == "end":
                print("[INFO] 모든 각도 촬영 완료 신호 수신")
                break

            try:
                deg = int(msg)
            except ValueError:
                continue

            print(f"[INFO] ESP32로부터 촬영 완료 각도 수신: {deg}")

            # Pi API 촬영 요청
            url = f"{base}/capture"
            params = {"name": name, "gender": gender}
            headers = {"x-auth": token}
            print(f"[INFO] Pi에 촬영 요청: {url} (각도 {deg})")
            try:
                with httpx.Client(timeout=120.0) as client:
                    resp = client.post(url, headers=headers, params=params)
                    resp.raise_for_status()
                    zip_bytes = resp.content
            except Exception as e:
                print(f"❌ Pi 촬영 실패 (각도 {deg}):", e)
                continue

            # ZIP 파일 풀기
            tmp_dir = raw_dir / f"{deg}"
            tmp_dir.mkdir(parents=True, exist_ok=True)
            try:
                with zipfile.ZipFile(io.BytesIO(zip_bytes)) as zf:
                    zf.extractall(tmp_dir)
            except zipfile.BadZipFile:
                print(f"❌ 각도 {deg}: ZIP 파일 손상")
                continue

            # PLY 파일 이름 정리
            src = tmp_dir / "model.ply"
            dst = raw_dir / f"cloud_{deg}_raw.ply"
            if src.exists():
                src.replace(dst)
                print(f"[INFO] 각도 {deg} 저장 완료: {dst}")
            else:
                print(f"❌ 각도 {deg}: model.ply 없음")

            # 다음 촬영 신호
            try:
                print(f"[INFO] ESP32에 다음 촬영 신호 전송")
                s.sendall(b"go\n")
            except Exception:
                pass
    finally:
        try:
            s.close()
        except Exception:
            pass

    print("[INFO] 모든 촬영 완료")
    return session_dir
