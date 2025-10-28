import webbrowser
import threading
import pathlib
from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from src.capture.pi_capture import capture_from_raspberry_pi_with_esp32

captured_session_dir = None 

def create_app(config): # FastAPI 앱 생성 함수 
    app = FastAPI(title="RPi Capture Web UI")

    # --- API 라우트 먼저 정의 ---
    @app.post("/start-capture") # /start-capture 라우트를 등록
    async def start_capture(req: Request):
        global captured_session_dir
        data = await req.json()
        name = data.get("name", "anon")
        gender = data.get("gender", "U")

        try:
            session_dir = capture_from_raspberry_pi_with_esp32(config, name=name, gender=gender)
            if not session_dir:
                print("Pi 촬영 실패")  # 추가
                return JSONResponse({"ok": False, "error": "Pi 촬영 실패"}, status_code=500)
            captured_session_dir = session_dir
        except Exception as e:
            print("예외 발생:", e)  # <-- 여기 추가
            return JSONResponse({"ok": False, "error": str(e)}, status_code=500)
        
        return JSONResponse({"ok": True, "message": "촬영 완료"})

    # --- StaticFiles는 마지막에 mount ---
    # / 경로로 접근하면 public/ 폴더의 index.html 같은 파일을 보여줌
    # 즉, 웹 UI가 들어있는 폴더 
    static_dir = pathlib.Path(__file__).parent / "public"
    app.mount("/", StaticFiles(directory=str(static_dir), html=True), name="static")

    return app


# def launch_web_ui(config):
#     import uvicorn

#     app = create_app(config)

#     # 웹 서버를 별도 스레드로 실행
#     server_thread = threading.Thread(
#         target=lambda: uvicorn.run(app, host="0.0.0.0", port=9000),
#         daemon=True
#     )
#     server_thread.start()

#     webbrowser.open("http://127.0.0.1:9000/")
#     # input("촬영 완료 후 엔터를 눌러 종료합니다...")

#     # 촬영 완료 후 session_dir 반환
#     global captured_session_dir
#     return captured_session_dir

def launch_web_ui(config):
    import uvicorn
    import webbrowser
    import time
    import threading

    global captured_session_dir  # ← 함수 최상단에서 한 번만 선언

    app = create_app(config)

    # 웹 서버를 별도 스레드로 실행
    server_thread = threading.Thread(
        target=lambda: uvicorn.run(app, host="0.0.0.0", port=9000),
        daemon=True
    )
    server_thread.start()

    webbrowser.open("http://127.0.0.1:9000/")
    # input("촬영 완료 후 엔터를 눌러 종료합니다...")

    # captured_session_dir가 설정될 때까지 대기
    while captured_session_dir is None:
        time.sleep(0.5)

    # 촬영 완료 후 session_dir 반환
    return captured_session_dir