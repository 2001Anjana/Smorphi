from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import asyncio
import subprocess

app = FastAPI() //

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

current_process = None


@app.get("/health")
def health():
    return {"status": "ok"}


@app.websocket("/ws/logs")
async def websocket_logs(websocket: WebSocket):
    global current_process

    await websocket.accept()

    try:
        while True:
            command = await websocket.receive_text()

            if current_process:
                await websocket.send_text("Process already running")
                continue

            await websocket.send_text(f"Running: {command}")

            current_process = await asyncio.create_subprocess_shell(
                command,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT
            )

            while True:
                line = await current_process.stdout.readline()
                if not line:
                    break

                try:
                    await websocket.send_text(line.decode().strip())
                except:
                    break

            current_process = None

    except:
        pass


@app.post("/bringup/stop")
def stop_bringup():
    global current_process

    if current_process:
        subprocess.call("pkill -SIGINT -f ros2", shell=True)
        current_process = None
        return {"status": "stopped"}

    return {"status": "not running"}
