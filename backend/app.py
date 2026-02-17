from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import asyncio
import os
import signal
import subprocess

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Registry of named running processes: { session_name: asyncio.subprocess.Process }
running_processes: dict[str, asyncio.subprocess.Process] = {}


@app.get("/health")
def health():
    return {"status": "ok"}


@app.get("/kill/{session_name}")
async def kill_session(session_name: str):
    """
    Send SIGINT (Ctrl+C) to the process group of a named session.
    This cleanly stops ROS2 launch files the same way Ctrl+C does.
    """
    process = running_processes.get(session_name)
    if process is None:
        return {"status": "not_found", "session": session_name}

    try:
        # os.killpg sends the signal to the entire process group (all child nodes too)
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        await asyncio.sleep(1.5)
        # If it's still alive after SIGINT, force kill
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass  # Already dead, that's fine
    except ProcessLookupError:
        pass  # Process already ended

    running_processes.pop(session_name, None)
    return {"status": "killed", "session": session_name}


async def stream_command(cmd: str, session_name: str, websocket: WebSocket):
    """
    Launch cmd in its own process group so we can kill the whole group later.
    Streams stdout/stderr line-by-line to the websocket.
    """
    # If a process with this session name is already running, kill it first
    old = running_processes.get(session_name)
    if old is not None:
        try:
            os.killpg(os.getpgid(old.pid), signal.SIGINT)
            await asyncio.sleep(1.0)
            os.killpg(os.getpgid(old.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        running_processes.pop(session_name, None)

    process = await asyncio.create_subprocess_shell(
        cmd,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
        # start_new_session=True creates a new process group for this command
        # so os.killpg can target the whole tree (launch file + all child nodes)
        start_new_session=True,
    )

    running_processes[session_name] = process
    await websocket.send_text(f"[{session_name}] Process started (PID {process.pid})")

    try:
        while True:
            line = await process.stdout.readline()
            if not line:
                break
            await websocket.send_text(f"[{session_name}] {line.decode().rstrip()}")
    except Exception:
        pass

    await process.wait()
    running_processes.pop(session_name, None)
    await websocket.send_text(f"[{session_name}] COMMAND_FINISHED (exit {process.returncode})")


@app.websocket("/ws/logs/{session_name}")
async def websocket_logs(websocket: WebSocket, session_name: str):
    """
    Each named session gets its own WebSocket endpoint.
    The frontend opens a new WebSocket per 'terminal', keeping them independent.
    """
    await websocket.accept()
    await websocket.send_text(f"[{session_name}] Session ready")

    try:
        while True:
            command = await websocket.receive_text()
            await websocket.send_text(f"[{session_name}] Running: {command}")
            await stream_command(command, session_name, websocket)
    except WebSocketDisconnect:
        # Clean up if the browser tab closes mid-run
        process = running_processes.get(session_name)
        if process:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            running_processes.pop(session_name, None)
