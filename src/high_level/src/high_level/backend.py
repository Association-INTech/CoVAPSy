# BackendAPI.py

import threading
import time
import logging
from typing import Any, Dict, Optional, List

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import numpy as np
import asyncio
from fastapi import WebSocket
from fastapi.websockets import WebSocketDisconnect

from programs.program import Program
from high_level.autotech_constant import (
    PORT_STREAMING_CAMERA,
    BACKEND_ON_START,
)

class BackendAPI(Program):
    """
    Programme backend web for control.
    - Respecte Program: start/kill/running/controls_car
    - Expose une API REST:
        GET  /api/status
        GET  /api/programs
        POST /api/programs/{id}/toggle
        POST /api/programs/{id}/start
        POST /api/programs/{id}/kill
        GET  /api/stream/camera
    - Sert un frontend statique (optionnel) via / (index.html)
    """

    def __init__(
        self,
        server: Any,
        host: str = "0.0.0.0",
        port: int = 8001,
        site_dir: Optional[str] = None,  # ex: "/home/intech/CoVAPSy/src/HL/site_controle"
        cors_allow_origins: Optional[List[str]] = None,
    ):
        super().__init__()
        self.logger = logging.getLogger(__name__)

        self.server = server
        self.controls_car = False
        self.running = False
        self.lidar_yaw = 0  # for lidar coordinate correction


        self.host = host
        self.port = port
        self._thread: Optional[threading.Thread] = None
        self._uvicorn_server: Optional[uvicorn.Server] = None

        self.app = FastAPI(title="CoVAPSy Remote Control API", version="1.0.0")

        # CORS
        if cors_allow_origins is None:
            cors_allow_origins = ["*"]
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=cors_allow_origins,
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        if site_dir:
            self.app.mount("/static", StaticFiles(directory=site_dir), name="static")

            @self.app.get("/", response_class=HTMLResponse)
            def index():
                # the frontend is served at /static/index.html
                return """
                <html>
                  <head><meta charset="utf-8"><title>CoVAPSy</title></head>
                  <body>
                    <h3>CoVAPSy Control</h3>
                    <p>Frontend: <a href="/static/index.html">/static/index.html</a></p>
                    <p>Lidar: <a href="/static/lidar.html">/static/lidar.html</a></p>
                  </body>
                </html>
                """

        self._setup_routes()

        time.sleep(1)  # litle delay to ensure server is ready
        if BACKEND_ON_START:
            self.start()
        


    # ----------------------------
    # Helpers: reading data from server
    # ----------------------------
    def _arduino(self):
        # server.arduino_I2C is a property returning the I2C Arduino instance
        return getattr(self.server, "arduino_I2C", None)

    def _get_telemetry(self) -> Dict[str, Any]:
        ard = self._arduino()
        voltage_lipo = getattr(ard, "voltage_lipo", 0.0) if ard else 0.0
        voltage_nimh = getattr(ard, "voltage_nimh", 0.0) if ard else 0.0
        current_speed = getattr(ard, "current_speed", 0.0) if ard else 0.0

        # Programm which currently controls the car
        last_ctrl = int(getattr(self.server, "last_programme_control", 0))
        programmes = getattr(self.server, "programme", [])
        prog_name = None
        if isinstance(programmes, list) and 0 <= last_ctrl < len(programmes):
            prog_name = type(programmes[last_ctrl]).__name__

        target_speed = float(getattr(self.server, "target_speed", 0.0))
        direction = float(getattr(self.server, "direction", 0.0))

        return {
            "battery": {
                "lipo": voltage_lipo,
                "nimh": voltage_nimh
            },
            "car": {
                "current_speed": current_speed,
                "target_speed": target_speed,
                "direction": direction,
                "car_control": prog_name,
                "program_id": last_ctrl
            },
            "timestamp": time.time(),
        }

    def _list_programs(self) -> List[Dict[str, Any]]:
        programmes = getattr(self.server, "programme", [])
        out: List[Dict[str, Any]] = []
        if not isinstance(programmes, list):
            return out

        for i, p in enumerate(programmes):
            out.append(
                {
                    "id": i,
                    "name": type(p).__name__,
                    "running": bool(getattr(p, "running", False)),
                    "controls_car": bool(getattr(p, "controls_car", False)),
                    "display": p.display() if hasattr(p, "display") else type(p).__name__,
                }
            )
        return out

    def _camera_stream_url(self) -> str:
        ip = getattr(getattr(self.server, "SOCKET_ADRESS", None), "IP", None)
        ip = getattr(self.server, "ip", None) or "192.168.1.10"
        return f"http://{ip}:{PORT_STREAMING_CAMERA}/cam/"
    
    def _lidar(self):
        return getattr(self.server, "lidar", None)

    def _get_lidar_points_cartesian(self):
        lidar = self._lidar()
        if not lidar or lidar.rDistance is None:
            return None

        r = np.asarray(lidar.rDistance)
        n = r.shape[0]

        # lidar angle values
        theta_lidar = np.linspace(
            -3*np.pi/4,   # -135°
            +3*np.pi/4,   # +135°
            n,
            endpoint=True
        )

        # orientation correction 
        theta_world = theta_lidar + self.lidar_yaw

        # projection
        # cartésien coordonates
        x = -np.sin(theta_world) * r
        y = np.cos(theta_world) * r


        return {
            "x": x.tolist(),
            "y": y.tolist(),
            "unit": "mm",
            "timestamp": time.time(),
        }



    # ----------------------------
    # Routes
    # ----------------------------
    def _setup_routes(self) -> None:
        @self.app.get("/api/status")
        def status():
            return {
                "backend": {"running": self.running, "host": self.host, "port": self.port},
                "telemetry": self._get_telemetry(),
                "programs": self._list_programs(),
            }

        @self.app.get("/api/programs")
        def programs():
            return self._list_programs()

        @self.app.post("/api/programs/{prog_id}/toggle")
        def toggle_program(prog_id: int):
            programmes = getattr(self.server, "programme", [])
            if not isinstance(programmes, list) or not (0 <= prog_id < len(programmes)):
                raise HTTPException(status_code=404, detail="Unknown program id")

            self.server.start_process(prog_id)
            # après action, renvoyer état mis à jour
            return {"status": "ok", "program_id": prog_id, "programs": self._list_programs()}

        @self.app.post("/api/programs/{prog_id}/start")
        def start_program(prog_id: int):
            programmes = getattr(self.server, "programme", [])
            if not isinstance(programmes, list) or not (0 <= prog_id < len(programmes)):
                raise HTTPException(status_code=404, detail="Unknown program id")

            p = programmes[prog_id]
            if getattr(p, "running", False):
                return {"status": "already_running", "program_id": prog_id}

            self.server.start_process(prog_id)
            return {"status": "ok", "program_id": prog_id}

        @self.app.post("/api/programs/{prog_id}/kill")
        def kill_program(prog_id: int):
            programmes = getattr(self.server, "programme", [])
            if not isinstance(programmes, list) or not (0 <= prog_id < len(programmes)):
                raise HTTPException(status_code=404, detail="Unknown program id")

            p = programmes[prog_id]
            if not getattr(p, "running", False):
                return {"status": "already_stopped", "program_id": prog_id}

            # toggle kill if running
            self.server.start_process(prog_id)
            return {"status": "ok", "program_id": prog_id}

        @self.app.get("/api/stream/camera")
        def camera_stream():
            # give the URL.
            return {"url": self._camera_stream_url()}
        
        @self.app.get("/api/lidar")
        def lidar_snapshot():
            data = self._get_lidar_points_cartesian()
            if data is None:
                raise HTTPException(status_code=503, detail="Lidar not available")
            return data
        
        @self.app.websocket("/api/lidar/ws")
        async def lidar_ws(ws: WebSocket):
            await ws.accept()
            self.logger.info("Lidar WS client connected")

            try:
                while True:
                    data = self._get_lidar_points_cartesian()
                    if data:
                        await ws.send_json(data)
                    await asyncio.sleep(0.1)
            except WebSocketDisconnect:
                self.logger.info("Lidar WS client disconnected")
        @self.app.websocket("/api/telemetry/ws")
        async def telemetry_ws(ws: WebSocket):
            await ws.accept()
            self.logger.info("Telemetry WS client connected")

            try:
                while True:
                    data = self._get_telemetry()
                    await ws.send_json(data)
                    await asyncio.sleep(0.25)  # 4 Hz
            except WebSocketDisconnect:
                self.logger.info("Telemetry WS client disconnected")


    # ----------------------------
    # Program interface
    # ----------------------------
    def start(self):
        if self.running:
            return
        self.running = True

        # Uvicorn
        config = uvicorn.Config(self.app, host=self.host, port=self.port, log_level="info")
        self._uvicorn_server = uvicorn.Server(config)

        def _run():
            try:
                self.logger.info("BackendAPI starting on %s:%d", self.host, self.port)
                self._uvicorn_server.run()
            except Exception as e:
                self.logger.error("BackendAPI crashed: %s", e, exc_info=True)
            finally:
                self.running = False
                self.logger.warning("BackendAPI stopped")

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()

    def kill(self):
        if not self.running:
            return
        self.running = False
        if self._uvicorn_server:
            self._uvicorn_server.should_exit = True
        self.logger.info("BackendAPI kill requested")

    def display(self):
        name = self.__class__.__name__
        return f"{name}\n(running)" if self.running else name
