const API = ""; // same origin (FastAPI)
const speedHistory = {
    real: [],
    demand: [],
    maxPoints: 100
};

function drawSpeedChart() {
    const canvas = document.getElementById("speedChart");
    if (!canvas) return;

    const ctx = canvas.getContext("2d");
    const w = canvas.width;
    const h = canvas.height;

    ctx.clearRect(0, 0, w, h);

    // axes (drawing axes)
    ctx.strokeStyle = "#444";
    ctx.beginPath();
    ctx.moveTo(40, 10);
    ctx.lineTo(40, h - 20);
    ctx.lineTo(w - 10, h - 20);
    ctx.stroke();



    const maxAbs = Math.max(
    ...speedHistory.real.map(Math.abs),
    ...speedHistory.demand.map(Math.abs),
    1
    );
    const yZero = h / 2;
    const scaleY = (h - 40) / (2 * maxAbs);
    // y=0 line
    ctx.strokeStyle = "#666";
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    ctx.moveTo(40, yZero);
    ctx.lineTo(w - 10, yZero);
    ctx.stroke();
    ctx.setLineDash([]);


    function drawCurve(data, color) {
        ctx.strokeStyle = color;
        ctx.beginPath();

        data.forEach((v, i) => {
            const x = 40 + (i / (data.length - 1)) * (w - 60);
            const y = yZero - v * scaleY;
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        });

        ctx.stroke();
    }

    drawCurve(speedHistory.demand, "#ffaa00"); // target
    drawCurve(speedHistory.real, "#00ff88");   // actual

    // legend

    ctx.fillStyle = "#ffaa00";
    ctx.fillText("Target", w - 100, 20);
    ctx.fillStyle = "#00ff88";
    ctx.fillText("Actual", w - 100, 35);
    ctx.font = "10px monospace";

}

function drawSteering(directionDeg) {
    const canvas = document.getElementById("steeringViz");
    if (!canvas) return;

    const ctx = canvas.getContext("2d");
    const w = canvas.width;
    const h = canvas.height;

    ctx.clearRect(0, 0, w, h);

    const centerX = w / 2;
    const centerY = h * 0.85;
    const radius = h * 0.65;

    /* -------- Protractor (-18° to +18°) -------- */
    ctx.strokeStyle = "#444";
    ctx.lineWidth = 1;

    const maxDeg = 18;

    ctx.beginPath();
    ctx.arc(
        centerX,
        centerY,
        radius,
        (-90 - maxDeg) * Math.PI / 180,
        (-90 + maxDeg) * Math.PI / 180
    );
    ctx.stroke();

    /* -------- Graduations (Marks) -------- */
    for (let d = -18; d <= 18; d += 6) {
        const a = (d - 90) * Math.PI / 180;

        ctx.beginPath();
        ctx.moveTo(
            centerX + Math.cos(a) * (radius - 10),
            centerY + Math.sin(a) * (radius - 10)
        );
        ctx.lineTo(
            centerX + Math.cos(a) * radius,
            centerY + Math.sin(a) * radius
        );
        ctx.stroke();

        ctx.fillStyle = "#777";
        ctx.font = "10px monospace";
        ctx.fillText(
            `${d}°`,
            centerX + Math.cos(a) * (radius + 10) - 8,
            centerY + Math.sin(a) * (radius + 10) + 3
        );
    }

    /* -------- Rod (target direction) -------- */
    const angleRad = (directionDeg - 90) * Math.PI / 180;

    ctx.strokeStyle = "#00ff88";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(
        centerX + Math.cos(angleRad) * (radius - 20),
        centerY + Math.sin(angleRad) * (radius - 20)
    );
    ctx.stroke();

    /* -------- Center -------- */
    ctx.fillStyle = "#00ff88";
    ctx.beginPath();
    ctx.arc(centerX, centerY, 5, 0, Math.PI * 2);
    ctx.fill();
}


async function fetchStatus() {
    const res = await fetch(`${API}/api/status`);
    if (!res.ok) {
        throw new Error(`API status failed: ${res.status}`);
    }
    return await res.json();
}

async function fetchCameraUrl() {
    const res = await fetch(`${API}/api/stream/camera`);
    if (!res.ok) {
        throw new Error(`Camera url API failed: ${res.status}`);
    }
    const data = await res.json();
    return data.url;
}


async function startProgram(id) {
    await fetch(`${API}/api/programs/${id}/start`, { method: "POST" });
    refreshPrograms();
}

async function killProgram(id) {
    await fetch(`${API}/api/programs/${id}/kill`, { method: "POST" });
    refreshPrograms();
}

function updateTelemetry(t) {
    document.getElementById("lipo").textContent = t.battery.lipo.toFixed(2);
    document.getElementById("nimh").textContent = t.battery.nimh.toFixed(2);
    document.getElementById("vitesse").textContent =
        t.car.current_speed.toFixed(2);

    document.getElementById("target_speed").textContent =
        t.car.target_speed.toFixed(2);

    document.getElementById("direction").textContent =
        t.car.direction.toFixed(2);
    document.getElementById("active_program").textContent =
        t.car.car_control ?? "None";
    document.getElementById("tof").textContent =
        t.car.tof.toFixed(2);
    speedHistory.real.push(t.car.current_speed);
    speedHistory.demand.push(t.car.target_speed);

    if (speedHistory.real.length > speedHistory.maxPoints) {
        speedHistory.real.shift();
        speedHistory.demand.shift();
    }

    drawSpeedChart();
    drawSteering(t.car.direction);

}

function updatePrograms(programs) {
    const tbody = document.getElementById("program_table");
    tbody.innerHTML = "";

    for (const p of programs) {
        const tr = document.createElement("tr");

        tr.innerHTML = `
            <td>${p.id}</td>
            <td>${p.name}</td>
            <td>${p.running ? "🟢" : "🔴"}</td>
            <td>${p.controls_car ? "🚗" : "-"}</td>
            <td>
                <button class="start" onclick="startProgram(${p.id})">Start</button>
                <button class="kill" onclick="killProgram(${p.id})">Kill</button>
            </td>
        `;

        tbody.appendChild(tr);
    }
}
async function loadModels() {
    try {
        const res = await fetch("/api/status");
        if (!res.ok) throw new Error("Failed to load models");

        const data = await res.json();
        const models = data.models;

        const select = document.getElementById("model_select");
        select.innerHTML = "";

        for (const m of models) {
            const option = document.createElement("option");
            option.value = m;
            option.textContent = m;
            select.appendChild(option);
        }

    } catch (e) {
        console.error("Failed to load models", e);
    }
}

async function startSelectedModel() {
    const select = document.getElementById("model_select");
    const model = select.value;

    try {
        const res = await fetch("/api/ai/start", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ model })
        });

        if (!res.ok) throw new Error("Failed to start AI");

        console.log("AI started with model:", model);

    } catch (e) {
        console.error("Start model failed", e);
    }
}

async function refreshPrograms() {
    try {
        const res = await fetch("/api/programs");
        if (!res.ok) {
        throw new Error(`Programs API failed: ${res.status}`);
    }
        const programs = await res.json();
        updatePrograms(programs);
    } catch (e) {
        console.error("Failed to refresh programs", e);
    }
}
function decodeBase64ToInt16Array(b64) {
  const bin = atob(b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  return new Int16Array(bytes.buffer);
}

function initLidar(retryDelay = 1000) {
    const canvas = document.getElementById("lidar");
    if (!canvas) return;

    const ctx = canvas.getContext("2d");
    const scale = 0.15; // 10 cm = 15 px

    const proto = location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(proto + "://" + location.host + "/api/lidar/ws");
    try{
    ws.onmessage = (e) => {
        const data = JSON.parse(e.data);
        const x = decodeBase64ToInt16Array(data.x);
        const y = decodeBase64ToInt16Array(data.y);
        const tof = data.tof;

        ctx.clearRect(0, 0, canvas.width, canvas.height);

        ctx.save();
        ctx.translate(canvas.width / 2, canvas.height / 2);

        /* ---------- Grid ---------- */
        const circleStepMM = 100; // 10 cm
        const circleCount = 15;

        ctx.strokeStyle = "#333";
        ctx.lineWidth = 1;

        for (let i = 1; i <= circleCount; i++) {
            const r = i * circleStepMM * scale;
            ctx.beginPath();
            ctx.arc(0, 0, r, 0, Math.PI * 2);
            ctx.stroke();

            if (i % (circleCount / 3) === 0){
            // distance label
            ctx.fillStyle = "#777";
            ctx.font = "10px monospace";
            ctx.fillText(`${i * 10} cm`, r + 2, 0);
            }
        }
        // Lidar FOV (270°)
        ctx.strokeStyle = "#444";
        ctx.lineWidth = 1;

        const fovMin = -135 * Math.PI / 180;
        const fovMax =  135 * Math.PI / 180;
        const fovRadius = 1500 * scale;

        ctx.beginPath();
        // left line
        ctx.moveTo(0, 0);
        ctx.lineTo(
        Math.sin(fovMin) * fovRadius,
        -Math.cos(fovMin) * fovRadius
        );
        ctx.stroke();

        // right line
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(
        Math.sin(fovMax) * fovRadius,
        -Math.cos(fovMax) * fovRadius
        );
        ctx.stroke();
        /* ---------- Axes (Axis) ---------- */
        ctx.strokeStyle = "#555";
        ctx.beginPath();
        ctx.moveTo(-canvas.width / 2, 0);
        ctx.lineTo(canvas.width / 2, 0);
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(0, -canvas.height / 2);
        ctx.lineTo(0, canvas.height / 2);
        ctx.stroke();

        /* ---------- LIDAR Points ---------- */
        ctx.fillStyle = "#00ff88";
        for (let i = 0; i < x.length; i++) {
            ctx.fillRect(
                x[i] * scale,
               -y[i] * scale,
                2, 2
            );
        }
        // Draw ToF point on lidar pov
        ctx.fillStyle = "#00eaff";
        const tofY = (tof + 30) * scale * 10; // assuming tof[0] is the distance in mm and the 30 is an offset to place it correctly on the canvas comparing distance of the tof and the lidar
        ctx.beginPath();
        ctx.arc(0, tofY, 5, 0, Math.PI );
        ctx.fill();
        ctx.restore();
    };
    }catch(e){
        console.error("Error in LIDAR WS onmessage:", e);
    }

    ws.onclose = () => {
        console.warn("LIDAR WS disconnected");

        setTimeout(() => {
            initLidar(Math.min(retryDelay * 2, 8000));
        }, retryDelay);
}
}
function initTelemetryWS() {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(proto + "://" + location.host + "/api/telemetry/ws");
    try{
            ws.onmessage = (e) => {
        const data = JSON.parse(e.data);
        updateTelemetry(data);
    };
    }catch(e){
    console.error("Error in Telemetry WS onmessage:", e);
}
    ws.onclose = () => {
        console.warn("Telemetry WS disconnected, retrying...");
        setTimeout(initTelemetryWS, 1000);
    };
}

async function loadProgramsOnce() {
    try {
        const res = await fetch("/api/programs");
        if (!res.ok) {
            throw new Error(`Programs API failed: ${res.status}`);
        }
        const programs = await res.json();
        updatePrograms(programs);
    } catch (e) {
        console.error("Failed to load programs", e);
    }
}


async function init() {
    try {
        const camUrl = await fetchCameraUrl();
        const camEl = document.getElementById("camera_frame");
        const camLink = document.getElementById("camera");
        loadModels();
        // const url = "http://10.255.28.97:8889/cam/";

        // if (Hls.isSupported()) {
        //     const hls = new Hls();
        //     hls.loadSource(url);
        //     hls.attachMedia(video);
        // } else if (video.canPlayType("application/vnd.apple.mpegurl")) {
        //     video.src = url;
        // }


        if (camEl && camLink) {
            camEl.src = camUrl;
            camLink.href = camUrl;

        } else {
            console.warn("Element #camera not found at initialization");
        }

        initLidar();
        initTelemetryWS();
        loadProgramsOnce();
    } catch (e) {
        console.error("Error in init:", e);
    }
}

window.addEventListener("DOMContentLoaded", init);