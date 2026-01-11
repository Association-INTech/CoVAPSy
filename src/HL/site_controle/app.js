const API = ""; // même origine (FastAPI)

async function fetchStatus() {
    const res = await fetch(`${API}/api/status`);
    return await res.json();
}

async function fetchCameraUrl() {
    const res = await fetch(`${API}/api/stream/camera`);
    const data = await res.json();
    return data.url;
}

async function toggleProgram(id) {
    await fetch(`${API}/api/programs/${id}/toggle`, { method: "POST" });
}

async function startProgram(id) {
    await fetch(`${API}/api/programs/${id}/start`, { method: "POST" });
}

async function killProgram(id) {
    await fetch(`${API}/api/programs/${id}/kill`, { method: "POST" });
}

function updateTelemetry(t) {
    document.getElementById("lipo").textContent = t.battery.lipo.toFixed(2);
    document.getElementById("nimh").textContent = t.battery.nimh.toFixed(2);
    document.getElementById("vitesse").textContent = t.car.vitesse_reelle.toFixed(2);
    document.getElementById("active_program").textContent =
        t.car.programme_controle ?? "Aucun";
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
                <button class="toggle" onclick="toggleProgram(${p.id})">Toggle</button>
                <button class="start" onclick="startProgram(${p.id})">Start</button>
                <button class="kill" onclick="killProgram(${p.id})">Kill</button>
            </td>
        `;

        tbody.appendChild(tr);
    }
}

async function refresh() {
    try {
        const data = await fetchStatus();
        updateTelemetry(data.telemetry);
        updatePrograms(data.programs);
    } catch (e) {
        console.error("API unreachable", e);
    }
}

async function init() {
    const camUrl = await fetchCameraUrl();
    document.getElementById("camera").src = camUrl;

    initLidar();
    setInterval(refresh, 250);
    refresh();
}

function initLidar() {
    const canvas = document.getElementById("lidar");
    if (!canvas) return;

    const ctx = canvas.getContext("2d");
    const scale = 0.15; // 10 cm = 15 px

    const proto = location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(proto + "://" + location.host + "/api/lidar/ws");

    ws.onmessage = (e) => {
        const data = JSON.parse(e.data);

        ctx.clearRect(0, 0, canvas.width, canvas.height);

        ctx.save();
        ctx.translate(canvas.width / 2, canvas.height / 2);

        /* ---------- Grille ---------- */
        const circleStepMM = 100; // 10 cm
        const circleCount = 15;

        ctx.strokeStyle = "#333";
        ctx.lineWidth = 1;

        for (let i = 1; i <= circleCount; i++) {
            const r = i * circleStepMM * scale;
            ctx.beginPath();
            ctx.arc(0, 0, r, 0, Math.PI * 2);
            ctx.stroke();

            ctx.fillStyle = "#777";
            ctx.font = "10px monospace";
            ctx.fillText(`${i * 10} cm`, r + 4, 0);
        }

        /* ---------- Axes ---------- */
        ctx.strokeStyle = "#555";
        ctx.beginPath();
        ctx.moveTo(-canvas.width / 2, 0);
        ctx.lineTo(canvas.width / 2, 0);
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(0, -canvas.height / 2);
        ctx.lineTo(0, canvas.height / 2);
        ctx.stroke();

        /* ---------- Points LIDAR ---------- */
        ctx.fillStyle = "#00ff88";
        for (let i = 0; i < data.x.length; i++) {
            ctx.fillRect(
                data.x[i] * scale,
               -data.y[i] * scale,
                2, 2
            );
        }

        ctx.restore();
    };

    ws.onclose = () => {
        console.warn("LIDAR WS disconnected");
    };
}


init();
 