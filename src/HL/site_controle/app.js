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

    // refresh toutes les 250 ms
    setInterval(refresh, 250);
    refresh();
}

init();
 