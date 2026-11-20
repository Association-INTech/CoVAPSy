/* CoVAPSy Pit-Wall — dashboard client (Plotly.js) */
"use strict";

// ============================================================ state
const PALETTE = ["#E10600", "#00D2BE", "#FF8700", "#3671C6", "#FFD700", "#B36BFF", "#52E252", "#FF5FA2", "#9AA4B2", "#27F4D2"];
const LS = (k, d) => { try { const v = localStorage.getItem(k); return v === null ? d : JSON.parse(v); } catch { return d; } };
const savedSel = LS("pw.sel", null);

const state = {
  runs: [],
  selected: new Set(savedSel || []),
  firstLoad: savedSel === null,
  colors: LS("pw.colors", {}),
  x: "sim_time", window: 50, cpLast: 300,
  mapFilter: "ALL",
  cpview: "cart", crash: "visit", crashrate: "cum", smooth: "scatter",
  auto: true, inflight: false, data: {},
};

const $ = (id) => document.getElementById(id);
const saveSel = () => localStorage.setItem("pw.sel", JSON.stringify([...state.selected]));

function colorOf(name) {
  if (!state.colors[name]) {
    const used = new Set(Object.values(state.colors));
    state.colors[name] = PALETTE.find((p) => !used.has(p)) || PALETTE[Object.keys(state.colors).length % PALETTE.length];
    localStorage.setItem("pw.colors", JSON.stringify(state.colors));
  }
  return state.colors[name];
}
function rgba(hex, a) {
  const n = parseInt(hex.slice(1), 16);
  return `rgba(${(n >> 16) & 255},${(n >> 8) & 255},${n & 255},${a})`;
}
const fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v) ? "—" : Number(v).toFixed(d));
const scale = (arr, k) => (arr || []).map((v) => (v === null ? null : v * k));
const hasData = (arr) => Array.isArray(arr) && arr.some((v) => v !== null && v !== undefined);

function bandTraces(x, yMean, yStd, hexColor, fillAlpha = 0.14, yaxis = "y", legendgroup = "", clipLo = null, clipHi = null) {
  if (!Array.isArray(x) || !Array.isArray(yMean) || !Array.isArray(yStd)) return [];
  const upper = [], lower = [], xv = [];
  for (let i = 0; i < x.length; i++) {
    const m = yMean[i], s = yStd[i] ?? 0;
    if (x[i] === null || m === null || m === undefined || Number.isNaN(m)) continue;
    let hi = m + s, lo = m - s;
    if (clipLo !== null) lo = Math.max(clipLo, lo);
    if (clipHi !== null) hi = Math.min(clipHi, hi);
    xv.push(x[i]);
    upper.push(hi);
    lower.push(lo);
  }
  if (xv.length < 2) return [];
  return [
    {
      type: "scatter", mode: "lines", x: xv, y: upper, yaxis,
      legendgroup, showlegend: false, hoverinfo: "skip",
      line: { width: 0, color: "rgba(0,0,0,0)" },
    },
    {
      type: "scatter", mode: "lines", x: xv, y: lower, yaxis,
      legendgroup, showlegend: false, hoverinfo: "skip",
      fill: "tonexty", fillcolor: rgba(hexColor, fillAlpha),
      line: { width: 0, color: "rgba(0,0,0,0)" },
    },
  ];
}

// ============================================================ plotly base
const GRID = "#222833", AXL = "#2c3340";
const axis = (extra = {}) => ({ gridcolor: GRID, zerolinecolor: AXL, linecolor: AXL, tickfont: { size: 10 }, title: { font: { size: 11 } }, ...extra });
function layout(extra = {}) {
  const base = {
    paper_bgcolor: "rgba(0,0,0,0)", plot_bgcolor: "#10131a",
    font: { family: "Titillium Web, Segoe UI, sans-serif", color: "#c9d1d9", size: 11 },
    margin: { l: 54, r: 54, t: 26, b: 40 },
    legend: { orientation: "h", x: 0, y: 1.1, font: { size: 10 }, bgcolor: "rgba(0,0,0,0)" },
    hovermode: "x unified",
    hoverlabel: { bgcolor: "#1b2029", bordercolor: "#3a4150", font: { family: "JetBrains Mono, monospace", size: 11 } },
    uirevision: state.x,
  };
  const out = { ...base, ...extra };
  for (const k of Object.keys(extra)) if (k.startsWith("xaxis") || k.startsWith("yaxis")) out[k] = axis(extra[k]);
  if (!extra.xaxis && !extra.polar) out.xaxis = axis();
  if (!extra.yaxis && !extra.polar) out.yaxis = axis();
  return out;
}
const CFG = { displaylogo: false, responsive: true, modeBarButtonsToRemove: ["lasso2d", "select2d", "autoScale2d"] };

function emptyPlot(div, msg, h) {
  Plotly.react(div, [], layout({
    xaxis: { visible: false }, yaxis: { visible: false }, margin: { l: 10, r: 10, t: 10, b: 10 },
    annotations: [{ text: msg, showarrow: false, xref: "paper", yref: "paper", x: 0.5, y: 0.5, font: { size: 12, color: "#7d8696" } }],
  }), CFG);
}

// ============================================================ data loop
async function tick(force = false) {
  if (state.inflight || (!force && (!state.auto || document.hidden))) return;
  state.inflight = true;
  try {
    const rj = await (await fetch("/api/runs")).json();
    state.runs = rj.runs;
    $("logdir").textContent = rj.log_dir;
    if (state.firstLoad && rj.runs.length) {
      rj.runs.forEach((r) => state.selected.add(r.run_name));
      state.firstLoad = false;
      saveSel();
    }
    renderRunList();

    const sel = rj.runs.map((r) => r.run_name).filter((n) => state.selected.has(n));
    if (sel.length) {
      const q = new URLSearchParams({ runs: sel.join(","), x: state.x, window: state.window, cp_last: state.cpLast });
      const dj = await (await fetch("/api/data?" + q)).json();
      state.data = dj.runs || {};
    } else {
      state.data = {};
    }
    renderAll();
    $("status-dot").className = "dot ok";
    $("status-text").textContent = `${sel.length}/${rj.runs.length} runs · ${new Date().toLocaleTimeString()}`;
  } catch (e) {
    console.error(e);
    $("status-dot").className = "dot err";
    $("status-text").textContent = "server unreachable";
  } finally {
    state.inflight = false;
  }
}

// ============================================================ sidebar
function renderRunList() {
  const el = $("run-list");
  if (!state.runs.length) {
    el.innerHTML = `<div class="hint">No <span class="mono">*.jsonl</span> files yet.<br>Run <span class="mono">generate_mock_runs.py</span> or start a training run with the TelemetryLogger.</div>`;
    return;
  }
  el.innerHTML = state.runs.map((r) => {
    const sel = state.selected.has(r.run_name);
    const c = colorOf(r.run_name);
    return `<label class="run-item ${sel ? "selected" : ""}" data-run="${r.run_name}">
      <input type="checkbox" ${sel ? "checked" : ""} />
      <span class="swatch" style="background:${c}"></span>
      <span><div class="rname" title="${r.run_name}">${r.run_name}</div>
        <div class="rmeta">${r.episodes} ep · ${r.size_kb} kB</div></span>
      <span class="badge ${r.live ? "live" : "idle"}">${r.live ? "LIVE" : "IDLE"}</span>
    </label>`;
  }).join("");
}

$("run-list").addEventListener("change", (ev) => {
  const item = ev.target.closest(".run-item");
  if (!item) return;
  const name = item.dataset.run;
  ev.target.checked ? state.selected.add(name) : state.selected.delete(name);
  saveSel();
  tick(true);
});
$("sel-all").onclick = () => { state.runs.forEach((r) => state.selected.add(r.run_name)); saveSel(); tick(true); };
$("sel-none").onclick = () => { state.selected.clear(); saveSel(); tick(true); };
$("sel-live").onclick = () => { state.selected = new Set(state.runs.filter((r) => r.live).map((r) => r.run_name)); saveSel(); tick(true); };

document.querySelectorAll(".seg").forEach((seg) => {
  seg.addEventListener("click", (ev) => {
    const b = ev.target.closest("button");
    if (!b) return;
    const ctl = seg.dataset.ctl;
    state[ctl] = b.dataset.v;
    document.querySelectorAll(`.seg[data-ctl="${ctl}"]`).forEach((s) => {
      s.querySelectorAll("button").forEach((x) => x.classList.toggle("active", x.dataset.v === state[ctl]));
    });
    if (ctl === "x") tick(true); else renderAll();
  });
});
for (const [id, key, lbl] of [["window", "window", "window-val"], ["cplast", "cpLast", "cplast-val"]]) {
  $(id).addEventListener("input", (e) => { $(lbl).textContent = e.target.value === "0" && id === "cplast" ? "ALL" : e.target.value; });
  $(id).addEventListener("change", (e) => { state[key] = +e.target.value; tick(true); });
}
$("btn-refresh").onclick = () => {
  state.auto = !state.auto;
  $("btn-refresh").classList.toggle("active", state.auto);
  $("btn-refresh").textContent = state.auto ? "⟳ AUTO 2s" : "❚❚ PAUSED";
  if (state.auto) tick(true);
};
document.addEventListener("visibilitychange", () => { if (!document.hidden) tick(true); });

// ============================================================ render
function allLoadedRuns() {
  const out = [];
  for (const r of state.runs) {
    if (!state.selected.has(r.run_name)) continue;
    const d = state.data[r.run_name];
    if (d && !d.empty) out.push(d);
  }
  // Include any virtual multi-map series returned by the server
  for (const [k, d] of Object.entries(state.data)) {
    if (d && !d.empty && !out.includes(d)) out.push(d);
  }
  return out;
}

function updateMapFilterBar(runs) {
  const maps = [...new Set(runs.map((r) => (r.kpi && r.kpi.map) || (r.meta && r.meta.map) || "piste0"))];
  const wrap = $("map-filter-wrap");
  const container = $("map-filter");
  if (!wrap || !container) return;
  if (maps.length <= 1) {
    wrap.style.display = "none";
    state.mapFilter = "ALL";
    return;
  }
  wrap.style.display = "flex";
  if (state.mapFilter !== "ALL" && !maps.includes(state.mapFilter)) state.mapFilter = "ALL";
  const opts = ["ALL", ...maps];
  container.innerHTML = opts.map((m) => `<button data-v="${m}" class="${state.mapFilter === m ? "active" : ""}">${m}</button>`).join("");
}

function selectedRuns() {
  const all = allLoadedRuns();
  updateMapFilterBar(all);
  if (state.mapFilter === "ALL") return all;
  return all.filter((r) => ((r.kpi && r.kpi.map) || (r.meta && r.meta.map) || "piste0") === state.mapFilter);
}

function renderAll() {
  const runs = selectedRuns();
  renderKPIs(runs);
  renderP1(runs); renderP2(runs); renderP3(runs); renderP4(runs); renderP5(runs); renderP6(runs);
  const eps = runs.reduce((s, r) => s + r.episodes, 0);
  $("foot-info").textContent = `${runs.length} runs · ${eps} episodes · window ${state.window} · cp window ${state.cpLast || "ALL"}`;
}

function renderKPIs(runs) {
  $("kpis").innerHTML = runs.map((r) => {
    const k = r.kpi, c = colorOf(r.run_name);
    const p95cls = k.p95_ms == null ? "" : k.p95_ms > 100 ? "bad" : k.p95_ms > 25 ? "warn" : "good";
    const crashCls = k.crash_rate == null ? "" : k.crash_rate > 60 ? "bad" : k.crash_rate > 30 ? "warn" : "good";
    const cpPct = k.mean_cp_pct ?? 0;
    const cpCls = cpPct >= 75 ? "good" : cpPct >= 30 ? "warn" : "";
    const nCp = k.n_checkpoints || 31;
    const meanCp = k.mean_cp != null ? Math.round(k.mean_cp) : 0;
    const maxCp = k.max_cp != null ? k.max_cp : 0;
    const mapName = k.map || (r.meta && r.meta.map) || "piste0";
    const nSims = k.n_sims || (r.meta && r.meta.n_sims) || 1;
    const nVeh = k.n_vehicles || (r.meta && r.meta.n_vehicles) || 1;
    const cell = (lbl, val, unit = "", cls = "", sub = "") =>
      `<div class="kpi-cell"><div class="lbl">${lbl}</div><div class="val ${cls}">${val}<small>${unit}</small></div>${sub ? `<div class="subval">${sub}</div>` : ""}</div>`;
    return `<div class="kpi" style="border-left-color:${c}">
      <div class="kpi-head">
        <span class="kpi-name" style="color:${c}">${r.run_name}</span>
        <div class="kpi-head-right">
          <span class="badge topo">MAP: ${mapName} | ${nSims} SIM × ${nVeh} VEH</span>
          <span class="badge ${r.live ? "live" : "idle"}">${r.live ? "● LIVE" : "IDLE"}</span>
        </div>
      </div>
      <div class="kpi-grid">
        ${cell("Speed", fmt(k.mean_speed), "m/s")}
        ${cell("Crash rate", fmt(k.crash_rate, 0), "%", crashCls)}
        ${cell("Survival", fmt(k.survival_s, 1), "s")}
        ${cell("p95 latency", fmt(k.p95_ms, 1), "ms", p95cls)}
        ${cell("Steer osc", k.steer_osc != null ? fmt(k.steer_osc * 1000, 1) : "—", "mrad")}
        ${cell("Best lap", fmt(k.best_lap_s, 2), "s", "", k.laps ? `${k.laps} laps` : "")}
        ${cell("Progress", `${fmt(k.mean_cp_pct, 1)}%`, `(CP ${meanCp}/${nCp})`, cpCls, `MAX CP ${maxCp}/${nCp}`)}
        ${cell("Sim time", fmt(k.cum_sim_time_min, 1), "min", "", `${r.episodes} ep · ${(k.total_steps / 1000).toFixed(1)}k st`)}
      </div></div>`;
  }).join("");
}

const xTitle = () => (state.x === "sim_time" ? "Cumulative Sim Time (min)" : state.x === "step" ? "Cumulative Timesteps (k)" : "Cohort Round / Episode");
const NO_RUNS = "Select one or more runs in the sidebar";

// ---------------------------------------------------------------- 01 checkpoint radar
function renderP1(runs) {
  const withCp = runs.filter((r) => r.checkpoint);
  if (!withCp.length) return emptyPlot("p1", runs.length ? "No checkpoint_id data yet (see README → supervisor patch)" : NO_RUNS);
  const traces = [];
  if (state.cpview === "cart") {
    let maxLid = 0.1;
    withCp.forEach((r) => {
      const cp = r.checkpoint, c = colorOf(r.run_name);
      cp.min_lidar_mean.forEach((v) => { if (v !== null) maxLid = Math.max(maxLid, v); });
      traces.push({
        type: "bar", x: cp.ids, y: cp.min_lidar_mean, yaxis: "y2", name: `${r.run_name} · clearance`,
        legendgroup: r.run_name, showlegend: false, customdata: cp.min_lidar_abs,
        marker: { color: rgba(c, 0.25), line: { color: rgba(c, 0.8), width: 1 } },
        hovertemplate: "clear %{y:.2f} m (worst %{customdata:.2f})<extra>" + r.run_name + "</extra>",
      });
      traces.push({
        type: "scatter", mode: "lines+markers", x: cp.ids, y: cp.mean_speed, name: r.run_name, legendgroup: r.run_name,
        line: { color: c, width: 2.5, shape: "spline", smoothing: 0.5 }, marker: { size: 5 }, connectgaps: false,
        hovertemplate: "%{y:.2f} m/s<extra>" + r.run_name + "</extra>",
      });
    });
    Plotly.react("p1", traces, layout({
      uirevision: "cart", barmode: "group", bargap: 0.25,
      xaxis: { title: { text: "Checkpoint ID (sector = next checkpoint)" }, dtick: 1 },
      yaxis: { title: { text: "Mean speed (m/s)" }, rangemode: "tozero" },
      yaxis2: { title: { text: "Min LiDAR clearance (m)" }, overlaying: "y", side: "right", showgrid: false, range: [0, maxLid * 2.2] },
    }), CFG);
  } else {
    withCp.forEach((r) => {
      const cp = r.checkpoint, c = colorOf(r.run_name);
      const th = cp.ids.map((i) => "CP" + i).concat(["CP0"]);
      const close = (a) => a.concat([a[0]]);
      traces.push({ type: "scatterpolar", subplot: "polar", r: close(cp.mean_speed), theta: th, name: r.run_name, legendgroup: r.run_name,
        mode: "lines+markers", line: { color: c, width: 2.2 }, marker: { size: 4 }, fill: "toself", fillcolor: rgba(c, 0.06),
        hovertemplate: "%{theta}: %{r:.2f} m/s<extra>" + r.run_name + "</extra>" });
      traces.push({ type: "scatterpolar", subplot: "polar2", r: close(cp.min_lidar_mean), theta: th, name: r.run_name, legendgroup: r.run_name,
        showlegend: false, mode: "lines+markers", line: { color: c, width: 2.2, dash: "dot" }, marker: { size: 4 },
        hovertemplate: "%{theta}: %{r:.2f} m<extra>" + r.run_name + "</extra>" });
    });
    const polar = (dom) => ({
      domain: { x: dom, y: [0, 0.92] }, bgcolor: "#10131a",
      angularaxis: { direction: "clockwise", rotation: 90, gridcolor: GRID, linecolor: AXL, tickfont: { size: 8 } },
      radialaxis: { gridcolor: GRID, linecolor: AXL, tickfont: { size: 9 }, angle: 90 },
    });
    Plotly.react("p1", traces, layout({
      uirevision: "polar", hovermode: "closest", polar: polar([0, 0.46]), polar2: polar([0.54, 1]),
      annotations: [
        { text: "MEAN SPEED (m/s)", x: 0.23, y: 1.02, xref: "paper", yref: "paper", showarrow: false, font: { size: 11, color: "#fff" } },
        { text: "MIN CLEARANCE (m)", x: 0.77, y: 1.02, xref: "paper", yref: "paper", showarrow: false, font: { size: 11, color: "#fff" } },
      ],
      legend: { orientation: "h", x: 0, y: -0.05, font: { size: 10 } },
    }), CFG);
  }
}

// ---------------------------------------------------------------- 02 crash blackspots
function renderP2(runs) {
  const withCp = runs.filter((r) => r.checkpoint);
  if (!withCp.length) { $("p2-sub").textContent = ""; return emptyPlot("p2", runs.length ? "No checkpoint data" : NO_RUNS); }
  const pct = state.crash === "pct";
  $("p2-sub").textContent = withCp.map((r) => `${r.run_name}: ${r.checkpoint.total_crashes}`).join(" · ") + " crashes";
  const traces = withCp.map((r) => {
    const cp = r.checkpoint, c = colorOf(r.run_name);
    return {
      type: "bar", x: cp.ids, y: pct ? cp.crash_pct : cp.crash_per_visit, name: r.run_name,
      marker: { color: rgba(c, 0.85) }, customdata: cp.ids.map((_, i) => [cp.crash_count[i], cp.visits[i]]),
      hovertemplate: (pct ? "%{y:.1f}% of crashes" : "Hazard %{y:.1f}% per visit") + " (%{customdata[0]} / %{customdata[1]} visits)<extra>" + r.run_name + "</extra>",
    };
  });
  Plotly.react("p2", traces, layout({
    uirevision: "p2" + state.crash, barmode: "group", bargap: 0.2,
    xaxis: { title: { text: "Checkpoint ID" }, dtick: 1 },
    yaxis: { title: { text: pct ? "Share of crashes (%)" : "Conditional crash hazard P(crash | visit) (%)" }, rangemode: "tozero" },
  }), CFG);
}

// ---------------------------------------------------------------- 03 performance vs survival
function renderP3(runs) {
  if (!runs.length) return emptyPlot("p3", NO_RUNS);
  const traces = [];
  const crashKey = state.crashrate === "cum" ? "crash_rate_cum" : "crash_rate_roll";
  runs.forEach((r) => {
    const s = r.series, c = colorOf(r.run_name);
    traces.push(...bandTraces(s.x, s.mean_speed, s.mean_speed_std, c, 0.15, "y", r.run_name, 0, null));
    traces.push({ type: "scattergl", mode: "lines", x: s.x, y: s.mean_speed, name: r.run_name, legendgroup: r.run_name,
      line: { color: c, width: 2.2 }, hovertemplate: "speed %{y:.2f} m/s" });
    traces.push(...bandTraces(s.x, s.survival, s.survival_std, c, 0.08, "y2", r.run_name, 0, null));
    traces.push({ type: "scattergl", mode: "lines", x: s.x, y: s.survival, yaxis: "y2", name: r.run_name + " survival", legendgroup: r.run_name, showlegend: false,
      line: { color: c, width: 1.4, dash: "dot" }, hovertemplate: "survival %{y:.0f} steps" });
    traces.push({ type: "scattergl", mode: "lines", x: s.x, y: s[crashKey], yaxis: "y3", name: r.run_name + " crash", legendgroup: r.run_name, showlegend: false,
      line: { color: rgba(c, 0.75), width: 1.4, dash: "dash" }, hovertemplate: "crash %{y:.1f}%" });
  });
  Plotly.react("p3", traces, layout({
    margin: { l: 50, r: 96, t: 26, b: 40 },
    xaxis: { title: { text: xTitle() }, domain: [0, 0.88] },
    yaxis: { title: { text: "Mean speed (m/s)" }, rangemode: "tozero" },
    yaxis2: { title: { text: "Survival (steps)" }, overlaying: "y", side: "right", anchor: "x", showgrid: false, rangemode: "tozero" },
    yaxis3: { title: { text: "Crash rate (%)" }, overlaying: "y", side: "right", anchor: "free", position: 0.985, showgrid: false, range: [0, 100] },
  }), CFG);
}

// ---------------------------------------------------------------- 04 control smoothness
function renderP4(runs) {
  if (!runs.length) return emptyPlot("p4", NO_RUNS);
  const traces = [];
  if (state.smooth === "scatter") {
    runs.forEach((r) => {
      const sc = r.scatter, c = colorOf(r.run_name);
      const y = scale(sc.steer_osc, 1000);
      traces.push({ type: "scattergl", mode: "markers", x: sc.mean_speed, y, name: r.run_name, legendgroup: r.run_name,
        marker: { color: rgba(c, 0.45), size: 6 }, customdata: sc.episode,
        hovertemplate: "ep %{customdata}<br>%{x:.2f} m/s · %{y:.1f} mrad/step<extra>" + r.run_name + "</extra>" });
      const mx = mean(sc.mean_speed), my = mean(y);
      if (mx !== null) traces.push({ type: "scatter", mode: "markers", x: [mx], y: [my], name: r.run_name + " centroid", legendgroup: r.run_name, showlegend: false,
        marker: { color: c, size: 16, symbol: "diamond", line: { color: "#fff", width: 1.5 } },
        hovertemplate: "centroid %{x:.2f} m/s · %{y:.1f} mrad/step<extra>" + r.run_name + "</extra>" });
    });
    Plotly.react("p4", traces, layout({
      uirevision: "p4s", hovermode: "closest",
      xaxis: { title: { text: "Episode mean speed (m/s)" } },
      yaxis: { title: { text: "Steering osc. (mrad/step)" }, rangemode: "tozero" },
    }), CFG);
  } else {
    runs.forEach((r) => {
      const c = colorOf(r.run_name);
      traces.push({ type: "scattergl", mode: "lines", x: r.series.x, y: scale(r.series.steer_osc, 1000), name: r.run_name,
        line: { color: c, width: 2 }, hovertemplate: "%{y:.1f} mrad/step" });
    });
    Plotly.react("p4", traces, layout({
      xaxis: { title: { text: xTitle() } },
      yaxis: { title: { text: "Steering osc. (mrad/step, rolling)" }, rangemode: "tozero" },
    }), CFG);
  }
}
function mean(a) {
  const v = (a || []).filter((x) => x !== null && x !== undefined);
  return v.length ? v.reduce((s, x) => s + x, 0) / v.length : null;
}

// ---------------------------------------------------------------- 05 real-time budget
function renderP5(runs) {
  const withInf = runs.filter((r) => hasData(r.series.inf_p50));
  if (!withInf.length) { emptyPlot("p5g", runs.length ? "No inference_ms logged" : NO_RUNS); return emptyPlot("p5", ""); }
  const n = withInf.length, pad = 0.02;
  const gauges = withInf.map((r, i) => {
    const c = colorOf(r.run_name), v = r.kpi.p95_ms ?? 0;
    const top = Math.max(120, v * 1.15);
    return {
      type: "indicator", mode: "gauge+number", value: v,
      number: { suffix: " ms", valueformat: ".1f", font: { size: 18, color: v > 100 ? "#E10600" : v > 25 ? "#FFB000" : "#3DDC84", family: "JetBrains Mono" } },
      title: { text: r.run_name, font: { size: 10.5, color: c } },
      domain: { x: [i / n + pad, (i + 1) / n - pad], y: [0, 0.85] },
      gauge: {
        shape: "angular", axis: { range: [0, top], tickfont: { size: 8 }, tickcolor: "#555", tickvals: [0, 25, 50, 100, Math.round(top)] },
        bar: { color: c, thickness: 0.3 }, bgcolor: "#10131a", borderwidth: 0,
        steps: [
          { range: [0, 25], color: "rgba(61,220,132,.16)" },
          { range: [25, 100], color: "rgba(255,176,0,.14)" },
          { range: [100, top], color: "rgba(225,6,0,.22)" },
        ],
        threshold: { line: { color: "#E10600", width: 3 }, thickness: 0.9, value: 25 },
      },
    };
  });
  Plotly.react("p5g", gauges, { paper_bgcolor: "rgba(0,0,0,0)", font: { family: "Titillium Web", color: "#c9d1d9" }, margin: { l: 18, r: 18, t: 22, b: 0 } }, { displayModeBar: false, responsive: true });

  const traces = [];
  let ymax = 110;
  withInf.forEach((r) => {
    const s = r.series, c = colorOf(r.run_name);
    (s.inf_p95_raw || []).forEach((v) => { if (v !== null) ymax = Math.max(ymax, v); });
    traces.push({ type: "scattergl", mode: "markers", x: s.x, y: s.inf_p95_raw, name: r.run_name + " p95 raw", legendgroup: r.run_name, showlegend: false,
      marker: { color: rgba(c, 0.18), size: 3 }, hoverinfo: "skip" });
    traces.push({ type: "scattergl", mode: "lines", x: s.x, y: s.inf_p50, name: r.run_name, legendgroup: r.run_name,
      line: { color: c, width: 2 }, hovertemplate: "p50 %{y:.1f} ms" });
    traces.push({ type: "scattergl", mode: "lines", x: s.x, y: s.inf_p95, name: r.run_name + " p95", legendgroup: r.run_name, showlegend: false,
      line: { color: c, width: 1.4, dash: "dash" }, hovertemplate: "p95 %{y:.1f} ms" });
  });
  const hline = (y, color, text) => ({
    shape: { type: "line", xref: "paper", x0: 0, x1: 1, y0: y, y1: y, line: { color, width: 2, dash: y === 25 ? "solid" : "dash" } },
    ann: { xref: "paper", x: 1, y: Math.log10(y), yref: "y", text, showarrow: false, xanchor: "right", yanchor: "bottom", font: { size: 9.5, color } },
  });
  const h1 = hline(25, "#E10600", "25 ms · 40 Hz LiDAR ceiling"), h2 = hline(100, "#FFB000", "100 ms · 10 Hz legacy");
  Plotly.react("p5", traces, layout({
    margin: { l: 54, r: 20, t: 8, b: 40 }, showlegend: false,
    xaxis: { title: { text: xTitle() } },
    yaxis: { title: { text: "Latency (ms, log)" }, type: "log", range: [0, Math.log10(ymax * 1.3)], tickvals: [1, 2, 5, 10, 25, 50, 100, 200, 500] },
    shapes: [h1.shape, h2.shape], annotations: [h1.ann, h2.ann],
  }), CFG);
}

// ---------------------------------------------------------------- 06 brain diagnostics
const MINI = (extra) => layout({ margin: { l: 44, r: 10, t: 6, b: 30 }, showlegend: false, font: { family: "Titillium Web", color: "#c9d1d9", size: 10 }, ...extra });

function norm01(a) {
  const v = (a || []).filter((x) => x !== null);
  if (!v.length) return a;
  const lo = Math.min(...v), hi = Math.max(...v);
  return a.map((x) => (x === null ? null : hi - lo < 1e-12 ? 0.5 : (x - lo) / (hi - lo)));
}

function renderP6(runs) {
  if (!runs.length) { ["p6a", "p6b", "p6c", "p6d", "p6e", "p6f"].forEach((id) => emptyPlot(id, NO_RUNS)); return; }

  // a) reward/CP vs speed scatter (de-confounded from survival length)
  Plotly.react("p6a", runs.map((r) => ({
    type: "scattergl", mode: "markers", x: r.scatter.mean_speed, y: r.scatter.reward_density || r.scatter.reward_per_cp || r.scatter.reward_per_step, name: r.run_name,
    marker: { color: rgba(colorOf(r.run_name), 0.5), size: 5 },
    hovertemplate: "%{x:.2f} m/s · %{y:.3f} rew/CP<extra>" + r.run_name + "</extra>",
  })), MINI({ uirevision: "p6a", hovermode: "closest", xaxis: { title: { text: "mean speed (m/s)" } }, yaxis: { title: { text: "reward / CP" } } }), CFG);

  // b) normalised reward/CP vs speed trend — divergence = reward hacking
  const tb = [];
  runs.forEach((r) => {
    const c = colorOf(r.run_name);
    const rewSeries = r.series.reward_density || r.series.reward_per_cp || r.series.total_reward;
    tb.push({ type: "scattergl", mode: "lines", x: r.series.x, y: norm01(rewSeries), name: r.run_name + " rew/CP", line: { color: c, width: 2 }, hovertemplate: "rew/CP (norm) %{y:.2f}" });
    tb.push({ type: "scattergl", mode: "lines", x: r.series.x, y: norm01(r.series.mean_speed), name: r.run_name + " speed", line: { color: c, width: 1.5, dash: "dot" }, hovertemplate: "speed (norm) %{y:.2f}" });
  });
  Plotly.react("p6b", tb, MINI({ xaxis: { title: { text: xTitle() } }, yaxis: { title: { text: "normalised [0-1]" }, range: [-0.05, 1.05] } }), CFG);

  // c) rolling Pearson r(reward/CP, mean_speed) with +/- 1 sigma band
  const tc = [];
  runs.forEach((r) => {
    const c = colorOf(r.run_name);
    tc.push(...bandTraces(r.series.x, r.series.corr_rv, r.series.corr_rv_std, c, 0.14, "y", r.run_name, -1.0, 1.0));
    tc.push({
      type: "scattergl", mode: "lines", x: r.series.x, y: r.series.corr_rv, name: r.run_name,
      line: { color: c, width: 2 }, hovertemplate: "r(rew/CP, v) = %{y:.3f}<extra>" + r.run_name + "</extra>",
    });
  });
  Plotly.react("p6c", tc, MINI({
    xaxis: { title: { text: xTitle() } }, yaxis: { title: { text: "Pearson r" }, range: [-1.05, 1.05], zeroline: true, zerolinecolor: "#555" },
  }), CFG);

  // d) policy entropy (discrete: H + Effective Active Bins K_eff; continuous/SAC: differential H + alpha)
  const isContinuousRun = (r) => {
    const algo = String((r.meta && r.meta.algo) || "").toUpperCase();
    return ["SAC", "TD3", "DDPG"].includes(algo) || (r.meta && r.meta.target_entropy != null);
  };
  const anyDiscrete = runs.some((r) => !isContinuousRun(r));
  const anyContinuous = runs.some((r) => isContinuousRun(r));
  const anyEntCoef = runs.some((r) => isContinuousRun(r) && r.train && hasData(r.train.ent_coef));
  const td = [];
  runs.forEach((r) => {
    const c = colorOf(r.run_name);
    const cont = isContinuousRun(r);
    if (r.train && hasData(r.train.entropy)) {
      if (cont) {
        td.push({
          type: "scattergl", mode: "lines+markers", x: r.train.x, y: r.train.entropy,
          name: r.run_name, line: { color: c, width: 2 }, marker: { size: 3 },
          hovertemplate: "H = %{y:.2f} nats (diff.)<extra>" + r.run_name + "</extra>",
        });
        if (hasData(r.train.ent_coef)) {
          td.push({
            type: "scattergl", mode: "lines", x: r.train.x, y: r.train.ent_coef, yaxis: "y2",
            name: r.run_name + " α", line: { color: rgba(c, 0.55), width: 1.4, dash: "dot" },
            hovertemplate: "α = %{y:.4f}<extra>" + r.run_name + "</extra>",
          });
        }
      } else {
        const keff = r.train.k_eff || r.train.entropy.map((h) => (h === null ? null : Math.exp(h / 2)));
        td.push({
          type: "scattergl", mode: "lines+markers", x: r.train.x, y: r.train.entropy, customdata: keff,
          name: r.run_name, line: { color: c, width: 2 }, marker: { size: 3 },
          hovertemplate: "H = %{y:.2f} nats · K<sub>eff</sub> = %{customdata:.2f} bins<extra>" + r.run_name + "</extra>",
        });
        td.push({
          type: "scattergl", mode: "lines", x: r.train.x, y: keff, yaxis: "y2",
          name: r.run_name + " K_eff", showlegend: false, hoverinfo: "skip",
          line: { color: rgba(c, 0.0), width: 0 },
        });
      }
    } else if (hasData(r.series.entropy_ep)) {
      td.push(...bandTraces(r.series.x, r.series.entropy_ep, r.series.entropy_ep_std, c, 0.14, "y", r.run_name, 0, null));
      if (cont) {
        td.push({
          type: "scattergl", mode: "lines", x: r.series.x, y: r.series.entropy_ep,
          name: r.run_name, line: { color: c, width: 2, dash: "dot" },
          hovertemplate: "H = %{y:.2f} nats (diff.)<extra>" + r.run_name + "</extra>",
        });
      } else {
        const keff = r.series.k_eff_ep || r.series.entropy_ep.map((h) => (h === null ? null : Math.exp(h / 2)));
        td.push({
          type: "scattergl", mode: "lines", x: r.series.x, y: r.series.entropy_ep, customdata: keff,
          name: r.run_name, line: { color: c, width: 2, dash: "dot" },
          hovertemplate: "H = %{y:.2f} nats · K<sub>eff</sub> = %{customdata:.2f} bins<extra>" + r.run_name + "</extra>",
        });
        td.push({
          type: "scattergl", mode: "lines", x: r.series.x, y: keff, yaxis: "y2",
          name: r.run_name + " K_eff", showlegend: false, hoverinfo: "skip",
          line: { color: rgba(c, 0.0), width: 0 },
        });
      }
    }
  });
  if (td.length) {
    const shapes = [];
    const annotations = [];
    if (anyDiscrete) {
      shapes.push({
        type: "rect", xref: "paper", yref: "y", x0: 0, x1: 1, y0: 1.8, y1: 2.5,
        fillcolor: "rgba(0, 210, 190, 0.10)",
        line: { color: "rgba(0, 210, 190, 0.45)", width: 1, dash: "dot" },
      });
      annotations.push({
        xref: "paper", yref: "y", x: 0.98, y: 2.15, xanchor: "right", showarrow: false,
        text: "Optimal (K_eff ≈ 2.5–3.5 bins)", font: { size: 9, color: "#00D2BE" },
      });
    }
    if (anyContinuous) {
      const targetRun = runs.find((r) => isContinuousRun(r) && r.meta && r.meta.target_entropy != null);
      const targetH = targetRun ? Number(targetRun.meta.target_entropy) : -2.0;
      shapes.push({
        type: "line", xref: "paper", yref: "y", x0: 0, x1: 1, y0: targetH, y1: targetH,
        line: { color: "rgba(0, 210, 190, 0.65)", width: 1.2, dash: "dash" },
      });
      annotations.push({
        xref: "paper", yref: "y", x: 0.98, y: targetH, yshift: 8, xanchor: "right", showarrow: false,
        text: `SAC target H (${targetH.toFixed(1)} nats)`, font: { size: 9, color: "#00D2BE" },
      });
    }
    const y2Title = anyContinuous && !anyDiscrete ? (anyEntCoef ? "α (ent_coef)" : "") : "K_eff (bins)";
    Plotly.react("p6d", td, MINI({
      margin: { l: 44, r: 46, t: 6, b: 30 },
      xaxis: { title: { text: xTitle() } },
      yaxis: { title: { text: "H (nats)" } },
      yaxis2: { title: { text: y2Title }, overlaying: "y", side: "right", showgrid: false },
      shapes,
      annotations,
    }), CFG);
  } else {
    emptyPlot("p6d", "No entropy yet — use TelemetryCallback or log_train(entropy=…)");
  }

  // e/f) optional slots
  optionalSlot("p6e", runs, "ts_gap", "‖πS − πT‖₂", "SLOT AVAILABLE · pass ts_gap=… to log_step()");
  optionalSlot("p6f", runs, "wm_err", "recon. error", "SLOT AVAILABLE · pass wm_err=… to log_step()");
}

function optionalSlot(div, runs, key, ytitle, msg) {
  const tr = runs.filter((r) => hasData(r.series[key])).map((r) => ({
    type: "scattergl", mode: "lines", x: r.series.x, y: r.series[key], name: r.run_name,
    line: { color: colorOf(r.run_name), width: 2 }, hovertemplate: "%{y:.4f}",
  }));
  if (!tr.length) return emptyPlot(div, msg);
  Plotly.react(div, tr, MINI({ xaxis: { title: { text: xTitle() } }, yaxis: { title: { text: ytitle }, rangemode: "tozero" } }), CFG);
}

// ============================================================ boot
tick(true);
setInterval(tick, 2000);
