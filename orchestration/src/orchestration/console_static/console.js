"use strict";

const $ = (id) => document.getElementById(id);
let token = "", presets = [], current = {}, shownRun = null, busy = false;
let prepareRequest = null, cameraFailed = false;
let gazeboAvailable = false;
const terminal = new Set(["COMPLETED", "FAILED", "ABORTED", "INTERRUPTED"]);
const text = (id, value) => { $(id).textContent = value ?? "—"; };
const show = (id, visible) => $(id).classList.toggle("hidden", !visible);
const pretty = (value) => JSON.stringify(value, null, 2);

async function api(path, body) {
  const response = await fetch(`/api/${path}`, body === undefined ? {} : {
    method: "POST", headers: {"Content-Type": "application/json", "X-Console-Token": token}, body: JSON.stringify(body)
  });
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `Request failed (${response.status})`);
  return data;
}
function error(message) { text("error", message); show("error", Boolean(message)); }
function selected() { return presets.find((preset) => preset.id === $("preset").value); }
function options(element, values) {
  element.replaceChildren(...values.map(([value, label]) => {
    const option = document.createElement("option"); option.value = value; option.textContent = label; return option;
  }));
}
function updatePreset() {
  const preset = selected();
  if (!preset) return;
  text("preset-description", `${preset.vehicle.replaceAll("_", " ")} · ${preset.scenario.replaceAll("_", " ")}`);
  text("configuration", pretty(preset.configuration));
  show("manual-note", preset.manual_arming); show("calibration-field", preset.requires_calibration);
  prepareRequest = null;
}
function updateWorld(wanted) {
  const matching = presets.filter((preset) => preset.world === $("world").value);
  options($("preset"), matching.map((preset) => [preset.id, preset.label]));
  if (matching.some((preset) => preset.id === wanted)) $("preset").value = wanted;
  updatePreset();
}
function renderChecks(data) {
  if (!data) return;
  const rows = data.checks.map((check) => {
    const row = document.createElement("div"); row.className = `check-row ${check.ok ? "ok" : "fail"}`;
    row.textContent = `${check.ok ? "✓" : "!"} ${check.name}`;
    const detail = document.createElement("span"); detail.textContent = check.detail; row.append(detail); return row;
  });
  $("checks").replaceChildren(...rows); text("provenance", pretty(data.provenance || {}));
}
function viewStatus(id, live, label) { text(id, label || (live ? "Live" : "Disconnected")); $(id).classList.toggle("live", live); }
function connectCamera() {
  if (!current.run_id) return;
  cameraFailed = false;
  $("camera").src = `/camera.mjpg?run=${encodeURIComponent(current.run_id)}&retry=${Date.now()}`;
  show("camera", true);
}
function render(snapshot) {
  current = snapshot;
  const state = snapshot.state;
  const active = !terminal.has(state) && state !== "IDLE";
  const waiting = ["CREATED", "STARTING", "READY", "PREPARING"].includes(state);
  text("session-state", state.toLowerCase().replaceAll("_", " "));
  $("connection-dot").className = `dot ${["PREPARED", "RUNNING"].includes(state) ? "live" : ""}`;
  const titles = {IDLE: "Ready to prepare", PREPARING: "Checking & preparing", CREATED: "Allocating simulation", STARTING: "Starting the local runtime", READY: "Waiting for live preflight", PREPARED: "Preflight inspection", STARTING_MISSION: "Admitting mission start", RUNNING: "Mission in progress", FINALIZING: "Finalizing evidence", COMPLETED: "Run completed", FAILED: "Run needs attention", ABORTED: "Simulation stopped", INTERRUPTED: "Interrupted run retained", ORPHANED: "Session owner disconnected", BUILDING: "Building local runtime images"};
  text("flight-title", titles[state] || state);
  let detail = waiting ? "Bringing up physics, sensors, and the disarmed vehicle. This does not start the mission." : snapshot.reason || "";
  if (state === "PREPARED") detail = snapshot.qgc_admission_ready ? "Native preflight is ready. Select this run's action file and start from QGroundControl when ready." : "Waiting for fresh vehicle, native clock, calibration, and producer readiness.";
  if (state === "RUNNING") detail = "The selected onboard mission owns flight commands. QGC controls remain available and interventions are recorded.";
  if (state === "RUNNING" && snapshot.recovery) {
    const recovered = snapshot.recovery.state === "landed_disarmed";
    const needsAttention = ["failed", "rejected", "interrupted"].includes(snapshot.recovery.state);
    text("flight-title", recovered ? "Flight recovery complete" : needsAttention ? "Flight recovery needs attention" : "Recovering flight");
    detail = recovered ? "The vehicle is confirmed landed and disarmed. Stop simulation to finalize this run."
      : needsAttention ? "Landing is not confirmed. Inspect QGC; Stop simulation ends this run."
      : "Onboard recovery is active. Watch QGC and the live views; Stop simulation remains separate.";
  }
  if (state === "BUILDING") detail = "Building matching native images. Open Runtime logs & build output below to follow progress.";
  if (snapshot.preflight && !snapshot.preflight.ready && (waiting || state === "PREPARED" || state === "STARTING_MISSION")) detail += ` ${snapshot.preflight.reasons.join("; ")}`;
  text("flight-detail", detail || "Select a preset, prepare the simulation, then use the run-bound actions in QGroundControl.");
  text("run-id", snapshot.run_id ? snapshot.run_id.slice(0, 8) : "No active run");
  const host = snapshot.host || {};
  const hostNames = {wsl2: "WSL2", macos: "macOS", linux: "Linux", amd64: "AMD64", arm64: "ARM64", "docker-desktop": "Docker Desktop", colima: "Colima", docker: "Docker"};
  text("host-platform", [host.kind, host.machine, host.provider].filter(Boolean).map((item) => hostNames[item] || item).join(" · ") || "Host platform unavailable");
  ["world", "preset", "calibration", "preparation-limit"].forEach((id) => { $(id).disabled = active || busy; });
  const cleanupReady = !snapshot.run_id || snapshot.cleanup === "complete";
  const cleanupRecovery = Boolean(snapshot.run_id) && terminal.has(state) && !cleanupReady;
  $("prepare").disabled = active || busy || !cleanupReady; $("build").disabled = active || busy || !cleanupReady;
  $("stop").disabled = (!active && !cleanupRecovery) || ["BUILDING", "FINALIZING"].includes(state) || busy;
  show("again", terminal.has(state) && Boolean(snapshot.run_id)); $("again").disabled = busy || !cleanupReady;
  if (snapshot.error || snapshot.observation_error) error(snapshot.error || snapshot.observation_error);
  renderChecks(snapshot.checks);
  if (snapshot.run_id && snapshot.run_id !== shownRun) {
    shownRun = snapshot.run_id;
    gazeboAvailable = false;
    show("gazebo", false); show("gazebo-empty", true); connectCamera();
  }
  const views = snapshot.views || {}, vehicle = views.qgc || {};
  if (views.gazebo?.available && !gazeboAvailable) {
    $("gazebo").src = `http://127.0.0.1:6080/operator.html?run=${encodeURIComponent(snapshot.run_id)}`;
    show("gazebo", true); show("gazebo-empty", false);
  }
  gazeboAvailable = Boolean(views.gazebo?.available);
  viewStatus("gazebo-status", Boolean(views.gazebo?.connected));
  const cameraLive = Boolean(views.camera?.fresh) && !cameraFailed;
  viewStatus("camera-status", cameraLive, cameraLive ? "Live sensor" : snapshot.run_id ? "Stale / disconnected" : "Disconnected");
  show("camera-stale", Boolean(snapshot.run_id) && !cameraLive); $("camera").classList.toggle("stale", !cameraLive);
  show("camera-empty", !snapshot.run_id || cameraFailed);
  const stamp = views.camera?.native_timestamp_ns;
  text("camera-stamp", stamp == null ? "Native sensor time —" : `Native sensor time ${(stamp / 1e9).toFixed(2)} s`);
  const qgcLaunchDetail = snapshot.qgc_launch?.opened === false ? snapshot.qgc_launch.detail : "";
  text("qgc-status", vehicle.qgc_fresh ? "QGC connected · live vehicle telemetry" : vehicle.fresh ? "SITL connected · waiting for QGC" : vehicle.qgc_detail || vehicle.detail || qgcLaunchDetail || `Native ${hostNames[host.kind] || "host"} window`);
  text("qgc-action-path", snapshot.qgc_action_path || "Prepare a run to publish its action file.");
  text("mode", vehicle.fresh ? vehicle.mode : "—");
  text("armed", vehicle.fresh && typeof vehicle.armed === "boolean" ? (vehicle.armed ? "Armed" : "Disarmed") : "—");
  text("altitude", vehicle.fresh && vehicle.relative_altitude_m != null ? `${vehicle.relative_altitude_m.toFixed(2)} m` : "—");
  text("elapsed", snapshot.admission && stamp != null ? `${Math.max(0, (stamp - snapshot.admission.native_epoch_ns) / 1e9).toFixed(2)} s` : snapshot.admission ? "—" : "Not started");
  show("recovery-status", Boolean(snapshot.recovery));
  if (snapshot.recovery) text("recovery-status", `Flight recovery: ${snapshot.recovery.state.replaceAll("_", " ")} — ${snapshot.recovery.detail}`);
  text("cleanup", `Cleanup: ${snapshot.cleanup || "pending"}`); text("evidence-path", snapshot.run_directory || "");
}
async function results() {
  const result = await api("results"), manifest = result.manifest;
  text("result-lifecycle", manifest?.terminal_status || (current.run_id ? "Awaiting terminal manifest" : "—"));
  text("result-physical", result.physical_outcome ? "Mission completion observed" : current.recovery?.state === "landed_disarmed" ? "Recovered: landed & disarmed" : "Not established");
  text("result-score", result.score?.achieved_score != null ? `${result.score.achieved_score} / ${result.score.maximum_available_score}` : "Not available");
  text("result-intervention", !current.run_id ? "—" : result.interventions ? "QGC control commands recorded" : "None recorded");
  text("result-artifacts", manifest ? (manifest.incomplete_paths?.length ? "Incomplete / invalid" : manifest.artifacts?.every((item) => item.validation === "valid") ? "Valid" : "See manifest validation") : "Not finalized");
  text("result-details", pretty(result));
}
async function action(operation) {
  if (busy) return;
  busy = true; error(""); render(current);
  try { await operation(); await poll(); } catch (failure) { error(failure.message); }
  finally { busy = false; render(current); }
}
async function prepare() {
  if (terminal.has(current.state)) prepareRequest = null;
  prepareRequest ||= crypto.randomUUID();
  const body = {preset_id: $("preset").value, request_id: prepareRequest, preparation_wall_seconds: Number($("preparation-limit").value)};
  if (selected()?.requires_calibration) body.calibration_source = $("calibration").value;
  render(await api("prepare", body));
}
async function poll() { render(await api("session")); }
$("world").addEventListener("change", () => updateWorld()); $("preset").addEventListener("change", updatePreset);
$("prepare").addEventListener("click", () => action(prepare));
$("again").addEventListener("click", () => action(async () => { prepareRequest = null; await prepare(); }));
$("stop").addEventListener("click", () => action(() => api("stop", {run_id: current.run_id})));
$("qgc").addEventListener("click", () => action(() => api("open-qgc", {})));
$("check").addEventListener("click", () => action(async () => renderChecks(await api("checks"))));
$("build").addEventListener("click", () => action(() => api("build", {})));
$("camera-reconnect").addEventListener("click", connectCamera);
$("camera").addEventListener("error", () => { cameraFailed = true; show("camera-stale", true); });
$("camera").addEventListener("load", () => { cameraFailed = false; show("camera-empty", false); });
window.addEventListener("message", (event) => {
  if (event.origin !== "http://127.0.0.1:6080" || event.source !== $("gazebo").contentWindow
      || event.data?.type !== "drone-sim-desktop" || event.data.run_id !== current.run_id
      || typeof event.data.connected !== "boolean") return;
  api("viewer-status", {run_id: current.run_id, connected: event.data.connected}).catch(() => {});
});

(async () => {
  try {
    token = (await api("bootstrap")).token;
    presets = (await api("catalog")).presets;
    options($("world"), [...new Set(presets.map((preset) => preset.world))].map((world) => [world, world.replaceAll("_", " ")]));
    $("world").value = "vertical_descent"; updateWorld("configured-descent");
    await poll();
    if (current.preset_id) { const preset = presets.find((row) => row.id === current.preset_id); if (preset) { $("world").value = preset.world; updateWorld(preset.id); } }
    api("checks").then(renderChecks).catch((failure) => error(failure.message));
    setInterval(async () => { try { await poll(); } catch (failure) { render({...current, can_start: false, views: {}}); text("session-state", "Console disconnected"); $("connection-dot").className = "dot error"; error(failure.message); } }, 2000);
    setInterval(async () => { try { await results(); if (document.querySelector(".logs").open) text("logs", pretty(await api("logs"))); } catch (_) {} if (cameraFailed && current.views?.camera?.fresh) connectCamera(); }, 4000);
  } catch (failure) { error(failure.message); }
})();
