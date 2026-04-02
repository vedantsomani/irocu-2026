import { useState, useEffect, useRef, useCallback } from "react";

// ─── CONFIG ──────────────────────────────────────────────────────────────────
const DEFAULT_API = window.location.origin; // Same origin in dev, or set manually
const POLL_MS = 200;

// ─── HELPERS ─────────────────────────────────────────────────────────────────
const fmt = (v, d = 1) => (v != null && isFinite(v) ? Number(v).toFixed(d) : "—");
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const deg2rad = (d) => (d * Math.PI) / 180;

async function apiPost(base, path, body = {}) {
  try {
    const r = await fetch(`${base}${path}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    return await r.json();
  } catch (e) {
    return { ok: false, message: String(e) };
  }
}

// ─── ATTITUDE INDICATOR (Canvas) ─────────────────────────────────────────────
function AttitudeIndicator({ roll = 0, pitch = 0, size = 200 }) {
  const ref = useRef(null);
  useEffect(() => {
    const c = ref.current;
    if (!c) return;
    const ctx = c.getContext("2d");
    const w = size, h = size, cx = w / 2, cy = h / 2, r = w * 0.42;
    ctx.clearRect(0, 0, w, h);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(deg2rad(-roll));

    const pitchPx = (pitch / 90) * r * 2;
    // Sky
    ctx.beginPath();
    ctx.rect(-r, -r * 2, r * 2, r * 2);
    ctx.fillStyle = "#1a5276";
    ctx.fill();
    // Ground
    ctx.beginPath();
    ctx.rect(-r, 0, r * 2, r * 2);
    ctx.fillStyle = "#6e3c10";
    ctx.fill();
    // Shift for pitch
    ctx.translate(0, pitchPx);
    // Horizon line
    ctx.beginPath();
    ctx.moveTo(-r, 0);
    ctx.lineTo(r, 0);
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 2;
    ctx.stroke();
    // Pitch ladder
    for (let p = -40; p <= 40; p += 10) {
      if (p === 0) continue;
      const y = (-p / 90) * r * 2;
      const half = p % 20 === 0 ? r * 0.3 : r * 0.15;
      ctx.beginPath();
      ctx.moveTo(-half, y);
      ctx.lineTo(half, y);
      ctx.strokeStyle = "rgba(255,255,255,0.5)";
      ctx.lineWidth = 1;
      ctx.stroke();
      if (p % 20 === 0) {
        ctx.fillStyle = "rgba(255,255,255,0.6)";
        ctx.font = "10px monospace";
        ctx.textAlign = "right";
        ctx.fillText(`${Math.abs(p)}`, -half - 4, y + 3);
      }
    }
    ctx.restore();

    // Clip circle
    ctx.save();
    ctx.globalCompositeOperation = "destination-in";
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();

    // Border
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.strokeStyle = "#555";
    ctx.lineWidth = 2;
    ctx.stroke();

    // Aircraft symbol (fixed)
    ctx.strokeStyle = "#f1c40f";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(cx - 30, cy);
    ctx.lineTo(cx - 10, cy);
    ctx.moveTo(cx + 10, cy);
    ctx.lineTo(cx + 30, cy);
    ctx.moveTo(cx, cy - 2);
    ctx.lineTo(cx, cy + 8);
    ctx.stroke();

    // Center dot
    ctx.beginPath();
    ctx.arc(cx, cy, 3, 0, Math.PI * 2);
    ctx.fillStyle = "#f1c40f";
    ctx.fill();
  }, [roll, pitch, size]);

  return <canvas ref={ref} width={size} height={size} style={{ display: "block" }} />;
}

// ─── POSITION MAP (Canvas) ───────────────────────────────────────────────────
function PositionMap({ x = 0, y = 0, trail = [], size = 200 }) {
  const ref = useRef(null);
  useEffect(() => {
    const c = ref.current;
    if (!c) return;
    const ctx = c.getContext("2d");
    const w = size, h = size, cx = w / 2, cy = h / 2;
    ctx.clearRect(0, 0, w, h);

    // Background grid
    ctx.strokeStyle = "rgba(255,255,255,0.06)";
    ctx.lineWidth = 0.5;
    const step = 20;
    for (let i = 0; i <= w; i += step) {
      ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, h); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(w, i); ctx.stroke();
    }

    // Scale: 1m = 20px
    const scale = 20;

    // Trail
    if (trail.length > 1) {
      ctx.beginPath();
      ctx.moveTo(cx + trail[0].y * scale, cy - trail[0].x * scale);
      for (let i = 1; i < trail.length; i++) {
        ctx.lineTo(cx + trail[i].y * scale, cy - trail[i].x * scale);
      }
      ctx.strokeStyle = "rgba(46, 204, 113, 0.4)";
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    // Home marker
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.strokeStyle = "#e74c3c";
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Current position
    const px = cx + y * scale;
    const py = cy - x * scale;
    ctx.beginPath();
    ctx.arc(px, py, 5, 0, Math.PI * 2);
    ctx.fillStyle = "#2ecc71";
    ctx.fill();
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 1;
    ctx.stroke();

    // Scale label
    ctx.fillStyle = "rgba(255,255,255,0.3)";
    ctx.font = "9px monospace";
    ctx.fillText("1m grid", 4, h - 4);
  }, [x, y, trail, size]);

  return <canvas ref={ref} width={size} height={size} style={{ display: "block" }} />;
}

// ─── BATTERY BAR ─────────────────────────────────────────────────────────────
function BatteryBar({ voltage, soc, cells = [] }) {
  const pct = soc != null ? clamp(soc, 0, 100) : 0;
  const color = pct > 40 ? "#2ecc71" : pct > 20 ? "#f39c12" : "#e74c3c";
  return (
    <div>
      <div style={{ display: "flex", justifyContent: "space-between", marginBottom: 4 }}>
        <span style={{ color: "#aaa", fontSize: 11 }}>BATTERY</span>
        <span style={{ color, fontSize: 12, fontWeight: 600 }}>{fmt(soc, 0)}%</span>
      </div>
      <div style={{ background: "#1a1a2e", borderRadius: 3, height: 10, overflow: "hidden" }}>
        <div style={{ width: `${pct}%`, height: "100%", background: color, borderRadius: 3, transition: "width 0.3s" }} />
      </div>
      <div style={{ display: "flex", justifyContent: "space-between", marginTop: 4, fontSize: 10, color: "#888" }}>
        <span>{fmt(voltage, 2)}V</span>
        <span>{cells.filter(c => c != null).map(c => fmt(c, 2) + "V").join(" · ") || "—"}</span>
      </div>
    </div>
  );
}

// ─── MAIN APP ────────────────────────────────────────────────────────────────
export default function App() {
  const [api, setApi] = useState(DEFAULT_API);
  const [telem, setTelem] = useState({});
  const [connected, setConnected] = useState(false);
  const [trail, setTrail] = useState([]);
  const [log, setLog] = useState([]);
  const [confirmForce, setConfirmForce] = useState(false);
  const [takeoffAlt, setTakeoffAlt] = useState(2.0);
  const trailRef = useRef([]);

  const addLog = useCallback((msg, type = "info") => {
    setLog((prev) => [...prev.slice(-50), { t: Date.now(), msg, type }]);
  }, []);

  // Telemetry polling
  useEffect(() => {
    let alive = true;
    const poll = async () => {
      while (alive) {
        try {
          const r = await fetch(`${api}/api/telemetry`, { cache: "no-store" });
          if (!alive) break;
          if (r.ok) {
            const d = await r.json();
            setTelem(d);
            setConnected(true);
            // Trail
            if (d.pos_x != null && d.pos_y != null) {
              const last = trailRef.current[trailRef.current.length - 1];
              if (!last || Math.abs(last.x - d.pos_x) > 0.02 || Math.abs(last.y - d.pos_y) > 0.02) {
                trailRef.current = [...trailRef.current.slice(-200), { x: d.pos_x, y: d.pos_y }];
                setTrail(trailRef.current);
              }
            }
          } else {
            setConnected(false);
          }
        } catch {
          setConnected(false);
        }
        await new Promise((r) => setTimeout(r, POLL_MS));
      }
    };
    poll();
    return () => { alive = false; };
  }, [api]);

  // Commands
  const doCmd = async (path, body, label) => {
    addLog(`→ ${label}...`, "cmd");
    const r = await apiPost(api, path, body);
    addLog(`← ${r.message || r.error || JSON.stringify(r)}`, r.ok === false ? "err" : "ok");
  };

  const doArm = () => doCmd("/api/arm", {}, "ARM");
  const doDisarm = () => doCmd("/api/disarm", {}, "DISARM");
  const doForceDisarm = () => { doCmd("/api/force_disarm", { confirm: true }, "FORCE DISARM"); setConfirmForce(false); };
  const doTakeoff = () => doCmd("/api/takeoff", { altitude: takeoffAlt }, `TAKEOFF ${takeoffAlt}m`);
  const doArmAndTakeoff = () => doCmd("/api/arm_and_takeoff", { altitude: takeoffAlt }, `ARM + TAKEOFF ${takeoffAlt}m`);
  const doHover = () => doCmd("/api/hover", {}, "HOVER HOLD");
  const doLand = () => doCmd("/api/land", {}, "SOFT LAND");
  const doMode = (m) => doCmd(`/api/mode/${m}`, {}, `MODE → ${m}`);

  const armed = telem.armed;
  const mode = telem.mode || "—";
  const guidanceMode = telem.guidance_mode || "IDLE";

  // Styles
  const panel = {
    background: "#0d0d1a",
    border: "1px solid #1a1a2e",
    borderRadius: 6,
    padding: "12px 14px",
    fontSize: 12,
  };
  const label = { color: "#666", fontSize: 10, textTransform: "uppercase", letterSpacing: 1 };
  const val = { color: "#e0e0e0", fontSize: 14, fontFamily: "'JetBrains Mono', 'Fira Code', monospace" };
  const row = { display: "flex", justifyContent: "space-between", alignItems: "center", padding: "3px 0" };

  const feedUrl = connected ? `${api}/feed/color` : "";

  return (
    <div style={{ background: "#080810", color: "#ccc", minHeight: "100vh", fontFamily: "'Inter', 'Segoe UI', sans-serif" }}>
      {/* Header */}
      <div style={{ background: "#0a0a16", borderBottom: "1px solid #1a1a2e", padding: "8px 16px", display: "flex", justifyContent: "space-between", alignItems: "center" }}>
        <div style={{ display: "flex", alignItems: "center", gap: 12 }}>
          <span style={{ fontWeight: 700, fontSize: 15, color: "#fff", letterSpacing: 2 }}>ASCEND GCS</span>
          <span style={{ fontSize: 10, color: "#555" }}>IRoC-U 2026 · Saarabai X</span>
        </div>
        <div style={{ display: "flex", alignItems: "center", gap: 10 }}>
          <input
            value={api}
            onChange={(e) => setApi(e.target.value)}
            style={{ background: "#111", border: "1px solid #222", color: "#888", padding: "4px 8px", borderRadius: 3, fontSize: 11, width: 200, fontFamily: "monospace" }}
            placeholder="http://pi-ip:5000"
          />
          <div style={{
            width: 8, height: 8, borderRadius: "50%",
            background: connected ? (armed ? "#e74c3c" : "#2ecc71") : "#555",
            boxShadow: connected ? `0 0 6px ${armed ? "#e74c3c" : "#2ecc71"}` : "none",
          }} />
          <span style={{ fontSize: 10, color: connected ? "#aaa" : "#555" }}>
            {connected ? (armed ? "ARMED" : "DISARMED") : "OFFLINE"}
          </span>
        </div>
      </div>

      {/* Main Grid */}
      <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 280px", gap: 10, padding: 12, maxWidth: 1400, margin: "0 auto" }}>

        {/* ── Col 1: Attitude + Position + Telem ── */}
        <div style={{ display: "flex", flexDirection: "column", gap: 10 }}>
          {/* Attitude + Position side by side */}
          <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 10 }}>
            <div style={panel}>
              <div style={{ ...label, marginBottom: 6 }}>ATTITUDE</div>
              <div style={{ display: "flex", justifyContent: "center" }}>
                <AttitudeIndicator roll={telem.roll || 0} pitch={telem.pitch || 0} size={170} />
              </div>
              <div style={{ display: "flex", justifyContent: "space-around", marginTop: 8, fontSize: 11 }}>
                <span>R <b style={{ color: "#3498db" }}>{fmt(telem.roll)}°</b></span>
                <span>P <b style={{ color: "#3498db" }}>{fmt(telem.pitch)}°</b></span>
                <span>Y <b style={{ color: "#3498db" }}>{fmt(telem.yaw)}°</b></span>
              </div>
            </div>
            <div style={panel}>
              <div style={{ ...label, marginBottom: 6 }}>POSITION NED</div>
              <div style={{ display: "flex", justifyContent: "center" }}>
                <PositionMap x={telem.pos_x || 0} y={telem.pos_y || 0} trail={trail} size={170} />
              </div>
              <div style={{ display: "flex", justifyContent: "space-around", marginTop: 8, fontSize: 11 }}>
                <span>N <b>{fmt(telem.pos_x)}m</b></span>
                <span>E <b>{fmt(telem.pos_y)}m</b></span>
                <span>ALT <b style={{ color: "#f1c40f" }}>{fmt(telem.alt_rel)}m</b></span>
              </div>
            </div>
          </div>

          {/* Core telemetry */}
          <div style={panel}>
            <div style={{ ...label, marginBottom: 6 }}>FLIGHT DATA</div>
            <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: "4px 16px" }}>
              {[
                ["Mode", mode, armed ? "#e74c3c" : "#2ecc71"],
                ["Guide", guidanceMode, guidanceMode !== "IDLE" ? "#2ecc71" : "#777"],
                ["Alt", `${fmt(telem.alt_rel)}m`, "#f1c40f"],
                ["Hdg", `${fmt(telem.yaw, 0)}°`],
                ["Vx", `${fmt(telem.vx, 2)} m/s`],
                ["Vy", `${fmt(telem.vy, 2)} m/s`],
                ["Vz", `${fmt(telem.vz, 2)} m/s`],
                ["Vis N", `${fmt(telem.viso_vn, 2)} m/s`, "#9b59b6"],
                ["Vis E", `${fmt(telem.viso_ve, 2)} m/s`, "#9b59b6"],
                ["Vis FPS", fmt(telem.vision_fps, 0), "#9b59b6"],
              ].map(([k, v, c], i) => (
                <div key={i} style={row}>
                  <span style={{ ...label, fontSize: 9 }}>{k}</span>
                  <span style={{ ...val, fontSize: 12, color: c || "#e0e0e0" }}>{v}</span>
                </div>
              ))}
            </div>
          </div>

          {/* IMU */}
          <div style={panel}>
            <div style={{ ...label, marginBottom: 6 }}>IMU / VIBRATION</div>
            <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: "2px 16px", fontSize: 11 }}>
              {[
                ["Ax", fmt(telem.imu_ax, 2)],
                ["Ay", fmt(telem.imu_ay, 2)],
                ["Az", fmt(telem.imu_az, 2)],
                ["Gx", fmt(telem.imu_gx, 1)],
                ["Gy", fmt(telem.imu_gy, 1)],
                ["Gz", fmt(telem.imu_gz, 1)],
                ["VibX", fmt(telem.vibe_x, 2)],
                ["VibY", fmt(telem.vibe_y, 2)],
                ["VibZ", fmt(telem.vibe_z, 2)],
              ].map(([k, v], i) => (
                <div key={i} style={row}>
                  <span style={{ color: "#555", fontSize: 9 }}>{k}</span>
                  <span style={{ fontFamily: "monospace", fontSize: 11 }}>{v}</span>
                </div>
              ))}
            </div>
          </div>
        </div>

        {/* ── Col 2: Camera + Battery ── */}
        <div style={{ display: "flex", flexDirection: "column", gap: 10 }}>
          <div style={panel}>
            <div style={{ ...label, marginBottom: 6 }}>CAMERA FEED</div>
            <div style={{
              background: "#000", borderRadius: 4, overflow: "hidden",
              aspectRatio: "4/3", display: "flex", alignItems: "center", justifyContent: "center",
            }}>
              {feedUrl ? (
                <img
                  src={feedUrl}
                  alt="Camera"
                  style={{ width: "100%", height: "100%", objectFit: "contain" }}
                  onError={(e) => { e.target.style.display = "none"; }}
                />
              ) : (
                <span style={{ color: "#333", fontSize: 12 }}>NO FEED</span>
              )}
            </div>
            <div style={{ display: "flex", justifyContent: "space-between", marginTop: 6, fontSize: 10, color: "#555" }}>
              <span>SRC: {telem.camera_source || "—"}</span>
              <span>{telem.camera_w || "—"}×{telem.camera_h || "—"}</span>
            </div>
          </div>

          <div style={panel}>
            <BatteryBar voltage={telem.bat_voltage} soc={telem.bat_soc} cells={telem.cell_voltages || []} />
            <div style={{ ...row, marginTop: 6, fontSize: 10, color: "#555" }}>
              <span>Current: {fmt(telem.bat_current, 1)}A</span>
              <span>Temp: {fmt(telem.bat_temp, 1)}°C</span>
            </div>
          </div>

          <div style={panel}>
            <div style={{ ...label, marginBottom: 6 }}>SYSTEM STATUS</div>
            {[
              ["Pixhawk", telem.pixhawk_ok],
              ["D435i", telem.d435i_ok],
              ["EKF", telem.ekf_ok],
              ["Camera", telem.camera_feed_ok],
            ].map(([name, ok], i) => (
              <div key={i} style={{ ...row, fontSize: 11 }}>
                <span>{name}</span>
                <span style={{ color: ok ? "#2ecc71" : "#e74c3c", fontWeight: 600, fontSize: 10 }}>
                  {ok ? "● OK" : "● FAIL"}
                </span>
              </div>
            ))}
            <div style={{ marginTop: 6, fontSize: 10, color: "#777", lineHeight: 1.4 }}>
              {telem.preflight_message || "—"}
            </div>
          </div>
        </div>

        {/* ── Col 3: Controls + Log ── */}
        <div style={{ display: "flex", flexDirection: "column", gap: 10 }}>
          {/* Flight Controls */}
          <div style={panel}>
            <div style={{ ...label, marginBottom: 8 }}>FLIGHT CONTROLS</div>

            <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 6 }}>
              <button
                onClick={doArm}
                disabled={!connected || armed}
                style={{
                  padding: "10px 0", border: "none", borderRadius: 4, cursor: "pointer",
                  background: !connected || armed ? "#1a1a1a" : "#1a5c2e",
                  color: !connected || armed ? "#444" : "#2ecc71",
                  fontWeight: 700, fontSize: 12, letterSpacing: 1,
                }}
              >
                ARM
              </button>
              <button
                onClick={doDisarm}
                disabled={!connected || !armed}
                style={{
                  padding: "10px 0", border: "none", borderRadius: 4, cursor: "pointer",
                  background: !connected || !armed ? "#1a1a1a" : "#5c1a1a",
                  color: !connected || !armed ? "#444" : "#e74c3c",
                  fontWeight: 700, fontSize: 12, letterSpacing: 1,
                }}
              >
                DISARM
              </button>
            </div>

            <div style={{ display: "flex", gap: 6, marginTop: 8 }}>
              <input
                type="number"
                value={takeoffAlt}
                onChange={(e) => setTakeoffAlt(parseFloat(e.target.value) || 2)}
                min={1} max={10} step={0.5}
                style={{
                  width: 60, background: "#111", border: "1px solid #222", color: "#ccc",
                  padding: "6px 8px", borderRadius: 3, fontSize: 12, fontFamily: "monospace",
                }}
              />
              <button
                onClick={doArmAndTakeoff}
                disabled={!connected || armed}
                style={{
                  flex: 1, padding: "8px 0", border: "none", borderRadius: 4, cursor: "pointer",
                  background: !connected || armed ? "#1a1a1a" : "#1a3c5c",
                  color: !connected || armed ? "#444" : "#3498db",
                  fontWeight: 700, fontSize: 11, letterSpacing: 1,
                }}
              >
                ARM + TAKEOFF {takeoffAlt}m
              </button>
            </div>

            {/* Takeoff only (if already armed) */}
            {armed && (
              <>
                <button
                  onClick={doTakeoff}
                  disabled={!connected}
                  style={{
                    width: "100%", marginTop: 6, padding: "7px 0", border: "none", borderRadius: 4,
                    cursor: "pointer", background: "#1a3c5c", color: "#3498db",
                    fontWeight: 600, fontSize: 11, letterSpacing: 1,
                  }}
                >
                  TAKEOFF {takeoffAlt}m (already armed)
                </button>

                <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 6, marginTop: 6 }}>
                  <button
                    onClick={doHover}
                    disabled={!connected}
                    style={{
                      padding: "7px 0", border: "none", borderRadius: 4, cursor: "pointer",
                      background: guidanceMode === "HOVER" ? "#1a5c44" : "#143126",
                      color: "#4ade80", fontWeight: 600, fontSize: 11, letterSpacing: 1,
                    }}
                  >
                    HOVER HOLD
                  </button>
                  <button
                    onClick={doLand}
                    disabled={!connected}
                    style={{
                      padding: "7px 0", border: "none", borderRadius: 4, cursor: "pointer",
                      background: guidanceMode === "LAND" ? "#5c3b1a" : "#352312",
                      color: "#f59e0b", fontWeight: 600, fontSize: 11, letterSpacing: 1,
                    }}
                  >
                    SOFT LAND
                  </button>
                </div>
              </>
            )}

            {/* Force Disarm */}
            <div style={{ marginTop: 8 }}>
              {!confirmForce ? (
                <button
                  onClick={() => setConfirmForce(true)}
                  disabled={!connected}
                  style={{
                    width: "100%", padding: "6px 0", border: "1px solid #5c1a1a", borderRadius: 4,
                    background: "transparent", color: "#e74c3c", fontSize: 10, cursor: "pointer",
                    letterSpacing: 1, opacity: connected ? 1 : 0.3,
                  }}
                >
                  FORCE DISARM (EMERGENCY)
                </button>
              ) : (
                <div style={{ display: "flex", gap: 6 }}>
                  <button
                    onClick={doForceDisarm}
                    style={{
                      flex: 1, padding: "6px 0", border: "none", borderRadius: 4,
                      background: "#e74c3c", color: "#fff", fontSize: 11, fontWeight: 700, cursor: "pointer",
                    }}
                  >
                    CONFIRM
                  </button>
                  <button
                    onClick={() => setConfirmForce(false)}
                    style={{
                      flex: 1, padding: "6px 0", border: "1px solid #333", borderRadius: 4,
                      background: "transparent", color: "#888", fontSize: 11, cursor: "pointer",
                    }}
                  >
                    CANCEL
                  </button>
                </div>
              )}
            </div>
          </div>

          {/* Mode Selection */}
          <div style={panel}>
            <div style={{ ...label, marginBottom: 6 }}>FLIGHT MODE: <span style={{ color: "#3498db" }}>{mode}</span></div>
            <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 4 }}>
              {["STABILIZE", "ALT_HOLD", "LOITER", "GUIDED", "POSHOLD"].map((m) => (
                <button
                  key={m}
                  onClick={() => doMode(m)}
                  disabled={!connected}
                  style={{
                    padding: "6px 0", border: "1px solid #222", borderRadius: 3,
                    background: mode === m ? "#1a3c5c" : "#111",
                    color: mode === m ? "#3498db" : "#666",
                    fontSize: 10, fontWeight: mode === m ? 700 : 400,
                    cursor: connected ? "pointer" : "default",
                    letterSpacing: 0.5,
                  }}
                >
                  {m}
                </button>
              ))}
            </div>
          </div>

          {/* Vision Info */}
          <div style={panel}>
            <div style={{ ...label, marginBottom: 6 }}>VISION</div>
            {[
              ["Method", telem.vision_method || "—"],
              ["Features", telem.vision_features != null ? telem.vision_features : "—"],
              ["FPS", fmt(telem.vision_fps, 0)],
              ["Depth", `${fmt(telem.rangefinder_dist, 2)}m`],
              ["Guide", guidanceMode],
              ["Target", `${fmt(telem.guidance_target_alt, 2)}m`],
            ].map(([k, v], i) => (
              <div key={i} style={{ ...row, fontSize: 11 }}>
                <span style={{ color: "#555" }}>{k}</span>
                <span style={{ fontFamily: "monospace" }}>{v}</span>
              </div>
            ))}
          </div>

          {/* Command Log */}
          <div style={{ ...panel, flex: 1, minHeight: 120, maxHeight: 200, overflow: "hidden", display: "flex", flexDirection: "column" }}>
            <div style={{ ...label, marginBottom: 4 }}>LOG</div>
            <div style={{ flex: 1, overflow: "auto", fontSize: 10, fontFamily: "monospace", lineHeight: 1.6 }}>
              {log.slice(-20).map((l, i) => (
                <div key={i} style={{ color: l.type === "err" ? "#e74c3c" : l.type === "ok" ? "#2ecc71" : "#666" }}>
                  <span style={{ color: "#333" }}>{new Date(l.t).toLocaleTimeString()} </span>
                  {l.msg}
                </div>
              ))}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
