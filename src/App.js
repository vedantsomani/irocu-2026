import { useState, useEffect, useRef } from "react";

/*
  ASCEND GCS — Live Telemetry Dashboard
  Connects to telemetry_server.py via WebSocket for real Pixhawk data.
  Falls back to simulated data if server is unavailable.

  To connect to real Pixhawk:
    1. Run telemetry_server.py on Jetson/Pi
    2. Set TELEMETRY_URL below to the server address
    3. Dashboard auto-connects and shows live data
*/

// ---- CHANGE THIS to your Jetson/Pi IP ----
const TELEMETRY_URL = "http://localhost:5000";
// e.g. "http://192.168.1.50:5000" if Jetson is on local network

const C = {
  bg:"#101214",surface:"#16181c",border:"#1f2328",borderLight:"#2a2f36",
  text:"#e6e8eb",dim:"#6b7280",muted:"#3b4048",
  accent:"#10b981",accentMuted:"#10b98130",
  warn:"#eab308",warnMuted:"#eab30825",
  crit:"#dc2626",critMuted:"#dc262625",
  blue:"#60a5fa",
};

// =============================================================================
// DATA HOOK — WebSocket with HTTP polling fallback
// =============================================================================
function useTelemetry() {
  const [data, setData] = useState(null);
  const [source, setSource] = useState("connecting");
  const trailRef = useRef([]);
  const altRef = useRef([]);
  const vzRef = useRef([]);
  const curRef = useRef([]);
  const simRef = useRef(0);

  // Try WebSocket first, fall back to HTTP polling, then to simulation
  useEffect(() => {
    let ws = null;
    let pollInterval = null;
    let simInterval = null;
    let destroyed = false;

    function processData(d) {
      // Append to history arrays
      const alt = d.rangefinder_dist > 0 ? d.rangefinder_dist : (d.alt_rel !== undefined ? d.alt_rel : -(d.pos_z || 0));
      trailRef.current = [...trailRef.current.slice(-400), {x: d.pos_x||0, y: d.pos_y||0}];
      altRef.current = [...altRef.current.slice(-120), alt];
      vzRef.current = [...vzRef.current.slice(-120), d.vz||0];
      curRef.current = [...curRef.current.slice(-120), d.bat_current||0];

      setData({
        ...d,
        alt_rel: alt,
        _trail: trailRef.current,
        _altH: altRef.current,
        _vzH: vzRef.current,
        _curH: curRef.current,
      });
    }

    // Attempt 1: WebSocket via socket.io
    function tryWebSocket() {
      // Simple polling since we can't import socket.io in artifact
      // Use HTTP polling instead
      tryHTTP();
    }

    // Attempt 2: HTTP polling
    function tryHTTP() {
      let failures = 0;
      pollInterval = setInterval(async () => {
        if (destroyed) return;
        try {
          const res = await fetch(`${TELEMETRY_URL}/api/telemetry`);
          if (res.ok) {
            const d = await res.json();
            if (!destroyed) {
              processData(d);
              setSource("live");
              failures = 0;
            }
          }
        } catch {
          failures++;
          if (failures > 5 && !simInterval) {
            setSource("simulated");
            startSimulation();
          }
        }
      }, 100); // 10Hz polling
    }

    // Attempt 3: Local simulation
    function startSimulation() {
      if (pollInterval) { clearInterval(pollInterval); pollInterval = null; }
      simInterval = setInterval(() => {
        if (destroyed) return;
        simRef.current += 0.1;
        const t = simRef.current;
        const phase = t<4?0:t<11?1:t<28?2:t<36?3:t<43?4:5;
        const modes = ["STABILIZE","AUTO","LOITER","LOITER","AUTO","LAND"];
        const alt = [0.08,Math.min((t-4)*0.72,5),5+Math.sin(t*0.4)*0.08,5+Math.sin(t*0.6)*0.12,Math.max(5-(t-36)*0.75,0.05),0.05][phase];
        const d = {
          pos_x:Math.sin(t*0.15)*0.25,pos_y:Math.cos(t*0.12)*0.25,pos_z:-alt,
          alt_rel:alt,vx:Math.sin(t*0.2)*0.08,vy:Math.cos(t*0.18)*0.06,
          vz:[0,0.72,Math.sin(t*0.5)*0.03,Math.sin(t*0.5)*0.03,-0.75,0][phase],
          roll:Math.sin(t*0.7)*3.5,pitch:Math.cos(t*0.6)*2.8,yaw:(t*4.5)%360,
          imu_ax:Math.sin(t*2)*0.25,imu_ay:Math.cos(t*1.8)*0.2,imu_az:-9.79+Math.sin(t*3)*0.1,
          imu_gx:Math.sin(t*1.5)*1.5,imu_gy:Math.cos(t*1.3)*1.2,imu_gz:Math.sin(t*0.8)*0.8,
          imu_temp:41+Math.sin(t*0.1)*2,
          bat_voltage:12.58-t*0.012,bat_current:[3,16,10,10,8,2][phase]+0.5,
          bat_soc:Math.max(98-t*0.55,12),bat_temp:34+t*0.12,
          cell_voltages:[4.18-t*0.004,4.16-t*0.004,4.17-t*0.004],
          flow_quality:phase>0&&phase<5?80+Math.random()*15:0,
          rangefinder_dist:alt<8?alt:0,
          heading:(t*4.5)%360,groundspeed:0.1,
          throttle:[0,65,45,45,55,0][phase],climb:[0,0.72,0,0,-0.75,0][phase],
          armed:phase>0&&phase<5,mode:modes[phase],
          ekf_ok:true,
          vibe_x:15+5*Math.sin(t),vibe_y:14,vibe_z:20+6*Math.sin(t*1.3),
          last_heartbeat:Date.now()/1000,
        };
        processData(d);
      }, 100);
    }

    tryWebSocket();

    return () => {
      destroyed = true;
      if (ws) ws.close();
      if (pollInterval) clearInterval(pollInterval);
      if (simInterval) clearInterval(simInterval);
    };
  }, []);

  return { data, source };
}

// =============================================================================
// UI COMPONENTS
// =============================================================================
function Num({v,d=2,u,size=13,color=C.text,weight=600}){
  return(<span><span style={{fontSize:size,fontWeight:weight,color,fontVariantNumeric:"tabular-nums"}}>{typeof v==="number"?v.toFixed(d):v}</span>{u&&<span style={{fontSize:9,color:C.dim,marginLeft:3,fontWeight:400}}>{u}</span>}</span>);
}
function Row({label,children,border=true}){
  return(<div style={{display:"flex",justifyContent:"space-between",alignItems:"baseline",padding:"5px 0",borderBottom:border?`1px solid ${C.border}`:"none"}}><span style={{fontSize:10,color:C.dim,textTransform:"uppercase",letterSpacing:"0.5px"}}>{label}</span><div>{children}</div></div>);
}
function SectionLabel({children}){
  return(<div style={{fontSize:9,fontWeight:700,color:C.muted,textTransform:"uppercase",letterSpacing:"1.5px",padding:"8px 0 4px",borderBottom:`1px solid ${C.border}`,marginBottom:2}}>{children}</div>);
}
function Spark({data,w=140,h=28,color=C.accent,base}){
  const ref=useRef(null);
  useEffect(()=>{
    const c=ref.current;if(!c||data.length<2)return;
    const ctx=c.getContext("2d");ctx.clearRect(0,0,c.width,c.height);
    const mn=base!==undefined?base:Math.min(...data);const mx=Math.max(...data);const range=mx-mn||1;
    ctx.beginPath();
    data.forEach((v,i)=>{const x=(i/(data.length-1))*c.width;const y=c.height-((v-mn)/range)*(c.height-4)-2;i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);});
    ctx.strokeStyle=color;ctx.lineWidth=1;ctx.stroke();
  },[data,color,w,h,base]);
  return <canvas ref={ref} width={w*2} height={h*2} style={{width:w,height:h,display:"block",opacity:0.8}} />;
}
function Indicator({label,ok}){
  return(<div style={{display:"flex",alignItems:"center",gap:5}}><div style={{width:5,height:5,borderRadius:1,background:ok?C.accent:C.crit}}/><span style={{fontSize:9,color:ok?C.dim:C.crit,letterSpacing:"0.5px"}}>{label}</span></div>);
}

function AttCanvas({roll,pitch,size=88}){
  const ref=useRef(null);
  useEffect(()=>{
    const c=ref.current;if(!c)return;const ctx=c.getContext("2d");
    const s=c.width,half=s/2;ctx.clearRect(0,0,s,s);
    ctx.save();ctx.beginPath();ctx.arc(half,half,half-1,0,Math.PI*2);ctx.clip();
    ctx.save();ctx.translate(half,half);ctx.rotate(roll*Math.PI/180);
    const po=(pitch/40)*half;
    ctx.fillStyle="#1a2a3a";ctx.fillRect(-s,-s+po,s*2,s);
    ctx.fillStyle="#3a2a1a";ctx.fillRect(-s,po,s*2,s);
    ctx.strokeStyle="#fff";ctx.lineWidth=1;ctx.globalAlpha=0.6;
    ctx.beginPath();ctx.moveTo(-s,po);ctx.lineTo(s,po);ctx.stroke();
    ctx.globalAlpha=1;ctx.restore();
    ctx.strokeStyle=C.accent;ctx.lineWidth=1.5;ctx.beginPath();
    ctx.moveTo(half-20,half);ctx.lineTo(half-6,half);ctx.moveTo(half+6,half);ctx.lineTo(half+20,half);
    ctx.moveTo(half,half+2);ctx.lineTo(half,half+7);ctx.stroke();ctx.restore();
    ctx.strokeStyle=C.border;ctx.lineWidth=1.5;ctx.beginPath();ctx.arc(half,half,half-1,0,Math.PI*2);ctx.stroke();
  },[roll,pitch,size]);
  return <canvas ref={ref} width={size*2} height={size*2} style={{width:size,height:size}} />;
}

function MapCanvas({trail,objects=[],size=200}){
  const ref=useRef(null);
  useEffect(()=>{
    const c=ref.current;if(!c)return;const ctx=c.getContext("2d");
    const s=c.width,half=s/2,ppm=50;
    ctx.clearRect(0,0,s,s);ctx.fillStyle=C.bg;ctx.fillRect(0,0,s,s);
    ctx.strokeStyle=C.border;ctx.lineWidth=0.5;
    for(let i=-6;i<=6;i++){const p=half+i*ppm;ctx.beginPath();ctx.moveTo(p,0);ctx.lineTo(p,s);ctx.stroke();ctx.beginPath();ctx.moveTo(0,p);ctx.lineTo(s,p);ctx.stroke();}
    ctx.strokeStyle=C.borderLight;ctx.lineWidth=0.5;ctx.beginPath();ctx.moveTo(half,0);ctx.lineTo(half,s);ctx.stroke();ctx.beginPath();ctx.moveTo(0,half);ctx.lineTo(s,half);ctx.stroke();
    if(trail.length>1){for(let i=1;i<trail.length;i++){const a=Math.min(i/trail.length,1);ctx.strokeStyle=`rgba(16,185,129,${a*0.5})`;ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(half+trail[i-1].y*ppm,half-trail[i-1].x*ppm);ctx.lineTo(half+trail[i].y*ppm,half-trail[i].x*ppm);ctx.stroke();}}
    objects.forEach(o=>{if(!o)return;const ox=half+(o.y||0)*ppm,oy=half-(o.x||0)*ppm;ctx.strokeStyle=C.warn;ctx.lineWidth=1;ctx.strokeRect(ox-6,oy-6,12,12);ctx.fillStyle=C.warn;ctx.font="bold 8px system-ui";ctx.textAlign="center";ctx.fillText(o.id||"?",ox,oy-10);});
    if(trail.length>0){const l=trail[trail.length-1];const px=half+l.y*ppm,py=half-l.x*ppm;ctx.fillStyle=C.accent;ctx.beginPath();ctx.arc(px,py,3,0,Math.PI*2);ctx.fill();}
    ctx.fillStyle=C.crit;ctx.fillRect(half-1.5,half-1.5,3,3);
    ctx.fillStyle=C.muted;ctx.font="8px system-ui";ctx.textAlign="center";ctx.fillText("N",half,10);
  },[trail,objects,size]);
  return <canvas ref={ref} width={size*2} height={size*2} style={{width:"100%",aspectRatio:"1",display:"block",border:`1px solid ${C.border}`}} />;
}

function CellBar({label,v}){
  const pct=Math.max(0,Math.min(100,((v-3.0)/1.2)*100));
  const col=v<3.3?C.crit:v<3.6?C.warn:C.dim;
  return(<div style={{flex:1}}><div style={{fontSize:8,color:C.muted,marginBottom:2}}>{label}</div><div style={{height:3,background:C.border,borderRadius:1,overflow:"hidden",marginBottom:2}}><div style={{width:`${pct}%`,height:"100%",background:col,borderRadius:1}}/></div><div style={{fontSize:10,fontWeight:600,color:C.text,fontVariantNumeric:"tabular-nums"}}>{v.toFixed(3)}</div></div>);
}

// =============================================================================
// DASHBOARD
// =============================================================================
export default function Dashboard(){
  const {data:d, source} = useTelemetry();
  if(!d) return <div style={{background:C.bg,height:"100vh",display:"flex",alignItems:"center",justifyContent:"center",color:C.dim,fontFamily:"monospace"}}>Connecting to telemetry...</div>;

  const handleArm = async () => {
    try { await fetch(`${TELEMETRY_URL}/api/arm`, { method: "POST" }); } catch (e) {}
  };

  const handleTakeoff = async () => {
    try { await fetch(`${TELEMETRY_URL}/api/takeoff`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ altitude: 4.0 }) }); } catch (e) {}
  };

  const handleDisarm = async () => {
    try { await fetch(`${TELEMETRY_URL}/api/disarm`, { method: "POST" }); } catch (e) {}
  };

  const alt = d.rangefinder_dist > 0 ? d.rangefinder_dist : (d.alt_rel || 0);
  const socColor = (d.bat_soc||0)>50?C.accent:(d.bat_soc||0)>25?C.warn:C.crit;
  const fq = d.flow_quality||0;
  const fqColor = fq>80?C.accent:fq>50?C.warn:fq>0?C.crit:C.muted;
  const modeColor = d.armed?C.accent:C.dim;
  const cells = d.cell_voltages||[];
  const cameraUrl = `${TELEMETRY_URL}/feed/color`;

  return(
    <div style={{width:"100%",minHeight:"100vh",background:C.bg,color:C.text,fontFamily:"'IBM Plex Mono','SF Mono','Consolas',monospace",fontSize:12,lineHeight:1.4}}>
      <link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@300;400;500;600;700&family=IBM+Plex+Sans:wght@400;500;600;700&display=swap" rel="stylesheet"/>

      {/* HEADER */}
      <div style={{display:"flex",alignItems:"center",justifyContent:"space-between",padding:"6px 16px",borderBottom:`1px solid ${C.border}`,background:C.surface}}>
        <div style={{display:"flex",alignItems:"center",gap:16}}>
          <span style={{fontFamily:"'IBM Plex Sans',sans-serif",fontSize:13,fontWeight:700,letterSpacing:3,color:C.text}}>ASCEND</span>
          <span style={{height:12,width:1,background:C.border}}/>
          <span style={{fontSize:9,color:C.dim,letterSpacing:1}}>TEAM SAARABAI X · IRoC-U 2026</span>
        </div>
        <div style={{display:"flex",alignItems:"center",gap:14}}>
          <span style={{fontSize:8,padding:"2px 6px",background:source==="live"?C.accentMuted:C.warnMuted,color:source==="live"?C.accent:C.warn,fontWeight:600,letterSpacing:1}}>
            {source==="live"?"LIVE TELEMETRY":source==="simulated"?"SIMULATED":"CONNECTING"}
          </span>
          <span style={{fontSize:8,padding:"2px 6px",background:C.warnMuted,color:C.warn,fontWeight:600,letterSpacing:1}}>GNSS DENIED</span>
          <span style={{fontSize:8,padding:"2px 6px",background:`${modeColor}20`,color:modeColor,fontWeight:600,letterSpacing:1}}>{d.mode||"—"}</span>
          <button onClick={handleArm} style={{cursor:"pointer",border:"none",fontSize:8,padding:"4px 8px",background:C.accentMuted,color:C.accent,fontWeight:600,letterSpacing:1}}>ARM</button>
          <button onClick={handleTakeoff} style={{cursor:"pointer",border:"none",fontSize:8,padding:"4px 8px",background:C.blue+"30",color:C.blue,fontWeight:600,letterSpacing:1}}>TAKEOFF (4m)</button>
          <button onClick={handleDisarm} style={{cursor:"pointer",border:"none",fontSize:8,padding:"4px 8px",background:C.critMuted,color:C.crit,fontWeight:600,letterSpacing:1}}>DISARM</button>
          <span style={{fontSize:8,padding:"2px 6px",background:d.armed?C.accentMuted:C.critMuted,color:d.armed?C.accent:C.crit,fontWeight:600,letterSpacing:1}}>{d.armed?"✓ ARMED":"X DISARMED"}</span>
        </div>
      </div>

      {/* BODY */}
      <div style={{display:"grid",gridTemplateColumns:"220px 1fr 1fr 200px",minHeight:"calc(100vh - 65px)"}}>

        {/* COL 1: NAV */}
        <div style={{borderRight:`1px solid ${C.border}`,padding:"0 12px 12px"}}>
          <SectionLabel>Position</SectionLabel>
          <Row label="X north"><Num v={d.pos_x||0} d={3} u="m"/></Row>
          <Row label="Y east"><Num v={d.pos_y||0} d={3} u="m"/></Row>
          <Row label="Z height" border={false}><Num v={alt} d={3} u="m" color={C.accent} size={15} weight={700}/></Row>

          <SectionLabel>Velocity</SectionLabel>
          <Row label="Vx"><Num v={d.vx||0} d={3} u="m/s"/></Row>
          <Row label="Vy"><Num v={d.vy||0} d={3} u="m/s"/></Row>
          <Row label="Vz"><Num v={d.vz||0} d={3} u="m/s" color={(d.vz||0)>0.1?C.accent:(d.vz||0)<-0.1?C.warn:C.text}/></Row>
          <Row label="H. speed" border={false}><Num v={Math.sqrt((d.vx||0)**2+(d.vy||0)**2)} d={3} u="m/s"/></Row>

          <SectionLabel>Heading</SectionLabel>
          <Row label="HDG"><Num v={d.heading||d.yaw||0} d={1} u="°"/></Row>
          <Row label="Throttle" border={false}><Num v={d.throttle||0} d={0} u="%"/></Row>

          <SectionLabel>Optical Flow</SectionLabel>
          <Row label="Quality"><Num v={fq} d={0} u="%" color={fqColor}/></Row>
          <Row label="Rangefinder" border={false}><Num v={d.rangefinder_dist||0} d={2} u="m"/></Row>
        </div>

        {/* COL 2: ATTITUDE + MAP */}
        <div style={{borderRight:`1px solid ${C.border}`,padding:"0 12px 12px"}}>
          <SectionLabel>Attitude</SectionLabel>
          <div style={{display:"flex",gap:16,justifyContent:"center",padding:"8px 0"}}>
            <div style={{textAlign:"center"}}>
              <AttCanvas roll={d.roll||0} pitch={d.pitch||0} size={88}/>
              <div style={{fontSize:9,color:C.dim,marginTop:4}}>R <span style={{color:C.text}}>{(d.roll||0).toFixed(1)}°</span> P <span style={{color:C.text}}>{(d.pitch||0).toFixed(1)}°</span></div>
            </div>
          </div>

          <SectionLabel>Position Map</SectionLabel>
          <MapCanvas trail={d._trail||[]} objects={d.identified_objects||[]} size={220}/>

          <SectionLabel>Camera Feed</SectionLabel>
          <img
            alt="Latest D435i diagnostic frame"
            src={cameraUrl}
            style={{width:"100%",display:"block",border:`1px solid ${C.border}`,background:C.bg,aspectRatio:"4 / 3",objectFit:"cover"}}
            onError={(e) => { e.target.style.display = 'none'; e.target.nextSibling.style.display = 'block'; }}
          />
          <div style={{display: "none", border:`1px solid ${C.border}`,padding:"12px",color:C.dim,fontSize:10,letterSpacing:"0.4px"}}>
            No live camera frame available from the backend.
          </div>
          <Row label="Camera source"><span style={{fontSize:10,color:C.text}}>{d.camera_source||"unavailable"}</span></Row>
          <Row label="Objects tracked" border={false}><Num v={(d.identified_objects||[]).length} d={0}/></Row>

          <SectionLabel>Vibration</SectionLabel>
          <Row label="X"><Num v={d.vibe_x||0} d={1} u="m/s²" color={(d.vibe_x||0)>30?C.crit:(d.vibe_x||0)>15?C.warn:C.text}/></Row>
          <Row label="Y"><Num v={d.vibe_y||0} d={1} u="m/s²" color={(d.vibe_y||0)>30?C.crit:(d.vibe_y||0)>15?C.warn:C.text}/></Row>
          <Row label="Z" border={false}><Num v={d.vibe_z||0} d={1} u="m/s²" color={(d.vibe_z||0)>30?C.crit:(d.vibe_z||0)>15?C.warn:C.text}/></Row>
        </div>

        {/* COL 3: CHARTS + IMU */}
        <div style={{borderRight:`1px solid ${C.border}`,padding:"0 12px 12px"}}>
          <SectionLabel>Altitude Profile</SectionLabel>
          <div style={{padding:"4px 0 2px"}}>
            <Spark data={d._altH||[]} h={50} w={260} color={C.accent} base={0}/>
            <div style={{display:"flex",justifyContent:"space-between",paddingTop:2}}><span style={{fontSize:8,color:C.muted}}>0m</span><span style={{fontSize:8,color:C.muted}}>7m</span></div>
          </div>

          <SectionLabel>Vertical Velocity</SectionLabel>
          <Spark data={d._vzH||[]} h={32} w={260} color={C.blue}/>

          <SectionLabel>IMU — Accelerometer</SectionLabel>
          <Row label="Ax"><Num v={d.imu_ax||0} d={3} u="m/s²"/></Row>
          <Row label="Ay"><Num v={d.imu_ay||0} d={3} u="m/s²"/></Row>
          <Row label="Az" border={false}><Num v={d.imu_az||0} d={3} u="m/s²"/></Row>

          <SectionLabel>IMU — Gyroscope</SectionLabel>
          <Row label="Gx"><Num v={d.imu_gx||0} d={2} u="°/s"/></Row>
          <Row label="Gy"><Num v={d.imu_gy||0} d={2} u="°/s"/></Row>
          <Row label="Gz" border={false}><Num v={d.imu_gz||0} d={2} u="°/s"/></Row>

          <SectionLabel>IMU — Thermal</SectionLabel>
          <Row label="Temperature" border={false}><Num v={d.imu_temp||0} d={1} u="°C" color={(d.imu_temp||0)>50?C.warn:C.text}/></Row>
        </div>

        {/* COL 4: BATTERY */}
        <div style={{padding:"0 12px 12px"}}>
          <SectionLabel>Battery</SectionLabel>
          <div style={{margin:"6px 0 8px"}}>
            <div style={{display:"flex",justifyContent:"space-between",marginBottom:3}}>
              <span style={{fontSize:9,color:C.dim}}>STATE OF CHARGE</span>
              <span style={{fontSize:12,fontWeight:700,color:socColor,fontVariantNumeric:"tabular-nums"}}>{(d.bat_soc||0).toFixed(1)}%</span>
            </div>
            <div style={{height:4,background:C.border,overflow:"hidden"}}><div style={{width:`${d.bat_soc||0}%`,height:"100%",background:socColor,transition:"width 0.5s"}}/></div>
          </div>
          <Row label="Voltage"><Num v={d.bat_voltage||0} d={2} u="V" color={(d.bat_voltage||0)<10.5?C.crit:C.text}/></Row>
          <Row label="Current"><Num v={d.bat_current||0} d={1} u="A"/></Row>
          <Row label="Pack temp" border={false}><Num v={d.bat_temp||0} d={1} u="°C" color={(d.bat_temp||0)>50?C.crit:C.text}/></Row>

          {cells.length>0 && <>
            <SectionLabel>Cell Voltages</SectionLabel>
            <div style={{display:"flex",gap:6,padding:"4px 0 0"}}>
              {cells.map((v,i)=><CellBar key={i} label={`C${i+1}`} v={v}/>)}
            </div>
          </>}

          <SectionLabel>Current Draw</SectionLabel>
          <Spark data={d._curH||[]} h={28} w={170} color={C.warn} base={0}/>

          <SectionLabel>System Status</SectionLabel>
          <div style={{display:"flex",flexDirection:"column",gap:4,paddingTop:4}}>
            <Indicator label="EKF3" ok={d.ekf_ok}/>
            <Indicator label="VISO" ok={d.flow_quality>0}/>
            <Indicator label="RC LINK" ok={(d.last_heartbeat||0)>Date.now()/1000-3}/>
            <Indicator label="D435i" ok={d.d435i_ok}/>
            <Indicator label="ARMED" ok={d.armed}/>
            <Indicator label="PIXHAWK" ok={d.pixhawk_ok}/>
          </div>
        </div>
      </div>

      {/* FOOTER */}
      <div style={{display:"flex",justifyContent:"space-between",alignItems:"center",padding:"4px 16px",borderTop:`1px solid ${C.border}`,background:C.surface,fontSize:9,color:C.muted}}>
        <span>PIXHAWK 2.4.8 · ARDUCOPTER 4.6.3 · F450 · 5010 750KV · D435i VIO</span>
        <span>BENNETT UNIVERSITY · TECHNOTIX · BC3</span>
      </div>
    </div>
  );
}
