#pragma once
#include <Arduino.h>
#include "config.h"   // CAM_HOST / CAM_STREAM_PORT for the camera page

// =====================================================================
//  WebUI.h — all front-end markup, styling and client JS.
//
//  Each subsystem is served as its OWN page (its own URL) so a phone
//  screen only ever shows one test at a time — no clutter. A shared
//  shell (nav + CSS + base WebSocket client) wraps every page.
//
//  Pages: /  /motors  /drive  /encoders  /imu  /camera   (range is planned)
//  The camera page embeds MJPEG straight from the separate ESP32-CAM board
//  (CamStreamer sketch) at CAM_HOST — video never passes through this ESP32.
//
//  Client/telemetry contract (JSON pushed on /ws at 10 Hz):
//    { up, estop, m:[pwm x4], a:[armed x4], inv:[inverted x4],
//      enc:[{c,r} x4], imu:{en,ok,h,r,p,cs,cg,ca,cm} }
//  Motor/encoder arrays share the index order LF, LR, RF, RR.
//  Every page defines window.onTelemetry(d) to consume it.
// =====================================================================

namespace web {

// ---------------------------------------------------------------- styles
inline const char* STYLE() { return R"css(
:root{--bg:#0f1115;--panel:#1a1d24;--panel2:#232733;--line:#2e3340;
--text:#e6e8ee;--muted:#8b91a0;--accent:#4d8dff;--good:#28c98b;
--warn:#f5b342;--bad:#ff5c6c;--radius:12px}
*{box-sizing:border-box}
body{margin:0;font-family:'Segoe UI',system-ui,Roboto,sans-serif;
background:var(--bg);color:var(--text);-webkit-tap-highlight-color:transparent}
nav{display:flex;gap:4px;align-items:center;padding:10px 14px;background:var(--panel);
border-bottom:1px solid var(--line);position:sticky;top:0;z-index:10;flex-wrap:wrap}
nav .brand{font-weight:700;margin-right:12px;font-size:16px}
nav a{color:var(--muted);text-decoration:none;padding:8px 12px;border-radius:8px;font-size:14px;font-weight:600}
nav a:hover{color:var(--text);background:var(--panel2)}
nav a.active{color:#fff;background:var(--accent)}
nav .soon{color:#5a606e;padding:8px 10px;font-size:13px}
nav .soon b{font-size:9px;background:var(--panel2);color:var(--muted);
padding:1px 5px;border-radius:6px;margin-left:4px;vertical-align:middle}
#conn{margin-left:auto;font-size:12px;font-weight:600;padding:5px 10px;border-radius:20px;
background:var(--panel2);color:var(--bad)}
#conn.ok{color:var(--good)}
main{max-width:880px;margin:0 auto;padding:18px 14px 48px}
h1{font-size:22px;margin:6px 0 2px}
.sub{color:var(--muted);font-size:13px;margin:0 0 18px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:14px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:var(--radius);padding:16px}
.card h2{margin:0 0 12px;font-size:15px;color:var(--text);display:flex;justify-content:space-between;align-items:center}
.tag{font-size:11px;font-weight:700;padding:3px 8px;border-radius:20px;background:var(--panel2);color:var(--muted)}
.tag.good{color:var(--good)} .tag.bad{color:var(--bad)} .tag.warn{color:var(--warn)}
.metric{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:30px;font-weight:700}
.metric.good{color:var(--good)} .metric.accent{color:var(--accent)}
.unit{font-size:13px;color:var(--muted);font-weight:500}
.row{display:flex;justify-content:space-between;align-items:center;padding:7px 0;border-bottom:1px solid var(--line)}
.row:last-child{border-bottom:none}
.row .k{color:var(--muted);font-size:14px}
button{font-family:inherit;font-weight:600;font-size:14px;border:none;border-radius:9px;
padding:11px 14px;cursor:pointer;background:var(--panel2);color:var(--text)}
button:active{transform:translateY(1px)}
button.pri{background:var(--accent);color:#fff}
button.good{background:var(--good);color:#06231a}
button.bad{background:var(--bad);color:#2a0608}
button.warn{background:var(--warn);color:#3a2a05}
button.wide{width:100%}
.btns{display:flex;gap:8px;flex-wrap:wrap}
.estop{width:100%;padding:18px;font-size:18px;font-weight:800;letter-spacing:.5px;
background:var(--bad);color:#2a0608;border-radius:var(--radius);margin-bottom:16px}
input[type=range]{width:100%;accent-color:var(--accent);height:30px}
input[type=number]{width:100%;background:var(--panel2);border:1px solid var(--line);
color:var(--text);padding:10px;border-radius:8px;font-size:15px;font-family:ui-monospace,monospace}
label{display:block;font-size:13px;color:var(--muted);margin:10px 0 4px}
table{width:100%;border-collapse:collapse;font-size:14px}
th,td{text-align:left;padding:9px 6px;border-bottom:1px solid var(--line)}
th{color:var(--muted);font-weight:600;font-size:12px;text-transform:uppercase;letter-spacing:.4px}
td.num{font-family:ui-monospace,monospace;text-align:right}
.hint{color:var(--muted);font-size:13px;line-height:1.5}
.step{background:var(--panel2);border-left:3px solid var(--accent);padding:12px 14px;border-radius:0 8px 8px 0;margin:10px 0}
.big{font-family:ui-monospace,monospace;font-size:40px;text-align:center;color:var(--accent);font-weight:700;margin:8px 0}
.bar{height:8px;background:var(--panel2);border-radius:6px;overflow:hidden;margin-top:5px}
.bar > span{display:block;height:100%;background:var(--good);width:0}
.dpad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:340px;margin:0 auto}
.dpad button{padding:24px 0;font-size:20px;user-select:none;-webkit-user-select:none;touch-action:none}
.dpad button.hold{background:var(--accent);color:#fff}
)css"; }

// ---------------------------------------------------- shared client base
inline const char* BASE_JS() { return R"js(
let ws;
function connect(){
  ws=new WebSocket('ws://'+location.host+'/ws');
  ws.onopen =()=>{const c=document.getElementById('conn');c.className='ok';c.textContent='connected';};
  ws.onclose=()=>{const c=document.getElementById('conn');c.className='';c.textContent='offline';setTimeout(connect,1500);};
  ws.onmessage=e=>{try{const d=JSON.parse(e.data);if(window.onTelemetry)window.onTelemetry(d);}catch(_){}};
}
function post(url){return fetch(url,{method:'POST'});}
function postJSON(url,obj){return fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(obj)});}
window.addEventListener('load',connect);
)js"; }

// ----------------------------------------------------------- page shell
inline String nav(const char* active) {
    auto link = [&](const char* href, const char* key, const char* label) {
        String s = "<a href='"; s += href; s += "'";
        if (strcmp(key, active) == 0) s += " class='active'";
        s += ">"; s += label; s += "</a>";
        return s;
    };
    String n = "<nav><span class='brand'>RC Car Test Bench</span>";
    n += link("/", "home", "Overview");
    n += link("/motors", "motors", "Motors");
    n += link("/drive", "drive", "Drive");
    n += link("/encoders", "encoders", "Encoders");
    n += link("/imu", "imu", "IMU");
    n += link("/camera", "camera", "Camera");
    n += "<span class='soon'>Range<b>SOON</b></span>";
    n += "<span id='conn'>offline</span></nav>";
    return n;
}

inline String page(const char* title, const char* active, const char* body, const char* script) {
    String h; h.reserve(strlen(body) + strlen(script) + 4096);
    h += "<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'>";
    h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    h += "<title>"; h += title; h += " · RC Car Test Bench</title><style>";
    h += STYLE();
    h += "</style></head><body>";
    h += nav(active);
    h += "<main>"; h += body; h += "</main>";
    h += "<script>"; h += BASE_JS(); h += script; h += "</script></body></html>";
    return h;
}

// ============================================================ OVERVIEW
inline const char* HOME_BODY() { return R"html(
<h1>System Overview</h1>
<p class='sub'>Connect your phone to the RC Car Test Bench WiFi, then pick a subsystem to test.</p>
<div class='grid'>
  <div class='card'><h2>Safety <span id='tEstop' class='tag good'>ARMED</span></h2>
    <button class='estop' onclick="post('/api/estop')">EMERGENCY STOP</button>
    <button class='wide good' onclick="post('/api/estop/clear')">Clear E-Stop</button>
  </div>
  <div class='card'><h2>Motors</h2>
    <div class='row'><span class='k'>Left-Front</span><span class='metric accent' id='hM0'>0</span></div>
    <div class='row'><span class='k'>Left-Rear</span><span class='metric accent' id='hM1'>0</span></div>
    <div class='row'><span class='k'>Right-Front</span><span class='metric accent' id='hM2'>0</span></div>
    <div class='row'><span class='k'>Right-Rear</span><span class='metric accent' id='hM3'>0</span></div>
    <a href='/motors'><button class='wide pri' style='margin-top:12px'>Open motor test &rarr;</button></a>
  </div>
  <div class='card'><h2>Drive</h2>
    <p class='hint'>Whole-car forward / backward / spin-left / spin-right test.</p>
    <a href='/drive'><button class='wide pri' style='margin-top:12px'>Open drive test &rarr;</button></a>
  </div>
  <div class='card'><h2>Encoders</h2>
    <div class='row'><span class='k'>Wheels reporting</span><span class='metric good' id='hEnc'>0/4</span></div>
    <a href='/encoders'><button class='wide pri' style='margin-top:12px'>Open encoder test &rarr;</button></a>
  </div>
  <div class='card'><h2>Camera</h2>
    <p class='hint'>Live MJPEG feed from the ESP32-CAM board.</p>
    <a href='/camera'><button class='wide pri' style='margin-top:12px'>Open camera &rarr;</button></a>
  </div>
  <div class='card'><h2>IMU (BNO055) <span id='tImu' class='tag'>--</span></h2>
    <div class='row'><span class='k'>Heading</span><span class='metric' id='hHead'>--</span></div>
    <a href='/imu'><button class='wide pri' style='margin-top:12px'>Open IMU test &rarr;</button></a>
  </div>
  <div class='card'><h2>Uptime</h2>
    <span class='metric' id='hUp'>0</span> <span class='unit'>seconds</span>
  </div>
</div>
)html"; }

inline const char* HOME_JS() { return R"js(
window.onTelemetry=d=>{
  d.m.forEach((v,i)=>document.getElementById('hM'+i).textContent=v);
  document.getElementById('hUp').textContent=d.up;
  const es=document.getElementById('tEstop');
  es.textContent=d.estop?'E-STOPPED':'ARMED';es.className='tag '+(d.estop?'bad':'good');
  let live=d.enc.filter(e=>Math.abs(e.r)>0.1).length;
  document.getElementById('hEnc').textContent=live+'/'+d.enc.length;
  const ti=document.getElementById('tImu'),hh=document.getElementById('hHead');
  if(!d.imu.en){ti.textContent='disabled';ti.className='tag';hh.textContent='--';}
  else if(!d.imu.ok){ti.textContent='no sensor';ti.className='tag warn';hh.textContent='--';}
  else{ti.textContent='live';ti.className='tag good';hh.textContent=d.imu.h.toFixed(0)+'°';}
};
)js"; }

// ============================================================== MOTORS
inline const char* MOTORS_BODY() { return R"html(
<h1>Motor Test</h1>
<p class='sub'>4WD with one TB6612FNG channel per wheel — each slider drives one motor independently.
A wheel does nothing until you <b>ARM</b> it; E-STOP and Disarm cut the drivers.
&#9888; TB6612 = 1.2&nbsp;A/channel: ramp up gently and never hold a stalled wheel.</p>
<button class='estop' onclick="estop()">EMERGENCY STOP</button>
<div class='card' style='margin-bottom:14px'><h2>All wheels</h2>
  <div class='btns'>
    <button class='good' onclick="armAll(1)">ARM ALL</button>
    <button class='bad' onclick="armAll(0)">Disarm all</button>
    <button onclick="modeAll('coast')">All coast</button>
    <button onclick="modeAll('brake')">All safe stop</button>
  </div>
</div>
<div class='grid' id='cards'></div>
<button class='wide good' style='margin-top:14px' onclick="post('/api/estop/clear')">Clear E-Stop</button>
)html"; }

inline const char* MOTORS_JS() { return R"js(
const M=[{k:'lf',n:'Left-Front'},{k:'lr',n:'Left-Rear'},{k:'rf',n:'Right-Front'},{k:'rr',n:'Right-Rear'}];
let armed={lf:false,lr:false,rf:false,rr:false};
let inv={lf:false,lr:false,rf:false,rr:false};
const cards=document.getElementById('cards');
M.forEach(({k,n})=>{
  cards.insertAdjacentHTML('beforeend',
    "<div class='card'><h2>"+n+" <span class='tag bad' id='"+k+"Arm'>DISARMED</span></h2>"+
    "<button class='wide' id='"+k+"ArmBtn' onclick=\"toggleArm('"+k+"')\">ARM "+n+"</button>"+
    "<label>Drive PWM <span id='"+k+"Lbl'>0</span> (-255..255)</label>"+
    "<input type='range' id='"+k+"Slide' min='-255' max='255' value='0' step='5'"+
    " oninput=\"drive('"+k+"',this.value)\" onchange=\"drive('"+k+"',this.value)\">"+
    "<div class='btns' style='margin:10px 0'>"+
    "<button onclick=\"quick('"+k+"',128)\">Fwd 50%</button>"+
    "<button onclick=\"quick('"+k+"',-128)\">Rev 50%</button>"+
    "<button onclick=\"mode('"+k+"','brake')\">Safe stop</button>"+
    "<button onclick=\"mode('"+k+"','coast')\">Coast</button></div>"+
    "<div class='row'><span class='k'>Direction (wiring fix)</span>"+
    "<button id='"+k+"Inv' onclick=\"toggleInv('"+k+"')\">Normal</button></div>"+
    "<div class='row'><span class='k'>Applied PWM</span><span class='metric accent' id='"+k+"Applied'>0</span></div>"+
    "<div class='row'><span class='k'>RPM</span><span class='metric good' id='"+k+"Rpm'>0</span></div></div>");
});
function drive(k,v){document.getElementById(k+'Lbl').textContent=v;post('/api/motor?ch='+k+'&pwm='+v);}
function quick(k,v){document.getElementById(k+'Slide').value=v;drive(k,v);}
function mode(k,m){resetSlider(k);post('/api/motor?ch='+k+'&mode='+m);}
function toggleArm(k){post('/api/motor?ch='+k+'&arm='+(armed[k]?0:1));resetSlider(k);}
function toggleInv(k){post('/api/motor?ch='+k+'&inv='+(inv[k]?0:1));}
function resetSlider(k){document.getElementById(k+'Slide').value=0;document.getElementById(k+'Lbl').textContent='0';}
function armAll(v){post('/api/motor?ch=all&arm='+v);M.forEach(({k})=>resetSlider(k));}
function modeAll(m){post('/api/motor?ch=all&mode='+m);M.forEach(({k})=>resetSlider(k));}
function estop(){post('/api/estop');M.forEach(({k})=>resetSlider(k));}
function paintArm(k,n,on){
  armed[k]=on;
  const tag=document.getElementById(k+'Arm'),btn=document.getElementById(k+'ArmBtn');
  tag.textContent=on?'ARMED':'DISARMED';tag.className='tag '+(on?'good':'bad');
  btn.textContent=(on?'Disarm ':'ARM ')+n;btn.className='wide'+(on?' bad':'');
}
window.onTelemetry=d=>{
  M.forEach(({k,n},i)=>{
    document.getElementById(k+'Applied').textContent=d.m[i];
    document.getElementById(k+'Rpm').textContent=d.enc[i].r.toFixed(0);
    paintArm(k,n,d.a[i]);
    if(d.inv){inv[k]=d.inv[i];const b=document.getElementById(k+'Inv');
      b.textContent=inv[k]?'REVERSED':'Normal';b.className=inv[k]?'warn':'';}
  });
};
)js"; }

// =============================================================== DRIVE
inline const char* DRIVE_BODY() { return R"html(
<h1>Drive Test</h1>
<p class='sub'>Whole-car motion: forward, backward and in-place spins (skid steer).
<b>ARM ALL</b> first, then <b>press and hold</b> a direction — the car stops when you let go.
&#9888; Put the car on the floor or a stand with clearance; wheels on both sides move.</p>
<button class='estop' onclick="estop()">EMERGENCY STOP</button>
<div class='card' style='margin-bottom:14px'><h2>Arm <span class='tag bad' id='dArm'>0/4 ARMED</span></h2>
  <div class='btns'>
    <button class='good' onclick="post('/api/motor?ch=all&arm=1')">ARM ALL</button>
    <button class='bad' onclick="post('/api/motor?ch=all&arm=0')">Disarm all</button>
  </div>
</div>
<div class='card' style='margin-bottom:14px'><h2>Speed</h2>
  <label>Drive PWM <span id='spdLbl'>120</span> (0..255)</label>
  <input type='range' id='spd' min='0' max='255' value='120' step='5'
         oninput="document.getElementById('spdLbl').textContent=this.value">
</div>
<div class='card' style='margin-bottom:14px'><h2>Direction (hold to drive)</h2>
  <div class='dpad'>
    <span></span><button id='bFwd'>&#9650;<br>Fwd</button><span></span>
    <button id='bLeft'>&#9664;<br>Left</button><button class='bad' onclick="stopCar()">STOP</button><button id='bRight'>&#9654;<br>Right</button>
    <span></span><button id='bRev'>&#9660;<br>Back</button><span></span>
  </div>
</div>
<div class='card'><h2>Wheels</h2>
  <table>
    <thead><tr><th>Wheel</th><th style='text-align:right'>PWM</th><th style='text-align:right'>RPM</th></tr></thead>
    <tbody id='dBody'></tbody>
  </table>
</div>
<button class='wide good' style='margin-top:14px' onclick="post('/api/estop/clear')">Clear E-Stop</button>
)html"; }

inline const char* DRIVE_JS() { return R"js(
const LABELS=['Left-Front','Left-Rear','Right-Front','Right-Rear'];
const tb=document.getElementById('dBody');
LABELS.forEach((n,i)=>tb.insertAdjacentHTML('beforeend',
  "<tr><td>"+n+"</td><td class='num' id='dP"+i+"'>0</td><td class='num' id='dR"+i+"'>0</td></tr>"));
let driving=null,tick=null;
function send(dir){post('/api/drive?dir='+dir+'&pwm='+document.getElementById('spd').value);}
function startDrive(btn,dir){
  driving=dir;btn.classList.add('hold');send(dir);
  clearInterval(tick);
  tick=setInterval(()=>{if(driving)send(driving);},300); // keep-alive re-send
}
function stopCar(){
  driving=null;clearInterval(tick);tick=null;
  document.querySelectorAll('.dpad .hold').forEach(b=>b.classList.remove('hold'));
  post('/api/drive?dir=stop');
}
function estop(){stopCar();post('/api/estop');}
[['bFwd','fwd'],['bRev','rev'],['bLeft','left'],['bRight','right']].forEach(([id,dir])=>{
  const b=document.getElementById(id);
  b.addEventListener('pointerdown',e=>{e.preventDefault();startDrive(b,dir);});
  ['pointerup','pointerleave','pointercancel'].forEach(ev=>
    b.addEventListener(ev,()=>{if(driving)stopCar();}));
  b.addEventListener('contextmenu',e=>e.preventDefault());
});
window.onTelemetry=d=>{
  d.m.forEach((v,i)=>{
    document.getElementById('dP'+i).textContent=v;
    document.getElementById('dR'+i).textContent=d.enc[i].r.toFixed(0);
  });
  const n=d.a.filter(x=>x).length,t=document.getElementById('dArm');
  t.textContent=(d.estop?'E-STOP':n+'/4 ARMED');
  t.className='tag '+(d.estop?'bad':n===4?'good':n?'warn':'bad');
};
)js"; }

// ============================================================ ENCODERS
inline const char* ENCODERS_BODY() { return R"html(
<h1>Encoder Test</h1>
<p class='sub'>Live quadrature counts and computed RPM for each of the four wheels.</p>
<div class='card'>
  <h2>Live Readings <button onclick="post('/api/encoder/reset?ch=all')">Reset all</button></h2>
  <table>
    <thead><tr><th>Wheel</th><th style='text-align:right'>Count</th><th style='text-align:right'>RPM</th><th></th></tr></thead>
    <tbody id='encBody'></tbody>
  </table>
</div>
<div class='grid'>
  <div class='card'><h2>Counts-Per-Revolution</h2>
    <p class='hint'>CPR converts raw counts into RPM. It is shared by all wheels and saved to flash.</p>
    <label>Active CPR</label>
    <input type='number' id='cprIn' step='1'>
    <button class='wide pri' style='margin-top:10px' onclick="saveCpr()">Save CPR to flash</button>
  </div>
  <div class='card'><h2>Calibration Wizard</h2>
    <div class='step'><b>1.</b> Pick a wheel and free its shaft to spin by hand.
      <label>Wheel</label>
      <select id='wizWheel' style='width:100%;padding:10px;background:var(--panel2);color:var(--text);border:1px solid var(--line);border-radius:8px'></select>
    </div>
    <div class='step'><b>2.</b> Mark the wheel, then zero the counter.
      <button class='wide' style='margin-top:8px' onclick="post('/api/encoder/reset?ch='+document.getElementById('wizWheel').value)">Zero this wheel</button>
    </div>
    <div class='step'><b>3.</b> Turn it <b>exactly one full revolution</b> forward. Captured count:
      <div class='big' id='wizCount'>0</div>
    </div>
    <div class='step'><b>4.</b> Save that count as the CPR.
      <button class='wide good' style='margin-top:8px' onclick="applyWiz()">Use count as CPR</button>
    </div>
  </div>
</div>
)html"; }

inline const char* ENCODERS_JS() { return R"js(
const LABELS=['Left-Front','Left-Rear','Right-Front','Right-Rear'];
let counts=[0,0,0,0];
const tb=document.getElementById('encBody'),sel=document.getElementById('wizWheel');
LABELS.forEach((n,i)=>{
  tb.insertAdjacentHTML('beforeend',
    "<tr><td>"+n+"</td><td class='num' id='c"+i+"'>0</td><td class='num' id='r"+i+"'>0</td>"+
    "<td style='text-align:right'><button onclick=\"post('/api/encoder/reset?ch="+i+"')\">Zero</button></td></tr>");
  sel.insertAdjacentHTML('beforeend',"<option value='"+i+"'>"+n+"</option>");
});
fetch('/api/encoder/cpr').then(r=>r.json()).then(d=>document.getElementById('cprIn').value=d.cpr);
function saveCpr(){const v=parseFloat(document.getElementById('cprIn').value);
  postJSON('/api/encoder/cpr',{cpr:v}).then(()=>alert('Saved CPR = '+v));}
function applyWiz(){const c=Math.abs(counts[parseInt(sel.value)]);
  if(!c){alert('Count is 0 — did you turn the wheel one full turn?');return;}
  document.getElementById('cprIn').value=c;saveCpr();}
window.onTelemetry=d=>{
  d.enc.forEach((e,i)=>{counts[i]=e.c;
    document.getElementById('c'+i).textContent=e.c;
    document.getElementById('r'+i).textContent=e.r.toFixed(1);});
  document.getElementById('wizCount').textContent=Math.abs(counts[parseInt(sel.value)]);
};
)js"; }

// ============================================================== CAMERA
inline const char* CAMERA_BODY() { return R"html(
<h1>Camera</h1>
<p class='sub'>Live MJPEG from the separate ESP32-CAM board. The stream goes straight
from the CAM to your phone — the drive ESP32 never touches video.</p>
<div class='card'>
  <h2>Live Feed <span class='tag' id='camStat'>connecting</span></h2>
  <img id='camImg' style='width:100%;border-radius:8px;background:#000;min-height:200px' alt=''>
  <div class='btns' style='margin-top:10px'>
    <button class='pri' onclick='snap()'>Snapshot</button>
    <button onclick='flipV()'>Flip &#8645;</button>
    <button onclick='mirrorH()'>Mirror &#8644;</button>
    <button onclick='reloadStream()'>Reconnect</button>
  </div>
  <div class='row' style='margin-top:6px'><span class='k'>FPS</span>
    <span class='metric good' id='camFps'>--</span></div>
  <div class='row'><span class='k'>Bitrate</span>
    <span><span class='metric accent' id='camKbps'>--</span> <span class='unit'>kbit/s</span></span></div>
  <div class='row'><span class='k'>Avg frame</span>
    <span><span class='metric' id='camKb'>--</span> <span class='unit'>KB</span></span></div>
</div>
<div class='card' style='margin-top:14px'><h2>Settings</h2>
  <label>Resolution (higher = slower)</label>
  <select id='camRes' onchange='setRes(this.value)'
    style='width:100%;padding:10px;background:var(--panel2);color:var(--text);border:1px solid var(--line);border-radius:8px'>
    <option value='5'>QVGA 320&times;240 (fastest)</option>
    <option value='8' selected>VGA 640&times;480 (default)</option>
    <option value='9'>SVGA 800&times;600</option>
    <option value='10'>XGA 1024&times;768</option>
    <option value='11'>HD 1280&times;720 (slow)</option>
  </select>
  <p class='hint' style='margin-top:10px'>If the feed shows <b>offline</b>: check the
  ESP32-CAM has power (5&nbsp;V &ge; 2&nbsp;A), give it ~10&nbsp;s to join the WiFi after
  power-up, then tap Reconnect. The CAM's own status page is at
  <span id='camAddr' style='font-family:ui-monospace,monospace'></span>.</p>
</div>
)html"; }

inline String CAMERA_JS() {
    // Inject the CAM's address from config.h so JS has the stream URLs.
    String s;
    s += "const CAM='http://";  s += CAM_HOST; s += "';";
    s += "const CAMS='http://"; s += CAM_HOST; s += ":"; s += CAM_STREAM_PORT; s += "';";
    s += R"js(
const img=document.getElementById('camImg'),st=document.getElementById('camStat');
document.getElementById('camAddr').textContent=CAM;
let vf=0,hm=0,retry=null;
function reloadStream(){
  clearTimeout(retry);retry=null;
  st.textContent='connecting';st.className='tag';
  img.src=CAMS+'/stream?t='+Date.now();   // cache-buster forces a fresh connect
}
img.addEventListener('load',()=>{st.textContent='LIVE';st.className='tag good';});
img.addEventListener('error',()=>{
  st.textContent='offline';st.className='tag bad';
  if(!retry)retry=setTimeout(()=>{retry=null;reloadStream();},4000);
});
function ctl(v,val){fetch(CAM+'/control?var='+v+'&val='+val).catch(()=>{});}
function setRes(v){ctl('framesize',v);}
function flipV(){vf=vf?0:1;ctl('vflip',vf);}
function mirrorH(){hm=hm?0:1;ctl('hmirror',hm);}
function snap(){window.open(CAM+'/capture?t='+Date.now(),'_blank');}
// FPS/bitrate: poll the CAM's cumulative counters and rate the deltas.
let prevStat=null;
function statVals(fps,kbps,kb){
  document.getElementById('camFps').textContent=fps;
  document.getElementById('camKbps').textContent=kbps;
  document.getElementById('camKb').textContent=kb;
}
setInterval(()=>{
  fetch(CAM+'/status').then(r=>r.json()).then(d=>{
    if(prevStat&&d.ms>prevStat.ms&&d.frames>=prevStat.frames){
      const dt=d.ms-prevStat.ms,df=d.frames-prevStat.frames,db=d.bytes-prevStat.bytes;
      statVals((df*1000/dt).toFixed(1),(db*8/dt).toFixed(0),
               df>0?(db/df/1024).toFixed(1):'--');
    }
    prevStat=d;
  }).catch(()=>{prevStat=null;statVals('--','--','--');});
},1000);
window.addEventListener('load',reloadStream);
)js";
    return s;
}

// ================================================================ IMU
inline const char* IMU_BODY() { return R"html(
<h1>IMU / Sensor Bridge</h1>
<p class='sub'>MPU/LiDAR/ToF sensors are planned on the Raspberry Pi side; ESP32 pins stay focused on motors and encoders.</p>
<div id='imuOff' class='card' style='display:none'>
  <h2>Pi sensor bridge disabled</h2>
  <p class='hint'>The Raspberry Pi should send future sensor/control data over WiFi. GPIO21/GPIO22 are now used by the Left-Rear encoder, so there is no spare wired UART pin on this ESP32 map.</p>
</div>
<div id='imuOn' style='display:none'>
  <div class='grid'>
    <div class='card'><h2>Heading <span class='tag' id='imuStat'>--</span></h2>
      <span class='metric accent' id='iHead'>--</span><span class='unit'> deg</span></div>
    <div class='card'><h2>Roll</h2><span class='metric' id='iRoll'>--</span><span class='unit'> deg</span></div>
    <div class='card'><h2>Pitch</h2><span class='metric' id='iPitch'>--</span><span class='unit'> deg</span></div>
  </div>
  <div class='card' style='margin-top:14px'><h2>Calibration (0&ndash;3 each)</h2>
    <div class='row'><span class='k'>System</span><div style='flex:1;margin-left:14px'><div class='bar'><span id='calSys'></span></div></div></div>
    <div class='row'><span class='k'>Gyro</span><div style='flex:1;margin-left:14px'><div class='bar'><span id='calGyro'></span></div></div></div>
    <div class='row'><span class='k'>Accel</span><div style='flex:1;margin-left:14px'><div class='bar'><span id='calAccel'></span></div></div></div>
    <div class='row'><span class='k'>Mag</span><div style='flex:1;margin-left:14px'><div class='bar'><span id='calMag'></span></div></div></div>
    <p class='hint' style='margin-top:10px'>Move the car through figure-8s until each bar reaches full.</p>
  </div>
</div>
)html"; }

inline const char* IMU_JS() { return R"js(
function bar(id,v){document.getElementById(id).style.width=(v/3*100)+'%';}
window.onTelemetry=d=>{
  const off=document.getElementById('imuOff'),on=document.getElementById('imuOn');
  if(!d.imu.en){off.style.display='block';on.style.display='none';return;}
  off.style.display='none';on.style.display='block';
  const st=document.getElementById('imuStat');
  if(!d.imu.ok){st.textContent='no sensor';st.className='tag warn';return;}
  st.textContent='live';st.className='tag good';
  document.getElementById('iHead').textContent=d.imu.h.toFixed(1);
  document.getElementById('iRoll').textContent=d.imu.r.toFixed(1);
  document.getElementById('iPitch').textContent=d.imu.p.toFixed(1);
  bar('calSys',d.imu.cs);bar('calGyro',d.imu.cg);bar('calAccel',d.imu.ca);bar('calMag',d.imu.cm);
};
)js"; }

} // namespace web
