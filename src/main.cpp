#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <AccelStepper.h>
#include "config.h"

// ===================== Motion =====================
AccelStepper s1(AccelStepper::HALF4WIRE, M1_IN1, M1_IN3, M1_IN2, M1_IN4);
AccelStepper s2(AccelStepper::HALF4WIRE, M2_IN1, M2_IN3, M2_IN2, M2_IN4);

static float rpmToStepsPerSec(int rpm){
  float sps = (float)rpm * (float)STEPS_PER_REV / 60.0f;
  if (sps > 1200.0f) sps = 1200.0f;
  if (sps < 50.0f)   sps = 50.0f;
  return sps;
}
static void applyMotionParams(){
  float sps = rpmToStepsPerSec(STEP_RPM);
  s1.setMaxSpeed(sps); s2.setMaxSpeed(sps);
  s1.setAcceleration(sps * 1.2f); s2.setAcceleration(sps * 1.2f);
}

static bool enabled = true;
static unsigned long nextDue1=0, nextDue2=0;
static int lastDir1=+1, lastDir2=+1;

static unsigned long intervalFromTPD(int tpd){
  if (tpd <= 0) return 0;
  return 86400000UL / (unsigned long)tpd;
}
static int pickDir(int plan, int& lastDir){
  if (plan==DIR_CW){ lastDir=+1; return +1; }
  if (plan==DIR_CCW){ lastDir=-1; return -1; }
  lastDir = (lastDir>0)?-1:+1; return lastDir;
}

// DPDT switch presets
static int currentMode=0, stableMode=0; static unsigned long lastModeReadMs=0;
static int readModeRaw(){
  int a=digitalRead(MODE_PIN_A), b=digitalRead(MODE_PIN_B);
  if (a==LOW && b==HIGH) return 0;
  if (a==HIGH && b==HIGH) return 1;
  if (a==HIGH && b==LOW) return 2;
  return 1;
}
static void updateModeDebounced(){
  int m=readModeRaw(); unsigned long now=millis();
  if (m!=currentMode){ currentMode=m; lastModeReadMs=now; }
  else if ((now-lastModeReadMs)>=MODE_DEBOUNCE_MS){ stableMode=m; }
}
static void applyModePreset(int mode){
  if (mode==0){ TPD_M1=500; TPD_M2=500; DIRPLAN_M1=DIR_ALT; DIRPLAN_M2=DIR_ALT; }
  else if (mode==2){ TPD_M1=800; TPD_M2=800; DIRPLAN_M1=DIR_CW; DIRPLAN_M2=DIR_CCW; }
}

// Turbo
static bool turboActive=false; static unsigned long turboEndMs=0; static bool turboM1=false, turboM2=false;
static bool turboStopping=false;  // flag to complete current rotation before stopping

static void startTurbo(bool m1,bool m2,unsigned long minutes){
  turboM1=m1; turboM2=m2; turboActive=true; turboStopping=false;
  turboEndMs=millis()+minutes*60UL*1000UL;
  long span=6L*STEPS_PER_REV*minutes;
  if (turboM1) s1.move(s1.distanceToGo()+span);
  if (turboM2) s2.move(s2.distanceToGo()+span);
}

static void updateTurbo(){
  if (!turboActive) return;
  
  // Check if time has expired
  if (millis()>=turboEndMs && !turboStopping){
    turboStopping=true;  // Enter stopping phase - complete current rotations
  }
  
  // If stopping, check if both motors have completed their current rotation
  if (turboStopping){
    bool m1Done = !turboM1 || (s1.distanceToGo()==0);
    bool m2Done = !turboM2 || (s2.distanceToGo()==0);
    
    if (m1Done && m2Done){
      // Both motors finished their rotations, fully stop turbo mode
      turboActive=false; turboM1=turboM2=false; turboStopping=false;
    }
    // Don't queue new rotations while stopping
    return;
  }
  
  // Normal operation - queue next rotation if motor completed current one
  if (turboM1 && s1.distanceToGo()==0) s1.move(2L*STEPS_PER_REV);
  if (turboM2 && s2.distanceToGo()==0) s2.move(2L*STEPS_PER_REV);
}

// ===================== Wi-Fi / Web =====================
WebServer server(80);
Preferences prefs;

String wifiSsid=WIFI_SSID;
String wifiPass=WIFI_PASS;

String ipToStr(const IPAddress& ip){ return String(ip[0])+"."+ip[1]+"."+ip[2]+"."+ip[3]; }
void indicate(bool on){ if (LED_PIN>=0){ pinMode(LED_PIN,OUTPUT); digitalWrite(LED_PIN,on?HIGH:LOW);} }

// Track HTTP server state and (re)start it only when Wi-Fi is ready.
static bool httpStarted = false;

static void startHttpServerIfNeeded() {
  if (!httpStarted) {
    delay(300);                 // give netif a moment after AP/STA up
    server.begin();
    httpStarted = true;
    Serial.println("HTTP: server started on port 80");
  }
}

static void saveWifiCreds(const String& ssid, const String& pass){
  if (!prefs.begin("winder", false)) return;
  prefs.putString("ssid", ssid);
  prefs.putString("wpass", pass);
  prefs.end();
  wifiSsid=ssid; wifiPass=pass;
}

// Event logging (helps diagnose join problems)
static void onWiFiEvent(WiFiEvent_t e){
  switch(e){
    case ARDUINO_EVENT_WIFI_STA_CONNECTED: Serial.println("STA: connected"); break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP: Serial.printf("STA: IP %s\n", ipToStr(WiFi.localIP()).c_str()); break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: Serial.println("STA: disconnected"); break;
    case ARDUINO_EVENT_WIFI_AP_START: Serial.println("AP: started"); break;
    case ARDUINO_EVENT_WIFI_AP_STOP: Serial.println("AP: stopped"); break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED: Serial.println("AP: station joined"); break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED: Serial.println("AP: station left"); break;
    default: break;
  }
}

// Strong AP bring-up: AP-only, country, tx power, channel sweep
static bool startAP(){
  Serial.println("Starting SoftAP (AP-only) …");
  WiFi.persistent(false);
  WiFi.disconnect(true, true); // drop STA state
  delay(100);

  WiFi.mode(WIFI_OFF);
  delay(50);

  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  WiFi.mode(WIFI_AP);
  delay(50);

  IPAddress ip(192,168,4,1), gw(192,168,4,1), mask(255,255,255,0);
  if (!WiFi.softAPConfig(ip, gw, mask)){
    Serial.println("softAPConfig failed");
  }

  const char* ap_ssid = AP_SSID;
  const char* ap_pass = (strlen(AP_PASS) ? AP_PASS : "winder1234"); // default WPA2 pass
  bool ok=false;
  int channels[3]={1,6,11};
  for (int i=0;i<3 && !ok;i++){
    int ch=channels[i];
    Serial.printf("softAP SSID=%s ch=%d …\n", ap_ssid, ch);
    ok = WiFi.softAP(ap_ssid, ap_pass, ch, /*hidden*/false, /*max_conn*/4);
    delay(300);
  }
  IPAddress apIP = WiFi.softAPIP();
  Serial.printf("AP %s %s at %s (pass: %s)\n", ap_ssid, ok?"UP":"FAILED", ipToStr(apIP).c_str(), ap_pass);

  // >>> start HTTP AFTER AP is up <<<
  httpStarted = false;         // force a clean (re)bind
  startHttpServerIfNeeded();

  return ok;
}

static bool startSTA(unsigned long timeout_ms=10000){
  if (wifiSsid.length()==0) return false;
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  Serial.printf("STA: connecting to %s …\n", wifiSsid.c_str());
  unsigned long t0=millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t0<timeout_ms){ delay(200); Serial.print("."); }
  if (WiFi.status()==WL_CONNECTED){
    Serial.printf("\nSTA: connected, IP %s\n", ipToStr(WiFi.localIP()).c_str());
    if (MDNS.begin("winder")){ MDNS.addService("http","tcp",80); Serial.println("mDNS: http://winder.local"); }

    // >>> start HTTP AFTER STA is up <<<
    httpStarted = false;        // force a clean (re)bind if we were in AP before
    startHttpServerIfNeeded();

    return true;
  }
  Serial.println("\nSTA: timeout");
  return false;
}


static void startWiFi(){
  WiFi.onEvent(onWiFiEvent);
  bool staOK = startSTA(8000);
  if (!staOK){
    Serial.println("Falling back to AP");
    if (!startAP()){
      Serial.println("AP failed; retrying once …");
      delay(500);
      startAP();
    }
  }
}

// ========== Persistence ==========
static void loadPrefs(){
  if (!prefs.begin("winder", true)) return;
  STEP_RPM   = prefs.getInt("rpm", STEP_RPM);
  TPD_M1     = prefs.getInt("tpd1", TPD_M1);
  TPD_M2     = prefs.getInt("tpd2", TPD_M2);
  DIRPLAN_M1 = prefs.getInt("dir1", DIRPLAN_M1);
  DIRPLAN_M2 = prefs.getInt("dir2", DIRPLAN_M2);
  wifiSsid   = prefs.getString("ssid", WIFI_SSID);
  wifiPass   = prefs.getString("wpass", WIFI_PASS);
  prefs.end();
}
static void savePrefs(){
  if (!prefs.begin("winder", false)) return;
  prefs.putInt("rpm", STEP_RPM);
  prefs.putInt("tpd1", TPD_M1);
  prefs.putInt("tpd2", TPD_M2);
  prefs.putInt("dir1", DIRPLAN_M1);
  prefs.putInt("dir2", DIRPLAN_M2);
  // Wi-Fi creds saved via saveWifiCreds()
  prefs.end();
}

// ===================== UI (HTML) =====================
static const char PAGE_INDEX[] PROGMEM = R"HTML(
<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Winder</title>
<style>
*{box-sizing:border-box}
:root{ --gap:12px; --rad:12px; --bg:#ffffff; --fg:#111; --card:#f7f7f7; --pill:#efefef; --border:#ddd; --muted:#666; }
:root[data-theme="dark"]{ --bg:#0f1115; --fg:#e9eef7; --card:#1a1f28; --pill:#232a34; --border:#2b3440; --muted:#9aa6b2; }
@media (prefers-color-scheme: dark){
  :root:not([data-theme="light"]){ --bg:#0f1115; --fg:#e9eef7; --card:#1a1f28; --pill:#232a34; --border:#2b3440; --muted:#9aa6b2; }
}
html,body{background:var(--bg); color:var(--fg)}
body{font-family:system-ui,Segoe UI,Roboto,sans-serif;margin:20px;max-width:820px}
h2{margin:0 0 10px}
fieldset{border:1px solid var(--border);border-radius:var(--rad);margin:14px 0;padding:12px;background:var(--card)}
legend{padding:0 6px;font-weight:600}
label{font-weight:600;display:block;margin:4px 0}
small.helper{display:block;color:var(--muted)}
input,select,button{ width:100%; padding:8px 10px;border-radius:8px;border:1px solid var(--border);background:var(--bg);color:var(--fg);font:inherit }
button{cursor:pointer}button:active{transform:translateY(1px)}
.row{display:flex;gap:var(--gap);flex-wrap:wrap;align-items:center}
.right{display:flex;gap:var(--gap);justify-content:flex-end;flex-wrap:wrap}
.pill{display:inline-block;padding:6px 10px;border-radius:999px;background:var(--pill);margin-right:8px}
.ok{background:#1b8f3a20}.warn{background:#f5a52426}
.note{font-size:.92rem;color:var(--muted)}
.pair{ display:grid; grid-template-columns: repeat(2, minmax(160px, 1fr)); gap:var(--gap) }
.pair.wide{ grid-template-columns: repeat(2, minmax(240px, 1fr)); }
.cell{ min-width:0 }
.pair-2{ display:grid; grid-template-columns: 220px 1fr; gap:var(--gap); align-items:end }
@media (max-width:560px){ .pair-2{ grid-template-columns: 1fr } }
.btnrow{ display:flex; gap:8px; flex-wrap:nowrap; }
@media (max-width:560px){ .btnrow{ flex-wrap:wrap } }
.btnrow button{ flex:1 1 0 }
@media (max-width:360px){ .pair{ grid-template-columns: 1fr } .pair.wide{ grid-template-columns: 1fr } }
</style>

<h2>Dual Watch Winder</h2>
<div class="row" style="margin-bottom:6px">
  <div id="net" class="pill">Loading…</div>
  <div id="runstate" class="pill">—</div>
  <span>Switch mode: <b id="swmode">—</b></span>
  <div class="right" style="margin-left:auto"><button id="themeToggle" title="Toggle dark mode">🌙</button></div>
</div>

<fieldset><legend>Controls</legend>
  <div class="row" style="gap:12px;align-items:center">
    <button id="start" style="max-width:180px">Start</button>
    <button id="stop"  style="max-width:180px">Stop</button>
  </div>
</fieldset>

<fieldset><legend>Parameters</legend>
  <div class="note" style="margin-bottom:8px">Tip: <b>typical automatic watches are ~650–800 TPD</b>.</div>

  <div class="pair">
    <div class="cell">
      <label for="tpd1">Motor 1 – TPD</label>
      <input type="number" id="tpd1" min="0" max="1200" step="50"/>
      <small class="helper">0 = disabled</small>
    </div>
    <div class="cell">
      <label for="dir1">Motor 1 – Direction</label>
      <select id="dir1"><option value="1">CW</option><option value="-1">CCW</option><option value="0">Alternate</option></select>
    </div>
  </div>

  <div class="pair" style="margin-top:10px">
    <div class="cell">
      <label for="tpd2">Motor 2 – TPD</label>
      <input type="number" id="tpd2" min="0" max="1200" step="50"/>
      <small class="helper">0 = disabled</small>
    </div>
    <div class="cell">
      <label for="dir2">Motor 2 – Direction</label>
      <select id="dir2"><option value="1">CW</option><option value="-1">CCW</option><option value="0">Alternate</option></select>
    </div>
  </div>

  <div class="right" style="margin-top:10px"><button id="save" style="max-width:160px">Save</button></div>
</fieldset>

<fieldset><legend>Turbo Mode</legend>
  <div class="pair-2">
    <div class="cell">
      <label for="tdur">Duration</label>
      <select id="tdur"><option value="5">5 min</option><option value="10">10 min</option></select>
    </div>
    <div class="cell">
      <label style="font-weight:700;margin-bottom:4px">Winder Select</label>
      <div class="btnrow">
        <button id="t1">Motor 1</button>
        <button id="t2">Motor 2</button>
        <button id="tboth">Both</button>
      </div>
    </div>
  </div>
  <div class="note" id="tstatus" style="margin-top:8px">—</div>
</fieldset>

<fieldset><legend>Status</legend>
  <div class="pair">
    <div class="cell">Next M1 in: <b id="n1">—</b></div>
    <div class="cell">Next M2 in: <b id="n2">—</b></div>
  </div>
</fieldset>

<fieldset><legend>Wi-Fi Setup</legend>
  <div class="note" style="margin-bottom:6px">Connect to <b>Winder-Setup</b>, then choose your home Wi-Fi and tap <b>Connect</b>.</div>
  <div class="pair wide">
    <div class="cell">
      <label for="wssid">Wi-Fi SSID</label>
      <select id="wssid">
        <option value="">(Scanning…)</option>
        <option value="__other__">Other…</option>
      </select>
      <input id="wssid_other" placeholder="Enter SSID" style="display:none;margin-top:8px"/>
    </div>
    <div class="cell">
      <label for="wpass">Password</label>
      <input id="wpass" type="password" placeholder="Password"/>
    </div>
  </div>
  <div class="right" style="margin-top:8px"><button id="wconnect" style="max-width:180px">Connect</button></div>
  <div class="note" id="wstatus" style="margin-top:8px">—</div>
</fieldset>

<script>
const $=s=>document.querySelector(s);
async function api(p,o={}){const r=await fetch(p,Object.assign({headers:{'Content-Type':'application/json'}},o));return r.json().catch(()=>({}))}
function fmt(ms){if(ms<0)return'—';const s=Math.round(ms/1000);const m=Math.floor(s/60),ss=s%60;return(m>0?m+'m ':'')+ss+'s'}

(function initTheme(){
  const saved=localStorage.getItem('theme');
  if(saved==='dark'||saved==='light') document.documentElement.setAttribute('data-theme',saved);
  $('#themeToggle').onclick=()=>{
    const cur=document.documentElement.getAttribute('data-theme');
    const next = cur==='dark' ? 'light' : 'dark';
    document.documentElement.setAttribute('data-theme', next);
    localStorage.setItem('theme', next);
  };
})();

async function refresh(){
  const s=await api('/status');
  $('#net').textContent=s.network||'—';$('#net').className='pill '+(s.network?.includes('AP')?'warn':'ok');
  $('#runstate').textContent=s.enabled?'Running':'Stopped';$('#runstate').className='pill '+(s.enabled?'ok':'');
  $('#tpd1').value=s.tpd1;$('#tpd2').value=s.tpd2;$('#dir1').value=s.dir1;$('#dir2').value=s.dir2;
  $('#swmode').textContent=s.switch_mode;
  $('#n1').textContent=fmt(s.next1_ms);$('#n2').textContent=fmt(s.next2_ms);
  $('#tstatus').textContent=s.turbo_active?('Turbo '+(s.turbo_m1&&s.turbo_m2?'Both':(s.turbo_m1?'M1':'M2'))+' '+fmt(s.turbo_left_ms)):'—';
}

async function loadSSIDs(){
  const sel=$('#wssid'); sel.innerHTML='<option value="">(Scanning…)</option><option value="__other__">Other…</option>';
  try{
    const res=await api('/scan');
    const list=(res && Array.isArray(res.ssids))?res.ssids:[];
    let html='';
    for(const s of list){ const esc=String(s).replace(/"/g,'&quot;'); html+=`<option value="${esc}">${esc}</option>`; }
    html+='<option value="__other__">Other…</option>';
    sel.innerHTML=html||'<option value="">(No networks found)</option><option value="__other__">Other…</option>';
  }catch(e){
    sel.innerHTML='<option value="">(Scan failed)</option><option value="__other__">Other…</option>';
  }
}
$('#wssid').addEventListener('change', ()=>{
  const other=$('#wssid').value==='__other__';
  $('#wssid_other').style.display=other?'block':'none';
});

$('#start').onclick=async()=>{await api('/start',{method:'POST',body:'{}'});refresh();}
$('#stop').onclick=async()=>{await api('/stop',{method:'POST',body:'{}'});refresh();}
$('#save').onclick=async()=>{const b={tpd1:+$('#tpd1').value,tpd2:+$('#tpd2').value,dir1:+$('#dir1').value,dir2:+$('#dir2').value};await api('/config',{method:'POST',body:JSON.stringify(b)});refresh();}
$('#t1').onclick=async()=>{await api('/turbo',{method:'POST',body:JSON.stringify({m1:true,m2:false,min:+$('#tdur').value})});refresh();}
$('#t2').onclick=async()=>{await api('/turbo',{method:'POST',body:JSON.stringify({m1:false,m2:true,min:+$('#tdur').value})});refresh();}
$('#tboth').onclick=async()=>{await api('/turbo',{method:'POST',body:JSON.stringify({m1:true,m2:true,min:+$('#tdur').value})});refresh();}
$('#wconnect').onclick=async()=>{
  let ssid=$('#wssid').value; if(ssid==='__other__') ssid=$('#wssid_other').value.trim();
  const pass=$('#wpass').value;
  if(!ssid){$('#wstatus').textContent='Please select or enter an SSID';return;}
  $('#wstatus').textContent='Connecting…';
  const res=await api('/wifi',{method:'POST',body:JSON.stringify({ssid,pass})});
  if(res.ok){$('#wstatus').innerHTML='Connected to <b>'+ssid+'</b><br>Open '+(res.mdns||'')+' or '+(res.ip||'');}
  else{$('#wstatus').textContent='Connection failed. Check SSID/password and try again.';}
};

refresh(); setInterval(refresh,3000); loadSSIDs();
</script>
)HTML";

// ===================== Routes =====================
static const char RESP_OK[] PROGMEM = "{\"ok\":true}";
void setupRoutes(){
  server.on("/", HTTP_GET, [](){ server.send_P(200,"text/html",PAGE_INDEX); });

  // Connectivity checks some OSes do
  server.on("/generate_204", HTTP_GET, [](){ server.send(204); });
  server.on("/hotspot-detect.html", HTTP_GET, [](){ server.send(200,"text/html","<meta http-equiv='refresh' content='0; url=/'/>"); });
  server.on("/connecttest.txt", HTTP_GET, [](){ server.send(200,"text/plain","OK"); });

  server.on("/status", HTTP_GET, [](){
    JsonDocument doc;
    String net = (WiFi.getMode()==WIFI_AP || WiFi.getMode()==WIFI_MODE_APSTA)
      ? ("AP: "+String(AP_SSID)+" ("+String(WiFi.softAPgetStationNum())+" client(s)) @ "+String(WiFi.softAPIP().toString().c_str()))
      : ("WiFi: "+WiFi.SSID()+" ("+String(WiFi.localIP().toString().c_str())+") / mDNS: http://winder.local");
    doc["network"]=net; doc["enabled"]=enabled; doc["switch_mode"]=stableMode;
    doc["tpd1"]=TPD_M1; doc["tpd2"]=TPD_M2; doc["dir1"]=DIRPLAN_M1; doc["dir2"]=DIRPLAN_M2;
    long now = (long)millis();

    long rem1 = (TPD_M1>0 && nextDue1>0) ? ((long)nextDue1 - now) : -1;
    if (rem1 < 0 && rem1 != -1) rem1 = 0;

    long rem2 = (TPD_M2>0 && nextDue2>0) ? ((long)nextDue2 - now) : -1;
    if (rem2 < 0 && rem2 != -1) rem2 = 0;

    long tleft = turboActive ? ((long)turboEndMs - (long)millis()) : 0;
    if (tleft < 0) tleft = 0;

    doc["next1_ms"] = rem1;
    doc["next2_ms"] = rem2;
    doc["turbo_active"] = turboActive;
    doc["turbo_m1"] = turboM1;
    doc["turbo_m2"] = turboM2;
    doc["turbo_left_ms"] = tleft;
    String out; serializeJson(doc,out); server.send(200,"application/json",out);
  });

  server.on("/start", HTTP_POST, [](){ enabled=true; server.send_P(200,"application/json",RESP_OK); });
  server.on("/stop",  HTTP_POST, [](){ enabled=false; server.send_P(200,"application/json",RESP_OK); });

  server.on("/config", HTTP_POST, [](){
    if (!server.hasArg("plain")){ server.send(400,"application/json","{\"ok\":false}"); return; }
    JsonDocument doc;
    if (deserializeJson(doc, server.arg("plain"))){ server.send(400,"application/json","{\"ok\":false}"); return; }
    int t1=doc["tpd1"]|TPD_M1, t2=doc["tpd2"]|TPD_M2;
    int d1=doc["dir1"]|DIRPLAN_M1, d2=doc["dir2"]|DIRPLAN_M2;
    t1=constrain(t1,0,1200); t2=constrain(t2,0,1200);
    if (!(d1==-1||d1==0||d1==+1)) d1=0; if (!(d2==-1||d2==0||d2==+1)) d2=0;
    TPD_M1=t1; TPD_M2=t2; DIRPLAN_M1=d1; DIRPLAN_M2=d2;
    savePrefs();
    unsigned long now=millis();
    nextDue1 = (TPD_M1>0)? now+intervalFromTPD(TPD_M1) : 0;
    nextDue2 = (TPD_M2>0)? now+intervalFromTPD(TPD_M2) : 0;
    server.send_P(200,"application/json",RESP_OK);
  });

  server.on("/turbo", HTTP_POST, [](){
    if (!server.hasArg("plain")){ server.send(400,"application/json","{\"ok\":false}"); return; }
    JsonDocument doc;
    if (deserializeJson(doc, server.arg("plain"))){ server.send(400,"application/json","{\"ok\":false}"); return; }
    bool m1 = doc["m1"] | false, m2 = doc["m2"] | false;
    int minutes = constrain((int)(doc["min"]|5), 1, 15);
    startTurbo(m1,m2,(unsigned long)minutes);
    server.send_P(200,"application/json",RESP_OK);
  });

  server.on("/wifi", HTTP_POST, [](){
    if (!server.hasArg("plain")){ server.send(400,"application/json","{\"ok\":false}"); return; }
    JsonDocument doc;
    if (deserializeJson(doc, server.arg("plain"))){ server.send(400,"application/json","{\"ok\":false}"); return; }
    String ssid=String((const char*)doc["ssid"]); String pass=String((const char*)doc["pass"]);
    ssid.trim(); pass.trim();
    if (ssid.length()==0){ server.send(400,"application/json","{\"ok\":false,\"err\":\"empty ssid\"}"); return; }
    saveWifiCreds(ssid, pass);

    // Try STA; on failure ensure AP is up (AP-only)
    if (startSTA(12000)){
      String ip = ipToStr(WiFi.localIP());
      if (MDNS.begin("winder")) MDNS.addService("http","tcp",80);
      String out=String("{\"ok\":true,\"ip\":\"")+ip+"\",\"mdns\":\"http://winder.local\"}";
      server.send(200,"application/json",out);
    } else {
      startAP();               // bring AP back
      httpStarted = false;     // <<< add this
      startHttpServerIfNeeded(); // <<< and this
      server.send(200,"application/json","{\"ok\":false,\"err\":\"connect_failed\"}");
    }
  });

  server.on("/scan", HTTP_GET, [](){
    JsonDocument doc;
    JsonArray arr = doc["ssids"].to<JsonArray>();
    int n = WiFi.scanNetworks(false, true);
    for (int i=0;i<n;i++){
      String s=WiFi.SSID(i); s.trim(); if (s.isEmpty()) continue;
      bool dup=false;
      for (JsonVariantConst v:arr){ const char* e=v.as<const char*>(); if (e && s.equals(e)){ dup=true; break; } }
      if (!dup) arr.add(s);
    }
    String out; serializeJson(doc,out);
    server.send(200,"application/json",out);
  });
}

// ===================== Setup / Loop =====================
void setup(){
  Serial.begin(115200);
  pinMode(MODE_PIN_A, INPUT_PULLUP); pinMode(MODE_PIN_B, INPUT_PULLUP);
  if (LED_PIN>=0){ pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW); }

  loadPrefs();
  applyMotionParams();
  s1.setCurrentPosition(0); s2.setCurrentPosition(0);

  unsigned long now=millis();
  nextDue1 = (TPD_M1>0)? now+intervalFromTPD(TPD_M1) : 0;
  nextDue2 = (TPD_M2>0)? now+intervalFromTPD(TPD_M2) : 0;

  for (int i=0;i<5;i++){ updateModeDebounced(); delay(10); }

  startWiFi();
  setupRoutes();
}

void loop(){
  server.handleClient();

  updateModeDebounced();
  if (stableMode!=1) applyModePreset(stableMode);

  updateTurbo();

  if (!enabled){
    indicate(false);
    if (s1.distanceToGo()!=0) s1.stop();
    if (s2.distanceToGo()!=0) s2.stop();
    s1.run(); s2.run();
    delay(2);
    return;
  }
  indicate(true);

  s1.run(); s2.run();

  unsigned long now=millis();
  if (!turboActive && TPD_M1>0 && nextDue1>0 && now>=nextDue1 && s1.distanceToGo()==0){
    int dir=pickDir(DIRPLAN_M1,lastDir1);
    s1.move((long)dir*STEPS_PER_REV);
    nextDue1 += intervalFromTPD(TPD_M1);
    if ((long)(nextDue1-now) > (long)(2*intervalFromTPD(TPD_M1))) nextDue1 = now+intervalFromTPD(TPD_M1);
  }
  if (!turboActive && TPD_M2>0 && nextDue2>0 && now>=nextDue2 && s2.distanceToGo()==0){
    int dir=pickDir(DIRPLAN_M2,lastDir2);
    s2.move((long)dir*STEPS_PER_REV);
    nextDue2 += intervalFromTPD(TPD_M2);
    if ((long)(nextDue2-now) > (long)(2*intervalFromTPD(TPD_M2))) nextDue2 = now+intervalFromTPD(TPD_M2);
  }
  delay(2);
}
