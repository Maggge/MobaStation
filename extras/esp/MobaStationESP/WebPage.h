/*
  Web interface of the MobaStation ESP firmware: one page, live data over the WebSocket /ws,
  network settings over /api/config, firmware update over /update.
*/
#pragma once

#include <Arduino.h>

static const char WEB_PAGE[] PROGMEM = R"=====(<!doctype html>
<html lang="de">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MobaStation</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--fg:#1d2330;--muted:#6b7280;--line:#e2e5ea;--accent:#2563eb;--ok:#15803d;--warn:#b45309;--bad:#b91c1c}
@media (prefers-color-scheme:dark){:root{--bg:#14171c;--card:#1d2129;--fg:#e6e8ec;--muted:#9aa1ad;--line:#2d333d;--accent:#60a5fa;--ok:#4ade80;--warn:#fbbf24;--bad:#f87171}}
*{box-sizing:border-box}
body{margin:0;font:15px/1.4 system-ui,-apple-system,"Segoe UI",sans-serif;background:var(--bg);color:var(--fg)}
header{display:flex;align-items:center;gap:12px;padding:12px 16px;background:var(--card);border-bottom:1px solid var(--line);position:sticky;top:0;z-index:1}
header h1{font-size:18px;margin:0;flex:1}
nav{display:flex;gap:4px;overflow-x:auto;padding:8px 16px;background:var(--card);border-bottom:1px solid var(--line)}
nav button{border:0;background:none;color:var(--muted);padding:8px 12px;border-radius:6px;font:inherit;cursor:pointer;white-space:nowrap}
nav button.on{background:var(--bg);color:var(--fg);font-weight:600}
main{max-width:900px;margin:0 auto;padding:16px}
section{display:none}section.on{display:block}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:16px;margin-bottom:16px}
.card h2{font-size:15px;margin:0 0 12px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:12px}
.v{font-size:22px;font-weight:600;font-variant-numeric:tabular-nums}.k{color:var(--muted);font-size:13px}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:var(--muted)}
.dot.ok{background:var(--ok)}.dot.bad{background:var(--bad)}.dot.warn{background:var(--warn)}
button.b{border:1px solid var(--line);background:var(--card);color:var(--fg);padding:8px 14px;border-radius:8px;font:inherit;cursor:pointer}
button.p{background:var(--accent);border-color:var(--accent);color:#fff}
button.d{background:var(--bad);border-color:var(--bad);color:#fff}
.row{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin:6px 0}
label{display:block;margin:10px 0 4px;color:var(--muted);font-size:13px}
input,select{width:100%;max-width:320px;padding:8px;border:1px solid var(--line);border-radius:8px;background:var(--bg);color:var(--fg);font:inherit}
input[type=checkbox]{width:auto}
table{width:100%;border-collapse:collapse}td,th{text-align:left;padding:8px 6px;border-bottom:1px solid var(--line);vertical-align:middle}
th{color:var(--muted);font-weight:500;font-size:13px}
.cv input{width:80px}.muted{color:var(--muted)}.small{font-size:13px}
#log{font:12px/1.5 ui-monospace,monospace;max-height:240px;overflow:auto;white-space:pre-wrap}
#toast{position:fixed;bottom:16px;left:50%;transform:translateX(-50%);background:var(--fg);color:var(--bg);padding:10px 16px;border-radius:8px;display:none}
</style>
</head>
<body>
<header><span class="dot" id="linkDot"></span><h1>MobaStation</h1><span class="small muted" id="netInfo"></span></header>
<nav id="nav">
<button data-s="status" class="on">Status</button><button data-s="station">Zentrale</button>
<button data-s="mobabus">MobaBus</button><button data-s="network">Netzwerk</button><button data-s="system">System</button>
</nav>
<main>

<section id="status" class="on">
<div class="card"><h2>Gleis</h2>
<div class="row"><span class="dot" id="railDot"></span><span class="v" id="rail">-</span></div>
<div class="row"><button class="b p" onclick="cmd({c:'power',on:true})">Spannung an</button>
<button class="b" onclick="cmd({c:'power',on:false})">Spannung aus</button>
<button class="b d" onclick="cmd({c:'stop'})">Nothalt</button></div></div>
<div class="card"><h2>Messwerte</h2><div class="grid">
<div><div class="k">Strom Hauptgleis</div><div class="v" id="main">-</div></div>
<div><div class="k">Strom Programmiergleis</div><div class="v" id="prog">-</div></div>
<div><div class="k">Spannung</div><div class="v" id="volt">-</div></div>
<div><div class="k">Z21-Clients</div><div class="v" id="clients">-</div></div></div></div>
<div class="card"><h2>Zentrale</h2><div class="grid">
<div><div class="k">Durchlaufzeit</div><div class="v" id="loop">-</div></div>
<div><div class="k">Freier RAM (min.)</div><div class="v" id="ram">-</div></div>
<div><div class="k">Laufzeit</div><div class="v" id="uptime">-</div></div>
<div><div class="k">Link-Fehler Mega / ESP</div><div class="v" id="linkErr">-</div></div>
<div><div class="k">Z21-Pakete rein / raus</div><div class="v" id="udp">-</div></div></div></div>
</section>

<section id="station">
<div class="card"><h2>Einstellungen der Zentrale</h2>
<p class="small muted">Gespeichert im EEPROM des Mega, sie gelten statt MobaStationConfig.h und Sketch.</p>
<label>Weichen</label><select id="s1"><option value="0">Nur MobaBus</option><option value="1">Nur DCC</option><option value="2">Kombiniert</option></select>
<label>Kombiniert: DCC bis Adresse, darüber MobaBus</label><input id="s2" type="number" min="0" max="2047">
<label>Spannungsfaktor (Messwert &times; Faktor = mV)</label><input id="s3" type="number" step="0.001" min="0.001">
<label><input id="s4" type="checkbox"> Gleisspannung nach dem Start einschalten</label>
<div class="row"><button class="b p" onclick="saveSettings()">Speichern</button>
<button class="b" onclick="cmd({c:'resetSettings'});toast('Gilt nach dem n�chsten Neustart der Zentrale')">Auf Werte der Config zur�cksetzen</button>
<span class="small muted" id="sMsg"></span></div></div>
<div class="card"><h2>Weiche schalten</h2>
<div class="row"><input id="tAddr" type="number" min="1" max="2048" placeholder="Adresse" style="max-width:140px">
<button class="b" onclick="turnout(0)">Gerade / Aus</button><button class="b" onclick="turnout(1)">Abzweig / An</button></div></div>
</section>

<section id="mobabus">
<div class="card"><h2>Module</h2>
<div class="row"><button class="b p" onclick="discover()">Module suchen</button><span class="small muted" id="mMsg"></span></div>
<table><thead><tr><th>Adresse</th><th>Modul</th><th>Slot</th><th></th></tr></thead><tbody id="mods"></tbody></table></div>
<div class="card" id="cvCard" style="display:none"><h2 id="cvTitle">CVs</h2>
<p class="small muted">Geänderte Werte gelten sofort und auch nach einem Neustart, bis &bdquo;Auf Sketch-Werte zurücksetzen&ldquo;.</p>
<table><thead><tr><th>CV</th><th>Bedeutung</th><th>Wert</th><th></th></tr></thead><tbody id="cvs"></tbody></table>
<div class="row"><button class="b" onclick="readAll()">Alle lesen</button><button class="b" onclick="resetCvs()">Auf Sketch-Werte zurücksetzen</button></div>
<label>Adresse ändern</label><div class="row"><input id="newAddr" type="number" min="1" max="16383" style="max-width:140px">
<button class="b" onclick="changeAddr()">Übernehmen</button></div></div>
</section>

<section id="network">
<div class="card"><h2>WLAN</h2>
<p class="small muted" id="nSource"></p>
<label>Betriebsart</label><select id="nMode"><option value="2">Router, eigener Access Point als Ersatz</option><option value="0">Nur Router</option><option value="1">Nur eigener Access Point</option></select>
<label>WLAN-Name (Router)</label><input id="nSsid" autocomplete="off">
<label>WLAN-Passwort (leer = unverändert)</label><input id="nPwd" type="password" autocomplete="new-password">
<label><input id="nDhcp" type="checkbox"> Adresse vom Router (DHCP)</label>
<label>IP-Adresse (auch Adresse des Access Points)</label><input id="nIp">
<label>Subnetzmaske</label><input id="nSubnet">
<label>Gateway</label><input id="nGw">
<label>Name eigener Access Point</label><input id="nApSsid">
<label>Passwort Access Point (min. 8 Zeichen, leer = unverändert)</label><input id="nApPwd" type="password" autocomplete="new-password">
<div class="row"><button class="b p" onclick="saveNet()">Speichern und neu verbinden</button>
<button class="b" onclick="resetNet()">Einstellungen der Zentrale verwenden</button></div>
<p class="small muted" id="nMsg"></p></div>
</section>

<section id="system">
<div class="card"><h2>ESP-Firmware</h2><p class="small" id="fw">-</p>
<label>Firmware-Datei (.bin)</label><input id="fwFile" type="file" accept=".bin">
<div class="row"><button class="b p" onclick="upload()">Aktualisieren</button><button class="b" onclick="restart()">ESP neu starten</button></div>
<p class="small muted" id="fwMsg"></p></div>
<div class="card"><h2>Protokoll</h2><div id="log"></div></div>
</section>
</main>
<div id="toast"></div>
<script>
const $=id=>document.getElementById(id);
const RAIL={0:['Spannung an','ok'],1:['Nothalt','warn'],2:['Spannung aus','bad'],4:['Kurzschluss','bad'],8:['Programmieren','warn']};
const TYPES={1:'Ausgang',2:'Doppelspule',3:'Servo',4:'16 Servos (PCA9685)',5:'Belebtes Haus',6:'Kerze / Feuer',7:'Blinklicht',100:'Rückmelder (8 Eingänge)'};
const SERVO={33:'Winkel Stellung 0',34:'Winkel Stellung 1',35:'Geschwindigkeit (0-255)',36:'Abschaltzeit (x10 ms)'};
const CVS={2:{33:'Abschaltzeit (x10 ms)'},3:SERVO,4:SERVO,
 5:{33:'An minimal (x10 s)',34:'An maximal (x10 s)',35:'Aus minimal (x10 s)',36:'Aus maximal (x10 s)',37:'Einschaltzeit (x10 s)',38:'Ausschaltzeit (x10 s)'},
 6:{33:'Helligkeit (0-255)',34:'Modus (0 Kerze, 1 Feuer)'},7:{33:'Periode (x10 ms)',34:'Überblenden (x10 ms)'}};
const BASE={7:'Version',8:'Hersteller (13 = DIY)',30:'Modultyp',31:'Anzahl Adressen'};
const STATUS={0:'',1:'unbekannte CV',2:'nur lesbar',3:'ungültiger Wert',254:'keine MobaBus-Adresse',255:'keine Antwort'};
let ws,mods={},cur=null,pending={};
function toast(t){const e=$('toast');e.textContent=t;e.style.display='block';clearTimeout(e._t);e._t=setTimeout(()=>e.style.display='none',3000)}
function log(t){const l=$('log');l.textContent+=new Date().toLocaleTimeString()+'  '+t+'\n';l.scrollTop=l.scrollHeight}
function cmd(o){if(ws&&ws.readyState==1)ws.send(JSON.stringify(o));else toast('Keine Verbindung')}
document.querySelectorAll('#nav button').forEach(b=>b.onclick=()=>{
 document.querySelectorAll('#nav button').forEach(x=>x.classList.toggle('on',x==b));
 document.querySelectorAll('section').forEach(s=>s.classList.toggle('on',s.id==b.dataset.s));
 if(b.dataset.s=='station')cmd({c:'settings'});if(b.dataset.s=='network')loadNet()});
function connect(){
 ws=new WebSocket('ws://'+location.host+'/ws');
 ws.onopen=()=>{log('Verbunden');cmd({c:'settings'})};
 ws.onclose=()=>{$('linkDot').className='dot bad';setTimeout(connect,2000)};
 ws.onmessage=e=>{const m=JSON.parse(e.data);(H[m.t]||(()=>{}))(m)}}
const fmtUp=s=>{const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h+' h '+m+' min'};
const H={
 status:m=>{$('linkDot').className='dot '+(m.link?'ok':'bad');
  if(m.rail!==undefined){const r=RAIL[m.rail]||['Status '+m.rail,'warn'];$('rail').textContent=r[0];$('railDot').className='dot '+r[1];
   $('main').textContent=m.main+' mA';$('prog').textContent=m.prog+' mA';$('volt').textContent=(m.volt/1000).toFixed(1)+' V';
   $('clients').textContent=m.clients;$('loop').textContent=m.loop+' µs';$('ram').textContent=m.ram+' B';$('uptime').textContent=fmtUp(m.uptime);
   $('linkErr').textContent=m.megaErr+' / '+m.espErr}else{$('rail').textContent=m.link?'-':'Zentrale nicht verbunden'}
  $('udp').textContent=m.udpIn+' / '+m.udpOut},
 net:m=>{$('netInfo').textContent={station:'WLAN ',ap:'Access Point ','station+ap':'WLAN + AP ',down:'kein WLAN '}[m.mode]+m.ip+(m.rssi?' ('+m.rssi+' dBm)':'');
  $('fw').textContent='Version '+m.version+', freier Speicher '+m.heap+' B'},
 setting:m=>{const e=$('s'+m.key);if(!e)return;
  if(m.key==3)e.value=(m.value/1000).toFixed(3);else if(m.key==4)e.checked=!!m.value;else e.value=m.value;
  if(!m.ok)$('sMsg').textContent='Wert abgelehnt'},
 module:m=>{mods[m.slot+':'+m.addr+':'+m.typeId]=m;renderMods()},
 cv:m=>{const id='cv'+m.cv;if(cur&&m.addr==cur.addr){const e=$(id);const s=$(id+'s');
   if(m.status==0&&e&&m.cv!=1&&m.cv!=9&&m.cv!=8)e.value=m.value;if(s)s.textContent=STATUS[m.status]||('Status '+m.status)}
  delete pending[m.addr+':'+m.cv]},
 log:m=>log(m.text)};
function renderMods(){const t=$('mods');t.innerHTML='';const l=Object.values(mods).sort((a,b)=>a.addr-b.addr);
 $('mMsg').textContent=l.length+' Module';
 for(const m of l){const tr=document.createElement('tr');const name=TYPES[m.typeId]||('Typ '+m.typeId);
  const addr=m.addr?m.addr+(m.channels>1?' - '+(m.addr+m.channels-1):''):'<span class="muted">ohne Adresse</span>';
  tr.innerHTML='<td>'+addr+'</td><td>'+name+'</td><td>'+m.slot+'</td><td class="row"></td>';
  const c=tr.lastChild;if(m.addr&&m.typeId!=100){
   for(const [l2,d] of [['Aus',0],['An',1]]){const b=document.createElement('button');b.className='b';b.textContent=l2;b.onclick=()=>cmd({c:'turnout',addr:m.addr,dir:d});c.append(b)}
   const b=document.createElement('button');b.className='b';b.textContent='CVs';b.onclick=()=>openCv(m);c.append(b)}
  t.append(tr)}}
function discover(){mods={};renderMods();$('mMsg').textContent='Suche...';cmd({c:'discover'})}
function openCv(m){cur=m;$('cvCard').style.display='block';$('cvTitle').textContent='CVs '+(TYPES[m.typeId]||'')+' an Adresse '+m.addr;
 $('newAddr').value=m.addr;const t=$('cvs');t.innerHTML='';const list=Object.assign({},CVS[m.typeId]||{},BASE);
 for(const cv of Object.keys(list).map(Number).sort((a,b)=>a-b)){const ro=cv in BASE;const tr=document.createElement('tr');tr.className='cv';
  tr.innerHTML='<td>'+cv+'</td><td>'+list[cv]+'</td><td><input id="cv'+cv+'" type="number" min="0" max="255"'+(ro?' disabled':'')+'></td><td><span class="small muted" id="cv'+cv+'s"></span></td>';
  if(!ro){const b=document.createElement('button');b.className='b';b.textContent='Schreiben';b.onclick=()=>writeCv(cv);tr.lastChild.prepend(b)}
  t.append(tr)}
 readAll();$('cvCard').scrollIntoView({behavior:'smooth'})}
function readAll(){if(!cur)return;document.querySelectorAll('#cvs input').forEach((e,i)=>{const cv=Number(e.id.slice(2));setTimeout(()=>cmd({c:'cv',addr:cur.addr,cv}),i*120)})}
function writeCv(cv){const v=Number($('cv'+cv).value);if(!(v>=0&&v<=255))return toast('Wert 0-255');cmd({c:'cv',addr:cur.addr,cv,value:v});setTimeout(()=>cmd({c:'cv',addr:cur.addr,cv}),300)}
function resetCvs(){cmd({c:'cv',addr:cur.addr,cv:8,value:8});toast('Zurückgesetzt');setTimeout(readAll,500);setTimeout(discover,1500)}
function changeAddr(){const a=Number($('newAddr').value);if(!(a>0&&a<16384))return toast('Adresse 1-16383');
 cmd({c:'cv',addr:cur.addr,cv:1,value:a&255});setTimeout(()=>cmd({c:'cv',addr:cur.addr,cv:9,value:a>>8}),200);
 toast('Neue Adresse '+a);setTimeout(discover,1000)}
function saveSettings(){$('sMsg').textContent='';
 cmd({c:'set',key:1,value:Number($('s1').value)});cmd({c:'set',key:2,value:Number($('s2').value)});
 cmd({c:'set',key:3,value:Math.round(Number($('s3').value)*1000)});cmd({c:'set',key:4,value:$('s4').checked?1:0});toast('Gespeichert')}
function turnout(d){const a=Number($('tAddr').value);if(a>0)cmd({c:'turnout',addr:a,dir:d})}
async function loadNet(){const r=await fetch('/api/config');const j=await r.json();const a=j.active;
 $('nMode').value=a.mode;$('nSsid').value=a.ssid;$('nDhcp').checked=a.dhcp;$('nIp').value=a.ip;$('nSubnet').value=a.subnet;$('nGw').value=a.gateway;$('nApSsid').value=a.apSsid;
 $('nSource').textContent=a.saved?'Diese Einstellungen wurden hier gespeichert und gelten statt MobaStationConfig.h.':'Aktiv sind die Einstellungen aus MobaStationConfig.h der Zentrale.'}
async function saveNet(){const b={mode:Number($('nMode').value),ssid:$('nSsid').value,pwd:$('nPwd').value,dhcp:$('nDhcp').checked,
 ip:$('nIp').value,subnet:$('nSubnet').value,gateway:$('nGw').value,apSsid:$('nApSsid').value,apPwd:$('nApPwd').value};
 const r=await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)});$('nMsg').textContent=await r.text()}
async function resetNet(){const r=await fetch('/api/config/reset',{method:'POST'});$('nMsg').textContent=await r.text()}
async function restart(){await fetch('/api/restart',{method:'POST'});toast('Neustart')}
function upload(){const f=$('fwFile').files[0];if(!f)return toast('Datei wählen');const fd=new FormData();fd.append('firmware',f);
 const x=new XMLHttpRequest();x.upload.onprogress=e=>$('fwMsg').textContent='Übertrage '+Math.round(e.loaded*100/e.total)+' %';
 x.onload=()=>$('fwMsg').textContent=x.responseText;x.onerror=()=>$('fwMsg').textContent='Fehler';x.open('POST','/update');x.send(fd)}
connect();
</script>
</body>
</html>
)=====";
