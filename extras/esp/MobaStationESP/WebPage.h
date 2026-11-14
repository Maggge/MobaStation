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
.cv input{width:72px}.cv td{padding:8px 4px}.cv button.b{padding:8px 10px}.muted{color:var(--muted)}.small{font-size:13px}
#log{font:12px/1.5 ui-monospace,monospace;max-height:240px;overflow:auto;white-space:pre-wrap}
.drive{display:flex;align-items:baseline;gap:12px;margin:8px 0}.drive .v{font-size:40px}
input[type=range]{max-width:none;padding:0;height:44px;accent-color:var(--accent)}
.fns{display:grid;grid-template-columns:repeat(auto-fill,minmax(58px,1fr));gap:6px;margin-top:12px}
.fns button{padding:10px 0}button.on{background:var(--accent);border-color:var(--accent);color:#fff}
button.big{flex:1;min-width:80px;padding:12px;white-space:nowrap}
.st{display:inline-block;min-width:64px;font-size:13px}.st.s0{color:var(--bad)}.st.s1{color:var(--ok)}
.item{display:flex;flex-wrap:wrap;align-items:center;gap:6px 10px;padding:8px 0;border-bottom:1px solid var(--line)}
.item .n{flex:1 1 150px}.item .btns{display:flex;gap:6px;margin-left:auto}.item .sub{font-size:13px;color:var(--muted)}
.ins{display:flex;flex-wrap:wrap;gap:4px}.in{width:24px;height:24px;border-radius:50%;border:1px solid var(--line);font-size:11px;display:flex;align-items:center;justify-content:center;color:var(--muted)}
.in.on{background:var(--ok);border-color:var(--ok);color:var(--card)}
.bits{display:flex;flex-wrap:wrap;gap:4px 12px}.cv .bits input{width:auto;margin:0}.bits label{display:inline-flex;align-items:center;gap:4px;margin:0;color:var(--fg);font-size:14px}
#toast{position:fixed;bottom:16px;left:50%;transform:translateX(-50%);background:var(--fg);color:var(--bg);padding:10px 16px;border-radius:8px;display:none}
</style>
</head>
<body>
<header><span class="dot" id="linkDot"></span><h1>MobaStation</h1><span class="small muted" id="netInfo"></span></header>
<nav id="nav">
<button data-s="status" class="on">Status</button><button data-s="drive">Steuern</button><button data-s="prog">Programmieren</button><button data-s="station">Zentrale</button>
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

<section id="prog">
<div class="card"><h2>Programmiergleis</h2>
<p class="small muted">Gelesen und geschrieben wird der Decoder auf dem Programmiergleis, dort darf nur eine Lok stehen. Jeder Zugriff dauert einige Sekunden.</p>
<div class="row"><span class="k">Lokadresse</span><span class="v" id="pAddr">-</span><button class="b" onclick="readAddr()">Adresse lesen</button></div>
<div class="row"><input id="pNewAddr" type="number" min="1" max="9999" placeholder="Neue Adresse" style="max-width:140px">
<button class="b" onclick="writeAddr()">Adresse schreiben</button></div>
<div class="row"><span class="small" id="pMsg"></span></div>
<div id="pCvs"></div>
<p class="small muted" id="p29"></p>
<div class="row"><button class="b" onclick="readCommon()">Alle lesen</button>
<button class="b d" onclick="factoryReset()">Werkseinstellung (CV8 = 8)</button></div>
<label>Beliebige CV</label><div class="row cv"><input id="pCv" type="number" min="1" max="1024" placeholder="CV">
<input id="pVal" type="number" min="0" max="255" placeholder="Wert">
<button class="b" onclick="progFree(false)">Lesen</button><button class="b" onclick="progFree(true)">Schreiben</button></div></div>
<div class="card"><h2>Hauptgleis (POM)</h2>
<p class="small muted">Schreibt eine CV der Lok mit dieser Adresse während der Fahrt. Lesen ist auf dem Hauptgleis nicht möglich (das bräuchte RailCom), der Decoder bestätigt nichts.</p>
<div class="row cv"><input id="mAddr" type="number" min="1" max="9999" placeholder="Lok" style="width:100px">
<input id="mCv" type="number" min="1" max="1024" placeholder="CV"><input id="mVal" type="number" min="0" max="255" placeholder="Wert">
<button class="b" onclick="pomWrite()">Schreiben</button></div>
<div class="row"><span class="small" id="mMsgPom"></span></div></div>
</section>

<section id="station">
<div class="card"><h2>Einstellungen der Zentrale</h2>
<p class="small muted">Gespeichert im EEPROM des Mega, sie gelten statt MobaStationConfig.h und Sketch.</p>
<label>Weichen</label><select id="s1"><option value="0">Nur MobaBus</option><option value="1">Nur DCC</option><option value="2">Kombiniert</option></select>
<label>Kombiniert: DCC bis Adresse, darüber MobaBus</label><input id="s2" type="number" min="0" max="2047">
<label>Spannungsfaktor (Messwert &times; Faktor = mV)</label><input id="s3" type="number" step="0.001" min="0.001">
<label><input id="s4" type="checkbox"> Gleisspannung nach dem Start einschalten</label>
<div class="row"><button class="b p" onclick="saveSettings()">Speichern</button>
<button class="b" onclick="cmd({c:'resetSettings'});toast('Gilt nach dem nächsten Neustart der Zentrale')">Auf Werte der Config zurücksetzen</button>
<span class="small muted" id="sMsg"></span></div></div>
</section>

<section id="drive">
<div class="card"><h2>Lok</h2>
<div class="row"><select id="locoSel" style="max-width:220px" onchange="if(this.value)selectLoco(Number(this.value))"></select>
<input id="locoAddr" type="number" min="1" max="9999" placeholder="Adresse" style="max-width:110px">
<button class="b" onclick="selectLoco(Number($('locoAddr').value))">Wählen</button>
<button class="b" onclick="saveLoco()">Merken</button><button class="b" onclick="forgetLoco()">Entfernen</button></div>
<div id="locoPanel" style="display:none">
<div class="drive"><span class="v" id="spd">0</span><span class="muted" id="spdInfo"></span><span style="flex:1"></span><span class="v" id="dirShow"></span></div>
<input id="throttle" type="range" min="0" max="126" value="0">
<div class="row"><button class="b big" onclick="setDir(0)">&#9664; Zurück</button><button class="b big" onclick="stopLoco()">Halt</button>
<button class="b big" onclick="setDir(1)">Vor &#9654;</button><button class="b d big" onclick="estopLoco()">Nothalt Lok</button></div>
<div class="fns" id="fns"></div></div></div>
<div class="card"><h2>Weichen und Zubehör</h2>
<div id="items"></div>
<p class="small muted" id="itemsEmpty">Noch keine Einträge. Weichen und Zubehör hier unten hinzufügen.</p>
<label>Hinzufügen</label><div class="row"><input id="iName" placeholder="Name" style="max-width:180px">
<input id="iAddr" type="number" min="1" max="16383" placeholder="Adresse" style="max-width:110px">
<select id="iKind" style="max-width:150px"><option value="t">Weiche</option><option value="s">Zubehör (An/Aus)</option></select>
<button class="b" onclick="addItem()">Hinzufügen</button></div>
<label>Direkt schalten</label><div class="row"><input id="qAddr" type="number" min="1" max="16383" placeholder="Adresse" style="max-width:110px">
<button class="b" onclick="quick(0)">0 / Abzweig / Aus</button><button class="b" onclick="quick(1)">1 / Gerade / An</button>
<span class="st" id="qState"></span></div></div>
</section>

<section id="mobabus">
<div class="card"><h2>Module</h2>
<div class="row"><button class="b p" onclick="discover()">Module suchen</button><span class="small muted" id="mMsg"></span></div>
<div id="mods"></div></div>
<div class="card" id="cvCard" style="display:none"><h2 id="cvTitle">CVs</h2>
<p class="small muted">Geänderte Werte gelten sofort und auch nach einem Neustart, bis &bdquo;Auf Sketch-Werte zurücksetzen&ldquo;.</p>
<div style="overflow-x:auto"><table><thead><tr><th>CV</th><th>Bedeutung</th><th>Wert</th><th></th></tr></thead><tbody id="cvs"></tbody></table></div>
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
//a CV is a name or {n:name, bits:number of checkboxes (1 = yes/no)}
const INV={n:'Invertiert',bits:1};
const CVS={1:{33:INV},2:{33:'Abschaltzeit (x10 ms)',34:INV},3:SERVO,4:SERVO,
 5:{33:'An minimal (x10 s)',34:'An maximal (x10 s)',35:'Aus minimal (x10 s)',36:'Aus maximal (x10 s)',37:'Einschaltzeit (x10 s)',38:'Ausschaltzeit (x10 s)',
  39:'Helligkeit max (0-255)',40:'Helligkeit min, dunkelste TV-Szene'},
 6:{33:'Helligkeit max (0-255)',34:'Modus (0 Kerze, 1 Feuer)',35:'Helligkeit min (0-255)'},
 7:{33:'Periode (x10 ms)',34:'Überblenden (x10 ms)',35:'Helligkeit max, an (0-255)',36:'Helligkeit min, aus (0-255)'},
 100:{33:{n:'Invertierte Eingänge',bits:8}}};
const BASE={7:'Version',8:'Hersteller (13 = DIY)',30:'Modultyp',31:'Anzahl Adressen'};
const STATUS={0:'',1:'unbekannte CV',2:'nur lesbar',3:'ungültiger Wert',254:'keine MobaBus-Adresse',255:'keine Antwort'};
let ws,mods={},cur=null,pending={},rb={},rbAsked={};
function toast(t){const e=$('toast');e.textContent=t;e.style.display='block';clearTimeout(e._t);e._t=setTimeout(()=>e.style.display='none',3000)}
function log(t){const l=$('log');l.textContent+=new Date().toLocaleTimeString()+'  '+t+'\n';l.scrollTop=l.scrollHeight}
const online=()=>ws&&ws.readyState==1;
function cmd(o){if(online())ws.send(JSON.stringify(o));else toast('Keine Verbindung')}
document.querySelectorAll('#nav button').forEach(b=>b.onclick=()=>{
 history.replaceState(null,'','#/'+b.dataset.s);
 document.querySelectorAll('#nav button').forEach(x=>x.classList.toggle('on',x==b));
 document.querySelectorAll('section').forEach(s=>s.classList.toggle('on',s.id==b.dataset.s));
 if(b.dataset.s=='station')cmd({c:'settings'});if(b.dataset.s=='network')loadNet();if(b.dataset.s=='drive')refreshDrive();if(b.dataset.s=='prog')renderProg();
 if(b.dataset.s=='mobabus'&&online()&&!Object.keys(mods).length)discover()});
function connect(){
 ws=new WebSocket('ws://'+location.host+'/ws');
 ws.onopen=()=>{log('Verbunden');cmd({c:'settings'});refreshDrive();if($('mobabus').classList.contains('on')&&!Object.keys(mods).length)discover()};
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
 module:m=>{mods[m.slot+':'+m.addr+':'+m.typeId]=m;renderMods();if(!m.addr)return;
  if(m.typeId==100){const g=Math.floor((m.addr-1)/10);if(!rbAsked[g]){rbAsked[g]=1;cmd({c:'rbus',group:g})}}else askTurnout(m.addr)},
 rbus:m=>{rb[m.group]=m.data;renderInputs()},
 cv:m=>{const id='cv'+m.cv;if(cur&&m.addr==cur.addr&&!!m.fb==(cur.typeId==100)){const e=$(id);const s=$(id+'s');
   if(m.status==0&&e&&m.cv!=1&&m.cv!=9&&!(m.cv==8&&m.value==8)){e.value=m.value;syncBits(m.cv)}if(s)s.textContent=m.status in STATUS?STATUS[m.status]:'Status '+m.status}
  delete pending[m.addr+':'+m.cv]},
 loco:m=>{if(m.addr!=loco.addr)return;loco.steps=m.steps;loco.fn=m.fn;const s=decodeSpeed(m.steps,m.speed);loco.dir=s[1];
  if(Date.now()-userT>800){loco.speed=s[0];$('throttle').value=s[0]}renderLoco()},
 turnout:m=>{tstate[m.addr]=m.state;renderStates();renderModStates()},
 prog:m=>progDone(m),
 log:m=>log(m.text)};
function el(tag,cls,text){const e=document.createElement(tag);if(cls)e.className=cls;if(text!==undefined)e.textContent=text;return e}
function renderMods(){const t=$('mods');t.innerHTML='';
 const l=Object.values(mods).sort((a,b)=>(a.typeId==100)-(b.typeId==100)||a.addr-b.addr);
 $('mMsg').textContent=l.length+' Module';
 for(const m of l){const d=el('div','item'),n=el('div','n'),fb=m.typeId==100;
  n.append(el('b','',TYPES[m.typeId]||('Typ '+m.typeId)),
   el('div','sub',(m.addr?(fb?'Rückmelder ':'Adresse ')+m.addr+(m.channels>1?' - '+(m.addr+m.channels-1):''):'ohne Adresse')+' · Slot '+m.slot));
  d.append(n);
  if(fb&&m.addr){const ins=el('div','ins');ins.dataset.a=m.addr;for(let i=1;i<=8;i++)ins.append(el('span','in',i));d.append(ins)}
  const bt=el('div','btns');
  if(m.addr&&!fb)for(const [txt,v] of [['Aus',0],['An',1]]){const b=el('button','b',txt);b.dataset.a=m.addr;b.dataset.d=v;b.onclick=()=>switchItem(m.addr,v);bt.append(b)}
  if(m.addr){const b=el('button','b','CVs');b.onclick=()=>openCv(m);bt.append(b)}
  d.append(bt);t.append(d)}
 renderModStates();renderInputs()}
function renderModStates(){document.querySelectorAll('#mods button[data-d]').forEach(b=>{const s=tstate[b.dataset.a];b.classList.toggle('on',s==Number(b.dataset.d)+1)})}
function rbGet(a){const g=rb[Math.floor((a-1)/10)];return g?g[(a-1)%10]:undefined}
function renderInputs(){document.querySelectorAll('#mods .ins').forEach(x=>{const v=rbGet(Number(x.dataset.a));
 [...x.children].forEach((e,i)=>e.classList.toggle('on',v!==undefined&&!!((v>>i)&1)))})}
function discover(){mods={};rbAsked={};renderMods();$('mMsg').textContent='Suche...';cmd({c:'discover'})}
function openCv(m){cur=m;$('cvCard').style.display='block';$('cvTitle').textContent='CVs '+(TYPES[m.typeId]||'')+' an Adresse '+m.addr;
 $('newAddr').value=m.addr;const t=$('cvs');t.innerHTML='';const list=Object.assign({},CVS[m.typeId]||{},BASE);
 for(const cv of Object.keys(list).map(Number).sort((a,b)=>a-b)){const ro=cv in BASE,def=list[cv],bits=def.bits||0;
  const tr=el('tr','cv');tr.append(el('td','',cv),el('td','',def.n||def));
  const td=el('td'),inp=el('input');inp.id='cv'+cv;inp.type='number';inp.min=0;inp.max=255;inp.disabled=ro;td.append(inp);
  const st=el('td');st.append(el('span','small muted'));st.lastChild.id='cv'+cv+'s';
  if(bits){inp.style.display='none';const w=el('div','bits');w.id='cv'+cv+'b';
   for(let i=0;i<bits;i++){const lb=el('label'),c=el('input');c.type='checkbox';
    //a checkbox writes at once, the value is read back
    c.onchange=()=>{let v=0;w.querySelectorAll('input').forEach((x,k)=>{if(x.checked)v|=1<<k});inp.value=v;writeCv(cv)};
    lb.append(c,bits>1?String(i+1):'ja');w.append(lb)}
   td.append(w)}
  else if(!ro){const b=el('button','b','Schreiben');b.onclick=()=>writeCv(cv);st.prepend(b)}
  tr.append(td,st);t.append(tr)}
 readAll();$('cvCard').scrollIntoView({behavior:'smooth'})}
//CV command for the open module, feedback modules have their own addresses
function mcv(o){cmd(Object.assign({c:'cv',addr:cur.addr},cur.typeId==100?{fb:1}:{},o))}
function syncBits(cv){const w=$('cv'+cv+'b');if(!w)return;const v=Number($('cv'+cv).value);w.querySelectorAll('input').forEach((x,k)=>x.checked=!!((v>>k)&1))}
function readAll(){if(!cur)return;document.querySelectorAll('#cvs input[type=number]').forEach((e,i)=>{const cv=Number(e.id.slice(2));setTimeout(()=>mcv({cv}),i*120)})}
function writeCv(cv){const v=Number($('cv'+cv).value);if(!(v>=0&&v<=255))return toast('Wert 0-255');mcv({cv,value:v});setTimeout(()=>mcv({cv}),300)}
function resetCvs(){mcv({cv:8,value:8});toast('Zurückgesetzt');setTimeout(readAll,500);setTimeout(discover,1500)}
function changeAddr(){const a=Number($('newAddr').value);if(!(a>0&&a<16384))return toast('Adresse 1-16383');
 mcv({cv:1,value:a&255});setTimeout(()=>mcv({cv:9,value:a>>8}),200);
 toast('Neue Adresse '+a);setTimeout(discover,1000)}
function saveSettings(){$('sMsg').textContent='';
 cmd({c:'set',key:1,value:Number($('s1').value)});cmd({c:'set',key:2,value:Number($('s2').value)});
 cmd({c:'set',key:3,value:Math.round(Number($('s3').value)*1000)});cmd({c:'set',key:4,value:$('s4').checked?1:0});toast('Gespeichert')}
//---- control page: the ESP passes the X bytes as Z21 client 127.0.0.1 to the station
const STEPS={0:14,2:28,4:126};
let loco={addr:0,steps:4,speed:0,dir:1,fn:0},userT=0,sendT=0,sendTimer=null,lists={locos:[],items:[]},tstate={};
const amsb=a=>((a>>8)&0x3F)|(a>=128?0xC0:0);
const store=(k,v)=>{try{if(v===undefined)return localStorage.getItem(k);localStorage.setItem(k,v)}catch(e){}};
function decodeSpeed(st,v){const dir=v>>7&1;let s;
 if(st==0){s=v&15;s=s<=1?0:s-1}else if(st==2){s=((v&15)<<1)|((v>>4)&1);s=s<=3?0:s-3}else{s=v&127;s=s<=1?0:s-1}
 return [s,dir]}
function encodeSpeed(st,s,dir,estop){let v;
 if(estop)v=1;else if(!s)v=0;else if(st==2){const t=s+3;v=(t>>1)|((t&1)<<4)}else v=s+1;
 return v|(dir?0x80:0)}
function sendDrive(estop){const a=loco.addr;if(!a)return;
 cmd({c:'x',d:[0xE4,0x10|(loco.steps==4?3:loco.steps),amsb(a),a&255,encodeSpeed(loco.steps,loco.speed,loco.dir,estop)]})}
function drive(){const now=Date.now();clearTimeout(sendTimer);
 if(now-sendT>=150){sendT=now;sendDrive()}else sendTimer=setTimeout(drive,150-(now-sendT))}
function selectLoco(a,quiet){if(!(a>0&&a<10000))return toast('Lokadresse 1-9999');loco={addr:a,steps:4,speed:0,dir:1,fn:0};
 store('loco',a);$('locoAddr').value=a;$('locoSel').value=lists.locos.some(l=>l.addr==a)?a:'';$('throttle').value=0;
 $('locoPanel').style.display='block';renderLoco();if(!quiet)cmd({c:'x',d:[0xE3,0xF0,amsb(a),a&255]})}
function renderLoco(){const max=STEPS[loco.steps]||126,t=$('throttle');t.max=max;
 $('spd').textContent=loco.speed;$('spdInfo').textContent='von '+max+' Fahrstufen';$('dirShow').textContent=loco.dir?'▶':'◀';
 const f=$('fns');if(!f.children.length)for(let i=0;i<=28;i++){const b=document.createElement('button');b.className='b';b.textContent='F'+i;
  b.onclick=()=>{const on=!(loco.fn>>>i&1);cmd({c:'x',d:[0xE4,0xF8,amsb(loco.addr),loco.addr&255,(on?0x40:0)|i]})};f.append(b)}
 [...f.children].forEach((b,i)=>b.classList.toggle('on',!!(loco.fn>>>i&1)))}
$('throttle').oninput=e=>{userT=Date.now();loco.speed=Number(e.target.value);$('spd').textContent=loco.speed;drive()};
function setDir(d){if(d==loco.dir)return;userT=Date.now();loco.dir=d;loco.speed=0;$('throttle').value=0;renderLoco();sendDrive()}
function stopLoco(){userT=Date.now();loco.speed=0;$('throttle').value=0;renderLoco();sendDrive()}
function estopLoco(){userT=Date.now();loco.speed=0;$('throttle').value=0;renderLoco();sendDrive(true)}
async function loadLists(){try{const r=await fetch('/api/items');lists=await r.json()}catch(e){}
 lists.locos=lists.locos||[];lists.items=lists.items||[];renderLists()}
async function saveLists(){const r=await fetch('/api/items',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(lists)});
 if(!r.ok)toast(await r.text())}
function renderLists(){const s=$('locoSel');s.innerHTML='';const o0=document.createElement('option');o0.value='';o0.textContent='Lok wählen';s.append(o0);
 for(const l of lists.locos){const o=document.createElement('option');o.value=l.addr;o.textContent=l.name+' ('+l.addr+')';s.append(o)}
 if(loco.addr)s.value=lists.locos.some(l=>l.addr==loco.addr)?loco.addr:'';
 const t=$('items');t.innerHTML='';$('itemsEmpty').style.display=lists.items.length?'none':'block';
 lists.items.forEach((it,k)=>{const tr=document.createElement('div');tr.className='item';
  const n=document.createElement('div');n.className='n';n.textContent=it.name+' ';
  const a=document.createElement('span');a.className='muted small';a.textContent=it.addr;n.append(a);
  const st=document.createElement('span');st.className='st';st.dataset.a=it.addr;st.dataset.k=it.kind;
  const c=document.createElement('div');c.className='btns';const lb=it.kind=='t'?['Abzweig','Gerade']:['Aus','An'];
  lb.forEach((l,d)=>{const b=document.createElement('button');b.className='b';b.textContent=l;b.onclick=()=>switchItem(it.addr,d);c.append(b)});
  const x=document.createElement('button');x.className='b';x.textContent='✕';x.title='Entfernen';
  x.onclick=()=>{if(confirm(it.name+' entfernen?')){lists.items.splice(k,1);saveLists();renderLists()}};c.append(x);
  tr.append(n,st,c);t.append(tr)});renderStates()}
function stateText(s,k){if(s==1)return k=='t'?'Abzweig':'Aus';if(s==2)return k=='t'?'Gerade':'An';return '?'}
function renderStates(){document.querySelectorAll('#items .st').forEach(e=>{const s=tstate[e.dataset.a];e.textContent=stateText(s,e.dataset.k);e.className='st'+(s==1?' s0':s==2?' s1':'')});
 const q=Number($('qAddr').value);const qs=$('qState');qs.textContent=q>0&&tstate[q]?'Stellung '+(tstate[q]-1):'';}
function askTurnout(a){cmd({c:'x',d:[0x43,((a-1)>>8)&0x3F,(a-1)&255]})}
function switchItem(a,d){cmd({c:'turnout',addr:a,dir:d})}
function quick(d){const a=Number($('qAddr').value);if(!(a>0&&a<16384))return toast('Adresse 1-16383');switchItem(a,d)}
function addItem(){const name=$('iName').value.trim(),addr=Number($('iAddr').value),kind=$('iKind').value;
 if(!name)return toast('Name fehlt');if(!(addr>0&&addr<16384))return toast('Adresse 1-16383');
 lists.items.push({name,addr,kind});lists.items.sort((a,b)=>a.addr-b.addr);saveLists();renderLists();askTurnout(addr);$('iName').value='';$('iAddr').value=''}
function saveLoco(){const a=Number($('locoAddr').value);if(!(a>0&&a<10000))return toast('Lokadresse 1-9999');
 const name=prompt('Name der Lok',(lists.locos.find(l=>l.addr==a)||{}).name||'Lok '+a);if(!name)return;
 lists.locos=lists.locos.filter(l=>l.addr!=a);lists.locos.push({addr:a,name});lists.locos.sort((x,y)=>x.name.localeCompare(y.name));
 saveLists();renderLists();if(loco.addr!=a)selectLoco(a)}
function forgetLoco(){const a=loco.addr||Number($('locoAddr').value);if(!lists.locos.some(l=>l.addr==a))return toast('Lok ist nicht gemerkt');
 lists.locos=lists.locos.filter(l=>l.addr!=a);saveLists();renderLists()}
let listsLoaded=false;
//lists at once, the states of the loco and the turnouts once the connection is there
async function refreshDrive(){if(!listsLoaded){listsLoaded=true;await loadLists();const a=Number(store('loco'));if(a>0&&!loco.addr)selectLoco(a,true)}
 if(!online())return;
 if(loco.addr)cmd({c:'x',d:[0xE3,0xF0,amsb(loco.addr),loco.addr&255]});
 lists.items.forEach((it,i)=>setTimeout(()=>askTurnout(it.addr),i*60))}
//---- programming page: one CV operation at a time, the station answers each with result or NACK
const PCV={1:'Kurze Adresse (1-127)',2:'Anfahrspannung',3:'Beschleunigung',4:'Bremsverzögerung',5:'Höchstgeschwindigkeit',
 6:'Mittlere Geschwindigkeit',7:'Version (nur lesen)',8:'Hersteller (nur lesen)',17:'Lange Adresse, hohes Byte',18:'Lange Adresse, niedriges Byte',29:'Konfiguration'};
const MAKER={13:'Selbstbau',62:'Tams',85:'Uhlenbrock',97:'Doehler & Haass',99:'Lenz',145:'Zimo',151:'ESU',161:'Roco'};
const PERR={nack:'Keine Antwort vom Decoder',short:'Kurzschluss auf dem Programmiergleis',timeout:'Zeitüberschreitung'};
let progQ=[],progBusy=null;
function progOp(write,cv,val){return new Promise(res=>{progQ.push({write,cv,val,res});progNext()})}
function progNext(){if(progBusy||!progQ.length)return;if(!online()){progQ.forEach(o=>o.res({error:'offline'}));progQ=[];return}
 const o=progBusy=progQ.shift(),c=o.cv-1;
 $('pMsg').textContent=(o.write?'Schreibe CV '+o.cv+' = '+o.val:'Lese CV '+o.cv)+' ...';
 cmd({c:'x',d:o.write?[0x24,0x12,(c>>8)&0x3F,c&255,o.val]:[0x23,0x11,(c>>8)&0x3F,c&255]});
 o.timer=setTimeout(()=>progDone({error:'timeout'}),15000)}
function progDone(m){const o=progBusy;if(!o)return;clearTimeout(o.timer);progBusy=null;
 if(!m.error){const e=$('pv'+o.cv);if(e)e.value=m.value;const s=$('ps'+o.cv);if(s)s.textContent=o.write?'geschrieben':'gelesen';if(o.cv==29)show29(m.value);if(o.cv==8)show8(m.value)}
 else{const s=$('ps'+o.cv);if(s)s.textContent='Fehler'}
 $('pMsg').textContent=m.error?'CV '+o.cv+': '+(PERR[m.error]||m.error):'CV '+o.cv+' = '+m.value;o.res(m);progNext()}
function show29(v){const t=[v&32?'lange Adresse':'kurze Adresse',v&2?'28/128 Fahrstufen':'14 Fahrstufen',v&4?'Analogbetrieb an':'Analogbetrieb aus'];
 if(v&1)t.push('Fahrtrichtung umgekehrt');if(v&8)t.push('RailCom an');if(v&16)t.push('eigene Geschwindigkeitskennlinie');$('p29').textContent='CV29 = '+v+': '+t.join(', ')}
function show8(v){const s=$('ps8');if(s)s.textContent=MAKER[v]||'Hersteller '+v}
function renderProg(){const t=$('pCvs');if(t.children.length)return;
 for(const cv of Object.keys(PCV).map(Number)){const ro=cv==7||cv==8;const tr=document.createElement('div');tr.className='item cv';
  tr.innerHTML='<div class="n"><b>CV'+cv+'</b> '+PCV[cv]+'<div class="small muted" id="ps'+cv+'"></div></div><input id="pv'+cv+'" type="number" min="0" max="255"'+(ro?' disabled':'')+'><div class="btns"></div>';
  const c=tr.lastChild;const r=document.createElement('button');r.className='b';r.textContent='Lesen';r.onclick=()=>progOp(false,cv);c.append(r);
  if(!ro){const w=document.createElement('button');w.className='b';w.textContent='Schreiben';w.onclick=()=>progWrite(cv,$('pv'+cv).value);c.append(w)}
  t.append(tr)}
 if(loco.addr&&!$('mAddr').value)$('mAddr').value=loco.addr}
function progWrite(cv,s){const v=Number(s);if(String(s).trim()===''||!(v>=0&&v<=255))return toast('Wert 0-255');return progOp(true,cv,v)}
function readCommon(){for(const cv of Object.keys(PCV).map(Number))progOp(false,cv)}
function progFree(w){const cv=Number($('pCv').value);if(!(cv>=1&&cv<=1024))return toast('CV 1-1024');
 if(w)progWrite(cv,$('pVal').value);else progOp(false,cv).then(m=>{if(!m.error)$('pVal').value=m.value})}
async function readAddr(){$('pAddr').textContent='...';const r29=await progOp(false,29);if(r29.error)return $('pAddr').textContent='-';
 let a;if(r29.value&32){const h=await progOp(false,17);if(h.error)return $('pAddr').textContent='-';const l=await progOp(false,18);if(l.error)return $('pAddr').textContent='-';a=((h.value&0x3F)<<8)|l.value}
 else{const r=await progOp(false,1);if(r.error)return $('pAddr').textContent='-';a=r.value}
 $('pAddr').textContent=a;$('pMsg').textContent='Lokadresse '+a+(r29.value&32?' (lang)':' (kurz)')}
async function writeAddr(){const a=Number($('pNewAddr').value);if(!(a>=1&&a<=9999))return toast('Adresse 1-9999');
 const r29=await progOp(false,29);if(r29.error)return;let r;
 if(a<128){r=await progOp(true,1,a);if(r.error)return;r=await progOp(true,29,r29.value&~32)}
 else{r=await progOp(true,17,0xC0|(a>>8));if(r.error)return;r=await progOp(true,18,a&255);if(r.error)return;r=await progOp(true,29,r29.value|32)}
 if(!r.error){$('pAddr').textContent=a;$('pMsg').textContent='Lokadresse '+a+' geschrieben'}}
function factoryReset(){if(confirm('Decoder auf dem Programmiergleis auf Werkseinstellung zurücksetzen?'))progOp(true,8,8)}
function pomWrite(){const a=Number($('mAddr').value),cv=Number($('mCv').value),v=Number($('mVal').value);
 if(!(a>=1&&a<=9999))return toast('Lokadresse 1-9999');if(!(cv>=1&&cv<=1024))return toast('CV 1-1024');if($('mVal').value===''||!(v>=0&&v<=255))return toast('Wert 0-255');
 const c=cv-1;cmd({c:'x',d:[0xE6,0x30,amsb(a),a&255,0xEC|((c>>8)&3),c&255,v]});$('mMsgPom').textContent='Lok '+a+': CV '+cv+' = '+v+' gesendet'}
$('qAddr').oninput=()=>{const a=Number($('qAddr').value);renderStates();if(a>0&&a<16384&&tstate[a]===undefined)askTurnout(a)};
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
//the tab in the address (eg. /#/drive) opens directly, for a bookmark on the phone
{const b=document.querySelector('#nav button[data-s="'+location.hash.slice(2).replace(/[^a-z]/g,'')+'"]');if(b)b.click()}
</script>
</body>
</html>
)=====";
