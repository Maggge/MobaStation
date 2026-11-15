/*
  Web interface of the MobaStation ESP firmware: one page, live data over the WebSocket /ws,
  network settings over /api/config, firmware update over /update. German is the source of every
  text, the table EN in the script lays English over it (browser language, switch on the page System).
*/
#pragma once

#include <Arduino.h>

static const char WEB_PAGE[] PROGMEM = R"=====(<!doctype html>
<html lang="de">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MobaStation</title>
<link rel="icon" href="data:,">
<style>
:root{--bg:#f4f5f7;--card:#fff;--fg:#1d2330;--muted:#6b7280;--line:#e2e5ea;--accent:#2563eb;--ok:#15803d;--warn:#b45309;--bad:#b91c1c;--warnbg:#fef3c7;--warnfg:#78350f}
@media (prefers-color-scheme:dark){:root{--bg:#14171c;--card:#1d2129;--fg:#e6e8ec;--muted:#9aa1ad;--line:#2d333d;--accent:#60a5fa;--ok:#4ade80;--warn:#fbbf24;--bad:#f87171;--warnbg:#3b2a07;--warnfg:#fde68a}}
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
.grp{border:1px solid var(--line);border-radius:10px;margin:12px 0;overflow:hidden}
.gh{display:flex;flex-wrap:wrap;align-items:center;gap:6px 10px;padding:10px 12px;background:var(--bg);border-bottom:1px solid var(--line)}
.gh .n{flex:1 1 180px}.gh .sub{font-size:13px;color:var(--muted)}.grp .item{padding:8px 12px}.grp .item:last-child{border-bottom:0}
.grp .empty{padding:10px 12px;font-size:13px;color:var(--muted)}
.gh .sub.err{color:var(--bad)}button:disabled{opacity:.45;cursor:default}
.modal{position:fixed;inset:0;background:rgba(0,0,0,.45);display:none;justify-content:center;align-items:flex-start;padding:32px 12px;overflow-y:auto;z-index:10}
.modal.open{display:flex}body.noscroll{overflow:hidden}
.dlg{background:var(--card);border-radius:12px;width:100%;max-width:760px;box-shadow:0 12px 40px rgba(0,0,0,.35);display:flex;flex-direction:column}
.dlg .dh{display:flex;align-items:center;gap:8px;padding:12px 8px 12px 16px;border-bottom:1px solid var(--line);position:sticky;top:-32px;background:var(--card);border-radius:12px 12px 0 0;z-index:1}
.dlg .dh h2{flex:1;margin:0;font-size:16px}.dlg .x{border:0;background:none;color:var(--muted);font-size:26px;line-height:1;width:40px;height:40px;cursor:pointer;border-radius:8px}
.dlg .x:hover{background:var(--bg)}.dlg .db{padding:12px 16px}
.dlg .df{display:flex;flex-wrap:wrap;gap:8px;align-items:center;padding:12px 16px;border-top:1px solid var(--line);position:sticky;bottom:-32px;background:var(--card);border-radius:0 0 12px 12px}
.dlg .df .sp{flex:1}.dlg .df .end{display:flex;gap:8px;margin-left:auto}
@media (max-width:640px){.modal{padding:0}.dlg{border-radius:0;min-height:100%}.dlg .dh{top:0;border-radius:0}.dlg .df{bottom:0;border-radius:0}}
.slot{padding:10px 0;border-bottom:1px solid var(--line)}.slot .no{display:inline-flex;width:24px;height:24px;border-radius:50%;background:var(--bg);border:1px solid var(--line);align-items:center;justify-content:center;font-size:12px;font-weight:600}.slot select,.slot input{width:auto;max-width:none}.slot .row{margin:4px 0}
.slot .fn{min-width:170px}.slot .ad{width:96px}.ios{display:flex;flex-wrap:wrap;gap:6px 10px}.ios .io{display:flex;align-items:center;gap:4px}
.ios label,.slot .opt{display:inline-flex;align-items:center;gap:4px;margin:0;color:var(--fg);font-size:13px}.warn{color:var(--warn)}.err{color:var(--bad)}
.bits{display:flex;flex-wrap:wrap;gap:4px 12px}.cv .bits input{width:auto;margin:0}.bits label{display:inline-flex;align-items:center;gap:4px;margin:0;color:var(--fg);font-size:14px}
.banner{display:none;align-items:center;flex-wrap:wrap;gap:8px 12px;margin:0 0 16px;padding:12px 16px;background:var(--warnbg);color:var(--warnfg);border:1px solid var(--warn);border-left-width:6px;border-radius:10px}
.banner.on{display:flex}.banner .t{flex:1 1 260px}.banner b{display:block;margin-bottom:2px}.banner button{white-space:nowrap}
input.attn{border:2px solid var(--warn);background:var(--warnbg);color:var(--warnfg)}
.hint{margin:6px 0 0;padding:8px 10px;max-width:320px;border-radius:8px;background:var(--warnbg);color:var(--warnfg);font-size:13px;display:none}.hint.on{display:block}
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
<div class="banner" id="pwWarn" role="alert"><span class="t"><b>&#9888; Standard-Passwort aktiv</b>Der Access Point nutzt noch das öffentliche Passwort „MobaStation“. Jeder in Funkreichweite kann sich verbinden und die Zentrale steuern.</span><button class="b p" onclick="fixPw()">Passwort ändern</button></div>

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
<p class="small muted">Gespeichert im EEPROM des Mega.</p>
<label>Weichen</label><select id="s1"><option value="0">Nur MobaBus</option><option value="1">Nur DCC</option><option value="2">Kombiniert</option></select>
<label>Kombiniert: DCC bis Adresse, darüber MobaBus</label><input id="s2" type="number" min="0" max="2047">
<label>Spannungsfaktor (Messwert &times; Faktor = mV)</label><input id="s3" type="number" step="0.001" min="0.001">
<label><input id="s4" type="checkbox"> Gleisspannung nach dem Start einschalten</label>
<div class="row"><button class="b p" onclick="saveSettings()">Speichern</button>
<button class="b" onclick="cmd({c:'resetSettings'});toast(L('Gilt nach dem nächsten Neustart der Zentrale'))">Auf Standardwerte zurücksetzen</button>
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
<div class="card"><div class="row"><h2 style="flex:1;margin:0">Module</h2><span class="small muted" id="mMsg"></span>
<button class="b p" onclick="discover()">Module suchen</button></div>
<div id="mods"></div></div>
</section>
<div class="modal" id="cvCard" onclick="if(event.target==this)closeDlg('cvCard')"><div class="dlg">
<div class="dh"><h2 id="cvTitle">CVs</h2><button class="x" onclick="closeDlg('cvCard')" title="Schließen">&times;</button></div>
<div class="db"><p class="small muted" id="cvInfo">Geänderte Werte gelten sofort und auch nach einem Neustart.</p>
<div style="overflow-x:auto"><table><thead><tr><th>CV</th><th>Bedeutung</th><th>Wert</th><th></th></tr></thead><tbody id="cvs"></tbody></table></div>
<div id="addrBox"><label>Adresse ändern</label><div class="row"><input id="newAddr" type="number" min="1" max="16383" style="max-width:140px">
<button class="b" onclick="changeAddr()">Übernehmen</button></div></div></div>
<div class="df"><button class="b" onclick="readAll()">Alle lesen</button><button class="b" id="cvReset" onclick="resetCvs()">Auf Sketch-Werte zurücksetzen</button>
<span class="sp"></span><button class="b" onclick="closeDlg('cvCard')">Schließen</button></div></div></div>
<div class="modal" id="bdCard" onclick="if(event.target==this)closeBoard()"><div class="dlg">
<div class="dh"><h2 id="bdTitle">Board</h2><button class="x" onclick="closeBoard()" title="Schließen">&times;</button></div>
<div class="db"><p class="small muted">Welche Funktion an welchem Anschluss hängt und unter welcher Adresse sie geschaltet wird. Beim Speichern baut das Board seine Module sofort neu auf. Helligkeit, Zeiten und Winkel stellst du danach in den CVs der Module ein.</p>
<div id="bdBody"></div></div>
<div class="df"><button class="b d" onclick="clearBoard()">Board leeren</button><span class="small" id="bdMsg" style="flex:1;text-align:right"></span>
<span class="end"><button class="b" onclick="openBoard(bd.id)">Neu laden</button><button class="b p" id="bdSave" onclick="saveBoard()">Speichern</button></span></div></div></div>

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
<p class="hint" id="nApHint">&#9888; Noch das Standard-Passwort „MobaStation“: hier ein eigenes eintragen und speichern.</p>
<div class="row"><button class="b p" onclick="saveNet()">Speichern und neu verbinden</button>
<button class="b" onclick="resetNet()">Netzwerk zurücksetzen</button></div>
<p class="small muted" id="nMsg"></p></div>
</section>

<section id="system">
<div class="card"><h2>ESP-Firmware</h2><p class="small" id="fw">-</p>
<label>Firmware-Datei (.bin)</label><input id="fwFile" type="file" accept=".bin">
<div class="row"><button class="b p" onclick="upload()">Aktualisieren</button><button class="b" onclick="restart()">ESP neu starten</button></div>
<p class="small muted" id="fwMsg"></p></div>
<div class="card"><h2>Sprache · Language</h2><select id="lang" onchange="setLang(this.value)"><option value="">Automatisch (Browser)</option><option value="de">Deutsch</option><option value="en">English</option></select></div>
<div class="card"><h2>Protokoll</h2><div id="log"></div></div>
</section>
</main>
<div id="toast"></div>
<script>
const $=id=>document.getElementById(id);
//---- language: German is the source, EN lays English over it (German text -> English, {} = a value).
//Static texts of the page are replaced once at the start, the scripts call L(). Auto = browser language.
let langSet='';try{langSet=localStorage.getItem('lang')||''}catch(e){}
const LANG=langSet||(/^de/i.test(navigator.language||'')?'de':'en');
const EN={
'Steuern':'Control','Programmieren':'Programming','Zentrale':'Station','Netzwerk':'Network',
'⚠ Standard-Passwort aktiv':'⚠ Default password active',
'Der Access Point nutzt noch das öffentliche Passwort „MobaStation“. Jeder in Funkreichweite kann sich verbinden und die Zentrale steuern.':'The access point still uses the public password "MobaStation". Anyone in WiFi range can connect and control the station.',
'Passwort ändern':'Change password',
'Gleis':'Track','Spannung an':'Power on','Spannung aus':'Power off','Nothalt':'Emergency stop','Kurzschluss':'Short circuit',
'Messwerte':'Measurements','Strom Hauptgleis':'Current main track','Strom Programmiergleis':'Current programming track','Spannung':'Voltage',
'Z21-Clients':'Z21 clients','Durchlaufzeit':'Loop time','Freier RAM (min.)':'Free RAM (min.)','Laufzeit':'Uptime',
'Link-Fehler Mega / ESP':'Link errors Mega / ESP','Z21-Pakete rein / raus':'Z21 packets in / out',
'Programmiergleis':'Programming track',
'Gelesen und geschrieben wird der Decoder auf dem Programmiergleis, dort darf nur eine Lok stehen. Jeder Zugriff dauert einige Sekunden.':'The decoder on the programming track is read and written, only one loco may stand there. Each access takes a few seconds.',
'Lokadresse':'Loco address','Adresse lesen':'Read address','Neue Adresse':'New address','Adresse schreiben':'Write address','Alle lesen':'Read all',
'Werkseinstellung (CV8 = 8)':'Factory reset (CV8 = 8)','Beliebige CV':'Any CV','Wert':'Value','Lesen':'Read','Schreiben':'Write',
'Hauptgleis (POM)':'Main track (POM)',
'Schreibt eine CV der Lok mit dieser Adresse während der Fahrt. Lesen ist auf dem Hauptgleis nicht möglich (das bräuchte RailCom), der Decoder bestätigt nichts.':'Writes a CV of the loco with this address while it runs. Reading is not possible on the main track (that would need RailCom), the decoder confirms nothing.',
'Lok':'Loco','Einstellungen der Zentrale':'Station settings','Gespeichert im EEPROM des Mega.':'Stored in the EEPROM of the Mega.',
'Weichen':'Turnouts','Nur MobaBus':'MobaBus only','Nur DCC':'DCC only','Kombiniert':'Combined',
'Kombiniert: DCC bis Adresse, darüber MobaBus':'Combined: DCC up to address, MobaBus above',
'Spannungsfaktor (Messwert × Faktor = mV)':'Voltage factor (reading × factor = mV)',
'Gleisspannung nach dem Start einschalten':'Switch on track power after the start','Speichern':'Save',
'Auf Standardwerte zurücksetzen':'Reset to defaults','Gilt nach dem nächsten Neustart der Zentrale':'Takes effect after the next restart of the station',
'Adresse':'Address','Wählen':'Select','Merken':'Save','Entfernen':'Remove','◀ Zurück':'◀ Reverse','Halt':'Stop','Vor ▶':'Forward ▶',
'Nothalt Lok':'Emergency stop loco','Weichen und Zubehör':'Turnouts and accessories',
'Noch keine Einträge. Weichen und Zubehör hier unten hinzufügen.':'No entries yet. Add turnouts and accessories below.',
'Hinzufügen':'Add','Weiche':'Turnout','Zubehör (An/Aus)':'Accessory (on/off)','Direkt schalten':'Switch directly',
'0 / Abzweig / Aus':'0 / Diverging / Off','1 / Gerade / An':'1 / Straight / On',
'Module':'Modules','Module suchen':'Search modules','Schließen':'Close',
'Geänderte Werte gelten sofort und auch nach einem Neustart.':'Changed values apply at once and also after a restart.',
'Bedeutung':'Meaning','Adresse ändern':'Change address','Übernehmen':'Apply','Auf Sketch-Werte zurücksetzen':'Reset to sketch values',
'Welche Funktion an welchem Anschluss hängt und unter welcher Adresse sie geschaltet wird. Beim Speichern baut das Board seine Module sofort neu auf. Helligkeit, Zeiten und Winkel stellst du danach in den CVs der Module ein.':'Which function is on which pin and which address switches it. Saving rebuilds the modules of the board at once. Brightness, times and angles are set afterwards in the CVs of the modules.',
'Board leeren':'Clear board','Neu laden':'Reload',
'WLAN':'WiFi','Betriebsart':'Mode','Router, eigener Access Point als Ersatz':'Router, own access point as fallback','Nur Router':'Router only',
'Nur eigener Access Point':'Own access point only','WLAN-Name (Router)':'WiFi name (router)','WLAN-Passwort (leer = unverändert)':'WiFi password (empty = unchanged)',
'Adresse vom Router (DHCP)':'Address from the router (DHCP)','IP-Adresse (auch Adresse des Access Points)':'IP address (also the address of the access point)',
'Subnetzmaske':'Subnet mask','Name eigener Access Point':'Name of the own access point',
'Passwort Access Point (min. 8 Zeichen, leer = unverändert)':'Access point password (min. 8 characters, empty = unchanged)',
'⚠ Noch das Standard-Passwort „MobaStation“: hier ein eigenes eintragen und speichern.':'⚠ Still the default password "MobaStation": enter your own here and save.',
'Speichern und neu verbinden':'Save and reconnect','Netzwerk zurücksetzen':'Reset network',
'ESP-Firmware':'ESP firmware','Firmware-Datei (.bin)':'Firmware file (.bin)','Aktualisieren':'Update','ESP neu starten':'Restart ESP',
'Automatisch (Browser)':'Automatic (browser)','Protokoll':'Log',
'Ausgang':'Output','Doppelspule':'Double coil','Belebtes Haus':'Lived-in house','Kerze / Feuer':'Candle / fire','Blinklicht':'Flashing light',
'LED (dimmbar)':'LED (dimmable)','Rückmelder (8 Eingänge)':'Feedback (8 inputs)',
'Winkel Stellung 0':'Angle position 0','Winkel Stellung 1':'Angle position 1','Geschwindigkeit (0-255)':'Speed (0-255)',
'Abschaltzeit (x10 ms)':'Power-off delay (x10 ms)','Invertiert':'Inverted',
'An minimal (x10 s)':'On minimum (x10 s)','An maximal (x10 s)':'On maximum (x10 s)','Aus minimal (x10 s)':'Off minimum (x10 s)',
'Aus maximal (x10 s)':'Off maximum (x10 s)','Einschaltzeit (x10 s)':'Switch-on time (x10 s)','Ausschaltzeit (x10 s)':'Switch-off time (x10 s)',
'Helligkeit max (0-255)':'Brightness max (0-255)','Helligkeit min, dunkelste TV-Szene':'Brightness min, darkest TV scene',
'Modus (0 Kerze, 1 Feuer)':'Mode (0 candle, 1 fire)','Helligkeit min (0-255)':'Brightness min (0-255)','Periode (x10 ms)':'Period (x10 ms)',
'Überblenden (x10 ms)':'Fade (x10 ms)','Helligkeit max, an (0-255)':'Brightness max, on (0-255)','Helligkeit min, aus (0-255)':'Brightness min, off (0-255)',
'Helligkeit an (0-255)':'Brightness on (0-255)','Helligkeit aus (0-255)':'Brightness off (0-255)','Invertierte Eingänge':'Inverted inputs',
'– frei –':'– free –','Digital (an/aus)':'Digital (on/off)','LED dimmbar':'LED dimmable','Kerze':'Candle','Feuer (1-2 LEDs)':'Fire (1-2 LEDs)',
'Wechselblinker':'Alternating flasher','Rückmelder (Eingänge)':'Feedback (inputs)',
'unbekanntes Register':'unknown register','nur lesbar':'read only','ungültiger Wert':'invalid value','RAM des Boards voll':'RAM of the board full',
'Anschluss doppelt belegt':'pin used twice','Anschluss kann diese Funktion nicht':'pin can\'t do this function','falsche Anzahl Anschlüsse':'wrong number of pins',
'Servo an einem Pin: D9/D10 haben dann kein PWM':'Servo on a pin: D9/D10 then have no PWM','keine MobaBus-Verbindung':'no MobaBus connection','keine Antwort':'no answer',
'Hersteller (13 = DIY)':'Manufacturer (13 = DIY)','Modultyp':'Module type','Anzahl Adressen':'Number of addresses',
'unbekannte CV':'unknown CV','keine MobaBus-Adresse':'no MobaBus address',
'Kurze Adresse (1-127)':'Short address (1-127)','Anfahrspannung':'Start voltage','Beschleunigung':'Acceleration','Bremsverzögerung':'Deceleration',
'Höchstgeschwindigkeit':'Maximum speed','Mittlere Geschwindigkeit':'Mid speed','Version (nur lesen)':'Version (read only)',
'Hersteller (nur lesen)':'Manufacturer (read only)','Lange Adresse, hohes Byte':'Long address, high byte','Lange Adresse, niedriges Byte':'Long address, low byte',
'Konfiguration':'Configuration','Selbstbau':'DIY',
'Keine Antwort vom Decoder':'No answer from the decoder','Kurzschluss auf dem Programmiergleis':'Short circuit on the programming track','Zeitüberschreitung':'Timeout',
'Keine Verbindung':'No connection','Verbunden':'Connected','Zentrale nicht verbunden':'Station not connected','WLAN + AP':'WiFi + AP','kein WLAN':'no WiFi',
'Version {}, freier Speicher {} B':'Version {}, free memory {} B','Wert abgelehnt':'Value rejected','Typ {}':'Type {}','Rückmelder':'Feedback',
'ohne Adresse':'no address','Aus':'Off','An':'On','{} Module auf {} Board':'{} modules on {} board','{} Module auf {} Boards':'{} modules on {} boards','Noch keine Module gefunden.':'No modules found yet.',
'Universal-Board {}':'Universal board {}','Module ohne Board-ID':'Modules without board ID',
'{} von {} Funktionen · RAM frei {} %':'{} of {} functions · RAM free {} %','Sketch-Board':'Sketch board','ältere MobaBus-Version':'older MobaBus version',
'{} Funktion(en) nicht aufgebaut':'{} function(s) not built','Konfigurieren':'Configure',
'Noch keine Funktionen, mit „Konfigurieren“ einrichten.':'No functions yet, set them up with "Configure".',
'Board-ID war doppelt vergeben und wurde neu gewürfelt':'The board ID was used twice and has been rerolled','Suche...':'Searching...','Modul':'Module',
'Adresse und Anschluss stellst du beim Board unter „Konfigurieren“ ein.':'Address and pin are set at the board with "Configure".',
'ja':'yes','Wert 0-255':'Value 0-255','Zurückgesetzt':'Reset done','Adresse 1-16383':'Address 1-16383','Neue Adresse {}':'New address {}',
'Fehler: {}':'Error: {}','Lese {}/{}':'Reading {}/{}','Licht, 1 kHz':'Lights, 1 kHz',
'Kein PCA9685 gefunden (I2C an A4/A5, Adresse 0x40 oder 0x41). Es stehen die Pins des Nano zur Verfügung.':'No PCA9685 found (I2C at A4/A5, address 0x40 or 0x41). The pins of the Nano are available.',
'Ungespeicherte Änderungen':'Unsaved changes','ohne Adresse wartet das Modul':'without an address the module waits','Rückmelder 1-20':'Feedback 1-20',
'wird als DCC-Weiche gesendet (Zentrale)':'is sent as a DCC turnout (Station)','Anschluss…':'Pin…','belegt':'in use',
'erster Anschluss ist Dauerlicht':'first pin is always on','ohne internen Pull-up':'without internal pull-up','+ Funktion hinzufügen':'+ Add function',
'Auf diesem Board ist noch nichts eingerichtet.':'Nothing is set up on this board yet.','Änderungen verwerfen?':'Discard the changes?',
'Alle Funktionen von Board {} entfernen?':'Remove all functions of board {}?','Speichere…':'Saving…','Funktion {}: ':'Function {}: ',
'Gespeichert, Module neu aufgebaut':'Saved, modules rebuilt','Gespeichert':'Saved','Lokadresse 1-9999':'Loco address 1-9999',
'von {} Fahrstufen':'of {} speed steps','Lok wählen':'Select loco','Abzweig':'Diverging','Gerade':'Straight','{} entfernen?':'Remove {}?',
'Stellung {}':'Position {}','Name fehlt':'Name missing','Name der Lok':'Name of the loco','Lok {}':'Loco {}','Lok ist nicht gemerkt':'The loco is not saved',
'Schreibe CV {} = {}':'Writing CV {} = {}','Lese CV {}':'Reading CV {}','geschrieben':'written','gelesen':'read','Fehler':'Error',
'lange Adresse':'long address','kurze Adresse':'short address','28/128 Fahrstufen':'28/128 speed steps','14 Fahrstufen':'14 speed steps',
'Analogbetrieb an':'analog mode on','Analogbetrieb aus':'analog mode off','Fahrtrichtung umgekehrt':'direction reversed','RailCom an':'RailCom on',
'eigene Geschwindigkeitskennlinie':'custom speed curve','Hersteller {}':'Manufacturer {}','Lokadresse {}':'Loco address {}','lang':'long','kurz':'short',
'Adresse 1-9999':'Address 1-9999','Lokadresse {} geschrieben':'Loco address {} written',
'Decoder auf dem Programmiergleis auf Werkseinstellung zurücksetzen?':'Reset the decoder on the programming track to factory settings?',
'Lok {}: CV {} = {} gesendet':'Loco {}: CV {} = {} sent','Gespeichert auf dem ESP.':'Stored on the ESP.',
'Noch nichts gespeichert: die Zentrale läuft als Access Point, trage hier deinen Router ein.':'Nothing saved yet: the station runs as an access point, enter your router here.',
'WLAN-Einstellungen löschen? Die Zentrale startet danach als Access Point MobaStation (Passwort MobaStation).':'Delete the WiFi settings? The station then starts as access point MobaStation (password MobaStation).',
'Neustart':'Restart','Datei wählen':'Choose a file','Übertrage {} %':'Uploading {} %',
//texts of the ESP firmware (HTTP answers, log)
'Ohne WLAN-Name ist nur Access Point oder Hybrid möglich':'Without a WiFi name only access point or hybrid mode is possible',
'Das Access-Point-Passwort braucht mindestens 8 Zeichen':'The access point password needs at least 8 characters',
'Speichern fehlgeschlagen':'Saving failed','Gespeichert, das WLAN startet neu':'Saved, the WiFi restarts','Ungültige Liste':'Invalid list',
'WLAN-Einstellungen gelöscht, Neustart als Access Point':'WiFi settings deleted, restarting as access point','Update OK, Neustart':'Update OK, restarting',
'Update fehlgeschlagen':'Update failed','Zentrale verbunden':'Station connected','Access Point beendet, Router erreichbar':'Access point stopped, router reachable',
'Firmware-Update über das Netzwerk startet':'Firmware update over the network starts'};
//log lines of the ESP with a value in them
const ENP=[[/^Access Point (.*) gestartet$/,'Access point $1 started'],[/^Firmware-Update fehlgeschlagen \((.*)\)$/,'Firmware update failed ($1)']];
function L(s,...a){s=String(s);if(LANG=='en'){if(EN.hasOwnProperty(s))s=EN[s];else{const p=ENP.find(p=>p[0].test(s));if(p)s=s.replace(p[0],p[1])}}
 let i=0;return a.length?s.replace(/\{\}/g,()=>a[i++]):s}
//translates the strings of a table in place
function Lt(o){if(LANG=='en')for(const k in o){const v=o[k];if(typeof v=='string')o[k]=L(v);else if(v&&typeof v=='object')Lt(v)}}
//static texts and placeholders of the page
function Ldom(){if(LANG!='en')return;document.documentElement.lang='en';
 const w=document.createTreeWalker(document.body,NodeFilter.SHOW_TEXT);let n;
 while(n=w.nextNode()){const v=n.nodeValue.trim();if(v&&EN.hasOwnProperty(v))n.nodeValue=n.nodeValue.replace(v,()=>EN[v])}
 document.querySelectorAll('[placeholder],[title]').forEach(e=>{for(const a of ['placeholder','title']){const v=e.getAttribute(a);if(v&&EN.hasOwnProperty(v))e.setAttribute(a,EN[v])}})}
function setLang(v){try{if(v)localStorage.setItem('lang',v);else localStorage.removeItem('lang')}catch(e){}location.reload()}
const RAIL={0:['Spannung an','ok'],1:['Nothalt','warn'],2:['Spannung aus','bad'],4:['Kurzschluss','bad'],8:['Programmieren','warn']};
const TYPES={1:'Ausgang',2:'Doppelspule',3:'Servo',4:'16 Servos (PCA9685)',5:'Belebtes Haus',6:'Kerze / Feuer',7:'Blinklicht',8:'LED (dimmbar)',100:'Rückmelder (8 Eingänge)'};
const SERVO={33:'Winkel Stellung 0',34:'Winkel Stellung 1',35:'Geschwindigkeit (0-255)',36:'Abschaltzeit (x10 ms)'};
//a CV is a name or {n:name, bits:number of checkboxes (1 = yes/no)}
const INV={n:'Invertiert',bits:1};
const CVS={1:{33:INV},2:{33:'Abschaltzeit (x10 ms)',34:INV},3:SERVO,4:SERVO,
 5:{33:'An minimal (x10 s)',34:'An maximal (x10 s)',35:'Aus minimal (x10 s)',36:'Aus maximal (x10 s)',37:'Einschaltzeit (x10 s)',38:'Ausschaltzeit (x10 s)',
  39:'Helligkeit max (0-255)',40:'Helligkeit min, dunkelste TV-Szene'},
 6:{33:'Helligkeit max (0-255)',34:'Modus (0 Kerze, 1 Feuer)',35:'Helligkeit min (0-255)'},
 7:{33:'Periode (x10 ms)',34:'Überblenden (x10 ms)',35:'Helligkeit max, an (0-255)',36:'Helligkeit min, aus (0-255)'},
 8:{33:'Helligkeit an (0-255)',34:'Helligkeit aus (0-255)',35:'Überblenden (x10 ms)'},
 100:{33:{n:'Invertierte Eingänge',bits:8}}};
//universal boards: functions, abilities an IO needs (1 in, 2 out, 4 PWM, 8 servo), number of IOs
const FN=['– frei –','Digital (an/aus)','LED dimmbar','Kerze','Feuer (1-2 LEDs)','Blinklicht','Wechselblinker','Belebtes Haus','Servo','Rückmelder (Eingänge)'];
const FNEED=[0,2,4,4,4,2,2,2,8,1],FMIN=[0,1,1,1,1,1,2,1,1,1],FMAX=[0,1,1,1,2,1,2,8,1,8];
const BSTAT={0:'',1:'unbekanntes Register',2:'nur lesbar',3:'ungültiger Wert',4:'RAM des Boards voll',5:'Anschluss doppelt belegt',
 6:'Anschluss kann diese Funktion nicht',7:'falsche Anzahl Anschlüsse',8:'Servo an einem Pin: D9/D10 haben dann kein PWM',254:'keine MobaBus-Verbindung',255:'keine Antwort'};
const hex=n=>n.toString(16).toUpperCase().padStart(4,'0');
const ioName=io=>io>=100?'PCA'+(io>=116?2:1)+' K'+((io-100)%16):io>=14?'A'+(io-14):'D'+io;
const BASE={7:'Version',8:'Hersteller (13 = DIY)',30:'Modultyp',31:'Anzahl Adressen'};
const STATUS={0:'',1:'unbekannte CV',2:'nur lesbar',3:'ungültiger Wert',254:'keine MobaBus-Adresse',255:'keine Antwort'};
[RAIL,TYPES,CVS,FN,BSTAT,BASE,STATUS].forEach(Lt);
let ws,mods={},cur=null,pending={},rb={},rbAsked={},boards={},bd=null;
function toast(t){const e=$('toast');e.textContent=t;e.style.display='block';clearTimeout(e._t);e._t=setTimeout(()=>e.style.display='none',3000)}
function log(t){const l=$('log');l.textContent+=new Date().toLocaleTimeString()+'  '+t+'\n';l.scrollTop=l.scrollHeight}
const online=()=>ws&&ws.readyState==1;
function cmd(o){if(online())ws.send(JSON.stringify(o));else toast(L('Keine Verbindung'))}
document.querySelectorAll('#nav button').forEach(b=>b.onclick=()=>{
 history.replaceState(null,'','#/'+b.dataset.s);
 document.querySelectorAll('#nav button').forEach(x=>x.classList.toggle('on',x==b));
 document.querySelectorAll('section').forEach(s=>s.classList.toggle('on',s.id==b.dataset.s));
 if(b.dataset.s=='station')cmd({c:'settings'});if(b.dataset.s=='network')loadNet();if(b.dataset.s=='drive')refreshDrive();if(b.dataset.s=='prog')renderProg();
 if(b.dataset.s=='mobabus'&&online()&&!Object.keys(mods).length)discover()});
function connect(){
 ws=new WebSocket('ws://'+location.host+'/ws');
 ws.onopen=()=>{log(L('Verbunden'));cmd({c:'settings'});refreshDrive();if($('mobabus').classList.contains('on')&&!Object.keys(mods).length)discover()};
 ws.onclose=()=>{$('linkDot').className='dot bad';setTimeout(connect,2000)};
 ws.onmessage=e=>{const m=JSON.parse(e.data);(H[m.t]||(()=>{}))(m)}}
const fmtUp=s=>{const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h+' h '+m+' min'};
const H={
 status:m=>{$('linkDot').className='dot '+(m.link?'ok':'bad');
  if(m.rail!==undefined){const r=RAIL[m.rail]||['Status '+m.rail,'warn'];$('rail').textContent=r[0];$('railDot').className='dot '+r[1];
   $('main').textContent=m.main+' mA';$('prog').textContent=m.prog+' mA';$('volt').textContent=(m.volt/1000).toFixed(1)+' V';
   $('clients').textContent=m.clients;$('loop').textContent=m.loop+' µs';$('ram').textContent=m.ram+' B';$('uptime').textContent=fmtUp(m.uptime);
   $('linkErr').textContent=m.megaErr+' / '+m.espErr}else{$('rail').textContent=m.link?'-':L('Zentrale nicht verbunden')}
  $('udp').textContent=m.udpIn+' / '+m.udpOut},
 net:m=>{pwWarn(m.apPwdDefault);$('netInfo').textContent=L({station:'WLAN',ap:'Access Point','station+ap':'WLAN + AP',down:'kein WLAN'}[m.mode])+' '+m.ip+(m.rssi?' ('+m.rssi+' dBm)':'');
  $('fw').textContent=L('Version {}, freier Speicher {} B',m.version,m.heap)},
 setting:m=>{const e=$('s'+m.key);if(!e)return;
  if(m.key==3)e.value=(m.value/1000).toFixed(3);else if(m.key==4)e.checked=!!m.value;else e.value=m.value;
  if(!m.ok)$('sMsg').textContent=L('Wert abgelehnt')},
 module:m=>{mods[m.board?'b'+m.board+':'+m.slot:m.slot+':'+m.addr+':'+m.typeId]=m;renderMods();if(m.flags&4)newBoardId();if(!m.addr)return;
  if(m.typeId==100){const g=Math.floor((m.addr-1)/10);if(!rbAsked[g]){rbAsked[g]=1;cmd({c:'rbus',group:g})}}else askTurnout(m.addr)},
 board:m=>{boards[m.board]=m;renderMods();if(m.flags&1)newBoardId()},
 breg:m=>bAnswer(m),
 rbus:m=>{rb[m.group]=m.data;renderInputs()},
 cv:m=>{const id='cv'+m.cv;if(cur&&m.addr==cur.addr&&!!m.fb==(cur.typeId==100)){const e=$(id);const s=$(id+'s');
   if(m.status==0&&e&&m.cv!=1&&m.cv!=9&&!(m.cv==8&&m.value==8)){e.value=m.value;syncBits(m.cv)}if(s)s.textContent=m.status in STATUS?STATUS[m.status]:'Status '+m.status}
  delete pending[m.addr+':'+m.cv]},
 loco:m=>{if(m.addr!=loco.addr)return;loco.steps=m.steps;loco.fn=m.fn;const s=decodeSpeed(m.steps,m.speed);loco.dir=s[1];
  if(Date.now()-userT>800){loco.speed=s[0];$('throttle').value=s[0]}renderLoco()},
 turnout:m=>{tstate[m.addr]=m.state;renderStates();renderModStates()},
 prog:m=>progDone(m),
 log:m=>log(L(m.text))};
function el(tag,cls,text){const e=document.createElement(tag);if(cls)e.className=cls;if(text!==undefined)e.textContent=text;return e}
function openDlg(id){$(id).classList.add('open');document.body.classList.add('noscroll')}
function closeDlg(id){$(id).classList.remove('open');if(!document.querySelector('.modal.open'))document.body.classList.remove('noscroll')}
document.addEventListener('keydown',e=>{if(e.key!='Escape')return;if($('bdCard').classList.contains('open'))closeBoard();else closeDlg('cvCard')});
function modRow(m){const d=el('div','item'),n=el('div','n'),fb=m.typeId==100;
 n.append(el('b','',TYPES[m.typeId]||L('Typ {}',m.typeId)),
  el('div','sub',m.addr?L(fb?'Rückmelder':'Adresse')+' '+m.addr+(m.channels>1?' - '+(m.addr+m.channels-1):''):L('ohne Adresse')));
 d.append(n);
 if(fb&&m.addr){const ins=el('div','ins');ins.dataset.a=m.addr;for(let i=1;i<=8;i++)ins.append(el('span','in',i));d.append(ins)}
 const bt=el('div','btns');
 if(m.addr&&!fb)for(const [txt,v] of [[L('Aus'),0],[L('An'),1]]){const b=el('button','b',txt);b.dataset.a=m.addr;b.dataset.d=v;b.onclick=()=>switchItem(m.addr,v);bt.append(b)}
 if(m.addr){const b=el('button','b','CVs');b.onclick=()=>openCv(m);bt.append(b)}
 d.append(bt);return d}
function renderMods(){const t=$('mods');t.innerHTML='';
 //one block per board: universal boards first, then sketch boards, modules without board id last
 const groups={};for(const m of Object.values(mods))(groups[m.board||0]=groups[m.board||0]||[]).push(m);
 for(const b of Object.values(boards))groups[b.board]=groups[b.board]||[];
 const keys=Object.keys(groups).map(Number).sort((a,b)=>(!a-!b)||(!!boards[b]-!!boards[a])||a-b);
 const n=Object.keys(mods).length;$('mMsg').textContent=n||keys.length?L(keys.length==1?'{} Module auf {} Board':'{} Module auf {} Boards',n,keys.length):'';
 if(!keys.length)t.append(el('p','small muted',L('Noch keine Module gefunden.')));
 for(const k of keys){const g=el('div','grp'),h=el('div','gh'),hn=el('div','n'),u=boards[k];
  hn.append(el('b','',u?L('Universal-Board {}',hex(k)):k?'Board '+hex(k):L('Module ohne Board-ID')));
  hn.append(el('div','sub',u?L('{} von {} Funktionen · RAM frei {} %',u.built,u.slots,u.pool):L(k?'Sketch-Board':'ältere MobaBus-Version')));
  if(u&&u.configured>u.built)hn.append(el('div','sub err',L('{} Funktion(en) nicht aufgebaut',u.configured-u.built)));
  h.append(hn);if(u){const b=el('button','b p',L('Konfigurieren'));b.onclick=()=>openBoard(k);h.append(b)}
  g.append(h);
  const l=groups[k].sort((a,b)=>(a.typeId==100)-(b.typeId==100)||(a.addr||1e5)-(b.addr||1e5)||a.slot-b.slot);
  if(!l.length)g.append(el('div','empty',L('Noch keine Funktionen, mit „Konfigurieren“ einrichten.')));
  for(const m of l)g.append(modRow(m));
  t.append(g)}
 renderModStates();renderInputs()}
//a board took a new id (it was used twice): search again, the old entries are gone then
let newIdT=0;function newBoardId(){if(Date.now()-newIdT<5000)return;newIdT=Date.now();toast(L('Board-ID war doppelt vergeben und wurde neu gewürfelt'));setTimeout(discover,1500)}
function renderModStates(){document.querySelectorAll('#mods button[data-d]').forEach(b=>{const s=tstate[b.dataset.a];b.classList.toggle('on',s==Number(b.dataset.d)+1)})}
function rbGet(a){const g=rb[Math.floor((a-1)/10)];return g?g[(a-1)%10]:undefined}
function renderInputs(){document.querySelectorAll('#mods .ins').forEach(x=>{const v=rbGet(Number(x.dataset.a));
 [...x.children].forEach((e,i)=>e.classList.toggle('on',v!==undefined&&!!((v>>i)&1)))})}
function discover(){mods={};boards={};rbAsked={};renderMods();$('mMsg').textContent=L('Suche...');cmd({c:'discover'})}
function openCv(m){cur=m;openDlg('cvCard');$('cvTitle').textContent=(TYPES[m.typeId]||L('Modul'))+' · '+L(m.typeId==100?'Rückmelder':'Adresse')+' '+m.addr;
 //the address of a module on a universal board belongs to the board configuration
 $('addrBox').style.display=m.flags&2?'none':'';
 $('cvInfo').textContent=L('Geänderte Werte gelten sofort und auch nach einem Neustart.')+(m.flags&2?' '+L('Adresse und Anschluss stellst du beim Board unter „Konfigurieren“ ein.'):'');
 $('cvReset').textContent=L(m.flags&2?'Auf Standardwerte zurücksetzen':'Auf Sketch-Werte zurücksetzen');
 $('newAddr').value=m.addr;const t=$('cvs');t.innerHTML='';const list=Object.assign({},CVS[m.typeId]||{},BASE);
 for(const cv of Object.keys(list).map(Number).sort((a,b)=>a-b)){const ro=cv in BASE,def=list[cv],bits=def.bits||0;
  const tr=el('tr','cv');tr.append(el('td','',cv),el('td','',def.n||def));
  const td=el('td'),inp=el('input');inp.id='cv'+cv;inp.type='number';inp.min=0;inp.max=255;inp.disabled=ro;td.append(inp);
  const st=el('td');st.append(el('span','small muted'));st.lastChild.id='cv'+cv+'s';
  if(bits){inp.style.display='none';const w=el('div','bits');w.id='cv'+cv+'b';
   for(let i=0;i<bits;i++){const lb=el('label'),c=el('input');c.type='checkbox';
    //a checkbox writes at once, the value is read back
    c.onchange=()=>{let v=0;w.querySelectorAll('input').forEach((x,k)=>{if(x.checked)v|=1<<k});inp.value=v;writeCv(cv)};
    lb.append(c,bits>1?String(i+1):L('ja'));w.append(lb)}
   td.append(w)}
  else if(!ro){const b=el('button','b',L('Schreiben'));b.onclick=()=>writeCv(cv);st.prepend(b)}
  tr.append(td,st);t.append(tr)}
 readAll()}
//CV command for the open module, feedback modules have their own addresses
function mcv(o){cmd(Object.assign({c:'cv',addr:cur.addr},cur.typeId==100?{fb:1}:{},o))}
function syncBits(cv){const w=$('cv'+cv+'b');if(!w)return;const v=Number($('cv'+cv).value);w.querySelectorAll('input').forEach((x,k)=>x.checked=!!((v>>k)&1))}
function readAll(){if(!cur)return;document.querySelectorAll('#cvs input[type=number]').forEach((e,i)=>{const cv=Number(e.id.slice(2));setTimeout(()=>mcv({cv}),i*120)})}
function writeCv(cv){const v=Number($('cv'+cv).value);if(!(v>=0&&v<=255))return toast(L('Wert 0-255'));mcv({cv,value:v});setTimeout(()=>mcv({cv}),300)}
function resetCvs(){mcv({cv:8,value:8});toast(L('Zurückgesetzt'));setTimeout(readAll,500);setTimeout(discover,1500)}
function changeAddr(){const a=Number($('newAddr').value);if(!(a>0&&a<16384))return toast(L('Adresse 1-16383'));
 mcv({cv:1,value:a&255});setTimeout(()=>mcv({cv:9,value:a>>8}),200);
 toast(L('Neue Adresse {}',a));setTimeout(discover,1000)}
//---- universal boards: registers are read and written one request at a time
let bq=[],bcur=null,btimer=null;
//only one request is on the way: a callback that adds requests must not start a second one
function breq(o,cb){bq.push(Object.assign({cb},o));if(!bcur)bnext()}
function bnext(){clearTimeout(btimer);bcur=bq.shift()||null;if(!bcur)return;
 const o={c:'breg',board:bcur.board,reg:bcur.reg};if(bcur.values)o.values=bcur.values;else o.count=bcur.count||1;cmd(o);
 btimer=setTimeout(()=>{const c=bcur;bcur=null;c.tries=(c.tries||0)+1;if(c.tries<3)bq.unshift(c);else c.cb({status:255,values:[]});if(!bcur)bnext()},900)}
function bAnswer(m){if(!bcur||m.board!=bcur.board||m.reg!=bcur.reg)return;const c=bcur;bcur=null;clearTimeout(btimer);c.cb(m);if(!bcur)bnext()}
const SLOT=s=>16+s*13;
function openBoard(id){bq=[];bcur=null;clearTimeout(btimer);
 bd={id,found:0,modes:[0,0],caps:{},orig:{},slots:[]};
 openDlg('bdCard');$('bdTitle').textContent=L('Universal-Board {}',hex(id));$('bdBody').innerHTML='';$('bdSave').disabled=true;bd.showNew=false;
 const msg=$('bdMsg');msg.className='small';let done=0,total=1,failed=false;
 const step=m=>{if(m.status){if(!failed)msg.textContent=L('Fehler: {}',BSTAT[m.status]||m.status);failed=true;return false}
  msg.textContent=L('Lese {}/{}',++done,total);if(done==total){msg.textContent='';buildSlots();renderBoard()}return true};
 breq({board:id,reg:1,count:3},m=>{if(!step(m))return;bd.modes=[m.values[0],m.values[1]];bd.origModes=bd.modes.slice();bd.found=m.values[2];
  const ios=[...Array(32).keys()];for(const b of [0,1])if(bd.found&(1<<b))for(let c=0;c<16;c++)ios.push(100+b*16+c);
  const reads=[];for(let i=0;i<ios.length;i+=3)reads.push(['c',256+ios[i],Math.min(3,ios.length-i)]);
  for(let r=16;r<SLOT(16);r+=3)reads.push(['s',r,Math.min(3,SLOT(16)-r)]);
  total=1+reads.length;
  //store first: the last answer builds the dialog
  for(const [k,reg,n] of reads)breq({board:id,reg,count:n},m=>{
   if(!m.status)m.values.forEach((v,i)=>{if(k=='c')bd.caps[reg-256+i]=v;else bd.orig[reg+i]=v});step(m)})})}
function buildSlots(){bd.slots=[];for(let s=0;s<16;s++){const o=bd.orig,r=SLOT(s);
 bd.slots.push({f:o[r],io:[1,2,3,4,5,6,7,8].map(i=>o[r+i]),p1:o[r+9],p2:o[r+10],addr:o[r+11]|(o[r+12]<<8)})}}
function capsOf(io){if(io>=100){const b=io>=116?1:0;return bd.found&(1<<b)?(bd.modes[b]?8:6):0}return bd.caps[io]||0}
function renderBoard(){const B=$('bdBody');B.innerHTML='';
 for(const b of [0,1])if(bd.found&(1<<b)){const r=el('div','row');r.append(el('span','','PCA9685 0x4'+b+' (PCA'+(b+1)+')'));
  const s=el('select');s.style.width='auto';[[L('Licht, 1 kHz'),0],['Servos, 50 Hz',1]].forEach(([t,v])=>{const o=el('option','',t);o.value=v;s.append(o)});
  s.value=bd.modes[b];s.onchange=()=>{bd.modes[b]=Number(s.value);renderBoard()};r.append(s);B.append(r)}
 if(!bd.found)B.append(el('p','small muted',L('Kein PCA9685 gefunden (I2C an A4/A5, Adresse 0x40 oder 0x41). Es stehen die Pins des Nano zur Verfügung.')));
 //IOs in use: slot and position
 const used={};bd.slots.forEach((sl,s)=>{if(sl.f)sl.io.slice(0,FMAX[sl.f]).forEach((io,i)=>{if(io!=255)used[io]=s+':'+i})});
 const allIo=Object.keys(bd.caps).map(Number).concat([...Array(32)].map((_,c)=>100+c)).filter((v,i,a)=>a.indexOf(v)==i&&capsOf(v)).sort((a,b)=>a-b);
 const firstFree=bd.slots.findIndex(sl=>!sl.f);
 const mode=Number($('s1').value),range=Number($('s2').value);
 bd.slots.forEach((sl,s)=>{if(!sl.f&&!(bd.showNew&&s==firstFree))return;
  const d=el('div','slot'),r=el('div','row');r.append(el('span','no',s+1));
  const f=el('select','fn');FN.forEach((t,i)=>{const o=el('option','',t);o.value=i;f.append(o)});f.value=sl.f;
  f.onchange=()=>{sl.f=Number(f.value);if(sl.f)bd.showNew=false;else{sl.io=sl.io.map(()=>255);sl.p1=0;sl.p2=0;sl.addr=0}renderBoard()};r.append(f);
  if(sl.f){const fb=sl.f==9;r.append(el('span','small muted',L(fb?'Rückmelder':'Adresse')));
   const a=el('input','ad');a.type='number';a.min=fb?1:1;a.max=fb?20:16383;a.value=sl.addr||'';a.oninput=()=>{sl.addr=Number(a.value)||0;hint();const dt=boardDirty();$('bdSave').disabled=!dt;if(dt){$('bdMsg').className='small warn';$('bdMsg').textContent=L('Ungespeicherte Änderungen')}};r.append(a);
   const h=el('span','small');r.append(h);
   const hint=()=>{h.className='small warn';h.textContent=!sl.addr?L('ohne Adresse wartet das Modul'):
    fb?(sl.addr>20?L('Rückmelder 1-20'):''):((mode==1||(mode==2&&sl.addr<=range))?L('wird als DCC-Weiche gesendet (Zentrale)'):'')};hint()}
  d.append(r);
  if(sl.f){const ios=el('div','ios'),need=FNEED[sl.f];
   for(let i=0;i<FMAX[sl.f];i++){const w=el('span','io'),sel=el('select'),cur=sl.io[i];
    const tv=sl.f==7&&(sl.p1>>i)&1,nd=need|(tv?4:0);
    const none=el('option','',i<FMIN[sl.f]?L('Anschluss…'):'–');none.value=255;sel.append(none);
    for(const io of allIo){const c=capsOf(io);if((c&nd)!=nd&&io!=cur)continue;
     const o=el('option','',ioName(io)+(io<100&&c&4?' ~':''));o.value=io;
     if(used[io]!==undefined&&used[io]!=s+':'+i){o.disabled=true;o.textContent+=' ('+L('belegt')+')'}
     if((c&nd)!=nd)o.textContent+=' (!)';sel.append(o)}
    sel.value=cur;sel.onchange=()=>{sl.io[i]=Number(sel.value);renderBoard()};w.append(sel);
    if(sl.f==7){const lb=el('label'),c=el('input');c.type='checkbox';c.checked=tv;c.onchange=()=>{sl.p1^=1<<i;renderBoard()};lb.append(c,'TV');w.append(lb)}
    ios.append(w)}
   d.append(ios);
   if(sl.f==7||sl.f==9){const lb=el('label','opt'),c=el('input');c.type='checkbox';c.checked=sl.p2&1;c.onchange=()=>{sl.p2^=1;renderBoard()};
    lb.append(c,L(sl.f==7?'erster Anschluss ist Dauerlicht':'ohne internen Pull-up'));d.append(lb)}}
  B.append(d)});
 if(firstFree>=0&&!bd.showNew){const a=el('button','b',L('+ Funktion hinzufügen'));a.style.marginTop='10px';a.onclick=()=>{bd.showNew=true;renderBoard()};B.append(a)}
 if(!bd.slots.some(sl=>sl.f)&&!bd.showNew)B.prepend(el('p','small',L('Auf diesem Board ist noch nichts eingerichtet.')));
 const dirty=boardDirty();$('bdSave').disabled=!dirty;if(dirty){$('bdMsg').className='small warn';$('bdMsg').textContent=L('Ungespeicherte Änderungen')}
 else if($('bdMsg').textContent==L('Ungespeicherte Änderungen'))$('bdMsg').textContent=''}
function boardDirty(){if(!bd||!bd.slots.length)return false;const r=boardRegs();
 return Object.keys(r).some(k=>r[k]!==bd.orig[k])||bd.modes.some((m,i)=>m!=bd.origModes[i])}
function closeBoard(){if(boardDirty()&&!confirm(L('Änderungen verwerfen?')))return;closeDlg('bdCard')}
function clearBoard(){if(!bd||!bd.slots.length||!confirm(L('Alle Funktionen von Board {} entfernen?',hex(bd.id))))return;
 bd.slots.forEach(sl=>{sl.f=0;sl.io=sl.io.map(()=>255);sl.p1=0;sl.p2=0;sl.addr=0});bd.modes=[0,0];bd.showNew=false;renderBoard();saveBoard()}
function boardRegs(){const r={};bd.slots.forEach((sl,s)=>{const b=SLOT(s),n=sl.f?FMAX[sl.f]:0;
 r[b]=sl.f;for(let i=0;i<8;i++)r[b+1+i]=i<n?sl.io[i]:255;r[b+9]=sl.f?sl.p1&255:0;r[b+10]=sl.f?sl.p2&255:0;
 const a=sl.f?sl.addr:0;r[b+11]=a&255;r[b+12]=a>>8});return r}
function saveBoard(){if(!bd)return;const regs=boardRegs(),id=bd.id,msg=$('bdMsg');msg.className='small';
 const ch=Object.keys(regs).map(Number).filter(r=>regs[r]!==bd.orig[r]).sort((a,b)=>a-b);
 const writes=[];for(const b of [0,1])if(bd.modes[b]!=bd.origModes[b])writes.push([1+b,[bd.modes[b]]]);
 for(let i=0;i<ch.length;){let n=1;while(n<4&&i+n<ch.length&&ch[i+n]==ch[i]+n)n++;writes.push([ch[i],ch.slice(i,i+n).map(r=>regs[r])]);i+=n}
 let bad=0;$('bdSave').disabled=true;msg.textContent=L('Speichere…');
 for(const [reg,values] of writes)breq({board:id,reg,values},m=>{if(m.status)bad=m.status});
 breq({board:id,reg:0,values:[1]},m=>{$('bdSave').disabled=false;
  const st=bad||m.status,slot=m.values[1];
  if(st){msg.className='small err';msg.textContent=(slot!==undefined&&slot<16?L('Funktion {}: ',slot+1):'')+(BSTAT[st]||'Status '+st)}
  else{msg.className='small';msg.textContent=L('Gespeichert, Module neu aufgebaut')}
  bd.orig=Object.assign(bd.orig,regs);bd.origModes=bd.modes.slice();bd.showNew=false;renderBoard();setTimeout(discover,800)})}
function saveSettings(){$('sMsg').textContent='';
 cmd({c:'set',key:1,value:Number($('s1').value)});cmd({c:'set',key:2,value:Number($('s2').value)});
 cmd({c:'set',key:3,value:Math.round(Number($('s3').value)*1000)});cmd({c:'set',key:4,value:$('s4').checked?1:0});toast(L('Gespeichert'))}
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
function selectLoco(a,quiet){if(!(a>0&&a<10000))return toast(L('Lokadresse 1-9999'));loco={addr:a,steps:4,speed:0,dir:1,fn:0};
 store('loco',a);$('locoAddr').value=a;$('locoSel').value=lists.locos.some(l=>l.addr==a)?a:'';$('throttle').value=0;
 $('locoPanel').style.display='block';renderLoco();if(!quiet)cmd({c:'x',d:[0xE3,0xF0,amsb(a),a&255]})}
function renderLoco(){const max=STEPS[loco.steps]||126,t=$('throttle');t.max=max;
 $('spd').textContent=loco.speed;$('spdInfo').textContent=L('von {} Fahrstufen',max);$('dirShow').textContent=loco.dir?'▶':'◀';
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
 if(!r.ok)toast(L(await r.text()))}
function renderLists(){const s=$('locoSel');s.innerHTML='';const o0=document.createElement('option');o0.value='';o0.textContent=L('Lok wählen');s.append(o0);
 for(const l of lists.locos){const o=document.createElement('option');o.value=l.addr;o.textContent=l.name+' ('+l.addr+')';s.append(o)}
 if(loco.addr)s.value=lists.locos.some(l=>l.addr==loco.addr)?loco.addr:'';
 const t=$('items');t.innerHTML='';$('itemsEmpty').style.display=lists.items.length?'none':'block';
 lists.items.forEach((it,k)=>{const tr=document.createElement('div');tr.className='item';
  const n=document.createElement('div');n.className='n';n.textContent=it.name+' ';
  const a=document.createElement('span');a.className='muted small';a.textContent=it.addr;n.append(a);
  const st=document.createElement('span');st.className='st';st.dataset.a=it.addr;st.dataset.k=it.kind;
  const c=document.createElement('div');c.className='btns';const lb=(it.kind=='t'?['Abzweig','Gerade']:['Aus','An']).map(x=>L(x));
  lb.forEach((l,d)=>{const b=document.createElement('button');b.className='b';b.textContent=l;b.onclick=()=>switchItem(it.addr,d);c.append(b)});
  const x=document.createElement('button');x.className='b';x.textContent='✕';x.title=L('Entfernen');
  x.onclick=()=>{if(confirm(L('{} entfernen?',it.name))){lists.items.splice(k,1);saveLists();renderLists()}};c.append(x);
  tr.append(n,st,c);t.append(tr)});renderStates()}
function stateText(s,k){if(s==1)return L(k=='t'?'Abzweig':'Aus');if(s==2)return L(k=='t'?'Gerade':'An');return '?'}
function renderStates(){document.querySelectorAll('#items .st').forEach(e=>{const s=tstate[e.dataset.a];e.textContent=stateText(s,e.dataset.k);e.className='st'+(s==1?' s0':s==2?' s1':'')});
 const q=Number($('qAddr').value);const qs=$('qState');qs.textContent=q>0&&tstate[q]?L('Stellung {}',tstate[q]-1):'';}
function askTurnout(a){cmd({c:'x',d:[0x43,((a-1)>>8)&0x3F,(a-1)&255]})}
function switchItem(a,d){cmd({c:'turnout',addr:a,dir:d})}
function quick(d){const a=Number($('qAddr').value);if(!(a>0&&a<16384))return toast(L('Adresse 1-16383'));switchItem(a,d)}
function addItem(){const name=$('iName').value.trim(),addr=Number($('iAddr').value),kind=$('iKind').value;
 if(!name)return toast(L('Name fehlt'));if(!(addr>0&&addr<16384))return toast(L('Adresse 1-16383'));
 lists.items.push({name,addr,kind});lists.items.sort((a,b)=>a.addr-b.addr);saveLists();renderLists();askTurnout(addr);$('iName').value='';$('iAddr').value=''}
function saveLoco(){const a=Number($('locoAddr').value);if(!(a>0&&a<10000))return toast(L('Lokadresse 1-9999'));
 const name=prompt(L('Name der Lok'),(lists.locos.find(l=>l.addr==a)||{}).name||L('Lok {}',a));if(!name)return;
 lists.locos=lists.locos.filter(l=>l.addr!=a);lists.locos.push({addr:a,name});lists.locos.sort((x,y)=>x.name.localeCompare(y.name));
 saveLists();renderLists();if(loco.addr!=a)selectLoco(a)}
function forgetLoco(){const a=loco.addr||Number($('locoAddr').value);if(!lists.locos.some(l=>l.addr==a))return toast(L('Lok ist nicht gemerkt'));
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
[PCV,MAKER,PERR].forEach(Lt);
let progQ=[],progBusy=null;
function progOp(write,cv,val){return new Promise(res=>{progQ.push({write,cv,val,res});progNext()})}
function progNext(){if(progBusy||!progQ.length)return;if(!online()){progQ.forEach(o=>o.res({error:'offline'}));progQ=[];return}
 const o=progBusy=progQ.shift(),c=o.cv-1;
 $('pMsg').textContent=(o.write?L('Schreibe CV {} = {}',o.cv,o.val):L('Lese CV {}',o.cv))+' ...';
 cmd({c:'x',d:o.write?[0x24,0x12,(c>>8)&0x3F,c&255,o.val]:[0x23,0x11,(c>>8)&0x3F,c&255]});
 o.timer=setTimeout(()=>progDone({error:'timeout'}),15000)}
function progDone(m){const o=progBusy;if(!o)return;clearTimeout(o.timer);progBusy=null;
 if(!m.error){const e=$('pv'+o.cv);if(e)e.value=m.value;const s=$('ps'+o.cv);if(s)s.textContent=L(o.write?'geschrieben':'gelesen');if(o.cv==29)show29(m.value);if(o.cv==8)show8(m.value)}
 else{const s=$('ps'+o.cv);if(s)s.textContent=L('Fehler')}
 $('pMsg').textContent=m.error?'CV '+o.cv+': '+(PERR[m.error]||m.error):'CV '+o.cv+' = '+m.value;o.res(m);progNext()}
function show29(v){const t=[v&32?'lange Adresse':'kurze Adresse',v&2?'28/128 Fahrstufen':'14 Fahrstufen',v&4?'Analogbetrieb an':'Analogbetrieb aus'];
 if(v&1)t.push('Fahrtrichtung umgekehrt');if(v&8)t.push('RailCom an');if(v&16)t.push('eigene Geschwindigkeitskennlinie');$('p29').textContent='CV29 = '+v+': '+t.map(x=>L(x)).join(', ')}
function show8(v){const s=$('ps8');if(s)s.textContent=MAKER[v]||L('Hersteller {}',v)}
function renderProg(){const t=$('pCvs');if(t.children.length)return;
 for(const cv of Object.keys(PCV).map(Number)){const ro=cv==7||cv==8;const tr=document.createElement('div');tr.className='item cv';
  tr.innerHTML='<div class="n"><b>CV'+cv+'</b> '+PCV[cv]+'<div class="small muted" id="ps'+cv+'"></div></div><input id="pv'+cv+'" type="number" min="0" max="255"'+(ro?' disabled':'')+'><div class="btns"></div>';
  const c=tr.lastChild;const r=document.createElement('button');r.className='b';r.textContent=L('Lesen');r.onclick=()=>progOp(false,cv);c.append(r);
  if(!ro){const w=document.createElement('button');w.className='b';w.textContent=L('Schreiben');w.onclick=()=>progWrite(cv,$('pv'+cv).value);c.append(w)}
  t.append(tr)}
 if(loco.addr&&!$('mAddr').value)$('mAddr').value=loco.addr}
function progWrite(cv,s){const v=Number(s);if(String(s).trim()===''||!(v>=0&&v<=255))return toast(L('Wert 0-255'));return progOp(true,cv,v)}
function readCommon(){for(const cv of Object.keys(PCV).map(Number))progOp(false,cv)}
function progFree(w){const cv=Number($('pCv').value);if(!(cv>=1&&cv<=1024))return toast('CV 1-1024');
 if(w)progWrite(cv,$('pVal').value);else progOp(false,cv).then(m=>{if(!m.error)$('pVal').value=m.value})}
async function readAddr(){$('pAddr').textContent='...';const r29=await progOp(false,29);if(r29.error)return $('pAddr').textContent='-';
 let a;if(r29.value&32){const h=await progOp(false,17);if(h.error)return $('pAddr').textContent='-';const l=await progOp(false,18);if(l.error)return $('pAddr').textContent='-';a=((h.value&0x3F)<<8)|l.value}
 else{const r=await progOp(false,1);if(r.error)return $('pAddr').textContent='-';a=r.value}
 $('pAddr').textContent=a;$('pMsg').textContent=L('Lokadresse {}',a)+' ('+L(r29.value&32?'lang':'kurz')+')'}
async function writeAddr(){const a=Number($('pNewAddr').value);if(!(a>=1&&a<=9999))return toast(L('Adresse 1-9999'));
 const r29=await progOp(false,29);if(r29.error)return;let r;
 if(a<128){r=await progOp(true,1,a);if(r.error)return;r=await progOp(true,29,r29.value&~32)}
 else{r=await progOp(true,17,0xC0|(a>>8));if(r.error)return;r=await progOp(true,18,a&255);if(r.error)return;r=await progOp(true,29,r29.value|32)}
 if(!r.error){$('pAddr').textContent=a;$('pMsg').textContent=L('Lokadresse {} geschrieben',a)}}
function factoryReset(){if(confirm(L('Decoder auf dem Programmiergleis auf Werkseinstellung zurücksetzen?')))progOp(true,8,8)}
function pomWrite(){const a=Number($('mAddr').value),cv=Number($('mCv').value),v=Number($('mVal').value);
 if(!(a>=1&&a<=9999))return toast(L('Lokadresse 1-9999'));if(!(cv>=1&&cv<=1024))return toast('CV 1-1024');if($('mVal').value===''||!(v>=0&&v<=255))return toast(L('Wert 0-255'));
 const c=cv-1;cmd({c:'x',d:[0xE6,0x30,amsb(a),a&255,0xEC|((c>>8)&3),c&255,v]});$('mMsgPom').textContent=L('Lok {}: CV {} = {} gesendet',a,cv,v)}
$('qAddr').oninput=()=>{const a=Number($('qAddr').value);renderStates();if(a>0&&a<16384&&tstate[a]===undefined)askTurnout(a)};
async function loadNet(){const r=await fetch('/api/config');const j=await r.json();const a=j.active;
 $('nMode').value=a.mode;$('nSsid').value=a.ssid;$('nDhcp').checked=a.dhcp;$('nIp').value=a.ip;$('nSubnet').value=a.subnet;$('nGw').value=a.gateway;$('nApSsid').value=a.apSsid;
 $('nSource').textContent=L(a.saved?'Gespeichert auf dem ESP.':'Noch nichts gespeichert: die Zentrale läuft als Access Point, trage hier deinen Router ein.');pwWarn(a.apPwdDefault)}
function pwWarn(on){on=!!on;$('pwWarn').classList.toggle('on',on);$('nApHint').classList.toggle('on',on);$('nApPwd').classList.toggle('attn',on)}
function fixPw(){document.querySelector('#nav button[data-s="network"]').click();setTimeout(()=>{const e=$('nApPwd');e.scrollIntoView({block:'center'});e.focus()},100)}
async function saveNet(){const b={mode:Number($('nMode').value),ssid:$('nSsid').value,pwd:$('nPwd').value,dhcp:$('nDhcp').checked,
 ip:$('nIp').value,subnet:$('nSubnet').value,gateway:$('nGw').value,apSsid:$('nApSsid').value,apPwd:$('nApPwd').value};
 const r=await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)});$('nMsg').textContent=L(await r.text())}
async function resetNet(){if(!confirm(L('WLAN-Einstellungen löschen? Die Zentrale startet danach als Access Point MobaStation (Passwort MobaStation).')))return;const r=await fetch('/api/config/reset',{method:'POST'});$('nMsg').textContent=L(await r.text())}
async function restart(){await fetch('/api/restart',{method:'POST'});toast(L('Neustart'))}
function upload(){const f=$('fwFile').files[0];if(!f)return toast(L('Datei wählen'));const fd=new FormData();fd.append('firmware',f);
 const x=new XMLHttpRequest();x.upload.onprogress=e=>$('fwMsg').textContent=L('Übertrage {} %',Math.round(e.loaded*100/e.total));
 x.onload=()=>$('fwMsg').textContent=L(x.responseText);x.onerror=()=>$('fwMsg').textContent=L('Fehler');x.open('POST','/update');x.send(fd)}
Ldom();$('lang').value=langSet;
connect();
//the tab in the address (eg. /#/drive) opens directly, for a bookmark on the phone
{const b=document.querySelector('#nav button[data-s="'+location.hash.slice(2).replace(/[^a-z]/g,'')+'"]');if(b)b.click()}
</script>
</body>
</html>
)=====";
