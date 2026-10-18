// "/" - a status page: everything comes from /api/info, refreshed every 5 s.
#pragma once
static const char PAGE_HTML[] = R"HTML(<!doctype html><html lang="sk"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>NestBox_v2O</title>
<style>body{font-family:sans-serif;background:#1e2227;color:#e8e8e8;margin:16px;max-width:900px}
h1{font-size:1.3rem}table{border-collapse:collapse;margin:.5rem 0}td,th{border:1px solid #444;padding:.25rem .5rem;text-align:left}
a{color:#8ab4f8}button{margin:.2rem;padding:.3rem .7rem}.bad{color:#ff8a80}.ok{color:#9be7a6}</style></head><body>
<h1>NestBox_v2O <span id="fw"></span></h1><div id="dev"></div>
<h2>Sniff</h2><div id="sn"></div>
<p><button onclick="c('SNIFF=1,120')">Štart 120 min</button><button onclick="c('SNIFF=0')">Stop</button></p>
<table id="bus"></table><h2>Súbory</h2><div id="files"></div><p id="msg"></p>
<script>
function c(cmd){fetch('/api/control?cmd='+encodeURIComponent(cmd)).then(r=>r.text()).then(t=>{msg.textContent=cmd+': '+t;setTimeout(load,1500)})}
function e(t){const d=document.createElement('div');d.textContent=t;return d}
function load(){fetch('/api/info').then(r=>r.json()).then(i=>{fw.textContent=i.fw;
dev.replaceChildren(e('ID '+i.id+' · FleetNest '+(i.login?'prihlásené':'NEprihlásené')+' · MAC '+i.mac+' · boot '+i.boot+' · beží '+Math.round(i.uptime_s/60)+' min · WiFi '+i.wifi+' ('+i.rssi+' dBm) · SD '+i.sd_used_mb+' / '+i.sd_mb+' MB'));
const s=i.sniff;sn.replaceChildren(e((s.on?'ZAPNUTÝ, zostáva '+s.min+' min, súbor S'+s.file:'vypnutý')+(s.refused?' · ODMIETNUTÝ (plná SD)':'')+' · vozidlo '+s.vehicle),
e('rámce '+s.frames+' · zapísané '+s.written+' · stratené pred SD '+s.dropped+' · SD chyby '+s.sderr+' · SD stratené '+s.sdlost));
bus.replaceChildren();const h=bus.insertRow();['CAN','čip','zapnutá','rýchlosť','rámce','straty / chyby'].forEach(x=>{const t=document.createElement('th');t.textContent=x;h.appendChild(t)});
s.buses.forEach(b=>{const r=bus.insertRow();const loss=b.bus==1?('missed '+b.missed+' overrun '+b.overrun+' bus_err '+b.buserr):('overflow '+b.overflow+' err '+b.errirq+' rec '+b.rec);
[b.bus,b.present?'áno':'NIE',b.on?'áno':'nie',b.speed+(b.found?' kbit/s':' ? (hľadá)'),b.frames,loss].forEach(x=>r.insertCell().textContent=x)})});
fetch('/api/sniff/list').then(r=>r.json()).then(l=>{files.replaceChildren();l.sort((a,b)=>b.n-a.n).forEach(f=>{const a=document.createElement('a');a.href='/api/sniff/'+f.n;a.textContent='S'+f.n+'.LOG ('+f.size+' B)';files.appendChild(a);files.appendChild(document.createElement('br'))})})}
load();setInterval(load,5000);
</script></body></html>)HTML";
