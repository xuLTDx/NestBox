// PG233 (18.11.2026): the install mini page (install.h) - status, WiFi scan, choose network + password.
// Served at "/" while the box is in install mode, on its own WiFi NestBox-<ID> (http://192.168.4.1).
// One request at a time (the httpd serves 2 clients), status every 3 s.
#pragma once
static const char INST_HTML[] = R"HTML(<!doctype html>
<html lang="sk"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>NestBox – inštalácia</title>
<style>
:root{--bg:#eef1f4;--card:#fff;--ink:#1c2530;--mute:#667383;--line:#d5dce3;--acc:#0b6bcb;--ok:#1d8a4e;--bad:#c23a3a;--chip:#e3e8ee}
@media (prefers-color-scheme:dark){:root{--bg:#12171d;--card:#1b222b;--ink:#e3e8ee;--mute:#8d9aa9;--line:#2c3642;--acc:#5aa7f2;--ok:#4cc27f;--bad:#ef6b6b;--chip:#26303b}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.4 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;padding:12px 16px 32px;max-width:560px;margin-inline:auto}
h1{font-size:20px;margin:4px 0 10px}section{background:var(--card);border-radius:10px;padding:12px 14px;margin-bottom:12px}
h2{font-size:12px;letter-spacing:.06em;text-transform:uppercase;color:var(--mute);margin:0 0 6px;font-weight:600}
table{width:100%;border-collapse:collapse}td{padding:5px 0;border-bottom:1px solid var(--line)}td:last-child{text-align:right;font-weight:600}
.ok{color:var(--ok)}.bad{color:var(--bad)}.mute{color:var(--mute);font-size:14px}
button,input{font:inherit;padding:10px 12px;border-radius:8px;border:1px solid var(--line);background:var(--chip);color:var(--ink)}
button{cursor:pointer}button.main{background:var(--acc);color:#fff;border-color:var(--acc)}
input{width:100%;margin:8px 0}
.net{display:flex;justify-content:space-between;align-items:center;width:100%;text-align:left;margin:4px 0;background:var(--card)}
.net.sel{border-color:var(--acc);outline:2px solid var(--acc)}
#msg{min-height:1.4em;margin-top:8px}
</style></head><body>
<h1>NestBox <span id="id"></span></h1>
<section><h2>Stav</h2><table>
<tr><td>Mobilná sieť</td><td id="cell">…</td></tr>
<tr><td>WiFi</td><td id="wifi">…</td></tr>
<tr><td>FleetNest</td><td id="login">…</td></tr>
<tr><td>SD karta</td><td id="sd">…</td></tr>
</table><div class="mute" id="fw"></div></section>
<section><h2>WiFi pre box</h2>
<div class="mute" id="chosen"></div>
<button id="scan">Hľadať siete</button>
<div id="nets"></div>
<div id="form" hidden><input id="pw" type="password" placeholder="Heslo siete" autocomplete="off">
<button class="main" id="save">Uložiť a pripojiť</button></div>
<div id="msg"></div></section>
<script>
var $=function(i){return document.getElementById(i)},sel='',busy=0;
function esc(s){return s.replace(/[&<>"]/g,function(c){return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]})}
function why(r){return r==15||r==204?'zlé heslo':r==201?'sieť nenájdená':r==202||r==2?'sieť odmietla box':'kód '+r}
function nets(d){var h='';(d.nets||[]).forEach(function(n,i){h+='<button class="net'+(n.s==sel?' sel':'')+'" data-i="'+i+'"><span>'+esc(n.s)+(n.o?' (otvorená)':'')+'</span><span class="mute">'+n.r+' dBm</span></button>'});
 $('nets').innerHTML=d.scan_s<0?'':h||'<div class="mute">Žiadna sieť nenájdená.</div>';
 Array.prototype.forEach.call($('nets').children,function(b){if(!b.dataset)return;b.onclick=function(){sel=d.nets[+b.dataset.i].s;$('form').hidden=false;$('pw').value='';$('pw').focus();nets(d)}})}
var last={};
function show(d){last=d;$('id').textContent=d.id;$('fw').textContent='firmvér '+d.fw;
 $('cell').textContent=d.cell+(d.csq?' · '+d.csq+' dBm':'');
 $('wifi').innerHTML=d.wifi.ssid?'<span class="ok">'+esc(d.wifi.ssid)+'</span> · '+d.wifi.ip:'<span class="mute">nepripojené</span>';
 $('login').innerHTML=d.login?'<span class="ok">prihlásený</span> ('+(d.link=='WIFI'?'WiFi':d.link=='CELL'?'mobilná sieť':d.link)+')':'<span class="bad">nie</span>';
 $('sd').textContent=d.sd_mb?d.sd_mb+' MB':'chýba';
 $('chosen').textContent=d.chosen?'Uložená sieť: '+d.chosen:'';
 var j=d.join,m='';
 if(j.ssid&&j.ssid==d.chosen){if(j.st=='joining')m='Pripájam na '+j.ssid+'…';else if(j.st=='ok')m=d.login&&d.link=='WIFI'?'<span class="ok">Hotovo – box je pripojený na '+esc(j.ssid)+' a prihlásený do FleetNest.</span>':'Pripojené na '+esc(j.ssid)+', čakám na FleetNest…';else if(j.st=='failed')m='<span class="bad">Nepripojil sa na '+esc(j.ssid)+': '+why(j.r)+'.</span>'}
 if(!busy&&m)$('msg').innerHTML=m;
 if(!sel)nets(d)}
function poll(){if(busy)return setTimeout(poll,1000);fetch('/api/inst').then(function(r){return r.json()}).then(show).catch(function(){}).then(function(){setTimeout(poll,3000)})}
$('scan').onclick=function(){sel='';$('form').hidden=true;$('msg').textContent='Hľadám…';fetch('/api/inst/scan').then(function(){setTimeout(function(){$('msg').textContent=''},4000)})};
$('save').onclick=function(){var p=$('pw').value;if(p&&p.length<8){$('msg').innerHTML='<span class="bad">Heslo WiFi má aspoň 8 znakov.</span>';return}
 busy=1;fetch('/api/inst/wifi',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'ssid='+encodeURIComponent(sel)+'&pass='+encodeURIComponent(p)})
 .then(function(r){return r.json()}).then(function(d){$('msg').innerHTML=d.ok?'Uložené, pripájam…':'<span class="bad">Neuložené.</span>';if(d.ok){sel='';$('form').hidden=true}})
 .catch(function(){$('msg').textContent='Spojenie s boxom sa prerušilo – pripoj sa znova na jeho WiFi.'}).then(function(){busy=0})};
poll();
</script></body></html>
)HTML";
