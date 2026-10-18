// "/" - the box's web page. The NestBox_v1F page (firmware_v5/telelogger/webui/index.html: layout,
// style, one request at a time - the httpd serves 2 clients) on the v2O API: /api/info, /api/live,
// /api/list, /api/events/<n>, /api/log/<n>, /api/sniff..., /api/control.
#pragma once
static const char PAGE_HTML[] = R"HTML(<!doctype html>
<html lang="sk"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>NestBox_v2O</title>
<style>
:root{--bg:#eef1f4;--card:#fff;--ink:#1c2530;--mute:#667383;--line:#d5dce3;--acc:#0b6bcb;--ok:#1d8a4e;--warn:#b86e00;--bad:#c23a3a;--chip:#e3e8ee}
@media (prefers-color-scheme:dark){:root{--bg:#12171d;--card:#1b222b;--ink:#e3e8ee;--mute:#8d9aa9;--line:#2c3642;--acc:#5aa7f2;--ok:#4cc27f;--warn:#e6a23c;--bad:#ef6b6b;--chip:#26303b}}
*{box-sizing:border-box}[hidden]{display:none!important}
body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.4 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;padding:12px 16px 32px;max-width:900px;margin-inline:auto}
header{display:flex;flex-wrap:wrap;align-items:baseline;gap:4px 12px;margin-bottom:10px}h1{font-size:20px;margin:0}
.mute{color:var(--mute);font-size:13px}
#status{border-radius:10px;padding:12px 14px;margin-bottom:12px;background:var(--card);border-left:6px solid var(--mute)}
#status b{font-size:17px;display:block}#status.ok{border-color:var(--ok)}#status.warn{border-color:var(--warn)}#status.bad{border-color:var(--bad)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;margin-bottom:12px}
.tile{background:var(--card);border-radius:10px;padding:10px 12px}
.tile h2,section h2{font-size:12px;letter-spacing:.06em;text-transform:uppercase;color:var(--mute);margin:0 0 4px;font-weight:600}
.big{font-size:24px;font-weight:600;font-variant-numeric:tabular-nums}.kv{font-size:13px;color:var(--mute);font-variant-numeric:tabular-nums}
section{background:var(--card);border-radius:10px;padding:10px 12px;margin-bottom:12px}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
td{padding:4px 0;border-bottom:1px solid var(--line)}td:last-child{text-align:right;font-weight:600}td.mute{text-align:left;font-weight:400;border:0}
.row{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-bottom:8px}
select,button,input{font:inherit;padding:6px 10px;border-radius:8px;border:1px solid var(--line);background:var(--chip);color:var(--ink)}
input{width:7em}button{cursor:pointer}button.danger{color:var(--bad)}
a.btn{font:inherit;padding:6px 10px;border-radius:8px;border:1px solid var(--line);background:var(--chip);color:var(--ink);text-decoration:none}
pre{margin:0;font:12px/1.5 ui-monospace,Consolas,monospace;white-space:pre-wrap;word-break:break-word;max-height:360px;overflow:auto}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:var(--mute);margin-right:6px}.dot.ok{background:var(--ok)}.dot.bad{background:var(--bad)}
#msg{min-height:1.4em;font-size:13px}
</style></head><body>
<header><h1 id="id">NestBox_v2O</h1><span class="mute" id="fw"></span><span class="mute" id="conn"><span class="dot"></span>pripájam…</span></header>
<div id="status"><b id="st">Načítavam…</b><span class="kv" id="st2"></span></div>
<div class="grid">
 <div class="tile"><h2>Napätie</h2><div class="big" id="v">–</div><div class="kv" id="v2"></div></div>
 <div class="tile"><h2>GPS</h2><div class="big" id="g">–</div><div class="kv" id="g2"></div></div>
 <div class="tile"><h2>Sieť</h2><div class="big" id="n">–</div><div class="kv" id="n2"></div></div>
 <div class="tile"><h2>SD karta</h2><div class="big" id="sd">–</div><div class="kv" id="sd2"></div></div>
</div>
<section><h2>Auto (CAN1, čerstvé hodnoty)</h2><table id="car"><tr><td class="mute">Žiadne čerstvé hodnoty z auta.</td></tr></table><div class="kv" id="can1"></div></section>
<section><h2>Udalosti na SD</h2>
 <div class="row"><select id="file" aria-label="Súbor"><option value="0">aktuálny súbor</option></select><button id="evr">Obnoviť</button><a id="evdl" class="btn" href="/api/log/0" download="aktualny.csv">Stiahnuť CSV</a></div>
 <pre id="ev">…</pre></section>
<section><h2>Sniff (všetky zbernice)</h2>
 <div class="row"><button id="snt"></button><select id="snm" aria-label="Ako dlho"><option value="30">30 min</option><option value="60">1 h</option><option value="120" selected>2 h</option><option value="240">4 h</option><option value="480">8 h</option></select><span class="mute" id="sns"></span></div>
 <table id="snb"></table>
 <div class="row" id="snf" hidden style="margin-top:8px"><label>ID <input id="sni" placeholder="napr. 552"></label><label>maska <input id="snk" placeholder="napr. 7FF"></label><button id="sna">Filtrovať</button><span class="mute">len zobrazenie, na SD ide všetko</span></div>
 <pre id="snl" hidden></pre>
 <div class="row" style="margin-top:8px"><select id="snfile" aria-label="Záznam"></select><a id="sndl" class="btn" href="#">Stiahnuť záznam</a></div></section>
<section><h2>Čítanie z auta (READ)</h2>
 <div class="row"><select id="rb"><option value="1">CAN1</option><option value="2">CAN2</option><option value="3">CAN3</option></select><label>ID dopytu <input id="rt" placeholder="710"></label><label>ID odpovede <input id="rr" placeholder="77A"></label><label>dopyt <input id="rq" placeholder="22 02 BD" style="width:9em"></label><button id="rgo">Odoslať</button></div>
 <div class="mute">Iba čítacie služby 22, 21, 01, 09, 19. Výsledok je v udalostiach na SD (riadok READ) a vo FleetNeste.</div></section>
<section><h2>Akcie</h2>
 <div class="row"><button id="ota">Skontrolovať OTA teraz</button><button id="rst" class="danger">Reštartovať</button></div>
 <div id="msg" class="mute"></div></section>
<script>
var $=function(i){return document.getElementById(i)};
var ENG={0:'zamknutý',1:'vypnutý',2:'štartuje',3:'beží',4:'zastavený (štart-stop)',5:'reštart',6:'núdzový reštart',7:'pripravuje sa'};
var q=Promise.resolve();
function get(u,t){var p=q.then(function(){var c=new AbortController(),h=setTimeout(function(){c.abort()},t||6000);return fetch(u,{signal:c.signal,cache:'no-store'}).finally(function(){clearTimeout(h)})});q=p.catch(function(){});return p}
function json(u){return get(u).then(function(r){return r.json()})}
function dur(s){s=Math.round(s);if(s<60)return s+' s';if(s<3600)return Math.floor(s/60)+' min '+(s%60)+' s';var h=Math.floor(s/3600);return h>47?Math.floor(h/24)+' d '+(h%24)+' h':h+' h '+Math.floor(s%3600/60)+' min'}
function kb(b){return b>1e9?(b/1e9).toFixed(1)+' GB':b>1e6?(b/1e6).toFixed(1)+' MB':Math.round(b/1e3)+' kB'}
function online(ok){$('conn').innerHTML='<span class="dot '+(ok?'ok':'bad')+'"></span>'+(ok?'online':'zariadenie neodpovedá')}
function fresh(a){return a>=0&&a<5000}
function live(){
 return json('/api/live').then(function(d){
  online(true);
  var el=$('status');
  if(d.engine){el.className='ok';$('st').textContent='Jazda – motor beží';$('st2').textContent=fresh(d.rpm_age)?d.rpm+' ot/min':''}
  else if(d.can1.last_age>=0&&d.can1.last_age<10000){el.className='warn';$('st').textContent='Auto zobudené, motor stojí';$('st2').textContent='CAN1 '+d.can1.kbit+' kbit/s'}
  else{el.className='';$('st').textContent=d.v<1?'Na stole (USB napájanie)':'Auto spí';$('st2').textContent='CAN1 '+(d.can1.found?'ticho':'hľadá rýchlosť')}
  $('v').textContent=d.v.toFixed(2)+' V';$('v2').textContent=d.v<1?'USB / bez auta (12 V nie je)':d.v>=13.2?'motor beží (nabíja)':'motor stojí';
  var g=d.gps;
  if(g.fix&&g.age>=0&&g.age<10000){$('g').textContent=g.kph.toFixed(0)+' km/h';$('g2').textContent=g.lat.toFixed(5)+', '+g.lng.toFixed(5)+' · '+g.sats+' sat'}
  else{$('g').textContent='bez fixu';$('g2').textContent='GPS je v modeme ('+d.net.cell+')'}
  $('n').textContent=d.net.link=='WIFI'?'WiFi':d.net.link=='CELL'?'Mobilná':'odpojené';
  $('n2').textContent='FleetNest '+(d.net.login?'prihlásené':'NEprihlásené')+' · '+d.net.tx+' paketov · '+kb(d.net.bytes)+' · modem: '+d.net.cell;
  $('sd2').textContent='súbor '+d.file+' · '+(d.pending?d.pending+' záznamov čaká na server':'všetko odoslané');
  var rows=[];
  if(fresh(d.odo_age))rows.push(['tachometer',d.odo+' km']);
  if(fresh(d.rpm_age))rows.push(['otáčky',d.rpm+' ot/min']);
  if(d.eng_state||fresh(d.rpm_age))rows.push(['stav motora',ENG[d.eng_state]||d.eng_state]);
  if(fresh(d.fuel_age))rows.push(['palivo (kandidát)',d.fuel+' l']);
  if(fresh(d.urea_age))rows.push(['AdBlue – dojazd (kandidát)',d.urea_km+' km']);
  $('car').innerHTML=rows.length?rows.map(function(r){return '<tr><td>'+r[0]+'</td><td>'+r[1]+'</td></tr>'}).join(''):'<tr><td class="mute">Žiadne čerstvé hodnoty z auta (profil '+d.veh+').</td></tr>';
  $('can1').textContent='CAN1: '+d.can1.frames+' rámcov od štartu';
 }).catch(function(){online(false)});
}
function info(){
 return json('/api/info').then(function(d){
  $('id').textContent=d.id+' · NestBox_v2O';$('fw').textContent='FW '+d.fw+' · boot '+d.boot+' · beží '+dur(d.uptime_s)+' · WiFi '+d.wifi+' ('+d.rssi+' dBm)';
  $('sd').textContent=(d.sd_used_mb/1024).toFixed(1)+' / '+(d.sd_mb/1024).toFixed(0)+' GB';
 }).catch(function(){});
}
function files(){
 return json('/api/list').then(function(l){
  l.sort(function(a,b){return b.id-a.id});
  $('file').innerHTML='<option value="0">aktuálny súbor</option>'+l.slice(0,40).map(function(f){return '<option value="'+f.id+'">'+f.id+(f.active?' (aktuálny)':'')+' · '+kb(f.size)+'</option>'}).join('');
 }).catch(function(){});
}
function events(){
 return get('/api/events/'+$('file').value).then(function(r){return r.text()}).then(function(t){
  var l=t.trim().split('\n');$('ev').textContent=t.trim()?l.reverse().join('\n'):'(žiadne udalosti)';
 }).catch(function(){$('ev').textContent='Nepodarilo sa načítať.'});
}
function cmd(c,ok){$('msg').textContent='Posielam…';return get('/api/control?cmd='+encodeURIComponent(c)).then(function(r){return r.text()}).then(function(t){$('msg').textContent=t.trim().indexOf('OK')==0?(ok||t):'Chyba: '+t})}
$('ota').onclick=function(){cmd('OTA_CHECK','OTA kontrola beží, výsledok v udalostiach.')};
$('rst').onclick=function(){if(confirm('Reštartovať zariadenie?'))cmd('RESTART','Reštartuje sa, stránka sa o chvíľu znova pripojí.').catch(function(){$('msg').textContent='Reštartuje sa…'})};
$('rgo').onclick=function(){var r='READ='+$('rb').value+','+$('rt').value.trim()+','+$('rr').value.trim()+','+$('rq').value.replace(/\s/g,'');cmd(r,null).then(function(){setTimeout(events,3000)})};
$('file').onchange=function(){var id=$('file').value;$('evdl').href='/api/log/'+id;$('evdl').download=(id=='0'?'aktualny':id)+'.csv';events()};$('evr').onclick=events;
var snOn=0,snId='',snMask='';
function sniff(){
 return json('/api/sniff').then(function(d){
  snOn=d.on;$('snt').textContent=d.on?'Vypnúť sniff':'Zapnúť sniff';$('snm').hidden=!!d.on;
  $('sns').textContent=d.on?'beží, zostáva '+d.min+' min · S'+d.file+(d.sd?'':' (SD nedostupná!)')+' · rámcov '+d.frames+' · zapísané '+d.written+' · stratené pred SD '+d.dropped:'vypnutý · vozidlo '+d.vehicle;
  $('snb').innerHTML=d.buses.map(function(b){var loss=b.bus==1?('missed '+b.missed+' · overrun '+b.overrun+' · chyby '+b.buserr):('pretečenie '+b.overflow+' · chyby '+b.errirq);return '<tr><td>CAN'+b.bus+(b.present?'':' (čip neodpovedá)')+' · '+b.speed+' kbit/s'+(b.found?'':' ?')+' · '+loss+'</td><td>'+b.frames+'</td></tr>'}).join('');
  $('snf').hidden=$('snl').hidden=!d.on;
  if(d.on)return get('/api/sniff/live?id='+encodeURIComponent(snId)+'&mask='+encodeURIComponent(snMask)).then(function(r){return r.text()}).then(function(t){$('snl').textContent=t.trim()?t.trim().split('\n').reverse().join('\n'):'(zatiaľ žiadne rámce)'});
 }).catch(function(){});
}
function snDl(){var n=$('snfile').value;$('sndl').href=n?'/api/sniff/'+n:'#';$('sndl').download='S'+n+'.LOG'}
function snFiles(){
 return json('/api/sniff/list').then(function(l){
  l.sort(function(a,b){return b.n-a.n});
  $('snfile').innerHTML=l.length?l.slice(0,40).map(function(f){return '<option value="'+f.n+'">S'+f.n+' · '+kb(f.size)+'</option>'}).join(''):'<option value="">žiadne záznamy</option>';snDl();
 }).catch(function(){});
}
$('snfile').onchange=snDl;
$('snt').onclick=function(){
 if(snOn){cmd('SNIFF=0','Sniff sa vypína, záznam sa uzavrie.').then(function(){setTimeout(function(){sniff();snFiles()},4000)});return}
 if(!confirm('Zapnúť sniff na '+$('snm').selectedOptions[0].text+'? Nahráva všetky zbernice, iba počúva. Po čase sa vypne sám.'))return;
 cmd('SNIFF=1,'+$('snm').value,'Sniff sa zapína.').then(function(){setTimeout(function(){sniff();snFiles()},4000)});
};
$('sna').onclick=function(){snId=$('sni').value.trim();snMask=$('snk').value.trim();sniff()};
sniff();snFiles();setInterval(sniff,3000);
function loop(){live().then(function(){setTimeout(loop,2000)})}
info().then(files).then(events).then(loop);
setInterval(info,15000);
setInterval(function(){if($('file').value=='0')events()},20000);
</script></body></html>)HTML";
