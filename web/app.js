const $=s=>document.querySelector(s);
const esc=s=>String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
const f=(v,d=1)=>v==null?'—':Number(v).toFixed(d);
const tm=ms=>{const s=Math.floor(ms/1000);return Math.floor(s/60)+':'+String(s%60).padStart(2,'0')};
const kb=b=>b>1024?(b/1024).toFixed(1)+' KB':b+' B';
async function j(u){const r=await fetch(u);if(!r.ok)throw new Error(r.status);return r.json()}
let sel=null,filtre='ALL',ultima=null;

async function live(){
  try{
    const d=await j('/api/live');
    const p=(t,on)=>`<span class="pill ${on?'on':''}">${on?'●':'○'} ${t}</span>`;
    $('#live').innerHTML=p('Sensor',d.sensor)+' '+p('Càmera',d.cam)+' '+p('HuskyLens',d.husky)+' '+
      `<span class="pill ${d.mision_activa?'on':''}">${d.mision_activa?'● '+d.mision_activa+' ('+tm(d.mision_ms)+')':'○ cap missió activa'}</span> `+
      `<span class="pill">RSSI S/C: ${d.salut.sensor_rssi}/${d.salut.cam_rssi} dBm</span> `+
      `<span class="pill">Temp S/C: ${f(d.salut.sensor_temp)}/${f(d.salut.cam_temp)} °C</span> `+
      `<span class="pill ${d.anomalies_tcp?'bad':''}">Anomalies TCP: ${d.anomalies_tcp}</span> `+
      `<span class="pill">Càmera: ${d.fps_cam} FPS</span> <span class="pill">Actiu: ${tm(d.uptime_s*1000)}</span>`;
  }catch(e){$('#live').innerHTML='<span class="pill bad">○ servidor sense resposta</span>'}
}

async function llista(){
  try{
    const ms=await j('/api/missions');
    $('#llista').innerHTML=ms.map(m=>`<div class="m ${m.nom===sel?'sel':''}" data-n="${m.nom}">
      ${m.activa?'<span style="color:var(--ok)">●</span> ':''}${esc(m.etiqueta)}
      <small>${esc(m.duracio)} · ${m.n_reg} deteccions</small></div>`).join('')||'<p style="color:var(--mut)">Cap missió guardada.</p>';
    document.querySelectorAll('.m').forEach(e=>e.onclick=()=>{sel=e.dataset.n;carrega();llista()});
  }catch(e){}
}

function grafic(id,t,linies,unit){
  const c=document.getElementById(id);if(!c)return;
  const W=c.width=c.clientWidth,H=c.height=110,x=c.getContext('2d');
  x.clearRect(0,0,W,H);x.font='10px Consolas';
  const tots=linies.flatMap(l=>l.v.filter(a=>a!=null));
  if(!tots.length){x.fillStyle='#8890a8';x.fillText('sense dades',8,20);return}
  let mn=Math.min(...tots),mx=Math.max(...tots);if(mn==mx){mn-=1;mx+=1}
  const T=t[t.length-1]||1,X0=38;
  x.strokeStyle='#2a3a66';x.strokeRect(X0,4,W-X0-4,H-18);
  x.fillStyle='#8890a8';x.fillText(f(mx)+unit,0,12);x.fillText(f(mn)+unit,0,H-8);
  x.fillText(tm(T),W-30,H-2);
  linies.forEach(l=>{x.strokeStyle=l.c;x.beginPath();let on=false;
    l.v.forEach((a,i)=>{if(a==null)return;
      const px=X0+t[i]/T*(W-X0-4),py=H-14-(a-mn)/(mx-mn)*(H-22);
      on?x.lineTo(px,py):x.moveTo(px,py);on=true});x.stroke()});
}

function render(d){
  const s=d.stats,q=d.qualitat_rssi,h=d.husky;
  const k=(v,l)=>`<div class="k"><b>${v}</b><span>${l}</span></div>`;
  const fila=(n,u,o)=>o?`<tr><td>${n}</td><td>${f(o.min)}${u}</td><td>${f(o.avg)}${u}</td><td>${f(o.max)}${u}</td></tr>`:`<tr><td>${n}</td><td colspan=3>—</td></tr>`;
  const mx=Math.max(1,...h.cronologia);
  let html=`<h2>${esc(d.nom)} ${d.activa?'<span style="color:var(--ok)">(en curs)</span>':''}</h2>
  <div class="grid">${k(tm(d.duracio_ms),'Durada')}${k(d.n_telemetria,'Mostres telemetria')}${k(d.freq_hz+' Hz','Freqüència real')}
  ${k(d.talls+(d.talls?' ('+f(d.max_tall_ms/1000)+'s màx)':''),'Talls de dades (>2s)')}${k(h.total,'Deteccions HuskyLens')}
  ${k(h.sense_deteccio,'Frames sense detecció')}${k(d.nivells.WARNING+' / '+d.nivells.ERROR,'Warnings / Errors')}</div>
  <h2>Estadístiques</h2><table><tr><th></th><th>Mín</th><th>Mitja</th><th>Màx</th></tr>
  ${fila('Temperatura','°C',s.temp)}${fila('Humitat','%',s.hum)}${fila('Pressió',' hPa',s.pres)}${fila('RSSI sensor',' dBm',s.rssi)}
  ${fila('Roll','°',s.roll)}${fila('Pitch','°',s.pitch)}${fila('Yaw','°',s.yaw)}</table>`;
  if(q)html+=`<h2>Qualitat de senyal (RSSI)</h2><div class="grid">${k(q.excelent+'%','Excel·lent (≥ -60)')}${k(q.acceptable+'%','Acceptable (≥ -75)')}${k(q.deficient+'%','Deficient (< -75)')}</div>`;
  html+=`<h2>Gràfics</h2><div class="charts">
  <div class="c"><div>Temperatura</div><canvas id="g1"></canvas></div>
  <div class="c"><div>Humitat</div><canvas id="g2"></canvas></div>
  <div class="c"><div>Pressió</div><canvas id="g3"></canvas></div>
  <div class="c"><div>RSSI</div><canvas id="g4"></canvas></div>
  <div class="c"><div>Orientació (<span style="color:#ff9f1c">roll</span> <span style="color:#c77dff">pitch</span> <span style="color:#00c9a7">yaw</span>)</div><canvas id="g5"></canvas></div></div>
  <h2>HuskyLens — per objecte ${h.algos.length?'('+esc(h.algos.join(', '))+')':''}</h2>`;
  html+=h.per_id.length?`<table><tr><th>Objecte</th><th>Deteccions</th><th>Àrea mitja</th><th>Àrea màx</th><th>Primer</th><th>Últim</th><th>Après</th></tr>`+
    h.per_id.map(o=>`<tr><td>${esc(o.nom)} (${o.id})</td><td>${o.n}</td><td>${o.area_mitja}</td><td>${o.max_area}</td><td>${tm(o.primer_ms)}</td><td>${tm(o.ultim_ms)}</td><td>${o.apres}</td></tr>`).join('')+`</table>
    <h2>Cronologia de deteccions</h2><div class="tl">${h.cronologia.map(n=>`<i style="height:${n/mx*100}%" title="${n}"></i>`).join('')}</div>`
    :'<p style="color:var(--mut)">Cap detecció registrada.</p>';
  html+=`<h2>Log d'esdeveniments</h2><div style="margin-bottom:6px">`+
    ['ALL','INFO','WARNING','ERROR'].map(n=>`<button class="${n===filtre?'act':''}" data-f="${n}">${n}${n!='ALL'?' ('+d.nivells[n]+')':''}</button>`).join(' ')+`</div><div id="log"></div>
  <h2>Fitxers</h2>`+Object.entries(d.fitxers).map(([n,b])=>`<a href="/download/${d.nom}/${n}">${n}</a> (${kb(b)})`).join(' · ');
  $('#detall').innerHTML=html;
  const t=d.series.t,S=d.series;
  grafic('g1',t,[{v:S.temp,c:'#ff6b6b'}],'°');grafic('g2',t,[{v:S.hum,c:'#4d96ff'}],'%');
  grafic('g3',t,[{v:S.pres,c:'#6bcb77'}],'');grafic('g4',t,[{v:S.rssi,c:'#ffd93d'}],'');
  grafic('g5',t,[{v:S.roll,c:'#ff9f1c'},{v:S.pitch,c:'#c77dff'},{v:S.yaw,c:'#00c9a7'}],'°');
  const pinta=()=>{$('#log').innerHTML=d.eventos.filter(l=>filtre=='ALL'||l.includes('['+filtre+']'))
    .map(l=>{const n=(l.match(/\[(INFO|WARNING|ERROR)\]/)||[])[1]||'INFO';return `<div class="${n}">${esc(l)}</div>`}).join('')};
  pinta();
  document.querySelectorAll('button[data-f]').forEach(b=>b.onclick=()=>{filtre=b.dataset.f;render(d)});
}

async function carrega(){
  if(!sel)return;
  try{ultima=await j('/api/mission/'+sel);render(ultima)}
  catch(e){$('#detall').innerHTML='<p class="ERROR">No s\'ha pogut carregar la missió.</p>'}
}
live();llista();
setInterval(live,2000);setInterval(llista,5000);
setInterval(()=>{if(ultima&&ultima.activa)carrega()},3000);