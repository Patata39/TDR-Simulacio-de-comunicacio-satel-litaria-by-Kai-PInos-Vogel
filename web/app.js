const $=s=>document.querySelector(s);
const esc=s=>String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
const f=(v,d=1)=>v==null?'—':Number(v).toFixed(d);
const tm=ms=>{const s=Math.floor(ms/1000);return Math.floor(s/60)+':'+String(s%60).padStart(2,'0')};
const kb=b=>b>1024?(b/1024).toFixed(1)+' KB':b+' B';
async function j(u){const r=await fetch(u);if(!r.ok)throw new Error(r.status);return r.json()}
let sel=null,filtre='ALL',ultima=null;
let mostraPics=false,cmpNom='',cmpDades=null,llistaM=[];
let alertesOn=false;try{alertesOn=localStorage.getItem('alertes')==='1'}catch(e){}
const prev={sensor:null,activa:false,caigut:false},mal={s:0,c:0},titolOrig=document.title;
let blinkId=null;

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
    viuPanell(d);
    comprovaAlertes(d);   
  }catch(e){
    $('#live').innerHTML='<span class="pill bad">○ servidor sense resposta</span>';
    if(alertesOn&&!prev.caigut){prev.caigut=true;avis('El servidor no respon','err')}  
  }
}

async function llista(){
  try{
    const ms=await j('/api/missions');
    llistaM=ms;  
    $('#llista').innerHTML=ms.map(m=>`<div class="m ${m.nom===sel?'sel':''}" data-n="${m.nom}">
      ${m.activa?'<span style="color:var(--ok)">●</span> ':''}${esc(m.etiqueta)}
      <small>${esc(m.duracio)} · ${m.n_reg} deteccions</small></div>`).join('')||'<p style="color:var(--mut)">Cap missió guardada.</p>';
    document.querySelectorAll('.m').forEach(e=>e.onclick=()=>{sel=e.dataset.n;carrega();llista()});
  }catch(e){}
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
    pintaGrafics(d);
  const pinta=()=>{$('#log').innerHTML=d.eventos.filter(l=>filtre=='ALL'||l.includes('['+filtre+']'))
    .map(l=>{const n=(l.match(/\[(INFO|WARNING|ERROR)\]/)||[])[1]||'INFO';return `<div class="${n}">${esc(l)}</div>`}).join('')};
  pinta();
  document.querySelectorAll('button[data-f]').forEach(b=>b.onclick=()=>{filtre=b.dataset.f;render(d)});
}

function grafic(id,t,linies,unit,cursor,mq){
  const c=document.getElementById(id);if(!c)return;
  c._g={t,linies,unit,mq};
  const W=c.width=c.clientWidth,H=c.height=110,x=c.getContext('2d');
  x.clearRect(0,0,W,H);x.font='10px Consolas';
  // els pics entren al rang de l'eix: la sèrie reduïda pot haver perdut l'extrem. No solucionaré això jaja.
  const tots=linies.flatMap(l=>[...l.v.filter(a=>a!=null),...(mostraPics&&l.p?l.p.map(p=>p.v):[])]);
  if(!tots.length){x.fillStyle='#8890a8';x.fillText('sense dades',8,20);return}
  let mn=Math.min(...tots),mx=Math.max(...tots);if(mn==mx){mn-=1;mx+=1}
  const T=Math.max(1,...linies.map(l=>{const tl=l.t||t;return tl[tl.length-1]||0})),X0=38,AW=W-X0-4;
  const Y=a=>H-14-(a-mn)/(mx-mn)*(H-22);
  x.strokeStyle='#2a3a66';x.strokeRect(X0,4,AW,H-18);
  x.fillStyle='#8890a8';x.fillText(f(mx)+unit,0,12);x.fillText(f(mn)+unit,0,H-8);
  x.fillText(tm(T),W-30,H-2);
  (mq||[]).forEach(m=>{if(m.ms>T)return;
    const px=X0+m.ms/T*AW;
    x.strokeStyle=m.n=='ERROR'?'#ff6b6b':'#ff9f1c';x.setLineDash([3,3]);
    x.beginPath();x.moveTo(px,4);x.lineTo(px,H-14);x.stroke();x.setLineDash([])});
  linies.forEach(l=>{
    const tl=l.t||t;
    x.strokeStyle=l.c;x.globalAlpha=l.cmp?0.65:1;x.setLineDash(l.cmp?[5,3]:[]);
    x.beginPath();let on=false;
    l.v.forEach((a,i)=>{if(a==null)return;
      const px=X0+tl[i]/T*AW,py=Y(a);
      on?x.lineTo(px,py):x.moveTo(px,py);on=true});
    x.stroke();x.setLineDash([]);x.globalAlpha=1;
    if(mostraPics&&l.p)l.p.forEach(p=>{  
      x.beginPath();x.arc(X0+p.ms/T*AW,Y(p.v),3.5,0,7);
      x.fillStyle=l.c;x.fill();x.strokeStyle='#fff';x.stroke()});
  });
  if(cursor!=null){
    const ms=cursor*T,px=X0+cursor*AW,tx=px>W*0.6?px-58:px+4;
    x.strokeStyle='#ffffff88';x.beginPath();x.moveTo(px,4);x.lineTo(px,H-14);x.stroke();
    x.fillStyle='#8890a8';x.fillText(tm(ms),tx,12);
    linies.forEach((l,k)=>{const tl=l.t||t;let i=0;while(i<tl.length-1&&tl[i]<ms)i++;
      x.fillStyle=l.c;x.fillText((l.cmp?'B ':'')+f(l.v[i])+unit,tx,23+k*11)});
  }
}

const IDS_G=['g1','g2','g3','g4','g5'];
function marquesLog(eventos){
  if(!eventos.length)return [];
  const ts=l=>new Date(l.slice(0,19).replace(' ','T')).getTime(),t0=ts(eventos[0]);
  return eventos.map(l=>{const n=(l.match(/\[(WARNING|ERROR)\]/)||[])[1];return n&&{n,ms:ts(l)-t0}}).filter(Boolean);
}
// ── NOU: pintaGrafics() amb selector de comparació, checkbox de pics i taula de pics
const VAR_G=[['g1',[['temp','#ff6b6b']],'°'],['g2',[['hum','#4d96ff']],'%'],
  ['g3',[['pres','#6bcb77']],''],['g4',[['rssi','#ffd93d']],''],
  ['g5',[['roll','#ff9f1c'],['pitch','#c77dff'],['yaw','#00c9a7']],'°']];

function taulaPics(d){
  const NOMS={temp:'Temperatura',hum:'Humitat',pres:'Pressió',rssi:'RSSI',roll:'Roll',pitch:'Pitch'};
  const fil=[];
  Object.entries(d.pics||{}).forEach(([k,ps])=>ps.forEach(p=>fil.push({k,...p})));
  fil.sort((a,b)=>Math.abs(b.z)-Math.abs(a.z));
  $('#pics').innerHTML=fil.length
    ?`<h2>Pics detectats (&gt;3σ) <small>${fil.length} · es mostren els 15 més extrems</small></h2>
      <table><tr><th>Variable</th><th>Moment</th><th>Valor</th><th>Desviació (σ)</th></tr>`+
      fil.slice(0,15).map(p=>`<tr><td>${NOMS[p.k]}</td><td>${tm(p.ms)}</td><td>${f(p.v)}</td><td>${f(p.z)}</td></tr>`).join('')+`</table>`
    :'<p style="color:var(--mut)">Cap pic &gt; 3σ en aquesta missió.</p>';
}

function pintaGrafics(d){
  if(cmpNom===d.nom){cmpNom='';cmpDades=null}   // no té sentit comparar-la amb ella mateixa,lol
  const mq=marquesLog(d.eventos),ch=document.querySelector('#detall .charts');
  ch.insertAdjacentHTML('beforebegin',`<div style="margin-bottom:6px">
    <label><input type="checkbox" id="chkPics" ${mostraPics?'checked':''}> Marcar pics (&gt;3σ)</label> ·
    <select id="selCmp"><option value="">Comparar amb…</option>${
      llistaM.filter(m=>m.nom!==d.nom).map(m=>`<option value="${m.nom}" ${m.nom===cmpNom?'selected':''}>${esc(m.etiqueta)}</option>`).join('')}</select>
    <small style="color:var(--mut)">B = discontínua</small></div>`);
  ch.insertAdjacentHTML('afterend','<div id="pics"></div>');

  const dibuixa=()=>{
    VAR_G.forEach(([id,claus,unit])=>{
      const L=claus.map(([k,c])=>({v:d.series[k],c,p:(d.pics||{})[k]}));
      if(cmpDades)claus.forEach(([k,c])=>L.push({v:cmpDades.series[k],t:cmpDades.series.t,c,cmp:true}));
      grafic(id,d.series.t,L,unit,null,mq);
    });
    const redibuixa=fx=>IDS_G.forEach(o=>{const g=document.getElementById(o)._g;grafic(o,g.t,g.linies,g.unit,fx,g.mq)});
    IDS_G.forEach(id=>{const c=document.getElementById(id);
      c.onmousemove=e=>{const r=c.getBoundingClientRect();
        redibuixa(Math.min(1,Math.max(0,(e.clientX-r.left-38)/(r.width-42))))};
      c.onmouseleave=()=>redibuixa(null)});
  };
  document.getElementById('chkPics').onchange=e=>{mostraPics=e.target.checked;dibuixa()};
  document.getElementById('selCmp').onchange=async e=>{
    cmpNom=e.target.value;cmpDades=null;
    if(cmpNom){try{cmpDades=await j('/api/mission/'+cmpNom)}catch(err){cmpNom=''}}
    if(d.nom===sel)dibuixa();   // si mentrestant s'ha canviat de missió, no pintem
  };
  taulaPics(d);dibuixa();
}

//alertes del navegador
function avis(txt,nivell){
  const b=document.getElementById('avisos'),e=document.createElement('div');
  e.className='avis '+nivell;
  e.textContent=new Date().toLocaleTimeString()+' · '+txt;
  e.onclick=()=>e.remove();
  b.prepend(e);while(b.children.length>5)b.lastChild.remove();
  if(document.hidden){
    if(!blinkId){let on=false;blinkId=setInterval(()=>{document.title=(on=!on)?'⚠ '+txt:titolOrig},1000)}
    if(window.Notification&&Notification.permission==='granted')new Notification('COCHECITO',{body:txt});
  }
}
document.addEventListener('visibilitychange',()=>{
  if(!document.hidden&&blinkId){clearInterval(blinkId);blinkId=null;document.title=titolOrig}});

function comprovaAlertes(d){
  const activa=!!d.mision_activa;
  if(alertesOn){
    if(prev.sensor===true&&!d.sensor)
      avis('Sensor desconnectat'+(prev.activa?' — la missió s\'ha finalitzat automàticament':''),'err');
    if(d.sensor&&activa){
      [['s','sensor',d.salut.sensor_rssi,true],['c','càmera',d.salut.cam_rssi,d.cam]].forEach(([k,n,v,ok])=>{
        if(!ok)return;
        if(v<-75){if(++mal[k]===3)avis('RSSI '+n+' deficient: '+v+' dBm','warn')}
        else if(v>-70)mal[k]=0;  
      });
    }else{mal.s=mal.c=0}
  }
  prev.sensor=d.sensor;prev.activa=activa;prev.caigut=false;
}

function pintaBtnAlertes(){
  const b=$('#btnAlertes');
  b.textContent=alertesOn?'🔔 Alertes: on':'🔕 Alertes: off';
  b.className=alertesOn?'act':'';
}
async function toggleAlertes(){
  alertesOn=!alertesOn;
  try{localStorage.setItem('alertes',alertesOn?'1':'0')}catch(e){}
  if(alertesOn&&window.Notification&&Notification.permission==='default')
    await Notification.requestPermission();   // el permís només es pot demanar amb un clic
  pintaBtnAlertes();
}

async function carrega(){
  if(!sel)return;
  try{ultima=await j('/api/mission/'+sel);render(ultima)}
  catch(e){$('#detall').innerHTML='<p class="ERROR">No s\'ha pogut carregar la missió.</p>'}
}
live();llista();
setInterval(live,2000);setInterval(llista,5000);
setInterval(()=>{if(ultima&&ultima.activa)carrega()},3000);
pintaBtnAlertes(); 
live();llista();