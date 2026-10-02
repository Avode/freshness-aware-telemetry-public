'use strict';
const $ = id => document.getElementById(id);
let session = null, scene = null, state = null, replaySeq = null, cameraKey = null, refreshing = false, lastResponse = Date.now();
const view = {x: 0, y: -2, scale: 2.6, initialized: false};
const canvas = $('estate'), ctx = canvas.getContext('2d');
const faults = {camera_freeze: 'Camera freeze', imu_dropout: 'IMU dropout', lidar_dropout: 'LiDAR dropout', position_dropout: 'Position dropout', position_jump: 'Reported pose jump', link_down: 'Telemetry link outage'};
const labels = [['maintenance_depot', 'MAINTENANCE DEPOT'], ['logistics_warehouse', 'WAREHOUSE'], ['field_office', 'FIELD OFFICE'], ['pump_house', 'PUMP HOUSE'], ['collection_west', 'COLLECTION WEST'], ['collection_east', 'COLLECTION EAST'], ['reservoir', 'RESERVOIR']];
const fmt = (v, digits=1) => Number.isFinite(v) ? v.toFixed(digits) : '—';
const metric = (id, value, unit) => { $(id).replaceChildren(document.createTextNode(value+' '), Object.assign(document.createElement('small'), {textContent:unit})); };
function badge(el, text, tone='neutral') { el.textContent=text; el.className='badge '+tone; }
function age(stamp) { return stamp == null ? null : Math.max(0, (state?.simulation_time || 0)-stamp); }
async function api(path, value) {
  const controller=new AbortController(), timer=setTimeout(()=>controller.abort(),6000);
  try {
    const response = await fetch(path, {...(value === undefined ? {} : {method:'POST', headers:{'Content-Type':'application/json', 'X-FleetScope-Token':session.token}, body:JSON.stringify(value)}),signal:controller.signal});
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || data.message || 'Request failed');
    return data;
  } finally { clearTimeout(timer); }
}
function text(parent, tag, value, cls='') { const el=document.createElement(tag); el.textContent=value; el.className=cls; parent.append(el); return el; }
function update(data) {
  state=data; const robot=data.robot, health=robot?.health, safety=robot?.safety, mission=robot?.mission;
  $('run-name').textContent=data.run_id;
  $('clock').textContent='SIM '+fmt(data.simulation_time,1)+' s';
  const mode=data.replay?'REPLAY':data.live?'LIVE':robot?'STALE':'WAITING';
  badge($('connection'), mode, data.replay?'warn':data.live?'good':'neutral');
  badge($('robot-health'), !robot?'AWAITING DATA':!data.live&&!data.replay?'STALE':health.status, health?.status==='HEALTHY'?'good':health?.status==='CRITICAL'?'bad':'warn');
  let message = data.replay ? 'Recorded observations · operator commands are disabled. Select Return to live to reconnect the view.' :
    !data.live ? (robot?'Telemetry is stale. Pose, camera and health are last received observations; live health is unknown.':'Waiting for robot telemetry. The robot is initializing its sensors and localization.') :
    safety?.held ? 'Robot held locally: '+safety.reason+'. Clear the fault, then release the hold and dispatch a new mission.' :
    health?.status==='DEGRADED' ? 'Degraded sensor health · '+health.issues.map(i=>i.message).join('; ') : 'Live estate observations · local navigation and edge monitoring are active.';
  if (data.center?.simulation_paused_or_stalled) message+=' Simulation clock is paused or stalled.';
  $('banner').textContent=message; $('banner').className='banner '+(safety?.held&&data.live?'bad':!data.live||health?.status==='DEGRADED'?'warn':'');
  metric('speed', fmt(robot?.state?.speed,2), 'm/s'); metric('pose-age',fmt(age(robot?.state?.stamp)), 's');
  const sensors=health?.sensors || {}; metric('sensor-count',robot?String(Object.values(sensors).filter(s=>s.status==='OK').length):'—','/ 5');
  metric('buffer',robot?.transport ? String(robot.transport.queued) : '—','packets');
  $('sigma').textContent='σ '+fmt(health?.reported_position_sigma_m,2)+' m · motion Δ '+fmt(health?.motion_disagreement_m,2)+' m';
  $('sensors').replaceChildren();
  for (const name of ['camera','imu','lidar','encoders','position']) {
    const s=sensors[name], row=document.createElement('tr'); text(row,'td',name);
    const cell=text(row,'td',''); badge(text(cell,'span',''),s?.status || 'WAITING', s?.status==='OK'?'good':s?'warn':'neutral');
    text(row,'td',fmt(s?.capture_age_s)+' s'); text(row,'td',fmt(s?.rate_hz)); $('sensors').append(row);
  }
  const ca=age(robot?.camera_stamp); $('camera-age').textContent=fmt(ca)+' s old';
  $('camera-state').textContent=!robot?.camera_stamp?'NO FRAME':data.replay?'RECORDED FRAME':!data.live||ca>2?'LAST RECEIVED FRAME':'RECEIVED CAMERA';
  if (robot?.camera_stamp != null) {
    const key=data.run_id+':'+robot.camera_stamp;
    if (key!==cameraKey) { cameraKey=key; $('camera').src='/api/camera?seq='+robot.seq; }
  } else { cameraKey=null; $('camera').hidden=true; $('camera-placeholder').hidden=false; }
  badge($('mission-status'), mission?.status || 'IDLE', mission?.status==='SUCCEEDED'?'good':mission?.status==='FAILED'?'bad':'neutral');
  $('checkpoints').replaceChildren();
  for (const [i, point] of (mission?.checkpoints || []).entries()) {
    const row=document.createElement('div'); row.className='checkpoint '+point.status.toLowerCase();
    text(row,'span',point.status==='REACHED'?'✓':String(i+1),'number'); text(row,'span',point.name); text(row,'small',point.status); $('checkpoints').append(row);
  }
  if (!mission?.checkpoints?.length) text($('checkpoints'),'p','Choose a route to begin.','muted small');
  $('safety').textContent=safety ? (safety.held ? 'HOLD · '+safety.reason : 'Motion enabled · '+(robot.drive_status || 'ready').replaceAll('_',' ')) : 'Waiting for safety supervisor';
  const enabled=session?.writable&&data.live&&!data.replay, busy=['RUNNING','WAITING_FOR_NAV2','CANCELING'].includes(mission?.status);
  $('start').disabled=!enabled||safety?.held||busy; $('cancel').disabled=!enabled||!busy; $('hold').disabled=!enabled||safety?.held;
  $('resume').disabled=!enabled||!safety?.held||!safety?.release_ready;
  const bounds=data.bounds; $('timeline').disabled=!bounds.count;
  $('timeline').min=bounds.first || 0; $('timeline').max=bounds.last || 1; $('timeline').value=data.replay?(robot?.seq || 0):(bounds.last || 1);
  $('replay-label').textContent=data.replay?'HISTORICAL OBSERVATIONS':'LIVE OBSERVATIONS';
  $('history-count').textContent=bounds.count.toLocaleString()+' recorded observations';
  $('snapshot-time').textContent=robot?'#'+robot.seq+' · '+fmt(robot.stamp)+' s':'—';
  $('go-live').disabled=!data.replay;
  $('events').replaceChildren();
  for (const event of data.events) {
    const row=document.createElement('button'); row.className='event '+event.kind;
    text(row,'span',fmt(event.stamp)+' s','event-time'); text(row,'span',event.message,'event-message');
    row.title='Replay received observations at this event'; row.addEventListener('click',()=>seek(event.seq)); $('events').append(row);
  }
  if (!data.events.length) text($('events'),'p','No incidents in the received history.','muted small');
  $('commands').replaceChildren();
  for (const command of data.commands) {
    const row=document.createElement('div'); row.className='command';
    text(row,'strong',command.request.action.toUpperCase()+(command.request.route?' / '+command.request.route:''));
    badge(text(row,'span',''), command.status,command.status==='ACKNOWLEDGED'?'good':command.status==='REJECTED'?'bad':'warn');
    text(row,'span',command.reason); $('commands').append(row);
  }
  if (!data.commands.length) text($('commands'),'span',data.replay?'Commands are disabled in replay.':'No operator commands in this run.','muted small');
  for (const button of $('faults').querySelectorAll('button')) button.disabled=!session?.writable||data.replay;
  draw();
}
async function refresh() {
  if(refreshing)return;
  refreshing=true;const selected=replaySeq;
  try { const data=await api('/api/state'+(selected===null?'':'?seq='+selected));lastResponse=Date.now();if(selected===replaySeq)update(data); }
  catch(error) {
    if(selected!==replaySeq)return;
    badge($('connection'),'SERVER OFFLINE','bad'); $('banner').textContent='Command-center connection lost. Displayed observations are frozen. '+error.message;
    $('banner').className='banner bad'; for (const id of ['start','cancel','hold','resume']) $(id).disabled=true;
    for(const button of $('faults').querySelectorAll('button'))button.disabled=true;
  } finally { refreshing=false;if(selected!==replaySeq)refresh(); }
}
function seek(seq) { replaySeq=Number(seq); refresh(); }
let seekTimer;
$('timeline').addEventListener('input',()=>{ clearTimeout(seekTimer); const seq=$('timeline').value; seekTimer=setTimeout(()=>seek(seq),80); });
$('go-live').addEventListener('click',()=>{replaySeq=null;refresh();});
$('camera').addEventListener('load',()=>{$('camera').hidden=false;$('camera-placeholder').hidden=true;});
$('camera').addEventListener('error',()=>{$('camera').hidden=true;$('camera-placeholder').hidden=false;});
for(const action of ['start','cancel','hold','resume']) $(action).addEventListener('click',async()=>{
  $(action).disabled=true; $('action-result').textContent='Sending '+action+'…';
  try { const result=await api('/api/commands',{action,route:action==='start'?$('route').value:undefined,request_id:crypto.randomUUID()}); $('action-result').textContent=result.message; }
  catch(error) { $('action-result').textContent=error.message; }
  refresh();
});
for (const [fault,label] of Object.entries(faults)) {
  const control=document.createElement('div');control.className='fault-control';text(control,'span',label);
  const buttons=text(control,'div','');
  for (const active of [true,false]) {
    const button=text(buttons,'button',active?'Inject':'Clear'); button.disabled=true;button.setAttribute('aria-label',(active?'Inject ':'Clear ')+label.toLowerCase());
    button.addEventListener('click',async()=>{
      button.disabled=true;
      try { const result=await api('/api/simulation',{fault,active}); $('fault-result').textContent=result.message; }
      catch(error){$('fault-result').textContent=error.message;}
      refresh();
    });
  } $('faults').append(control);
}
$('export').addEventListener('click',()=>{
  const blob=new Blob([JSON.stringify({run:state?.run_id,events:state?.events},null,2)],{type:'application/json'});
  const url=URL.createObjectURL(blob), anchor=document.createElement('a');anchor.href=url;anchor.download='fleetscope-events.json';anchor.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
});
function fit(){ const r=canvas.getBoundingClientRect(); view.scale=Math.min(r.width/210,r.height/205);view.x=0;view.y=0;view.initialized=true;draw(); }
function focus(){const p=state?.robot?.state?.pose;if(p){view.x=p[0];view.y=p[1];view.scale=9;draw();}}
$('fit').onclick=fit;$('follow').onclick=focus;$('focus-husky').onclick=focus;
$('zoom-in').onclick=()=>{view.scale=Math.min(30,view.scale*1.3);draw();};$('zoom-out').onclick=()=>{view.scale=Math.max(1,view.scale/1.3);draw();};
for(const id of ['show-returns','show-labels'])$(id).onchange=draw;
let dragging=null;
canvas.addEventListener('pointerdown',e=>{dragging={x:e.clientX,y:e.clientY,wx:view.x,wy:view.y};canvas.setPointerCapture(e.pointerId);});
canvas.addEventListener('pointermove',e=>{if(dragging){view.x=dragging.wx-(e.clientX-dragging.x)/view.scale;view.y=dragging.wy+(e.clientY-dragging.y)/view.scale;draw();}});
canvas.addEventListener('pointerup',()=>dragging=null);canvas.addEventListener('pointercancel',()=>dragging=null);
canvas.addEventListener('wheel',e=>{e.preventDefault();view.scale=Math.max(1,Math.min(30,view.scale*Math.exp(-e.deltaY*.001)));draw();},{passive:false});
const resizeObserver=new ResizeObserver(()=>{if(!view.initialized)fit();else draw();});resizeObserver.observe(canvas);
function draw(){
  const r=canvas.getBoundingClientRect(),dpr=window.devicePixelRatio||1,w=r.width,h=r.height;if(!w||!h)return;
  if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
  ctx.setTransform(dpr,0,0,dpr,0,0);ctx.fillStyle='#dce2cf';ctx.fillRect(0,0,w,h);
  const s=view.scale;ctx.translate(w/2,h/2);ctx.scale(s,-s);ctx.translate(-view.x,-view.y);
  ctx.strokeStyle='#bec9b2';ctx.lineWidth=.35/s;ctx.beginPath();for(let i=-100;i<=100;i+=10){ctx.moveTo(i,-95);ctx.lineTo(i,95);ctx.moveTo(-100,i);ctx.lineTo(100,i);}ctx.stroke();
  for(const e of scene?.entities || []){
    if(e.name==='estate_ground'||e.name.startsWith('palm_'))continue;
    ctx.save();ctx.translate(e.pos[0],e.pos[1]);ctx.rotate(e.yaw);
    for(const shape of e.shapes){
      ctx.save();ctx.translate(shape.pos[0],shape.pos[1]);ctx.rotate(shape.rotation?.[2]||0);
      const color=shape.color.map((c,i)=>i<3?Math.round(c*255):c);ctx.fillStyle=`rgba(${color.join(',')})`;
      if(shape.kind==='box'){ctx.fillRect(-shape.size[0]/2,-shape.size[1]/2,shape.size[0],shape.size[1]);}
      else if(shape.kind==='cylinder'){ctx.beginPath();ctx.arc(0,0,shape.size[0],0,Math.PI*2);ctx.fill();}
      ctx.restore();
    }ctx.restore();
  }
  for(const e of scene?.entities || [])if(e.name.startsWith('palm_')){
    ctx.save();ctx.translate(e.pos[0],e.pos[1]);ctx.fillStyle='#526f4899';ctx.beginPath();ctx.arc(.5,-.5,2.4,0,Math.PI*2);ctx.fill();
    for(let i=0;i<7;i++){ctx.rotate(Math.PI*2/7);ctx.fillStyle=i%2?'#467454':'#648359';ctx.beginPath();ctx.ellipse(1.2,0,1.9,.58,0,0,Math.PI*2);ctx.fill();}ctx.restore();
  }
  const line=(points,color,width,dash=[])=>{if(!points?.length)return;ctx.strokeStyle=color;ctx.lineWidth=width/s;ctx.setLineDash(dash.map(x=>x/s));ctx.beginPath();points.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.stroke();ctx.setLineDash([]);};
  line(state?.trail,'#b69339aa',2);
  line(state?.robot?.planned_path?.points,'#176f76',2,[5,4]);
  if($('show-returns').checked){ctx.fillStyle='#9b62bcbb';for(const p of state?.robot?.returns||[])ctx.fillRect(p[0]-.16,p[1]-.16,.32,.32);}
  for(const [x,y,label] of [[-17,-33,'GO2'],[9,-33,'X500']]){ctx.strokeStyle='#64716a';ctx.lineWidth=1/s;ctx.setLineDash([3/s,3/s]);ctx.beginPath();ctx.arc(x,y,2.1,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);labelAt(label,x,y-3,'#58675e',9);}
  for(const [i,point]of (state?.robot?.mission?.checkpoints||[]).entries()){
    const [x,y]=point.pose;ctx.fillStyle=point.status==='REACHED'?'#447259':'#e9d296';ctx.strokeStyle='#536956';ctx.lineWidth=1/s;ctx.beginPath();ctx.arc(x,y,7/s,0,Math.PI*2);ctx.fill();ctx.stroke();labelAt(String(i+1),x,y,'#203c32',9);
  }
  const robot=state?.robot?.state;
  if(robot){const [x,y,a]=robot.pose;ctx.save();ctx.translate(x,y);
    const c=robot.covariance_xy||[0,0,0,0],b=(c[1]+c[2])/2,disc=Math.sqrt((c[0]-c[3])**2+4*b*b),major=Math.sqrt(Math.max(.01,(c[0]+c[3]+disc)/2)),minor=Math.sqrt(Math.max(.01,(c[0]+c[3]-disc)/2));
    ctx.save();ctx.rotate(.5*Math.atan2(2*b,c[0]-c[3]));ctx.fillStyle='#bb91312b';ctx.strokeStyle='#ad7c2d99';ctx.lineWidth=1/s;ctx.beginPath();ctx.ellipse(0,0,2*major,2*minor,0,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.restore();
    const positionFresh=state.live&&age(robot.stamp)<=1.&&state.robot.health?.sensors?.position?.status==='OK';ctx.rotate(a);ctx.fillStyle=positionFresh?'#e4b748':'#a4a590';ctx.strokeStyle='#413a26';ctx.lineWidth=1.5/s;const size=8/s;ctx.beginPath();ctx.moveTo(size,0);ctx.lineTo(-size,-size*.7);ctx.lineTo(-size*.5,0);ctx.lineTo(-size,size*.7);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();labelAt('HUSKY 01'+(!state.replay&&!positionFresh?' · LAST POSE':''),x,y-14/s,'#3b3923',10,true);
  }
  if($('show-labels').checked)for(const [name,label]of labels){const e=scene?.entities.find(e=>e.name===name);if(e)labelAt(label,e.pos[0],e.pos[1]+7,'#2d4b3b',8);}
  labelAt('N',view.x+(w/2-24)/s,view.y+(h/2-24)/s,'#3b5140',12,true);
  const metres=s>8?5:20;$('map-scale').textContent=metres+' m';$('map-scale').style.width=(metres*s)+'px';
  function labelAt(label,x,y,color,size,bold=false){ctx.save();ctx.translate(x,y);ctx.scale(1/s,-1/s);ctx.font=`${bold?'600':'500'} ${size}px system-ui`;ctx.textAlign='center';ctx.textBaseline='middle';ctx.lineWidth=3;ctx.strokeStyle='#e0e6d3cc';ctx.strokeText(label,0,0);ctx.fillStyle=color;ctx.fillText(label,0,0);ctx.restore();}
}
async function init(){try{[session,scene]=await Promise.all([api('/api/session'),api('/api/scene')]);await refresh();setInterval(()=>{
  if(Date.now()-lastResponse>3000){badge($('connection'),'SERVER STALE','warn');$('banner').textContent='No recent response from the command center. Displayed observations are frozen.';for(const id of ['start','cancel','hold','resume'])$(id).disabled=true;for(const button of $('faults').querySelectorAll('button'))button.disabled=true;}
  refresh();
},1000);draw();}catch(error){$('banner').textContent=error.message;}}
init();
