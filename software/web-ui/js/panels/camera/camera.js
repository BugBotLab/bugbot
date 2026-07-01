import { defaultCamURLFromWS } from '../../shared/utils.js';

// DOM in this panel
const root         = document.getElementById('panel-camera');
const camUrlInput  = root.querySelector('#camUrl');
const camImg       = root.querySelector('#camStream');
const overlay      = root.querySelector('#camOverlay');
const octx         = overlay.getContext('2d');
const camStartBtn  = root.querySelector('#camStart');
const camStopBtn   = root.querySelector('#camStop');
const camSnapBtn   = root.querySelector('#camSnap');
const visionModeSel= root.querySelector('#visionMode');
const downloadBtn  = root.querySelector('#downloadFrame');

// hydrate defaults (wsUrl lives in header, already on page)
const wsUrlEl = document.getElementById('wsUrl');
camUrlInput.value = defaultCamURLFromWS(wsUrlEl.value);
camUrlInput.addEventListener('input', ()=> camUrlInput.dataset.userEdited = '1');

function camBase(){
  try { const u=new URL(camUrlInput.value); return `${u.protocol}//${u.host}`; }
  catch { return ""; }
}

let camRunning=false, camSession=0, camRetryT=null;
let cvReady = false, faceC=null, gray=null, faces=null;
let rafId = 0;

function clearCamRetry(){ if(camRetryT){ clearTimeout(camRetryT); camRetryT=null; } }
function makeStreamUrl(){
  const raw = camUrlInput.value || "http://192.168.4.1:81/mjpeg";
  const u = new URL(raw);
  u.searchParams.set('sid', String(camSession));
  u.searchParams.set('_', String(Date.now()));
  return u.toString();
}
function sizeOverlayToImg(){
  const w = camImg.naturalWidth || 0;
  const h = camImg.naturalHeight || 0;
  if (!w || !h) return false;
  overlay.width = w; overlay.height = h; return true;
}

async function ensureCascade(){
  const cc1 = new cv.CascadeClassifier();
  if (cc1.load('haarcascade_frontalface_default.xml')) { cc1.delete(); return true; }
  cc1.delete();
  try{
    const resp = await fetch('haarcascade_frontalface_default.xml', { cache:'no-store' });
    if (!resp.ok) throw new Error('cascade fetch ' + resp.status);
    const buf = new Uint8Array(await resp.arrayBuffer());
    if (Module.FS && Module.FS.analyzePath && !Module.FS.analyzePath('/haarcascade_frontalface_default.xml').exists) {
      Module.FS_createDataFile('/', 'haarcascade_frontalface_default.xml', buf, true, false, false);
    }
    const cc2 = new cv.CascadeClassifier(); const ok = cc2.load('haarcascade_frontalface_default.xml'); cc2.delete(); return ok;
  } catch { return false; }
}

function startVisionLoop(){
  if (!cvReady) return;
  cancelAnimationFrame(rafId);
  rafId = requestAnimationFrame(visionLoop);
}

function visionLoop(){
  if (visionModeSel.value==='off' || !camRunning || !camImg.naturalWidth){
    octx.clearRect(0,0,overlay.width, overlay.height);
    rafId = requestAnimationFrame(visionLoop);
    return;
  }
  if (!sizeOverlayToImg()){
    rafId = requestAnimationFrame(visionLoop);
    return;
  }

  try{
    const w = overlay.width, h = overlay.height;
    const tmp = document.createElement('canvas'); tmp.width=w; tmp.height=h; const tctx = tmp.getContext('2d');
    tctx.drawImage(camImg, 0,0,w,h);
    const imgData = tctx.getImageData(0,0,w,h);

    const src = cv.matFromImageData(imgData);
    if (!gray)  gray  = new cv.Mat();
    if (!faces) faces = new cv.RectVector();
    if (!faceC) faceC = new cv.CascadeClassifier();

    cv.cvtColor(src, gray, cv.COLOR_RGBA2GRAY);
    faceC.detectMultiScale(gray, faces, 1.2, 3, 0, new cv.Size(30,30));

    octx.clearRect(0,0,w,h);
    octx.lineWidth = 2; octx.strokeStyle = '#4da3ff';
    for (let i=0;i<faces.size();i++){ const r = faces.get(i); octx.strokeRect(r.x,r.y,r.width,r.height); }

    src.delete();
  }catch(e){
    // likely CORS taint from camera
    octx.clearRect(0,0,overlay.width, overlay.height);
  }

  rafId = requestAnimationFrame(visionLoop);
}

// OpenCV readiness: cv calls _main as soon as opencv.js runs (configured in opencv-loader.js)
if (window.cv) markCvReady(); else {
  // wait until cv is defined by opencv.js
  const readyInt = setInterval(()=>{
    if (window.cv) { clearInterval(readyInt); markCvReady(); }
  }, 50);
}

function markCvReady(){
  cvReady = true;
  gray  = new cv.Mat();
  faces = new cv.RectVector();
  faceC = new cv.CascadeClassifier();
  ensureCascade().then(()=>{
    if (!faceC.load('haarcascade_frontalface_default.xml')) {
      console.warn('Cascade failed to load');
    }
    startVisionLoop();
  });
}

// UI actions
camStartBtn.addEventListener('click', ()=>{
  if (camRunning) return;
  clearCamRetry(); camSession++; camRunning=true;

  camImg.crossOrigin = 'anonymous';
  camImg.decoding = 'sync';
  camImg.referrerPolicy = 'no-referrer';

  camImg.onload = ()=> sizeOverlayToImg();
  camImg.onerror = ()=>{
    if (!camRunning) return;
    camRetryT = setTimeout(()=> camImg.src = makeStreamUrl(), 800);
  };

  if (!camImg.isConnected){
    const hidden = document.createElement('div'); hidden.style.display='none'; hidden.appendChild(camImg); document.body.appendChild(hidden);
  }
  camImg.src = makeStreamUrl();
  startVisionLoop();
});

camStopBtn.addEventListener('click', ()=>{
  camRunning=false; camSession++; clearCamRetry();
  camImg.removeAttribute('src'); camImg.src='';
  octx.clearRect(0,0,overlay.width, overlay.height);
});

camSnapBtn.addEventListener('click', ()=>{
  const base = camBase(); if (!base) return alert('Bad camera URL');
  window.open(`${base}/camera.jpg?_=${Date.now()}`, '_blank');
});

downloadBtn.addEventListener('click', ()=>{
  try{
    if (!overlay.width || !overlay.height) return;
    const w=overlay.width, h=overlay.height;
    const tmp=document.createElement('canvas'); tmp.width=w; tmp.height=h; const tctx=tmp.getContext('2d');
    tctx.drawImage(camImg,0,0,w,h);
    tctx.drawImage(overlay,0,0);
    const url = tmp.toDataURL('image/png');
    const a=document.createElement('a'); a.href=url; a.download=`frame_${Date.now()}.png`; a.click();
  }catch{ alert('Save failed (CORS?).'); }
});
