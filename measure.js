// ════════════════════════════════════════════════════════════════
//  Separates Mess-Fenster — vollständig isolierte Three.js-Szene
//  Kopiert selektierte STL-Meshes, misst relativ zum Roboter-Base.
//  Keine Verbindung zum Haupt-Renderer → keine Gimbal/Event-Konflikte.
// ════════════════════════════════════════════════════════════════
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

let mwScene, mwCamera, mwRenderer, mwControls, mwRaf = null;
let mwBaseInv = new THREE.Matrix4();   // Welt → Base-Koordinaten
let mwMode = 'pp';
let mwP1 = null, mwP2 = null;
let mwMarkers = [];
let mwCircle = [];
let mwInited = false;

function $(id) { return document.getElementById(id); }

// ── Fenster öffnen: selektierte Meshes + Base übergeben ───────────
window.openMeasureWindow = function(selectedMeshes, baseMatrixWorld) {
  const win = $('measureWindow');
  if (!win) return;
  win.style.display = 'flex';

  // Base-Inverse für Koordinatenbezug
  mwBaseInv.copy(baseMatrixWorld || new THREE.Matrix4()).invert();

  if (!mwInited) _mwInit();

  // Szene leeren (alte Mess-Geometrie + kopierte Meshes entfernen)
  _mwClearScene();
  _mwResetMeasure();

  // Selektierte Meshes als Klone einfügen
  const box = new THREE.Box3();
  let count = 0;
  for (const src of (selectedMeshes || [])) {
    if (!src || !src.geometry) continue;
    src.updateMatrixWorld(true);
    const geo = src.geometry.clone();
    geo.applyMatrix4(src.matrixWorld);   // in Weltkoordinaten backen
    const mat = new THREE.MeshStandardMaterial({ color: 0x5a8fcf, metalness: .3, roughness: .55, side: THREE.DoubleSide });
    const m = new THREE.Mesh(geo, mat);
    m.userData.mwClone = true;
    // Kanten
    const edges = new THREE.LineSegments(
      new THREE.EdgesGeometry(geo, 25),
      new THREE.LineBasicMaterial({ color: 0x000000, transparent: true, opacity: .3 })
    );
    edges.userData.mwClone = true;
    m.add(edges);
    mwScene.add(m);
    box.expandByObject(m);
    count++;
  }

  if (count === 0) {
    $('mw-hint').textContent = 'Keine Objekte selektiert — Fenster schließen, Objekt anklicken, erneut öffnen';
  } else {
    $('mw-hint').textContent = 'Klick: P1 setzen';
  }

  // Roboter-Base-Ursprung als kleines Achsenkreuz anzeigen
  const baseOrigin = new THREE.Vector3().setFromMatrixPosition(baseMatrixWorld || new THREE.Matrix4());
  const axes = new THREE.AxesHelper(150);
  axes.position.copy(baseOrigin);
  axes.userData.mwClone = true;
  mwScene.add(axes);

  // Kamera auf Inhalt zentrieren
  if (!box.isEmpty()) {
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3()).length();
    mwControls.target.copy(center);
    mwCamera.position.copy(center).add(new THREE.Vector3(size, size, size).multiplyScalar(0.6));
    mwCamera.near = size / 1000; mwCamera.far = size * 100;
    mwCamera.updateProjectionMatrix();
  }
  mwControls.update();

  _mwResize();
  if (!mwRaf) _mwAnimate();
};

function _mwInit() {
  const canvas = $('mw-canvas');
  mwScene = new THREE.Scene();
  mwScene.background = new THREE.Color(0x0d1825);

  mwCamera = new THREE.PerspectiveCamera(50, 1, 1, 100000);
  mwCamera.position.set(800, 800, 800);

  mwRenderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  mwRenderer.setPixelRatio(window.devicePixelRatio);

  mwControls = new OrbitControls(mwCamera, canvas);
  mwControls.enableDamping = true;
  mwControls.dampingFactor = 0.1;

  // Licht
  mwScene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 1.6));
  const dir = new THREE.DirectionalLight(0xffffff, 2.2);
  dir.position.set(1, 2, 1);
  mwScene.add(dir);
  const grid = new THREE.GridHelper(4000, 40, 0x223344, 0x1a2530);
  grid.rotation.x = Math.PI / 2;  // Z-up
  mwScene.add(grid);

  // Klick-Messung (auf Canvas, isoliert)
  canvas.addEventListener('pointerup', _mwPick);
  canvas.addEventListener('pointerdown', _mwPickDown);

  // Buttons
  $('mw-close').addEventListener('click', closeMeasureWindow);
  $('mw-reset').addEventListener('click', () => { _mwResetMeasure(); $('mw-hint').textContent = 'Klick: P1 setzen'; });
  $('mw-mode-pp').addEventListener('click', () => _mwSetMode('pp'));
  $('mw-mode-3c').addEventListener('click', () => _mwSetMode('3c'));

  // Ansichts-Buttons
  document.querySelectorAll('.mw-view').forEach(btn => {
    btn.addEventListener('click', () => _mwSetView(btn.dataset.view));
  });

  // Fenster verschieben per Titelleiste
  const win = $('measureWindow');
  const bar = $('mw-titlebar');
  let dragging = false, dragOX = 0, dragOY = 0, startL = 0, startT = 0;
  bar.addEventListener('pointerdown', e => {
    if (e.target.tagName === 'BUTTON') return;
    dragging = true;
    dragOX = e.clientX; dragOY = e.clientY;
    const rect = win.getBoundingClientRect();
    startL = rect.left; startT = rect.top;
    // bei erstem Verschieben rechts/unten-Positionierung auf px fixieren
    win.style.left = startL + 'px'; win.style.top = startT + 'px';
    win.style.right = 'auto'; win.style.bottom = 'auto';
    bar.setPointerCapture(e.pointerId);
    e.preventDefault();
  });
  bar.addEventListener('pointermove', e => {
    if (!dragging) return;
    win.style.left = (startL + e.clientX - dragOX) + 'px';
    win.style.top  = (startT + e.clientY - dragOY) + 'px';
  });
  bar.addEventListener('pointerup', e => { dragging = false; try { bar.releasePointerCapture(e.pointerId); } catch(_){} });

  // Resize des Fensters → Renderer/Canvas anpassen
  if (window.ResizeObserver) {
    const ro = new ResizeObserver(() => { if (win.style.display !== 'none') _mwResize(); });
    ro.observe(win);
  }

  window.addEventListener('resize', () => { if ($('measureWindow').style.display !== 'none') _mwResize(); });

  mwInited = true;
}

window.closeMeasureWindow = function() {
  const win = $('measureWindow');
  if (win) win.style.display = 'none';
  if (mwRaf) { cancelAnimationFrame(mwRaf); mwRaf = null; }
  _mwClearScene();
  _mwResetMeasure();
};

function _mwClearScene() {
  if (!mwScene) return;
  const toRemove = [];
  mwScene.traverse(o => { if (o.userData && o.userData.mwClone) toRemove.push(o); });
  toRemove.forEach(o => {
    if (o.geometry) o.geometry.dispose();
    if (o.material) { if (Array.isArray(o.material)) o.material.forEach(m=>m.dispose()); else o.material.dispose(); }
    mwScene.remove(o);
  });
}

function _mwResetMeasure() {
  mwP1 = null; mwP2 = null;
  mwMarkers.forEach(m => mwScene && mwScene.remove(m));
  mwMarkers = [];
  mwCircle = [];
  ['mw-dist','mw-dx','mw-dy','mw-dz','mw-p1'].forEach(id => { const e = $(id); if (e) e.textContent = '—'; });
}

function _mwSetMode(mode) {
  mwMode = mode;
  $('mw-mode-pp').style.background = mode==='pp' ? 'rgba(37,99,235,.4)' : 'rgba(255,255,255,.05)';
  $('mw-mode-pp').style.color = mode==='pp' ? '#90c0ff' : '#6a8fa8';
  $('mw-mode-3c').style.background = mode==='3c' ? 'rgba(37,99,235,.4)' : 'rgba(255,255,255,.05)';
  $('mw-mode-3c').style.color = mode==='3c' ? '#90c0ff' : '#6a8fa8';
  _mwResetMeasure();
  $('mw-hint').textContent = mode==='pp' ? 'Klick: P1 setzen' : '3 Punkte auf Kreis klicken';
}

let mwDownX = 0, mwDownY = 0;
function _mwPickDown(e) { mwDownX = e.clientX; mwDownY = e.clientY; }

function _mwMarker(pos, color) {
  const m = new THREE.Mesh(
    new THREE.SphereGeometry(8, 12, 10),
    new THREE.MeshBasicMaterial({ color, depthTest: false })
  );
  m.position.copy(pos); m.renderOrder = 999;
  mwScene.add(m); mwMarkers.push(m);
  return m;
}

function _mwLine(a, b) {
  const g = new THREE.BufferGeometry().setFromPoints([a, b]);
  const l = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0xffcc00, depthTest: false }));
  l.renderOrder = 998;
  mwScene.add(l); mwMarkers.push(l);
}

function _mwToBase(pt) {
  return pt.clone().applyMatrix4(mwBaseInv);
}

function _mwShowResult(p1, p2) {
  const lp1 = _mwToBase(p1), lp2 = _mwToBase(p2);
  const dx = lp2.x-lp1.x, dy = lp2.y-lp1.y, dz = lp2.z-lp1.z;
  const dist = Math.sqrt(dx*dx+dy*dy+dz*dz);
  const r = v => Math.round(v);
  $('mw-dist').textContent = r(dist)+' mm';
  $('mw-dx').textContent = r(dx)+' mm';
  $('mw-dy').textContent = r(dy)+' mm';
  $('mw-dz').textContent = r(dz)+' mm';
  $('mw-p1').textContent = `${r(lp1.x)}, ${r(lp1.y)}, ${r(lp1.z)}`;
}

function _mwPick(event) {
  if (Math.abs(event.clientX-mwDownX)+Math.abs(event.clientY-mwDownY) > 4) return; // war Drehen
  if (event.button !== 0) return;

  const rect = mwRenderer.domElement.getBoundingClientRect();
  const mx = ((event.clientX-rect.left)/rect.width)*2-1;
  const my = -((event.clientY-rect.top)/rect.height)*2+1;
  const rc = new THREE.Raycaster();
  rc.setFromCamera(new THREE.Vector2(mx, my), mwCamera);
  const targets = [];
  mwScene.traverse(o => { if (o.isMesh && o.userData.mwClone) targets.push(o); });
  const hits = rc.intersectObjects(targets, false);
  if (!hits.length) return;
  const pt = hits[0].point.clone();

  if (mwMode === 'pp') {
    if (!mwP1) {
      mwP1 = pt; _mwMarker(pt, 0xff4444);
      const lp = _mwToBase(pt);
      $('mw-p1').textContent = `${Math.round(lp.x)}, ${Math.round(lp.y)}, ${Math.round(lp.z)}`;
      $('mw-hint').textContent = 'Klick: P2 setzen';
    } else {
      mwP2 = pt; _mwMarker(pt, 0x44ff88);
      _mwLine(mwP1, mwP2);
      _mwShowResult(mwP1, mwP2);
      $('mw-hint').textContent = 'Neuer Klick: neue Messung';
      mwP1 = null; mwP2 = null;
    }
  } else {
    // 3-Punkt-Kreis
    const colors = [0xff4444, 0xff8844, 0xffcc44];
    _mwMarker(pt, colors[mwCircle.length] || 0xffffff);
    mwCircle.push(pt);
    if (mwCircle.length === 3) {
      const ctr = _mwCircumcenter(mwCircle[0], mwCircle[1], mwCircle[2]);
      _mwMarker(ctr, 0x00ddff);
      const r = ctr.distanceTo(mwCircle[0]);
      // Kreislinie
      const n = mwCircle[1].clone().sub(mwCircle[0]).cross(mwCircle[2].clone().sub(mwCircle[0])).normalize();
      const u = mwCircle[0].clone().sub(ctr).normalize();
      const v = n.clone().cross(u);
      const pts = [];
      for (let i = 0; i <= 64; i++) { const a = (i/64)*Math.PI*2; pts.push(ctr.clone().addScaledVector(u, Math.cos(a)*r).addScaledVector(v, Math.sin(a)*r)); }
      const cl = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: 0x00ddff, depthTest: false }));
      cl.renderOrder = 998; mwScene.add(cl); mwMarkers.push(cl);
      // Linie Mittelpunkt → Base-Ursprung
      const baseOrigin = new THREE.Vector3().setFromMatrixPosition(mwBaseInv.clone().invert());
      _mwLine(baseOrigin, ctr);
      _mwShowResult(baseOrigin, ctr);
      const lc = _mwToBase(ctr);
      $('mw-hint').textContent = `Kreis r=${Math.round(r)}mm  M=(${Math.round(lc.x)}, ${Math.round(lc.y)}, ${Math.round(lc.z)}) — neue Messung: 3 Punkte`;
      mwCircle = [];
    } else {
      $('mw-hint').textContent = `Punkt ${mwCircle.length}/3`;
    }
  }
}

function _mwCircumcenter(p1, p2, p3) {
  const v1 = p2.clone().sub(p1), v2 = p3.clone().sub(p1);
  const n = v1.clone().cross(v2).normalize();
  const u = v1.clone().normalize();
  const w = n.clone().cross(u).normalize();
  const bx = v1.dot(u), by = v1.dot(w);
  const cx = v2.dot(u), cy = v2.dot(w);
  const d = 2*(bx*cy - by*cx);
  if (Math.abs(d) < 1e-9) return p1.clone().add(p2).add(p3).divideScalar(3);
  const ux = (cy*(bx*bx+by*by) - by*(cx*cx+cy*cy)) / d;
  const uy = (bx*(cx*cx+cy*cy) - cx*(bx*bx+by*by)) / d;
  return p1.clone().addScaledVector(u, ux).addScaledVector(w, uy);
}

function _mwSetView(view) {
  if (!mwScene || !mwCamera || !mwControls) return;
  // Bounding-Box aller kopierten Meshes
  const box = new THREE.Box3();
  mwScene.traverse(o => { if (o.isMesh && o.userData.mwClone) box.expandByObject(o); });
  if (box.isEmpty()) return;
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z) || 500;
  const dist = maxDim * 1.8;

  // Z-up Koordinatensystem (wie RobModel)
  const dirs = {
    top:    new THREE.Vector3(0, 0, 1),
    bottom: new THREE.Vector3(0, 0, -1),
    front:  new THREE.Vector3(0, -1, 0),
    back:   new THREE.Vector3(0, 1, 0),
    right:  new THREE.Vector3(1, 0, 0),
    left:   new THREE.Vector3(-1, 0, 0),
    iso:    new THREE.Vector3(1, -1, 1).normalize(),
  };
  const d = dirs[view] || dirs.iso;
  mwCamera.position.copy(center).addScaledVector(d, dist);
  // up-Vektor: bei top/bottom auf Y, sonst Z
  if (view === 'top' || view === 'bottom') mwCamera.up.set(0, 1, 0);
  else mwCamera.up.set(0, 0, 1);
  mwControls.target.copy(center);
  mwControls.update();

  // Button-Highlight
  document.querySelectorAll('.mw-view').forEach(b => {
    const on = b.dataset.view === view;
    b.style.background = on ? 'rgba(37,99,235,.4)' : 'rgba(255,255,255,.05)';
    b.style.color = on ? '#90c0ff' : '#a0b0c0';
    b.style.borderColor = on ? 'rgba(37,99,235,.6)' : 'rgba(255,255,255,.15)';
  });
}

function _mwResize() {
  if (!mwRenderer) return;
  const canvas = $('mw-canvas');
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (w === 0 || h === 0) return;
  mwRenderer.setSize(w, h, false);
  mwCamera.aspect = w / h;
  mwCamera.updateProjectionMatrix();
}

function _mwAnimate() {
  mwRaf = requestAnimationFrame(_mwAnimate);
  if (mwControls) mwControls.update();
  if (mwRenderer && mwScene && mwCamera) mwRenderer.render(mwScene, mwCamera);
}
