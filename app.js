// ═══════════════════════════════════════════════════
//  RobModel — Robot Kinematik Editor  v38
//  app.js — Haupt-Anwendungslogik (ES Module)
// ═══════════════════════════════════════════════════

import * as THREE from 'three';
import { OrbitControls }    from 'three/addons/controls/OrbitControls.js';
import { STLLoader }        from 'three/addons/loaders/STLLoader.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';

// ── Hilfsfunktionen ──────────────────────────────────────────────
const $ = id => document.getElementById(id);
const qsa = s => [...document.querySelectorAll(s)];
const num = v => { if (v === null || v === undefined || v === '') return null; const n = Number(String(v).replace(',', '.')); return Number.isFinite(n) ? n : null; };
const deg = v => THREE.MathUtils.degToRad(Number(v) || 0);
const fmt = b => b > 1048576 ? (b / 1048576).toFixed(1) + ' MB' : (b / 1024).toFixed(1) + ' KB';
const norm = s => String(s || '').toLowerCase().replace(/\\/g, '/').split('/').pop().replace(/\.stl$/i, '').trim();
const zipName = s => (s || 'RobModel_export').replace(/[^a-z0-9]+/gi, '_').replace(/^_+|_+$/g, '') || 'RobModel_export';
const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

// ── KR8-Zielwerte (default) ───────────────────────────────────────
const KR8_TARGET = [
  { x: 450,  y: 0, z: 150  },  // A1 Rz  — Display X=Three.Z(oben), Display Z=Three.X(horiz)
  { x: 0,    y: 0, z: 610  },  // A2 Ry
  { x: 200,  y: 0, z: 0    },  // A3 Ry
  { x: 0,    y: 0, z: 630  },  // A4 Rx
  { x: 0,    y: 0, z: 80   },  // A5 Ry
  { x: 0,    y: 0, z: 0    },  // A6 Rx
];
function defOffset(i) { return { ...KR8_TARGET[i] }; }

// ── State ─────────────────────────────────────────────────────────
const state = {
  files: [], stls: [], xmls: [], jsons: [],
  buffers: new Map(),
  robotName: '', mode: 'leer',
  packageJson: null,
  toolName: 'tool1_tcp.stl',
  exportStlMode: 'transformed',
  referencePose: [0, -90, 90, 0, 0, 0],
  robotTr: { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 },
  toolTr:  { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 },
  activeTcp: 'auftragen',
  axisPoints: ['A1','A2','A3','A4','A5','A6'].map((name, i) => ({ name, ...defOffset(i), rx: 0, ry: 0, rz: 0, source: 'KR8 Zielwert' })),
  selectedAxis: 0,
  jointAngles: [0, -90, 90, 0, 0, 0],
  axisStlMap: { A1:null, A2:null, A3:null, A4:null, A5:null, A6:null }, // manuelle STL-Zuweisung
  simulation: { active: false, axis: null, raf: null },
  joints: ['A1','A2','A3','A4','A5','A6'].map((n, i) => ({
    name: n,
    axis: ['Rz','Ry','Ry','Rx','Ry','Rx'][i],
    offset: defOffset(i),
    min: null, max: null,
    rotationSign: [-1,1,1,-1,1,-1][i], status: 'KR8 Zielwert'
  })),
  tcp: {
    auftragen: { x: null, y: null, z: null, rz: null, ry: null, rx: null, toolLength: 0, toolStl: '', status: 'manuell' },
    abtragen:  { x: null, y: null, z: null, rz: null, ry: null, rx: null, toolLength: 0, toolStl: '', status: 'manuell' },
  }
};

// ── Three.js Variablen ────────────────────────────────────────────
let scene, camera, renderer, controls, grid, robotGroup, toolGroup, tcpMarker, kinematicsRoot;
let axisPointGroup, axisLine, transformControls, raycaster, mouse, csHelperGroup;
const meshes = new Map();
const axisMeshes = [];
const axisPivotGroups = [];
const skeletonCyls = []; const skeletonSphs = [];
const LINK_R=[28,20,16,12,8,6]; const JOINT_R=[40,38,30,24,20,16];
const LINK_COLOR=0xcc4400; const JOINT_COLOR=0xe8a020;
const loader = new STLLoader();

// ── Fehleranzeige ──────────────────────────────────────────────────
function showError(msg) {
  const d = document.createElement('div');
  d.style.cssText = 'position:fixed;top:0;left:0;right:0;background:#c00;color:#fff;padding:8px 12px;z-index:9999;font-family:monospace;font-size:12px;white-space:pre-wrap';
  d.textContent = 'FEHLER: ' + msg;
  document.body.prepend(d);
  console.error(msg);
}
window.addEventListener('unhandledrejection', e => showError(String(e.reason)));

// ── Init ───────────────────────────────────────────────────────────
try { init3d(); } catch (e) { showError('init3d: ' + e); }
try {
  rebuildRobotKinematics();   // Pivot-Chain beim Start aufbauen
  applyJointRotations();      // Referenzpose anwenden
  renderAll();
  updateAxisPointVisuals();
} catch (e) { showError('renderAll: ' + e); }
animate();

// ── 3D-Szene ───────────────────────────────────────────────────────
function init3d() {
  const canvas = $('viewer');
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x050b14);
  camera = new THREE.PerspectiveCamera(45, 1, 1, 100000);
  camera.position.set(1600, -2200, 1300);
  camera.up.set(0, 0, 1);
  controls = new OrbitControls(camera, canvas);
  controls.enablePan = true;
  controls.screenSpacePanning = true;
  controls.mouseButtons = { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.PAN, RIGHT: THREE.MOUSE.ROTATE };
  canvas.addEventListener('contextmenu', e => e.preventDefault());
  canvas.addEventListener('auxclick', e => { if (e.button === 1) { e.preventDefault(); e.stopPropagation(); } }, true);
  canvas.addEventListener('mousedown', e => { if (e.button === 1) e.preventDefault(); }, true);
  controls.target.set(450, 0, 550);
  controls.update();

  scene.add(new THREE.HemisphereLight(0xffffff, 0x94a3b8, 2.4));
  const d = new THREE.DirectionalLight(0xffffff, 2.1);
  d.position.set(1500, -2000, 2500);
  scene.add(d);

  grid = new THREE.GridHelper(4000, 40, 0x1b3454, 0x0f2038);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);
  scene.add(new THREE.AxesHelper(900));

  robotGroup = new THREE.Group();
  toolGroup = new THREE.Group();
  kinematicsRoot = new THREE.Group(); // keine STL-Transformation!
  scene.add(robotGroup, toolGroup, kinematicsRoot);

  axisPointGroup = new THREE.Group();
  scene.add(axisPointGroup);
  csHelperGroup = new THREE.Group();
  scene.add(csHelperGroup);

  raycaster = new THREE.Raycaster();
  mouse = new THREE.Vector2();

  transformControls = new TransformControls(camera, renderer.domElement);
  transformControls.addEventListener('dragging-changed', e => controls.enabled = !e.value);
  transformControls.addEventListener('objectChange', onAxisObjectMoved);
  scene.add(transformControls);

  tcpMarker = new THREE.Group();
  tcpMarker.add(new THREE.Mesh(
    new THREE.SphereGeometry(28, 20, 12),
    new THREE.MeshStandardMaterial({ color: '#8b5cf6', emissive: '#7c3aed', emissiveIntensity: .3 })
  ));
  tcpMarker.visible = false;
  toolGroup.add(tcpMarker);

  new ResizeObserver(resize).observe(canvas.parentElement);
  resize();
  setTimeout(() => setView('iso'), 0);
}

function resize() {
  const r = $('viewer').parentElement.getBoundingClientRect();
  const w = r.width || window.innerWidth;
  const h = r.height || window.innerHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}

function animate() { requestAnimationFrame(animate); renderer.render(scene, camera); }

// ── Transforms ─────────────────────────────────────────────────────
function defaultRobotTr() { return { x: 0, y: 0, z: 0, rx: -90, ry: 0, rz: -90 }; }
function defaultToolTr()  { return { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 }; }

function readInputs(p) {
  return { x: num($(p+'X').value)||0, y: num($(p+'Y').value)||0, z: num($(p+'Z').value)||0,
    rx: num($(p+'Rx').value)||0, ry: num($(p+'Ry').value)||0, rz: num($(p+'Rz').value)||0 };
}
function setInputs(p, tr) {
  ['X','Y','Z','Rx','Ry','Rz'].forEach(k => $(p+k).value = tr[k.toLowerCase()] || 0);
}
function applyTransforms() {
  state.robotTr = readInputs('r');
  state.toolTr  = readInputs('t');
  // Gruppen: nur Position, keine Rotation (Rotation liegt auf den Meshes)
  const _rx = deg(state.robotTr.rx), _ry = deg(state.robotTr.ry), _rz = deg(state.robotTr.rz);
  robotGroup.position.set(state.robotTr.x, state.robotTr.y, state.robotTr.z);
  robotGroup.rotation.set(0, 0, 0);
  kinematicsRoot.position.set(state.robotTr.x, state.robotTr.y, state.robotTr.z);
  toolGroup.position.set(state.toolTr.x, state.toolTr.y, state.toolTr.z);
  toolGroup.rotation.set(0, 0, 0);
  // STL-Korrektur auf ALLE Meshes gleichmäßig anwenden (Roboter, Podest, Tool)
  for (const [, mesh] of meshes) mesh.rotation.set(_rx, _ry, _rz);
  if (axisPointGroup) { axisPointGroup.position.set(0,0,0); axisPointGroup.rotation.set(0,0,0); axisPointGroup.scale.set(1,1,1); }
  applyJointRotations();
  scene.updateMatrixWorld(true);
  renderIssues();
}

function fitCamera() {
  const box = new THREE.Box3().setFromObject(robotGroup);
  box.expandByObject(toolGroup);
  if (!Number.isFinite(box.min.x)) return;
  const size = box.getSize(new THREE.Vector3()), center = box.getCenter(new THREE.Vector3());
  controls.target.copy(center);
  camera.position.set(center.x + size.length() * .85, center.y - size.length() * 1.15, center.z + size.length() * .7);
  controls.update();
}

function ground(group, p) {
  group.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(group);
  if (!Number.isFinite(box.min.z)) return;
  const tr = p === 'r' ? state.robotTr : state.toolTr;
  tr.z += -box.min.z;
  setInputs(p, tr);
  applyTransforms();
}

// ── Kinematik ──────────────────────────────────────────────────────
function fixedAxisType(i)    { return ['Rz','Ry','Ry','Rx','Ry','Rx'][i] || 'Rz'; }
function nominalAxisVec(i)   { return ['z','y','y','x','y','x'][i] || 'z'; }
function axisDirectionLabel(i) { return fixedAxisType(i) + ' · ' + nominalAxisVec(i).toUpperCase(); }

function cumulativeAxisPositions() {
  // Regeln aus Excel (World→Skeleton-Logik, pivot[0] immer Ursprung):
  // Seg1(A1): XZ tauschen  → Three.X=p.z, Three.Z=p.x
  // Seg2(A2): unverändert  → Three.X=p.x, Three.Z=p.z
  // Seg3(A3): Z→X          → Three.X=0,   Three.Z=p.x
  // Seg4(A4): X→Z          → Three.X=p.z, Three.Z=0
  // Seg5(A5): X→Z          → Three.X=p.z, Three.Z=0
  const rules = [
    (p) => ({ dx: num(p.z)||0, dz: num(p.x)||0 }),  // A1: XZ tauschen
    (p) => ({ dx: num(p.x)||0, dz: num(p.z)||0 }),  // A2: unverändert
    (p) => ({ dx: 0,           dz: num(p.x)||0 }),  // A3: Z→X
    (p) => ({ dx: num(p.z)||0, dz: 0           }),  // A4: X→Z
    (p) => ({ dx: num(p.z)||0, dz: 0           }),  // A5: X→Z
  ];
  let x=0, y=0, z=0;
  const pts = [new THREE.Vector3(0,0,0)];
  for (let i=0; i<5; i++) {
    const p = state.axisPoints[i];
    const {dx, dz} = rules[i](p);
    x += dx; y += num(p.y)||0; z += dz;
    pts.push(new THREE.Vector3(x,y,z));
  }
  return pts;
}

function syncJointsFromAxisPoints() {
  state.axisPoints.forEach((p, i) => {
    if (!state.joints[i]) return;
    p.x = num(p.x) ?? 0; p.y = num(p.y) ?? 0; p.z = num(p.z) ?? 0;
    state.joints[i].offset = { x: p.x, y: p.y, z: p.z };
    if (String(p.source).includes('XML')) state.joints[i].status = 'XML AxisPos';
  });
}

function rebuildRobotKinematics() {
  clearGroup(robotGroup); clearGroup(toolGroup);
  axisPivotGroups.length = 0;
  toolGroup.add(tcpMarker);
  const pts = cumulativeAxisPositions();
  for (let i = 0; i < 6; i++) {
    const g = new THREE.Group(); g.name = 'Pivot ' + (i + 1); g.userData.axisIndex = i;
    axisPivotGroups[i] = g;
    if (i === 0) { kinematicsRoot.add(g); g.position.copy(pts[0] || new THREE.Vector3()); }
    else {
      axisPivotGroups[i-1].add(g);
      g.position.copy((pts[i] || new THREE.Vector3()).clone().sub(pts[i-1] || new THREE.Vector3()));
    }
  }
  for (const [path, mesh] of meshes) {
    mesh.position.set(0,0,0); mesh.scale.set(1,1,1);
    mesh.rotation.set(deg(state.robotTr.rx), deg(state.robotTr.ry), deg(state.robotTr.rz));
    const file = state.stls.find(f => f.path === path) || { name: mesh.name };
    if (isTool(file)) { toolGroup.add(mesh); continue; }
    // Manuelle Zuweisung prüfen
    let assignedAxis = null;
    for (const [ax, stlName] of Object.entries(state.axisStlMap)) {
      if (stlName && norm(stlName) === norm(file.name)) { assignedAxis = ax; break; }
    }
    const key = assignedAxis || partKey(file.name);
    const m = key.match(/^A([1-6])$/);
    if (m) {
      const idx = Number(m[1]) - 1;
      const pivot = pts[idx] || new THREE.Vector3();
      axisPivotGroups[idx].add(mesh);
      mesh.position.copy(pivot.clone().multiplyScalar(-1));
    } else { robotGroup.add(mesh); }
  }
  applyJointRotations();
}

function applyJointRotations() {
  // Display-Winkel 0 = Referenzpose → Offset von Referenzpose anwenden
  const r = Math.PI / 180;
  const ref = parseReferencePose();
  axisPivotGroups.forEach((g, i) => {
    const a = ((state.jointAngles[i] || 0) - (ref[i] || 0)) * (num(state.joints[i]?.rotationSign) ?? 1) * r;
    const v = nominalAxisVec(i);
    g.rotation.set(0, 0, 0);
    if      (v === 'x') g.rotation.x = a;
    else if (v === 'y') g.rotation.y = a;
    else                g.rotation.z = a;
  });
  scene.updateMatrixWorld(true);
  updateSkeletonPositions();
}

function parseReferencePose() {
  const raw = ($('refPose')?.value || '0,-90,90,0,0,0').split(',').map(v => Number(String(v).trim().replace(',','.')));
  if (raw.length === 6 && raw.every(Number.isFinite)) { state.referencePose = raw; return raw; }
  state.referencePose = [0,-90,90,0,0,0]; return state.referencePose;
}
function setJointAnglesToReferencePose() {
  state.jointAngles = [0, -90, 90, 0, 0, 0];  // Referenzpose
  while (state.jointAngles.length < 6) state.jointAngles.push(0);
}

// ── Achspunkte visuell ─────────────────────────────────────────────
function clearAxisPointVisuals() {
  if (!axisPointGroup) return;
  while (axisPointGroup.children.length) {
    const c = axisPointGroup.children.pop();
    c.geometry?.dispose?.(); c.material?.dispose?.();
  }
  axisMeshes.length = 0;
  if (transformControls) transformControls.detach();
}


function buildSkeletonCylinder(from, to, radius) {
  const v1 = from.clone(), v2 = to.clone();
  const dir = new THREE.Vector3().subVectors(v2, v1);
  const len = dir.length(); if (len < 1) return null;
  const geo = new THREE.CylinderGeometry(radius, radius, len, 10);
  const mat = new THREE.MeshPhongMaterial({ color: LINK_COLOR, shininess: 80, specular: 0x444444 });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.copy(v1).addScaledVector(dir.normalize(), len * .5);
  const up = new THREE.Vector3(0,1,0);
  const ax = new THREE.Vector3().crossVectors(up, dir).normalize();
  const ang = Math.acos(Math.max(-1, Math.min(1, up.dot(dir))));
  if (ax.length() > 0.001) mesh.quaternion.setFromAxisAngle(ax, ang);
  else if (dir.y < 0) mesh.rotation.z = Math.PI;
  mesh.userData.isSkelCyl = true;
  return mesh;
}

function rebuildSkeletonMeshes(pts) {
  // Alte Zylinder/Kugeln aus axisPointGroup entfernen
  for (const m of [...skeletonCyls, ...skeletonSphs]) axisPointGroup.remove(m);
  skeletonCyls.length = 0; skeletonSphs.length = 0;

  // Zylinder zwischen aufeinanderfolgenden Punkten
  for (let i = 0; i < pts.length - 1; i++) {
    const cyl = buildSkeletonCylinder(pts[i], pts[i+1], LINK_R[i] || 5);
    if (cyl) { axisPointGroup.add(cyl); skeletonCyls.push(cyl); }
  }
  // Kugeln an Gelenkpunkten (ab Index 1 = A1)
  for (let i = 1; i < pts.length; i++) {
    const mat = new THREE.MeshPhongMaterial({
      color: JOINT_COLOR, shininess: 120, specular: 0x666666
    });
    const sph = new THREE.Mesh(new THREE.SphereGeometry(JOINT_R[i-1] || 10, 12, 8), mat);
    sph.position.copy(pts[i]);
    sph.userData.isSkelSph = true;
    axisPointGroup.add(sph); skeletonSphs.push(sph);
  }
}

function updateSkeletonPositions() {
  if (!axisPivotGroups.length || !axisPointGroup) return;
  scene.updateMatrixWorld(true);

  // Live-Positionen
  const livePts = [new THREE.Vector3(0,0,0)];
  axisPivotGroups.forEach(g => {
    const wp = new THREE.Vector3(); g.getWorldPosition(wp); livePts.push(wp);
  });
  const pts6 = livePts.slice(0,6);

  // Zylinder neu bauen (da Position+Rotation sich ändert)
  rebuildSkeletonMeshes(pts6);

  // Kugeln + Labels (Sprites) verschieben
  axisPointGroup.children.forEach(child => {
    const idx = child.userData.skeletonIdx;
    if (idx === undefined) return;
    if (child.isSprite) {
      child.position.copy(pts6[idx]).add(new THREE.Vector3(0,0,85));
    } else if (!child.userData.isSkelCyl && !child.userData.isSkelSph) {
      child.position.copy(pts6[idx]);
    }
  });

  // Linie ausblenden (Zylinder ersetzen sie)
  if (axisLine) axisLine.visible = false;

  updateCSHelper();
}

function updateAxisPointVisuals() {
  if (!axisPointGroup) return;
  clearAxisPointVisuals();
  scene.updateMatrixWorld(true);

  // Live-Weltpositionen der Pivot-Gruppen
  const livePts = [new THREE.Vector3(0,0,0)];
  if (axisPivotGroups.length) {
    axisPivotGroups.forEach(g => {
      const wp = new THREE.Vector3(); g.getWorldPosition(wp); livePts.push(wp);
    });
  } else {
    cumulativeAxisPositions().forEach(p => livePts.push(p));
  }
  const pts6 = livePts.slice(0, 6);

  // ── RobSimul-Stil: Zylinder + abgestufte Kugeln ──────────────
  rebuildSkeletonMeshes(pts6);

  // ── Auswählbare (unsichtbare) Raycaster-Kugeln + Labels ──────
  pts6.forEach((p, i) => {
    if (i === 0) return; // A1 (Ursprung) nicht selektierbar
    const hitGeo = new THREE.SphereGeometry(JOINT_R[i-1] || 10, 8, 6);
    const hitMat = new THREE.MeshBasicMaterial({ visible: false });
    const hit = new THREE.Mesh(hitGeo, hitMat);
    hit.position.copy(p);
    hit.userData.axisIndex = i - 1;
    hit.name = 'A' + (i + 1);
    axisPointGroup.add(hit);
    axisMeshes.push(hit);

    const lbl = makeAxisLabel('A' + (i + 1), p.clone().add(new THREE.Vector3(0, 0, JOINT_R[i-1] * 2 + 20)));
    lbl.userData.skeletonIdx = i;
    axisPointGroup.add(lbl);
  });

  const selected = axisMeshes[state.selectedAxis];
  if (selected) transformControls.attach(selected);
  updateCSHelper();

  const badge = $('axisSelBadge');
  if (badge) {
    badge.textContent = selected ? selected.name : '';
    badge.style.display = selected ? '' : 'none';
  }
}

function makeAxisLabel(text, pos) {
  const c = document.createElement('canvas'); c.width = 160; c.height = 80;
  const ctx = c.getContext('2d');
  ctx.fillStyle = 'rgba(5,11,20,.88)'; ctx.fillRect(0,0,160,80);
  ctx.strokeStyle = '#ff8a00'; ctx.lineWidth = 4; ctx.strokeRect(3,3,154,74);
  ctx.fillStyle = '#ffffff'; ctx.font = 'bold 42px Arial'; ctx.fillText(text, 42, 54);
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), transparent: true, depthTest: false }));
  sp.scale.set(180, 90, 1); sp.renderOrder = 999; sp.position.copy(pos);
  return sp;
}

function pickAxisPoint(event) {
  if (!axisMeshes.length) return;
  const rect = renderer.domElement.getBoundingClientRect();
  mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(mouse, camera);
  const hit = raycaster.intersectObjects(axisMeshes, false)[0];
  if (hit) { state.selectedAxis = hit.object.userData.axisIndex; updateAxisPointVisuals(); event.preventDefault(); }
}

function onAxisObjectMoved() {
  const mesh = transformControls.object; if (!mesh) return;
  const idx = mesh.userData.axisIndex;
  const pts = cumulativeAxisPositions();
  const prev = idx > 0 ? pts[idx-1] : new THREE.Vector3(0,0,0);
  const local = mesh.position.clone().sub(prev);
  const p = state.axisPoints[idx];
  p.x = Number(local.x.toFixed(3)); p.y = Number(local.y.toFixed(3)); p.z = Number(local.z.toFixed(3)); p.source = 'manuell';
  syncJointsFromAxisPoints(); rebuildRobotKinematics(); applyTransforms(); renderRows(); renderIssues();
}


function updateCSHelper() {
  // Altes CS-Helper entfernen
  while (csHelperGroup.children.length) csHelperGroup.children.pop();
  if (!axisMeshes.length) return;

  const pts = cumulativeAxisPositions();
  const pos = pts[state.selectedAxis] || new THREE.Vector3();
  const L = 380;   // Pfeillänge mm
  const H = 30;    // Pfeilkopf
  // Display X-Spalte = Three.Z (oben), Display Z-Spalte = Three.X (horizontal)
  const axes = [
    { dir: new THREE.Vector3(0,0,1), color: 0xff3333, label: 'X' },  // X → oben
    { dir: new THREE.Vector3(0,1,0), color: 0x33dd33, label: 'Y' },
    { dir: new THREE.Vector3(1,0,0), color: 0x3388ff, label: 'Z' },  // Z → horizontal
  ];
  axes.forEach(({ dir, color, label }) => {
    const arrow = new THREE.ArrowHelper(dir, pos, L, color, H, H * 0.6);
    arrow.line.material.linewidth = 4;
    arrow.renderOrder = 998;
    csHelperGroup.add(arrow);

    // Label-Sprite
    const c = document.createElement('canvas'); c.width = 128; c.height = 128;
    const ctx = c.getContext('2d');
    ctx.clearRect(0,0,128,128);
    ctx.fillStyle = '#' + color.toString(16).padStart(6,'0');
    ctx.font = 'bold 90px Arial';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(label, 64, 64);
    const sp = new THREE.Sprite(new THREE.SpriteMaterial({
      map: new THREE.CanvasTexture(c), transparent: true, depthTest: false
    }));
    sp.scale.set(160, 160, 1);
    sp.position.copy(pos).addScaledVector(dir, L + 100);
    sp.renderOrder = 999;
    csHelperGroup.add(sp);
  });
}

function selectAxisPoint(i) {
  state.selectedAxis = Math.max(0, Math.min(5, Number(i) || 0));
  updateAxisPointVisuals();
  updateCSHelper();
}

// ── STL laden ──────────────────────────────────────────────────────
function partKey(n) {
  const s = norm(n);
  const a = s.match(/a([1-6])$/); if (a) return 'A' + a[1];
  if (/tool|tcp|meltio/.test(s)) return 'Tool';
  if (/podest|base/.test(s)) return 'Base';
  return 'A1';
}
const colors = { Base:'#333', A1:'#fff', A2:'#999', A3:'#ff7f00', A4:'#ff7f00', A5:'#999', A6:'#666', Tool:'#64748b' };
function isTool(f) { const n = norm(f.name||f); const tool = norm(state.tcp.auftragen.toolStl||state.toolName); return n===tool||/tool1_tcp|tool|tcp|meltio/.test(n); }
function findStl(stem) { const s = norm(stem); return state.stls.find(f=>norm(f.name)===s)?.name || state.stls.find(f=>norm(f.name).includes(s)||s.includes(norm(f.name)))?.name || null; }
function clearGroup(g) { while (g.children.length) g.remove(g.children[0]); }

async function loadStls() {
  for (const f of state.stls) {
    try {
      const u8 = state.buffers.get(f.path);
      const g = loader.parse(u8.buffer.slice(u8.byteOffset, u8.byteOffset + u8.byteLength));
      g.computeVertexNormals();
      const mat = new THREE.MeshStandardMaterial({ color: colors[partKey(f.name)], roughness: .62, metalness: .08 });
      const mesh = new THREE.Mesh(g, mat); mesh.name = f.name;
      meshes.set(f.path, mesh);
    } catch (e) { console.warn(e); }
  }
  rebuildRobotKinematics(); applyTransforms(); fitCamera();
}

// ── ZIP / JSON ─────────────────────────────────────────────────────
const typeOf = n => /\.stl$/i.test(n)?'STL':/\.xml$/i.test(n)?'XML':/\.json$/i.test(n)?'JSON':'Datei';
function splitFiles() { state.stls=state.files.filter(f=>f.type==='STL'); state.xmls=state.files.filter(f=>f.type==='XML'); state.jsons=state.files.filter(f=>f.type==='JSON'); }

async function readZip(file) {
  const zip = await JSZip.loadAsync(file);
  const files = [], buffers = new Map();
  for (const e of Object.values(zip.files).filter(f => !f.dir)) {
    const data = await e.async('uint8array');
    buffers.set(e.name, data);
    files.push({ path: e.name, name: e.name.split('/').pop(), size: data.byteLength, type: typeOf(e.name) });
  }
  return { files, buffers };
}

function resetData() {
  for (const m of meshes.values()) { m.parent?.remove(m); m.geometry?.dispose?.(); m.material?.dispose?.(); }
  meshes.clear();
  state.files=[]; state.stls=[]; state.xmls=[]; state.jsons=[];
  state.buffers=new Map(); state.packageJson=null; state.mode='leer'; state.robotName='';
  state.jointAngles=[0,-90,90,0,0,0]; state.referencePose=[0,-90,90,0,0,0];
  if ($('refPose')) $('refPose').value='0,-90,90,0,0,0';
  state.robotTr={x:0,y:0,z:0,rx:0,ry:0,rz:0}; state.toolTr={x:0,y:0,z:0,rx:0,ry:0,rz:0};
  state.axisPoints=['A1','A2','A3','A4','A5','A6'].map((name,i)=>({name,...defOffset(i),rx:0,ry:0,rz:0,source:'KR8 Zielwert'}));
  state.selectedAxis=0; clearAxisPointVisuals();
  setInputs('r', state.robotTr); setInputs('t', state.toolTr);
  state.simulation={active:false,axis:null,raf:null};
  state.joints=['A1','A2','A3','A4','A5','A6'].map((n,i)=>({name:n,axis:fixedAxisType(i),offset:defOffset(i),min:null,max:null,rotationSign:1,status:'KR8 Zielwert'}));
  state.tcp.auftragen={x:null,y:null,z:null,rz:null,ry:null,rx:null,toolLength:0,toolStl:'',status:'manuell'};
  state.tcp.abtragen={...state.tcp.auftragen};
}

function enableSave()  { $('downloadJson').disabled=false; $('downloadZip').disabled=false; }
function disableSave() { $('downloadJson').disabled=true;  $('downloadZip').disabled=true; }

async function loadSourceZip(file) {
  resetData(); state.mode='source';
  const z = await readZip(file); state.files=z.files; state.buffers=z.buffers;
  splitFiles();
  state.robotTr=defaultRobotTr(); setInputs('r', state.robotTr);
  state.toolTr=defaultToolTr();  setInputs('t', state.toolTr);
  if (state.xmls[0]) parseXml(new TextDecoder('utf-8').decode(state.buffers.get(state.xmls[0].path)));
  setJointAnglesToReferencePose();
  await loadStls(); enableSave(); renderAll(); setView('iso');
}

async function loadPackageZip(file) {
  resetData(); state.mode='package';
  const z = await readZip(file); state.files=z.files; state.buffers=z.buffers;
  splitFiles();
  if (state.jsons[0]) {
    try { state.packageJson=JSON.parse(new TextDecoder('utf-8').decode(state.buffers.get(state.jsons[0].path))); applyJsonToState(state.packageJson); }
    catch (e) { state.packageJson=null; }
  }
  zeroAllTransforms();
  await loadStls(); enableSave(); renderAll(); setView('iso');
}

async function loadJsonFile(file) {
  resetData(); state.mode='json';
  try {
    const j=JSON.parse(await file.text()); state.packageJson=j; applyJsonToState(j);
    zeroAllTransforms(); enableSave(); renderAll(); setView('iso');
  } catch (e) { alert('JSON konnte nicht gelesen werden: '+e.message); renderAll(); }
}

function zeroAllTransforms() {
  state.robotTr={x:0,y:0,z:0,rx:0,ry:0,rz:0}; state.toolTr={x:0,y:0,z:0,rx:0,ry:0,rz:0};
  state.jointAngles=[0,-90,90,0,0,0];
  setInputs('r',state.robotTr); setInputs('t',state.toolTr);
  renderJointAngleRows();
}

// ── XML-Parser ─────────────────────────────────────────────────────
function parseXml(text) {
  const xml = new DOMParser().parseFromString(text,'application/xml');
  state.robotName = state.xmls[0]?.name?.replace(/\.xml$/i,'') || 'Robot';
  const gotAxis = parseAxisPositions(xml);
  if (/irb\s*4600|4600-40-2_55/i.test(text)) {
    const mins=[-180,-90,-180,-179,-125,-179], maxs=[180,150,75,179,120,179];
    state.joints.forEach((j,i)=>{j.min=mins[i];j.max=maxs[i];if(!gotAxis){j.offset=defOffset(i);state.axisPoints[i]={...state.axisPoints[i],...defOffset(i),source:'KR8 Zielwert'}}j.status=gotAxis?'XML AxisPos':'vorgeschlagen'});
  }
  parseTools(xml); syncJointsFromAxisPoints(); normalizeKnownOffsets(); updateAxisPointVisuals();
}

function direct(parent, tag) {
  if (!parent) return null;
  const el = [...parent.children].find(c => c.tagName === tag);
  return el ? el.getAttribute('DefaultValue') : null;
}
function readDR(parent, tagName) {
  const child = [...(parent?.children||[])].find(c=>c.tagName===tagName);
  const v = child?.getAttribute('DefaultValue'); if (v===undefined||v===null) return null;
  const n = num(v); return n===null?v:n;
}
function readToolOrientation(toolData) {
  const orient = [...toolData.children].find(c=>c.tagName==='ToolOrientation'); if (!orient) return null;
  const add = [...orient.children].find(c=>c.tagName==='AdditionalTransform');
  return { rx:(num(readDR(orient,'Rx'))??0)+(num(readDR(add,'Rx'))??0), ry:(num(readDR(orient,'Ry'))??0)+(num(readDR(add,'Ry'))??0), rz:(num(readDR(orient,'Rz'))??0)+(num(readDR(add,'Rz'))??0) };
}
function cleanTcpOrientation(tcp) {
  const raw = {rz:num(tcp.rz),ry:num(tcp.ry),rx:num(tcp.rx)};
  const hasFrac = [raw.rz,raw.ry,raw.rx].some(v=>v!==null&&Math.abs(v)>0&&Math.abs(v)<1);
  if (hasFrac) { tcp.rz=0;tcp.ry=0;tcp.rx=0;return tcp; }
  tcp.rz=raw.rz??0; tcp.ry=raw.ry??0; tcp.rx=raw.rx??0; return tcp;
}
function parseTools(xml) {
  const sc=[...xml.querySelectorAll('SCType')]; let holder=sc.find(n=>/TAdditiveToolHolder/i.test(n.getAttribute('Type')||''))||sc.find(n=>n.parentElement?.querySelector('ToolData')); if(!holder)return;
  let n=holder.nextElementSibling,toolData=null,img=null;
  while(n&&n.tagName!=='SCType'){if(n.tagName==='ToolData')toolData=n;if(n.tagName==='ImageFile')img=n;n=n.nextElementSibling}
  toolData=toolData||holder.parentElement?.querySelector('ToolData'); img=img||holder.parentElement?.querySelector('ImageFile'); if(!toolData)return;
  const stem=(img?.getAttribute('DefaultValue')||'').replace(/\\/g,'/').split('/').pop().replace(/\.osd$/i,'');
  const stl=findStl(stem)||findStl('tool1_tcp')||stem+'.stl'; state.toolName=stl;
  const orientation=readToolOrientation(toolData);
  const tcp={x:num(direct(toolData,'X')),y:num(direct(toolData,'Z')),z:num(direct(toolData,'Y')),rz:orientation?orientation.rz:num(direct(toolData,'A')),ry:orientation?orientation.ry:num(direct(toolData,'B')),rx:orientation?orientation.rx:num(direct(toolData,'C')),toolLength:num(direct(toolData,'ToolLength'))??0,toolStl:stl,status:'gefunden'};
  cleanTcpOrientation(tcp); state.tcp.auftragen=tcp; state.tcp.abtragen={...tcp,status:'vorgeschlagen'};
}

function parseAxisPositions(xml) {
  let found=false;
  state.axisPoints=['A1','A2','A3','A4','A5','A6'].map(name=>({name,x:0,y:0,z:0,rx:0,ry:0,rz:0,source:'leer'}));
  const params=[...xml.querySelectorAll('ParameterName')];
  for(const p of params) {
    const name=p.getAttribute('DefaultValue')||''; const m=name.match(/^AxisA([1-6])Pos$/i); if(!m)continue;
    const xmlAxis=Number(m[1]); if(xmlAxis<2||xmlAxis>6)continue;
    const idx=xmlAxis-2; let tx=0,ty=0,tz=0; let matrix=p.nextElementSibling;
    while(matrix&&matrix.tagName!=='Matrix')matrix=matrix.nextElementSibling;
    if(matrix)[...matrix.querySelectorAll('SCType')].forEach(sc=>{const type=sc.getAttribute('Type')||'';const v=num(sc.getAttribute('DefaultValue'))||0;if(type==='TTranslateX')tx+=v;if(type==='TTranslateY')ty+=v;if(type==='TTranslateZ')tz+=v;});
    const point={name:'A'+(idx+1),x:0,y:0,z:0,rx:0,ry:0,rz:0,source:'XML '+name};
    if(xmlAxis===2){point.x=tx;point.y=ty;point.z=tz;}
    else if(xmlAxis===3){point.x=Math.abs(ty);}
    else if(xmlAxis===4){point.z=Math.abs(ty);}
    else if(xmlAxis===5){point.x=Math.abs(tz);}
    else if(xmlAxis===6){point.x=Math.abs(tx);}
    state.axisPoints[idx]=point; found=true;
  }
  state.axisPoints[5]={name:'A6',x:0,y:0,z:0,rx:0,ry:0,rz:0,source:found?'Ende / 0':'leer'};
  syncJointsFromAxisPoints(); return found;
}

function closeVal(a,b){return Math.abs((num(a)||0)-b)<.001}
function setKnownOffsets(){state.joints.forEach((j,i)=>{j.offset=defOffset(i);j.axis=fixedAxisType(i);});state.axisPoints=state.joints.map((j,i)=>({name:'A'+(i+1),...defOffset(i),rx:0,ry:0,rz:0,source:'KR8 Zielwert'}));}
function normalizeKnownOffsets(){const o=state.joints.map(j=>j.offset||{});const wS=closeVal(o[0].x,0)&&closeVal(o[0].y,0)&&closeVal(o[0].z,0)&&closeVal(o[1].x,495)&&closeVal(o[1].y,175);const wD=closeVal(o[0].x,495)&&closeVal(o[0].y,175)&&closeVal(o[0].z,0);const ok=closeVal(o[0].x,175)&&closeVal(o[0].y,0)&&closeVal(o[0].z,495);if((wS||wD)&&!ok)setKnownOffsets();}

// ── JSON ───────────────────────────────────────────────────────────
function applyJsonToState(j) {
  if(!j)return;
  state.robotName=j.name||state.robotName;
  if(Array.isArray(j.stlRefAngles)&&j.stlRefAngles.length===6){state.referencePose=j.stlRefAngles.map(v=>Number(v)||0);if($('refPose'))$('refPose').value=state.referencePose.join(',');}
  if(Array.isArray(j.jointAngles)&&j.jointAngles.length===6)state.jointAngles=j.jointAngles.map(v=>Number(v)||0);
  if(Array.isArray(j.joints)){state.joints=j.joints.map((v,i)=>({name:v.name||('A'+(i+1)),axis:fixedAxisType(i),offset:v.offset||{x:null,y:null,z:null},min:num(v.min),max:num(v.max),rotationSign:num(v.rotationSign??v.rotationDirection??v.dir)??1,status:v.status||'JSON'}));state.axisPoints=state.joints.map((v,i)=>({name:v.name||('A'+(i+1)),x:num(v.offset?.x),y:num(v.offset?.y),z:num(v.offset?.z),rx:0,ry:0,rz:0,source:'JSON'}));}
  if(j.tcp){state.tcp.auftragen=cleanTcpOrientation({...(j.tcp.auftragen||j.tcp),toolLength:j.tcp.auftragen?.toolLength??0,status:'JSON'});state.tcp.abtragen=cleanTcpOrientation({...(j.tcp.abtragen||j.tcp.auftragen||j.tcp),toolLength:j.tcp.abtragen?.toolLength??0,status:'JSON'});}
  const toolName=j.sceneModels?.tool?.name||j.tcp?.auftragen?.toolStl||j.tcp?.auftragen?.stlName;
  if(toolName)state.toolName=String(toolName).endsWith('.stl')?toolName:toolName+'.stl';
  normalizeKnownOffsets();
}

function buildJson() {
  return {
    name: state.robotName||'Robot',
    joints: state.joints.map((j,i)=>({name:j.name,axis:fixedAxisType(i),offset:{x:num(j.offset?.x)??0,y:num(j.offset?.y)??0,z:num(j.offset?.z)??0},min:num(j.min),max:num(j.max)})),
    stlRefAngles: parseReferencePose(),
    tcp: { auftragen: { ...state.tcp.auftragen }, abtragen: { ...state.tcp.abtragen } },
    stlFiles: Object.fromEntries(['A1','A2','A3','A4','A5','A6'].map(k=>[k,{name:'—',posx:0,posy:0,posz:0,posrx:0,posry:0,posrz:0,color:'#e8a020'}])),
    sceneModels: { pedestal:{px:0,py:0,pz:0,rx:0,ry:0,rz:0,name:'podest'}, tool:{px:0,py:0,pz:0,rx:0,ry:0,rz:0,name:state.toolName||'tool1_tcp'} }
  };
}

// ── Export ─────────────────────────────────────────────────────────
async function downloadJson() { dl(new Blob([JSON.stringify(buildJson(),null,2)],{type:'application/json'}),zipName(state.robotName)+'.json'); }
async function downloadZip() {
  const zip=new JSZip(), base=zipName(state.robotName||'RobModel_export');
  zip.file(base+'.json',JSON.stringify(buildJson(),null,2));
  for(const[,mesh] of meshes) zip.file(mesh.name,exportBinaryStl(mesh));
  dl(await zip.generateAsync({type:'blob'}),base+'.zip');
}
function dl(blob,name){const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=name;document.body.appendChild(a);a.click();setTimeout(()=>{URL.revokeObjectURL(a.href);a.remove()},500);}

function exportBinaryStl(mesh) {
  scene.updateMatrixWorld(true); mesh.updateMatrixWorld(true);
  const g=mesh.geometry.index?mesh.geometry.toNonIndexed():mesh.geometry.clone();
  g.applyMatrix4(mesh.matrixWorld);
  const pos=g.getAttribute('position'),tri=Math.floor(pos.count/3),buf=new ArrayBuffer(84+tri*50),view=new DataView(buf);
  view.setUint32(80,tri,true);
  const a=new THREE.Vector3(),b=new THREE.Vector3(),c=new THREE.Vector3(),n=new THREE.Vector3();
  let off=84;
  for(let i=0;i<tri;i++){a.fromBufferAttribute(pos,i*3);b.fromBufferAttribute(pos,i*3+1);c.fromBufferAttribute(pos,i*3+2);n.subVectors(c,b).cross(a.clone().sub(b)).normalize();[n.x,n.y,n.z,a.x,a.y,a.z,b.x,b.y,b.z,c.x,c.y,c.z].forEach(v=>{view.setFloat32(off,v,true);off+=4});view.setUint16(off,0,true);off+=2;}
  g.dispose(); return new Uint8Array(buf);
}

// ── Simulation ─────────────────────────────────────────────────────
function stopSimulation(){if(state.simulation?.raf)cancelAnimationFrame(state.simulation.raf);state.simulation={active:false,axis:null,raf:null};}
function simulateAxis(axisIndex){
  stopSimulation();
  const j=state.joints[axisIndex],min=num(j?.min)??-180,max=num(j?.max)??180;
  const startValue=num(state.jointAngles[axisIndex])??0,duration=4200,start=performance.now();
  const base=state.jointAngles.slice();
  selectAxisPoint(axisIndex);
  state.simulation={active:true,axis:axisIndex,raf:null};
  const lerp=(a,b,t)=>a+(b-a)*t,smooth=t=>0.5-0.5*Math.cos(t*Math.PI);
  const step=now=>{
    const t=Math.min(1,(now-start)/duration),phase=t*3;
    let value=startValue;
    if(phase<=1)value=lerp(startValue,max,smooth(phase));
    else if(phase<=2)value=lerp(max,min,smooth(phase-1));
    else value=lerp(min,startValue,smooth(phase-2));
    state.jointAngles=base.slice();state.jointAngles[axisIndex]=value;
    applyJointRotations();renderJointAngleRows();
    if(t<1&&state.simulation.active&&state.simulation.axis===axisIndex){state.simulation.raf=requestAnimationFrame(step);}
    else{state.jointAngles=base.slice();state.jointAngles[axisIndex]=startValue;applyJointRotations();renderJointAngleRows();state.simulation={active:false,axis:null,raf:null};}
  };
  state.simulation.raf=requestAnimationFrame(step);
}

// ── Render-Funktionen ──────────────────────────────────────────────
function renderAll(){renderAxisStlRows();renderRows();renderJointAngleRows();updateAxisPointVisuals();renderTcp();renderIssues();const b=$('fileBadge');b.textContent=state.files.length?`${state.stls.length} STL · ${state.xmls.length} XML · ${state.jsons.length} JSON`:state.mode==='package'?'Package geladen':'Keine Datei geladen';const tb=$('toolBadge');if(tb)tb.textContent=state.tcp.auftragen.toolStl||state.toolName||'—';}

function renderJointAngleRows(){const el=$('jointAngleRows');if(!el)return;el.innerHTML=state.jointAngles.map((v,i)=>`<div class="field"><label>${state.joints[i]?.name||'A'+(i+1)} ${fixedAxisType(i)}</label><input data-joint-angle="${i}" type="number" step="1" value="${v??0}"></div>`).join('');}


function renderRows(){$('jointRows').innerHTML=state.joints.map((j,i)=>`<tr data-param-row="${i}" class="${i===state.selectedAxis?'sel':''}" ><td><b>${esc(j.name)}</b></td><td><input class="angleInput" data-joint-angle="${i}" type="number" step="0.1" value="${state.jointAngles?.[i]??0}"></td><td><span class="axisDir">${axisDirectionLabel(i)}</span></td><td><input data-j="${i}" data-f="x" value="${j.offset?.x??''}"></td><td><input data-j="${i}" data-f="y" value="${j.offset?.y??''}"></td><td><input data-j="${i}" data-f="z" value="${j.offset?.z??''}"></td><td><input data-j="${i}" data-f="min" value="${j.min??''}"></td><td><input data-j="${i}" data-f="max" value="${j.max??''}"></td><td><select class="dirSel" data-j="${i}" data-f="rotationSign"><option value="1" ${(num(j.rotationSign)??1)>=0?'selected':''}>+</option><option value="-1" ${(num(j.rotationSign)??1)<0?'selected':''}>−</option></select></td><td><button class="simBtn" data-sim-axis="${i}">▶</button></td></tr>`).join('');}

function renderTcp(){qsa('.tab').forEach(t=>t.classList.toggle('active',t.dataset.mode===state.activeTcp));const tcp=state.tcp[state.activeTcp];qsa('[data-tcp]').forEach(i=>i.value=tcp?.[i.dataset.tcp]??'');const x=num(tcp?.x),y=num(tcp?.y),z=num(tcp?.z);tcpMarker.visible=x!==null||y!==null||z!==null;tcpMarker.position.set(x||0,y||0,z||0);}

function renderIssues(){const issues=[];if(state.mode==='package')issues.push(...packageIssues());else{if(!state.files.length)issues.push(['warn','Keine Datei geladen.']);if(!state.xmls.length&&state.mode==='source')issues.push(['warn','XML fehlt.']);if(!state.stls.length&&state.files.length)issues.push(['warn','Keine STL-Dateien.']);if(state.files.length&&!state.tcp.auftragen.toolStl)issues.push(['warn','Tool-STL nicht gefunden.']);if(state.files.length&&!state.axisPoints.some(p=>String(p.source).includes('XML')))issues.push(['warn','Rotationspunkte nicht aus XML gelesen.']);state.joints.forEach(j=>{if(j.min===null||j.max===null)issues.push(['warn',`${j.name}: Achsgrenzen fehlen.`]);});}if(!issues.length)issues.push(['ok','Alle Prüfpunkte OK.']);$('issues').innerHTML=issues.map(([t,m])=>`<div class="issue ${t}"><span class="badge ${t}">${t==='bad'?'Fehler':t==='ok'?'OK':'Prüfen'}</span> ${esc(m)}</div>`).join('');}

function packageIssues(){const issues=[];if(state.jsons.length!==1)issues.push(['bad',`Package: ${state.jsons.length} JSON (erwartet: 1).`]);if(!state.packageJson)issues.push(['bad','JSON nicht lesbar.']);if(!state.stls.length)issues.push(['bad','Keine STL.']);const j=state.packageJson;if(j){if(!Array.isArray(j.joints)||j.joints.length!==6)issues.push(['bad','joints: 6 Achsen erwartet.']);const refs=[];Object.values(j.stlFiles||{}).forEach(v=>{if(v?.name)refs.push(v.name)});Object.values(j.sceneModels||{}).forEach(v=>{if(v?.name)refs.push(v.name)});const stlSet=new Set(state.stls.map(f=>norm(f.name)));refs.forEach(r=>{if(!stlSet.has(norm(r)))issues.push(['warn',`STL fehlt: ${r}.stl`]);});}for(const f of state.stls){if(f.size>12000000)issues.push(['warn',`${f.name}: sehr groß (${fmt(f.size)}).`]);}return issues;}


// ── Kameraansichten ────────────────────────────────────────────────
function sceneBox(){const box=new THREE.Box3().setFromObject(robotGroup);box.expandByObject(toolGroup);box.expandByObject(axisPointGroup);if(!Number.isFinite(box.min.x)){box.min.set(-500,-500,0);box.max.set(1500,500,1500);}return box;}
function setView(view){
  const box=sceneBox(),center=box.getCenter(new THREE.Vector3()),size=box.getSize(new THREE.Vector3());
  const dist=Math.max(size.length()*.85,1200);
  const pos={iso:new THREE.Vector3(center.x+dist,center.y-dist,center.z+dist*.65),top:new THREE.Vector3(center.x,center.y,center.z+dist),bottom:new THREE.Vector3(center.x,center.y,center.z-dist),front:new THREE.Vector3(center.x,center.y-dist,center.z),back:new THREE.Vector3(center.x,center.y+dist,center.z),left:new THREE.Vector3(center.x-dist,center.y,center.z),right:new THREE.Vector3(center.x+dist,center.y,center.z)}[view];
  if(!pos)return;
  camera.position.copy(pos);controls.target.copy(center);
  if(view==='top')camera.up.set(0,1,0);else if(view==='bottom')camera.up.set(0,-1,0);else camera.up.set(0,0,1);
  camera.updateProjectionMatrix();controls.update();
  qsa('[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view===view));
}

// ── Achsen-STL UI ──────────────────────────────────────────────
let _axisStlTarget = null;

function renderAxisStlRows() {
  const el = $('axisStlRows');
  if (!el) return;
  el.innerHTML = ['A1','A2','A3','A4','A5','A6'].map(ax => {
    const name = state.axisStlMap[ax] ? norm(state.axisStlMap[ax]) : '—';
    const hasFile = state.axisStlMap[ax] !== null;
    return `<div class="axis-stl-row">
      <span class="axis-stl-label">${ax}</span>
      <span class="axis-stl-name${hasFile ? ' has-file' : ''}" title="${name}">${name}</span>
      <button class="axis-stl-btn" data-ax="${ax}">+ STL</button>
      ${hasFile ? `<button class="axis-stl-clear" data-ax="${ax}">✕</button>` : ''}
    </div>`;
  }).join('');
}

function initAxisStlEvents() {
  document.addEventListener('click', e => {
    const btn = e.target.closest('.axis-stl-btn');
    const clr = e.target.closest('.axis-stl-clear');
    if (btn) { _axisStlTarget = btn.dataset.ax; $('axisStlInput').click(); }
    if (clr) {
      state.axisStlMap[clr.dataset.ax] = null;
      renderAxisStlRows();
      rebuildRobotKinematics(); applyTransforms();
    }
  });
  $('axisStlInput').addEventListener('change', async e => {
    const file = e.target.files[0]; if (!file || !_axisStlTarget) return;
    const buf = await file.arrayBuffer();
    const u8 = new Uint8Array(buf);
    const geom = loader.parse(u8.buffer); geom.computeVertexNormals();
    const ax = _axisStlTarget;
    const mat = new THREE.MeshStandardMaterial({ color: colors[ax] || 0xe8a020, roughness: .62, metalness: .08 });
    const mesh = new THREE.Mesh(geom, mat); mesh.name = file.name;
    // In state aufnehmen
    state.axisStlMap[ax] = file.name;
    const fObj = { path: file.name, name: file.name, type: 'STL', size: file.size };
    // Alten Mesh für diese Achse entfernen
    state.stls = state.stls.filter(f => {
      const k = state.axisStlMap[ax] === norm(f.name) ? ax : partKey(f.name);
      return k !== ax;
    });
    state.stls.push(fObj);
    state.files = state.stls;
    state.buffers.set(file.name, u8);
    meshes.set(file.name, mesh);
    rebuildRobotKinematics(); applyTransforms();
    renderAxisStlRows(); renderAll();
    e.target.value = '';
  });
}


// ── Theme-System (wie RobSimul) ────────────────────────────────
const THEMES      = ['dark','bg-pro','bg-white','bg-minimal','bg-win11','bg-deep','bg-vivid','bg-matrix'];
const THEME_NAMES = ['Dark','Pro','White','Minimal','Win11','Deep','Vivid','Matrix'];
const THEME_BG    = [0x070d1a,0x1e1e1e,0xf0f0eb,0xf4f4f4,0xf3f6fc,0x000408,0x1a0a2e,0x000800];
const THEME_GRID  = [0x0e1e30,0x2d2d30,0xbbbbaa,0xcccccc,0xc8d8e8,0x0a1020,0x2a1040,0x001400];
let _themeIdx = 0;

function applyTheme(idx) {
  _themeIdx = ((idx % THEMES.length) + THEMES.length) % THEMES.length;
  THEMES.forEach(t => document.body.classList.remove(t));
  if (THEMES[_themeIdx] !== 'dark') document.body.classList.add(THEMES[_themeIdx]);
  if (scene) scene.background = new THREE.Color(THEME_BG[_themeIdx]);
  if (grid)  { if(Array.isArray(grid.material)) grid.material.forEach(m=>m.color.set(THEME_GRID[_themeIdx])); else grid.material.color.set(THEME_GRID[_themeIdx]); }
  const btn = $('themeBtn');
  if (btn) btn.title = 'Theme: ' + THEME_NAMES[_themeIdx];
  try { localStorage.setItem('robmodel_theme', _themeIdx); } catch(e){}
}

window.selectAxisPoint = selectAxisPoint;

// ── Event-Listener ─────────────────────────────────────────────────
$('newBtn').onclick     = () => { resetData(); disableSave(); renderAll(); setView('iso'); };
$('downloadJson').onclick = downloadJson;
$('downloadZip').onclick  = downloadZip;
$('resetView').onclick    = () => setView('iso');
$('toggleGrid').onclick   = () => grid.visible = !grid.visible;
$('robotGround').onclick  = () => ground(robotGroup,'r');
$('robotReset').onclick   = () => { state.robotTr=defaultRobotTr(); setInputs('r',state.robotTr); applyTransforms(); fitCamera(); };
$('toolGround').onclick   = () => ground(toolGroup,'t');
$('toolReset').onclick    = () => { state.toolTr=defaultToolTr(); setInputs('t',state.toolTr); applyTransforms(); fitCamera(); };
initAxisStlEvents();
// Theme laden + Button
try { const saved = localStorage.getItem('robmodel_theme'); if(saved !== null) applyTheme(parseInt(saved)); } catch(e){}
$('themeBtn').onclick = () => applyTheme(_themeIdx + 1);
$('jointReset').onclick   = () => { stopSimulation(); setJointAnglesToReferencePose(); renderJointAngleRows(); renderRows(); applyTransforms(); };
$('sourceZip').addEventListener('change', e => e.target.files[0] && loadSourceZip(e.target.files[0]).catch(err=>alert(err.message)));
$('checkZip').addEventListener('change',  e => e.target.files[0] && loadPackageZip(e.target.files[0]).catch(err=>alert(err.message)));
$('jsonInput').addEventListener('change',  e => e.target.files[0] && loadJsonFile(e.target.files[0]));
if($('refPose'))$('refPose').addEventListener('input',()=>{setJointAnglesToReferencePose();renderJointAngleRows();applyJointRotations();renderIssues();});
['rX','rY','rZ','rRx','rRy','rRz','tX','tY','tZ','tRx','tRy','tRz'].forEach(id=>$(id).addEventListener('input',applyTransforms));
qsa('.tab').forEach(t=>t.onclick=()=>{state.activeTcp=t.dataset.mode;renderTcp();renderIssues();});
qsa('[data-view]').forEach(b=>{b.addEventListener('click',e=>{if(e.button===0)setView(b.dataset.view);});b.addEventListener('mousedown',e=>{if(e.button!==0)e.preventDefault();});});

document.addEventListener('input',e=>{
  const t=e.target;
  if(t.dataset.jointAngle!==undefined){state.jointAngles[Number(t.dataset.jointAngle)]=num(t.value)||0;applyJointRotations();renderJointAngleRows();renderIssues();return;}
  if(t.dataset.tcp){state.tcp[state.activeTcp][t.dataset.tcp]=['toolStl','status'].includes(t.dataset.tcp)?t.value:num(t.value);renderTcp();renderIssues();}
  if(t.dataset.axisPoint!==undefined){const p=state.axisPoints[Number(t.dataset.axisPoint)],f=t.dataset.axisField;p[f]=num(t.value);p.source='manuell';syncJointsFromAxisPoints();rebuildRobotKinematics();applyTransforms();updateAxisPointVisuals();renderRows();renderIssues();}
  if(t.dataset.j!==undefined){const idx=Number(t.dataset.j),j=state.joints[idx],f=t.dataset.f;if(['x','y','z'].includes(f)){j.offset[f]=num(t.value);state.axisPoints[idx][f]=num(t.value);state.axisPoints[idx].source='manuell';rebuildRobotKinematics();applyTransforms();updateAxisPointVisuals();}else if(['min','max'].includes(f))j[f]=num(t.value);else if(f==='rotationSign'){j[f]=num(t.value);applyJointRotations();}else j[f]=t.value;renderIssues();}
});

document.addEventListener('click',e=>{
  const t=e.target;
  if(t.dataset.simAxis!==undefined)simulateAxis(Number(t.dataset.simAxis));
  const row=t.closest?.('[data-param-row]');
  if(row&&!t.matches('input,select,button'))selectAxisPoint(row.dataset.paramRow);
});

renderer.domElement.addEventListener('pointerdown', pickAxisPoint);

// Drag & Drop auf Viewer
const dz=$('dropZone');
['dragenter','dragover'].forEach(ev=>dz.addEventListener(ev,e=>{e.preventDefault();dz.classList.add('drag-over');}));
['dragleave','drop'].forEach(ev=>dz.addEventListener(ev,e=>{e.preventDefault();dz.classList.remove('drag-over');}));
dz.addEventListener('drop',e=>{const f=e.dataTransfer.files[0];if(f)loadSourceZip(f).catch(err=>alert(err.message));});

