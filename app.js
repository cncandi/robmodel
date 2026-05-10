// ═══════════════════════════════════════════════════
//  RobModel — Robot Kinematik Editor  v38
//  app.js — Haupt-Anwendungslogik (ES Module)
// ═══════════════════════════════════════════════════

import * as THREE from 'three';
import { OrbitControls }    from 'three/addons/controls/OrbitControls.js';
import { STLLoader }        from 'three/addons/loaders/STLLoader.js';
import { ColladaLoader }    from 'three/addons/loaders/ColladaLoader.js';
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

// ── Farben ────────────────────────────────────────────────────────
const colors = { Base:'#333333', A1:'#ffffff', A2:'#999999', A3:'#ff7f00', A4:'#ff7f00', A5:'#999999', A6:'#666666', Tool:'#64748b' };

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
  rebuildRobotKinematics();
  state.jointAngles = [0, -90, 90, 0, 0, 0];
  applyJointRotations();
  renderAll();
  
  updateAxisPointVisuals();
} catch (e) { showError('renderAll: ' + e); }
animate();

// ── 3D-Szene ───────────────────────────────────────────────────────
function init3d() {
  const canvas = $('viewer');
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
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

// ── Endeffektor TCP-Marker ──────────────────────────────────────
let effTcpHelper = null;

function updateEffTcpMarker() {
  if (effTcpHelper) {
    if (effTcpHelper.parent) effTcpHelper.parent.remove(effTcpHelper);
    effTcpHelper = null;
  }
  const showMarker = toolMountMode === 'a6' || !!state.effStl;
  if (!showMarker || !scene) return;
  if (toolMountMode === 'a6' && (!axisPivotGroups || !axisPivotGroups[5])) return;

  const eo = state.effOffset || {};
  const g = new THREE.Group();

  // Dicke Achsen als Zylinder (r=4, l=80)
  const axDef = [
    { color: '#ff3333', axis: new THREE.Vector3(1,0,0) },
    { color: '#33ff33', axis: new THREE.Vector3(0,1,0) },
    { color: '#3388ff', axis: new THREE.Vector3(0,0,1) },
  ];
  axDef.forEach(({ color, axis }) => {
    const mat = new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity:.4 });
    // Schaft
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(3, 3, 80, 8), mat);
    shaft.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0), axis);
    shaft.position.copy(axis).multiplyScalar(40);
    g.add(shaft);
    // Pfeilspitze
    const tip = new THREE.Mesh(new THREE.ConeGeometry(7, 18, 8), mat);
    tip.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0), axis);
    tip.position.copy(axis).multiplyScalar(89);
    g.add(tip);
  });

  // Ursprungskugel
  g.add(new THREE.Mesh(
    new THREE.SphereGeometry(8, 12, 8),
    new THREE.MeshStandardMaterial({ color: '#a855f7', emissive: '#7c3aed', emissiveIntensity:.6 })
  ));

  // Basis-Rotation A6-Werkzeugrahmen:
  // Z+ = Stoßrichtung, X+ = nach unten, Y+ = nach hinten
  const baseMatrix = new THREE.Matrix4().makeBasis(
    new THREE.Vector3( 0, -1,  0),  // X → down
    new THREE.Vector3( 0,  0, -1),  // Y → backward
    new THREE.Vector3( 1,  0,  0)   // Z → forward (Stoßrichtung)
  );
  const baseQuat = new THREE.Quaternion().setFromRotationMatrix(baseMatrix);
  // +90° um lokale Z-Achse (Kalibrierung)
  const zRot = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,0,1), Math.PI/2);
  baseQuat.multiply(zRot);

  // User-Offset (im Werkzeugrahmen, additiv)
  const offsetQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(
    (eo.rx||0)*Math.PI/180,
    (eo.ry||0)*Math.PI/180,
    (eo.rz||0)*Math.PI/180,
    'XYZ'
  ));

  // Translation im Werkzeugrahmen: Offset-Vektor mit Basis-Rotation transformieren
  const localOffset = new THREE.Vector3(eo.x||0, eo.y||0, eo.z||0);
  localOffset.applyQuaternion(baseQuat);
  g.position.copy(localOffset);
  g.quaternion.copy(baseQuat).multiply(offsetQuat);
  const parent = (toolMountMode === 'a6' && axisPivotGroups[5]) ? axisPivotGroups[5] : toolGroup;
  parent.add(g);
  effTcpHelper = g;
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
// ── Tool-Mount-Modus ────────────────────────────────────────────
let toolMountMode = 'world'; // 'world' | 'a6'

function setToolMode(mode) {
  toolMountMode = mode;
  ['world','a6'].forEach(m => {
    const btn = $('toolMode' + m.charAt(0).toUpperCase() + m.slice(1));
    if (btn) btn.classList.toggle('active', m === mode);
  });
  if (mode === 'world') detachToolFromA6();
  applyTransforms();
  updateEffTcpMarker();
}

function attachToolToA6() {
  if (!axisPivotGroups || !axisPivotGroups[5]) return;
  const _rx = deg(state.robotTr.rx), _ry = deg(state.robotTr.ry), _rz = deg(state.robotTr.rz);
  for (const [path, mesh] of meshes) {
    const file = state.stls.find(f => f.path === path) || { name: mesh.name };
    if (isTool(file)) {
      if (mesh.parent !== axisPivotGroups[5]) axisPivotGroups[5].add(mesh);
      mesh.position.set(0, 0, 0);
      // +180° um X damit Werkzeug korrekt ausgerichtet ist
      mesh.rotation.set(_rx, _ry, _rz + Math.PI);
    }
  }
}

function detachToolFromA6() {
  const _rx = deg(state.robotTr.rx), _ry = deg(state.robotTr.ry), _rz = deg(state.robotTr.rz);
  for (const [path, mesh] of meshes) {
    const file = state.stls.find(f => f.path === path) || { name: mesh.name };
    if (isTool(file)) {
      if (mesh.parent !== toolGroup) toolGroup.add(mesh);
      mesh.position.set(0, 0, 0);
      mesh.rotation.set(_rx, _ry, _rz);
    }
  }
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
  if (toolMountMode === 'a6') attachToolToA6(); else detachToolFromA6();
  updateEffTcpMarker();
  
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

  // Pivot-Weltpositionen holen
  const pivotPts = [];
  axisPivotGroups.forEach(g => {
    const wp = new THREE.Vector3(); g.getWorldPosition(wp); pivotPts.push(wp);
  });
  const skelPts = [new THREE.Vector3(0,0,0), ...pivotPts];

  // Zylinder + Kugeln neu bauen
  rebuildSkeletonMeshes(skelPts);

  // Labels + Hit-Kugeln verschieben
  axisPointGroup.children.forEach(child => {
    const idx = child.userData.skeletonIdx;
    if (idx === undefined || child.userData.isSkelCyl || child.userData.isSkelSph) return;
    const p = pivotPts[idx];
    if (!p) return;
    if (child.isSprite) {
      child.position.copy(p).add(new THREE.Vector3(0, 0, (JOINT_R[idx]||10)*2+20));
    } else {
      child.position.copy(p);
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

  // Pivot-Weltpositionen A1-A6
  const pivotPts = [];
  if (axisPivotGroups.length) {
    axisPivotGroups.forEach(g => {
      const wp = new THREE.Vector3(); g.getWorldPosition(wp); pivotPts.push(wp);
    });
  } else {
    cumulativeAxisPositions().forEach(p => pivotPts.push(p));
  }

  // Skeleton: Ursprung → A1 → A2 → … → A6 (7 Punkte, 6 Zylinder)
  const skelPts = [new THREE.Vector3(0,0,0), ...pivotPts];
  rebuildSkeletonMeshes(skelPts);

  // Labels + unsichtbare Raycaster-Kugeln: A1-A6 an den Pivot-Positionen
  pivotPts.forEach((p, i) => {
    const label = 'A' + (i + 1);
    const hitGeo = new THREE.SphereGeometry(JOINT_R[i] || 10, 8, 6);
    const hitMat = new THREE.MeshBasicMaterial({ visible: false });
    const hit = new THREE.Mesh(hitGeo, hitMat);
    hit.position.copy(p);
    hit.userData.axisIndex = i;
    hit.name = label;
    axisPointGroup.add(hit);
    axisMeshes.push(hit);

    const lbl = makeAxisLabel(label, p.clone().add(new THREE.Vector3(0, 0, (JOINT_R[i] || 10) * 2 + 20)));
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
  syncJointsFromAxisPoints(); rebuildRobotKinematics(); applyTransforms(); renderRows(); 
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
  state.sourceRobotId = null; state.sourceRobotName = null; // ID wenn aus Library geladen
  state.effStl   = null;
  state.umfStls  = [];
  state.effOffset = {x:0,y:0,z:0,rx:0,ry:0,rz:0};
  // Reset offset inputs
  ['eff-ox','eff-oy','eff-oz','eff-orx','eff-ory','eff-orz'].forEach(id=>{const el=$(id);if(el)el.value='0';});
  updateEffTcpMarker();
  renderEffRow?.(); renderUmfRows?.();
}

function enableSave()  { $('downloadJson').disabled=false; $('downloadZip').disabled=false; $('roblibBtn').disabled=false; }
function disableSave() { $('downloadJson').disabled=true;  $('downloadZip').disabled=true;  $('roblibBtn').disabled=true; }

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


// ── TCP ↔ Endeffektor TCP-Offset Sync ──────────────────────────
function setEffOffsetFromTcp(tcp) {
  // tcp.a=Rz, tcp.b=Ry, tcp.c=Rx (RobSimul XYZABC convention)
  // eff-o fields: x,y,z,rx(=c),ry(=b),rz(=a)
  state.effOffset = {
    x:  tcp.x  ?? 0,
    y:  tcp.y  ?? 0,
    z:  tcp.z  ?? 0,
    rx: tcp.c  ?? tcp.rx ?? 0,
    ry: tcp.b  ?? tcp.ry ?? 0,
    rz: tcp.a  ?? tcp.rz ?? 0
  };
  const map = {'eff-ox':'x','eff-oy':'y','eff-oz':'z','eff-orx':'rx','eff-ory':'ry','eff-orz':'rz'};
  Object.entries(map).forEach(([id,k]) => { const el=$(id); if(el) el.value = state.effOffset[k]||0; });
}

function syncTcpFromEffOffset() {
  const eo = state.effOffset || {};
  // Keep existing tcp state, only update position values
  state.tcp.auftragen = { ...state.tcp.auftragen,
    x: eo.x||0, y: eo.y||0, z: eo.z||0,
    rx: eo.rx||0, ry: eo.ry||0, rz: eo.rz||0,
    a: eo.rz||0, b: eo.ry||0, c: eo.rx||0
  };
  state.tcp.abtragen = { ...state.tcp.auftragen };
}

async function loadDemoKr8() {
  const BASE  = '../stl/';
  const FILES = ['podest.stl','a1.stl','a2.stl','a3.stl','a4.stl','a5.stl','a6.stl','tool1_tcp.stl'];
  resetData(); state.mode='source'; state.robotName='KR8';
  const btn = $('demoBtn');
  if (btn) { btn.disabled=true; btn.textContent='Lade…'; }
  try {
    // Achsgrenzen aus KR8-JSON laden
    const kr8Res = await fetch('./kr8_robsimul_v37_zielwerte.json');
    if (kr8Res.ok) {
      const kr8 = await kr8Res.json();
      if (Array.isArray(kr8.joints)) {
        kr8.joints.forEach((j,i) => {
          if (state.joints[i]) {
            state.joints[i].min = num(j.min) ?? -180;
            state.joints[i].max = num(j.max) ??  180;
          }
        });
      }
    }
    for (const fname of FILES) {
      const res = await fetch(BASE + fname);
      if (!res.ok) throw new Error(fname + ': HTTP ' + res.status);
      const buf = new Uint8Array(await res.arrayBuffer());
      state.buffers.set(fname, buf);
      state.files.push({ path: fname, name: fname, size: buf.byteLength, type: 'STL' });
    }
    splitFiles();
    state.stls.forEach(f => {
      const key = partKey(f.name);
      if (/^A[1-6]$/.test(key)) state.axisStlMap[key] = f.name;
    });
    state.robotTr={x:0,y:0,z:0,rx:0,ry:0,rz:0}; setInputs('r', state.robotTr);
    state.toolTr ={x:0,y:0,z:0,rx:0,ry:0,rz:0}; setInputs('t', state.toolTr);
    setJointAnglesToReferencePose();
    // KR8 TCP (tool1_tcp): x=364.5mm, z=46.5mm, ry=90°
    state.tcp.auftragen = { x:364.5, y:0, z:46.5, rx:0, ry:90, rz:0, toolLength:0, toolStl:'tool1_tcp', status:'KR8 Demo' };
    state.tcp.abtragen  = { ...state.tcp.auftragen };
    setEffOffsetFromTcp(state.tcp.auftragen);
    await loadStls(); enableSave(); renderAll(); setView('iso');
  } catch(e) {
    alert('Demo-Load fehlgeschlagen: ' + e.message);
    resetData(); renderAll();
  } finally {
    if (btn) { btn.disabled=false; btn.textContent='Example'; }
  }
}

function zeroAllTransforms() {
  state.robotTr={x:0,y:0,z:0,rx:0,ry:0,rz:0}; state.toolTr={x:0,y:0,z:0,rx:0,ry:0,rz:0};
  state.jointAngles=[0,-90,90,0,0,0];
  setInputs('r',state.robotTr); setInputs('t',state.toolTr);
  
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
  if(j.stlRotation){
    const r=j.stlRotation;
    state.robotTr={...state.robotTr, rx:r.rx||0, ry:r.ry||0, rz:r.rz||0};
    const set=(id,v)=>{const el=$(id);if(el)el.value=v;};
    set('rRx',r.rx||0); set('rRy',r.ry||0); set('rRz',r.rz||0);
  }
  if(j.tcp){state.tcp.auftragen=cleanTcpOrientation({...(j.tcp.auftragen||j.tcp),toolLength:j.tcp.auftragen?.toolLength??0,status:'JSON'});state.tcp.abtragen=cleanTcpOrientation({...(j.tcp.abtragen||j.tcp.auftragen||j.tcp),toolLength:j.tcp.abtragen?.toolLength??0,status:'JSON'});setEffOffsetFromTcp(state.tcp.auftragen);}
  const toolName=j.sceneModels?.tool?.name||j.tcp?.auftragen?.toolStl||j.tcp?.auftragen?.stlName;
  if(toolName)state.toolName=String(toolName).endsWith('.stl')?toolName:toolName+'.stl';
  normalizeKnownOffsets();
}

function buildJson() {
  const axNames = ['A1','A2','A3','A4','A5','A6'];
  const stlFiles = Object.fromEntries(axNames.map((ax, i) => {
    const src = state.axisStlMap[ax] || state.stls.find(f => partKey(f.name) === ax)?.name || '';
    const name = norm(src) || ('a'+(i+1));
    return [ax, { name, posx:0, posy:0, posz:0, posrx:0, posry:0, posrz:0, color: colors[ax] || '#e8a020' }];
  }));
  const tcp = state.tcp.auftragen;
  const toolName = norm(state.toolName || tcp.toolStl || '') || 'tool1_tcp';
  const eo = state.effOffset || {};
  const tcpX = eo.x??num(tcp.x)??0, tcpY = eo.y??num(tcp.y)??0, tcpZ = eo.z??num(tcp.z)??0;
  const tcpA = eo.rz??num(tcp.rz)??0, tcpB = eo.ry??num(tcp.ry)??0, tcpC = eo.rx??num(tcp.rx)??0;
  // STL-Orientierung speichern
  const rTr = state.robotTr || {};
  return {
    name: state.robotName || 'Robot',
    stlRotation: { rx: rTr.rx||0, ry: rTr.ry||0, rz: rTr.rz||0 },
    joints: state.joints.map((j,i) => ({
      name: j.name,
      axis: fixedAxisType(i),
      offset: { x: num(j.offset?.z)??0, y: num(j.offset?.y)??0, z: num(j.offset?.x)??0 },
      min: num(j.min) ?? -180,
      max: num(j.max) ??  180
    })),
    stlRefAngles: parseReferencePose(),
    tcp: { x: tcpX??0, y: tcpY??0, z: tcpZ??0, a: tcpA??0, b: tcpB??0, c: tcpC??0 },
    stlFiles,
    sceneModels: {
      pedestal: { px:0, py:0, pz:0, rx:0, ry:0, rz:0, name: 'podest' },
      tool:     { px:0, py:0, pz:0, rx:0, ry:0, rz:0, name: toolName }
    }
  };
  if (state.effStl) {
    const eo = state.effOffset || {};
    result.endeffektor = {
      name: norm(state.effStl.name), stl: 'endeffektor.stl',
      px: eo.x||0, py: eo.y||0, pz: eo.z||0,
      rx: eo.rx||0, ry: eo.ry||0, rz: eo.rz||0
    };
  }
  if (state.umfStls?.length) result.umfeld = state.umfStls.map((u,i) => ({ name: norm(u.name), stl: 'umfeld_'+(i+1)+'.stl', px:0, py:0, pz:0, rx:0, ry:0, rz:0 }));
  return result;
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
    applyJointRotations();
    if(t<1&&state.simulation.active&&state.simulation.axis===axisIndex){state.simulation.raf=requestAnimationFrame(step);}
    else{state.jointAngles=base.slice();state.jointAngles[axisIndex]=startValue;applyJointRotations();state.simulation={active:false,axis:null,raf:null};}
  };
  state.simulation.raf=requestAnimationFrame(step);
}

// ── Render-Funktionen ──────────────────────────────────────────────
function renderAll(){renderAxisStlRows();renderRows();updateAxisPointVisuals();renderTcp();const b=$('fileBadge');b.textContent=state.files.length?`${state.stls.length} STL · ${state.xmls.length} XML · ${state.jsons.length} JSON`:state.mode==='package'?'Package geladen':'Keine Datei geladen';}

function renderJointAngleRows(){const el=$('jointAngleRows');if(!el)return;el.innerHTML=state.jointAngles.map((v,i)=>`<div class="field"><label>${state.joints[i]?.name||'A'+(i+1)} ${fixedAxisType(i)}</label><input data-joint-angle="${i}" type="number" step="1" value="${v??0}"></div>`).join('');}


function renderRows(){$('jointRows').innerHTML=state.joints.map((j,i)=>{
  const ax=j.name||'A'+(i+1);
  const mapped=state.axisStlMap[ax];
  const fallback=state.stls.find(f=>partKey(f.name)===ax)?.name||null;
  const src=mapped||fallback; const stlName=src?norm(src):'—'; const col=colors[ax]||'#999999';
  return `<tr data-param-row="${i}" class="${i===state.selectedAxis?'sel':''}">
    <td><b>${esc(j.name)}</b></td>
    <td><input class="angleInput" data-joint-angle="${i}" type="number" step="0.1" value="${state.jointAngles?.[i]??0}"></td>
    <td><span class="axisDir">${axisDirectionLabel(i)}</span></td>
    <td><input data-j="${i}" data-f="x" value="${j.offset?.x??''}"></td>
    <td><input data-j="${i}" data-f="y" value="${j.offset?.y??''}"></td>
    <td><input data-j="${i}" data-f="z" value="${j.offset?.z??''}"></td>
    <td><input data-j="${i}" data-f="min" value="${j.min??''}"></td>
    <td><input data-j="${i}" data-f="max" value="${j.max??''}"></td>
    <td><select class="dirSel" data-j="${i}" data-f="rotationSign"><option value="1" ${(num(j.rotationSign)??1)>=0?'selected':''}>+</option><option value="-1" ${(num(j.rotationSign)??1)<0?'selected':''}>−</option></select></td>
    <td><input type="color" data-axis-color="${ax}" value="${col}" style="width:26px;height:22px;border:none;border-radius:3px;cursor:pointer;padding:0" title="Farbe ${ax}"></td>
    <td style="max-width:80px">
      <button data-axis-stl-label="${ax}" style="font-size:10px;padding:2px 5px;background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.15);border-radius:3px;cursor:pointer;color:#6a8fa8;max-width:76px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;display:block;width:100%" title="${stlName} — klicken zum Laden">${stlName}</button>
      <input type="file" data-axis-stl-input="${ax}" accept=".stl" style="display:none">
    </td>
    <td><button class="simBtn" data-sim-axis="${i}">▶</button></td>
  </tr>`;}).join('');}

function renderTcp(){qsa('.tab').forEach(t=>t.classList.toggle('active',t.dataset.mode===state.activeTcp));const tcp=state.tcp[state.activeTcp];qsa('[data-tcp]').forEach(i=>i.value=tcp?.[i.dataset.tcp]??'');const x=num(tcp?.x),y=num(tcp?.y),z=num(tcp?.z);tcpMarker.visible=x!==null||y!==null||z!==null;tcpMarker.position.set(x||0,y||0,z||0);}




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
    const mapped = state.axisStlMap[ax];
    const fallback = state.stls.find(f => partKey(f.name) === ax)?.name || null;
    const src = mapped || fallback;
    const name = src ? norm(src) : '—';
    const hasFile = !!src;
    const col = colors[ax] || '#999999';
    return `<div class="axis-stl-row">
      <span class="axis-stl-label">${ax}</span>
      <input type="color" class="axis-color-pick" data-ax="${ax}" value="${col}" title="Farbe ${ax}">
      <span class="axis-stl-name${hasFile ? ' has-file' : ''}" title="${name}">${name}</span>
      <button class="axis-stl-btn" data-ax="${ax}">+ STL</button>
      ${hasFile ? `<button class="axis-stl-clear" data-ax="${ax}">✕</button>` : ''}
    </div>`;
  }).join('');
}

function setAxisColor(ax, hex) {
  colors[ax] = hex;
  for (const [path, mesh] of meshes) {
    const file = state.stls.find(f => f.path === path) || { name: mesh.name };
    const key = (state.axisStlMap[ax] && norm(state.axisStlMap[ax]) === norm(file.name)) ? ax : partKey(file.name);
    if (key === ax) mesh.material.color.set(hex);
  }
}

function initAxisStlEvents() {
  document.addEventListener('input', e => {
    const cp = e.target.closest('.axis-color-pick');
    if (cp) setAxisColor(cp.dataset.ax, cp.value);
  });
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


// ── Library-Zugriff für Endeffektor & Umfeld ────────────────────
const ROBLIB_API_BASE = 'https://cnc-technik.de/robsimul/roblib/api.php';

async function libFetchByType(type) {
  const r = await fetch(ROBLIB_API_BASE + '?action=list');
  if (!r.ok) throw new Error('HTTP ' + r.status);
  const data = await r.json();
  return (data.robots || []).filter(e => (e.type || 'robot') === type);
}

async function libLoadZipAndExtract(zipUrl, stlPattern) {
  const r = await fetch(zipUrl);
  if (!r.ok) throw new Error('Download fehlgeschlagen');
  const zip = await JSZip.loadAsync(await r.arrayBuffer());
  const results = [];
  for (const name of Object.keys(zip.files)) {
    if (!zip.files[name].dir && stlPattern.test(name)) {
      const buf = await zip.files[name].async('uint8array');
      results.push({ name: name.split('/').pop(), buf });
    }
  }
  return results;
}

function libRenderList(container, items, onSelect) {
  if (!items.length) {
    container.innerHTML = '<span style="color:#4a6a8a">Keine Einträge gefunden.</span>';
    return;
  }
  container.innerHTML = items.map((item, i) =>
    `<div data-lib-idx="${i}" style="padding:4px 6px;cursor:pointer;border-radius:3px;display:flex;align-items:center;gap:6px;margin-bottom:2px">
      ${item.thumb_url ? `<img src="${item.thumb_url}" style="width:28px;height:28px;object-fit:cover;border-radius:2px;flex-shrink:0">` : '<span style="width:28px;text-align:center;font-size:16px">📦</span>'}
      <span style="color:#d8e8f0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap">${item.name}</span>
    </div>`
  ).join('');
  container.querySelectorAll('[data-lib-idx]').forEach(row => {
    row.onmouseover = () => row.style.background = 'rgba(255,255,255,.06)';
    row.onmouseout  = () => row.style.background = '';
    row.onclick = () => onSelect(items[parseInt(row.dataset.libIdx)]);
  });
}

// Endeffektor Library
$('effLibRefreshBtn').addEventListener('click', async () => {
  const el = $('effLibList');
  el.textContent = 'Lade…';
  try {
    const items = await libFetchByType('endeffektor');
    libRenderList(el, items, async item => {
      el.textContent = 'Lade ' + item.name + '…';
      try {
        const stls = await libLoadZipAndExtract(item.zip_url, /endeffektor.*\.stl$/i);
        if (!stls.length) throw new Error('Keine endeffektor.stl in ZIP');
        state.effStl = { path: stls[0].name, name: item.name + '.stl', buf: stls[0].buf };
        // Load offset from JSON if available in ZIP
        try {
          const jsonFiles = Object.keys(zip.files).filter(n => n.endsWith('.json'));
          if (jsonFiles.length) {
            const jd = JSON.parse(await zip.files[jsonFiles[0]].async('string'));
            if (jd.endeffektor) {
              const eo = jd.endeffektor;
              state.effOffset = { x:eo.px||0, y:eo.py||0, z:eo.pz||0, rx:eo.rx||0, ry:eo.ry||0, rz:eo.rz||0 };
              ['eff-ox','eff-oy','eff-oz','eff-orx','eff-ory','eff-orz'].forEach(id => {
                const key = id==='eff-ox'?'x':id==='eff-oy'?'y':id==='eff-oz'?'z':id==='eff-orx'?'rx':id==='eff-ory'?'ry':'rz';
                const el=$(id); if(el) el.value = state.effOffset[key]||0;
              });
            }
          }
        } catch(e) {}
        renderEffRow();
        updateEffTcpMarker();
        el.textContent = '✓ ' + item.name + ' geladen';
      } catch(e) { el.textContent = 'Fehler: ' + e.message; }
    });
  } catch(e) { el.textContent = 'Fehler: ' + e.message; }
});

// Umfeld Library
$('umfLibRefreshBtn').addEventListener('click', async () => {
  const el = $('umfLibList');
  el.textContent = 'Lade…';
  try {
    const items = await libFetchByType('umfeld');
    libRenderList(el, items, async item => {
      el.textContent = 'Lade ' + item.name + '…';
      try {
        const stls = await libLoadZipAndExtract(item.zip_url, /umfeld.*\.stl$/i);
        if (!stls.length) throw new Error('Keine umfeld*.stl in ZIP');
        if (!state.umfStls) state.umfStls = [];
        stls.forEach(s => state.umfStls.push({ path: s.name, name: item.name + ' · ' + s.name, buf: s.buf }));
        renderUmfRows();
        el.textContent = '✓ ' + item.name + ' (' + stls.length + ' STL) geladen';
      } catch(e) { el.textContent = 'Fehler: ' + e.message; }
    });
  } catch(e) { el.textContent = 'Fehler: ' + e.message; }
});

// ── Endeffektor & Umfeld STL ────────────────────────────────────
// state.effStl  = { path, name, buf }  (ein Endeffektor)
// state.umfStls = [{ path, name, buf }]  (mehrere Umfeld-Teile)

function renderEffRow() {
  const el = $('effStlRows');
  const badge = $('effBadge');
  if (!el) return;
  if (state.effStl) {
    el.innerHTML = `<div style="display:flex;align-items:center;gap:6px;padding:3px 0;font-size:13px;color:#d8e8f0">
      <span style="flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap">${norm(state.effStl.name)}</span>
      <button id="effClearBtn" style="background:rgba(204,51,51,.2);border:1px solid rgba(204,51,51,.4);color:#f87171;border-radius:3px;padding:2px 6px;font-size:12px;cursor:pointer">✕</button>
    </div>`;
    $('effClearBtn').onclick = () => { state.effStl = null; state.effOffset = {x:0,y:0,z:0,rx:0,ry:0,rz:0}; ['eff-ox','eff-oy','eff-oz','eff-orx','eff-ory','eff-orz'].forEach(id=>{const el=$(id);if(el)el.value='0';}); syncTcpFromEffOffset(); renderEffRow(); updateEffTcpMarker(); };
    if (badge) badge.textContent = norm(state.effStl.name);
  } else {
    el.innerHTML = '';
    if (badge) badge.textContent = '—';
  }
}

function renderUmfRows() {
  const el = $('umfStlRows');
  const badge = $('umfBadge');
  if (!el) return;
  el.innerHTML = (state.umfStls || []).map((u, i) =>
    `<div style="display:flex;align-items:center;gap:6px;padding:2px 0;font-size:12px;color:#d8e8f0">
      <span style="flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap">${norm(u.name)}</span>
      <button data-umf-idx="${i}" style="background:rgba(204,51,51,.2);border:1px solid rgba(204,51,51,.4);color:#f87171;border-radius:3px;padding:1px 5px;font-size:11px;cursor:pointer">✕</button>
    </div>`
  ).join('');
  el.querySelectorAll('[data-umf-idx]').forEach(btn => {
    btn.onclick = () => { state.umfStls.splice(parseInt(btn.dataset.umfIdx), 1); renderUmfRows(); };
  });
  if (badge) badge.textContent = (state.umfStls || []).length;
}

// File inputs
$('effStlInput').addEventListener('change', async e => {
  const file = e.target.files[0]; if (!file) return;
  const buf = new Uint8Array(await file.arrayBuffer());
  state.effStl = { path: file.name, name: file.name, buf };
  renderEffRow();
  e.target.value = '';
});

$('umfStlInput').addEventListener('change', async e => {
  const files = Array.from(e.target.files);
  if (!state.umfStls) state.umfStls = [];
  for (const file of files) {
    const buf = new Uint8Array(await file.arrayBuffer());
    state.umfStls.push({ path: file.name, name: file.name, buf });
  }
  renderUmfRows();
  e.target.value = '';
});

// ── Endeffektor & Umfeld ────────────────────────────────────────
const ROBLIB_API = 'https://www.cnc-technik.de/robsimul/roblib/api.php';

function openRoblibModal() {
  $('rl-name').value   = state.robotName || '';
  $('rl-achsen').value = 6;
  $('rl-msg').style.display = 'none';

  // Canvas-Screenshot als Thumbnail vorbelegen
  try {
    renderer.render(scene, camera);
    const canvas = renderer.domElement;
    canvas.toBlob(blob => {
      if (!blob) return;
      const file = new File([blob], 'screenshot.jpg', { type: 'image/jpeg' });
      const dt = new DataTransfer();
      dt.items.add(file);
      $('rl-thumb').files = dt.files;
      $('rl-thumb-preview').src = URL.createObjectURL(blob);
      $('rl-thumb-preview').style.display = 'block';
    }, 'image/jpeg', 0.92);
  } catch(e) { /* kein Screenshot möglich */ }

  $('roblibModal').style.display = 'flex';
}

// Type change handler — show/hide robot-specific fields
function rlTypeChanged() {
  const type = $('rl-type')?.value || 'robot';
  const fields = $('rl-robot-fields');
  if (fields) {
    const labels = fields.querySelectorAll('label');
    labels.forEach(l => l.style.display = type === 'robot' ? '' : 'none');
  }
  const nameField = $('rl-name');
  if (nameField) {
    const placeholders = {robot:'KR 8 R1420', endeffektor:'Greifer 2-Finger', umfeld:'Sicherheitszaun'};
    nameField.placeholder = placeholders[type] || '';
  }
}
$('rl-type')?.addEventListener('change', rlTypeChanged);

async function uploadToRoblib() {
  const isUpdate = _rlMode === 'update' && state.sourceRobotId;
  const btn  = $('rl-submit');
  const msg  = $('rl-msg');
  const wrap = $('rl-progress-wrap');
  const bar  = $('rl-progress-bar');
  const lbl  = $('rl-progress-label');
  const pct  = $('rl-progress-pct');

  const show = (text, ok) => {
    msg.textContent = text; msg.className = 'rl-msg ' + (ok ? 'rl-ok' : 'rl-err');
    msg.style.display = ''; wrap.style.display = 'none';
  };
  const setProgress = (label, percent) => {
    wrap.style.display = ''; msg.style.display = 'none';
    lbl.textContent = label;
    pct.textContent = Math.round(percent) + '%';
    bar.style.width = percent + '%';
    bar.style.background = percent === 100 ? '#22c55e' : '#2563eb';
  };

  const type = $('rl-type')?.value || 'robot';
  const fields = {
    name:  $('rl-name').value.trim(),
    type,
    user:  $('rl-user').value.trim(),
    pass:  $('rl-pass').value,
  };
  if (!fields.name) { show('Name fehlt.', false); return; }
  if (!fields.user || !fields.pass) { show('Zugangsdaten fehlen.', false); return; }
  // Robot-specific fields
  if (type === 'robot') {
    Object.assign(fields, {
      marke:                   $('rl-marke').value.trim(),
      modell:                  $('rl-modell').value.trim(),
      achsen:                  $('rl-achsen').value.trim(),
      reichweite_mm:           $('rl-reichweite').value.trim(),
      nutzlast_kg:             $('rl-nutzlast').value.trim(),
      gewicht_kg:              $('rl-gewicht').value.trim(),
      wiederholgenauigkeit_mm: $('rl-wdh').value.trim(),
    });
    for (const [k,v] of Object.entries(fields)) {
      if (!v && !['type','pass'].includes(k)) { show('Feld "'+k+'" fehlt.', false); return; }
    }
  }

  btn.disabled = true; btn.textContent = 'Lade…';
  try {
    // 1. ZIP erstellen
    setProgress('Erstelle ZIP…', 5);
    const prevMode = toolMountMode;
    if (prevMode !== 'world') { detachToolFromA6(); scene.updateMatrixWorld(true); }
    const zip  = new JSZip();
    const base = zipName(state.robotName || 'robot');
    zip.file(base + '.json', JSON.stringify(buildJson(), null, 2));
    for (const [, mesh] of meshes) zip.file(mesh.name, exportBinaryStl(mesh));
    if (state.effStl?.buf) zip.file('endeffektor.stl', state.effStl.buf);
    if (state.umfStls?.length) state.umfStls.forEach((u, i) => zip.file('umfeld_' + (i+1) + '.stl', u.buf));
    const zipBlob = await zip.generateAsync({ type: 'blob' }, m => setProgress('Komprimiere…', 5 + m.percent * 0.4));
    if (prevMode !== 'world') attachToolToA6();
    setProgress('Lade hoch…', 45);

    const fd = new FormData();
    for (const [k, v] of Object.entries(fields)) fd.append(k, v);
    if (isUpdate) fd.append('id', state.sourceRobotId);
    fd.append('zip', zipBlob, base + '.zip');
    const thumb = $('rl-thumb').files[0];
    if (thumb) fd.append('thumb', thumb, thumb.name);

    // 2. XHR mit Upload-Progress
    const data = await new Promise((resolve, reject) => {
      const xhr = new XMLHttpRequest();
      xhr.open('POST', ROBLIB_API + '?action=' + (isUpdate ? 'update' : 'upload'));
      xhr.upload.onprogress = e => {
        if (e.lengthComputable) setProgress('Lade hoch…', 45 + (e.loaded / e.total) * 50);
      };
      xhr.onload = () => {
        try { resolve(JSON.parse(xhr.responseText)); }
        catch (e) { reject(new Error('Ungültige Serverantwort')); }
      };
      xhr.onerror = () => reject(new Error('Verbindungsfehler'));
      xhr.send(fd);
    });

    if (data.ok) {
      setProgress('Fertig!', 100);
      state.sourceRobotId = data.robot?.id || state.sourceRobotId;
      setTimeout(() => show((isUpdate?'↻ Aktualisiert: ':'✓ Hochgeladen: ') + data.robot.name, true), 600);
    } else {
      show('Fehler: ' + data.error, false);
    }
  } catch (e) {
    show('Fehler: ' + e.message, false);
  } finally {
    btn.disabled = false; btn.textContent = 'Hochladen';
  }
}

$('demoBtn').onclick    = () => loadDemoKr8().catch(e => alert(e.message));
// Endeffektor TCP-Offset Inputs
['eff-ox','eff-oy','eff-oz','eff-orx','eff-ory','eff-orz'].forEach(id => {
  const el = $(id); if (!el) return;
  const field = id.replace('eff-o','').replace('eff-o','');
  el.addEventListener('input', () => {
    if (!state.effOffset) state.effOffset = {};
    const key = id === 'eff-ox' ? 'x' : id === 'eff-oy' ? 'y' : id === 'eff-oz' ? 'z' :
                id === 'eff-orx' ? 'rx' : id === 'eff-ory' ? 'ry' : 'rz';
    state.effOffset[key] = parseFloat(el.value) || 0;
    syncTcpFromEffOffset();
    updateEffTcpMarker();
  });
});
$('roblibBtn').onclick  = openRoblibModal;
$('toolModeWorld').onclick = () => setToolMode('world');
$('toolModeA6').onclick    = () => setToolMode('a6');
$('roblibClose').onclick = () => { $('roblibModal').style.display = 'none'; };
$('rl-submit').onclick  = uploadToRoblib;
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
$('toggleParam').onclick = () => {
  const footer = $('paramFooter');
  const btn    = $('toggleParam');
  const collapsed = footer.classList.toggle('collapsed');
  btn.textContent = collapsed ? '▲' : '▼';
};
$('resetView').onclick    = () => setView('iso');
$('toggleGrid').onclick   = () => grid.visible = !grid.visible;
initAxisStlEvents();
// Theme laden + Button
try { const saved = localStorage.getItem('robmodel_theme'); if(saved !== null) applyTheme(parseInt(saved)); } catch(e){}
$('themeBtn').onclick = () => applyTheme(_themeIdx + 1);
// Load ZIP: normal = load, Ctrl+Click label = demo
$('sourceZip').addEventListener('change', e => {
  if (!e.target.files[0]) return;
  loadSourceZip(e.target.files[0]).catch(err => alert(err.message));
});
// Ctrl+Click on the Load ZIP label triggers demo
document.querySelector('label[for="sourceZip"]').addEventListener('click', e => {
  if (e.ctrlKey) { e.preventDefault(); loadDemoKr8().catch(err => alert(err.message)); }
});
$('checkZip').addEventListener('change',  e => e.target.files[0] && loadPackageZip(e.target.files[0]).catch(err=>alert(err.message)));
$('jsonInput').addEventListener('change',  e => e.target.files[0] && loadJsonFile(e.target.files[0]));
if($('refPose'))$('refPose').addEventListener('input',()=>{setJointAnglesToReferencePose();applyJointRotations();});
// Hidden inputs for JS compat — still trigger applyTransforms
['rX','rY','rZ','rRx','rRy','rRz','tX','tY','tZ','tRx','tRy','tRz'].forEach(id=>{const el=$(id);if(el)el.addEventListener('input',applyTransforms);});
qsa('.tab').forEach(t=>t.onclick=()=>{state.activeTcp=t.dataset.mode;renderTcp();});
qsa('[data-view]').forEach(b=>{b.addEventListener('click',e=>{if(e.button===0)setView(b.dataset.view);});b.addEventListener('mousedown',e=>{if(e.button!==0)e.preventDefault();});});

document.addEventListener('input',e=>{
  const t=e.target;
  if(t.dataset.jointAngle!==undefined){state.jointAngles[Number(t.dataset.jointAngle)]=num(t.value)||0;applyJointRotations();return;}
  if(t.dataset.tcp){state.tcp[state.activeTcp][t.dataset.tcp]=['toolStl','status'].includes(t.dataset.tcp)?t.value:num(t.value);renderTcp();}
  if(t.dataset.axisPoint!==undefined){const p=state.axisPoints[Number(t.dataset.axisPoint)],f=t.dataset.axisField;p[f]=num(t.value);p.source='manuell';syncJointsFromAxisPoints();rebuildRobotKinematics();applyTransforms();updateAxisPointVisuals();renderRows();}
  if(t.dataset.j!==undefined){const idx=Number(t.dataset.j),j=state.joints[idx],f=t.dataset.f;if(['x','y','z'].includes(f)){j.offset[f]=num(t.value);state.axisPoints[idx][f]=num(t.value);state.axisPoints[idx].source='manuell';rebuildRobotKinematics();applyTransforms();updateAxisPointVisuals();}else if(['min','max'].includes(f))j[f]=num(t.value);else if(f==='rotationSign'){j[f]=num(t.value);applyJointRotations();}else j[f]=t.value;}
  // Farb-Picker in Parameterzeile
  if(t.dataset.axisColor){const ax=t.dataset.axisColor;colors[ax]=t.value;updateMeshColor(ax,t.value);renderRows();}
});

document.addEventListener('click',e=>{
  const t=e.target;
  if(t.dataset.simAxis!==undefined)simulateAxis(Number(t.dataset.simAxis));
  // STL-Button in Parameterzeile
  if(t.dataset.axisStlLabel){
    const input = t.parentElement.querySelector('[data-axis-stl-input]');
    if(input) input.click();
    return;
  }
  const row=t.closest?.('[data-param-row]');
  if(row&&!t.matches('input,select,button,label'))selectAxisPoint(row.dataset.paramRow);
});

// STL-Datei aus Parameterzeile laden (delegiert über document)
document.addEventListener('change', e => {
  const t = e.target;
  const ax = t.dataset.axisStlInput;
  if (ax && t.files[0]) {
    const file = t.files[0];
    file.arrayBuffer().then(buf => {
      const u8 = new Uint8Array(buf);
      state.buffers.set(file.name, u8);
      // Alte Datei dieser Achse aus state.files entfernen
      state.files = state.files.filter(f => partKey(f.name) !== ax || f.name === file.name);
      if (!state.files.find(f => f.name === file.name))
        state.files.push({ path: file.name, name: file.name, size: buf.byteLength, type: 'STL' });
      state.axisStlMap[ax] = file.name;
      splitFiles();
      loadStls().then(() => { renderRows(); enableSave(); });
    });
    t.value = '';
  }
});

renderer.domElement.addEventListener('pointerdown', pickAxisPoint);

// Drag & Drop auf Viewer
const dz=$('dropZone');
['dragenter','dragover'].forEach(ev=>dz.addEventListener(ev,e=>{e.preventDefault();dz.classList.add('drag-over');}));
['dragleave','drop'].forEach(ev=>dz.addEventListener(ev,e=>{e.preventDefault();dz.classList.remove('drag-over');}));
dz.addEventListener('drop',e=>{const f=e.dataTransfer.files[0];if(f)loadSourceZip(f).catch(err=>alert(err.message));});


// ── STL-Export aus BufferGeometry ───────────────────────────────
function stlFromGeometry(geo) {
  const g = geo.index ? geo.toNonIndexed() : geo.clone();
  const pos = g.getAttribute('position');
  const tri = Math.floor(pos.count / 3);
  const buf = new ArrayBuffer(84 + tri * 50);
  const view = new DataView(buf);
  view.setUint32(80, tri, true);
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3(), n = new THREE.Vector3();
  let off = 84;
  for (let i = 0; i < tri; i++) {
    a.fromBufferAttribute(pos,i*3); b.fromBufferAttribute(pos,i*3+1); c.fromBufferAttribute(pos,i*3+2);
    n.subVectors(c,b).cross(a.clone().sub(b)).normalize();
    [n.x,n.y,n.z,a.x,a.y,a.z,b.x,b.y,b.z,c.x,c.y,c.z].forEach(v=>{view.setFloat32(off,v,true);off+=4;});
    view.setUint16(off,0,true); off+=2;
  }
  g.dispose(); return buf;
}

// ── Roboter Library (roblib) ────────────────────────────────────
let _rlMode = 'new'; // 'new' | 'update'
let _allRobots = [];

function openRobotLibModal() {
  $('robotLibModal').style.display = 'flex';
  $('rl-lib-status').textContent = '';
}

function setRlMode(mode) {
  _rlMode = mode;
  $('rl-mode-new').style.background    = mode==='new'    ? 'rgba(37,99,235,.3)' : 'rgba(255,255,255,.05)';
  $('rl-mode-new').style.color         = mode==='new'    ? '#60a5fa' : '#6a8fa8';
  $('rl-mode-new').style.borderColor   = mode==='new'    ? '#2563eb' : 'rgba(255,255,255,.2)';
  $('rl-mode-update').style.background = mode==='update' ? 'rgba(255,96,0,.2)'   : 'rgba(255,255,255,.05)';
  $('rl-mode-update').style.color      = mode==='update' ? '#ff6000' : '#6a8fa8';
  $('rl-mode-update').style.borderColor= mode==='update' ? '#ff6000' : 'rgba(255,255,255,.2)';
  $('rl-modal-title').textContent = mode==='update'
    ? '↻ ROBLIB aktualisieren: ' + (state.sourceRobotName||'')
    : '→ ROBLIB hochladen';
  $('rl-submit').textContent = mode==='update' ? 'Aktualisieren' : 'Hochladen';
}

async function rlLoadRobotList() {
  const status = $('rl-lib-status');
  const list   = $('rl-lib-list');
  status.textContent = 'Lade…';
  try {
    const r = await fetch(ROBLIB_API + '?action=list');
    const d = await r.json();
    _allRobots = (d.robots || []).filter(r => (r.type||'robot') === 'robot');
    rlRenderRobotList(_allRobots);
    status.textContent = _allRobots.length + ' Roboter verfügbar';
  } catch(e) {
    status.textContent = 'Fehler: ' + e.message;
  }
}

function rlRenderRobotList(robots) {
  const list = $('rl-lib-list');
  if (!robots.length) {
    list.innerHTML = '<div style="padding:16px;font-family:monospace;font-size:11px;color:#4a6a8a">Keine Roboter gefunden.</div>';
    return;
  }
  list.innerHTML = robots.map((r, i) =>
    `<div data-robot-idx="${i}" style="padding:8px 12px;cursor:pointer;border-bottom:1px solid rgba(255,255,255,.04);display:flex;align-items:center;gap:10px">
      ${r.thumb_url ? `<img src="${r.thumb_url}" style="width:40px;height:40px;object-fit:cover;border-radius:3px;flex-shrink:0">` : '<span style="width:40px;text-align:center;font-size:22px">🦾</span>'}
      <div style="flex:1;min-width:0">
        <div style="font-family:monospace;font-size:12px;color:#d8e8f0;font-weight:700">${r.name}</div>
        <div style="font-family:monospace;font-size:10px;color:#6a8fa8">${r.marke||''} ${r.modell||''} · ${r.achsen||0} Achsen · ${r.reichweite_mm||0}mm</div>
      </div>
    </div>`
  ).join('');

  list.querySelectorAll('[data-robot-idx]').forEach(row => {
    row.onmouseover = () => row.style.background = 'rgba(255,255,255,.04)';
    row.onmouseout  = () => row.style.background = '';
    row.onclick = () => rlLoadRobotFromLib(robots[parseInt(row.dataset.robotIdx)]);
  });
}

async function rlLoadRobotFromLib(robot) {
  const status = $('rl-lib-status');
  const bar    = $('rl-lib-bar');
  const prog   = $('rl-lib-progress');
  prog.style.display = 'block'; bar.style.width = '10%';
  status.textContent = 'Lade ' + robot.name + '…';

  try {
    const buf = await new Promise((res, rej) => {
      const xhr = new XMLHttpRequest();
      xhr.open('GET', ROBLIB_API + '?action=download&id=' + robot.id);
      xhr.responseType = 'arraybuffer';
      xhr.onprogress = e => { if (e.lengthComputable) bar.style.width = (10 + e.loaded/e.total*80) + '%'; };
      xhr.onload = () => res(xhr.response);
      xhr.onerror = () => rej(new Error('Download fehlgeschlagen'));
      xhr.send();
    });

    bar.style.width = '95%';
    status.textContent = 'Verarbeite…';

    const file = new File([buf], robot.name + '.zip', { type: 'application/zip' });
    await loadSourceZip(file);

    // Merke Quelle für Update-Funktion
    state.sourceRobotId   = robot.id;
    state.sourceRobotName = robot.name;

    bar.style.width = '100%'; bar.style.background = '#22c55e';
    status.textContent = '✓ ' + robot.name + ' geladen';

    // Upload-Modal vorbereiten
    $('rl-name').value  = robot.name;
    $('rl-marke').value = robot.marke  || '';
    $('rl-modell').value= robot.modell || '';
    $('rl-achsen').value= robot.achsen || 6;
    $('rl-reichweite').value = robot.reichweite_mm || '';
    $('rl-nutzlast').value   = robot.nutzlast_kg   || '';
    $('rl-gewicht').value    = robot.gewicht_kg     || '';
    $('rl-wdh').value        = robot.wiederholgenauigkeit_mm || '';
    $('rl-mode-update').style.display = '';
    setRlMode('update');

    setTimeout(() => { $('robotLibModal').style.display = 'none'; prog.style.display = 'none'; bar.style.background = '#ff6000'; }, 800);
  } catch(e) {
    status.textContent = 'Fehler: ' + e.message;
    prog.style.display = 'none';
  }
}

$('robotLibBtn').onclick   = openRobotLibModal;
$('robotLibClose').onclick = () => { $('robotLibModal').style.display = 'none'; };
$('rl-lib-refresh').addEventListener('click', rlLoadRobotList);
$('rl-lib-search').addEventListener('input', () => {
  const q = $('rl-lib-search').value.toLowerCase();
  rlRenderRobotList(q ? _allRobots.filter(r => (r.name+r.marke+r.modell).toLowerCase().includes(q)) : _allRobots);
});

// ── ROS / GitHub Import ──────────────────────────────────────────
let _rosData = null;

// ── Kuratierte Roboterliste ──────────────────────────────────────
const ROS_ROBOTS = [
  // ABB
  {name:'ABB IRB 120',    url:'https://github.com/ros-industrial/abb/tree/kinetic-devel/abb_irb120_support'},
  {name:'ABB IRB 1200',   url:'https://github.com/ros-industrial/abb/tree/kinetic-devel/abb_irb1200_support'},
  {name:'ABB IRB 1600',   url:'https://github.com/ros-industrial/abb/tree/kinetic-devel/abb_irb1600_support'},
  {name:'ABB IRB 2400',   url:'https://github.com/ros-industrial/abb/tree/kinetic-devel/abb_irb2400_support'},
  {name:'ABB IRB 4400',   url:'https://github.com/ros-industrial/abb/tree/kinetic-devel/abb_irb4400_support'},
  {name:'ABB IRB 6640',   url:'https://github.com/ros-industrial/abb/tree/kinetic-devel/abb_irb6640_support'},
  // Fanuc
  {name:'Fanuc CR-7iA',           url:'https://github.com/ros-industrial/fanuc/tree/kinetic-devel/fanuc_cr7ia_support'},
  {name:'Fanuc LR Mate 200iD',    url:'https://github.com/ros-industrial/fanuc/tree/kinetic-devel/fanuc_lrmate200id_support'},
  {name:'Fanuc M-10iA',           url:'https://github.com/ros-industrial/fanuc/tree/kinetic-devel/fanuc_m10ia_support'},
  {name:'Fanuc M-20iA',           url:'https://github.com/ros-industrial/fanuc/tree/kinetic-devel/fanuc_m20ia_support'},
  {name:'Fanuc M-710iC/50',       url:'https://github.com/ros-industrial/fanuc/tree/kinetic-devel/fanuc_m710ic_support'},
  // Franka
  {name:'Franka Panda',   url:'https://github.com/frankaemika/franka_ros/tree/develop/franka_description'},
  // KUKA
  {name:'KUKA KR 3',      url:'https://github.com/ros-industrial/kuka_experimental/tree/kinetic-devel/kuka_kr3_support'},
  {name:'KUKA KR 6 R700', url:'https://github.com/ros-industrial/kuka_experimental/tree/kinetic-devel/kuka_kr6_support'},
  {name:'KUKA KR 10',     url:'https://github.com/ros-industrial/kuka_experimental/tree/kinetic-devel/kuka_kr10_support'},
  {name:'KUKA KR 16',     url:'https://github.com/ros-industrial/kuka_experimental/tree/kinetic-devel/kuka_kr16_support'},
  {name:'KUKA KR 210',    url:'https://github.com/ros-industrial/kuka_experimental/tree/kinetic-devel/kuka_kr210_support'},
  // Universal Robots
  {name:'UR3',  url:'https://github.com/ros-industrial/universal_robot/tree/melodic-devel/ur_description'},
  {name:'UR5',  url:'https://github.com/ros-industrial/universal_robot/tree/melodic-devel/ur_description'},
  {name:'UR10', url:'https://github.com/ros-industrial/universal_robot/tree/melodic-devel/ur_description'},
  // Yaskawa / Motoman
  {name:'Yaskawa GP7',    url:'https://github.com/ros-industrial/motoman/tree/melodic-devel/motoman_gp7_support'},
  {name:'Yaskawa GP12',   url:'https://github.com/ros-industrial/motoman/tree/melodic-devel/motoman_gp12_support'},
  {name:'Yaskawa MH5',    url:'https://github.com/ros-industrial/motoman/tree/melodic-devel/motoman_mh5_support'},
  {name:'Yaskawa MH12',   url:'https://github.com/ros-industrial/motoman/tree/melodic-devel/motoman_mh12_support'},
].sort((a,b) => a.name.localeCompare(b.name));


const colladaLoader = new ColladaLoader();

function openRosModal() {
  $('rosModal').style.display = 'flex';
  $('ros-status').textContent = '';
  $('ros-result').style.display = 'none';
  $('ros-msg').style.display = 'none';
  $('ros-progress-wrap').style.display = 'none';
  // Render robot list
  const list = $('ros-robot-list');
  if (list) {
    list.innerHTML = ROS_ROBOTS.map((r,i) => {
      const brand = r.name.split(' ')[0];
      const colors = {ABB:'#ff6600',Fanuc:'#ffcc00',Franka:'#0066ff',KUKA:'#ff6600',UR:'#004488',Universal:'#004488',Yaskawa:'#006600'};
      const col = colors[brand] || '#2563eb';
      return `<div data-ros-idx="${i}" style="padding:6px 10px;cursor:pointer;border-bottom:1px solid rgba(255,255,255,.04);display:flex;align-items:center;gap:8px">
        <span style="font-family:monospace;font-size:10px;padding:1px 5px;border-radius:3px;background:${col}22;color:${col};border:1px solid ${col}44;white-space:nowrap">${brand.toUpperCase()}</span>
        <span style="font-family:monospace;font-size:12px;color:#d8e8f0">${r.name}</span>
      </div>`;
    }).join('');
    // Event delegation — works in ES module scope
    list.onclick = e => {
      const row = e.target.closest('[data-ros-idx]');
      if (!row) return;
      const r = ROS_ROBOTS[parseInt(row.dataset.rosIdx)];
      $('ros-url').value = r.url;
      $('ros-analyze').click();
    };
    list.onmouseover = e => { const row = e.target.closest('[data-ros-idx]'); if (row) row.style.background='rgba(255,255,255,.06)'; };
    list.onmouseout  = e => { const row = e.target.closest('[data-ros-idx]'); if (row) row.style.background=''; };
  }
}

function rosSelectRobot(idx) {
  const r = ROS_ROBOTS[idx];
  $('ros-url').value = r.url;
  $('ros-analyze').click();
}
$('rosClose').onclick = () => { $('rosModal').style.display = 'none'; };

function rosParseUrl(url) {
  // https://github.com/owner/repo/tree/branch/path/to/pkg
  const m = url.match(/github\.com\/([^/]+)\/([^/]+)(?:\/tree\/([^/]+))?(?:\/(.+))?/);
  if (!m) return null;
  return { owner: m[1], repo: m[2], branch: m[3] || 'main', path: m[4] || '' };
}

async function rosApiFetch(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error('GitHub API: ' + r.status);
  return r.json();
}

async function rosRawFetch(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error('Raw fetch: ' + r.status);
  return r.text();
}

async function rosRawBinary(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error('Download: ' + r.status);
  return r.arrayBuffer();
}

function rosSetStatus(txt, pct) {
  $('ros-status').textContent = txt;
  if (pct !== undefined) {
    $('ros-progress-wrap').style.display = 'block';
    $('ros-progress-bar').style.width = pct + '%';
  }
}

function rosMsg(txt, ok) {
  const el = $('ros-msg');
  el.textContent = txt;
  el.style.cssText = 'display:block;padding:8px 12px;border-radius:4px;font-family:monospace;font-size:12px;margin-top:8px;' +
    (ok ? 'background:rgba(34,197,94,.15);color:#4ade80;border:1px solid rgba(34,197,94,.3)'
        : 'background:rgba(239,68,68,.15);color:#f87171;border:1px solid rgba(239,68,68,.3)');
}

// Parse URDF XML → joint chain info
function rosParseUrdf(xmlStr) {
  const doc = new DOMParser().parseFromString(xmlStr, 'text/xml');
  const robotName = doc.querySelector('robot')?.getAttribute('name') || '';

  // Build joint map
  const joints = [];
  doc.querySelectorAll('joint').forEach(j => {
    const type = j.getAttribute('type');
    if (!['revolute','continuous','prismatic'].includes(type)) return;
    const name   = j.getAttribute('name') || '';
    const parent = j.querySelector('parent')?.getAttribute('link') || '';
    const child  = j.querySelector('child')?.getAttribute('link') || '';
    const origin = j.querySelector('origin');
    const xyz    = (origin?.getAttribute('xyz') || '0 0 0').split(' ').map(Number);
    const rpy    = (origin?.getAttribute('rpy') || '0 0 0').split(' ').map(Number);
    const limit  = j.querySelector('limit');
    const lower  = limit ? parseFloat(limit.getAttribute('lower') || '-3.14') : -3.14;
    const upper  = limit ? parseFloat(limit.getAttribute('upper') || '3.14')  :  3.14;
    const axis   = (j.querySelector('axis')?.getAttribute('xyz') || '0 0 1').split(' ').map(Number);
    joints.push({ name, type, parent, child, xyz, rpy, lower, upper, axis });
  });

  // Build link→mesh map
  const meshMap = {};
  doc.querySelectorAll('link').forEach(l => {
    const lname = l.getAttribute('name') || '';
    const mesh  = l.querySelector('visual mesh');
    if (mesh) {
      let fn = (mesh.getAttribute('filename') || '').replace(/^.*\//, '').replace(/\.dae$/i, '.stl');
      meshMap[lname] = fn;
    }
  });

  // Traverse kinematic chain from base to tip
  const chain = [];
  let current = null;
  // Find root link (parent not appearing as child)
  const childLinks = new Set(joints.map(j => j.child));
  const roots = joints.filter(j => !childLinks.has(j.parent));
  if (roots.length) current = roots[0].parent;

  let safety = 0;
  while (current && safety++ < 20) {
    const next = joints.find(j => j.parent === current);
    if (!next) break;
    chain.push({ ...next, parentMesh: meshMap[current] || '', childMesh: meshMap[next.child] || '' });
    current = next.child;
  }

  return { robotName, chain, meshMap };
}

$('ros-analyze').onclick = async function() {
  const url = $('ros-url').value.trim();
  if (!url) return;
  const info = rosParseUrl(url);
  if (!info) { rosMsg('Ungültige GitHub-URL', false); return; }
  $('ros-result').style.display = 'none';
  $('ros-msg').style.display = 'none';
  _rosData = null;

  try {
    rosSetStatus('Analysiere Repository…', 10);
    // Fetch full tree
    const treeUrl = `https://api.github.com/repos/${info.owner}/${info.repo}/git/trees/${info.branch}?recursive=1`;
    const tree = await rosApiFetch(treeUrl);
    if (!tree.tree) throw new Error(tree.message || 'Kein Tree gefunden');

    const base = info.path ? info.path + '/' : '';
    const allFiles = tree.tree.map(f => f.path);

    // Find STL files
    const stlFiles = allFiles.filter(p => p.startsWith(base) && /\.stl$|\.dae$/i.test(p));
    // Find URDF files
    const urdfFiles = allFiles.filter(p => p.startsWith(base) && /\.urdf$|\.urdf\.xacro$/.test(p) && !p.includes('test'));

    rosSetStatus(`Gefunden: ${stlFiles.length} STL, ${urdfFiles.length} URDF`, 30);

    if (!stlFiles.length) throw new Error('Keine STL-Dateien gefunden. Pfad prüfen.');

    // Try to load URDF
    let parsed = null;
    for (const uf of urdfFiles.slice(0, 3)) {
      try {
        const rawUrl = `https://raw.githubusercontent.com/${info.owner}/${info.repo}/${info.branch}/${uf}`;
        rosSetStatus('Lese URDF: ' + uf.split('/').pop(), 50);
        const xmlStr = await rosRawFetch(rawUrl);
        parsed = rosParseUrdf(xmlStr);
        if (parsed.chain.length >= 4) break;
      } catch(e) { /* try next */ }
    }

    // Group STL by visual/ preference
    const visualStls = stlFiles.filter(p => /visual|meshes/i.test(p));
    const stlOnly = stlFiles.filter(p => /\.stl$/i.test(p));
    const meshPool = (visualStls.length ? visualStls : stlFiles);

    // Auto-map axes: use URDF chain or filename heuristics
    const axisMap = [];
    if (parsed && parsed.chain.length >= 4) {
      for (let i = 0; i < Math.min(6, parsed.chain.length); i++) {
        const j = parsed.chain[i];
        // Find matching STL in meshPool
        const stl = meshPool.find(p => {
          const fn = p.toLowerCase().split('/').pop();
          return fn === j.childMesh.toLowerCase() ||
                 fn.replace('.stl','') === j.childMesh.toLowerCase().replace('.stl','');
        }) || meshPool.find(p => {
          const fn = p.toLowerCase();
          return fn.includes('link_' + (i+1)) || fn.includes('link' + (i+1)) ||
                 fn.includes('_' + (i+1) + '.stl') || fn.includes('a' + (i+1) + '.stl');
        }) || meshPool[i] || '';

        const xyzMm = j.xyz.map(v => Math.round(v * 1000));
        axisMap.push({
          axis: 'A' + (i+1),
          stl: stl.split('/').pop(),
          stlPath: stl,
          x: xyzMm[0], y: xyzMm[1], z: xyzMm[2],
          minDeg: Math.round(j.lower * 180 / Math.PI),
          maxDeg: Math.round(j.upper * 180 / Math.PI),
          axisType: j.axis
        });
      }
    } else {
      // Heuristic: sort and map
      const sorted = meshPool
        .filter(p => /link[\s_-]?[1-6]|a[1-6]\.stl|joint[1-6]/i.test(p))
        .sort();
      for (let i = 0; i < Math.min(6, sorted.length || meshPool.length); i++) {
        const stl = sorted[i] || meshPool[i] || '';
        axisMap.push({ axis: 'A' + (i+1), stl: stl.split('/').pop(), stlPath: stl, x:0, y:0, z:0, minDeg:-180, maxDeg:180, axisType:[0,0,1] });
      }
    }

    // Also find base/podest and tool STL
    const podestStl = meshPool.find(p => /base|pedest|world|link_0/i.test(p)) || '';
    const toolStl   = meshPool.find(p => /tool|tcp|ee|flange|wrist|link_[7-9]/i.test(p)) || '';

    _rosData = {
      info, tree: allFiles, stlFiles, meshPool,
      axisMap, podestStl, toolStl,
      robotName: parsed?.robotName || info.repo,
      parsed
    };

    // Show result
    $('ros-robot-name').value = _rosData.robotName;
    $('ros-axis-table').innerHTML = axisMap.map((a, i) => `
      <div style="display:grid;grid-template-columns:32px 1fr 60px 60px 50px 50px;gap:4px;align-items:center;padding:3px 0;border-bottom:1px solid rgba(255,255,255,.05)">
        <span style="color:#60a5fa;font-weight:700">${a.axis}</span>
        <span style="color:#d8e8f0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap" title="${a.stlPath}">${a.stl || '—'}</span>
        <span style="color:#6a8fa8">x:${a.x}</span>
        <span style="color:#6a8fa8">z:${a.z}</span>
        <span style="color:#4ade80">${a.minDeg}°</span>
        <span style="color:#f87171">${a.maxDeg}°</span>
      </div>`).join('') +
      (podestStl ? `<div style="color:#6a8fa8;margin-top:4px;font-size:10px">Basis: ${podestStl.split('/').pop()}</div>` : '') +
      (toolStl   ? `<div style="color:#6a8fa8;font-size:10px">Tool: ${toolStl.split('/').pop()}</div>` : '');

    $('ros-result').style.display = 'block';
    rosSetStatus(`✓ ${axisMap.length} Achsen erkannt${parsed ? ' (URDF)' : ' (Heuristik)'}`, 100);

  } catch(e) {
    rosMsg('Fehler: ' + e.message, false);
    rosSetStatus('', undefined);
    $('ros-progress-wrap').style.display = 'none';
  }
};

$('ros-load').onclick = async function() {
  if (!_rosData) return;
  const { info, axisMap, podestStl, toolStl, parsed } = _rosData;
  const robotName = $('ros-robot-name').value.trim() || info.repo;
  const rawBase = `https://raw.githubusercontent.com/${info.owner}/${info.repo}/${info.branch}/`;

  const btn = $('ros-load');
  btn.disabled = true;
  resetData(); state.mode = 'source'; state.robotName = robotName;

  try {
    const total = axisMap.length + (podestStl ? 1 : 0) + (toolStl ? 1 : 0);
    let loaded = 0;

    const loadStlFile = async (stlPath, targetName) => {
      if (!stlPath) return;
      rosSetStatus('Lade ' + stlPath.split('/').pop() + '…', Math.round(loaded/total*90));
      const isDae = /\.dae$/i.test(stlPath);
      if (isDae) {
        await new Promise((res, rej) => {
          colladaLoader.load(rawBase + stlPath, dae => {
            try {
              const meshes3 = [];
              dae.scene.updateMatrixWorld(true);
              dae.scene.traverse(c => { if (c.isMesh && c.geometry) meshes3.push(c); });
              if (meshes3.length) {
                // Merge all sub-meshes into one geometry (scale m→mm)
                const combined = new THREE.BufferGeometry();
                const positions = [];
                meshes3.forEach(m => {
                  const g = m.geometry.clone();
                  g.applyMatrix4(m.matrixWorld);
                  const pos = g.getAttribute('position');
                  for (let k = 0; k < pos.count; k++) {
                    positions.push(pos.getX(k)*1000, pos.getY(k)*1000, pos.getZ(k)*1000);
                  }
                  g.dispose();
                });
                combined.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
                combined.computeVertexNormals();
                const buf = stlFromGeometry(combined);
                state.buffers.set(targetName, new Uint8Array(buf));
                state.files.push({ path: targetName, name: targetName, size: buf.byteLength, type: 'STL' });
                combined.dispose();
              }
              loaded++; res();
            } catch(e) { loaded++; res(); } // skip bad mesh
          }, undefined, e => { loaded++; res(); }); // skip on error
        });
      } else {
        // STL: load and scale m→mm
        const raw = await rosRawBinary(rawBase + stlPath);
        const geo = loader.parse(raw);
        geo.computeVertexNormals();
        // Scale vertices ×1000 (ROS uses meters, RobModel uses mm)
        const pos = geo.getAttribute('position');
        for (let k = 0; k < pos.count; k++) {
          pos.setXYZ(k, pos.getX(k)*1000, pos.getY(k)*1000, pos.getZ(k)*1000);
        }
        pos.needsUpdate = true;
        const buf = stlFromGeometry(geo);
        geo.dispose();
        state.buffers.set(targetName, new Uint8Array(buf));
        state.files.push({ path: targetName, name: targetName, size: buf.byteLength, type: 'STL' });
        loaded++;
      }
    };

    // Load axis STLs
    for (let i = 0; i < axisMap.length; i++) {
      const a = axisMap[i];
      if (a.stlPath) await loadStlFile(a.stlPath, 'a' + (i+1) + '.stl');
    }
    if (podestStl) await loadStlFile(podestStl, 'podest.stl');
    if (toolStl)   await loadStlFile(toolStl, 'tool1_tcp.stl');

    splitFiles();
    state.stls.forEach(f => {
      const key = partKey(f.name);
      if (/^A[1-6]$/.test(key)) state.axisStlMap[key] = f.name;
    });

    // Apply joint data from URDF chain
    if (parsed?.chain) {
      parsed.chain.slice(0, 6).forEach((j, i) => {
        if (state.joints[i]) {
          // Convert xyz (meters) to mm, x↔z swap for RobModel display
          state.joints[i].offset = {
            x: Math.round(j.xyz[2] * 1000), // z in URDF → x in RobModel display
            y: Math.round(j.xyz[1] * 1000),
            z: Math.round(j.xyz[0] * 1000)  // x in URDF → z in RobModel display
          };
          state.axisPoints[i].x = state.joints[i].offset.x;
          state.axisPoints[i].y = state.joints[i].offset.y;
          state.axisPoints[i].z = state.joints[i].offset.z;
          state.joints[i].min = Math.round(j.lower * 180 / Math.PI);
          state.joints[i].max = Math.round(j.upper * 180 / Math.PI);
          // Determine axis type from URDF axis vector
          const ax = j.axis.map(Math.abs);
          const maxIdx = ax.indexOf(Math.max(...ax));
          state.joints[i].axis = ['Rx','Ry','Rz'][maxIdx];
        }
      });
      syncJointsFromAxisPoints?.();
    }

    state.robotTr = {x:0,y:0,z:0,rx:0,ry:0,rz:0}; setInputs('r', state.robotTr);
    state.toolTr  = {x:0,y:0,z:0,rx:0,ry:0,rz:0}; setInputs('t', state.toolTr);
    setJointAnglesToReferencePose();
    await loadStls(); enableSave(); renderAll(); setView('iso');
    $('rosModal').style.display = 'none';
    rosSetStatus('', undefined);
    $('ros-progress-wrap').style.display = 'none';

  } catch(e) {
    rosMsg('Fehler beim Laden: ' + e.message, false);
    resetData(); renderAll();
  } finally {
    btn.disabled = false; btn.textContent = 'Laden & in RobModel öffnen';
  }
};
