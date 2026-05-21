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
  { x: 150,  y: 0, z: 450  },  // A1 Rz  — X=horizontal (Three.X), Z=vertikal (Three.Z)
  { x: 0,    y: 0, z: 610  },  // A2 Ry — z-offset (normales Mapping)
  { x: 0,    y: 0, z: 200  },  // A3 Ry
  { x: 630,  y: 0, z: 0    },  // A4 Rx
  { x: 80,   y: 0, z: 0    },  // A5 Ry
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
  axisStlMap: { A1:null, A2:null, A3:null, A4:null, A5:null, A6:null },
  axisStlParts: { A1:[], A2:[], A3:[], A4:[], A5:[], A6:[] },
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
  },
  effektoren: [],
  activeEff: 0,
  umfElemente: [],
  activeUmf: 0,
  schienen: [],
  objekte: [],
  positioners: [],
  festeObjekte: [],
};

// ── Three.js Variablen ────────────────────────────────────────────
let scene, camera, perspCamera, orthoCamera, isOrtho=false, renderer, controls, grid, robotGroup, toolGroup, tcpMarker, kinematicsRoot, railGroup;
let axisPointGroup, axisLine, transformControls, raycaster, mouse, csHelperGroup;
var objekteGroups = [];
var effektorGroups = [];
var positionerGroups = [];
var umfGroups = [];
var festeGrps = []; // [{containerGrp, pivotGrp, meshGrp, pivotSphere}]
const meshes = new Map();
const axisMeshes = [];
const axisPivotGroups = [];
const skeletonCyls = []; const skeletonSphs = [];
const LINK_R=[28,20,16,12,8,6];
let _jointSizeScale = 50/30;  // Skalierungsfaktor für Knotenpunkte (Standard=50)
function JOINT_R_at(i) { const base=[40,38,30,24,20,16]; return Math.round((base[i]||10)*_jointSizeScale); }
const LINK_COLOR=0xcc4400; const JOINT_COLOR=0xe8a020;
const loader = new STLLoader();

// ── Fehleranzeige ──────────────────────────────────────────────────
var _paramTab = 'a'; // must be before renderAll
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
  perspCamera = new THREE.PerspectiveCamera(45, 1, 1, 100000);
  perspCamera.position.set(1600, -2200, 1300);
  perspCamera.up.set(0, 0, 1);
  orthoCamera = new THREE.OrthographicCamera(-1, 1, 1, -1, 1, 100000);
  orthoCamera.position.set(1600, -2200, 1300);
  orthoCamera.up.set(0, 0, 1);
  camera = perspCamera;
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
  railGroup = new THREE.Group();
  scene.add(railGroup);

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
  const showMarker = toolMountMode === 'a6' || state.effektoren.length > 0;
  if (!showMarker || !scene) return;
  if (toolMountMode === 'a6' && (!axisPivotGroups || !axisPivotGroups[5])) return;

  const eo = state.effektoren[state.activeEff]?.offset || {};
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
  if (isOrtho) {
    _orthoUpdateFrustum(w / h);
  } else {
    perspCamera.aspect = w / h;
    perspCamera.updateProjectionMatrix();
  }
}
function _orthoUpdateFrustum(aspect) {
  const dist = orthoCamera.position.distanceTo(controls.target);
  const fovRad = THREE.MathUtils.degToRad(perspCamera.fov / 2);
  const h2 = Math.tan(fovRad) * dist * orthoCamera.zoom;
  orthoCamera.left   = -h2 * aspect;
  orthoCamera.right  =  h2 * aspect;
  orthoCamera.top    =  h2;
  orthoCamera.bottom = -h2;
  orthoCamera.updateProjectionMatrix();
}
function toggleCameraMode() {
  const w = renderer.domElement.width;
  const h = renderer.domElement.height;
  const aspect = w / h;
  if (isOrtho) {
    // → Perspektive
    perspCamera.position.copy(orthoCamera.position);
    perspCamera.quaternion.copy(orthoCamera.quaternion);
    perspCamera.up.copy(orthoCamera.up);
    camera = perspCamera;
    controls.object = camera;
    transformControls.camera = camera;
    perspCamera.aspect = aspect;
    perspCamera.updateProjectionMatrix();
  } else {
    // → Orthografisch: Frustum so setzen dass Sichtgröße identisch bleibt
    orthoCamera.position.copy(perspCamera.position);
    orthoCamera.quaternion.copy(perspCamera.quaternion);
    orthoCamera.up.copy(perspCamera.up);
    orthoCamera.zoom = 1;
    camera = orthoCamera;
    controls.object = camera;
    transformControls.camera = camera;
    _orthoUpdateFrustum(aspect);
  }
  isOrtho = !isOrtho;
  controls.update();
  const btn = $('camModeBtn');
  if (btn) {
    btn.textContent = isOrtho ? '🔲 Perspektive' : '⬜ Ortho';
    btn.style.background = isOrtho ? 'rgba(37,99,235,.2)' : 'rgba(255,255,255,.05)';
    btn.style.borderColor = isOrtho ? 'rgba(37,99,235,.4)' : 'rgba(255,255,255,.15)';
    btn.style.color = isOrtho ? '#60a5fa' : '#6a8fa8';
  }
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
  // Rotation auf einzelne Meshes (kinematicsRoot bleibt unrotiert → Skelett bleibt stabil)
  kinematicsRoot.rotation.set(0, 0, 0);
  kinematicsRoot.position.set(state.robotTr.x, state.robotTr.y, state.robotTr.z);
  toolGroup.position.set(state.toolTr.x, state.toolTr.y, state.toolTr.z);
  toolGroup.rotation.set(_rx, _ry, _rz);
  // STL-Korrektur auf ALLE Meshes gleichmäßig anwenden
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
  let x=0, y=0, z=0;
  const pts = [new THREE.Vector3(0,0,0)];
  for (let i=0; i<5; i++) {
    const p = state.axisPoints[i];
    if (!p) { pts.push(new THREE.Vector3(x,y,z)); continue; }
    x += num(p.x)||0; z += num(p.z)||0;  // einheitliches Mapping für alle Achsen
    y += num(p.y)||0;
    pts.push(new THREE.Vector3(x,y,z));
  }
  return pts;
}

function syncJointsFromAxisPoints() {
  // Altes Format migrieren: axisPoints[1].x wurde früher als z-Offset gespeichert
  const ap1 = state.axisPoints[1];
  if (ap1 && num(ap1.x) !== 0 && (num(ap1.z) || 0) === 0 && ap1.source !== 'manuell') {
    ap1.z = ap1.x; ap1.x = 0;
    if (state.joints[1]?.offset) { state.joints[1].offset.z = state.joints[1].offset.x; state.joints[1].offset.x = 0; }
  }
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
    mesh.userData.axisKey = key;
    const partDef=(state.axisStlParts[key]||[]).find(p=>norm(p.name)===norm(file.name));
    if (partDef) mesh.material.color.set(partDef.color||'#e8a020');
    const m = key.match(/^A([1-6])$/);
    if (m) {
      const idx = Number(m[1]) - 1;
      const pivot = pts[idx] || new THREE.Vector3();
      axisPivotGroups[idx].add(mesh);
      mesh.position.copy(pivot.clone().multiplyScalar(-1));
    } else { robotGroup.add(mesh); }
  }
  applyJointRotations();
  // Re-attach a6-mounted labels to new A6 pivot
  (state.objekte||[]).forEach((_,i)=>{ if(state.objekte[i]?.mountMode==='a6') rebuildObjektMesh(i); });
  // Re-attach Endeffektoren to new A6 pivot
  (state.effektoren||[]).forEach((_,i)=>rebuildEffMesh(i));
  // Rebuild Umgebung meshes
  (state.umfElemente||[]).forEach((_,i)=>rebuildUmfMesh(i));
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
  const ref = parseReferencePose();
  state.jointAngles = ref.slice();
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
    const sph = new THREE.Mesh(new THREE.SphereGeometry(JOINT_R_at(i-1) || 10, 12, 8), mat);
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
      child.position.copy(p).add(new THREE.Vector3(0, 0, (JOINT_R_at(idx)||10)*2+20));
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
  if (!(state.joints||[]).length) return; // no robot — skip skeleton
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
    const hitGeo = new THREE.SphereGeometry(JOINT_R_at(i) || 10, 8, 6);
    const hitMat = new THREE.MeshBasicMaterial({ visible: false });
    const hit = new THREE.Mesh(hitGeo, hitMat);
    hit.position.copy(p);
    hit.userData.axisIndex = i;
    hit.name = label;
    hit.layers.set(2);  // Eigenes Layer für Raycast-Isolation
    axisPointGroup.add(hit);
    axisMeshes.push(hit);

    const lbl = makeAxisLabel(label, p.clone().add(new THREE.Vector3(0, 0, (JOINT_R_at(i) || 10) * 2 + 20)));
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
  sp.userData.isAxisLabel = true;
  return sp;
}

function pickAxisPoint(event) {
  if (!axisMeshes.length) return;
  // Kein Pick während laufendem Drag
  if (transformControls && transformControls.dragging) return;
  // TransformControls vorher lösen damit er den Klick nicht schluckt
  if (transformControls) transformControls.detach();

  const rect = renderer.domElement.getBoundingClientRect();
  mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(mouse, camera);
  var _prevLayers = raycaster.layers.mask;
  raycaster.layers.set(2);  // Nur Hit-Spheres (Layer 2) — Roboter-Meshes ignorieren
  const hit = raycaster.intersectObjects(axisMeshes, false)[0];
  raycaster.layers.mask = _prevLayers;
  if (hit) {
    state.selectedAxis = hit.object.userData.axisIndex;
    selectAxisPoint(state.selectedAxis);
    event.preventDefault();
  }
}

function onAxisObjectMoved() {
  const mesh = transformControls.object; if (!mesh) return;
  const idx = mesh.userData.axisIndex;
  if (idx === 0) return;  // A1 ist immer an Ursprung — nicht verschiebbar
  const apIdx = idx - 1;  // axisPoints-Index
  const pts = cumulativeAxisPositions();
  const prev = pts[idx - 1] || new THREE.Vector3(0,0,0);
  const local = mesh.position.clone().sub(prev);
  const p = state.axisPoints[apIdx];
  if (!p) return;

  // Einheitliches Mapping: XYZ-Pfeil → XYZ-Offset für alle Achsen
  p.x = Number(local.x.toFixed(3));
  p.y = Number(local.y.toFixed(3));
  p.z = Number(local.z.toFixed(3));
  p.source = 'manuell';
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
  // Aktive Zeile in Tabelle hervorheben
  if (typeof renderRows === 'function') renderRows();
  // TransformControls direkt aktivieren (außer A1)
  if (state.selectedAxis > 0 && axisMeshes[state.selectedAxis]) {
    transformControls.attach(axisMeshes[state.selectedAxis]);
    transformControls.visible = true;
  } else {
    transformControls.detach();
  }
}

// ── STEP / OCCT Import ─────────────────────────────────────────────
let _occtModule = null, _occtLoading = null;
function getOCCT() {
  if (_occtModule) return Promise.resolve(_occtModule);
  if (_occtLoading) return _occtLoading;
  const initOcct = () => occtimportjs({
    locateFile: p => 'https://cdn.jsdelivr.net/npm/occt-import-js@0.0.23/dist/' + p
  }).then(m => { _occtModule = m; _occtLoading = null; return m; });
  if (typeof occtimportjs !== 'undefined') {
    return (_occtLoading = initOcct());
  }
  return (_occtLoading = new Promise((res, rej) => {
    const s = document.createElement('script');
    s.src = 'https://cdn.jsdelivr.net/npm/occt-import-js@0.0.23/dist/occt-import-js.js';
    s.onload = () => initOcct().then(res).catch(rej);
    s.onerror = () => rej(new Error('occt-import-js laden fehlgeschlagen'));
    document.head.appendChild(s);
  }));
}
function stepToGeometry(arrayBuffer) {
  return getOCCT().then(occt => {
    occt.FS.writeFile('/input.stp', new Uint8Array(arrayBuffer));
    const result = occt.ReadStepFile('/input.stp', null);
    if (!result.success || !result.meshes?.length) throw new Error('STEP lesen fehlgeschlagen');
    const positions = [], normals = [], indices = [];
    let offset = 0;
    for (const mesh of result.meshes) {
      const pos = mesh.attributes.position.array;
      const nor = mesh.attributes.normal?.array;
      const idx = mesh.index?.array;
      for (const v of pos) positions.push(v);
      if (nor) for (const v of nor) normals.push(v);
      if (idx) for (const v of idx) indices.push(v + offset);
      offset += pos.length / 3;
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    if (normals.length) geo.setAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));
    if (indices.length) geo.setIndex(indices);
    geo.computeVertexNormals();
    return geo;
  });
}

// ── OSD → Binary STL ─────────────────────────────────────────────
function osdToBinaryStl(arrayBuffer) {
  const data = new Uint8Array(arrayBuffer);
  if (!data.length) return new ArrayBuffer(84);
  // Prüfen ob bereits binäres STL (80-Byte-Header, dann uint32 Dreiecksanzahl)
  if (data.length >= 84) {
    const view84 = new DataView(arrayBuffer);
    const triCount = view84.getUint32(80, true);
    if (triCount > 0 && 84 + triCount * 50 === data.length) {
      return arrayBuffer;  // schon binäres STL
    }
  }
  const view = new DataView(arrayBuffer);
  const hdr = new TextDecoder('utf-8',{fatal:false}).decode(data.slice(0,100));
  const isV2 = hdr.includes('+SprutCAM: OpenGL stream file (version 2.00)');
  const headerSize = isV2 ? 105 : 165;
  const CMD = [4,0,12,12,12,isV2?1:0,64,128,48,72,104];
  const LINE = new Set([1,2,3]);
  const prims = [];
  let cur = null, snx=0,sny=0,snz=0;
  for (let i=headerSize; i<data.length; ) {
    const code=data[i], ls=i+1, cs=CMD[code]??0;
    i += 1 + cs;
    switch(code) {
      case 0: { const t=view.getInt32(ls,true); if(t<10){cur={t,vx:[],vy:[],vz:[],nx:[],ny:[],nz:[]};prims.push(cur);} break; }
      case 1: {
        if(cur){ for(let j=0;j<cur.vx.length;j++){const x=cur.vx[j],y=cur.vy[j],z=cur.vz[j];cur.vx[j]=-z;cur.vy[j]=x;cur.vz[j]=y;} for(let j=0;j<cur.nx.length;j++){const x=cur.nx[j],y=cur.ny[j],z=cur.nz[j];cur.nx[j]=-z;cur.ny[j]=x;cur.nz[j]=y;} }
        cur=null; break;
      }
      case 2: { snx=view.getFloat32(ls,true);sny=view.getFloat32(ls+4,true);snz=view.getFloat32(ls+8,true); if(cur){cur.nx.push(snx);cur.ny.push(sny);cur.nz.push(snz);} break; }
      case 3: { if(cur){cur.vx.push(view.getFloat32(ls,true));cur.vy.push(view.getFloat32(ls+4,true));cur.vz.push(view.getFloat32(ls+8,true));if(cur.nx.length<cur.vx.length){cur.nx.push(snx);cur.ny.push(sny);cur.nz.push(snz);}} break; }
    }
  }
  const tris=[];
  const cn=(ax,ay,az,bx,by,bz,cx,cy,cz)=>{const ux=bx-ax,uy=by-ay,uz=bz-az,vx=cx-ax,vy=cy-ay,vz=cz-az;const nx=uy*vz-uz*vy,ny=uz*vx-ux*vz,nz=ux*vy-uy*vx;const l=Math.sqrt(nx*nx+ny*ny+nz*nz);return l>1e-10?[nx/l,ny/l,nz/l]:[0,0,1];};
  for(const p of prims){
    if(LINE.has(p.t)) continue;
    const n=p.vx.length; if(n<3) continue;
    let idx;
    if(p.t===5){idx=[];for(let i=0;i<n-2;i++){if(i%2===0){idx.push(i,i+1,i+2);}else{idx.push(i+1,i,i+2);}}}
    else if(p.t===6){idx=[];for(let i=1;i<n-1;i++)idx.push(0,i,i+1);}
    else{idx=Array.from({length:n},(_,i)=>i);}
    for(let i=0;i+2<idx.length;i+=3){
      const ia=idx[i],ib=idx[i+1],ic=idx[i+2];
      const ax=p.vx[ia],ay=p.vy[ia],az=p.vz[ia];
      const bx=p.vx[ib],by=p.vy[ib],bz=p.vz[ib];
      const cx=p.vx[ic],cy=p.vy[ic],cz=p.vz[ic];
      let fn=p.nx.length===n?[p.nx[ia],p.ny[ia],p.nz[ia]]:null;
      if(fn){const l=Math.sqrt(fn[0]*fn[0]+fn[1]*fn[1]+fn[2]*fn[2]);if(l>1e-10){fn=[fn[0]/l,fn[1]/l,fn[2]/l];}else fn=cn(ax,ay,az,bx,by,bz,cx,cy,cz);}else fn=cn(ax,ay,az,bx,by,bz,cx,cy,cz);
      tris.push(fn[0],fn[1],fn[2],ax,ay,az,bx,by,bz,cx,cy,cz);
      tris.push(-fn[0],-fn[1],-fn[2],ax,ay,az,cx,cy,cz,bx,by,bz);
    }
  }
  const tc=tris.length/12, out=new ArrayBuffer(84+tc*50), dv=new DataView(out);
  dv.setUint32(80,tc,true);
  let off=84;
  for(let i=0;i<tris.length;i+=12){for(let j=0;j<12;j++){dv.setFloat32(off,tris[i+j],true);off+=4;}dv.setUint16(off,0,true);off+=2;}
  return out;
}

async function parseGeometry(arrayBuffer, fileName) {
  const ext = (fileName || '').split('.').pop().toLowerCase();
  if (ext === 'stp' || ext === 'step') return stepToGeometry(arrayBuffer);
  if (ext === 'osd') return loader.parse(osdToBinaryStl(arrayBuffer));
  return loader.parse(arrayBuffer);
}
async function extractFromZip(file) {
  const zip = await JSZip.loadAsync(file);
  const entries = Object.values(zip.files).filter(f => !f.dir && /\.(stp|step|stl|osd)$/i.test(f.name));
  if (!entries.length) throw new Error('Keine STL/STEP-Datei im ZIP gefunden');
  const first = entries[0];
  return { buf: await first.async('arraybuffer'), name: first.name.split('/').pop() };
}

// Liest eine STL/OSD/ZIP-Datei und gibt {buf: Uint8Array, name: string} zurück
async function readStlFile(file) {
  let rawBuf, fname;
  if (/\.zip$/i.test(file.name)) {
    const r = await extractFromZip(file); rawBuf = r.buf; fname = r.name;
  } else { rawBuf = await file.arrayBuffer(); fname = file.name; }
  if (/\.osd$/i.test(fname)) { rawBuf = osdToBinaryStl(rawBuf); fname = fname.replace(/\.osd$/i, '.stl'); }
  return { buf: new Uint8Array(rawBuf), name: fname };
}

// ── STL laden ──────────────────────────────────────────────────────
function partKey(n) {
  const s = norm(n);
  const a = s.match(/a([1-6])$/); if (a) return 'A' + a[1];
  const j = s.match(/joint[_ \-]*([1-6])/i); if (j) return 'A' + j[1];
  if (/tool|tcp|meltio/.test(s)) return 'Tool';
  if (/podest|base/.test(s)) return 'Base';
  return 'A1';
}
function isTool(f) { const n = norm(f.name||f); const tool = norm(state.tcp.auftragen.toolStl||state.toolName); return n===tool||/tool1_tcp|tool|tcp|meltio/.test(n); }
function findStl(stem) { const s = norm(stem); return state.stls.find(f=>norm(f.name)===s)?.name || state.stls.find(f=>norm(f.name).includes(s)||s.includes(norm(f.name)))?.name || null; }
function clearGroup(g) { while (g.children.length) g.remove(g.children[0]); }

async function loadStls() {
  Object.entries(state.axisStlParts||{}).forEach(([ax,parts]) => {
    parts.forEach(p => {
      if (p.buf&&!state.buffers.has(p.name)){state.buffers.set(p.name,p.buf);if(!state.stls.find(f=>f.name===p.name))state.stls.push({path:p.name,name:p.name,type:'STL',size:p.buf.byteLength});}
    });
  });
  // STL-Rotation aus Inputs lesen und in Geometrie einbrennen
  const stlRx = parseFloat($('rRx')?.value || 0) || 0;
  const stlRy = parseFloat($('rRy')?.value || 0) || 0;
  const stlRz = parseFloat($('rRz')?.value || 0) || 0;
  const hasRot = stlRx || stlRy || stlRz;
  const rotMatrix = hasRot ? new THREE.Matrix4().makeRotationFromEuler(
    new THREE.Euler(stlRx*Math.PI/180, stlRy*Math.PI/180, stlRz*Math.PI/180)
  ) : null;

  for (const f of state.stls) {
    try {
      const u8 = state.buffers.get(f.path);
      const g = await parseGeometry(u8.buffer.slice(u8.byteOffset, u8.byteOffset + u8.byteLength), f.name);
      // Rotation NICHT einbrennen — wird über kinematicsRoot/robotGroup angewendet
      g.computeVertexNormals();
      const mat = new THREE.MeshStandardMaterial({ color: colors[partKey(f.name)], roughness: .62, metalness: .08 });
      const mesh = new THREE.Mesh(g, mat); mesh.name = f.name;
      meshes.set(f.path, mesh);
    } catch (e) { console.warn(e); }
  }
  rebuildRobotKinematics(); applyTransforms(); fitCamera();
}

// ── ZIP / JSON ─────────────────────────────────────────────────────
const typeOf = n => /\.(stl|osd)$/i.test(n)?'STL':/\.xml$/i.test(n)?'XML':/\.json$/i.test(n)?'JSON':'Datei';
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
  state.effektoren = [];
  state.activeEff  = 0;
  state.umfStls   = [];
  state.umfElemente = [];
  state.activeUmf   = 0;
  ['A1','A2','A3','A4','A5','A6'].forEach(ax => { state.axisStlMap[ax]=null; state.axisStlParts[ax]=[]; });
  updateEffTcpMarker();
  renderEffRow?.(); renderUmfRows?.();
}

function enableSave()  { $('downloadJson').disabled=false; $('downloadZip').disabled=false; $('roblibBtn').disabled=false; }
function disableSave() { $('downloadJson').disabled=true;  $('downloadZip').disabled=true; /* roblibBtn always enabled */ }

function clearAll() {
  if(!confirm('Kompletten Viewport leeren?')) return;
  clearAll._inner(true);
}
clearAll._inner = function(clearSkeleton) {
  // Reset robot state
  resetData(); disableSave();
  if (clearSkeleton) state.joints = []; // Achszeilen aus Tabelle entfernen
  // Clear robot 3D groups
  clearGroup(robotGroup);
  clearGroup(toolGroup); toolGroup.add(tcpMarker);
  clearGroup(kinematicsRoot);
  clearGroup(axisPointGroup);
  clearGroup(csHelperGroup);
  skeletonCyls.length=0; skeletonSphs.length=0; axisPivotGroups.length=0;
  kinematicsRoot.position.set(0,0,0);
  // Reset rail
  if(railGroup) clearGroup(railGroup);
  state.schienen=[];
  // Reset objects
  (objekteGroups||[]).forEach(g=>{ if(g&&g.parent) g.parent.remove(g); });
  objekteGroups.length=0; state.objekte=[];
  // Reset positioners
  (state.positioners||[]).forEach((_,i)=>_removePosGroup(i));
  positionerGroups.length=0; state.positioners=[];
  // Reset feste Objekte
  (festeGrps||[]).forEach(g=>{ if(g?.parent) g.parent.remove(g); });
  festeGrps.length=0; state.festeObjekte=[];
  // Reset Umgebung
  (umfGroups||[]).forEach(g=>{ if(g&&g.parent) g.parent.remove(g); });
  umfGroups.length=0; state.umfElemente=[];
  // File-Inputs leeren damit gleiche Dateien/Ordner erneut geladen werden können
  const _si=$('sourceZip'); if(_si) _si.value='';
  const _sf=$('sourceFolder'); if(_sf) _sf.value='';
  const _rp=$('refPose'); if(_rp) _rp.value='0,-90,90,0,0,0';
  state.referencePose=[0,-90,90,0,0,0];
  // Render minimal
  renderRailRows(); renderObjRows(); renderPosRows(); renderFixRows();
  renderAxisStlRows(); renderRows(); renderTcp();
  setView('iso');
}

async function loadSourceZip(file) {
  resetData(); state.mode='source';
  const z = await readZip(file); state.files=z.files; state.buffers=z.buffers;
  splitFiles();
  const hasOsd=(state.stls||[]).some(f=>f.name&&/\.osd$/i.test(f.name));
  state.robotTr=hasOsd?{x:0,y:0,z:0,rx:0,ry:0,rz:0}:defaultRobotTr(); setInputs('r', state.robotTr);
  state.toolTr=defaultToolTr();  setInputs('t', state.toolTr);
  if (state.xmls[0]) parseXml(new TextDecoder('utf-8').decode(state.buffers.get(state.xmls[0].path)));
  setJointAnglesToReferencePose();
  ['rRx','rRy','rRz'].forEach(function(id,i){ const el=document.getElementById(id); if(el) el.value=[90,0,-90][i]; });
  await loadStls(); enableSave(); renderAll(); setView('iso');
}


async function loadSourceFolder(files) {
  if (!files || !files.length) return;
  resetData(); state.mode = 'source';
  const buffers = new Map();
  const fileList = [];

  for (const file of files) {
    const data = await file.arrayBuffer();
    const u8 = new Uint8Array(data);
    // webkitRelativePath = "OrdnerName/unterordner/datei.stl" → Pfad ab zweitem Segment
    const relPath = file.webkitRelativePath || file.name;
    const parts = relPath.split('/');
    // Ordnername oben abschneiden → nur Unterstruktur
    const path = parts.length > 1 ? parts.slice(1).join('/') : parts[0];
    const name = parts[parts.length - 1];
    if (!name || /^\./.test(name)) continue;  // versteckte Dateien ignorieren
    buffers.set(path, u8);
    fileList.push({ path, name, size: u8.byteLength, type: typeOf(name) });
  }

  state.files = fileList;
  state.buffers = buffers;
  splitFiles();

  const hasOsd = (state.stls||[]).some(f => /\.osd$/i.test(f.name));
  state.robotTr = hasOsd ? {x:0,y:0,z:0,rx:0,ry:0,rz:0} : defaultRobotTr();
  setInputs('r', state.robotTr);
  state.toolTr = defaultToolTr(); setInputs('t', state.toolTr);

  if (state.xmls[0]) parseXml(new TextDecoder('utf-8').decode(state.buffers.get(state.xmls[0].path)));
  setJointAnglesToReferencePose();

  // Rotation vor loadStls setzen
  ['rRx','rRy','rRz'].forEach(function(id,i){ const el=document.getElementById(id); if(el) el.value=[90,0,-90][i]; });

  await loadStls(); enableSave(); renderAll(); setView('iso');
}

async function loadPackageZip(file) {
  const z = await readZip(file);
  // Check for config.json (new component format)
  if (z.buffers.has('config.json')) {
    const cfg = JSON.parse(new TextDecoder().decode(z.buffers.get('config.json')));
    if (cfg.type && cfg.type !== 'robot') {
      const zip = await JSZip.loadAsync(await file.arrayBuffer());
      await loadComponentFromZip(zip);
      return;
    }
  }
  // Existing robot package
  resetData(); state.mode='package';
  state.files=z.files; state.buffers=z.buffers;
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
  const offset = { x: tcp.x??0, y: tcp.y??0, z: tcp.z??0, rx: tcp.c??tcp.rx??0, ry: tcp.b??tcp.ry??0, rz: tcp.a??tcp.rz??0 };
  if (state.effektoren[state.activeEff]) state.effektoren[state.activeEff].offset = offset;
  else { state.effektoren.push({ stlFile: null, offset }); state.activeEff = 0; }
}

function syncTcpFromActiveEff() {
  const eo = state.effektoren[state.activeEff]?.offset || {};
  state.tcp.auftragen = { ...state.tcp.auftragen,
    x: eo.x||0, y: eo.y||0, z: eo.z||0,
    rx: eo.rx||0, ry: eo.ry||0, rz: eo.rz||0,
    a: eo.rz||0, b: eo.ry||0, c: eo.rx||0
  };
  state.tcp.abtragen = { ...state.tcp.auftragen };
}
function syncTcpFromEffOffset() { syncTcpFromActiveEff(); }

function loadExampleGreifer() {
  const N=4, R=35, eMax=30, h=70, w=20, d=15;
  const teile=[];
  for(let i=0;i<N;i++){
    const angleDeg=i*(360/N);
    const rad=angleDeg*Math.PI/180;
    teile.push({
      name:'Backe '+(i+1), objectType:'box', color:'#2288cc',
      length:d, width:w, height:h,
      moveAngle:angleDeg, eMin:0, eMax:eMax, ePos:0, labelNum:1,
      offset:{ x:Math.cos(rad)*R, y:Math.sin(rad)*R, z:70, rx:0, ry:90, rz:angleDeg }
    });
  }
  state.effektoren.push({
    name:'Greifer 4-Backen', objectType:'box', color:'#607080',
    length:80, width:80, height:30,
    offset:{x:0,y:0,z:15,rx:0,ry:0,rz:0},
    ePos:0, teile
  });
  effektorGroups.push(null);
  rebuildRobotKinematics(); applyTransforms();
  renderEffRow(); renderAll();
  enableSave?.();
}
$('exGreiferBtn')?.addEventListener('click', loadExampleGreifer);

async function loadDemoKr8() {
  const BASE  = '../stl/';
  const FILES = ['podest.stl','a1.stl','a2.stl','a3.stl','a4.stl','a5.stl','a6.stl','tool1_tcp.stl'];
  resetData(); state.mode='source'; state.robotName='KR8';
  const btn = $('newBtn');
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
  setJointAnglesToReferencePose();  // Kanonenstellung statt hardcoded [0,-90,90,0,0,0]
  setInputs('r',state.robotTr); setInputs('t',state.toolTr);
  
}

// ── XML-Parser ─────────────────────────────────────────────────────
function parseMachineStateParameters(xml) {
  const msp = xml.querySelector('MachineStateParameters');
  if (!msp) return;
  for (let i = 1; i <= 6; i++) {
    const node = msp.querySelector('AxisA' + i + 'Pos');
    if (!node) continue;
    const minEl = node.querySelector('Min');
    const maxEl = node.querySelector('Max');
    const initEl = node.querySelector('InitialValue');
    const mn = minEl ? parseFloat(minEl.getAttribute('DefaultValue')) : null;
    const mx = maxEl ? parseFloat(maxEl.getAttribute('DefaultValue')) : null;
    const iv = initEl ? parseFloat(initEl.getAttribute('DefaultValue')) : null;
    const j = state.joints[i - 1];
    if (!j) continue;
    // Min/Max aus XML: Achsgrenzen übernehmen (Skelett-Positionen bleiben ignoriert)
    if (mn !== null && !isNaN(mn)) j.min = mn;
    if (mx !== null && !isNaN(mx)) j.max = mx;
    if (iv !== null && !isNaN(iv)) state.jointAngles[i - 1] = iv;
  }
}

function parseXml(text) {
  const xml = new DOMParser().parseFromString(text,'application/xml');
  state.robotName = state.xmls[0]?.name?.replace(/\.xml$/i,'') || 'Robot';
  const gotAxis = false; // Achsendpunkte aus XML ignorieren — Skelett bleibt wie es ist
  // parseAxisPositions(xml);  ← deaktiviert
  if (/irb\s*4600|4600-40-2_55/i.test(text)) {
    const mins=[-180,-90,-180,-179,-125,-179], maxs=[180,150,75,179,120,179];
    state.joints.forEach((j,i)=>{j.min=mins[i];j.max=maxs[i];if(!gotAxis){j.offset=defOffset(i);state.axisPoints[i]={...state.axisPoints[i],...defOffset(i),source:'KR8 Zielwert'}}j.status=gotAxis?'XML AxisPos':'vorgeschlagen'});
  }
  parseMachineStateParameters(xml);
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
  const ad={};
  for(const p of params) {
    const name=p.getAttribute('DefaultValue')||''; const m=name.match(/^AxisA([1-6])Pos$/i); if(!m)continue;
    const xmlAxis=Number(m[1]); if(xmlAxis<2||xmlAxis>6)continue;
    let tx=0,ty=0,tz=0; let matrix=p.nextElementSibling;
    while(matrix&&matrix.tagName!=='Matrix')matrix=matrix.nextElementSibling;
    if(matrix)[...matrix.querySelectorAll('SCType')].forEach(sc=>{const type=sc.getAttribute('Type')||'';const v=num(sc.getAttribute('DefaultValue'))||0;if(type==='TTranslateX')tx+=v;if(type==='TTranslateY')ty+=v;if(type==='TTranslateZ')tz+=v;});
    ad[xmlAxis]={tx,ty,tz}; found=true;
  }
  if(!found){syncJointsFromAxisPoints();return false;}
  // A2: shoulder position directly from matrix translations
  const a2=ad[2]||{}, a3=ad[3]||{}, a4=ad[4]||{}, a5=ad[5]||{}, a6=ad[6]||{};
  const px2=a2.tx||0, py2=a2.ty||0, pz2=a2.tz||0;
  // A3: extend from A2 along reach (ty=arm length)
  const px3=px2+Math.abs(a3.ty||0), py3=py2+(a3.tx||0), pz3=pz2+(a3.tz||0);
  // A4: small offset from A3
  const px4=px3+Math.abs(a4.tx||0), py4=py3+Math.abs(a4.ty||0)*(a4.ty<0?-1:0), pz4=pz3-(Math.abs(a4.ty||0));
  // A5: forearm extends from A4 (tz=arm length)
  const px5=px4+Math.abs(a5.tz||0)+Math.abs(a5.ty||0), py5=py4, pz5=pz4+(a5.tx||0);
  // A6: wrist
  const px6=px5+Math.abs(a6.tx||0)+Math.abs(a6.ty||0), py6=py5, pz6=pz5;
  state.axisPoints[0]={name:'A1',x:0,y:0,z:0,rx:0,ry:0,rz:0,source:'XML'};
  state.axisPoints[1]={name:'A2',x:px2,y:py2,z:pz2,rx:0,ry:0,rz:0,source:'XML AxisA2Pos'};
  state.axisPoints[2]={name:'A3',x:px3,y:py3,z:pz3,rx:0,ry:0,rz:0,source:'XML AxisA3Pos'};
  state.axisPoints[3]={name:'A4',x:px4,y:py4,z:pz4,rx:0,ry:0,rz:0,source:'XML AxisA4Pos'};
  state.axisPoints[4]={name:'A5',x:px5,y:py5,z:pz5,rx:0,ry:0,rz:0,source:'XML AxisA5Pos'};
  state.axisPoints[5]={name:'A6',x:0,y:0,z:0,rx:0,ry:0,rz:0,source:'Ende / 0'};
  syncJointsFromAxisPoints(); return found;
}

function closeVal(a,b){return Math.abs((num(a)||0)-b)<.001}
function setKnownOffsets(){state.joints.forEach((j,i)=>{j.offset=defOffset(i);j.axis=fixedAxisType(i);});state.axisPoints=state.joints.map((j,i)=>({name:'A'+(i+1),...defOffset(i),rx:0,ry:0,rz:0,source:'KR8 Zielwert'}));}
function normalizeKnownOffsets(){if((state.joints||[]).length<2)return;const o=state.joints.map(j=>j.offset||{});const wS=closeVal(o[0].x,0)&&closeVal(o[0].y,0)&&closeVal(o[0].z,0)&&closeVal(o[1].x,495)&&closeVal(o[1].y,175);const wD=closeVal(o[0].x,495)&&closeVal(o[0].y,175)&&closeVal(o[0].z,0);const ok=closeVal(o[0].x,175)&&closeVal(o[0].y,0)&&closeVal(o[0].z,495);if((wS||wD)&&!ok)setKnownOffsets();}

// ── JSON ───────────────────────────────────────────────────────────
function applyJsonToState(j) {
  if(!j)return;
  state.robotName=j.name||state.robotName;
  if(Array.isArray(j.stlRefAngles)&&j.stlRefAngles.length===6){state.referencePose=j.stlRefAngles.map(v=>Number(v)||0);if($('refPose'))$('refPose').value=state.referencePose.join(',');}
  if(Array.isArray(j.jointAngles)&&j.jointAngles.length===6)state.jointAngles=j.jointAngles.map(v=>Number(v)||0);
  if(Array.isArray(j.joints)){state.joints=j.joints.map((v,i)=>({name:v.name||('A'+(i+1)),axis:fixedAxisType(i),offset:{x:num(v.offset?.x)??null,y:num(v.offset?.y)??null,z:num(v.offset?.z)??null},min:num(v.min),max:num(v.max),rotationSign:num(v.rotationSign??v.rotationDirection??v.dir)??1,status:v.status||'JSON'}));state.axisPoints=state.joints.map((v,i)=>({name:v.name||('A'+(i+1)),x:num(v.offset?.x),y:num(v.offset?.y),z:num(v.offset?.z),rx:0,ry:0,rz:0,source:'JSON'}));}
  // Endeffektoren: neues Format (Array) + backward compat (Singular)
  if (Array.isArray(j.endeffektoren) && j.endeffektoren.length > 0) {
    state.effektoren = j.endeffektoren.map(e => ({
      stlFile: null,
      offset: { x:e.px||0, y:e.py||0, z:e.pz||0, rx:e.rx||0, ry:e.ry||0, rz:e.rz||0 },
      stlName: e.stl || null, name: e.name || null, typ: e.typ || 'auftragend'
    }));
    state.activeEff = 0;
  } else if (j.endeffektor) {
    const e = j.endeffektor;
    state.effektoren = [{ stlFile: null, offset: { x:e.px||0, y:e.py||0, z:e.pz||0, rx:e.rx||0, ry:e.ry||0, rz:e.rz||0 }, stlName: e.stl||null, name: e.name||null, typ: e.typ||'auftragend' }];
    state.activeEff = 0;
  }
  if (Array.isArray(j.umfeld) && j.umfeld.length) {
    state.umfElemente = j.umfeld.map(e => ({
      stlFile: null,
      offset: { x:e.px||0, y:e.py||0, z:e.pz||0, rx:e.rx||0, ry:e.ry||0, rz:e.rz||0 },
      stlName: e.stl||null, name: e.name||null
    }));
    state.activeUmf = 0;
  }
  if(j.stlRotation){
    const r=j.stlRotation;
    const set=(id,v)=>{const el=$(id);if(el)el.value=v;};
    set('rRx',r.rx||0); set('rRy',r.ry||0); set('rRz',r.rz||0);
  }
  if(j.tcp){state.tcp.auftragen=cleanTcpOrientation({...(j.tcp.auftragen||j.tcp),toolLength:j.tcp.auftragen?.toolLength??0,status:'JSON'});state.tcp.abtragen=cleanTcpOrientation({...(j.tcp.abtragen||j.tcp.auftragen||j.tcp),toolLength:j.tcp.abtragen?.toolLength??0,status:'JSON'});setEffOffsetFromTcp(state.tcp.auftragen);}
  const toolName=j.sceneModels?.tool?.name||j.tcp?.auftragen?.toolStl||j.tcp?.auftragen?.stlName;
  if(toolName)state.toolName=String(toolName).endsWith('.stl')?toolName:toolName+'.stl';
  normalizeKnownOffsets();
}

function buildJson() {
  const stlRx = parseFloat($('rRx')?.value||0)||0;
  const stlRy = parseFloat($('rRy')?.value||0)||0;
  const stlRz = parseFloat($('rRz')?.value||0)||0;
  const axNames = ['A1','A2','A3','A4','A5','A6'];
  const stlFiles = Object.fromEntries(axNames.map((ax, i) => {
    const parts = state.axisStlParts[ax]||[];
    if (parts.length>0) return [ax, parts.map(p=>({name:norm(p.name),color:p.color||'#e8a020'}))];
    const s = state.axisStlMap[ax]||state.stls.find(f=>partKey(f.name)===ax)?.name||'';
    return [ax, [{name:norm(s)||('a'+(i+1)),color:colors[ax]||'#e8a020'}]];
  }));
  const tcp = state.tcp.auftragen;
  const toolName = norm(state.toolName || tcp.toolStl || '') || 'tool1_tcp';
  const eo = state.effektoren[state.activeEff]?.offset || {};
  const tcpX = eo.x??num(tcp.x)??0, tcpY = eo.y??num(tcp.y)??0, tcpZ = eo.z??num(tcp.z)??0;
  const tcpA = eo.rz??num(tcp.rz)??0, tcpB = eo.ry??num(tcp.ry)??0, tcpC = eo.rx??num(tcp.rx)??0;
  const result = {
    name: state.robotName || 'Robot',
    stlRotation: {rx:stlRx, ry:stlRy, rz:stlRz},
    joints: state.joints.map((j,i) => ({
      name: j.name,
      axis: fixedAxisType(i),
      offset: { x: num(j.offset?.x)??0, y: num(j.offset?.y)??0, z: num(j.offset?.z)??0 },
      min: num(j.min) ?? -180,
      max: num(j.max) ??  180
    })),
    stlRefAngles: parseReferencePose(),
    tcp: { x: tcpX??0, y: tcpY??0, z: tcpZ??0, a: tcpA??0, b: tcpB??0, c: tcpC??0 },
    stlFiles,
    sceneModels: {
      pedestal: { px:0, py:0, pz:0, rx:0, ry:0, rz:0, name: (state.sceneModels?.pedestal?.name || 'podest').replace(/\.(stl|osd)$/i,'') },
      tool:     { px:0, py:0, pz:0, rx:0, ry:0, rz:0, name: toolName }
    }
  };
  if (state.effektoren.length > 0) {
    result.endeffektoren = state.effektoren.map((eff, i) => {
      const eo = eff.offset || {};
      return { name: norm(eff.stlFile?.name || ('Effektor '+(i+1))), stl: 'endeffektor_'+(i+1)+'.stl', typ: eff.typ||'auftragend', px: eo.x||0, py: eo.y||0, pz: eo.z||0, rx: eo.rx||0, ry: eo.ry||0, rz: eo.rz||0 };
    });
  }
  const allUmf = [...(state.umfElemente||[]), ...(state.umfStls||[]).map(u=>({stlFile:u,offset:{x:0,y:0,z:0,rx:0,ry:0,rz:0}}))];
  if (allUmf.length) result.umfeld = allUmf.map((u,i) => {
    const o = u.offset||{}; return { name: norm(u.stlFile?.name||u.name||('Umfeld '+(i+1))), stl: 'umfeld_'+(i+1)+'.stl', px:o.x||0, py:o.y||0, pz:o.z||0, rx:o.rx||0, ry:o.ry||0, rz:o.rz||0 };
  });
  if ((state.schienen||[]).length) result.schienen = state.schienen.map(r=>{
    const eAx = 'E'+(r.eNumber||1);
    const parts = (state.axisStlParts[eAx]||[]).map(p=>({name:norm(p.name),color:p.color||'#2563eb'}));
    return { name:r.name||'Rail', length_mm:r.length_mm||2000, height_mm:r.height_mm||200, width_mm:r.width_mm||400, axis:r.axis||'X+', eNumber:r.eNumber||1, eMin:r.eMin??0, eMax:r.eMax??r.length_mm??2000, stlFiles: parts.length ? {[eAx]: parts} : undefined };
  });
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
    // Winkelfeld in Tabelle live aktualisieren
    var _inp=document.querySelector('[data-joint-angle="'+axisIndex+'"]');
    if(_inp) _inp.value=value.toFixed(1);
    if(t<1&&state.simulation.active&&state.simulation.axis===axisIndex){state.simulation.raf=requestAnimationFrame(step);}
    else{state.jointAngles=base.slice();state.jointAngles[axisIndex]=startValue;applyJointRotations();state.simulation={active:false,axis:null,raf:null};}
  };
  state.simulation.raf=requestAnimationFrame(step);
}

// ── Render-Funktionen ──────────────────────────────────────────────
function renderAll(){renderAxisStlRows();renderRows();updateAxisPointVisuals();renderTcp();const b=$('fileBadge');b.textContent=state.files.length?`${state.stls.length} STL · ${state.xmls.length} XML · ${state.jsons.length} JSON`:state.mode==='package'?'Package geladen':'Keine Datei geladen';}


function setParamTab(tab) {
  _paramTab = tab;
  const on='rgba(37,99,235,.3)', onB='1px solid rgba(37,99,235,.6)', off='rgba(255,255,255,.05)', offB='1px solid rgba(255,255,255,.15)';
  [['a','tabAachsen'],['e','tabEachsen'],['l','tabLabels']].forEach(([t,id])=>{
    const btn=$(id); if(!btn) return;
    btn.style.background=tab===t?on:off; btn.style.border=tab===t?onB:offB; btn.style.color=tab===t?'#60a5fa':'#6a8fa8';
  });
  renderRows();
}
window.setParamTab = setParamTab;

function renderRows(){
  const ph=$('colPosHeader'), dh=$('colDirHeader');
  if(_paramTab==='l'){if(ph)ph.textContent='mm';if(dh)dh.textContent='Richtung';}
  else{if(ph)ph.textContent='Winkel °';if(dh)dh.textContent='Dreht';}
  const rail = (state.schienen||[])[0];

  if (_paramTab === 'e') {
    const positioners = state.positioners||[];
    if(!rail && !positioners.length){
      $('jointRows').innerHTML=`<tr><td colspan="12" style="color:#4a6a8a;font-family:monospace;font-size:11px;padding:8px">Keine externen Achsen definiert.</td></tr>`;
      return;
    }
    // Build full E-tab HTML once to avoid innerHTML+= event-listener loss
    let eTabHtml='';
    if(rail){
      const eAx='E'+(rail.eNumber||1);
      const parts=state.axisStlParts[eAx]||[];
      const col=(parts[0]?.color)||'#2563eb';
      eTabHtml=`<tr><td><b>${eAx}</b></td><td><input data-e-pos type="number" step="1" value="${rail.ePos||0}" min="${rail.eMin??0}" max="${rail.eMax??rail.length_mm??2000}" style="width:70px"></td><td><span class="axisDir">${rail.axis||'X+'}</span></td><td colspan="3" style="color:#4a6a8a;font-size:10px;text-align:center">—</td><td><input data-e-min type="number" step="1" value="${rail.eMin??0}" style="width:60px"></td><td><input data-e-max type="number" step="1" value="${rail.eMax??rail.length_mm??2000}" style="width:60px"></td><td style="color:#4a6a8a">—</td><td><label style="display:inline-block;width:26px;height:22px;border-radius:3px;background:${col};border:1px solid rgba(255,255,255,.25);cursor:pointer;overflow:hidden"><input type="color" data-axis-color="${eAx}" value="${col}" style="opacity:0;width:1px;height:1px;position:absolute"></label></td><td><button class="axis-stl-btn" data-ax="${eAx}" style="font-size:10px;padding:3px 7px;background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.15);border-radius:3px;cursor:pointer;color:${parts.length?'#d8e8f0':'#6a8fa8'};width:100%">${parts.length?parts.length+' Part'+(parts.length>1?'s':''):'+ STL'}</button></td><td><button data-e-sim style="background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.15);border-radius:3px;padding:2px 7px;cursor:pointer;color:#9ab">▶</button></td></tr>`;
    }
    const posRows=(state.positioners||[]).map((p,i)=>{
      const eAx='E'+(p.eNum||i+2);
      const parts=state.axisStlParts[eAx]||[];
      const col=(parts[0]?.color)||p.color||'#e8a020';
      return `<tr style="border-top:1px solid rgba(255,255,255,.08)"><td><b>${eAx}</b> <span style="font-size:9px;color:#6a8fa8">${p.name||''}</span></td><td><input data-pos-angle type="number" step="1" value="${p.ePos||0}" min="${p.eMin??-180}" max="${p.eMax??180}" data-pi="${i}" style="width:70px"></td><td><span class="axisDir">R${p.rotAxis||'Y+'}</span></td><td colspan="3" style="color:#4a6a8a;font-size:10px;text-align:center">—</td><td><input data-pos-min type="number" step="1" value="${p.eMin??-180}" data-pi="${i}" style="width:60px"></td><td><input data-pos-max type="number" step="1" value="${p.eMax??180}" data-pi="${i}" style="width:60px"></td><td style="color:#4a6a8a">—</td><td><label style="display:inline-block;width:26px;height:22px;border-radius:3px;background:${col};border:1px solid rgba(255,255,255,.25);cursor:pointer;overflow:hidden"><input type="color" value="${col}" data-pos-color data-pi="${i}" style="opacity:0;width:1px;height:1px;position:absolute"></label></td><td><button data-pos-stl data-pi="${i}" style="font-size:10px;padding:3px 7px;background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.15);border-radius:3px;cursor:pointer;color:${parts.length?'#d8e8f0':'#6a8fa8'};width:100%">${parts.length?parts.length+' Part'+(parts.length>1?'s':''):'+ STL'}</button></td><td><button data-pos-sim data-pi="${i}" style="background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.15);border-radius:3px;padding:2px 7px;cursor:pointer;color:#9ab">▶</button></td></tr>`;
    }).join('');
    $('jointRows').innerHTML = eTabHtml + posRows;
    if(rail){
      $('jointRows').querySelector('[data-e-pos]')?.addEventListener('input',e=>{if(rail){rail.ePos=parseFloat(e.target.value)||0;rebuildRailMeshes();}});
      $('jointRows').querySelector('[data-e-min]')?.addEventListener('input',e=>{if(rail)rail.eMin=parseFloat(e.target.value)||0;});
      $('jointRows').querySelector('[data-e-max]')?.addEventListener('input',e=>{if(rail)rail.eMax=parseFloat(e.target.value)||0;});
      $('jointRows').querySelector('[data-e-sim]')?.addEventListener('click',()=>{
        if(!rail) return;
        if(rail._simInterval){ clearInterval(rail._simInterval); delete rail._simInterval; return; }
        const min=rail.eMin??0, max=rail.eMax??rail.length_mm??2000;
        let pos=rail.ePos||0, dir=1;
        rail._simInterval=setInterval(()=>{
          pos+=dir*(max-min)/60;
          if(pos>=max){pos=max;dir=-1;} else if(pos<=min){pos=min;dir=1;}
          rail.ePos=pos;
          const inp=$('jointRows').querySelector('[data-e-pos]');
          if(inp) inp.value=Math.round(pos);
          rebuildRailMeshes();
        },16);
      });
    }
    $('jointRows').querySelectorAll('[data-pos-angle]').forEach(inp=>inp.addEventListener('input',()=>{
      const i=+inp.dataset.pi; if(!state.positioners[i])return;
      state.positioners[i].ePos=parseFloat(inp.value)||0; rebuildPositionerMesh(i);
    }));
    $('jointRows').querySelectorAll('[data-pos-min]').forEach(inp=>inp.addEventListener('input',()=>{ const i=+inp.dataset.pi; if(state.positioners[i]) state.positioners[i].eMin=parseFloat(inp.value)||0; }));
    $('jointRows').querySelectorAll('[data-pos-max]').forEach(inp=>inp.addEventListener('input',()=>{ const i=+inp.dataset.pi; if(state.positioners[i]) state.positioners[i].eMax=parseFloat(inp.value)||0; }));
    $('jointRows').querySelectorAll('[data-pos-color]').forEach(inp=>inp.addEventListener('change',()=>{ const i=+inp.dataset.pi; if(!state.positioners[i])return; state.positioners[i].color=inp.value; rebuildPositionerMesh(i); renderRows(); }));
    $('jointRows').querySelectorAll('[data-pos-stl]').forEach(btn=>btn.addEventListener('click',()=>{ const i=+btn.dataset.pi; openAxisPartsModal('E'+(state.positioners[i]?.eNum||i+2)); }));
    $('jointRows').querySelectorAll('[data-pos-sim]').forEach(btn=>btn.addEventListener('click',()=>{
      const i=+btn.dataset.pi; const p=state.positioners[i]; if(!p)return;
      if(p._simInterval){clearInterval(p._simInterval);delete p._simInterval;return;}
      const min=p.eMin??-180, max=p.eMax??180;
      let ang=p.ePos||0, dir=1;
      p._simInterval=setInterval(()=>{
        ang+=dir*(max-min)/120;
        if(ang>=max){ang=max;dir=-1;} else if(ang<=min){ang=min;dir=1;}
        p.ePos=ang;
        const inp=$('jointRows').querySelector(`[data-pos-angle][data-pi="${i}"]`);
        if(inp) inp.value=Math.round(ang);
        rebuildPositionerMesh(i);
      },16);
    }));
    return;
  }

  if (_paramTab === 'l') {
    const objekte = state.objekte||[];
    // Collect all labeled members: objekte + endeffektor teile
    const groups=new Map();
    objekte.forEach((o,i)=>{
      const k=o.labelNum||i+1;
      if(!groups.has(k))groups.set(k,{objekte:[],teile:[]});
      groups.get(k).objekte.push({o,i});
    });
    (state.effektoren||[]).forEach((eff,ei)=>{
      (eff.teile||[]).forEach((t,ti)=>{
        if(t.labelNum==null) return;
        const k=t.labelNum;
        if(!groups.has(k))groups.set(k,{objekte:[],teile:[]});
        groups.get(k).teile.push({t,ei,ti,eff});
      });
    });
    if(!groups.size){$('jointRows').innerHTML=`<tr><td colspan="12" style="color:#4a6a8a;font-family:monospace;font-size:11px;padding:8px">Keine Labels — über 📦 Bewegliche Objekte oder Endeffektor-Teile anlegen.</td></tr>`;return;}
    let html='';
    groups.forEach((members,key)=>{
      const firstObj=members.objekte[0]?.o;
      const firstTeil=members.teile[0]?.t;
      const first=firstObj||firstTeil;
      const mountIcon=(members.objekte.some(m=>m.o.mountMode==='a6')||members.teile.length)?'🦾':'🌍';
      const axisStr=[...new Set([...members.objekte.map(m=>m.o.axis||'Y+'),...members.teile.map(m=>m.t.axis||'Y+')])].join('/');
      const nameStr=[...members.objekte.map(m=>m.o.name||''),...members.teile.map(m=>m.t.name||'')].filter(Boolean).join(', ');
      const parts=state.axisStlParts['Label'+key]||[];
      const col=(parts[0]?.color)||first.color||'#4499cc';
      const curPos=first.ePos||0;
      const minVal=first.eMin||0;
      const maxVal=first.eMax??1000;
      html+=`<tr>
        <td><b>Label${key}</b> <span style="font-size:9px;color:#6a8fa8">${mountIcon}</span></td>
        <td><input data-gl-pos data-gl-key="${key}" type="number" step="1" value="${curPos}" min="${minVal}" max="${maxVal}" style="width:70px"></td>
        <td><span class="axisDir">${axisStr}</span></td>
        <td colspan="3" style="color:#6a8fa8;font-size:10px">${nameStr}</td>
        <td><input data-gl-min data-gl-key="${key}" type="number" step="1" value="${minVal}" style="width:60px"></td>
        <td><input data-gl-max data-gl-key="${key}" type="number" step="1" value="${maxVal}" style="width:60px"></td>
        <td style="color:#4a6a8a">—</td>
        <td><label style="display:inline-block;width:26px;height:22px;border-radius:3px;background:${col};border:1px solid rgba(255,255,255,.25);cursor:pointer;overflow:hidden"><input type="color" data-gl-color data-gl-key="${key}" value="${col}" style="opacity:0;width:1px;height:1px;position:absolute"></label></td>
        <td><button data-gl-stl data-gl-key="${key}" style="font-size:10px;padding:3px 7px;background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.15);border-radius:3px;cursor:pointer;color:${parts.length?'#d8e8f0':'#6a8fa8'};width:100%">${parts.length?parts.length+' Part'+(parts.length>1?'s':''):'+ STL'}</button></td>
        <td><button data-gl-sim data-gl-key="${key}" style="background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.15);border-radius:3px;padding:2px 7px;cursor:pointer;color:#9ab">▶</button></td>
      </tr>`;
    });
    $('jointRows').innerHTML=html;

    // Helper: update all members for a key
    function applyLabelPos(key,val) {
      const m=groups.get(key); if(!m) return;
      m.objekte.forEach(({o,i})=>{o.ePos=val;rebuildObjektMesh(i);});
      m.teile.forEach(({t,ei})=>{t.ePos=val;rebuildEffMesh(ei);});
      applyTransforms();
    }

    $('jointRows').querySelectorAll('[data-gl-pos]').forEach(inp=>inp.addEventListener('input',()=>{
      applyLabelPos(+inp.dataset.glKey, parseFloat(inp.value)||0);
    }));
    $('jointRows').querySelectorAll('[data-gl-min]').forEach(inp=>inp.addEventListener('input',()=>{
      const key=+inp.dataset.glKey,val=parseFloat(inp.value)||0;
      const m=groups.get(key); if(!m) return;
      m.objekte.forEach(({o})=>o.eMin=val); m.teile.forEach(({t})=>t.eMin=val);
    }));
    $('jointRows').querySelectorAll('[data-gl-max]').forEach(inp=>inp.addEventListener('input',()=>{
      const key=+inp.dataset.glKey,val=parseFloat(inp.value)||1000;
      const m=groups.get(key); if(!m) return;
      m.objekte.forEach(({o})=>o.eMax=val); m.teile.forEach(({t})=>t.eMax=val);
    }));
    $('jointRows').querySelectorAll('[data-gl-color]').forEach(inp=>inp.addEventListener('change',()=>{
      const key=+inp.dataset.glKey;
      const m=groups.get(key); if(!m) return;
      m.objekte.forEach(({o,i})=>{o.color=inp.value;rebuildObjektMesh(i);});
      m.teile.forEach(({t,ei})=>{t.color=inp.value;rebuildEffMesh(ei);});
      renderRows();
    }));
    $('jointRows').querySelectorAll('[data-gl-stl]').forEach(btn=>btn.addEventListener('click',()=>{
      openAxisPartsModal('Label'+btn.dataset.glKey);
    }));
    $('jointRows').querySelectorAll('[data-gl-sim]').forEach(btn=>btn.addEventListener('click',()=>{
      const key=+btn.dataset.glKey;
      const m=groups.get(key); if(!m) return;
      const first=(m.objekte[0]?.o)||(m.teile[0]?.t); if(!first) return;
      const simKey='_simGroup_'+key;
      if(first[simKey]){clearInterval(first[simKey]);[...m.objekte.map(x=>x.o),...m.teile.map(x=>x.t)].forEach(o=>delete o[simKey]);return;}
      const min=first.eMin||0, max=first.eMax||1000;
      let pos=first.ePos||0, dir=1;
      const iv=setInterval(()=>{
        pos+=dir*(max-min)/60;
        if(pos>=max){pos=max;dir=-1;}else if(pos<=min){pos=min;dir=1;}
        applyLabelPos(key,pos);
        const inp=$('jointRows').querySelector(`[data-gl-pos][data-gl-key="${key}"]`);
        if(inp)inp.value=Math.round(pos);
      },16);
      [...m.objekte.map(x=>x.o),...m.teile.map(x=>x.t)].forEach(o=>o[simKey]=iv);
    }));
    return;
  }

  $('jointRows').innerHTML = state.joints.map((j,i)=>{
    const ax=j.name||'A'+(i+1);
    const parts=state.axisStlParts[ax]||[];
    const col=(parts[0]?.color)||colors[ax]||'#999999';
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
      <td><label style="display:inline-block;width:26px;height:22px;border-radius:3px;background:${col};border:1px solid rgba(255,255,255,.25);cursor:pointer;overflow:hidden" title="Farbe ${ax}"><input type="color" data-axis-color="${ax}" value="${col}" style="opacity:0;width:1px;height:1px;position:absolute"></label></td>
      <td><button class="axis-stl-btn" data-ax="${ax}" style="font-size:10px;padding:3px 7px;background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.15);border-radius:3px;cursor:pointer;color:${parts.length?'#d8e8f0':'#6a8fa8'};width:100%">${parts.length?parts.length+' Part'+(parts.length>1?'s':''):'+ STL'}</button></td>
      <td><button class="simBtn" data-sim-axis="${i}">▶</button></td>
    </tr>`;}).join('');
}

// ── Bewegliche Objekte ────────────────────────────────────────────
var _omAxis = 'Y+';

function renderObjRows() {
  const el=$('objRows'); if(!el) return;
  const badge=$('objBadge');
  const objekte=state.objekte||[];
  if(badge) badge.textContent=objekte.length||'0';
  if(!objekte.length){el.innerHTML='';return;}
  const fs='background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.1);border-radius:3px;padding:2px 5px;font-family:inherit;font-size:12px;color:#d8e8f0;outline:none;width:100%';
  el.innerHTML=objekte.map((o,i)=>`
    <div style="border:1px solid rgba(255,255,255,.1);border-radius:4px;padding:8px;margin-bottom:6px">
      <div style="display:flex;align-items:center;gap:6px;margin-bottom:4px">
        <span style="flex:1;font-family:monospace;font-size:12px;color:var(--txt)">Label${o.labelNum||i+1} — ${o.type==='cylinder'?'🔵':'📦'} ${o.name||''}</span>
        <button data-obj-edit="${i}" style="background:rgba(37,99,235,.15);border:1px solid rgba(37,99,235,.4);color:#60a5fa;border-radius:3px;padding:1px 7px;cursor:pointer;font-size:11px">✏️</button>
        <button data-obj-del="${i}" style="background:rgba(204,51,51,.15);border:1px solid rgba(204,51,51,.3);color:#f87171;border-radius:3px;padding:1px 6px;cursor:pointer;font-size:12px">✕</button>
      </div>
      <div style="font-size:10px;color:#6a8fa8;font-family:monospace">${o.axis||'Y+'} | ${o.eMin||0}–${o.eMax||1000} mm</div>
    </div>`).join('');
  el.querySelectorAll('[data-obj-del]').forEach(btn=>btn.addEventListener('click',()=>{
    const i=+btn.dataset.objDel;
    if(state.objekte[i]?._simInterval) clearInterval(state.objekte[i]._simInterval);
    if(objekteGroups[i]){ scene.remove(objekteGroups[i]); objekteGroups[i]=null; }
    state.objekte.splice(i,1); objekteGroups.splice(i,1);
    renderObjRows(); renderRows();
  }));
  el.querySelectorAll('[data-obj-edit]').forEach(btn=>btn.addEventListener('click',()=>{
    openObjModal(+btn.dataset.objEdit);
  }));
}

function rebuildUmfMesh(i) {
  const u=state.umfElemente[i]; if(!u) return;
  if(!umfGroups[i]) umfGroups[i]=new THREE.Group();
  const grp=umfGroups[i];
  if(grp.parent!==scene){ if(grp.parent) grp.parent.remove(grp); scene.add(grp); }
  while(grp.children.length) grp.remove(grp.children[0]);
  const o=u.offset||{}, deg2=Math.PI/180;
  grp.position.set(o.x||0,o.y||0,o.z||0);
  grp.rotation.set((o.rx||0)*deg2,(o.ry||0)*deg2,(o.rz||0)*deg2,'XYZ');
  if(u.stlFile?.buf){
    const geo=loader.parse(u.stlFile.buf.buffer||u.stlFile.buf); geo.computeVertexNormals();
    grp.add(new THREE.Mesh(geo,new THREE.MeshPhongMaterial({color:u.color||0x3a5a7a,shininess:60})));
  } else if(u.objectType&&u.objectType!=='stl'){
    const L=u.length||2000,W=u.width||200,H=u.height||2000,R=u.radius||500;
    const geo=u.objectType==='cylinder'?new THREE.CylinderGeometry(R,R,H,32):new THREE.BoxGeometry(L,H,W);
    grp.add(new THREE.Mesh(geo,new THREE.MeshPhongMaterial({color:u.color||0x3a5a7a,shininess:60,transparent:true,opacity:0.85,side:THREE.DoubleSide})));
  }
}

function rebuildObjektMesh(i) {
  const o=state.objekte[i]; if(!o) return;
  if(!objekteGroups[i]) objekteGroups[i]=new THREE.Group();
  const grp=objekteGroups[i];
  // Correct parent: a6 → axisPivotGroups[5], else scene
  const targetParent=(o.mountMode==='a6'&&axisPivotGroups[5])?axisPivotGroups[5]:scene;
  if(grp.parent!==targetParent){ if(grp.parent) grp.parent.remove(grp); targetParent.add(grp); }
  while(grp.children.length) grp.remove(grp.children[0]);
  const lbl='Label'+(o.labelNum||i+1);
  const parts=state.axisStlParts[lbl]||[];
  if(parts.length){
    parts.forEach(p=>{ if(!p.buf)return; const geo=loader.parse(p.buf.buffer||p.buf); geo.computeVertexNormals(); grp.add(new THREE.Mesh(geo,new THREE.MeshPhongMaterial({color:p.color||0x4499cc,shininess:60}))); });
  } else if(o.showBox!==false){
    const L=o.length||500,H=o.height||500,W=o.width||500,R=o.radius||200;
    const geo=o.type==='cylinder'?new THREE.CylinderGeometry(R,R,H,32):new THREE.BoxGeometry(L,H,W);
    grp.add(new THREE.Mesh(geo,new THREE.MeshPhongMaterial({color:o.color||0x4499cc,shininess:60,transparent:true,opacity:0.85,side:THREE.DoubleSide})));
  }
  // Position: base offset + movement along axis
  const p=o.ePos||0, ax=o.axis||'Y+', bo=o.boxOffset||{}, deg=Math.PI/180;
  var cx=0,cy=0,cz=0;
  if(o.moveAngle!=null){const rad=o.moveAngle*deg;cx=Math.cos(rad)*p;cy=Math.sin(rad)*p;}
  else if(ax==='X+')cx=p; else if(ax==='X-')cx=-p; else if(ax==='Y+')cy=p; else if(ax==='Y-')cy=-p; else if(ax==='Z+')cz=p; else cz=-p;
  grp.position.set((bo.x||0)+cx,(bo.y||0)+cy,(bo.z||0)+cz);
  grp.rotation.set((bo.rx||0)*deg,(bo.ry||0)*deg,(bo.rz||0)*deg,'XYZ');
}

function rebuildEffMesh(effIdx) {
  const eff=state.effektoren[effIdx]; if(!eff) return;
  if(!effektorGroups[effIdx]) effektorGroups[effIdx]=new THREE.Group();
  const grp=effektorGroups[effIdx];
  const targetParent=(axisPivotGroups[5])||scene;
  if(grp.parent!==targetParent){if(grp.parent)grp.parent.remove(grp);targetParent.add(grp);}
  while(grp.children.length)grp.remove(grp.children[0]);
  const o=eff.offset||{}, deg2=Math.PI/180;
  grp.position.set(o.x||0,o.y||0,o.z||0);
  grp.rotation.set((o.rx||0)*deg2,(o.ry||0)*deg2,(o.rz||0)*deg2,'XYZ');
  const teile=eff.teile||[];
  const ePos=eff.ePos||0;
  if(teile.length){
    teile.forEach(t=>{
      const tGrp=new THREE.Group(); grp.add(tGrp);
      if(t.stlFile?.buf){
        const geo=loader.parse(t.stlFile.buf.buffer||t.stlFile.buf); geo.computeVertexNormals();
        tGrp.add(new THREE.Mesh(geo,new THREE.MeshPhongMaterial({color:t.color||0x607080,shininess:60})));
      } else {
        const L=t.length||100,W=t.width||50,H=t.height||50,R=t.radius||25;
        const geo=t.objectType==='cylinder'?new THREE.CylinderGeometry(R,R,H,32):new THREE.BoxGeometry(L,H,W);
        tGrp.add(new THREE.Mesh(geo,new THREE.MeshPhongMaterial({color:t.color||0x607080,shininess:60,transparent:true,opacity:.85})));
      }
      const bo=t.offset||{}, ax=t.axis||'Y+';
      const eMax=t.eMax??t.ePos??0;
      const p=Math.min(Math.max(t.ePos||0,t.eMin||0),eMax);
      let cx=0,cy=0,cz=0;
      if(t.moveAngle!=null){
        const rad=t.moveAngle*Math.PI/180;
        cx=Math.cos(rad)*p; cy=Math.sin(rad)*p;
      } else {
        if(ax==='X+')cx=p;else if(ax==='X-')cx=-p;else if(ax==='Y+')cy=p;else if(ax==='Y-')cy=-p;else if(ax==='Z+')cz=p;else cz=-p;
      }
      tGrp.position.set((bo.x||0)+cx,(bo.y||0)+cy,(bo.z||0)+cz);
      tGrp.rotation.set((bo.rx||0)*deg2,(bo.ry||0)*deg2,(bo.rz||0)*deg2,'XYZ');
    });
  } else {
    // Backward compat: single geometry on eff itself
    if(eff.stlFile?.buf){
      const geo=loader.parse(eff.stlFile.buf.buffer||eff.stlFile.buf); geo.computeVertexNormals();
      grp.add(new THREE.Mesh(geo,new THREE.MeshPhongMaterial({color:eff.color||0x607080,shininess:60})));
    } else if(eff.objectType&&eff.objectType!=='stl'){
      const L=eff.length||200,W=eff.width||200,H=eff.height||300,R=eff.radius||80;
      const geo=eff.objectType==='cylinder'?new THREE.CylinderGeometry(R,R,H,32):new THREE.BoxGeometry(L,H,W);
      grp.add(new THREE.Mesh(geo,new THREE.MeshPhongMaterial({color:eff.color||0x607080,shininess:60,transparent:true,opacity:.85})));
    }
  }
}

function openObjModal(editIdx) {
  const o = editIdx>=0 ? (state.objekte||[])[editIdx] : null;
  $('om-name').value   = o?.name    || '';
  $('om-type').value   = o?.type    || 'box';
  $('om-num').value    = o?.labelNum||(state.objekte||[]).length+1;
  if($('om-mount')) $('om-mount').value = o?.mountMode||'world';
  $('om-length').value = o?.length  || 500;
  $('om-width').value  = o?.width   || 500;
  $('om-height').value = o?.height  || 500;
  $('om-radius').value = o?.radius  || 200;
  $('om-min').value    = o?.eMin    ?? 0;
  $('om-max').value    = o?.eMax    ?? 1000;
  $('om-start').value  = o?.ePos    ?? 0;
  if($('om-moveangle')) $('om-moveangle').value = o?.moveAngle??'';
  $('om-show').checked = o?.showBox !== false;
  $('om-edit-idx').value = editIdx >= 0 ? editIdx : -1;
  const bo=o?.boxOffset||{};
  $('om-ox').value=bo.x||0;$('om-oy').value=bo.y||0;$('om-oz').value=bo.z||0;
  $('om-orx').value=bo.rx||0;$('om-ory').value=bo.ry||0;$('om-orz').value=bo.rz||0;
  _omAxis=o?.axis||'Y+';
  document.querySelectorAll('.om-axis-btn').forEach(b=>{
    const on=b.dataset.ax===_omAxis;
    b.style.background=on?'rgba(37,99,235,.3)':'rgba(255,255,255,.05)';
    b.style.border=on?'1px solid rgba(37,99,235,.6)':'1px solid rgba(255,255,255,.15)';
    b.style.color=on?'#60a5fa':'#6a8fa8';
  });
  omTypeChanged();
  $('objModal').style.display='flex';
}

function omTypeChanged(){
  const t=$('om-type')?.value||'box';
  const bf=$('om-box-fields'), cf=$('om-cyl-fields');
  if(bf) bf.style.display=t==='box'?'contents':'none';
  if(cf) cf.style.display=t==='cylinder'?'contents':'none';
}
window.omTypeChanged=omTypeChanged;

document.querySelectorAll('.om-axis-btn').forEach(b=>b.addEventListener('click',()=>{
  _omAxis=b.dataset.ax;
  document.querySelectorAll('.om-axis-btn').forEach(x=>{
    const on=x.dataset.ax===_omAxis;
    x.style.background=on?'rgba(37,99,235,.3)':'rgba(255,255,255,.05)';
    x.style.border=on?'1px solid rgba(37,99,235,.6)':'1px solid rgba(255,255,255,.15)';
    x.style.color=on?'#60a5fa':'#6a8fa8';
  });
}));

$('objAddBtn')?.addEventListener('click',()=>openObjModal(-1));
$('objModalClose')?.addEventListener('click',()=>{ $('objModal').style.display='none'; });

$('om-submit')?.addEventListener('click',()=>{
  const editIdx=parseInt($('om-edit-idx').value);
  const entry={
    name:      $('om-name').value||('Objekt '+(state.objekte.length+1)),
    type:      $('om-type').value||'box',
    labelNum:  parseInt($('om-num').value)||1,
    mountMode: $('om-mount')?.value||'world',
    length:    parseFloat($('om-length').value)||500,
    width:     parseFloat($('om-width').value)||500,
    height:    parseFloat($('om-height').value)||500,
    radius:    parseFloat($('om-radius').value)||200,
    axis:      _omAxis,
    eMin:      parseFloat($('om-min').value)||0,
    eMax:      parseFloat($('om-max').value)||1000,
    ePos:      parseFloat($('om-start').value)||0,
    moveAngle: $('om-moveangle')?.value!=='' ? parseFloat($('om-moveangle').value) : null,
    showBox:   $('om-show').checked===true,
    color:     '#4499cc',
    boxOffset: {
      x: parseFloat($('om-ox').value)||0, y: parseFloat($('om-oy').value)||0, z: parseFloat($('om-oz').value)||0,
      rx:parseFloat($('om-orx').value)||0,ry:parseFloat($('om-ory').value)||0,rz:parseFloat($('om-orz').value)||0
    }
  };
  if(editIdx>=0){ state.objekte[editIdx]=entry; }
  else { state.objekte.push(entry); objekteGroups.push(null); }
  const idx=editIdx>=0?editIdx:state.objekte.length-1;
  // STL → axisStlParts
  if(_omStlBuf){ const lbl='Label'+(entry.labelNum||1); state.axisStlParts[lbl]=[{name:entry.name+'.stl',color:entry.color||'#4499cc',buf:_omStlBuf}]; _omStlBuf=null; }
  rebuildObjektMesh(idx);
  renderObjRows(); renderRows();
  $('objModal').style.display='none';
});

// ── Positionierer ─────────────────────────────────────────────────
function renderPosRows() {
  const el=$('posRows'); if(!el) return;
  const badge=$('posBadge');
  const positioners=state.positioners||[];
  if(badge) badge.textContent=positioners.length||'0';
  if(!positioners.length){el.innerHTML='';return;}
  el.innerHTML=positioners.map((p,i)=>`
    <div style="border:1px solid rgba(255,255,255,.1);border-radius:4px;padding:8px;margin-bottom:6px">
      <div style="display:flex;align-items:center;gap:6px;margin-bottom:4px">
        <span style="flex:1;font-family:monospace;font-size:12px;color:var(--txt)">E${p.eNum||i+2} — ${p.type==='cylinder'?'🔵':'📦'} ${p.name||''}</span>
        <button data-pos-edit="${i}" style="background:rgba(37,99,235,.15);border:1px solid rgba(37,99,235,.4);color:#60a5fa;border-radius:3px;padding:1px 7px;cursor:pointer;font-size:11px">✏️</button>
        <button data-pos-del="${i}" style="background:rgba(204,51,51,.15);border:1px solid rgba(204,51,51,.3);color:#f87171;border-radius:3px;padding:1px 6px;cursor:pointer;font-size:12px">✕</button>
      </div>
      <div style="font-size:10px;color:#6a8fa8;font-family:monospace">R${p.rotAxis||'Y'} | ${p.eMin??-180}°–${p.eMax??180}°${p.parentIdx>=0?' | dreht mit E'+(state.positioners[p.parentIdx]?.eNum||'?'):''}</div>
    </div>`).join('');
  el.querySelectorAll('[data-pos-del]').forEach(btn=>btn.addEventListener('click',()=>{
    const i=+btn.dataset.posDel;
    if(state.positioners[i]?._simInterval) clearInterval(state.positioners[i]._simInterval);
    _removePosGroup(i);
    state.positioners.splice(i,1); positionerGroups.splice(i,1);
    rebuildAllPositioners(); renderPosRows(); renderRows();
  }));
  el.querySelectorAll('[data-pos-edit]').forEach(btn=>btn.addEventListener('click',()=>openPosModal(+btn.dataset.posEdit)));
}

function _removePosGroup(i) {
  const g=positionerGroups[i]; if(!g) return;
  if(g.pivotGrp?.parent) g.pivotGrp.parent.remove(g.pivotGrp);
  else if(g.containerGrp?.parent) g.containerGrp.parent.remove(g.containerGrp);
  if(g.pivotSphere?.parent) g.pivotSphere.parent.remove(g.pivotSphere);
}

function rebuildPositionerMesh(i) {
  const p=state.positioners[i]; if(!p) return;
  const deg=Math.PI/180;
  // Remove old group
  _removePosGroup(i);
  const eAx='E'+(p.eNum||i+2);
  const parts=state.axisStlParts[eAx]||[];
  // Container group: positioned by boxOffset
  const containerGrp=new THREE.Group();
  containerGrp.position.set(p.boxOffset?.x||0, p.boxOffset?.y||0, p.boxOffset?.z||0);
  containerGrp.rotation.set((p.boxOffset?.rx||0)*deg,(p.boxOffset?.ry||0)*deg,(p.boxOffset?.rz||0)*deg,'XYZ');
  // Pivot group: at pivot point, rotation applied here
  const pivotGrp=new THREE.Group();
  pivotGrp.position.set(p.pivotX||0, p.pivotY||0, p.pivotZ||0);
  const ang=(p.ePos||0)*deg;
  const rotAx = (p.rotAxis||'Y+').replace(/[+-]/,'');
  const rotSign = (p.rotAxis||'Y+').includes('-') ? -1 : 1;
  if(rotAx==='X') pivotGrp.rotation.x=ang*rotSign;
  else if(rotAx==='Z') pivotGrp.rotation.z=ang*rotSign;
  else pivotGrp.rotation.y=ang*rotSign;
  containerGrp.add(pivotGrp);
  // Mesh group: offset by -pivot so mesh stays at world position
  const meshGrp=new THREE.Group();
  meshGrp.position.set(-(p.pivotX||0),-(p.pivotY||0),-(p.pivotZ||0));
  pivotGrp.add(meshGrp);
  // Build mesh
  if(parts.length){
    parts.forEach(pt=>{ if(!pt.buf)return; const geo=loader.parse(pt.buf.buffer||pt.buf); geo.computeVertexNormals(); meshGrp.add(new THREE.Mesh(geo,new THREE.MeshPhongMaterial({color:pt.color||0xe8a020,shininess:60}))); });
  } else if(p.showBox!==false){
    const L=p.length||500,H=p.height||100,W=p.width||500,R=p.radius||300;
    const geo=p.type==='cylinder'?new THREE.CylinderGeometry(R,R,H,32):new THREE.BoxGeometry(L,H,W);
    meshGrp.add(new THREE.Mesh(geo,new THREE.MeshPhongMaterial({color:p.color||0xe8a020,transparent:true,opacity:0.5,side:THREE.DoubleSide})));
  }
  // Yellow pivot sphere
  const pivotSphere=new THREE.Mesh(
    new THREE.SphereGeometry(18,16,10),
    new THREE.MeshStandardMaterial({color:0xe8a020,emissive:0xe8a020,emissiveIntensity:.4})
  );
  // Sphere at pivot world pos within container
  const sphereGrp=new THREE.Group();
  sphereGrp.position.set(p.pivotX||0, p.pivotY||0, p.pivotZ||0);
  sphereGrp.add(pivotSphere);
  containerGrp.add(sphereGrp);
  // Attach to parent or scene
  if(p.parentIdx>=0 && positionerGroups[p.parentIdx]?.pivotGrp){
    positionerGroups[p.parentIdx].pivotGrp.add(containerGrp);
  } else {
    scene.add(containerGrp);
  }
  positionerGroups[i]={containerGrp, pivotGrp, meshGrp, pivotSphere:sphereGrp};
}

function rebuildAllPositioners(){
  (state.positioners||[]).forEach((_,i)=>rebuildPositionerMesh(i));
}

function openPosModal(editIdx){
  const p=editIdx>=0?(state.positioners||[])[editIdx]:null;
  // Populate parent dropdown
  const sel=$('pm-parent'); if(sel){
    sel.innerHTML='<option value="-1">— keiner —</option>';
    (state.positioners||[]).forEach((pos,i)=>{ if(i!==editIdx) sel.innerHTML+=`<option value="${i}" ${p?.parentIdx===i?'selected':''}>E${pos.eNum||i+2} — ${pos.name||''}</option>`; });
  }
  $('pm-name').value    = p?.name     || '';
  $('pm-type').value    = p?.type     || 'cylinder';
  $('pm-enum').value    = p?.eNum     || (state.positioners.length+2);
  $('pm-radius').value  = p?.radius   || 300;
  $('pm-length').value  = p?.length   || 500;
  $('pm-width').value   = p?.width    || 500;
  $('pm-height').value  = p?.height   || 100;
  $('pm-rotaxis').value = p?.rotAxis  || 'Y';
  $('pm-min').value     = p?.eMin     ?? -180;
  $('pm-max').value     = p?.eMax     ?? 180;
  $('pm-start').value   = p?.ePos     ?? 0;
  $('pm-px').value      = p?.pivotX   || 0;
  $('pm-py').value      = p?.pivotY   || 0;
  $('pm-pz').value      = p?.pivotZ   || 0;
  $('pm-show').checked  = p?.showBox  !== false;
  $('pm-edit-idx').value= editIdx>=0?editIdx:-1;
  const bo=p?.boxOffset||{};
  $('pm-ox').value=bo.x||0;$('pm-oy').value=bo.y||0;$('pm-oz').value=bo.z||0;
  $('pm-orx').value=bo.rx||0;$('pm-ory').value=bo.ry||0;$('pm-orz').value=bo.rz||0;
  pmTypeChanged();
  $('posModal').style.display='flex';
}

function pmTypeChanged(){
  const t=$('pm-type')?.value||'cylinder';
  const bf=$('pm-box-fields'), cf=$('pm-cyl-fields');
  if(bf) bf.style.display=t==='box'?'contents':'none';
  if(cf) cf.style.display=t==='cylinder'?'contents':'none';
}
window.pmTypeChanged=pmTypeChanged;

$('posAddBtn')?.addEventListener('click',()=>openPosModal(-1));
$('posModalClose')?.addEventListener('click',()=>{ $('posModal').style.display='none'; });

$('pm-submit')?.addEventListener('click',()=>{
  const editIdx=parseInt($('pm-edit-idx').value);
  const entry={
    name:      $('pm-name').value||('Pos '+(state.positioners.length+1)),
    type:      $('pm-type').value||'cylinder',
    eNum:      parseInt($('pm-enum').value)||2,
    radius:    parseFloat($('pm-radius').value)||300,
    length:    parseFloat($('pm-length').value)||500,
    width:     parseFloat($('pm-width').value)||500,
    height:    parseFloat($('pm-height').value)||100,
    rotAxis:   $('pm-rotaxis').value||'Y',
    eMin:      parseFloat($('pm-min').value)??-180,
    eMax:      parseFloat($('pm-max').value)??180,
    ePos:      parseFloat($('pm-start').value)||0,
    pivotX:    parseFloat($('pm-px').value)||0,
    pivotY:    parseFloat($('pm-py').value)||0,
    pivotZ:    parseFloat($('pm-pz').value)||0,
    showBox:   $('pm-show').checked===true,
    color:     '#e8a020',
    parentIdx: parseInt($('pm-parent').value)??-1,
    boxOffset: {
      x:parseFloat($('pm-ox').value)||0,y:parseFloat($('pm-oy').value)||0,z:parseFloat($('pm-oz').value)||0,
      rx:parseFloat($('pm-orx').value)||0,ry:parseFloat($('pm-ory').value)||0,rz:parseFloat($('pm-orz').value)||0
    }
  };
  if(editIdx>=0){ _removePosGroup(editIdx); state.positioners[editIdx]=entry; }
  else { state.positioners.push(entry); positionerGroups.push(null); }
  const idx=editIdx>=0?editIdx:state.positioners.length-1;
  // STL → axisStlParts
  if(_pmStlBuf){ const eAx='E'+(entry.eNum||2); state.axisStlParts[eAx]=[{name:entry.name+'.stl',color:entry.color||'#e8a020',buf:_pmStlBuf}]; _pmStlBuf=null; }
  rebuildAllPositioners();
  renderPosRows(); renderRows();
  $('posModal').style.display='none';
});

// ── Feste Objekte ─────────────────────────────────────────────────
function renderFixRows() {
  const el=$('fixRows'); if(!el) return;
  const badge=$('fixBadge');
  const items=state.festeObjekte||[];
  if(badge) badge.textContent=items.length||'0';
  if(!items.length){el.innerHTML='';return;}
  el.innerHTML=items.map((o,i)=>`
    <div style="border:1px solid rgba(255,255,255,.1);border-radius:4px;padding:8px;margin-bottom:6px">
      <div style="display:flex;align-items:center;gap:6px">
        <div style="width:12px;height:12px;border-radius:2px;background:${o.color||'#607080'};flex-shrink:0"></div>
        <span style="flex:1;font-family:monospace;font-size:12px;color:var(--txt)">${o.type==='cylinder'?'🔵':'📦'} ${o.name||'Objekt'}</span>
        <button data-fix-edit="${i}" style="background:rgba(37,99,235,.15);border:1px solid rgba(37,99,235,.4);color:#60a5fa;border-radius:3px;padding:1px 7px;cursor:pointer;font-size:11px">✏️</button>
        <button data-fix-del="${i}" style="background:rgba(204,51,51,.15);border:1px solid rgba(204,51,51,.3);color:#f87171;border-radius:3px;padding:1px 6px;cursor:pointer;font-size:12px">✕</button>
      </div>
    </div>`).join('');
  el.querySelectorAll('[data-fix-del]').forEach(btn=>btn.addEventListener('click',()=>{
    const i=+btn.dataset.fixDel;
    if(festeGrps[i]?.parent) festeGrps[i].parent.remove(festeGrps[i]);
    state.festeObjekte.splice(i,1); festeGrps.splice(i,1);
    renderFixRows();
  }));
  el.querySelectorAll('[data-fix-edit]').forEach(btn=>btn.addEventListener('click',()=>openFixModal(+btn.dataset.fixEdit)));
}

function rebuildFixMesh(i) {
  const o=state.festeObjekte[i]; if(!o) return;
  if(!festeGrps[i]){ festeGrps[i]=new THREE.Group(); scene.add(festeGrps[i]); }
  const g=festeGrps[i];
  while(g.children.length) g.remove(g.children[0]);
  const deg=Math.PI/180;
  g.position.set(o.x||0,o.y||0,o.z||0);
  g.rotation.set((o.rx||0)*deg,(o.ry||0)*deg,(o.rz||0)*deg,'XYZ');
  if(o.stlFile?.buf){
    const geo=loader.parse(o.stlFile.buf.buffer||o.stlFile.buf); geo.computeVertexNormals();
    g.add(new THREE.Mesh(geo,new THREE.MeshPhongMaterial({color:o.color||0x607080,shininess:60})));
  } else if(o.showBox!==false){
    const L=o.length||500,H=o.height||500,W=o.width||500,R=o.radius||200;
    const geo=o.type==='cylinder'?new THREE.CylinderGeometry(R,R,H,32):new THREE.BoxGeometry(L,H,W);
    g.add(new THREE.Mesh(geo,new THREE.MeshPhongMaterial({color:o.color||0x607080,transparent:true,opacity:0.6,side:THREE.DoubleSide})));
  }
}

function openFixModal(editIdx) {
  const o=editIdx>=0?(state.festeObjekte||[])[editIdx]:null;
  $('fm-name').value   = o?.name   || '';
  $('fm-type').value   = o?.type   || 'box';
  $('fm-color').value  = o?.color  || '#607080';
  $('fm-length').value = o?.length || 500;
  $('fm-width').value  = o?.width  || 500;
  $('fm-height').value = o?.height || 500;
  $('fm-radius').value = o?.radius || 200;
  $('fm-x').value  = o?.x  || 0; $('fm-y').value  = o?.y  || 0; $('fm-z').value  = o?.z  || 0;
  $('fm-rx').value = o?.rx || 0; $('fm-ry').value = o?.ry || 0; $('fm-rz').value = o?.rz || 0;
  $('fm-show').checked = o?.showBox !== false;
  $('fm-edit-idx').value = editIdx>=0?editIdx:-1;
  fmTypeChanged();
  $('fixModal').style.display='flex';
}

function fmTypeChanged(){
  const t=$('fm-type')?.value||'box';
  const bf=$('fm-box-fields'), cf=$('fm-cyl-fields');
  if(bf) bf.style.display=t==='box'?'contents':'none';
  if(cf) cf.style.display=t==='cylinder'?'contents':'none';
}
window.fmTypeChanged=fmTypeChanged;
window.openFixModal=openFixModal;

// ── STL-in-Modal Logik für alle Komponentenmodals ─────────────────
var _rmStlBuf=null, _pmStlBuf=null, _omStlBuf=null, _fmStlBuf=null;

function _wireModalStl(prefix, getBufVar, setBufVar) {
  const btn=$(prefix+'-stl-btn'), input=$(prefix+'-stl-file'), clear=$(prefix+'-stl-clear'), disp=$(prefix+'-stl-display');
  btn?.addEventListener('click',()=>input?.click());
  clear?.addEventListener('click',()=>{ setBufVar(null); if(disp){disp.textContent='';} });
  input?.addEventListener('change',async e=>{
    const f=e.target.files[0]; if(!f) return;
    try {
      const { buf, name } = await readStlFile(f);
      setBufVar(buf);
      if(disp){ disp.textContent=name; disp.style.color='#4499cc'; }
    } catch(er) { alert('Fehler: '+er.message); }
    e.target.value='';
  });
}

_wireModalStl('rm', ()=>_rmStlBuf, v=>{ _rmStlBuf=v; });
_wireModalStl('pm', ()=>_pmStlBuf, v=>{ _pmStlBuf=v; });
_wireModalStl('om', ()=>_omStlBuf, v=>{ _omStlBuf=v; });
_wireModalStl('fm', ()=>_fmStlBuf, v=>{ _fmStlBuf=v; });

// Reset STL bufs when modals open
const _origOpenObjModal = openObjModal;
window.openObjModal = function(editIdx) { _omStlBuf=null; _origOpenObjModal(editIdx); };
const _origOpenFixModal = openFixModal;
window.openFixModal = function(editIdx) { _fmStlBuf=null; _origOpenFixModal(editIdx); };

$('fixAddBtn')?.addEventListener('click',()=>openFixModal(-1));
$('fixModalClose')?.addEventListener('click',()=>{ $('fixModal').style.display='none'; });

$('fm-submit')?.addEventListener('click',()=>{
  const editIdx=parseInt($('fm-edit-idx').value);
  const entry={
    name:    $('fm-name').value||'Objekt',
    type:    $('fm-type').value||'box',
    color:   $('fm-color').value||'#607080',
    length:  parseFloat($('fm-length').value)||500,
    width:   parseFloat($('fm-width').value)||500,
    height:  parseFloat($('fm-height').value)||500,
    radius:  parseFloat($('fm-radius').value)||200,
    x: parseFloat($('fm-x').value)||0, y: parseFloat($('fm-y').value)||0, z: parseFloat($('fm-z').value)||0,
    rx:parseFloat($('fm-rx').value)||0,ry:parseFloat($('fm-ry').value)||0,rz:parseFloat($('fm-rz').value)||0,
    showBox: $('fm-show').checked===true,
    stlFile: _fmStlBuf ? {name:$('fm-name').value+'.stl',buf:_fmStlBuf} : (editIdx>=0?state.festeObjekte[editIdx]?.stlFile:null)
  };
  _fmStlBuf=null;
  if(editIdx>=0){ state.festeObjekte[editIdx]=entry; }
  else { state.festeObjekte.push(entry); festeGrps.push(null); }
  rebuildFixMesh(editIdx>=0?editIdx:state.festeObjekte.length-1);
  renderFixRows();
  $('fixModal').style.display='none';
});
var _gimbalActive = false;
var _gimbalMode = 'translate'; // 'translate' | 'rotate'
var _gimbalTarget = null; // {type, idx, grp}

function _getGimbalMeshes() {
  const result = [];
  if(typeof railGroup!=='undefined' && railGroup) {
    railGroup.traverse(c=>{ if(c.isMesh) result.push({mesh:c, type:'rail', idx:0, grp:railGroup}); });
  }
  (objekteGroups||[]).forEach((g,i)=>{ if(g) g.traverse(c=>{ if(c.isMesh) result.push({mesh:c, type:'obj', idx:i, grp:g}); }); });
  (positionerGroups||[]).forEach((g,i)=>{ if(g?.containerGrp) g.containerGrp.traverse(c=>{ if(c.isMesh) result.push({mesh:c, type:'pos', idx:i, grp:g.containerGrp}); }); });
  (festeGrps||[]).forEach((g,i)=>{ if(g) g.traverse(c=>{ if(c.isMesh) result.push({mesh:c, type:'fix', idx:i, grp:g}); }); });
  (effektorGroups||[]).forEach((g,i)=>{ if(g) g.traverse(c=>{ if(c.isMesh) result.push({mesh:c, type:'eff', idx:i, grp:g}); }); });
  (umfGroups||[]).forEach((g,i)=>{ if(g) g.traverse(c=>{ if(c.isMesh) result.push({mesh:c, type:'umf', idx:i, grp:g}); }); });
  return result;
}

function _gimbalPick(event) {
  if(!_gimbalActive) return;
  if(event.button !== undefined && event.button !== 0) return; // left click only
  if(transformControls.dragging) return;
  const rect = renderer.domElement.getBoundingClientRect();
  const clientX = event.clientX ?? event.touches?.[0]?.clientX; if(clientX===undefined) return;
  const clientY = event.clientY ?? event.touches?.[0]?.clientY;
  const mx = ((clientX-rect.left)/rect.width)*2-1;
  const my = -((clientY-rect.top)/rect.height)*2+1;
  const rc = new THREE.Raycaster();
  rc.setFromCamera(new THREE.Vector2(mx,my), camera);
  const allMeshes = _getGimbalMeshes();
  if(!allMeshes.length) return;
  const hits = rc.intersectObjects(allMeshes.map(m=>m.mesh), false);
  if(!hits.length){ transformControls.detach(); _gimbalTarget=null; return; }
  const hit = allMeshes.find(m=>m.mesh===hits[0].object);
  if(!hit) return;
  _gimbalTarget = hit;
  transformControls.setMode(_gimbalMode);
  transformControls.setSize(0.8);
  transformControls.attach(hit.grp);
}

function _gimbalChanged() {
  if(!_gimbalTarget || !transformControls.object) return;
  const grp = transformControls.object;
  const deg = 180/Math.PI;
  const {type, idx} = _gimbalTarget;
  const bo = {
    x: Math.round(grp.position.x), y: Math.round(grp.position.y), z: Math.round(grp.position.z),
    rx: Math.round(grp.rotation.x*deg), ry: Math.round(grp.rotation.y*deg), rz: Math.round(grp.rotation.z*deg)
  };

  function fillModal(prefix) {
    ['x','y','z','rx','ry','rz'].forEach(k=>{
      const el=$(prefix+'-o'+k); if(el) el.value=bo[k]||0;
    });
  }

  if(type==='rail' && state.schienen[0]){
    state.schienen[0].boxOffset=bo;
    // Rail modal
    if($('railModal')?.style.display!=='none') fillModal('rm');
    // Rail panel inputs
    const panel=$('rail-panel');
    if(panel){ ['x','y','z','rx','ry','rz'].forEach(k=>{ const inp=panel.querySelector(`#rail-${k}`); if(inp) inp.value=bo[k]||0; }); }
  } else if(type==='obj' && state.objekte[idx]){
    state.objekte[idx].boxOffset=bo;
    if($('objModal')?.style.display!=='none') fillModal('om');
  } else if(type==='pos' && state.positioners[idx]){
    Object.assign(state.positioners[idx].boxOffset, bo);
    if($('posModal')?.style.display!=='none') fillModal('pm');
  } else if(type==='fix' && state.festeObjekte[idx]){
    Object.assign(state.festeObjekte[idx], {x:bo.x,y:bo.y,z:bo.z,rx:bo.rx,ry:bo.ry,rz:bo.rz});
    if($('fixModal')?.style.display!=='none'){
      ['x','y','z','rx','ry','rz'].forEach(k=>{ const el=$('fm-'+k); if(el) el.value=bo[k]||0; });
    }
  } else if(type==='eff' && state.effektoren[idx]){
    state.effektoren[idx].offset = Object.assign(state.effektoren[idx].offset||{}, bo);
    rebuildEffMesh(idx);
  } else if(type==='umf' && state.umfElemente[idx]){
    state.umfElemente[idx].offset = Object.assign(state.umfElemente[idx].offset||{}, bo);
    rebuildUmfMesh(idx);
    if($('umfModal')?.style.display!=='none'){
      ['x','y','z','rx','ry','rz'].forEach(k=>{ const el=$('umf-'+k); if(el) el.value=bo[k]||0; });
    }
  }
}

// Wire up gimbal toggle
$('gimbalToggle')?.addEventListener('click',()=>{
  _gimbalActive = !_gimbalActive;
  const btn=$('gimbalToggle'), modeBtn=$('gimbalModeBtn');
  if(_gimbalActive){
    btn.style.background='rgba(37,99,235,.3)'; btn.style.borderColor='rgba(37,99,235,.6)'; btn.style.color='#60a5fa';
    if(modeBtn) modeBtn.style.display='';
    transformControls.detach();
    transformControls.removeEventListener('objectChange', onAxisObjectMoved); // pause axis listener
    transformControls.addEventListener('objectChange', _gimbalChanged);
    renderer.domElement.addEventListener('pointerdown', _gimbalPick);
  } else {
    btn.style.background='rgba(255,255,255,.05)'; btn.style.borderColor='rgba(255,255,255,.15)'; btn.style.color='#6a8fa8';
    if(modeBtn) modeBtn.style.display='none';
    transformControls.removeEventListener('objectChange', _gimbalChanged);
    transformControls.addEventListener('objectChange', onAxisObjectMoved); // restore axis listener
    renderer.domElement.removeEventListener('pointerdown', _gimbalPick);
    transformControls.detach(); _gimbalTarget=null;
    const selected = axisMeshes[state.selectedAxis];
    if(selected) transformControls.attach(selected);
  }
});

$('gimbalModeBtn')?.addEventListener('click',()=>{
  _gimbalMode = _gimbalMode==='translate' ? 'rotate' : 'translate';
  const btn=$('gimbalModeBtn');
  btn.textContent = _gimbalMode==='translate' ? 'T' : 'R';
  if(_gimbalTarget) transformControls.setMode(_gimbalMode);
});

// ── Messtool ──────────────────────────────────────────────────────
var _measureActive = false;
var _measureP1 = null;
var _measureP2 = null;
var _measureLine = null;
var _measureSpheres = [];

function _measureReset() {
  _measureP1 = null; _measureP2 = null;
  if(_measureLine){ scene.remove(_measureLine); _measureLine=null; }
  _measureSpheres.forEach(s=>scene.remove(s)); _measureSpheres=[];
  $('msr-dist').textContent='—'; $('msr-dx').textContent='—';
  $('msr-dy').textContent='—'; $('msr-dz').textContent='—';
  $('msr-axy').textContent='—'; $('msr-ayz').textContent='—';
  $('msr-hint').textContent='Klick: P1 setzen';
}

function _measureSphere(pos, color) {
  const m = new THREE.Mesh(
    new THREE.SphereGeometry(12,12,8),
    new THREE.MeshStandardMaterial({color, emissive:color, emissiveIntensity:.5, depthTest:false})
  );
  m.position.copy(pos); m.renderOrder=999;
  scene.add(m); _measureSpheres.push(m); return m;
}

function _measureDrawLine(p1, p2) {
  if(_measureLine) scene.remove(_measureLine);
  const geo = new THREE.BufferGeometry().setFromPoints([p1, p2]);
  _measureLine = new THREE.Line(geo, new THREE.LineBasicMaterial({color:0x60a5fa, depthTest:false, linewidth:2}));
  _measureLine.renderOrder=998;
  scene.add(_measureLine);
}

function _measureUpdate(p1, p2) {
  const dx = p2.x-p1.x, dy = p2.y-p1.y, dz = p2.z-p1.z;
  const dist = Math.sqrt(dx*dx+dy*dy+dz*dz);
  const r = v => Math.round(v);
  const deg = v => (v*180/Math.PI).toFixed(1)+'°';
  $('msr-dist').textContent = r(dist)+' mm';
  $('msr-dx').textContent = r(dx)+' mm';
  $('msr-dy').textContent = r(dy)+' mm';
  $('msr-dz').textContent = r(dz)+' mm';
  // Angle in XY plane (horizontal)
  $('msr-axy').textContent = deg(Math.atan2(dy, dx));
  // Inclination from XY plane (elevation)
  const horiz = Math.sqrt(dx*dx+dy*dy);
  $('msr-ayz').textContent = deg(Math.atan2(dz, horiz));
}

function _measurePick(event) {
  if(!_measureActive) return;
  if(event.button !== undefined && event.button !== 0) return;
  const rect = renderer.domElement.getBoundingClientRect();
  const mx = ((event.clientX-rect.left)/rect.width)*2-1;
  const my = -((event.clientY-rect.top)/rect.height)*2+1;
  const rc = new THREE.Raycaster();
  rc.setFromCamera(new THREE.Vector2(mx,my), camera);
  // Collect all pickable meshes
  const pickable = [];
  scene.traverse(obj=>{ if(obj.isMesh && obj.visible) pickable.push(obj); });
  const hits = rc.intersectObjects(pickable, false);
  if(!hits.length) return;
  const pt = hits[0].point.clone();
  if(!_measureP1) {
    _measureP1 = pt;
    _measureSphere(pt, 0xff4444);
    $('msr-hint').textContent = 'P1 gesetzt — Klick: P2 setzen';
  } else {
    _measureP2 = pt;
    _measureSphere(pt, 0x44ff88);
    _measureDrawLine(_measureP1, _measureP2);
    _measureUpdate(_measureP1, _measureP2);
    $('msr-hint').textContent = 'Neuer Klick: neue Messung starten';
    // Next click resets
    _measureP1 = null; _measureP2 = null;
  }
}

$('measureBtn')?.addEventListener('click',()=>{
  _measureActive = !_measureActive;
  const btn = $('measureBtn');
  const panel = $('measurePanel');
  if(_measureActive){
    btn.style.background='rgba(37,99,235,.3)'; btn.style.borderColor='rgba(37,99,235,.6)'; btn.style.color='#60a5fa';
    if(panel) panel.style.display='';
    _measureReset();
    renderer.domElement.addEventListener('pointerdown', _measurePick);
    // Pause gimbal if active
    if(_gimbalActive) $('gimbalToggle').click();
  } else {
    btn.style.background='rgba(255,255,255,.05)'; btn.style.borderColor='rgba(255,255,255,.15)'; btn.style.color='#6a8fa8';
    if(panel) panel.style.display='none';
    renderer.domElement.removeEventListener('pointerdown', _measurePick);
    _measureReset();
  }
});

function renderTcp(){qsa('.tab').forEach(t=>t.classList.toggle('active',t.dataset.mode===state.activeTcp));const tcp=state.tcp[state.activeTcp];qsa('[data-tcp]').forEach(i=>i.value=tcp?.[i.dataset.tcp]??'');const x=num(tcp?.x),y=num(tcp?.y),z=num(tcp?.z);tcpMarker.visible=x!==null||y!==null||z!==null;tcpMarker.position.set(x||0,y||0,z||0);}




// ── Kameraansichten ────────────────────────────────────────────────
function sceneBox(){
  const box=new THREE.Box3().setFromObject(robotGroup);
  box.expandByObject(toolGroup);
  box.expandByObject(axisPointGroup);
  (festeGrps||[]).forEach(g=>{ if(g) box.expandByObject(g); });
  (objekteGroups||[]).forEach(g=>{ if(g) box.expandByObject(g); });
  (positionerGroups||[]).forEach(g=>{ if(g?.containerGrp) box.expandByObject(g.containerGrp); });
  if(railGroup?.children?.length) box.expandByObject(railGroup);
  (umfGroups||[]).forEach(g=>{ if(g) box.expandByObject(g); });
  if(!Number.isFinite(box.min.x)){box.min.set(-500,-500,0);box.max.set(1500,500,1500);}
  return box;
}
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
  const rail = (state.schienen||[])[0];
  const axes = ['A1','A2','A3','A4','A5','A6'];
  if (rail) axes.push('E'+(rail.eNumber||1));
  el.innerHTML = axes.map(ax => {
    const parts=state.axisStlParts[ax]||[];
    const hasFile=parts.length>0;
    const lbl=hasFile?`${parts.length} Part${parts.length>1?'s':''}`:'—';
    return `<div class="axis-stl-row">
      <span class="axis-stl-label">${ax}</span>
      <span class="axis-stl-name${hasFile?' has-file':''}">${lbl}</span>
      <button class="axis-stl-btn" data-ax="${ax}">STL</button>
      ${hasFile?`<button class="axis-stl-clear" data-ax="${ax}">✕</button>`:''}
    </div>`;
  }).join('');
}

function setAxisColor(ax, hex) {
  colors[ax] = hex;
  const m = ax.match(/^A([1-6])$/);
  if (!m) return;
  const grp = axisPivotGroups[parseInt(m[1]) - 1];
  if (!grp) return;
  grp.children.forEach(child => {
    if (child.isMesh && child.material) {
      child.material.color.set(hex);
      child.material.needsUpdate = true;
    }
  });
}


// ── Multi-STL Parts Modal ─────────────────────────────────────────
let _axisPartsTarget = null;
let _axisPartsPending = null;

function openAxisPartsModal(ax) {
  _axisPartsTarget = ax;
  _axisPartsPending = (state.axisStlParts[ax]||[]).map(p=>({name:p.name,color:p.color,buf:p.buf}));
  $('axisPartsTitle').textContent = 'STL Parts — ' + ax;
  renderAxisPartsList(ax);
  const m = $('axisPartsModal');
  m.style.cssText = 'display:flex;position:fixed;inset:0;z-index:2000;background:rgba(0,0,0,.6);align-items:center;justify-content:center';
}

// Library type filter tabs
document.addEventListener('click', e => {
  const tab = e.target.closest('.lib-tab-btn');
  if (!tab) return;
  _libTypeFilter = tab.dataset.libType || 'all';
  document.querySelectorAll('.lib-tab-btn').forEach(b => {
    const active = b.dataset.libType === _libTypeFilter;
    b.style.background = active ? 'rgba(37,99,235,.2)' : 'rgba(255,255,255,.05)';
    b.style.borderColor = active ? 'rgba(37,99,235,.4)' : 'rgba(255,255,255,.15)';
    b.style.color = active ? '#60a5fa' : '#6a8fa8';
  });
  // Re-render the visible library list
  if (window._libAllItems) renderLibList(window._libAllItems);
});

window.openAxisPartsModal = openAxisPartsModal;
window.saveAxisPartsModal = saveAxisPartsModal;
window.cancelAxisPartsModal = cancelAxisPartsModal;
window.closeAxisPartsModal = closeAxisPartsModal;
window.openNewItemModal = openNewItemModal;

function saveAxisPartsModal() {
  const savedAx = _axisPartsTarget;
  $('axisPartsModal').style.display='none';
  _axisPartsTarget=null; _axisPartsPending=null;
  rebuildRobotKinematics(); applyTransforms();
  renderAxisStlRows(); renderAll();
  // Rebuild object mesh if saved axis is a Label
  if(savedAx && savedAx.startsWith('Label')){
    const idx=(state.objekte||[]).findIndex(o=>'Label'+(o.labelNum||0)===savedAx);
    if(idx>=0) rebuildObjektMesh(idx);
  }
  // Rebuild rail mesh if saved axis is an E-axis
  if(savedAx && savedAx.startsWith('E')) rebuildRailMeshes();
  // Rebuild positioner if saved axis matches
  if(savedAx && savedAx.startsWith('E')){
    (state.positioners||[]).forEach((p,i)=>{ if('E'+(p.eNum||i+2)===savedAx) rebuildPositionerMesh(i); });
  }
}

function cancelAxisPartsModal() {
  const ax=_axisPartsTarget;
  if (ax && _axisPartsPending) {
    state.axisStlParts[ax]=_axisPartsPending;
    state.axisStlMap[ax]=_axisPartsPending[0]?.name||null;
    rebuildRobotKinematics(); applyTransforms();
  }
  $('axisPartsModal').style.display='none';
  _axisPartsTarget=null; _axisPartsPending=null;
  renderAxisStlRows(); renderAll();
}

function closeAxisPartsModal() { cancelAxisPartsModal(); }

function renderAxisPartsList(ax) {
  const el=$('axisPartsList'); if(!el)return;
  const parts=state.axisStlParts[ax]||[];
  if (!parts.length) { el.innerHTML='<div style="color:var(--txt3);font-size:12px;padding:8px 0">Keine STL. Klicke „+ STL hinzufügen“.</div>'; return; }
  el.innerHTML=parts.map((p,i)=>`
    <div style="display:flex;align-items:center;gap:8px;padding:7px 0;border-bottom:1px solid rgba(255,255,255,.07)">
      <label style="width:28px;height:28px;border-radius:3px;background:${p.color||'#e8a020'};border:1px solid rgba(255,255,255,.2);cursor:pointer;flex-shrink:0;overflow:hidden">
        <input type="color" value="${p.color||'#e8a020'}" data-parts-ax="${ax}" data-parts-idx="${i}" style="opacity:0;width:1px;height:1px;position:absolute">
      </label>
      <span style="flex:1;font-family:monospace;font-size:11px;color:var(--txt);overflow:hidden;text-overflow:ellipsis;white-space:nowrap">${p.name}</span>
      <button data-parts-del="${i}" data-parts-ax="${ax}" style="background:rgba(204,51,51,.2);border:1px solid rgba(204,51,51,.4);color:#f87171;border-radius:3px;padding:2px 8px;cursor:pointer;font-size:12px">&#x2715;</button>
    </div>`).join('');
  el.querySelectorAll('[data-parts-del]').forEach(b=>{
    b.onclick=()=>{ state.axisStlParts[ax].splice(+b.dataset.partsDel,1); renderAxisPartsList(ax); rebuildRobotKinematics(); applyTransforms(); };
  });
  el.querySelectorAll('input[type=color][data-parts-idx]').forEach(inp=>{
    inp.addEventListener('change',()=>{
      const p=state.axisStlParts[ax][+inp.dataset.partsIdx]; if(!p)return;
      p.color=inp.value; inp.closest('label').style.background=inp.value;
      for(const[,mesh]of meshes){if(norm(mesh.name)===norm(p.name)){mesh.material.color.set(inp.value);mesh.material.needsUpdate=true;}}
    });
  });
}

async function _loadPartFile(file, ax) {
  let rawBuf, fname;
  if (/\.zip$/i.test(file.name)) {
    try { const r = await extractFromZip(file); rawBuf=r.buf; fname=r.name; } catch(er){alert(er.message);return;}
  } else { rawBuf = await file.arrayBuffer(); fname = file.name; }
  let geom;
  try { geom = await parseGeometry(rawBuf, fname); geom.computeVertexNormals(); } catch(er){alert('Fehler: '+er.message);return;}
  const u8 = new Uint8Array(rawBuf);
  const stlBuf = /\.(stp|step)$/i.test(fname) ? new Uint8Array(stlFromGeometry(geom)) : /\.osd$/i.test(fname) ? new Uint8Array(osdToBinaryStl(rawBuf)) : u8;
  const displayName = fname.replace(/\.(stp|step|osd)$/i, '.stl');
  if (!state.axisStlParts[ax]) state.axisStlParts[ax] = [];
  if (!state.axisStlParts[ax].find(p => norm(p.name)===norm(displayName)))
    state.axisStlParts[ax].push({name:displayName, color:'#e8a020', buf:stlBuf});
  state.axisStlMap[ax] = state.axisStlParts[ax][0].name;
  if (!state.stls.find(f=>f.name===displayName))
    state.stls.push({path:displayName, name:displayName, type:'STL', size:stlBuf.byteLength});
  state.files = state.stls;
  state.buffers.set(displayName, stlBuf);
  const mat = new THREE.MeshStandardMaterial({color:'#e8a020', roughness:.62, metalness:.08});
  const mesh = new THREE.Mesh(geom, mat); mesh.name = displayName;
  meshes.set(displayName, mesh);
}

document.addEventListener('DOMContentLoaded', () => {
  $('axisPartsModal')?.addEventListener('click', e => {
    if (e.target === $('axisPartsModal')) closeAxisPartsModal();
  });
  const fi = $('axisPartsFileInput');
  if (fi) fi.addEventListener('change', async e => {
    const ax = _axisPartsTarget; if (!ax) return;
    const files = Array.from(e.target.files); e.target.value = '';
    for (const file of files) await _loadPartFile(file, ax);
    rebuildRobotKinematics(); applyTransforms();
    renderAxisPartsList(ax); renderAxisStlRows();
  });
});

function initAxisStlEvents() {
  document.addEventListener('change', e => {
    const cp = e.target.closest('.axis-color-pick, [data-axis-color]');
    if (!cp) return;
    const ax = cp.dataset.ax || cp.dataset.axisColor;
    const hex = cp.value;
    setAxisColor(ax, hex);
    // Update label background
    const lbl = cp.closest('label');
    if (lbl) lbl.style.background = hex;
  });
  document.addEventListener('click', e => {
    const btn = e.target.closest('.axis-stl-btn');
    const clr = e.target.closest('.axis-stl-clear');
    if (btn) { _axisPartsTarget = btn.dataset.ax; openAxisPartsModal(btn.dataset.ax); }
    if (clr) {
      const ax=clr.dataset.ax;state.axisStlMap[ax]=null;state.axisStlParts[ax]=[];
      renderAxisStlRows();rebuildRobotKinematics();applyTransforms();
    }
  });
  $('axisStlInput').addEventListener('change', async e => {
    let file = e.target.files[0]; if (!file || !_axisStlTarget) return;
    let rawBuf, fname;
    if (/\.zip$/i.test(file.name)) {
      try { const r = await extractFromZip(file); rawBuf = r.buf; fname = r.name; }
      catch(er) { alert(er.message); e.target.value=''; return; }
    } else { rawBuf = await file.arrayBuffer(); fname = file.name; }
    let geom;
    try { geom = await parseGeometry(rawBuf, fname); geom.computeVertexNormals(); }
    catch(er) { alert('Fehler beim Laden: ' + er.message); e.target.value=''; return; }
    const u8 = new Uint8Array(rawBuf);
    const stlBuf = /\.(stp|step)$/i.test(fname) ? new Uint8Array(stlFromGeometry(geom)) : /\.osd$/i.test(fname) ? new Uint8Array(osdToBinaryStl(rawBuf)) : u8;
    const displayName = fname.replace(/\.(stp|step|osd)$/i, '.stl');
    const ax = _axisPartsTarget||_axisStlTarget;
    if (!state.axisStlParts[ax]) state.axisStlParts[ax]=[];
    if (!state.axisStlParts[ax].find(p=>norm(p.name)===norm(displayName)))
      state.axisStlParts[ax].push({name:displayName,color:'#e8a020',buf:stlBuf});
    state.axisStlMap[ax]=state.axisStlParts[ax][0]?.name||displayName;
    if (!state.stls.find(f=>f.name===displayName)) state.stls.push({path:displayName,name:displayName,type:'STL',size:stlBuf.byteLength});
    state.files=state.stls; state.buffers.set(displayName,stlBuf);
    const mat=new THREE.MeshStandardMaterial({color:'#e8a020',roughness:.62,metalness:.08});
    const mesh=new THREE.Mesh(geom,mat); mesh.name=displayName;
    meshes.set(displayName,mesh);
    rebuildRobotKinematics(); applyTransforms();
    renderAxisStlRows(); renderAll();
    e.target.value = '';
  });
}


// ── Library-Zugriff für Endeffektor & Umfeld ────────────────────

async function libFetchByType(type) {
  const r = await fetch(ROBLIB_API + '?action=list');
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

// Endeffektor und Umfeld Library: jetzt über die zentrale Library (Library-Button in Toolbar)

// ── Endeffektor & Umfeld STL ────────────────────────────────────
// state.effStl  = { path, name, buf }  (ein Endeffektor)
// state.umfStls = [{ path, name, buf }]  (mehrere Umfeld-Teile)

// ── Endeffektor Modal ─────────────────────────────────────────────
var _effStlBuf = null;

function renderEffRow() {
  const el=$('effStlRows'), badge=$('effBadge'); if(!el) return;
  const effs=state.effektoren||[];
  if(badge) badge.textContent=effs.length||'0';
  if(!effs.length){el.innerHTML='';return;}
  el.innerHTML=effs.map((e,i)=>{
    const teile=e.teile||[];
    const allMin=teile.length?Math.min(...teile.map(t=>t.eMin||0)):0;
    const allMax=teile.length?Math.max(...teile.map(t=>t.eMax??50)):50;
    const sliderHtml='';
    const teileHtml=teile.map((t,j)=>`
      <div style="display:flex;align-items:center;gap:4px;margin-top:4px;padding:3px 4px;background:rgba(255,255,255,.03);border-radius:3px">
        <span style="flex:1;font-family:monospace;font-size:11px;color:#aac">${t.name||'Teil '+(j+1)} <span style="color:#6a8fa8">${t.axis||'Y+'} ${t.eMin||0}…${t.eMax??50}mm</span></span>
        <button data-teil-stl="${i}-${j}" style="background:rgba(255,255,255,.05);border:1px solid rgba(255,255,255,.12);color:${t.stlFile?'#4499cc':'#6a8fa8'};border-radius:3px;padding:0 5px;cursor:pointer;font-size:11px" title="STL laden">📁</button>
        <button data-teil-edit="${i}-${j}" style="background:rgba(37,99,235,.12);border:1px solid rgba(37,99,235,.3);color:#60a5fa;border-radius:3px;padding:0 5px;cursor:pointer;font-size:11px">✏️</button>
        <button data-teil-del="${i}-${j}" style="background:rgba(204,51,51,.12);border:1px solid rgba(204,51,51,.25);color:#f87171;border-radius:3px;padding:0 5px;cursor:pointer;font-size:11px">✕</button>
      </div>`).join('');
    return `
    <div style="border:1px solid rgba(255,255,255,.1);border-radius:4px;padding:8px;margin-bottom:6px">
      <div style="display:flex;align-items:center;gap:6px">
        <span style="flex:1;font-family:monospace;font-size:12px;color:var(--txt)">🔧 ${e.name||'Endeffektor '+(i+1)}</span>
        <button data-eff-stl="${i}" style="background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.15);color:${e.stlFile?'#4499cc':'#6a8fa8'};border-radius:3px;padding:1px 6px;cursor:pointer;font-size:11px" title="Basis-STL laden">📁</button>
        <button data-teil-add="${i}" style="background:rgba(34,197,94,.12);border:1px solid rgba(34,197,94,.3);color:#4ade80;border-radius:3px;padding:1px 6px;cursor:pointer;font-size:11px" title="Backe / Teil hinzufügen">+ Teil</button>
        <button data-eff-edit="${i}" style="background:rgba(37,99,235,.15);border:1px solid rgba(37,99,235,.4);color:#60a5fa;border-radius:3px;padding:1px 7px;cursor:pointer;font-size:11px">✏️</button>
        <button data-eff-del="${i}" style="background:rgba(204,51,51,.15);border:1px solid rgba(204,51,51,.3);color:#f87171;border-radius:3px;padding:1px 6px;cursor:pointer;font-size:12px">✕</button>
      </div>
      ${teileHtml}
      ${sliderHtml}
    </div>`;
  }).join('');
  el.querySelectorAll('[data-eff-stl]').forEach(btn=>btn.addEventListener('click',()=>{
    const inp=$('effStlInput'); if(!inp) return;
    inp.dataset.effIdx=btn.dataset.effStl; inp.dataset.teilIdx=''; inp.click();
  }));
  el.querySelectorAll('[data-teil-add]').forEach(btn=>btn.addEventListener('click',()=>openTeilModal(+btn.dataset.teilAdd,-1)));
  el.querySelectorAll('[data-teil-stl]').forEach(btn=>btn.addEventListener('click',()=>{
    const [ei,ti]=btn.dataset.teilStl.split('-').map(Number);
    const inp=$('effStlInput'); if(!inp) return;
    inp.dataset.effIdx=ei; inp.dataset.teilIdx=ti; inp.click();
  }));
  el.querySelectorAll('[data-teil-edit]').forEach(btn=>btn.addEventListener('click',()=>{
    const [ei,ti]=btn.dataset.teilEdit.split('-').map(Number); openTeilModal(ei,ti);
  }));
  el.querySelectorAll('[data-teil-del]').forEach(btn=>btn.addEventListener('click',()=>{
    const [ei,ti]=btn.dataset.teilDel.split('-').map(Number);
    (state.effektoren[ei].teile||[]).splice(ti,1);
    renderEffRow(); rebuildEffMesh(ei); applyTransforms();
  }));
  el.querySelectorAll('[data-eff-del]').forEach(btn=>btn.addEventListener('click',()=>{
    const i=+btn.dataset.effDel;
    if(effektorGroups[i]?.parent) effektorGroups[i].parent.remove(effektorGroups[i]);
    effektorGroups.splice(i,1); state.effektoren.splice(i,1);
    renderEffRow(); applyTransforms();
  }));
  el.querySelectorAll('[data-eff-edit]').forEach(btn=>btn.addEventListener('click',()=>openEffModal(+btn.dataset.effEdit)));
}

var _teAxis='Y+', _teStlBuf=null;
function openTeilModal(effIdx, teilIdx) {
  _teStlBuf=null; _teAxis='Y+';
  const t=(teilIdx>=0)?(state.effektoren[effIdx]?.teile||[])[teilIdx]:null;
  $('te-name').value=t?.name||'';
  if($('te-labelnum')) $('te-labelnum').value=t?.labelNum??1;
  $('te-type').value=t?.objectType||'box';
  $('te-color').value=t?.color||'#607080';
  $('te-length').value=t?.length||100; $('te-width').value=t?.width||50;
  $('te-height').value=t?.height||50; $('te-radius').value=t?.radius||25;
  $('te-min').value=t?.eMin??0; $('te-max').value=t?.eMax??50;
  const bo=t?.offset||{};
  $('te-ox').value=bo.x||0;$('te-oy').value=bo.y||0;$('te-oz').value=bo.z||0;
  $('te-orx').value=bo.rx||0;$('te-ory').value=bo.ry||0;$('te-orz').value=bo.rz||0;
  $('te-stl-name').textContent=t?.stlFile?.name||'Keine STL';
  if($('te-moveangle')) $('te-moveangle').value=t?.moveAngle??'';
  if(t?.stlFile?.buf) _teStlBuf=t.stlFile.buf;
  _teAxis=t?.axis||'Y+';
  document.querySelectorAll('.te-axis-btn').forEach(b=>{
    const on=b.dataset.ax===_teAxis;
    b.style.background=on?'rgba(37,99,235,.3)':'rgba(255,255,255,.05)';
    b.style.border=on?'1px solid rgba(37,99,235,.6)':'1px solid rgba(255,255,255,.15)';
    b.style.color=on?'#60a5fa':'#6a8fa8';
  });
  $('te-edit-eff-idx').value=effIdx; $('te-edit-teil-idx').value=teilIdx;
  teTypeChanged();
  $('teilModal').style.display='flex';
}
window.openTeilModal=openTeilModal;
function teTypeChanged(){
  const t=$('te-type')?.value||'box';
  const bf=$('te-box-fields'),cf=$('te-cyl-fields'),sf=$('te-stl-field');
  if(bf)bf.style.display=t==='box'?'contents':'none';
  if(cf)cf.style.display=t==='cylinder'?'contents':'none';
  if(sf)sf.style.display=t==='stl'?'':'none';
}
window.teTypeChanged=teTypeChanged;
document.querySelectorAll('.te-axis-btn').forEach(b=>b.addEventListener('click',()=>{
  _teAxis=b.dataset.ax;
  document.querySelectorAll('.te-axis-btn').forEach(x=>{
    const on=x.dataset.ax===_teAxis;
    x.style.background=on?'rgba(37,99,235,.3)':'rgba(255,255,255,.05)';
    x.style.border=on?'1px solid rgba(37,99,235,.6)':'1px solid rgba(255,255,255,.15)';
    x.style.color=on?'#60a5fa':'#6a8fa8';
  });
}));
$('te-stl-btn')?.addEventListener('click',()=>$('te-stl-input')?.click());
$('te-stl-input')?.addEventListener('change',async e=>{
  const f=e.target.files[0]; if(!f) return;
  try { const r=await readStlFile(f); _teStlBuf=r.buf; $('te-stl-name').textContent=r.name; $('te-stl-name').style.color='#4499cc'; }
  catch(er){ alert('Fehler: '+er.message); }
  e.target.value='';
});
$('teilModalClose')?.addEventListener('click',()=>{ $('teilModal').style.display='none'; });
$('te-submit')?.addEventListener('click',()=>{
  const effIdx=parseInt($('te-edit-eff-idx').value);
  const teilIdx=parseInt($('te-edit-teil-idx').value);
  const eff=state.effektoren[effIdx]; if(!eff) return;
  if(!eff.teile) eff.teile=[];
  const t2=$('te-type')?.value||'box';
  const entry={
    name:$('te-name').value||('Teil '+(eff.teile.length+1)),
    labelNum: parseInt($('te-labelnum')?.value)||null,
    moveAngle: $('te-moveangle')?.value!=='' ? parseFloat($('te-moveangle').value) : null,
    objectType:t2, color:$('te-color').value||'#607080',
    length:parseFloat($('te-length').value)||100, width:parseFloat($('te-width').value)||50,
    height:parseFloat($('te-height').value)||50, radius:parseFloat($('te-radius').value)||25,
    eMin:parseFloat($('te-min').value)||0, eMax:parseFloat($('te-max').value)??50,
    axis:_teAxis,
    stlFile:_teStlBuf?{name:($('te-name').value||'teil')+'.stl',buf:_teStlBuf}:(teilIdx>=0?eff.teile[teilIdx]?.stlFile:null),
    offset:{x:parseFloat($('te-ox').value)||0,y:parseFloat($('te-oy').value)||0,z:parseFloat($('te-oz').value)||0,
      rx:parseFloat($('te-orx').value)||0,ry:parseFloat($('te-ory').value)||0,rz:parseFloat($('te-orz').value)||0}
  };
  if(teilIdx>=0) eff.teile[teilIdx]=entry; else eff.teile.push(entry);
  renderEffRow(); rebuildEffMesh(effIdx); applyTransforms();
  $('teilModal').style.display='none';
});

function openEffModal(editIdx) {
  _effStlBuf=null;
  const e=editIdx>=0?(state.effektoren||[])[editIdx]:null;
  $('eff-name').value  = e?.name  || '';
  $('eff-type').value  = e?.objectType||e?.typ||'box';
  $('eff-color').value = e?.color || '#607080';
  $('eff-length').value= e?.length||200; $('eff-width').value=e?.width||200;
  $('eff-height').value= e?.height||300; $('eff-radius').value=e?.radius||80;
  $('eff-x').value=e?.offset?.x||0; $('eff-y').value=e?.offset?.y||0; $('eff-z').value=e?.offset?.z||0;
  $('eff-rx').value=e?.offset?.rx||0;$('eff-ry').value=e?.offset?.ry||0;$('eff-rz').value=e?.offset?.rz||0;
  $('eff-stl-name').textContent=e?.stlFile?.name||'Keine STL gewählt';
  if(e?.stlFile?.buf) _effStlBuf=e.stlFile.buf;
  $('eff-edit-idx').value=editIdx>=0?editIdx:-1;
  effTypeChanged();
  $('effModal').style.display='flex';
}
window.openEffModal=openEffModal;

function effTypeChanged(){
  const t=$('eff-type')?.value||'box';
  const pf=$('eff-prim-fields'),bf=$('eff-box-fields'),cf=$('eff-cyl-fields'),sf=$('eff-stl-field');
  if(pf) pf.style.display=t==='stl'?'none':'contents';
  if(bf) bf.style.display=t==='box'?'contents':'none';
  if(cf) cf.style.display=t==='cylinder'?'contents':'none';
  if(sf) sf.style.display=t==='stl'?'':'none';
}
window.effTypeChanged=effTypeChanged;

$('eff-stl-btn')?.addEventListener('click',()=>$('eff-stl-input')?.click());
$('eff-stl-input')?.addEventListener('change',async e=>{
  const f=e.target.files[0]; if(!f) return;
  try { const r=await readStlFile(f); _effStlBuf=r.buf; $('eff-stl-name').textContent=r.name; $('eff-stl-name').style.color='#4499cc'; }
  catch(er){ alert('Fehler: '+er.message); }
  e.target.value='';
});
$('effModalClose')?.addEventListener('click',()=>{ $('effModal').style.display='none'; });
$('eff-submit')?.addEventListener('click',()=>{
  const editIdx=parseInt($('eff-edit-idx').value);
  const t=$('eff-type')?.value||'box';
  const entry={
    name:$('eff-name').value||'Endeffektor', objectType:t,
    color:$('eff-color').value||'#607080',
    length:parseFloat($('eff-length').value)||200, width:parseFloat($('eff-width').value)||200,
    height:parseFloat($('eff-height').value)||300, radius:parseFloat($('eff-radius').value)||80,
    stlFile: _effStlBuf ? {name:$('eff-name').value+'.stl', buf:_effStlBuf} : (editIdx>=0?state.effektoren[editIdx]?.stlFile:null),
    offset:{x:parseFloat($('eff-x').value)||0,y:parseFloat($('eff-y').value)||0,z:parseFloat($('eff-z').value)||0,
      rx:parseFloat($('eff-rx').value)||0,ry:parseFloat($('eff-ry').value)||0,rz:parseFloat($('eff-rz').value)||0}
  };
  state.effektoren=state.effektoren||[];
  if(editIdx>=0) state.effektoren[editIdx]=entry; else state.effektoren.push(entry);
  state.activeEff=editIdx>=0?editIdx:state.effektoren.length-1;
  renderEffRow(); rebuildRobotKinematics(); applyTransforms(); updateEffTcpMarker?.();
  $('effModal').style.display='none';
});

// ── Umgebung Modal ────────────────────────────────────────────────
var _umfStlBuf = null;

function renderUmfRows() {
  const el=$('umfStlRows'), badge=$('umfBadge'); if(!el) return;
  const elms=state.umfElemente||[];
  if(badge) badge.textContent=elms.length||'0';
  if(!elms.length){el.innerHTML='';return;}
  el.innerHTML=elms.map((u,i)=>`
    <div style="border:1px solid rgba(255,255,255,.1);border-radius:4px;padding:8px;margin-bottom:6px">
      <div style="display:flex;align-items:center;gap:6px">
        <span style="flex:1;font-family:monospace;font-size:12px;color:var(--txt)">🏭 ${u.name||'Umgebung '+(i+1)}</span>
        <button data-umf-stl="${i}" style="background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.15);color:${u.stlFile?'#4499cc':'#6a8fa8'};border-radius:3px;padding:1px 6px;cursor:pointer;font-size:11px" title="STL laden">📁</button>
        <button data-umf-edit="${i}" style="background:rgba(37,99,235,.15);border:1px solid rgba(37,99,235,.4);color:#60a5fa;border-radius:3px;padding:1px 7px;cursor:pointer;font-size:11px">✏️</button>
        <button data-umf-del="${i}" style="background:rgba(204,51,51,.15);border:1px solid rgba(204,51,51,.3);color:#f87171;border-radius:3px;padding:1px 6px;cursor:pointer;font-size:12px">✕</button>
      </div>
    </div>`).join('');
  el.querySelectorAll('[data-umf-stl]').forEach(btn=>btn.addEventListener('click',()=>{
    const inp=$('umfStlInput'); if(!inp) return;
    inp.dataset.umfIdx=btn.dataset.umfStl; inp.click();
  }));
  el.querySelectorAll('[data-umf-del]').forEach(btn=>btn.addEventListener('click',()=>{
    const di=+btn.dataset.umfDel;
    if(umfGroups[di]&&umfGroups[di].parent) umfGroups[di].parent.remove(umfGroups[di]);
    state.umfElemente.splice(di,1); umfGroups.splice(di,1);
    renderUmfRows(); applyTransforms();
  }));
  el.querySelectorAll('[data-umf-edit]').forEach(btn=>btn.addEventListener('click',()=>openUmfModal(+btn.dataset.umfEdit)));
}

function openUmfModal(editIdx) {
  _umfStlBuf=null;
  const u=editIdx>=0?(state.umfElemente||[])[editIdx]:null;
  $('umf-name').value  = u?.name  || '';
  $('umf-type').value  = u?.objectType||(u?.stlFile?'stl':'box');
  $('umf-color').value = u?.color || '#3a5a7a';
  $('umf-length').value= u?.length||2000; $('umf-width').value=u?.width||200;
  $('umf-height').value= u?.height||2000; $('umf-radius').value=u?.radius||500;
  const o=u?.offset||{};
  $('umf-x').value=o.x||0;$('umf-y').value=o.y||0;$('umf-z').value=o.z||0;
  $('umf-rx').value=o.rx||0;$('umf-ry').value=o.ry||0;$('umf-rz').value=o.rz||0;
  $('umf-show').checked=u?.showBox!==false;
  $('umf-stl-name').textContent=u?.stlFile?.name||'Keine STL gewählt';
  if(u?.stlFile?.buf) _umfStlBuf=u.stlFile.buf;
  $('umf-edit-idx').value=editIdx>=0?editIdx:-1;
  umfTypeChanged();
  $('umfModal').style.display='flex';
}
window.openUmfModal=openUmfModal;

function umfTypeChanged(){
  const t=$('umf-type')?.value||'box';
  const pf=$('umf-prim-fields'),bf=$('umf-box-fields'),cf=$('umf-cyl-fields'),sf=$('umf-stl-field');
  if(pf) pf.style.display=t==='stl'?'none':'contents';
  if(bf) bf.style.display=t==='box'?'contents':'none';
  if(cf) cf.style.display=t==='cylinder'?'contents':'none';
  if(sf) sf.style.display=t==='stl'?'':'none';
}
window.umfTypeChanged=umfTypeChanged;

$('umf-stl-btn')?.addEventListener('click',()=>$('umf-stl-input')?.click());
$('umf-stl-input')?.addEventListener('change',async e=>{
  const f=e.target.files[0]; if(!f) return;
  try { const r=await readStlFile(f); _umfStlBuf=r.buf; $('umf-stl-name').textContent=r.name; $('umf-stl-name').style.color='#4499cc'; }
  catch(er){ alert('Fehler: '+er.message); }
  e.target.value='';
});
$('umfModalClose')?.addEventListener('click',()=>{ $('umfModal').style.display='none'; });
$('umf-submit')?.addEventListener('click',()=>{
  const editIdx=parseInt($('umf-edit-idx').value);
  const t=$('umf-type')?.value||'box';
  const entry={
    name:$('umf-name').value||'Umgebung', objectType:t, color:$('umf-color').value||'#3a5a7a',
    length:parseFloat($('umf-length').value)||2000, width:parseFloat($('umf-width').value)||200,
    height:parseFloat($('umf-height').value)||2000, radius:parseFloat($('umf-radius').value)||500,
    showBox:$('umf-show').checked!==false,
    stlFile: _umfStlBuf ? {name:$('umf-name').value+'.stl',buf:_umfStlBuf} : (editIdx>=0?state.umfElemente[editIdx]?.stlFile:null),
    offset:{x:parseFloat($('umf-x').value)||0,y:parseFloat($('umf-y').value)||0,z:parseFloat($('umf-z').value)||0,
      rx:parseFloat($('umf-rx').value)||0,ry:parseFloat($('umf-ry').value)||0,rz:parseFloat($('umf-rz').value)||0}
  };
  state.umfElemente=state.umfElemente||[];
  if(editIdx>=0) state.umfElemente[editIdx]=entry; else state.umfElemente.push(entry);
  renderUmfRows(); rebuildRobotKinematics(); applyTransforms();
  $('umfModal').style.display='none';
});

// ── Schienen (parametrisch) ───────────────────────────────────────
function renderRailRows() {
  const el = $('railRows'); if(!el) return;
  const badge = $('railBadge');
  const btn = $('railAddBtn');
  const schienen = state.schienen || [];
  if(badge) badge.textContent = schienen.length||'0';
  // Max 1 Rail: Button text ändern
  if(btn) {
    if(schienen.length) { btn.textContent='ändern'; btn.title='Schiene bearbeiten'; }
    else { btn.textContent='+'; btn.title='Neue Schiene erstellen'; }
  }
  if(!schienen.length){ el.innerHTML=''; return; }
  const r=schienen[0];
  el.innerHTML = `<div style="border:1px solid rgba(255,255,255,.1);border-radius:4px;padding:8px">
    <div style="display:flex;align-items:center;gap:6px;margin-bottom:6px">
      <span style="flex:1;font-family:monospace;font-size:12px;color:var(--txt)">${r.name||'Rail'}</span>
      <button id="railDelBtn" style="min-width:28px;height:28px;cursor:pointer;background:rgba(204,51,51,.15);border:1px solid rgba(204,51,51,.3);color:#f87171;border-radius:3px;font-size:12px">✕</button>
    </div>
    <div style="font-size:11px;color:#6a8fa8;font-family:monospace">${r.length_mm} × ${r.width_mm} × ${r.height_mm} mm | ${r.axis}</div>
  </div>`;
  $('railDelBtn')?.addEventListener('click',()=>{
    state.schienen=[];
    kinematicsRoot.position.set(0,0,0);
    robotGroup.position.set(state.robotTr?.x||0, state.robotTr?.y||0, state.robotTr?.z||0);
    scene.updateMatrixWorld(true);
    updateAxisPointVisuals(); updateSkeletonPositions();
    renderRailRows(); rebuildRailMeshes();
  });
}

function rebuildRailMeshes() {
  if(!railGroup) return;
  clearGroup(railGroup);
  const r = (state.schienen||[])[0];
  if(!r) return;
  const L=r.length_mm||2000, H=r.height_mm||200, W=r.width_mm||400, ax=r.axis||'X+';
  const p=r.ePos||0, bo=r.boxOffset||{}, deg=Math.PI/180;
  const eAx = 'E'+(r.eNumber||1);
  const parts = state.axisStlParts[eAx]||[];
  if (parts.length) {
    parts.forEach(pt=>{
      if(!pt.buf) return;
      const geo=loader.parse(pt.buf.buffer||pt.buf); geo.computeVertexNormals();
      railGroup.add(new THREE.Mesh(geo,new THREE.MeshPhongMaterial({color:pt.color||0x2563eb,shininess:60})));
    });
  }
  if (r.showBox !== false && !parts.length) {
    const geo=(ax==='X+'||ax==='X-')?new THREE.BoxGeometry(L,H,W):
              (ax==='Y+'||ax==='Y-')?new THREE.BoxGeometry(W,L,H):
                                     new THREE.BoxGeometry(W,H,L);
    railGroup.add(new THREE.Mesh(geo,new THREE.MeshPhongMaterial({color:0x2563eb,transparent:true,opacity:0.3,side:THREE.DoubleSide})));
  }
  // Rail stays fixed at boxOffset
  railGroup.position.set(bo.x||0, bo.y||0, bo.z||0);
  railGroup.rotation.set((bo.rx||0)*deg, (bo.ry||0)*deg, (bo.rz||0)*deg, 'XYZ');
  // Movement direction
  var cx=0, cy=0, cz=0;
  if      (ax==='X+') cx =  p;
  else if (ax==='X-') cx = -p;
  else if (ax==='Y+') cy =  p;
  else if (ax==='Y-') cy = -p;
  else if (ax==='Z+') cz =  p;
  else                cz = -p;

  if (r.robotMoves === true) {
    // Roboter verfährt, Rail bleibt
    var bx=state.robotTr?.x||0, by=state.robotTr?.y||0, bz=state.robotTr?.z||0;
    kinematicsRoot.position.set(bx+cx, by+cy, bz+cz);
    robotGroup.position.set(bx+cx, by+cy, bz+cz);
    scene.updateMatrixWorld(true);
    updateAxisPointVisuals();
    updateSkeletonPositions();
  } else {
    // Rail verfährt, Roboter bleibt
    var bx=state.robotTr?.x||0, by=state.robotTr?.y||0, bz=state.robotTr?.z||0;
    kinematicsRoot.position.set(bx, by, bz);
    robotGroup.position.set(bx, by, bz);
    railGroup.position.set((bo.x||0) - cx, (bo.y||0) - cy, (bo.z||0) - cz);
    scene.updateMatrixWorld(true);
    updateAxisPointVisuals();
    updateSkeletonPositions();
  }
}

$('railAddBtn')?.addEventListener('click',()=>{
  const existing = (state.schienen||[])[0];
  $('rm-name').value   = existing?.name       || '';
  $('rm-length').value = existing?.length_mm  || 2000;
  $('rm-height').value = existing?.height_mm  || 200;
  $('rm-width').value  = existing?.width_mm   || 400;
  $('rm-min').value    = existing?.eMin       ?? 0;
  $('rm-max').value    = existing?.eMax       ?? (existing?.length_mm || 2000);
  $('rm-start').value  = existing?.ePos       ?? 0;
  if($('rm-show')) $('rm-show').checked = existing?.showBox !== false;
  if($('rm-robot-moves')) $('rm-robot-moves').checked = existing?.robotMoves !== false;
  const bo = existing?.boxOffset||{};
  if($('rm-ox'))  $('rm-ox').value  = bo.x  ||0;
  if($('rm-oy'))  $('rm-oy').value  = bo.y  ||0;
  if($('rm-oz'))  $('rm-oz').value  = bo.z  ||0;
  if($('rm-orx')) $('rm-orx').value = bo.rx ||0;
  if($('rm-ory')) $('rm-ory').value = bo.ry ||0;
  if($('rm-orz')) $('rm-orz').value = bo.rz ||0;
  _rmAxis = existing?.axis || 'Y+';
  document.querySelectorAll('.rm-axis-btn').forEach(b=>{
    const on=b.dataset.ax===_rmAxis;
    b.style.background=on?'rgba(37,99,235,.3)':'rgba(255,255,255,.05)';
    b.style.border=on?'1px solid rgba(37,99,235,.6)':'1px solid rgba(255,255,255,.15)';
    b.style.color=on?'#60a5fa':'#6a8fa8';
  });
  $('railModal').style.display='flex';
});

let _rmAxis='Y+';
document.querySelectorAll('.rm-axis-btn').forEach(b=>{
  b.addEventListener('click',()=>{
    _rmAxis=b.dataset.ax;
    document.querySelectorAll('.rm-axis-btn').forEach(x=>{
      const on=x.dataset.ax===_rmAxis;
      x.style.background=on?'rgba(37,99,235,.3)':'rgba(255,255,255,.05)';
      x.style.border=on?'1px solid rgba(37,99,235,.6)':'1px solid rgba(255,255,255,.15)';
      x.style.color=on?'#60a5fa':'#6a8fa8';
    });
  });
});

$('rm-submit')?.addEventListener('click',()=>{
  if(!state.schienen) state.schienen=[];
  const entry = {
    name: $('rm-name').value||('Rail 1'),
    length_mm: parseFloat($('rm-length').value)||2000,
    height_mm: parseFloat($('rm-height').value)||200,
    width_mm:  parseFloat($('rm-width').value)||400,
    axis: _rmAxis,
    eNumber: parseInt($('rm-enum')?.value)||1,
    eMin:  parseFloat($('rm-min')?.value)  || 0,
    eMax:  parseFloat($('rm-max')?.value)  || 2000,
    ePos:  parseFloat($('rm-start')?.value)|| 0,
    boxOffset: {
      x:  parseFloat($('rm-ox')?.value) ||0, y:  parseFloat($('rm-oy')?.value) ||0, z:  parseFloat($('rm-oz')?.value) ||0,
      rx: parseFloat($('rm-orx')?.value)||0, ry: parseFloat($('rm-ory')?.value)||0, rz: parseFloat($('rm-orz')?.value)||0
    },
    showBox: $('rm-show')?.checked === true,
    robotMoves: $('rm-robot-moves')?.checked === true
  };
  const old = state.schienen[0];
  if(old?._simInterval){ clearInterval(old._simInterval); delete old._simInterval; }
  state.schienen[0] = entry;
  // STL → axisStlParts
  if(_rmStlBuf){ const eAx='E'+(entry.eNumber||1); state.axisStlParts[eAx]=[{name:entry.name+'.stl',color:'#2563eb',buf:_rmStlBuf}]; _rmStlBuf=null; }
  renderRailRows(); rebuildRailMeshes(); renderRows();
  $('railModal').style.display='none';
});

$('railModalClose')?.addEventListener('click',()=>{ $('railModal').style.display='none'; });

// ── Externe Achsen Modal ──────────────────────────────────────────
function renderExtAxesModal() {
  const body = $('extAxesBody'); if(!body) return;
  const rail = (state.schienen||[])[0];
  if(!rail){ body.innerHTML='<div style="font-size:11px;color:#4a6a8a;font-family:monospace">Keine externen Achsen definiert.</div>'; return; }
  const eAx = 'E'+(rail.eNumber||1);
  const parts = state.axisStlParts[eAx]||[];
  const col = parts[0]?.color||'#2563eb';
  body.innerHTML = `
    <div style="overflow-x:auto">
    <table style="width:100%;border-collapse:collapse;font-family:monospace;font-size:12px">
      <thead><tr style="color:#6a8fa8;font-size:10px;letter-spacing:.05em">
        <th style="padding:4px 6px;text-align:left">Achse</th>
        <th style="padding:4px 6px">mm</th>
        <th style="padding:4px 6px">Richtung</th>
        <th style="padding:4px 6px">Min</th>
        <th style="padding:4px 6px">Max</th>
        <th style="padding:4px 6px">🎨</th>
        <th style="padding:4px 6px">STL</th>
        <th style="padding:4px 6px">Sim</th>
      </tr></thead>
      <tbody id="extAxesRows"></tbody>
    </table></div>`;
  const inp = (id,val,extra='')=>`<input id="${id}" type="number" step="1" value="${val}" ${extra} style="width:68px;background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.1);border-radius:3px;padding:2px 4px;color:#d8e8f0;font-family:monospace;font-size:12px;outline:none">`;
  $('extAxesRows').innerHTML = `<tr style="border-top:1px solid rgba(255,255,255,.08)">
    <td style="padding:4px 6px"><b>${eAx}</b></td>
    <td style="padding:4px 2px">${inp('ea-pos', rail.ePos||0,'min="'+(rail.eMin??0)+'" max="'+(rail.eMax??rail.length_mm??2000)+'"')}</td>
    <td style="padding:4px 6px;color:#9ab">${rail.axis||'X+'}</td>
    <td style="padding:4px 2px">${inp('ea-min', rail.eMin??0)}</td>
    <td style="padding:4px 2px">${inp('ea-max', rail.eMax??rail.length_mm??2000)}</td>
    <td style="padding:4px 6px"><label style="display:inline-block;width:26px;height:22px;border-radius:3px;background:${col};border:1px solid rgba(255,255,255,.25);cursor:pointer;overflow:hidden"><input type="color" id="ea-color" value="${col}" style="opacity:0;width:1px;height:1px;position:absolute"></label></td>
    <td style="padding:4px 6px"><button id="ea-stl-btn" style="font-size:10px;padding:3px 7px;background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.15);border-radius:3px;cursor:pointer;color:${parts.length?'#d8e8f0':'#6a8fa8'};white-space:nowrap">${parts.length?parts.length+' Part'+(parts.length>1?'s':''):'+ STL'}</button></td>
    <td style="padding:4px 6px"><button id="ea-sim-btn" style="background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.15);border-radius:3px;padding:2px 7px;cursor:pointer;color:#9ab">▶</button></td>
  </tr>`;
  $('ea-pos')?.addEventListener('input',()=>{ if(rail){rail.ePos=parseFloat($('ea-pos').value)||0; rebuildRailMeshes();}});
  $('ea-min')?.addEventListener('input',()=>{ if(rail) rail.eMin=parseFloat($('ea-min').value)||0; });
  $('ea-max')?.addEventListener('input',()=>{ if(rail) rail.eMax=parseFloat($('ea-max').value)||0; });
  $('ea-color')?.addEventListener('change',()=>{
    const c=$('ea-color').value;
    if(!state.axisStlParts[eAx]) state.axisStlParts[eAx]=[];
    if(state.axisStlParts[eAx][0]) state.axisStlParts[eAx][0].color=c;
    $('ea-color').closest('label').style.background=c;
    rebuildRailMeshes(); renderAxisStlRows();
  });
  $('ea-stl-btn')?.addEventListener('click',()=>{ openAxisPartsModal(eAx); $('extAxesModal').style.display='none'; });
  $('ea-sim-btn')?.addEventListener('click',()=>{
    if(!rail) return;
    const max=rail.eMax??rail.length_mm??2000;
    let pos=0, dir=1;
    const step=()=>{
      pos+=dir*(max/60);
      if(pos>=max){pos=max;dir=-1;} else if(pos<=0){pos=0;dir=1;}
      rail.ePos=pos;
      if($('ea-pos')) $('ea-pos').value=Math.round(pos);
      rebuildRailMeshes();
    };
    if(rail._simInterval) clearInterval(rail._simInterval);
    rail._simInterval=setInterval(step,16);
    setTimeout(()=>{ clearInterval(rail._simInterval); delete rail._simInterval; },3000);
  });
}

$('extAxesBtn')?.addEventListener('click',()=>{ renderExtAxesModal(); $('extAxesModal').style.display='flex'; });
$('extAxesClose')?.addEventListener('click',()=>{ $('extAxesModal').style.display='none'; });



// effAddBtn/umfAddBtn jetzt mit onclick in HTML — Fallback via JS:
$('effAddBtn')?.addEventListener('click', () => openEffModal(-1));
$('umfAddBtn')?.addEventListener('click', () => openUmfModal(-1));

// Legacy effStlInput (hidden, kept for compat)
$('effStlInput')?.addEventListener('change', async e => {
  let file = e.target.files[0]; if (!file) return;
  try {
    const { buf, name: fname } = await readStlFile(file);
    const effIdx = parseInt($('effStlInput').dataset.effIdx ?? state.activeEff ?? '0');
    const teilIdx = $('effStlInput').dataset.teilIdx !== '' ? parseInt($('effStlInput').dataset.teilIdx) : NaN;
    if (!isNaN(teilIdx) && teilIdx >= 0) {
      const t = (state.effektoren[effIdx]?.teile||[])[teilIdx];
      if (t) { t.stlFile = { path: fname, name: fname, buf }; rebuildEffMesh(effIdx); applyTransforms(); }
    } else if (effIdx >= 0 && effIdx < (state.effektoren||[]).length) {
      state.effektoren[effIdx].stlFile = { path: fname, name: fname, buf };
      rebuildEffMesh(effIdx); applyTransforms();
    }
  } catch(er) { alert('Fehler: '+er.message); }
  renderEffRow(); e.target.value = '';
});

// Legacy umfStlInput (hidden, kept for compat)
$('umfStlInput')?.addEventListener('change', async e => {
  const rawFiles = Array.from(e.target.files);
  if (!state.umfElemente) state.umfElemente = [];
  for (const file of rawFiles) {
    try {
      const { buf, name: fname } = await readStlFile(file);
      const umfIdx = parseInt($('umfStlInput').dataset.umfIdx ?? '-1');
      if (umfIdx >= 0 && umfIdx < state.umfElemente.length) {
        state.umfElemente[umfIdx].stlFile = { path: fname, name: fname, buf };
      } else {
        state.umfElemente.push({ stlFile:{path:fname,name:fname,buf}, offset:{x:0,y:0,z:0,rx:0,ry:0,rz:0} });
      }
    } catch(er) { alert('Fehler: '+er.message); }
  }
  renderUmfRows(); e.target.value = '';
});

// ── Endeffektor & Umfeld ────────────────────────────────────────
const ROBLIB_API = 'https://www.cnc-technik.de/robsimul/roblib/api.php';

let _libTypeFilter = 'all';

function filterLibItems(items) {
  if (_libTypeFilter === 'all') return items;
  // label stored as 'object' on server; fixture stored as 'fixture'
  if (_libTypeFilter === 'label') return items.filter(r => (r.type||'robot') === 'object');
  return items.filter(r => (r.type||'robot') === _libTypeFilter);
}

function renderLibList(items) {
  window._libAllItems = items;
  const filtered = filterLibItems(items);
  const container = $('rl-lib-list');
  if (!container) return;
  const TYPE_ICON = {robot:'🦾',endeffektor:'🔧',umfeld:'🏭',positioner:'🔄',object:'🧱',label:'📦',fixture:'🧱',station:'🏗️',rail:'🛤️'};
  if (!filtered.length) {
    container.innerHTML = '<div style="padding:16px;font-family:monospace;font-size:11px;color:#4a6a8a">Keine Einträge.</div>';
    return;
  }
  container.innerHTML = filtered.map((r) => {
    const thumb = r.thumb_url ? `<img src="${r.thumb_url}" style="width:44px;height:44px;object-fit:cover;border-radius:4px;flex-shrink:0;background:#0f1e2e" onerror="this.style.display='none'">` : `<span style="font-size:20px;width:44px;text-align:center;flex-shrink:0">${TYPE_ICON[r.type||'robot']||'📦'}</span>`;
    return `
    <div data-lib-ri="${items.indexOf(r)}" style="display:flex;align-items:center;gap:10px;padding:8px 12px;border-bottom:1px solid rgba(255,255,255,.05);cursor:pointer" class="lib-row">
      ${thumb}
      <div style="flex:1;min-width:0">
        <div style="font-family:monospace;font-size:12px;color:#d8e8f0;white-space:nowrap;overflow:hidden;text-overflow:ellipsis">${r.name}</div>
        <div style="font-size:10px;color:#4a6a8a">${r.marke||''} ${r.modell||''} <span style="color:#2a5a7a">${r.type||'robot'}</span></div>
      </div>
      <button data-lib-load="${items.indexOf(r)}" style="font-size:10px;padding:2px 8px;background:rgba(37,99,235,.2);border:1px solid rgba(37,99,235,.4);color:#60a5fa;border-radius:3px;cursor:pointer;font-family:monospace">Laden</button>
    </div>`;
  }).join('');
  container.querySelectorAll('[data-lib-load]').forEach(btn => {
    btn.onclick = e => { e.stopPropagation(); loadFromLib(items[+btn.dataset.libLoad]); };
  });
  container.querySelectorAll('.lib-row').forEach(row => {
    row.onclick = e => { if (!e.target.closest('button')) loadFromLib(items[+row.dataset.libRi]); };
  });
}

function openNewItemModal(type) {
  _lastLibRobot = null;
  const t = $('rl-type'); if(t) { t.value = type; }
  if (typeof rlTypeChanged === 'function') rlTypeChanged();
  $('rl-name').value = '';
  const btnU = $('rl-mode-update'); if(btnU) btnU.style.display='none';
  $('rl-msg').style.display='none';
  try {
    renderer.render(scene, camera);
    renderer.domElement.toBlob(blob => {
      if (!blob) return;
      const file = new File([blob],'screenshot.jpg',{type:'image/jpeg'});
      const dt = new DataTransfer(); dt.items.add(file);
      $('rl-thumb').files = dt.files;
      $('rl-thumb-preview').src = URL.createObjectURL(blob);
      $('rl-thumb-preview').style.display = 'block';
    }, 'image/jpeg', 0.92);
  } catch(e) {}
  $('roblibModal').style.display = 'flex';
}

function openRoblibModal() {
  $('rl-msg').style.display = 'none';
  const r = _lastLibRobot;
  if (r) {
    const set = (id, val) => { const el=$(id); if(el) el.value = val||''; };
    set('rl-name',       r.name);
    set('rl-type',       r.type || 'robot');
    if (typeof rlTypeChanged === 'function') rlTypeChanged();
    set('rl-marke',      r.marke);
    set('rl-modell',     r.modell);
    set('rl-achsen',     r.achsen || 6);
    set('rl-reichweite', r.reichweite_mm);
    set('rl-nutzlast',   r.nutzlast_kg);
    set('rl-gewicht',    r.gewicht_kg);
    set('rl-wdh',        r.wiederholgenauigkeit_mm);
    set('rl-rail-length', r.length_mm || 2000);
    set('rl-rail-height', r.height_mm || 200);
    set('rl-rail-width',  r.width_mm  || 400);
    if (r.axis && $('rl-rail-axis')) $('rl-rail-axis').value = r.axis;
    const btnU = $('rl-mode-update');
    if (btnU) btnU.style.display = '';
  } else {
    // Auto-detect type based on scene content
    const hasRobot    = (state.joints||[]).length > 0;
    const hasRail     = (state.schienen||[]).length > 0;
    const hasPos      = (state.positioners||[]).length > 0;
    const hasLabels   = (state.objekte||[]).length > 0;
    const hasFixture  = (state.festeObjekte||[]).length > 0;
    const hasEff      = (state.effektoren||[]).length > 0;
    const hasUmf      = (state.umfElemente||[]).length > 0;
    const typeCount   = [hasRobot, hasRail, hasPos, hasLabels, hasFixture, hasEff, hasUmf].filter(Boolean).length;

    let autoType = 'robot';
    if (typeCount > 1) {
      autoType = 'station';
    } else if (hasFixture)  autoType = 'fixture';
    else if (hasRail)       autoType = 'rail';
    else if (hasPos)        autoType = 'positioner';
    else if (hasLabels)     autoType = 'label';
    else if (hasEff)        autoType = 'endeffektor';
    else if (hasUmf)        autoType = 'umfeld';
    else if (hasRobot)      autoType = 'robot';
    const typeEl = $('rl-type'); if(typeEl) typeEl.value = autoType;
    // Pre-fill name from the right component
    const autoName =
      autoType === 'fixture'    ? (state.festeObjekte[0]?.name || '') :
      autoType === 'rail'       ? (state.schienen[0]?.name || '') :
      autoType === 'positioner' ? (state.positioners[0]?.name || '') :
      autoType === 'label'      ? (state.objekte[0]?.name || '') :
      autoType === 'endeffektor'? (state.effektoren[0]?.name || '') :
      autoType === 'station'    ? '' :
      (state.robotName || '');
    $('rl-name').value = autoName;
    $('rl-achsen').value = 6;
    const btnU = $('rl-mode-update');
    if (btnU) btnU.style.display = 'none';
    if (typeof rlTypeChanged === 'function') rlTypeChanged();
  }

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

// Type change handler
function rlTypeChanged() {
  const type = $('rl-type')?.value || 'robot';
  // Robot fields
  const rf=$('rl-robot-fields');
  if(rf) rf.querySelectorAll('label').forEach(l=>l.style.display=type==='robot'?'':'none');
  // Rail fields
  const raf=$('rl-rail-fields'); if(raf) raf.style.display=type==='rail'?'contents':'none';
  // Station fields
  const sf=$('rl-station-fields'); if(sf) sf.style.display=type==='station'?'':'none';
  if(type==='station') _buildStationChecks();
  // Component selector
  const cs=$('rl-comp-select'); if(cs) cs.style.display=['positioner','label','fixture'].includes(type)?'':'none';
  if(type==='positioner') _fillCompSelect(state.positioners||[], p=>`E${p.eNum||'?'} — ${p.name||''}`);
  if(type==='label')      _fillCompSelect(state.objekte||[],    o=>`Label${o.labelNum||'?'} — ${o.name||''}`);
  if(type==='fixture')    _fillCompSelect(state.festeObjekte||[],o=>o.name||'Objekt');
  // Placeholder
  const ph={robot:'KR 8 R1420', rail:'Linear Track', positioner:'Drehtisch', label:'Palette', fixture:'Tisch', endeffektor:'Greifer', umfeld:'Zelle', station:'Schweißzelle'};
  const nf=$('rl-name'); if(nf&&!nf.value) nf.placeholder=ph[type]||'Name';
}
window.rlTypeChanged=rlTypeChanged;

function _fillCompSelect(arr, labelFn) {
  const sel=$('rl-comp-idx'); if(!sel) return;
  sel.innerHTML=arr.map((o,i)=>`<option value="${i}">${labelFn(o)}</option>`).join('');
}

function _buildStationChecks() {
  const el=$('rl-station-checks'); if(!el) return;
  const items=[];
  if((state.joints||[]).length) items.push({key:'robot', label:`🦾 Roboter — ${state.robotName||'geladen'}`});
  if((state.schienen||[]).length) items.push({key:'rail', label:`🛤️ Rail — ${state.schienen[0].name||''}`});
  (state.positioners||[]).forEach((p,i)=>items.push({key:`pos_${i}`, label:`🔄 E${p.eNum||'?'} ${p.name||''}`}));
  (state.objekte||[]).forEach((o,i)=>items.push({key:`lbl_${i}`, label:`📦 Label${o.labelNum||'?'} ${o.name||''}`}));
  (state.festeObjekte||[]).forEach((o,i)=>items.push({key:`fix_${i}`, label:`🧱 ${o.name||'Festes Obj.'}`}));
  (state.effektoren||[]).forEach((e,i)=>items.push({key:`eff_${i}`, label:`🔧 Endeffektor ${i+1}`}));
  (state.umfElemente||[]).forEach((u,i)=>items.push({key:`umf_${i}`, label:`🏭 Umgebung ${i+1}`}));
  el.innerHTML=items.map(({key,label})=>`<label style="display:flex;align-items:center;gap:6px;cursor:pointer"><input type="checkbox" data-station-key="${key}" checked style="accent-color:var(--acc)"> ${label}</label>`).join('');
}
$('rl-type')?.addEventListener('change', rlTypeChanged);

async function loadStlBufsIntoState(stlBufs) {
  // Add raw buffers to state.buffers so loadStls() can pick them up
  for (const [key, buf] of Object.entries(stlBufs)) {
    const fname = key + '.stl';
    state.buffers.set(fname, buf);
    if (!state.files.find(f=>f.name===fname)) state.files.push({path:fname,name:fname,size:buf.byteLength,type:'STL'});
  }
  splitFiles();
  await loadStls();
}
function _addStlsToZip(zip, axisKeys) {
  // Adds STL buffers from state.axisStlParts to zip under stl/
  for(const key of axisKeys){
    const parts=state.axisStlParts[key]||[];
    parts.forEach((p,pi)=>{ if(p.buf) zip.file(`stl/${key}${pi>0?'_'+pi:''}.stl`, p.buf); });
  }
}

function buildRailJson(idx=0) {
  const r=state.schienen[idx]; if(!r) return null;
  const eAx='E'+(r.eNumber||1);
  const parts=(state.axisStlParts[eAx]||[]).map(p=>({name:p.name||eAx+'.stl',color:p.color||'#2563eb'}));
  return { type:'rail', name:r.name||'Rail', length_mm:r.length_mm||2000, height_mm:r.height_mm||200,
    width_mm:r.width_mm||400, axis:r.axis||'X+', eNumber:r.eNumber||1,
    eMin:r.eMin??0, eMax:r.eMax??r.length_mm??2000, showBox:r.showBox!==false,
    boxOffset:r.boxOffset||{}, stlFiles:parts.length?{[eAx]:parts}:undefined };
}

function buildPositionerJson(idx) {
  const p=state.positioners[idx]; if(!p) return null;
  const eAx='E'+(p.eNum||idx+2);
  const parts=(state.axisStlParts[eAx]||[]).map(pt=>({name:pt.name||eAx+'.stl',color:pt.color||p.color||'#e8a020'}));
  return { type:'positioner', name:p.name||'Positionierer', objectType:p.type||'cylinder',
    eNum:p.eNum||idx+2, rotAxis:p.rotAxis||'Y+', pivotX:p.pivotX||0, pivotY:p.pivotY||0, pivotZ:p.pivotZ||0,
    eMin:p.eMin??-180, eMax:p.eMax??180, ePos:p.ePos||0,
    radius:p.radius||300, length:p.length||500, width:p.width||500, height:p.height||100,
    showBox:p.showBox!==false, color:p.color||'#e8a020', parentIdx:p.parentIdx??-1,
    boxOffset:p.boxOffset||{}, stlFiles:parts.length?{[eAx]:parts}:undefined };
}

function buildLabelJson(idx) {
  const o=state.objekte[idx]; if(!o) return null;
  const lbl='Label'+(o.labelNum||idx+1);
  const parts=(state.axisStlParts[lbl]||[]).map(p=>({name:p.name||lbl+'.stl',color:p.color||o.color||'#4499cc'}));
  return { type:'label', name:o.name||'Label', objectType:o.type||'box', labelNum:o.labelNum||idx+1,
    mountMode:o.mountMode||'world',
    length:o.length||500, width:o.width||500, height:o.height||500, radius:o.radius||200,
    axis:o.axis||'Y+', eMin:o.eMin||0, eMax:o.eMax||1000, ePos:o.ePos||0,
    showBox:o.showBox!==false, color:o.color||'#4499cc', boxOffset:o.boxOffset||{},
    stlFiles:parts.length?{[lbl]:parts}:undefined };
}

function buildFixtureJson(idx) {
  const o=state.festeObjekte[idx]; if(!o) return null;
  return { type:'fixture', name:o.name||'Objekt', objectType:o.type||'box',
    length:o.length||500, width:o.width||500, height:o.height||500, radius:o.radius||200,
    color:o.color||'#607080', x:o.x||0, y:o.y||0, z:o.z||0,
    rx:o.rx||0, ry:o.ry||0, rz:o.rz||0, showBox:o.showBox!==false,
    stlFile: o.stlFile ? `stl/fix_${idx}.stl` : undefined };
}

function buildEffectorJson(idx) {
  const e=(state.effektoren||[])[idx]; if(!e) return null;
  return { type:'endeffektor', name:e.name||'Endeffektor', mountMode:e.mountMode||'world',
    stlFile:e.stlFile?`stl/eff_${idx}.stl`:undefined, offset:e.offset||{} };
}

function buildEnvironmentJson(idx) {
  const u=(state.umfElemente||[])[idx]; if(!u) return null;
  return { type:'umfeld', name:u.name||'Umgebung', stlFile:`stl/umf_${idx}.stl`, offset:u.offset||{} };
}

async function buildComponentZip(type, idx) {
  const zip = new JSZip();
  let cfg = null;
  if(type==='robot'){
    const prevMode=toolMountMode; if(prevMode!=='world'){detachToolFromA6();scene.updateMatrixWorld(true);}
    cfg = buildJson();
    for(const [,mesh] of meshes) zip.file(`stl/${mesh.name}`,exportBinaryStl(mesh));
    state.effektoren.forEach((e,i)=>{ if(e.stlFile?.buf) zip.file(`stl/eff_${i}.stl`,e.stlFile.buf); });
    if(prevMode!=='world') attachToolToA6();
  } else if(type==='rail'){
    cfg=buildRailJson(0);
    if(state.schienen[0]) _addStlsToZip(zip,['E'+(state.schienen[0].eNumber||1)]);
  } else if(type==='positioner'){
    const all=(state.positioners||[]).map((p,i)=>{ const j=buildPositionerJson(i); if(j) _addStlsToZip(zip,['E'+(p.eNum||i+2)]); return j; }).filter(Boolean);
    cfg = all.length===1 ? all[0] : {type:'positioner', items:all};
  } else if(type==='label'){
    const all=(state.objekte||[]).map((o,i)=>{ const j=buildLabelJson(i); if(j) _addStlsToZip(zip,['Label'+(o.labelNum||i+1)]); return j; }).filter(Boolean);
    cfg = all.length===1 ? all[0] : {type:'label', items:all};
  } else if(type==='fixture'){
    const o=state.festeObjekte[idx]; if(!o) return null;
    const j=buildFixtureJson(idx);
    if(j && o.stlFile?.buf) zip.file('stl/fix_0.stl', o.stlFile.buf);
    // Pfad normalisieren: immer fix_0 im ZIP, egal welcher Index
    if(j && j.stlFile) j.stlFile = 'stl/fix_0.stl';
    cfg = j;
  } else if(type==='endeffektor'){
    const all=(state.effektoren||[]).map((e,i)=>{ const j=buildEffectorJson(i); if(j&&e?.stlFile?.buf) zip.file(`stl/eff_${i}.stl`,e.stlFile.buf); return j; }).filter(Boolean);
    cfg = all.length===1 ? all[0] : {type:'endeffektor', items:all};
  } else if(type==='umfeld'){
    const all=(state.umfElemente||[]).map((u,i)=>{ const j=buildEnvironmentJson(i); if(j&&u?.stlFile?.buf) zip.file(`stl/umf_${i}.stl`,u.stlFile.buf); return j; }).filter(Boolean);
    cfg = all.length===1 ? all[0] : {type:'umfeld', items:all};
  } else if(type==='station'){
    cfg = await buildStationJson(zip);
  }
  if(!cfg) return null;
  zip.file('config.json', JSON.stringify(cfg, null, 2));
  return zip;
}

async function buildStationJson(zip) {
  // Collect selected station components
  const checks=[...$('rl-station-checks')?.querySelectorAll('[data-station-key]')||[]];
  const selected=new Set(checks.filter(c=>c.checked).map(c=>c.dataset.stationKey));
  const cfg = { type:'station', name:$('rl-name').value.trim()||'Station', components:{} };

  if(selected.has('robot') && (state.joints||[]).length){
    const prevMode=toolMountMode; if(prevMode!=='world'){detachToolFromA6();scene.updateMatrixWorld(true);}
    cfg.components.robot=buildJson();
    for(const [,mesh] of meshes) zip.file(`stl/${mesh.name}`,exportBinaryStl(mesh));
    if(prevMode!=='world') attachToolToA6();
  }
  if(selected.has('rail') && state.schienen[0]){
    cfg.components.rail=buildRailJson(0);
    _addStlsToZip(zip,['E'+(state.schienen[0].eNumber||1)]);
  }
  cfg.components.positioners=[];
  (state.positioners||[]).forEach((p,i)=>{
    if(!selected.has(`pos_${i}`)) return;
    const pj=buildPositionerJson(i); if(!pj) return;
    cfg.components.positioners.push(pj);
    _addStlsToZip(zip,['E'+(p.eNum||i+2)]);
  });
  cfg.components.labels=[];
  (state.objekte||[]).forEach((o,i)=>{
    if(!selected.has(`lbl_${i}`)) return;
    const lj=buildLabelJson(i); if(!lj) return;
    cfg.components.labels.push(lj);
    _addStlsToZip(zip,['Label'+(o.labelNum||i+1)]);
  });
  cfg.components.fixtures=[];
  (state.festeObjekte||[]).forEach((o,i)=>{
    if(!selected.has(`fix_${i}`)) return;
    const fj=buildFixtureJson(i); if(fj) cfg.components.fixtures.push(fj);
  });
  cfg.components.effectors=[];
  (state.effektoren||[]).forEach((e,i)=>{
    if(!selected.has(`eff_${i}`)) return;
    const ej=buildEffectorJson(i); if(!ej) return;
    cfg.components.effectors.push(ej);
    if(e?.stlFile?.buf) zip.file(`stl/eff_${i}.stl`,e.stlFile.buf);
  });
  cfg.components.environment=[];
  (state.umfElemente||[]).forEach((u,i)=>{
    if(!selected.has(`umf_${i}`)) return;
    const uj=buildEnvironmentJson(i); if(!uj) return;
    cfg.components.environment.push(uj);
    if(u?.stlFile?.buf) zip.file(`stl/umf_${i}.stl`,u.stlFile.buf);
  });
  return cfg;
}

async function updateRoblib() {
  if (!_lastLibRobot) { alert('Kein Library-Eintrag geladen.'); return; }
  const btn  = $('rl-submit');
  const msg  = $('rl-msg');
  const wrap = $('rl-progress-wrap');
  const bar  = $('rl-progress-bar');
  const lbl  = $('rl-progress-label');
  const pct  = $('rl-progress-pct');
  const show = (text, ok) => { msg.textContent=text; msg.className='rl-msg '+(ok?'rl-ok':'rl-err'); msg.style.display=''; wrap.style.display='none'; };
  const setProgress = (label, percent) => { wrap.style.display=''; msg.style.display='none'; lbl.textContent=label; pct.textContent=Math.round(percent)+'%'; bar.style.width=percent+'%'; bar.style.background=percent===100?'#22c55e':'#2563eb'; };
  const type = $('rl-type')?.value||'robot';
  const fields = { id: _lastLibRobot.id, name: $('rl-name').value.trim(), type };
  if (type === 'robot') {
    Object.assign(fields, {
      marke: $('rl-marke').value.trim(), modell: $('rl-modell').value.trim(),
      achsen: $('rl-achsen').value.trim(), reichweite_mm: $('rl-reichweite').value.trim(),
      nutzlast_kg: $('rl-nutzlast').value.trim(), gewicht_kg: $('rl-gewicht').value.trim(),
      wiederholgenauigkeit_mm: $('rl-wdh').value.trim(),
    });
  }
  if (!fields.id) { show('Keine ID vorhanden.', false); return; }
  btn.disabled = true; btn.textContent = 'Aktualisiere…';
  try {
    // ── Rail ─────────────────────────────────────────────────────
    if (type === 'rail') {
      const railData = {
        name: fields.name, type: 'rail',
        length_mm: parseFloat($('rl-rail-length')?.value)||2000,
        height_mm: parseFloat($('rl-rail-height')?.value)||200,
        width_mm:  parseFloat($('rl-rail-width')?.value)||400,
        axis: $('rl-rail-axis')?.value||'X+'
      };
      setProgress('Erstelle ZIP…', 5);
      const zip2 = new JSZip();
      const base2 = zipName(fields.name || 'rail');
      zip2.file(base2+'.json', JSON.stringify(railData, null, 2));
      const zipBlob2 = await zip2.generateAsync({type:'blob'}, m => setProgress('Komprimiere…', 5+m.percent*0.4));
      setProgress('Lade hoch…', 45);
      const fd2 = new FormData();
      for (const [k,v] of Object.entries(fields)) fd2.append(k,v);
      fd2.append('zip', zipBlob2, base2+'.zip');
      const thumb2 = $('rl-thumb').files[0];
      if (thumb2) fd2.append('thumb', thumb2, thumb2.name);
      const res2 = await new Promise((resolve, reject) => {
        const xhr = new XMLHttpRequest();
        xhr.open('POST', ROBLIB_API + '?action=update');
        xhr.upload.onprogress = e => { if(e.lengthComputable) setProgress('Lade hoch…', 45+(e.loaded/e.total)*50); };
        xhr.onload = () => { try { resolve(JSON.parse(xhr.responseText)); } catch(e) { reject(new Error('Ungültige Serverantwort')); } };
        xhr.onerror = () => reject(new Error('Verbindungsfehler'));
        xhr.send(fd2);
      });
      if (res2.ok) { setProgress('Fertig!', 100); setTimeout(() => { show('✓ '+fields.name+' aktualisiert', true); btn.disabled=false; btn.textContent='Aktualisieren'; }, 600); }
      else { show(res2.error||'Fehler.', false); btn.disabled=false; btn.textContent='Aktualisieren'; }
      return;
    }
    setProgress('Erstelle ZIP…', 5);
    const idx = parseInt($('rl-comp-idx')?.value)||0;
    const zip = await buildComponentZip(type, idx);
    if (!zip) { show('Keine Daten für diesen Typ.', false); btn.disabled=false; btn.textContent='Aktualisieren'; return; }
    setProgress('Komprimiere…', 20);
    const base = zipName(fields.name || type);
    const zipBlob = await zip.generateAsync({type:'blob'}, m => setProgress('Komprimiere…', 20+m.percent*0.25));
    setProgress('Lade hoch…', 45);
    const fd = new FormData();
    for (const [k,v] of Object.entries(fields)) fd.append(k, v);
    fd.append('zip', zipBlob, base+'.zip');
    const thumb = $('rl-thumb').files[0];
    if (thumb) fd.append('thumb', thumb, thumb.name);
    const data = await new Promise((resolve, reject) => {
      const xhr = new XMLHttpRequest();
      xhr.open('POST', ROBLIB_API + '?action=update');
      xhr.upload.onprogress = e => { if(e.lengthComputable) setProgress('Lade hoch…', 45+(e.loaded/e.total)*50); };
      xhr.onload = () => { try { resolve(JSON.parse(xhr.responseText)); } catch(e) { reject(new Error('Ungültige Serverantwort')); } };
      xhr.onerror = () => reject(new Error('Verbindungsfehler'));
      xhr.send(fd);
    });
    if (data.ok) {
      setProgress('Fertig!', 100);
      setTimeout(() => { show('✓ ' + fields.name + ' aktualisiert', true); btn.disabled=false; btn.textContent='Aktualisieren'; }, 600);
    } else { show(data.error || 'Fehler beim Aktualisieren.', false); btn.disabled=false; btn.textContent='Aktualisieren'; }
  } catch(e) { show('Fehler: '+e.message, false); btn.disabled=false; btn.textContent='Aktualisieren'; }
}

async function uploadToRoblib() {
  const btn=$('rl-submit'), msg=$('rl-msg'), wrap=$('rl-progress-wrap'), bar=$('rl-progress-bar'), lbl=$('rl-progress-label'), pct=$('rl-progress-pct');
  const show=(text,ok)=>{ msg.textContent=text; msg.className='rl-msg '+(ok?'rl-ok':'rl-err'); msg.style.display=''; wrap.style.display='none'; };
  const setP=(label,percent)=>{ wrap.style.display=''; msg.style.display='none'; lbl.textContent=label; pct.textContent=Math.round(percent)+'%'; bar.style.width=percent+'%'; bar.style.background=percent===100?'#22c55e':'#2563eb'; };
  const type=$('rl-type')?.value||'robot';
  const name=$('rl-name').value.trim();
  if(!name){show('Name fehlt.',false);return;}
  // Map internal types to server category types
  const serverTypeMap = { label:'object', positioner:'positioner', rail:'rail', endeffektor:'endeffektor', umfeld:'umfeld', station:'station', robot:'robot', fixture:'fixture' };
  const serverType = serverTypeMap[type] || type;
  const fields={name, type:serverType};
  if(type==='robot'){
    const missing=['rl-marke','rl-modell','rl-achsen','rl-reichweite','rl-nutzlast','rl-gewicht','rl-wdh'].find(id=>!$(id)?.value.trim());
    if(missing){show('Pflichtfeld fehlt.',false);return;}
    Object.assign(fields,{marke:$('rl-marke').value.trim(),modell:$('rl-modell').value.trim(),
      achsen:$('rl-achsen').value,reichweite_mm:$('rl-reichweite').value,
      nutzlast_kg:$('rl-nutzlast').value,gewicht_kg:$('rl-gewicht').value,wiederholgenauigkeit_mm:$('rl-wdh').value});
  }
  btn.disabled=true; btn.textContent='Lade…';
  try {
    setP('Erstelle ZIP…',5);
    const idx=parseInt($('rl-comp-idx')?.value)||0;
    const zip=await buildComponentZip(type,idx);
    if(!zip){show('Keine Daten für diesen Typ.',false);btn.disabled=false;btn.textContent='Hochladen';return;}
    setP('Komprimiere…',20);
    const base=zipName(name);
    const zipBlob=await zip.generateAsync({type:'blob'},m=>setP('Komprimiere…',20+m.percent*0.25));
    setP('Lade hoch…',45);
    const fd=new FormData();
    for(const [k,v] of Object.entries(fields)) fd.append(k,v);
    fd.append('zip',zipBlob,base+'.zip');
    const thumb=$('rl-thumb').files[0]; if(thumb) fd.append('thumb',thumb,thumb.name);
    const beschr=$('rl-beschreibung')?.value?.trim(); if(beschr) fd.append('beschreibung',beschr);
    const data=await new Promise((res,rej)=>{
      const xhr=new XMLHttpRequest(); xhr.open('POST',ROBLIB_API+'?action=upload');
      xhr.upload.onprogress=e=>{if(e.lengthComputable)setP('Lade hoch…',45+(e.loaded/e.total)*50);};
      xhr.onload=()=>{try{res(JSON.parse(xhr.responseText));}catch(e){rej(new Error('Serverantwort ungültig'));}};
      xhr.onerror=()=>rej(new Error('Verbindungsfehler')); xhr.send(fd);
    });
    if(data.ok){setP('Fertig!',100);setTimeout(()=>show('✓ Hochgeladen: '+name,true),600);}
    else show('Fehler: '+(data.error||'Unbekannt'),false);
  } catch(e){show('Fehler: '+e.message,false);}
  btn.disabled=false; btn.textContent='Hochladen';
}


// ── Roboter Library Modal ─────────────────────────────────────────
let _libRobots = [];

function openRobotLibModal() {
  $('robotLibModal').style.display = 'flex';
  if (!_libRobots.length) loadRobotLibList();
}

async function loadRobotLibList() {
  const status = $('rl-lib-status');
  const listEl = $('rl-lib-list');
  const bar    = $('rl-lib-bar');
  const prog   = $('rl-lib-progress');
  status.textContent = 'Lade…';
  prog.style.display = 'block'; bar.style.width = '30%';
  try {
    const r = await fetch(ROBLIB_API + '?action=list');
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const data = await r.json();
    _libRobots = data.robots || [];
    window._libAllItems = _libRobots;
    bar.style.width = '100%'; prog.style.display = 'none';
    status.textContent = _libRobots.length + ' Einträge';
    renderLibList(_libRobots);
  } catch(e) {
    prog.style.display = 'none';
    status.textContent = 'Fehler: ' + e.message;
  }
}

function renderRobotLibList(query) {
  const q = (query || '').toLowerCase();
  const filtered = q ? _libRobots.filter(r => r.name.toLowerCase().includes(q)||(r.marke||'').toLowerCase().includes(q)) : _libRobots;
  renderLibList(filtered);
}

let _lastLibRobot = null; // zuletzt geladener Library-Eintrag

async function loadRobotFromLib(robot) {
  _lastLibRobot = robot;
  const status = $('rl-lib-status');
  const bar    = $('rl-lib-bar');
  const prog   = $('rl-lib-progress');
  status.textContent = 'Lade ' + robot.name + '…';
  prog.style.display = 'block'; bar.style.width = '10%';
  try {
    const res = await fetch(robot.zip_url);
    if (!res.ok) throw new Error('Download fehlgeschlagen: HTTP ' + res.status);
    bar.style.width = '40%';
    const zip = await JSZip.loadAsync(await res.arrayBuffer());
    bar.style.width = '65%';

    // Check if this is a non-robot component (new format with config.json, or type field in JSON)
    const jsonEntry = Object.keys(zip.files).find(n => !zip.files[n].dir && /\.json$/i.test(n));
    if (jsonEntry) {
      let cfg;
      try { cfg = JSON.parse(await zip.files[jsonEntry].async('string')); } catch(e) {}
      if (cfg?.type && cfg.type !== 'robot') {
        // Redirect to component loader
        await loadComponentFromZip(zip);
        bar.style.width = '100%'; prog.style.display = 'none';
        status.textContent = '✓ ' + robot.name + ' geladen';
        setTimeout(() => { $('robotLibModal').style.display = 'none'; }, 600);
        return;
      }
    }

    resetData(); state.mode = 'package';

    // JSON (jsonEntry already found above)
    if (jsonEntry) {
      try {
        state.packageJson = JSON.parse(await zip.files[jsonEntry].async('string'));
        applyJsonToState(state.packageJson);
        setJointAnglesToReferencePose();
      } catch(e) { console.warn('JSON parse error', e); }
    }

    // STLs + OSD Dateien
    for (const name of Object.keys(zip.files)) {
      if (zip.files[name].dir || !/\.(stl|osd)$/i.test(name)) continue;
      let buf = await zip.files[name].async('uint8array');
      if (/\.osd$/i.test(name)) buf = new Uint8Array(osdToBinaryStl(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength)));
      const fname = name.split('/').pop().replace(/\.osd$/i, '.stl');
      state.buffers.set(fname, buf);
      state.files.push({ path: fname, name: fname, size: buf.byteLength, type: 'STL' });
    }
    splitFiles();

    // Effektoren-STLs: stlName → buffer aus ZIP
    state.effektoren.forEach((eff) => {
      if (!eff.stlFile && eff.stlName) {
        const fname = eff.stlName.split('/').pop();
        const buf = state.buffers.get(fname);
        if (buf) eff.stlFile = { path: fname, name: eff.name || fname, buf };
      }
    });

    // Achsfarben + STL-Zuweisung aus JSON
    if (state.packageJson?.stlFiles) {
      Object.entries(state.packageJson.stlFiles).forEach(([ax, info]) => {
        if (!ax.match(/^A[1-6]$/)) return;
        const parts = Array.isArray(info) ? info : [info];
        state.axisStlParts[ax] = [];
        parts.forEach(p => {
          const stlName=(p.name||'').replace(/\.(stl|osd)$/i,'')+'.stl';
          const buf=state.buffers.get(stlName);
          state.axisStlParts[ax].push({name:stlName,color:p.color||'#e8a020',buf:buf||null});
          if (!state.axisStlMap[ax]&&(buf||state.stls.find(f=>f.name===stlName))) state.axisStlMap[ax]=stlName;
          if (p.color) colors[ax]=p.color;
        });
      });
    }

    bar.style.width = '85%';
    zeroAllTransforms();
    await loadStls(); enableSave(); renderAll(); setView('iso');
    bar.style.width = '100%'; prog.style.display = 'none';
    status.textContent = '✓ ' + robot.name + ' geladen';
    $('robotLibModal').style.display = 'none';
  } catch(e) {
    prog.style.display = 'none';
    status.textContent = 'Fehler: ' + e.message;
  }
}

// ── Universeller Library-Loader ───────────────────────────────────
async function loadFromLib(item) {
  const type = item.type === 'object' ? 'fixture' : (item.type || 'robot');
  if (type === 'robot') { loadRobotFromLib(item); return; }
  const status = $('rl-lib-status');
  const prog   = $('rl-lib-progress');
  const bar    = $('rl-lib-bar');
  status.textContent = 'Lade ' + item.name + '…';
  prog.style.display = 'block'; bar.style.width = '10%';
  try {
    const res = await fetch(item.zip_url);
    if (!res.ok) throw new Error('HTTP ' + res.status);
    bar.style.width = '40%';
    const zip = await JSZip.loadAsync(await res.arrayBuffer());
    bar.style.width = '70%';
    await loadComponentFromZip(zip);
    bar.style.width = '100%'; prog.style.display = 'none';
    status.textContent = '✓ ' + item.name + ' geladen';
    _lastLibRobot = item;
    setTimeout(() => { $('robotLibModal').style.display = 'none'; }, 600);
  } catch(e) {
    prog.style.display = 'none';
    status.textContent = 'Fehler: ' + e.message;
  }
}

// Liest config.json aus ZIP und dispatcht an den richtigen Loader
async function loadComponentFromZip(zip) {
  // Try config.json first (new format)
  let cfgEntry = zip.files['config.json'];
  let cfg;

  if (cfgEntry) {
    cfg = JSON.parse(await cfgEntry.async('string'));
  } else {
    // Fallback: find any *.json file (old format)
    const jsonKey = Object.keys(zip.files).find(n => !zip.files[n].dir && /\.json$/i.test(n));
    if (jsonKey) {
      cfg = JSON.parse(await zip.files[jsonKey].async('string'));
      // Old rail format had no explicit type field — detect by content
      if (!cfg.type) {
        if (cfg.length_mm !== undefined || cfg.axis !== undefined) cfg.type = 'rail';
        else if (cfg.joints?.length) cfg.type = 'robot';
        else cfg.type = 'fixture';
      }
      if (cfg.type === 'object') cfg.type = 'fixture';
    } else {
      // No JSON at all — check if there's a bare STL → treat as fixture
      const stlKey = Object.keys(zip.files).find(n => !zip.files[n].dir && /\.stl$/i.test(n));
      if (!stlKey) throw new Error('Keine JSON-Konfiguration in ZIP gefunden');
      const fname = stlKey.split('/').pop().replace(/\.stl$/i,'');
      cfg = { type: 'fixture', name: fname, objectType: 'stl', showBox: false,
              x:0, y:0, z:0, rx:0, ry:0, rz:0, stlFile: stlKey };
    }
  }

  // Collect all STL/OSD buffers: stl/XX.stl|osd → key without extension
  const stlBufs = {};
  for (const name of Object.keys(zip.files)) {
    if (zip.files[name].dir || !/\.(stl|osd)$/i.test(name)) continue;
    const key = name.split('/').pop().replace(/\.(stl|osd)$/i,'');
    let raw = await zip.files[name].async('uint8array');
    if (/\.osd$/i.test(name)) raw = new Uint8Array(osdToBinaryStl(raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.byteLength)));
    stlBufs[key] = raw;
  }

  const type = cfg.type;
  // Clear only same-type components before loading
  if (type === 'fixture') {
    (festeGrps||[]).forEach(g=>{if(g?.parent) g.parent.remove(g);}); festeGrps.length=0; state.festeObjekte=[];
  } else if (type === 'rail') {
    if(railGroup) while(railGroup.children.length) railGroup.remove(railGroup.children[0]);
    state.schienen=[];
  } else if (type === 'positioner') {
    // append, don't clear
  } else if (type === 'label') {
    // append, don't clear
  }
  if      (type === 'rail')        applyRailConfig(cfg, stlBufs);
  else if (type === 'positioner')  {
    const items = cfg.items || [cfg];
    items.forEach(p => applyPositionerConfig(p, stlBufs));
  }
  else if (type === 'label')       {
    const items = cfg.items || [cfg];
    items.forEach(l => applyLabelConfig(l, stlBufs));
  }
  else if (type === 'fixture')     {
    const items = cfg.items || [cfg];
    items.forEach(f => applyFixtureConfig(f, stlBufs));
  }
  else if (type === 'endeffektor') {
    const items = cfg.items || [cfg];
    items.forEach(e => applyEffectorConfig(e, stlBufs));
  }
  else if (type === 'umfeld')      {
    const items = cfg.items || [cfg];
    items.forEach(u => applyEnvironmentConfig(u, stlBufs));
  }
  else if (type === 'station')     applyStationConfig(cfg, stlBufs);
  else if (type === 'robot')       {
    applyJsonToState(cfg);
    // Rotation VOR loadStlBufsIntoState setzen — loadStls() liest rRx/rRy/rRz am Anfang
    const _setV = function(id,v){var el=document.getElementById(id);if(el)el.value=v;};
    _setV('rRx',90); _setV('rRy',0); _setV('rRz',-90);
    await loadStlBufsIntoState(stlBufs);
    rebuildRobotKinematics(); applyTransforms(); enableSave();
  }
  else throw new Error('Unbekannter Typ: ' + type);
  renderAll(); setView('iso');
}

function _setStlParts(axKey, stlBufs, fallbackColor) {
  const buf = stlBufs[axKey];
  if (buf) {
    state.axisStlParts[axKey] = [{
      name: axKey + '.stl', color: fallbackColor || '#4499cc', buf
    }];
  }
}

function applyRailConfig(cfg, stlBufs) {
  const eAx = 'E' + (cfg.eNumber || 1);
  const entry = {
    name: cfg.name || 'Rail', length_mm: cfg.length_mm || 2000,
    height_mm: cfg.height_mm || 200, width_mm: cfg.width_mm || 400,
    axis: cfg.axis || 'X+', eNumber: cfg.eNumber || 1,
    eMin: cfg.eMin ?? 0, eMax: cfg.eMax ?? cfg.length_mm ?? 2000,
    ePos: 0, showBox: cfg.showBox !== false, boxOffset: cfg.boxOffset || {}
  };
  // Apply STL parts from config stlFiles or raw stlBufs
  if (cfg.stlFiles?.[eAx]) {
    const parts = Array.isArray(cfg.stlFiles[eAx]) ? cfg.stlFiles[eAx] : [cfg.stlFiles[eAx]];
    state.axisStlParts[eAx] = parts.map((p,i) => {
      const key = (p.name||eAx).replace(/\.stl$/i,'');
      return { name: key+'.stl', color: p.color||'#2563eb', buf: stlBufs[key]||stlBufs[eAx]||null };
    });
  } else { _setStlParts(eAx, stlBufs, '#2563eb'); }
  state.schienen = [entry];
  rebuildRailMeshes();
  renderRailRows(); renderRows();
}

function applyPositionerConfig(cfg, stlBufs) {
  const eAx = 'E' + (cfg.eNum || 2);
  const entry = {
    name: cfg.name||'Positionierer', type: cfg.objectType||cfg.type_||'cylinder',
    eNum: cfg.eNum||2, rotAxis: cfg.rotAxis||'Y+',
    pivotX: cfg.pivotX||0, pivotY: cfg.pivotY||0, pivotZ: cfg.pivotZ||0,
    eMin: cfg.eMin??-180, eMax: cfg.eMax??180, ePos: 0,
    radius: cfg.radius||300, length: cfg.length||500, width: cfg.width||500, height: cfg.height||100,
    showBox: cfg.showBox!==false, color: cfg.color||'#e8a020',
    parentIdx: cfg.parentIdx??-1, boxOffset: cfg.boxOffset||{}
  };
  if (cfg.stlFiles?.[eAx]) {
    const parts = Array.isArray(cfg.stlFiles[eAx]) ? cfg.stlFiles[eAx] : [cfg.stlFiles[eAx]];
    state.axisStlParts[eAx] = parts.map(p => {
      const key = (p.name||eAx).replace(/\.stl$/i,'');
      return { name: key+'.stl', color: p.color||cfg.color||'#e8a020', buf: stlBufs[key]||stlBufs[eAx]||null };
    });
  } else { _setStlParts(eAx, stlBufs, cfg.color||'#e8a020'); }
  state.positioners.push(entry);
  positionerGroups.push(null);
  rebuildPositionerMesh(state.positioners.length - 1);
  renderPosRows(); renderRows();
}

function applyLabelConfig(cfg, stlBufs) {
  const lbl = 'Label' + (cfg.labelNum || 1);
  const entry = {
    name: cfg.name||'Label', type: cfg.objectType||'box', labelNum: cfg.labelNum||1,
    length: cfg.length||500, width: cfg.width||500, height: cfg.height||500, radius: cfg.radius||200,
    axis: cfg.axis||'Y+', eMin: cfg.eMin||0, eMax: cfg.eMax||1000, ePos: 0,
    showBox: cfg.showBox!==false, color: cfg.color||'#4499cc', boxOffset: cfg.boxOffset||{},
    mountMode: cfg.mountMode||'world'
  };
  if (cfg.stlFiles?.[lbl]) {
    const parts = Array.isArray(cfg.stlFiles[lbl]) ? cfg.stlFiles[lbl] : [cfg.stlFiles[lbl]];
    state.axisStlParts[lbl] = parts.map(p => {
      const key = (p.name||lbl).replace(/\.stl$/i,'');
      return { name: key+'.stl', color: p.color||cfg.color||'#4499cc', buf: stlBufs[key]||stlBufs[lbl]||null };
    });
  } else { _setStlParts(lbl, stlBufs, cfg.color||'#4499cc'); }
  state.objekte.push(entry);
  objekteGroups.push(null);
  rebuildObjektMesh(state.objekte.length - 1);
  renderObjRows(); renderRows();
}

function applyFixtureConfig(cfg, stlBufs) {
  // STL-Buffer: aus explizitem stlFile-Pfad oder erstem verfügbaren Buffer
  const stlKey = cfg.stlFile ? cfg.stlFile.split('/').pop().replace(/\.stl$/i,'') : null;
  const buf = (stlKey && stlBufs[stlKey]) || stlBufs['fix_0'] || (Object.keys(stlBufs).length===1 ? Object.values(stlBufs)[0] : null);
  const entry = {
    name: cfg.name||'Objekt', type: cfg.objectType||'box',
    length: cfg.length||500, width: cfg.width||500, height: cfg.height||500, radius: cfg.radius||200,
    color: cfg.color||'#607080',
    x: cfg.x||0, y: cfg.y||0, z: cfg.z||0,
    rx: cfg.rx||0, ry: cfg.ry||0, rz: cfg.rz||0,
    showBox: cfg.showBox!==false,
    stlFile: buf ? { name: (cfg.name||'objekt')+'.stl', buf } : null
  };
  state.festeObjekte.push(entry);
  festeGrps.push(null);
  rebuildFixMesh(state.festeObjekte.length - 1);
  renderFixRows();
}

function applyEffectorConfig(cfg, stlBufs) {
  const buf = stlBufs['eff_0'] || Object.values(stlBufs)[0] || null;
  const eff = { name: cfg.name||'Endeffektor', mountMode: cfg.mountMode||'world',
    offset: cfg.offset||{}, stlFile: buf ? { name: cfg.name+'.stl', buf } : null };
  state.effektoren = state.effektoren||[];
  state.effektoren.push(eff);
  renderEffRow?.();
}

function applyEnvironmentConfig(cfg, stlBufs) {
  const buf = stlBufs['umf_0'] || Object.values(stlBufs)[0] || null;
  if (!buf) return;
  const u = { name: cfg.name||'Umgebung', stlFile: { name: cfg.name+'.stl', buf }, offset: cfg.offset||{} };
  state.umfElemente = state.umfElemente||[];
  state.umfElemente.push(u);
  renderUmfRows?.();
}

function applyStationConfig(cfg, stlBufs) {
  const c = cfg.components || {};
  if (c.robot && (c.robot.joints||[]).length) {
    applyJsonToState(c.robot);
    // Robot STLs (A1..A6)
    if (c.robot.stlFiles) {
      Object.entries(c.robot.stlFiles).forEach(([ax, info]) => {
        if (!ax.match(/^A[1-6]$/)) return;
        const parts = Array.isArray(info) ? info : [info];
        state.axisStlParts[ax] = parts.map(p => {
          const key = (p.name||ax).replace(/\.stl$/i,'');
          return { name: key+'.stl', color: p.color||'#e8a020', buf: stlBufs[key]||null };
        });
        if (parts[0]?.color) colors[ax] = parts[0].color;
      });
    }
    rebuildRobotKinematics(); applyTransforms(); enableSave();
  }
  if (c.rail) applyRailConfig(c.rail, stlBufs);
  (c.positioners||[]).forEach(p => applyPositionerConfig(p, stlBufs));
  (c.labels||[]).forEach(l => applyLabelConfig(l, stlBufs));
  (c.fixtures||[]).forEach(f => applyFixtureConfig(f));
  (c.effectors||[]).forEach(e => applyEffectorConfig(e, stlBufs));
  (c.environment||[]).forEach(e => applyEnvironmentConfig(e, stlBufs));
}

$('robotLibBtn').onclick   = openRobotLibModal;

// ── Komponenten-Library Buttons (↻) ──────────────────────────────
function openLibFiltered(type) {
  // Set filter and open Library modal
  _libTypeFilter = type;
  // Update filter tab visual
  document.querySelectorAll('.lib-tab-btn').forEach(btn => {
    const on = btn.dataset.libType === type;
    btn.style.background = on ? 'rgba(37,99,235,.2)' : 'rgba(255,255,255,.05)';
    btn.style.border = on ? '1px solid rgba(37,99,235,.4)' : '1px solid rgba(255,255,255,.15)';
    btn.style.color = on ? '#60a5fa' : '#6a8fa8';
    if(on) btn.classList.add('on'); else btn.classList.remove('on');
  });
  openRobotLibModal();
  if (!_libRobots.length) loadRobotLibList();
  else renderLibList(_libRobots);
}

$('posLibRefreshBtn')?.addEventListener('click', () => openLibFiltered('positioner'));
$('railLibRefreshBtn')?.addEventListener('click', () => openLibFiltered('rail'));
$('objLibRefreshBtn')?.addEventListener('click',  () => openLibFiltered('label'));
$('fixLibRefreshBtn')?.addEventListener('click',  () => openLibFiltered('fixture'));
$('robotLibClose').onclick = () => { $('robotLibModal').style.display = 'none'; };
$('rl-lib-refresh').addEventListener('click', () => { _libRobots = []; loadRobotLibList(); });
$('rl-lib-search').addEventListener('input', e => renderRobotLibList(e.target.value));

// demoBtn removed
$('newBtn')?.addEventListener('click', () => loadDemoKr8().catch(err => alert(err.message)));
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
$('rl-submit').onclick  = () => { $('rl-submit').textContent.includes('Aktualis') ? updateRoblib() : uploadToRoblib(); };
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

// ── ROBLIB Modal Modus ─────────────────────────────────────────────
function setRlMode(mode) {
  const isNew = mode === 'new';
  const btnNew    = $('rl-mode-new');
  const btnUpdate = $('rl-mode-update');
  const submitBtn = $('rl-submit');
  if (btnNew)    { btnNew.style.background    = isNew  ? 'rgba(37,99,235,.3)' : 'rgba(255,255,255,.05)'; btnNew.style.color    = isNew  ? '#60a5fa' : '#6a8fa8'; btnNew.style.border    = isNew  ? '1px solid #2563eb' : '1px solid rgba(255,255,255,.2)'; }
  if (btnUpdate) { btnUpdate.style.background = !isNew ? 'rgba(34,197,94,.3)' : 'rgba(255,255,255,.05)'; btnUpdate.style.color = !isNew ? '#4ade80' : '#6a8fa8'; btnUpdate.style.border = !isNew ? '1px solid #22c55e' : '1px solid rgba(255,255,255,.2)'; }
  if (submitBtn) submitBtn.textContent = isNew ? 'Hochladen' : 'Aktualisieren';
}
window.setRlMode = setRlMode;

// ── Event-Listener ─────────────────────────────────────────────────
$('newBtn').onclick     = () => { resetData(); disableSave(); renderAll(); setView('iso'); };
$('clearAllBtn')?.addEventListener('click', function(e) {
  if (e.ctrlKey) {
    // STRG+Neu: alles außer Skelett (joints + axisPoints) löschen
    if (!confirm('Alles außer Skelett leeren?')) return;
    // Joints und axisPoints sichern — falls leer: Standardskelett erzeugen
    var savedJoints = (state.joints && state.joints.length)
      ? JSON.parse(JSON.stringify(state.joints))
      : ['A1','A2','A3','A4','A5','A6'].map(function(n,i){return {name:n,axis:fixedAxisType(i),offset:defOffset(i),min:null,max:null,rotationSign:1,status:'Standard'};});
    var savedAxisPoints = (state.axisPoints && state.axisPoints.length)
      ? JSON.parse(JSON.stringify(state.axisPoints))
      : savedJoints.map(function(j,i){return Object.assign({name:'A'+(i+1),rx:0,ry:0,rz:0,source:'Standard'},defOffset(i));});
    clearAll._inner(false);
    // Skelett wiederherstellen
    state.joints = savedJoints;
    state.axisPoints = savedAxisPoints;
    // Skelett neu aufbauen
    syncJointsFromAxisPoints();
    rebuildRobotKinematics();
    applyTransforms();
    updateAxisPointVisuals();
    renderRows();
  } else {
    clearAll();
  }
});
$('downloadJson').onclick = downloadJson;
$('downloadZip').onclick  = downloadZip;
$('toggleParam').onclick = () => {
  const footer = $('paramFooter');
  const btn    = $('toggleParam');
  const collapsed = footer.classList.toggle('collapsed');
  btn.textContent = collapsed ? '▲' : '▼';
};
$('resetView').onclick    = () => setView('iso');
$('applyStlRotBtn').onclick = () => {
  // Geometry neu laden mit aktuellen Rotationswerten
  for (const m of meshes.values()) { m.parent?.remove(m); m.geometry?.dispose?.(); }
  meshes.clear();
  loadStls().then(() => { renderRows(); });
};
$('toggleGrid').onclick   = () => grid.visible = !grid.visible;
$('camModeBtn')?.addEventListener('click', toggleCameraMode);

let _axisLabelsVisible = true;
// Robot-Visibility: 0=sichtbar, 1=durchsichtig, 2=unsichtbar
var _robotVisState = 0;

function _setRobotOpacity(grp, opacity) {
  grp.traverse(function(obj) {
    if (!obj.isMesh || !obj.material) return;
    // Material klonen falls noch nicht getan, um andere Objekte nicht zu beeinflussen
    if (!obj._origMaterial) obj._origMaterial = obj.material;
    if (opacity < 1.0) {
      if (obj.material === obj._origMaterial) obj.material = obj._origMaterial.clone();
      obj.material.transparent = true;
      obj.material.opacity     = opacity;
      obj.material.depthWrite  = false;
      obj.material.needsUpdate = true;
    } else {
      // Original-Material wiederherstellen
      if (obj._origMaterial) { obj.material = obj._origMaterial; obj._origMaterial = undefined; }
      obj.material.transparent = false;
      obj.material.opacity     = 1.0;
      obj.material.depthWrite  = true;
      obj.material.needsUpdate = true;
    }
  });
}

$('robotVisBtn')?.addEventListener('click', () => {
  _robotVisState = (_robotVisState + 1) % 3;

  var groups = [];
  if (robotGroup)     groups.push(robotGroup);
  if (kinematicsRoot) groups.push(kinematicsRoot);
  if (toolGroup)      groups.push(toolGroup);
  (effektorGroups||[]).forEach(function(g){ if(g) groups.push(g); });
  if (railGroup)      groups.push(railGroup);
  (positionerGroups||[]).forEach(function(g){ if(g?.containerGrp) groups.push(g.containerGrp); });

  groups.forEach(function(grp) {
    if (_robotVisState === 2) {
      grp.visible = false;
    } else {
      grp.visible = true;
      _setRobotOpacity(grp, _robotVisState === 1 ? 0.25 : 1.0);
    }
  });

  // Button-Style
  var btn = $('robotVisBtn');
  if (btn) {
    var labels = ['🤖 Roboter','👻 Roboter','⬜ Roboter'];
    var bgs    = ['rgba(37,99,235,.2)','rgba(255,165,0,.2)','rgba(255,255,255,.05)'];
    var bcs    = ['rgba(37,99,235,.4)','rgba(255,165,0,.4)','rgba(255,255,255,.15)'];
    var cols   = ['#60a5fa','#fbbf24','#6a8fa8'];
    var tips   = ['Klick: durchsichtig','Klick: ausblenden','Klick: einblenden'];
    btn.textContent   = labels[_robotVisState];
    btn.style.background  = bgs[_robotVisState];
    btn.style.borderColor = bcs[_robotVisState];
    btn.style.color       = cols[_robotVisState];
    btn.title             = tips[_robotVisState];
  }
});

$('axisLabelBtn')?.addEventListener('click', () => {
  _axisLabelsVisible = !_axisLabelsVisible;
  axisPointGroup.traverse(o => { if (o.userData.isAxisLabel) o.visible = _axisLabelsVisible; });
  const btn = $('axisLabelBtn');
  if (btn) {
    btn.style.background = _axisLabelsVisible ? 'rgba(37,99,235,.2)' : 'rgba(255,255,255,.05)';
    btn.style.borderColor = _axisLabelsVisible ? 'rgba(37,99,235,.4)' : 'rgba(255,255,255,.15)';
    btn.style.color = _axisLabelsVisible ? '#60a5fa' : '#6a8fa8';
  }
});
initAxisStlEvents();
// Theme laden + Button
try { const saved = localStorage.getItem('robmodel_theme'); if(saved !== null) applyTheme(parseInt(saved)); } catch(e){}
$('themeBtn').onclick = () => applyTheme(_themeIdx + 1);
// Load ZIP: normal = load, Ctrl+Click label = demo
$('sourceZip').addEventListener('change', e => {
  if (!e.target.files[0]) return;
  const file = e.target.files[0];
  e.target.value = '';
  loadSourceZip(file).catch(err => alert(err.message));
});
// Ctrl+Click on the Load ZIP label triggers demo
document.querySelector('label[for="sourceZip"]').addEventListener('click', e => {
  if (e.ctrlKey) { e.preventDefault(); loadDemoKr8().catch(err => alert(err.message)); }
});
$('sourceFolder').addEventListener('change', e => {
  if (!e.target.files.length) return;
  const files = Array.from(e.target.files);
  e.target.value = '';  // Reset damit gleicher Ordner erneut geladen werden kann
  loadSourceFolder(files).catch(err => alert(err.message));
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
  // axisColor: handled by Enter/blur (hex text input)
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
    (async () => {
      let rawBuf, fname;
      if (/\.zip$/i.test(file.name)) {
        try { const r = await extractFromZip(file); rawBuf = r.buf; fname = r.name; }
        catch(er) { alert(er.message); t.value=''; return; }
      } else { rawBuf = await file.arrayBuffer(); fname = file.name; }
      let u8;
      if (/\.(stp|step)$/i.test(fname)) {
        try {
          const geo = await parseGeometry(rawBuf, fname);
          u8 = new Uint8Array(stlFromGeometry(geo));
          fname = fname.replace(/\.(stp|step)$/i, '.stl');
        } catch(er) { alert('STEP Fehler: ' + er.message); t.value=''; return; }
      } else { u8 = new Uint8Array(rawBuf); }
      state.buffers.set(fname, u8);
      state.files = state.files.filter(f => partKey(f.name) !== ax || f.name === fname);
      if (!state.files.find(f => f.name === fname))
        state.files.push({ path: fname, name: fname, size: u8.byteLength, type: 'STL' });
      state.axisStlMap[ax] = fname;
      splitFiles();
      loadStls().then(() => { renderRows(); enableSave(); });
      t.value = '';
    })();
  }
});

renderer.domElement.addEventListener('pointerdown', pickAxisPoint);

// Drag & Drop auf Viewer
const dz=$('dropZone');
['dragenter','dragover'].forEach(ev=>dz.addEventListener(ev,e=>{e.preventDefault();dz.classList.add('drag-over');}));
['dragleave','drop'].forEach(ev=>dz.addEventListener(ev,e=>{e.preventDefault();dz.classList.remove('drag-over');}));
dz.addEventListener('drop', async e => {
  const files = Array.from(e.dataTransfer.files);
  if (!files.length) return;
  const zip  = files.find(f => /\.zip$/i.test(f.name));
  const stls = files.filter(f => /\.(stl|osd|stp|step)$/i.test(f.name));
  if (zip) { loadSourceZip(zip).catch(err=>alert(err.message)); return; }
  if (stls.length) { _dropStlsQueue(stls); return; }
  const items = Array.from(e.dataTransfer.items||[]);
  if (items.length) {
    const allFiles = [];
    for (const item of items) {
      const entry = item.webkitGetAsEntry?.();
      if (entry?.isDirectory) { await _collectEntries(entry, allFiles); }
      else if (item.getAsFile) { const f=item.getAsFile(); if(f) allFiles.push(f); }
    }
    if (allFiles.length) { loadSourceFolder(allFiles).catch(err=>alert(err.message)); return; }
  }
  loadSourceZip(files[0]).catch(err=>alert(err.message));
});

async function _collectEntries(entry, out) {
  if (entry.isFile) { await new Promise(res=>entry.file(f=>{out.push(f);res()},res)); }
  else if (entry.isDirectory) {
    const reader = entry.createReader();
    await new Promise(res=>reader.readEntries(async entries=>{for(const en of entries)await _collectEntries(en,out);res()}));
  }
}

var _dropPendingStls = [];
function _dropStlsQueue(files) { _dropPendingStls=files.slice(); _dropNextStl(); }
function _dropNextStl() {
  if (!_dropPendingStls.length) { _closeDropDlg(); return; }
  const file=_dropPendingStls[0], nm=file.name.toLowerCase();
  var guess=null;
  const jm=nm.match(/(?:joint[\s_-]*|^a)([1-6])/i); if(jm) guess='A'+jm[1];
  if(/podest|base|pedestal/i.test(nm)) guess='Podest';
  if(/tool|tcp|werkzeug/i.test(nm)) guess='Tool';
  if(/greifer|effekt|endeffektor/i.test(nm)) guess='Endeffektor';
  if(/umfeld|umgebung|fence|zaun/i.test(nm)) guess='Umgebung';
  if(/positionier|drehtisch/i.test(nm)) guess='Positionierer';
  if(/schiene|rail|track/i.test(nm)) guess='Schiene';
  if(/fest|fixture|tisch/i.test(nm)) guess='Festes Obj.';
  _showDropAxisPicker(file, guess);
}
function _showDropAxisPicker(file, preselect) {
  var dlg=document.getElementById('drop-axis-dlg');
  if (!dlg) { dlg=document.createElement('div'); dlg.id='drop-axis-dlg'; dlg.style.cssText='position:fixed;inset:0;z-index:9500;background:rgba(0,0,0,.65);display:flex;align-items:center;justify-content:center'; document.body.appendChild(dlg); }
  dlg.innerHTML='<div style="background:var(--bg2);border:1px solid var(--acc);border-radius:8px;padding:18px 22px;font-family:monospace;min-width:320px">'
    +'<div style="color:var(--acc);font-weight:700;margin-bottom:4px;font-size:.9em">📂 STL zuweisen ('+_dropPendingStls.length+' verbleibend)</div>'
    +'<div style="color:var(--txt2);font-size:.82em;margin-bottom:12px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap">'+file.name+'</div>'
    +'<div style="display:grid;grid-template-columns:repeat(4,1fr);gap:5px;margin-bottom:10px" id="drop-dlg-btns"></div>'
    +'<div style="display:flex;gap:6px;justify-content:flex-end;margin-top:8px">'
    +'<button id="drop-skip-btn" style="padding:5px 12px;background:var(--bg3);border:1px solid var(--bdr);border-radius:4px;color:var(--txt2);cursor:pointer;font-family:monospace">Überspringen</button>'
    +'<button id="drop-cancel-btn" style="padding:5px 12px;background:rgba(220,60,60,.2);border:1px solid rgba(220,60,60,.4);border-radius:4px;color:#f87171;cursor:pointer;font-family:monospace">Abbrechen</button>'
    +'</div></div>';
  document.getElementById('drop-skip-btn').onclick=_dropSkipStl;
  document.getElementById('drop-cancel-btn').onclick=_dropCancelAll;
  var btns=document.getElementById('drop-dlg-btns');
  ['A1','A2','A3','A4','A5','A6','Podest','Tool','Endeffektor','Umgebung','Positionierer','Schiene','Festes Obj.','Bew. Obj.'].forEach(function(ax){
    var b=document.createElement('button');
    b.textContent=ax;
    b.style.cssText='padding:4px 6px;border-radius:3px;cursor:pointer;font-family:monospace;font-size:.78em;white-space:nowrap;'+(ax===preselect?'background:var(--acc);color:#000;border:none;font-weight:700;':'background:var(--bg3);border:1px solid var(--bdr);color:var(--txt2);');
    b.addEventListener('click',(function(a){return function(){_dropAssignStl(file,a);};})(ax));
    btns.appendChild(b);
  });
  dlg.style.display='flex';
}
function _closeDropDlg(){var d=document.getElementById('drop-axis-dlg');if(d)d.style.display='none';}
function _dropSkipStl(){_dropPendingStls.shift();_closeDropDlg();_dropNextStl();}
function _dropCancelAll(){_dropPendingStls=[];_closeDropDlg();}

async function _dropAssignStl(file, cat) {
  _closeDropDlg();
  try {
    const rawBuf=await file.arrayBuffer(), fname=file.name;
    let geom; try{geom=await parseGeometry(rawBuf,fname);geom.computeVertexNormals();}catch(er){alert('Fehler: '+er.message);_dropPendingStls.shift();_dropNextStl();return;}
    const u8=new Uint8Array(rawBuf);
    const stlBuf=/\.(stp|step)$/i.test(fname)?new Uint8Array(stlFromGeometry(geom)):/\.osd$/i.test(fname)?new Uint8Array(osdToBinaryStl(rawBuf)):u8;
    const dn=fname.replace(/\.(stp|step|osd)$/i,'.stl');
    if(/^A[1-6]$/.test(cat)){
      if(!state.axisStlParts[cat])state.axisStlParts[cat]=[];
      if(!state.axisStlParts[cat].find(p=>norm(p.name)===norm(dn)))state.axisStlParts[cat].push({name:dn,color:'#e8a020',buf:stlBuf});
      state.axisStlMap[cat]=state.axisStlParts[cat][0]?.name||dn;
      state.buffers.set(dn,stlBuf);
      if(!state.stls.find(f=>f.name===dn))state.stls.push({path:dn,name:dn,type:'STL',size:stlBuf.byteLength});
      state.files=state.stls;
      const mesh=new THREE.Mesh(geom,new THREE.MeshStandardMaterial({color:'#e8a020',roughness:.62,metalness:.08}));mesh.name=dn;meshes.set(dn,mesh);
      rebuildRobotKinematics();applyTransforms();renderAxisStlRows();
    } else if(cat==='Podest'||cat==='Tool'){
      state.buffers.set(dn,stlBuf);
      if(!state.stls.find(f=>f.name===dn))state.stls.push({path:dn,name:dn,type:'STL',size:stlBuf.byteLength});
      state.files=state.stls;
      const mesh=new THREE.Mesh(geom,new THREE.MeshStandardMaterial({color:cat==='Podest'?'#334455':'#2563eb',roughness:.6,metalness:.1}));mesh.name=dn;meshes.set(dn,mesh);
      rebuildRobotKinematics();applyTransforms();
    } else if(cat==='Endeffektor'){
      const eff={name:dn.replace('.stl',''),color:'#607080',offset:{x:0,y:0,z:0,rx:0,ry:0,rz:0},ePos:0,
        teile:[{name:dn,objectType:'stl',color:'#607080',offset:{x:0,y:0,z:0,rx:0,ry:0,rz:0},stlFile:{name:dn,buf:stlBuf}}]};
      state.effektoren.push(eff); effektorGroups.push(null);
      state.activeEff=state.effektoren.length-1; rebuildEffMesh(state.activeEff); renderEffRow?.();
    } else if(cat==='Umgebung'){
      state.umfElemente.push({name:dn.replace('.stl',''),stlFile:{name:dn,buf:stlBuf,path:dn},offset:{x:0,y:0,z:0,rx:0,ry:0,rz:0}});
      umfGroups.push(null); state.buffers.set(dn,stlBuf);
      rebuildUmfMesh(state.umfElemente.length-1); renderUmfRows?.();
    } else if(cat==='Positionierer'){
      const eAx='E'+(state.positioners.length+2);
      const pos={name:dn.replace('.stl',''),eNum:state.positioners.length+2,type:'stl',rotAxis:'Z+',
        pivotX:0,pivotY:0,pivotZ:0,eMin:-180,eMax:180,ePos:0,showBox:false,boxOffset:{x:0,y:0,z:0,rx:0,ry:0,rz:0}};
      state.axisStlParts[eAx]=[{name:dn,color:'#e8a020',buf:stlBuf}];
      state.positioners.push(pos); positionerGroups.push(null);
      rebuildPositionerMesh?.(state.positioners.length-1); renderPosRows?.();
    } else if(cat==='Schiene'){
      const eAx='E1';
      state.schienen=[{name:dn.replace('.stl',''),length_mm:2000,height_mm:200,width_mm:400,axis:'X+',
        eNumber:1,eMin:0,eMax:2000,ePos:0,showBox:false,boxOffset:{x:0,y:0,z:0,rx:0,ry:0,rz:0}}];
      state.axisStlParts[eAx]=[{name:dn,color:'#2563eb',buf:stlBuf}];
      rebuildRailMeshes(); renderRailRows?.();
    } else if(cat==='Festes Obj.'){
      const fidx=state.festeObjekte.length;
      state.festeObjekte.push({name:dn.replace('.stl',''),stlFile:{name:dn,buf:stlBuf},color:'#606060',x:0,y:0,z:0,rx:0,ry:0,rz:0});
      festeGrps.push(null); rebuildFixMesh(fidx); renderFixRows?.();
    } else if(cat==='Bew. Obj.'){
      const oidx=state.objekte.length, lbl='Label'+(oidx+1);
      state.objekte.push({name:dn.replace('.stl',''),labelNum:oidx+1,axis:'Y+',ePos:0,mountMode:'fixed',showBox:false,boxOffset:{x:0,y:0,z:0,rx:0,ry:0,rz:0}});
      state.axisStlParts[lbl]=[{name:dn,color:'#4499cc',buf:stlBuf}];
      objekteGroups.push(null); rebuildObjektMesh(oidx); renderObjRows?.();
    }
    renderAll();enableSave?.();
  } catch(err){alert('Fehler: '+err.message);}
  _dropPendingStls.shift();_dropNextStl();
}


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




// ── Knotenpunkt-Größen-Schieberegler ───────────────────────────────
const _jSlider = document.getElementById('jointSizeSlider');
const _jVal    = document.getElementById('jointSizeVal');
if (_jSlider) {
  _jSlider.addEventListener('input', function() {
    _jointSizeScale = parseInt(this.value) / 30;  // 30 = Basiswert
    if (_jVal) _jVal.textContent = this.value;
    updateAxisPointVisuals();  // Kugeln neu bauen mit neuer Größe
  });
}

// ── ESC: TransformControls für Achspunkte deaktivieren ─────────────
document.addEventListener('keydown', function(e) {
  if (e.key === 'Escape') {
    if (transformControls && transformControls.object &&
        transformControls.object.userData && transformControls.object.userData.axisIndex !== undefined) {
      transformControls.detach();
    }
  }
});

// ═══════════════════════════════════════════════════

async function tryLoadAxisPng(buffers) {
  // Sucht nach axis.png (oder axis.PNG, Axis.png, ...) in den geladenen Dateien
  const imgKey = Array.from(buffers.keys()).find(k =>
    /^(?:.*[/\\])?axis\.(png|jpg|jpeg|webp)$/i.test(k)
  );
  if (!imgKey) return false;

  const u8 = buffers.get(imgKey);
  if (!u8 || u8.byteLength === 0) return false;

  // Base64 kodieren
  let b64 = '';
  const chunk = 8192;
  for (let i = 0; i < u8.length; i += chunk) {
    b64 += String.fromCharCode(...u8.subarray(i, i + chunk));
  }
  b64 = btoa(b64);
  const mime = /\.jpe?g$/i.test(imgKey) ? 'image/jpeg' : /\.webp$/i.test(imgKey) ? 'image/webp' : 'image/png';

  // Status anzeigen
  const notice = document.createElement('div');
  notice.style.cssText = 'position:fixed;top:60px;right:16px;z-index:9000;background:#1a3050;border:1px solid #4488cc;color:#aaddff;padding:10px 16px;border-radius:6px;font-family:monospace;font-size:12px';
  notice.textContent = '📐 axis.png gefunden — analysiere Kinematik…';
  document.body.appendChild(notice);

  try {
    // PHP-Proxy verwenden (verhindert CORS-Fehler beim direkten API-Aufruf)
    const _proxyUrl = (typeof ROBMODEL_BASE !== 'undefined' ? ROBMODEL_BASE : '') + 'axis_analyze.php';
    const resp = await fetch(_proxyUrl, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ image: b64, mime: mime })
    });

    const raw = await resp.text();
    // Debug: bei Fehler anzeigen
    let data;
    try { data = JSON.parse(raw); } catch(e) {
      throw new Error('Server-Antwort kein JSON: ' + raw.slice(0,200));
    }
    if (!resp.ok || data.error) throw new Error(data.error?.message || data.error || 'HTTP ' + resp.status);
    const text = (data.content || []).map(c => c.text || '').join('');
    // JSON robust extrahieren: erstes { bis letztes }
    const jsonStart = text.indexOf('{'), jsonEnd = text.lastIndexOf('}');
    if (jsonStart < 0 || jsonEnd < 0) throw new Error('Kein JSON in Antwort: ' + text.slice(0,200));
    const json = JSON.parse(text.slice(jsonStart, jsonEnd + 1));

    // axisOffsets → state.axisPoints
    if (Array.isArray(json.axisOffsets)) {
      json.axisOffsets.forEach((a, i) => {
        if (!state.axisPoints[i]) return;
        state.axisPoints[i].x = Number(a.x) || 0;
        state.axisPoints[i].y = Number(a.y) || 0;
        state.axisPoints[i].z = Number(a.z) || 0;
        state.axisPoints[i].source = 'axis.png';
      });
    }

    // axisLimits → state.joints
    if (Array.isArray(json.axisLimits)) {
      json.axisLimits.forEach((l, i) => {
        if (!state.joints[i]) return;
        if (l.min !== null && l.min !== undefined) state.joints[i].min = Number(l.min);
        if (l.max !== null && l.max !== undefined) state.joints[i].max = Number(l.max);
      });
    }

    syncJointsFromAxisPoints();
    rebuildRobotKinematics();
    applyTransforms();
    updateAxisPointVisuals();
    renderRows();

    notice.style.background = '#1a4020';
    notice.style.borderColor = '#44cc88';
    notice.style.color = '#88ffcc';
    notice.textContent = '✓ Kinematik aus axis.png übernommen';
    setTimeout(() => notice.remove(), 3000);
    return true;

  } catch(err) {
    notice.style.background = '#402020';
    notice.style.borderColor = '#cc4444';
    notice.style.color = '#ffaaaa';
    notice.textContent = '⚠ axis.png: ' + err.message;
    setTimeout(() => notice.remove(), 4000);
    return false;
  }
}
