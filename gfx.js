// ═══════════════════════════════════════════════════════════════════
//  RobModel — Grafikqualität (three.js 0.160, ES-Module)
//   standard : bisherige Darstellung (ohne Umgebung, ohne Tonemapping, Schatten nur am Objekt)
//   high     : Studio-Umgebung (RoomEnvironment), PBR-Material mit Klarlack (MeshPhysicalMaterial),
//              Schattenboden mit weichen Schatten, ACES-Tonemapping
//   max      : wie high + Ambient Occlusion (GTAO), als Abdunklung über das fertige Bild gelegt
//  In allen Stufen: STL-Normalen mit Knickwinkel geglättet (Rundungen glatt, Kanten scharf).
//  Gleiche Stufen und Begriffe wie RobSimul / RobSimul Simple (js/graphics.js).
// ═══════════════════════════════════════════════════════════════════
import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { GTAOPass } from 'three/addons/postprocessing/GTAOPass.js';
import { FullScreenQuad } from 'three/addons/postprocessing/Pass.js';

const LEVELS = ['standard', 'high', 'max'];
const LBL = { standard: 'Standard', high: 'Hoch', max: 'Maximal' };
const KEY = 'robmodel.quality';

// ── Glatte STL-Normalen (Knickwinkel) ──────────────────────────────
export function smoothNormals(geo, creaseDeg = 32) {
  const pos = geo.attributes.position;
  if (!pos) return geo;
  if (geo.index) { THREE.BufferGeometry.prototype.computeVertexNormals.call(geo); return geo; }
  const p = pos.array, nV = pos.count, nT = Math.floor(nV / 3), cosC = Math.cos(creaseDeg * Math.PI / 180);
  const fn = new Float32Array(nT * 3), fu = new Float32Array(nT * 3);
  for (let t = 0; t < nT; t++) {
    const i = t * 9, ax = p[i+3]-p[i], ay = p[i+4]-p[i+1], az = p[i+5]-p[i+2], bx = p[i+6]-p[i], by = p[i+7]-p[i+1], bz = p[i+8]-p[i+2];
    const nx = ay*bz-az*by, ny = az*bx-ax*bz, nz = ax*by-ay*bx, l = Math.hypot(nx, ny, nz) || 1;
    fn[t*3] = nx; fn[t*3+1] = ny; fn[t*3+2] = nz; fu[t*3] = nx/l; fu[t*3+1] = ny/l; fu[t*3+2] = nz/l;
  }
  const map = new Map(), key = new Array(nV);
  for (let v = 0; v < nV; v++) {
    const k = Math.round(p[v*3]*500) + '_' + Math.round(p[v*3+1]*500) + '_' + Math.round(p[v*3+2]*500);
    key[v] = k; const l2 = map.get(k); if (l2) l2.push(v / 3 | 0); else map.set(k, [v / 3 | 0]);
  }
  const out = new Float32Array(nV * 3);
  for (let v = 0; v < nV; v++) {
    const t0 = v / 3 | 0, list = map.get(key[v]); let sx = 0, sy = 0, sz = 0;
    for (const t1 of list) {
      if (fu[t0*3]*fu[t1*3] + fu[t0*3+1]*fu[t1*3+1] + fu[t0*3+2]*fu[t1*3+2] >= cosC) { sx += fn[t1*3]; sy += fn[t1*3+1]; sz += fn[t1*3+2]; }
    }
    const ln = Math.hypot(sx, sy, sz) || 1; out[v*3] = sx/ln; out[v*3+1] = sy/ln; out[v*3+2] = sz/ln;
  }
  geo.setAttribute('normal', new THREE.BufferAttribute(out, 3));
  geo.userData.smoothed = true;
  return geo;
}
// STL-Lader: geglättete Normalen; ein späteres geo.computeVertexNormals() glättet ebenso (statt flach)
if (!STLLoader.prototype._gfx) {
  const orig = STLLoader.prototype.parse;
  STLLoader.prototype.parse = function (data) {
    const g = orig.call(this, data);
    smoothNormals(g, 32);
    g.userData.posVer = g.attributes.position.version;
    g.computeVertexNormals = function () {
      if (!this.userData.smoothed || this.attributes.position.version !== this.userData.posVer) smoothNormals(this, 32);
      this.userData.posVer = this.attributes.position.version;
    };
    return g;
  };
  STLLoader.prototype._gfx = true;
}

// ── Grafik-Stufen ──────────────────────────────────────────────────
// opts: { renderer, scene, getCamera(), getTarget(), sun, fill, hemi, onChange? }
export function initGfx(opts) {
  const { renderer, scene, sun, fill, hemi } = opts;
  let quality = 'high';
  try {
    const q = new URLSearchParams(location.search).get('quality') || localStorage.getItem(KEY);
    if (LEVELS.includes(q)) quality = q;
  } catch (e) {}

  const STD = { hemi: hemi ? hemi.intensity : 0, sun: sun.intensity, fill: fill ? fill.intensity : 0, sunPos: sun.position.clone() };
  const HI = { hemi: 0.35, sun: 2.2, fill: 0.35 };
  const SUN_DIR = sun.position.clone().normalize();
  const EXPOSURE = 0.95, ENV_INTENSITY = 0.6;

  sun.shadow.mapSize.set(4096, 4096);
  sun.shadow.bias = -0.0004;
  sun.shadow.normalBias = 1.2;
  sun.shadow.radius = 3;
  scene.add(sun.target);

  const ground = new THREE.Mesh(new THREE.PlaneGeometry(60000, 60000),
    new THREE.ShadowMaterial({ opacity: 0.3, transparent: true, depthWrite: false }));
  ground.position.z = -0.5; ground.receiveShadow = true; ground.renderOrder = -1;
  ground.userData.gfx = true; ground.visible = false; ground.raycast = () => {};
  scene.add(ground);

  let envTex = null;
  function ensureEnv() {
    if (envTex) return envTex;
    const pmrem = new THREE.PMREMGenerator(renderer);
    envTex = pmrem.fromScene(new RoomEnvironment(renderer), 0.04).texture;
    pmrem.dispose();
    return envTex;
  }

  // Standard-Materialien undurchsichtiger Bauteile bekommen in high/max ein Physical-Gegenstück:
  // lackierte Teile mit Klarlack, graue/dunkle Teile metallisch. Leuchtende Marker bleiben unverändert.
  function eligible(m) {
    return m && m.isMeshStandardMaterial && !m.isMeshPhysicalMaterial &&
      !(m.emissiveIntensity > 0 && m.emissive && m.emissive.getHex() !== 0) && !m.userData.gfxSkip;
  }
  function physFrom(m) {
    const c = m.color.clone(), hsl = {}; c.getHSL(hsl, THREE.SRGBColorSpace);
    const metal = hsl.s < 0.12 && hsl.l < 0.55;
    const p = new THREE.MeshPhysicalMaterial({
      color: c, map: m.map,
      metalness: metal ? 0.6 : 0.05, roughness: metal ? 0.36 : 0.42,
      clearcoat: metal ? 0.1 : 0.45, clearcoatRoughness: 0.2,
      envMapIntensity: ENV_INTENSITY,
      transparent: m.transparent, opacity: m.opacity, side: m.side,
      depthWrite: m.depthWrite, depthTest: m.depthTest, wireframe: m.wireframe
    });
    p.userData.fromStd = true;
    return p;
  }
  let casterKey = '';
  function prepare(hi) {
    const keys = [];
    scene.traverse(o => {
      if (o.userData.gfx || !o.isMesh || !o.material || Array.isArray(o.material)) return;
      const u = o.userData;
      if (eligible(o.material) && o.material !== u.matStd) { u.matStd = o.material; u.matPhys = null; }
      const ms = u.matStd;
      if (!ms || (o.material !== ms && o.material !== u.matPhys)) return;
      if (hi && !u.matPhys) u.matPhys = physFrom(ms);
      const mp = u.matPhys;
      if (mp) {   // Farbe, Sichtbarkeit und Transparenz folgen dem Original (Hervorhebungen, Themes)
        if (!mp.color.equals(ms.color)) mp.color.copy(ms.color);
        mp.opacity = ms.opacity; mp.transparent = ms.transparent; mp.visible = ms.visible; mp.side = ms.side; mp.wireframe = ms.wireframe;
      }
      o.material = hi ? mp : ms;
      if (hi && o.castShadow && o.visible) keys.push(o.id);
    });
    const key = keys.join(',');
    if (hi && key !== casterKey) { casterKey = key; fitShadow(); }
  }
  function fitShadow() {
    const box = new THREE.Box3(), tmp = new THREE.Box3();
    scene.traverse(o => { if (o.isMesh && o.castShadow && o.visible && o.geometry && !o.userData.gfx) { tmp.setFromObject(o); if (!tmp.isEmpty() && isFinite(tmp.min.x)) box.union(tmp); } });
    if (box.isEmpty()) box.set(new THREE.Vector3(-1500, -1500, 0), new THREE.Vector3(1500, 1500, 2000));
    const c = box.getCenter(new THREE.Vector3()), r = Math.min(20000, Math.max(800, box.getSize(new THREE.Vector3()).length() * 0.6));
    sun.target.position.set(c.x, c.y, 0);
    sun.position.copy(sun.target.position).addScaledVector(SUN_DIR, r * 3);
    const cam = sun.shadow.camera;
    cam.left = -r; cam.right = r; cam.top = r; cam.bottom = -r; cam.near = r * 0.5; cam.far = r * 6;
    cam.updateProjectionMatrix();
    sun.shadow.needsUpdate = true;
  }

  // ── Ambient Occlusion (max) ──────────────────────────────────────
  // Das Bild wird normal (mit Tonemapping, sRGB, Kantenglättung des Canvas) gezeichnet; danach wird nur die
  // AO-Abdunklung multiplikativ darübergelegt. So bleiben Hintergrund, Linien und Beschriftungen farbtreu.
  let gtao = null, aoRT = null, aoQuad = null;
  function buildAO() {
    disposeAO();
    const size = renderer.getDrawingBufferSize(new THREE.Vector2());
    gtao = new GTAOPass(scene, opts.getCamera(), size.x, size.y);
    gtao.output = GTAOPass.OUTPUT.Denoise;
    gtao.updateGtaoMaterial({ radius: 120, distanceExponent: 1, thickness: 80, scale: 1.5, samples: 16 });
    gtao.updatePdMaterial({ lumaPhi: 10, depthPhi: 2, normalPhi: 3, radius: 6, rings: 2, samples: 16 });
    // Beschriftungen (Sprites), Overlays ohne Tiefentest und stark transparente Flächen nicht in die AO aufnehmen
    const ov = gtao.overrideVisibility.bind(gtao);
    gtao.overrideVisibility = function () {
      ov();
      this.scene.traverse(o => {
        const m = o.material;
        if (o.isSprite || (o.isMesh && m && !Array.isArray(m) && (m.depthTest === false || (m.transparent && m.opacity < 0.5)))) o.visible = false;
      });
    };
    aoRT = new THREE.WebGLRenderTarget(size.x, size.y);
    aoQuad = new FullScreenQuad(new THREE.ShaderMaterial({
      uniforms: { tAO: { value: aoRT.texture }, strength: { value: 1.0 } },
      vertexShader: 'varying vec2 vUv; void main(){ vUv = uv; gl_Position = vec4(position.xy, 0.0, 1.0); }',
      fragmentShader: 'uniform sampler2D tAO; uniform float strength; varying vec2 vUv; void main(){ float ao = texture2D(tAO, vUv).r; gl_FragColor = vec4(vec3(mix(1.0, ao, strength)), 1.0); }',
      blending: THREE.CustomBlending, blendEquation: THREE.AddEquation,
      blendSrc: THREE.DstColorFactor, blendDst: THREE.ZeroFactor,
      blendSrcAlpha: THREE.ZeroFactor, blendDstAlpha: THREE.OneFactor,
      depthTest: false, depthWrite: false, toneMapped: false, transparent: true
    }));
  }
  function disposeAO() {
    if (!gtao) return;
    try { gtao.dispose(); aoRT.dispose(); aoQuad.material.dispose(); aoQuad.dispose(); } catch (e) {}
    gtao = aoRT = aoQuad = null;
  }
  // Tiefenbereich an den Abstand anpassen (genauere Tiefe für Schatten und AO)
  function fitDepth(cam) {
    const t = opts.getTarget ? opts.getTarget() : null;
    if (!t) return;
    const d = cam.position.distanceTo(t);
    let n, f;
    if (cam.isPerspectiveCamera) { n = Math.max(2, d * 0.02); f = d + 30000; }
    else { n = Math.max(-30000, d - 30000); f = d + 30000; }
    if (Math.abs(cam.near - n) > 1e-6 || Math.abs(cam.far - f) > 1e-6) { cam.near = n; cam.far = f; cam.updateProjectionMatrix(); }
  }

  function syncGround() {
    const b = scene.background && scene.background.isColor ? scene.background : null;
    const lum = b ? 0.2126 * b.r + 0.7152 * b.g + 0.0722 * b.b : 0;
    ground.material.opacity = lum > 0.5 ? 0.18 : 0.32;
  }

  function apply() {
    const hi = quality !== 'standard';
    renderer.toneMapping = hi ? THREE.ACESFilmicToneMapping : THREE.NoToneMapping;
    renderer.toneMappingExposure = EXPOSURE;
    scene.environment = hi ? ensureEnv() : null;
    if (hemi) hemi.intensity = hi ? HI.hemi : STD.hemi;
    sun.intensity = hi ? HI.sun : STD.sun;
    if (fill) fill.intensity = hi ? HI.fill : STD.fill;
    if (!hi) { sun.position.copy(STD.sunPos); sun.target.position.set(0, 0, 0); }
    ground.visible = hi;
    casterKey = '';
    if (quality === 'max') { if (!gtao) buildAO(); } else disposeAO();
    scene.traverse(o => { const ms = o.material ? (Array.isArray(o.material) ? o.material : [o.material]) : []; ms.forEach(m => { if (m) m.needsUpdate = true; }); });
    document.querySelectorAll('[data-quality]').forEach(b => b.classList.toggle('on', b.dataset.quality === quality));
    document.querySelectorAll('[data-gfx-label]').forEach(el => { el.textContent = 'Grafik ' + LBL[quality]; });
    if (opts.onChange) opts.onChange(quality);
  }

  function render() {
    const hi = quality !== 'standard', cam = opts.getCamera();
    prepare(hi);
    syncGround();
    if (hi) fitDepth(cam);
    if (gtao && quality === 'max') {
      gtao.camera = cam;
      gtao.render(renderer, aoRT, aoRT);          // Normalen/Tiefe → AO → entrauscht, nach aoRT
      renderer.setRenderTarget(null);
      renderer.render(scene, cam);                // fertiges Bild
      const ac = renderer.autoClear; renderer.autoClear = false;
      aoQuad.render(renderer);                    // AO multiplikativ darüber
      renderer.autoClear = ac;
    } else renderer.render(scene, cam);
  }
  function resize(w, h) {
    if (!gtao) return;
    const size = renderer.getDrawingBufferSize(new THREE.Vector2());
    gtao.setSize(size.x, size.y); aoRT.setSize(size.x, size.y);
  }
  function setQuality(q) {
    if (!LEVELS.includes(q)) return false;
    quality = q;
    try { localStorage.setItem(KEY, q); } catch (e) {}
    apply();
    return true;
  }
  const api = {
    render, resize, setQuality, smoothNormals,
    cycle: () => setQuality(LEVELS[(LEVELS.indexOf(quality) + 1) % LEVELS.length]),
    quality: () => quality, levels: LEVELS.slice()
  };
  apply();
  return api;
}
