/**
 * cable-system.js  –  Kabel-Physik + Editor für RobModel / RobSimul
 *
 * Einbindung (klassisches Script, VOR </body>):
 *   <script src="./js/cable-system.js"></script>
 *
 * Initialisierung nach Szenen-Aufbau:
 *   window.cableSystem.init(THREE, scene, { upAxis: 'z' });   // robmodel  (Z=oben)
 *   window.cableSystem.init(THREE, scene, { upAxis: 'y' });   // robsimul  (Y=oben)
 *
 * Anzeige umschalten:
 *   window.cableSystem.toggleVisible();
 *
 * Kabelstand nach Roboterupdate (z.B. nach applyJointRotations):
 *   window.cableSystem.refresh();
 */

class CableSystem {

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 1 – Datenmodell
  // cables[]:  Liste aller Kabel  { thickness, anchors:[{x,y,z,segLen}] }
  // sceneObjs: Aktive 3D-Objekte  [{meshes:[], marks:[]}]
  // cableMaxR: Max. Spannungsquotient pro Kabel (für Tabellenanzeige)
  // ═══════════════════════════════════════════════════════════════════════════

  constructor() {
    this.THREE     = null;
    this.scene     = null;
    this.upAxis    = 'y';   // 'y' (robsimul) oder 'z' (robmodel)
    this.visible   = false;
    this.cables    = [];
    this.selCab    = 0;
    this.selAnch   = 0;
    this.sceneObjs = [];
    this.cableMaxR = [];
    this.N_PTS     = 40;    // Simulations-Punkte pro Segment
    this._sGeoSm   = null;
    this._sGeoLg   = null;
    this._panel    = null;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 2 – Initialisierung
  // Erhält THREE und scene von der Hostanwendung.
  // upAxis bestimmt Schwerkraftrichtung und Bodenebene.
  // ═══════════════════════════════════════════════════════════════════════════

  init(THREE, scene, opts = {}) {
    this.THREE   = THREE;
    this.scene   = scene;
    this.upAxis  = opts.upAxis || 'y';

    // Standard-Kabel (2 Ankerpunkte, 1 Segment)
    const up = this.upAxis;
    this.cables = [{
      thickness: 5,
      anchors: [
        { x: -400, y: up==='z'?1200:0, z: up==='z'?0:500, segLen: 800 },
        { x:  400, y: up==='z'?1200:0, z: up==='z'?0:500, segLen: 800 },
      ]
    }];

    this._sGeoSm = new THREE.SphereGeometry(6,  12,  9);
    this._sGeoLg = new THREE.SphereGeometry(10, 14, 10);
    this._buildPanel();
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 3 – Physik: Kabelsimulation (Verlet + PBD)
  //
  // _simCable(p1, p2, L):
  //   Modelliert Kabel als Kette von N_PTS Massepunkten.
  //   Gravity + Längenconstraints + Bodenebene.
  //   → { taut, ratio, pts }
  //
  // _resolveFloor(p):
  //   Verhindert Punkte unterhalb der Bodenebene (y<0 oder z<0 je nach upAxis).
  // ═══════════════════════════════════════════════════════════════════════════

  _resolveFloor(p) {
    const ax = this.upAxis;
    if (ax === 'z' && p.z < 0) { p.z = 0; return true; }
    if (ax === 'y' && p.y < 0) { p.y = 0; return true; }
    return false;
  }

  _simCable(p1, p2, L) {
    const T  = this.THREE;
    const chord = p1.distanceTo(p2);

    // Gespannte oder zu kurze Segmente früh behandeln
    if (chord >= L - 0.01) return { taut: true,  ratio: Math.min(chord / L, 1), pts: null };
    if (chord < 0.5)        return { taut: false, ratio: 0,                       pts: null };

    const N    = this.N_PTS;
    const rL   = L / N;
    const pos  = [], old = [];
    for (let i = 0; i <= N; i++) {
      const p = p1.clone().lerp(p2, i / N);
      pos.push(p); old.push(p.clone());
    }

    const GY    = 0.14;   // mm/Schritt Schwerkraft
    const DAMP  = 0.85;   // Geschwindigkeitsdämpfung
    const STEPS = 280;    // Simulationsschritte
    const CP    = 8;      // Constraint-Durchläufe je Schritt
    const up    = this.upAxis;

    for (let s = 0; s < STEPS; s++) {
      // Verlet: Trägheit + Schwerkraft
      for (let i = 1; i < N; i++) {
        const v = pos[i].clone().sub(old[i]).multiplyScalar(DAMP);
        old[i].copy(pos[i]);
        pos[i].add(v);
        if (up === 'z') pos[i].z -= GY;
        else            pos[i].y -= GY;
      }
      // Constraints
      for (let c = 0; c < CP; c++) {
        // Längenconstraint: jedes Segment auf rL normieren
        for (let i = 0; i < N; i++) {
          const d = pos[i].distanceTo(pos[i + 1]);
          if (d < 1e-4) continue;
          const corr = new T.Vector3()
            .subVectors(pos[i + 1], pos[i])
            .multiplyScalar((d - rL) / (d * 2));
          if (i > 0)   pos[i].add(corr);
          if (i < N)   pos[i + 1].sub(corr);
        }
        // Endpunkte fixieren
        pos[0].copy(p1);
        pos[N].copy(p2);
        // Bodenebene
        for (let i = 1; i < N; i++) this._resolveFloor(pos[i]);
      }
    }

    return { taut: false, ratio: chord / L, pts: pos };
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 4 – Rendering
  //
  // refresh():    Löscht alte Objekte, simuliert alle Kabel, baut neue Meshes.
  // _tensionColor(r): Farbe nach Spannungsquotient  (schwarz → orange → rot).
  // _makeTube():  TubeGeometry aus Punktliste.
  // ═══════════════════════════════════════════════════════════════════════════

  _tensionColor(r) {
    if (r >= 1.0) return { color: 0xff1111, emissive: 0x440000, emissiveIntensity: .6 };
    if (r >  .92) return { color: 0xff4400, emissive: 0x110000, emissiveIntensity: .2 };
    if (r >  .75) return { color: 0x885500, emissive: 0,        emissiveIntensity:  0 };
    return             { color: 0xcccccc, emissive: 0,        emissiveIntensity:  0 };
  }

  _makeTube(pts, col, radius) {
    const T    = this.THREE;
    const curve = new T.CatmullRomCurve3(pts, false, 'centripetal');
    const geo   = new T.TubeGeometry(curve, Math.max(pts.length, 5), radius, 8, false);
    return new T.Mesh(geo, new T.MeshStandardMaterial({ roughness: .85, metalness: .05, ...col }));
  }

  refresh() {
    if (!this.scene) return;
    const T = this.THREE;

    // Alte Szenenobjekte entfernen + Speicher freigeben
    this.sceneObjs.forEach(o => {
      o.meshes.forEach(m => { this.scene.remove(m); m.geometry.dispose(); m.material.dispose(); });
      o.marks.forEach(m  => { this.scene.remove(m); m.material.dispose(); });
    });
    this.sceneObjs = [];
    this.cableMaxR = [];

    if (!this.visible) { this._updateErrorBanner([]); return; }

    const broken = [];

    this.cables.forEach((cab, ci) => {
      const obj    = { meshes: [], marks: [] };
      const aCount = cab.anchors.length;
      const pos    = cab.anchors.map(a => new T.Vector3(a.x, a.y, a.z));
      let   maxR   = 0;

      // Ankerpunkt-Marker (Kugeln)
      for (let ai = 0; ai < aCount; ai++) {
        const sel = (ci === this.selCab && ai === this.selAnch);
        const mat = new T.MeshStandardMaterial({
          color: 0x66aaff, emissive: 0x2244aa,
          emissiveIntensity: sel ? .7 : .2,
          roughness: .3, metalness: .5
        });
        const m = new T.Mesh(sel ? this._sGeoLg : this._sGeoSm, mat);
        m.position.copy(pos[ai]);
        this.scene.add(m);
        obj.marks.push(m);
      }

      // Kabelsegmente
      for (let seg = 0; seg < aCount - 1; seg++) {
        const res  = this._simCable(pos[seg], pos[seg + 1], cab.anchors[seg].segLen);
        maxR = Math.max(maxR, res.ratio);
        const col  = this._tensionColor(res.ratio);
        if (res.taut) broken.push({ ci, seg });
        const pts  = res.pts ?? [0,1,2,3,4].map(k => pos[seg].clone().lerp(pos[seg + 1], k / 4));
        this.scene.add(this._makeTube(pts, col, cab.thickness));
        obj.meshes.push(obj.meshes[obj.meshes.length - 1] || this.scene.children[this.scene.children.length - 1]);
      }

      this.sceneObjs.push(obj);
      this.cableMaxR.push(maxR);
    });

    this._renderTable();
    this._updateErrorBanner(broken);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 5 – Panel: Aufbau
  //
  // Erzeugt ein schwebendes Panel (position:fixed, rechte Seite),
  // das über window.cableSystem.toggleVisible() ein-/ausgeblendet wird.
  // Passt sich mit CSS-Variablen dem App-Thema an.
  // ═══════════════════════════════════════════════════════════════════════════

  _buildPanel() {
    if (this._panel) return;

    // Styles (nutzt App-CSS-Variablen mit Fallbacks)
    const style = document.createElement('style');
    style.textContent = `
      #cs-panel{
        position:fixed;top:0;right:0;width:265px;height:100vh;
        background:var(--bg4,#0c1824);border-left:1px solid var(--bdr,#1b3454);
        display:none;flex-direction:column;overflow-y:auto;z-index:8500;
        font-family:monospace;font-size:12px;color:var(--txt2,#8aacca);
      }
      #cs-panel .cs-head{
        padding:8px 12px;border-bottom:1px solid var(--bdr,#1b3454);
        display:flex;align-items:center;justify-content:space-between;
        background:var(--bg4,#0c1824);flex-shrink:0;
      }
      #cs-panel .cs-title{font-weight:700;letter-spacing:2px;color:var(--acc,#f90);font-size:11px}
      #cs-panel .cs-close{background:none;border:none;color:var(--txt2,#8aacca);cursor:pointer;font-size:16px;padding:0 4px;line-height:1}
      #cs-panel .cs-close:hover{color:var(--txt,#c8d8e8)}
      #cs-panel .cs-sec{padding:8px 12px;border-bottom:1px solid var(--bdr,#1b3454)}
      #cs-panel .cs-lbl{font-size:9px;letter-spacing:2px;color:var(--txt3,#4a6a8a);text-transform:uppercase;margin-bottom:5px;display:flex;justify-content:space-between;align-items:center}
      #cs-panel .cs-row{display:flex;align-items:center;gap:6px;margin-bottom:4px}
      #cs-panel .cs-ax{width:14px;text-align:center;font-weight:700;font-size:10px}
      #cs-panel .cs-ax.x{color:#ff5555}
      #cs-panel .cs-ax.y{color:#55cc77}
      #cs-panel .cs-ax.z{color:#5599ff}
      #cs-panel input[type=range]{flex:1;height:2px;-webkit-appearance:none;background:var(--bdr,#1b3454);outline:none;border-radius:1px}
      #cs-panel input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:10px;height:10px;border-radius:50%;background:var(--acc,#f90);cursor:pointer}
      #cs-panel .cs-val{font-size:9px;width:38px;text-align:right;color:var(--txt3,#4a6a8a)}
      #cs-panel .cs-sep{height:1px;background:var(--bdr,#1b3454);margin:3px 0}

      /* Kabel-Tabelle */
      #cs-ctable .cs-crow{display:grid;grid-template-columns:22px 68px 1fr 26px 16px;align-items:center;gap:4px;padding:5px 6px;cursor:pointer;border:1px solid transparent;border-radius:3px;font-size:9px;transition:background .1s}
      #cs-ctable .cs-crow:hover{background:rgba(255,255,255,.05)}
      #cs-ctable .cs-crow.on{background:rgba(255,138,0,.08);border-color:rgba(255,138,0,.2)}
      .cs-cnum{color:var(--txt3,#4a6a8a);font-weight:700;letter-spacing:1px}
      .cs-crow.on .cs-cnum{color:var(--acc,#f90)}
      .cs-cinfo{color:var(--txt3,#4a6a8a);font-size:8.5px}
      .cs-ctbar{height:3px;background:rgba(255,255,255,.08);border-radius:2px;overflow:hidden}
      .cs-ctpct{text-align:right;color:var(--txt3,#4a6a8a);font-size:8.5px}
      .cs-cdel{background:none;border:none;color:rgba(255,255,255,.2);cursor:pointer;font:12px/1 monospace;padding:0}
      .cs-cdel:hover{color:#ff5555}

      /* Anker-Chips */
      #cs-achips{display:flex;flex-wrap:wrap;gap:3px;margin-bottom:6px}
      .cs-chip{padding:2px 7px;font:9px/1 monospace;cursor:pointer;border:1px solid var(--bdr,#1b3454);background:transparent;color:var(--txt3,#4a6a8a);transition:all .12s;letter-spacing:.5px}
      .cs-chip:hover{border-color:var(--txt2,#8aacca);color:var(--txt,#c8d8e8)}
      .cs-chip.on{background:var(--acc,#f90);border-color:var(--acc,#f90);color:#000;font-weight:700}

      /* Kleine Buttons */
      .cs-btn{padding:2px 8px;font:10px/1 monospace;cursor:pointer;border:1px solid var(--bdr,#1b3454);background:transparent;color:var(--txt2,#8aacca);transition:all .12s}
      .cs-btn:hover{border-color:var(--txt2,#8aacca);color:var(--txt,#c8d8e8)}
      .cs-dim{opacity:.3;pointer-events:none}

      /* Fehler-Banner */
      #cs-err{display:none;position:fixed;top:10px;left:50%;transform:translateX(-50%);
        background:rgba(180,0,0,.92);border:1px solid #cc2222;color:#fff;
        font:bold 10px/1 monospace;letter-spacing:2px;padding:8px 18px;
        text-transform:uppercase;z-index:9500;animation:cs-blink .65s ease-in-out infinite}
      @keyframes cs-blink{0%,100%{opacity:1}50%{opacity:.6}}
    `;
    document.head.appendChild(style);

    // Panel-HTML
    const panel = document.createElement('div');
    panel.id = 'cs-panel';
    panel.innerHTML = `
      <div class="cs-head">
        <span class="cs-title">KABEL</span>
        <button class="cs-close" onclick="window.cableSystem.setVisible(false)">✕</button>
      </div>

      <!-- Kabel-Tabelle -->
      <div class="cs-sec">
        <div class="cs-lbl">
          Kabel
          <button class="cs-btn" onclick="window.cableSystem.addCable()">+ Kabel</button>
        </div>
        <div id="cs-ctable"></div>
      </div>

      <!-- Ausgewähltes Kabel: Dicke -->
      <div class="cs-sec" id="cs-detail">
        <div class="cs-lbl" id="cs-detlbl">KABEL 1</div>
        <div class="cs-row">
          <span class="cs-ax" style="width:28px;color:var(--txt2,#8aacca)">R</span>
          <input type="range" id="cs-slt" min="1" max="30" value="5" step="0.5" oninput="window.cableSystem.updThick()">
          <span class="cs-val" id="cs-sltv">5 mm</span>
        </div>

        <!-- Ankerpunkte -->
        <div class="cs-sep"></div>
        <div class="cs-lbl">
          Ankerpunkte
          <div style="display:flex;gap:3px">
            <button class="cs-btn" onclick="window.cableSystem.addAnchor()">+</button>
            <button class="cs-btn" onclick="window.cableSystem.delAnchor()">−</button>
          </div>
        </div>
        <div id="cs-achips"></div>

        <!-- Anker-Editor -->
        <div id="cs-aedit">
          <div class="cs-row"><span class="cs-ax x">X</span><input type="range" id="cs-ax" min="-2000" max="2000" value="0" step="5" oninput="window.cableSystem.updA()"><span class="cs-val" id="cs-axv">0</span></div>
          <div class="cs-row"><span class="cs-ax y">Y</span><input type="range" id="cs-ay" min="-200"  max="3000" value="500" step="5" oninput="window.cableSystem.updA()"><span class="cs-val" id="cs-ayv">500</span></div>
          <div class="cs-row"><span class="cs-ax z">Z</span><input type="range" id="cs-az" min="-2000" max="2000" value="0" step="5" oninput="window.cableSystem.updA()"><span class="cs-val" id="cs-azv">0</span></div>
          <div class="cs-row" id="cs-lrow">
            <span class="cs-ax" style="width:28px;font-size:9px;color:var(--txt3,#4a6a8a)">L→</span>
            <input type="range" id="cs-al" min="50" max="2000" value="800" step="10" oninput="window.cableSystem.updA()">
            <span class="cs-val" id="cs-alv">800</span>
          </div>
        </div>
      </div>
    `;
    document.body.appendChild(panel);

    // Fehler-Banner (außerhalb des Panels)
    const err = document.createElement('div');
    err.id = 'cs-err';
    document.body.appendChild(err);

    this._panel = panel;
    this._renderTable();
    this._loadDetail();
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 6 – Panel: Kabel-Tabelle
  // ═══════════════════════════════════════════════════════════════════════════

  _renderTable() {
    const div = document.getElementById('cs-ctable');
    if (!div) return;
    div.innerHTML = this.cables.map((cab, i) => {
      const r   = this.cableMaxR[i] || 0;
      const pct = Math.min(Math.round(r * 100), 100);
      const bc  = r >= 1 ? '#aa1111' : r > .92 ? '#aa3300' : r > .75 ? '#664400' : '#1a5a2a';
      return `<div class="cs-crow${i === this.selCab ? ' on' : ''}" onclick="window.cableSystem.selCab_f(${i})">
        <span class="cs-cnum">K${i+1}</span>
        <span class="cs-cinfo">${cab.anchors.length}A·${cab.thickness}mm</span>
        <div class="cs-ctbar"><div style="width:${pct}%;background:${bc};height:100%;border-radius:2px"></div></div>
        <span class="cs-ctpct">${pct}%</span>
        <button class="cs-cdel" onclick="event.stopPropagation();window.cableSystem.delCable(${i})">×</button>
      </div>`;
    }).join('');
  }

  selCab_f(i) {
    this.selCab  = i;
    this.selAnch = Math.min(this.selAnch, this.cables[i].anchors.length - 1);
    this._loadDetail();
    this.refresh();
  }

  addCable() {
    if (this.cables.length >= 8) return;
    const up = this.upAxis;
    this.cables.push({ thickness: 5, anchors: [
      { x: 0, y: up==='z'?1200:500, z: up==='z'?-400:0, segLen: 800 },
      { x: 0, y: up==='z'?1200:500, z: up==='z'? 400:0, segLen: 800 },
    ]});
    this.selCab  = this.cables.length - 1;
    this.selAnch = 0;
    this._loadDetail();
    this.refresh();
  }

  delCable(i) {
    if (this.cables.length <= 1) return;
    this.cables.splice(i, 1);
    this.selCab  = Math.min(this.selCab,  this.cables.length - 1);
    this.selAnch = Math.min(this.selAnch, this.cables[this.selCab].anchors.length - 1);
    this._loadDetail();
    this.refresh();
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 7 – Panel: Kabel-Detail (Dicke + Ankerpunkte)
  // ═══════════════════════════════════════════════════════════════════════════

  _loadDetail() {
    const cab = this.cables[this.selCab];
    const dl = document.getElementById('cs-detlbl');
    if (dl) dl.textContent = `KABEL ${this.selCab + 1}  ·  ${cab.anchors.length} Anker`;
    const slt = document.getElementById('cs-slt');
    if (slt) { slt.value = cab.thickness; document.getElementById('cs-sltv').textContent = cab.thickness + ' mm'; }
    this._renderChips();
    this._loadAnchorEdit();
  }

  _renderChips() {
    const div = document.getElementById('cs-achips');
    if (!div) return;
    div.innerHTML = '';
    this.cables[this.selCab].anchors.forEach((_, i) => {
      const btn = document.createElement('button');
      btn.className = 'cs-chip' + (i === this.selAnch ? ' on' : '');
      btn.textContent = `A${i + 1}`;
      btn.onclick = () => this.selA(i);
      div.appendChild(btn);
    });
  }

  _loadAnchorEdit() {
    const a      = this.cables[this.selCab].anchors[this.selAnch];
    const isLast = this.selAnch === this.cables[this.selCab].anchors.length - 1;
    const set    = (id, val, sfx='') => {
      const el = document.getElementById(id);
      if (el) { el.value = val; }
      const vd = document.getElementById(id + 'v');
      if (vd) vd.textContent = val + sfx;
    };
    set('cs-ax', a.x); set('cs-ay', a.y); set('cs-az', a.z);
    set('cs-al', a.segLen);
    document.getElementById('cs-axv').textContent = a.x;
    document.getElementById('cs-ayv').textContent = a.y;
    document.getElementById('cs-azv').textContent = a.z;
    document.getElementById('cs-alv').textContent = a.segLen;
    const lrow = document.getElementById('cs-lrow');
    if (lrow) lrow.className = 'cs-row' + (isLast ? ' cs-dim' : '');
  }

  selA(i)     { this.selAnch = i; this._renderChips(); this._loadAnchorEdit(); this.refresh(); }

  updA() {
    const a = this.cables[this.selCab].anchors[this.selAnch];
    a.x = +document.getElementById('cs-ax').value;
    a.y = +document.getElementById('cs-ay').value;
    a.z = +document.getElementById('cs-az').value;
    a.segLen = +document.getElementById('cs-al').value;
    document.getElementById('cs-axv').textContent = a.x;
    document.getElementById('cs-ayv').textContent = a.y;
    document.getElementById('cs-azv').textContent = a.z;
    document.getElementById('cs-alv').textContent = a.segLen;
    this.refresh();
  }

  updThick() {
    const v = +document.getElementById('cs-slt').value;
    this.cables[this.selCab].thickness = v;
    document.getElementById('cs-sltv').textContent = v + ' mm';
    this.refresh();
  }

  addAnchor() {
    const cab = this.cables[this.selCab];
    if (cab.anchors.length >= 8) return;
    const last = cab.anchors.at(-1);
    cab.anchors.push({ x: last.x + 200, y: last.y, z: last.z, segLen: last.segLen });
    this.selAnch = cab.anchors.length - 1;
    this._renderChips(); this._loadAnchorEdit(); this.refresh();
  }

  delAnchor() {
    const cab = this.cables[this.selCab];
    if (cab.anchors.length <= 2) return;
    cab.anchors.splice(this.selAnch, 1);
    this.selAnch = Math.min(this.selAnch, cab.anchors.length - 1);
    this._renderChips(); this._loadAnchorEdit(); this.refresh();
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 8 – Sichtbarkeit + Fehler-Banner
  // ═══════════════════════════════════════════════════════════════════════════

  setVisible(v) {
    this.visible = v;
    if (this._panel) this._panel.style.display = v ? 'flex' : 'none';
    this.refresh();
  }

  toggleVisible() { this.setVisible(!this.visible); }

  _updateErrorBanner(broken) {
    const el = document.getElementById('cs-err');
    if (!el) return;
    if (broken.length) {
      el.textContent = 'KABEL GERISSEN — ' + broken.map(b => `K${b.ci+1} SEG ${b.seg+1}→${b.seg+2}`).join(' · ');
      el.style.display = 'block';
    } else {
      el.style.display = 'none';
    }
  }
}

// Singleton – global verfügbar
window.cableSystem = new CableSystem();
