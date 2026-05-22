/**
 * cable-system.js  –  Kabel-Physik + Editor  (RobModel / RobSimul)
 *
 * RobModel (Editor):
 *   init(THREE, scene, { upAxis:'z' })
 *   setTrackObjects({ tcp: tcpMarker })   ← nach jedem Roboter-Rebuild aufrufen
 *   refresh()                              ← in applyJointRotations() aufrufen
 *
 * RobSimul (Viewer):
 *   init(THREE, scene, { upAxis:'y', viewerOnly:true })
 *   loadCables(arr)                        ← in applyKinematicData() aufrufen
 *
 * Anker-Track-Modi:
 *   track: null          → fester Weltpunkt (X/Y/Z-Slider)
 *   track: 'tcp'         → folgt dem TCP-Marker des Roboters
 *   track: 'eff_N'       → folgt Endeffektor N  (setTrackObjects befüllen)
 *
 * JSON-Schlüssel: "cables"  →  [{ thickness, anchors:[{x,y,z,segLen,track?}] }]
 */

class CableSystem {

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 1 – Zustand
  // ═══════════════════════════════════════════════════════════════════════════

  constructor() {
    this.THREE      = null;
    this.scene      = null;
    this.upAxis     = 'y';
    this.viewerOnly = false;
    this.visible    = false;
    this._cables    = [];
    this.selCab     = 0;
    this.selAnch    = 0;
    this.sceneObjs  = [];
    this.cableMaxR  = [];
    this._trackMap  = {};   // { tcp: Object3D, eff_0: Object3D, ... }
    this.N_PTS      = 40;
    this._sGeoSm    = null;
    this._sGeoLg    = null;
    this._panel     = null;
  }

  // Getter für JSON-Export
  get cables() { return this._cables; }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 2 – Initialisierung
  // ═══════════════════════════════════════════════════════════════════════════

  init(THREE, scene, opts = {}) {
    this.THREE      = THREE;
    this.scene      = scene;
    this.upAxis     = opts.upAxis     || 'y';
    this.viewerOnly = opts.viewerOnly || false;

    this._sGeoSm = new THREE.SphereGeometry(6,  12,  9);
    this._sGeoLg = new THREE.SphereGeometry(10, 14, 10);

    if (!this.viewerOnly) {
      const up = this.upAxis;
      this._cables = [{
        thickness: 5,
        anchors: [
          { x:-400, y:up==='z'?1200:0, z:up==='z'?0:500, segLen:800, track:null },
          { x: 400, y:up==='z'?1200:0, z:up==='z'?0:500, segLen:800, track:null },
        ]
      }];
      this._buildPanel();
    } else {
      this._buildErrorBanner();
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 3 – Track-Objekte
  // setTrackObjects({ tcp: Object3D, eff_0: Object3D, ... })
  // Muss nach jedem Roboter-Rebuild aufgerufen werden.
  // ═══════════════════════════════════════════════════════════════════════════

  setTrackObjects(map) {
    this._trackMap = map || {};
    this._updateTrackDropdown();  // Optionen im Editor aktualisieren
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 4 – Lade-API (für JSON-Import)
  // ═══════════════════════════════════════════════════════════════════════════

  loadCables(arr) {
    if (!Array.isArray(arr)) return;
    this._cables = JSON.parse(JSON.stringify(arr));
    this.selCab  = 0;
    this.selAnch = 0;
    this.visible = arr.length > 0;
    if (!this.viewerOnly && this._panel) {
      this._panel.style.display = this.visible ? 'flex' : 'none';
      this._renderTable();
      this._loadDetail();
    }
    this._updateSideCard();
    this.refresh();
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 5 – Ankerpunkt-Weltposition
  // Wertet track-Feld aus: null → XYZ, 'tcp'/'eff_N' → getWorldPosition()
  // ═══════════════════════════════════════════════════════════════════════════

  _anchorWorldPos(ci, ai) {
    const T = this.THREE;
    const a = this._cables[ci].anchors[ai];
    if (a.track && this._trackMap[a.track]) {
      const obj = this._trackMap[a.track];
      const wp  = new T.Vector3();
      obj.getWorldPosition(wp);
      if (a.ox || a.oy || a.oz) {
        const wq = new T.Quaternion();
        obj.getWorldQuaternion(wq);
        wp.add(new T.Vector3(a.ox||0, a.oy||0, a.oz||0).applyQuaternion(wq));
      }
      return wp;
    }
    return new T.Vector3(a.x || 0, a.y || 0, a.z || 0);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 6 – Physik: Verlet PBD
  // ═══════════════════════════════════════════════════════════════════════════

  _resolveFloor(p) {
    if (this.upAxis === 'z' && p.z < 0) { p.z = 0; return true; }
    if (this.upAxis === 'y' && p.y < 0) { p.y = 0; return true; }
    return false;
  }

  _simCable(p1, p2, L) {
    const T = this.THREE, chord = p1.distanceTo(p2);
    if (chord >= L - 0.01) return { taut:true,  ratio:Math.min(chord/L,1), pts:null };
    if (chord <       0.5) return { taut:false, ratio:0,                   pts:null };

    const N = this.N_PTS, rL = L/N;
    const pos=[], old=[];
    for (let i=0;i<=N;i++){const p=p1.clone().lerp(p2,i/N);pos.push(p);old.push(p.clone());}

    const GY=0.14, DM=0.85, STEPS=280, CP=8, up=this.upAxis;
    for (let s=0;s<STEPS;s++){
      for(let i=1;i<N;i++){
        const v=pos[i].clone().sub(old[i]).multiplyScalar(DM);
        old[i].copy(pos[i]); pos[i].add(v);
        if(up==='z')pos[i].z-=GY; else pos[i].y-=GY;
      }
      for(let c=0;c<CP;c++){
        for(let i=0;i<N;i++){
          const d=pos[i].distanceTo(pos[i+1]);
          if(d<1e-4)continue;
          const cr=new T.Vector3().subVectors(pos[i+1],pos[i]).multiplyScalar((d-rL)/(d*2));
          if(i>0)pos[i].add(cr); if(i<N)pos[i+1].sub(cr);
        }
        pos[0].copy(p1); pos[N].copy(p2);
        for(let i=1;i<N;i++)this._resolveFloor(pos[i]);
      }
    }
    return {taut:false, ratio:chord/L, pts:pos};
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 7 – Rendering
  // ═══════════════════════════════════════════════════════════════════════════

  _tensionColor(r) {
    if(r>=1.0)return{color:0xff1111,emissive:0x440000,emissiveIntensity:.6};
    if(r>.92) return{color:0xff4400,emissive:0x110000,emissiveIntensity:.2};
    if(r>.75) return{color:0x885500,emissive:0,emissiveIntensity:0};
    return         {color:0xcccccc,emissive:0,emissiveIntensity:0};
  }

  _makeTube(pts, col, radius) {
    const T=this.THREE;
    const g=new T.TubeGeometry(new T.CatmullRomCurve3(pts,false,'centripetal'),Math.max(pts.length,5),radius,8,false);
    return new T.Mesh(g,new T.MeshStandardMaterial({roughness:.85,metalness:.05,...col}));
  }

  refresh() {
    if (!this.scene) return;
    const T=this.THREE;
    this.sceneObjs.forEach(o=>{
      o.meshes.forEach(m=>{this.scene.remove(m);m.geometry.dispose();m.material.dispose();});
      o.marks.forEach(m=>{this.scene.remove(m);m.material.dispose();});
    });
    this.sceneObjs=[]; this.cableMaxR=[];

    if (!this.visible||this._cables.length===0){this._updateErrorBanner([]);return;}

    const broken=[];
    this._cables.forEach((cab,ci)=>{
      const obj={meshes:[],marks:[]}, aCount=cab.anchors.length;
      const pos=cab.anchors.map((_,ai)=>this._anchorWorldPos(ci,ai));
      let maxR=0;

      for(let ai=0;ai<aCount;ai++){
        const sel=(!this.viewerOnly&&ci===this.selCab&&ai===this.selAnch);
        const isTracked=!!(cab.anchors[ai].track);
        const col=isTracked?0xffaa00:0x66aaff;
        const mat=new T.MeshStandardMaterial({color:col,emissive:col,emissiveIntensity:sel?.8:.25,roughness:.3,metalness:.5});
        const m=new T.Mesh(sel?this._sGeoLg:this._sGeoSm,mat);
        m.position.copy(pos[ai]); this.scene.add(m); obj.marks.push(m);
      }

      for(let seg=0;seg<aCount-1;seg++){
        const res=this._simCable(pos[seg],pos[seg+1],cab.anchors[seg].segLen);
        maxR=Math.max(maxR,res.ratio);
        const col=this._tensionColor(res.ratio);
        if(res.taut)broken.push({ci,seg});
        const pts=res.pts??[0,1,2,3,4].map(k=>pos[seg].clone().lerp(pos[seg+1],k/4));
        const mesh=this._makeTube(pts,col,cab.thickness);
        this.scene.add(mesh); obj.meshes.push(mesh);
      }
      this.sceneObjs.push(obj); this.cableMaxR.push(maxR);
    });

    if(!this.viewerOnly) this._renderTable();
    this._updateErrorBanner(broken);
    this._updateSideCard();
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 8 – Seitenleisten-Karte (robmodel rechtes Panel)
  // Kompakte Übersicht in #kabel-rows (falls vorhanden).
  // ═══════════════════════════════════════════════════════════════════════════

  _updateSideCard() {
    const div=document.getElementById('kabel-rows'); if(!div) return;
    const badge=document.getElementById('kabelBadge');
    if(badge) badge.textContent=this._cables.length;
    div.innerHTML=this._cables.length===0
      ? '<span style="font-size:10px;color:#3a5a7a">Keine Kabel</span>'
      : this._cables.map((c,i)=>{
          const r=this.cableMaxR[i]||0;
          const pct=Math.min(Math.round(r*100),100);
          const bc=r>=1?'#aa1111':r>.92?'#aa3300':r>.75?'#664400':'#1a5a2a';
          return `<div style="display:flex;align-items:center;gap:7px;padding:4px 0;border-bottom:1px solid rgba(255,255,255,.05);font-size:11px">
            <span style="font-weight:700;color:#c8d8e8;min-width:20px">K${i+1}</span>
            <span style="color:#6a8fa8">${c.anchors.length}A · ${c.thickness}mm</span>
            <div style="flex:1;height:3px;background:rgba(255,255,255,.06);border-radius:2px;overflow:hidden">
              <div style="width:${pct}%;height:100%;background:${bc};border-radius:2px"></div></div>
            <span style="color:#4a6a8a;font-size:10px;min-width:28px;text-align:right">${pct}%</span>
            <button onclick="window.cableSystem.selCab_f(${i});window.cableSystem.setVisible(true)"
              style="background:none;border:none;color:#5a7a9a;cursor:pointer;font-size:12px;padding:0 2px" title="Bearbeiten">✏</button>
          </div>`;
        }).join('');
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 9 – Floating Editor-Panel (nur Editor-Modus)
  // ═══════════════════════════════════════════════════════════════════════════

  _buildErrorBanner() {
    if(document.getElementById('cs-err'))return;
    const s=document.createElement('style');
    s.textContent='#cs-err{display:none;position:fixed;top:10px;left:50%;transform:translateX(-50%);background:rgba(180,0,0,.92);border:1px solid #cc2222;color:#fff;font:bold 10px/1 monospace;letter-spacing:2px;padding:8px 18px;text-transform:uppercase;z-index:9500;animation:cs-blink .65s ease-in-out infinite}@keyframes cs-blink{0%,100%{opacity:1}50%{opacity:.6}}';
    document.head.appendChild(s);
    const e=document.createElement('div');e.id='cs-err';document.body.appendChild(e);
  }

  _buildPanel() {
    if(this._panel)return;
    const s=document.createElement('style');
    s.textContent=`
      #cs-panel{position:fixed;top:0;right:0;width:265px;height:100vh;background:var(--bg4,#0c1824);border-left:1px solid var(--bdr,#1b3454);display:none;flex-direction:column;overflow-y:auto;z-index:8500;font-family:monospace;font-size:12px;color:var(--txt2,#8aacca)}
      #cs-panel .cs-h{padding:8px 12px;border-bottom:1px solid var(--bdr,#1b3454);display:flex;align-items:center;justify-content:space-between;flex-shrink:0}
      #cs-panel .cs-title{font-weight:700;letter-spacing:2px;color:var(--acc,#f90);font-size:11px}
      #cs-panel .cs-x{background:none;border:none;color:var(--txt2,#8aacca);cursor:pointer;font-size:16px;padding:0 4px;line-height:1}
      #cs-panel .cs-x:hover{color:#fff}
      #cs-panel .cs-s{padding:8px 12px;border-bottom:1px solid var(--bdr,#1b3454)}
      #cs-panel .cs-lbl{font-size:9px;letter-spacing:2px;color:var(--txt3,#4a6a8a);text-transform:uppercase;margin-bottom:5px;display:flex;justify-content:space-between;align-items:center}
      #cs-panel .cs-row{display:flex;align-items:center;gap:6px;margin-bottom:4px}
      .cs-ax{width:14px;text-align:center;font-weight:700;font-size:10px}
      .cs-ax.x{color:#ff5555}.cs-ax.y{color:#55cc77}.cs-ax.z{color:#5599ff}
      #cs-panel input[type=range]{flex:1;height:2px;-webkit-appearance:none;background:var(--bdr,#1b3454);outline:none;border-radius:1px}
      #cs-panel input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:10px;height:10px;border-radius:50%;background:var(--acc,#f90);cursor:pointer}
      #cs-panel select{flex:1;background:var(--bg4,#0c1824);border:1px solid var(--bdr,#1b3454);color:var(--txt2,#8aacca);font-family:monospace;font-size:10px;padding:2px 4px;border-radius:2px;cursor:pointer}
      #cs-panel .cs-val{font-size:9px;width:38px;text-align:right;color:var(--txt3,#4a6a8a)}
      #cs-panel .cs-sep{height:1px;background:var(--bdr,#1b3454);margin:3px 0}
      #cs-ctable .cs-crow{display:grid;grid-template-columns:22px 68px 1fr 26px 16px;align-items:center;gap:4px;padding:5px 6px;cursor:pointer;border:1px solid transparent;border-radius:3px;font-size:9px;transition:background .1s}
      #cs-ctable .cs-crow:hover{background:rgba(255,255,255,.05)}
      #cs-ctable .cs-crow.on{background:rgba(255,138,0,.08);border-color:rgba(255,138,0,.2)}
      .cs-cnum{color:var(--txt3,#4a6a8a);font-weight:700;letter-spacing:1px}.cs-crow.on .cs-cnum{color:var(--acc,#f90)}
      .cs-cinfo{color:var(--txt3,#4a6a8a);font-size:8.5px}.cs-ctbar{height:3px;background:rgba(255,255,255,.08);border-radius:2px;overflow:hidden}
      .cs-ctpct{text-align:right;color:var(--txt3,#4a6a8a);font-size:8.5px}
      .cs-cdel{background:none;border:none;color:rgba(255,255,255,.2);cursor:pointer;font:12px/1 monospace;padding:0}.cs-cdel:hover{color:#ff5555}
      #cs-achips{display:flex;flex-wrap:wrap;gap:3px;margin-bottom:6px}
      .cs-chip{padding:2px 7px;font:9px/1 monospace;cursor:pointer;border:1px solid var(--bdr,#1b3454);background:transparent;color:var(--txt3,#4a6a8a);transition:all .12s}
      .cs-chip:hover{border-color:var(--txt2,#8aacca);color:var(--txt,#c8d8e8)}
      .cs-chip.on{background:var(--acc,#f90);border-color:var(--acc,#f90);color:#000;font-weight:700}
      .cs-btn{padding:2px 8px;font:10px/1 monospace;cursor:pointer;border:1px solid var(--bdr,#1b3454);background:transparent;color:var(--txt2,#8aacca);transition:all .12s}
      .cs-btn:hover{border-color:var(--txt2,#8aacca);color:var(--txt,#c8d8e8)}
      .cs-dim{opacity:.3;pointer-events:none}
      .cs-tracked{opacity:.3;pointer-events:none}
      #cs-err{display:none;position:fixed;top:10px;left:50%;transform:translateX(-50%);background:rgba(180,0,0,.92);border:1px solid #cc2222;color:#fff;font:bold 10px/1 monospace;letter-spacing:2px;padding:8px 18px;text-transform:uppercase;z-index:9500;animation:cs-blink .65s ease-in-out infinite}
      @keyframes cs-blink{0%,100%{opacity:1}50%{opacity:.6}}
    `;
    document.head.appendChild(s);

    const p=document.createElement('div');p.id='cs-panel';
    p.innerHTML=`
      <div class="cs-h">
        <span class="cs-title">KABEL-EDITOR</span>
        <button class="cs-x" onclick="window.cableSystem.setVisible(false)">✕</button>
      </div>
      <div class="cs-s">
        <div class="cs-lbl">Kabel <button class="cs-btn" onclick="window.cableSystem.addCable()">+ Kabel</button></div>
        <div id="cs-ctable"></div>
      </div>
      <div class="cs-s">
        <div class="cs-lbl" id="cs-detlbl">KABEL 1</div>
        <div class="cs-row">
          <span class="cs-ax" style="width:28px;color:var(--txt2,#8aacca)">R</span>
          <input type="range" id="cs-slt" min="1" max="30" value="5" step="0.5" oninput="window.cableSystem.updThick()">
          <span class="cs-val" id="cs-sltv">5 mm</span>
        </div>
        <div class="cs-sep"></div>
        <div class="cs-lbl">Ankerpunkte
          <div style="display:flex;gap:3px">
            <button class="cs-btn" onclick="window.cableSystem.addAnchor()">+</button>
            <button class="cs-btn" onclick="window.cableSystem.delAnchor()">−</button>
          </div>
        </div>
        <div id="cs-achips"></div>
        <div class="cs-row" style="margin-bottom:6px">
          <span class="cs-ax" style="width:28px;font-size:9px;color:var(--txt3,#4a6a8a)">Ref</span>
          <select id="cs-ref" onchange="window.cableSystem.updRef()">
            <option value="">Welt (X/Y/Z)</option>
          </select>
        </div>
        <div id="cs-xyz-block">
          <div class="cs-row"><span class="cs-ax x">X</span><input type="range" id="cs-ax" min="-2000" max="2000" value="0" step="5" oninput="window.cableSystem.updA()"><span class="cs-val" id="cs-axv">0</span></div>
          <div class="cs-row"><span class="cs-ax y">Y</span><input type="range" id="cs-ay" min="-200" max="3000" value="500" step="5" oninput="window.cableSystem.updA()"><span class="cs-val" id="cs-ayv">500</span></div>
          <div class="cs-row"><span class="cs-ax z">Z</span><input type="range" id="cs-az" min="-2000" max="2000" value="0" step="5" oninput="window.cableSystem.updA()"><span class="cs-val" id="cs-azv">0</span></div>
        </div>
        <div class="cs-row" id="cs-lrow">
          <span class="cs-ax" style="width:28px;font-size:9px;color:var(--txt3,#4a6a8a)">L→</span>
          <input type="range" id="cs-al" min="50" max="2000" value="800" step="10" oninput="window.cableSystem.updA()">
          <span class="cs-val" id="cs-alv">800</span>
        </div>
      </div>`;
    document.body.appendChild(p);

    const e=document.createElement('div');e.id='cs-err';document.body.appendChild(e);
    this._panel=p;
    this._renderTable();
    this._loadDetail();
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 10 – Track-Dropdown aktualisieren
  // Wird aufgerufen wenn setTrackObjects() neue Objekte registriert.
  // ═══════════════════════════════════════════════════════════════════════════

  _updateTrackDropdown() {
    const sel=document.getElementById('cs-ref'); if(!sel)return;
    const cur=sel.value;
    const labels={'tcp':'TCP','eff_0':'Endeffektor 1','eff_1':'Endeffektor 2','eff_2':'Endeffektor 3'};
    sel.innerHTML='<option value="">Welt (X/Y/Z)</option>'+
      Object.keys(this._trackMap).map(k=>`<option value="${k}"${cur===k?' selected':''}>${labels[k]||k}</option>`).join('');
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 11 – Editor-UI: Kabel-Tabelle
  // ═══════════════════════════════════════════════════════════════════════════

  _renderTable() {
    const div=document.getElementById('cs-ctable');if(!div)return;
    div.innerHTML=this._cables.map((cab,i)=>{
      const r=this.cableMaxR[i]||0,pct=Math.min(Math.round(r*100),100);
      const bc=r>=1?'#aa1111':r>.92?'#aa3300':r>.75?'#664400':'#1a5a2a';
      return `<div class="cs-crow${i===this.selCab?' on':''}" onclick="window.cableSystem.selCab_f(${i})">
        <span class="cs-cnum">K${i+1}</span>
        <span class="cs-cinfo">${cab.anchors.length}A·${cab.thickness}mm</span>
        <div class="cs-ctbar"><div style="width:${pct}%;background:${bc};height:100%;border-radius:2px"></div></div>
        <span class="cs-ctpct">${pct}%</span>
        <button class="cs-cdel" onclick="event.stopPropagation();window.cableSystem.delCable(${i})">×</button>
      </div>`;
    }).join('');
  }

  selCab_f(i){this.selCab=i;this.selAnch=Math.min(this.selAnch,this._cables[i].anchors.length-1);this._loadDetail();this.refresh();}
  addCable(){
    if(this._cables.length>=8)return;
    const up=this.upAxis;
    this._cables.push({thickness:5,anchors:[
      {x:0,y:up==='z'?1200:500,z:up==='z'?-400:0,segLen:800,track:null},
      {x:0,y:up==='z'?1200:500,z:up==='z'? 400:0,segLen:800,track:null}]});
    this.selCab=this._cables.length-1;this.selAnch=0;this._loadDetail();this.refresh();
  }
  delCable(i){
    if(this._cables.length<=1)return;
    this._cables.splice(i,1);this.selCab=Math.min(this.selCab,this._cables.length-1);
    this.selAnch=Math.min(this.selAnch,this._cables[this.selCab].anchors.length-1);
    this._loadDetail();this.refresh();
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 12 – Editor-UI: Anker-Detail
  // ═══════════════════════════════════════════════════════════════════════════

  _loadDetail(){
    const cab=this._cables[this.selCab];if(!cab)return;
    const dl=document.getElementById('cs-detlbl');if(dl)dl.textContent=`KABEL ${this.selCab+1}  ·  ${cab.anchors.length} Anker`;
    const slt=document.getElementById('cs-slt');
    if(slt){slt.value=cab.thickness;document.getElementById('cs-sltv').textContent=cab.thickness+' mm';}
    this._renderChips();this._loadAnchorEdit();
  }

  _renderChips(){
    const div=document.getElementById('cs-achips');if(!div)return;
    div.innerHTML='';
    this._cables[this.selCab].anchors.forEach((a,i)=>{
      const btn=document.createElement('button');
      const tracked=!!(a.track);
      btn.className='cs-chip'+(i===this.selAnch?' on':'');
      btn.textContent=`A${i+1}`+(tracked?'🔗':'');
      btn.onclick=()=>this.selA(i);
      div.appendChild(btn);
    });
  }

  _loadAnchorEdit(){
    const a=this._cables[this.selCab].anchors[this.selAnch];if(!a)return;
    const isLast=this.selAnch===this._cables[this.selCab].anchors.length-1;
    const tracked=!!(a.track);

    // Referenz-Dropdown
    const ref=document.getElementById('cs-ref');
    if(ref){this._updateTrackDropdown();ref.value=a.track||'';}

    // XYZ-Slider (gesperrt wenn Track aktiv)
    [['cs-ax',a.x],['cs-ay',a.y],['cs-az',a.z]].forEach(([id,val])=>{
      const el=document.getElementById(id);if(el)el.value=val||0;
      const vd=document.getElementById(id+'v');if(vd)vd.textContent=val||0;
    });
    const xyz=document.getElementById('cs-xyz-block');
    if(xyz)xyz.className=tracked?'cs-tracked':'';

    document.getElementById('cs-al').value=a.segLen;
    document.getElementById('cs-alv').textContent=a.segLen;
    const lr=document.getElementById('cs-lrow');if(lr)lr.className='cs-row'+(isLast?' cs-dim':'');
  }

  selA(i){this.selAnch=i;this._renderChips();this._loadAnchorEdit();this.refresh();}
  updRef(){
    const a=this._cables[this.selCab].anchors[this.selAnch];
    a.track=document.getElementById('cs-ref').value||null;
    this._renderChips();this._loadAnchorEdit();this.refresh();
  }
  updA(){
    const a=this._cables[this.selCab].anchors[this.selAnch];
    if(!a.track){
      a.x=+document.getElementById('cs-ax').value; document.getElementById('cs-axv').textContent=a.x;
      a.y=+document.getElementById('cs-ay').value; document.getElementById('cs-ayv').textContent=a.y;
      a.z=+document.getElementById('cs-az').value; document.getElementById('cs-azv').textContent=a.z;
    }
    a.segLen=+document.getElementById('cs-al').value; document.getElementById('cs-alv').textContent=a.segLen;
    this.refresh();
  }
  updThick(){
    const v=+document.getElementById('cs-slt').value;
    this._cables[this.selCab].thickness=v;document.getElementById('cs-sltv').textContent=v+' mm';this.refresh();
  }
  addAnchor(){
    const cab=this._cables[this.selCab];if(cab.anchors.length>=8)return;
    const last=cab.anchors.at(-1);
    cab.anchors.push({x:last.x+200,y:last.y,z:last.z,segLen:last.segLen,track:null});
    this.selAnch=cab.anchors.length-1;this._renderChips();this._loadAnchorEdit();this.refresh();
  }
  delAnchor(){
    const cab=this._cables[this.selCab];if(cab.anchors.length<=2)return;
    cab.anchors.splice(this.selAnch,1);this.selAnch=Math.min(this.selAnch,cab.anchors.length-1);
    this._renderChips();this._loadAnchorEdit();this.refresh();
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Abschnitt 13 – Sichtbarkeit + Fehler-Banner
  // ═══════════════════════════════════════════════════════════════════════════

  setVisible(v){
    this.visible=v;
    if(!this.viewerOnly&&this._panel)this._panel.style.display=v?'flex':'none';
    this.refresh();
  }
  toggleVisible(){this.setVisible(!this.visible);}

  _updateErrorBanner(broken){
    const el=document.getElementById('cs-err');if(!el)return;
    if(broken.length){el.textContent='KABEL GERISSEN — '+broken.map(b=>`K${b.ci+1} SEG ${b.seg+1}→${b.seg+2}`).join(' · ');el.style.display='block';}
    else el.style.display='none';
  }
}

window.cableSystem = new CableSystem();
