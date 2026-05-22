/**
 * cable-system.js  –  Kabel-Editor (RobModel) / Kabel-Viewer (RobSimul)
 *
 * RobModel:  init(THREE, scene, {upAxis:'z'})
 *            setTrackObjects({tcp, a1..a6})  ← am Ende von rebuildRobotKinematics()
 *            refresh()                        ← in applyJointRotations()
 *
 * RobSimul:  init(THREE, scene, {upAxis:'y', viewerOnly:true})
 *            loadCables(arr)                  ← in applyKinematicData()
 *
 * Track-Modi im Anker:  null → Welt-XYZ   |   'a1'..'a6' → Gelenkpunkt   |   'tcp' → Werkzeugspitze
 *
 * JSON-Schlüssel "cables": [{thickness, anchors:[{x,y,z,segLen,track?}]}]
 */

class CableSystem {

  // ─── 1. Zustand ────────────────────────────────────────────────────────────

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
    this._trackMap  = {};
    this.N_PTS      = 40;
    this._sGeoSm    = null;
    this._sGeoLg    = null;
    this._panel     = null;
    this._TRACK_LABELS = {
      a1:'A1 – Achse 1', a2:'A2 – Achse 2', a3:'A3 – Achse 3',
      a4:'A4 – Achse 4', a5:'A5 – Achse 5', a6:'A6 – Flansch',
      tcp:'TCP – Werkzeugspitze'
    };
  }

  get cables() { return this._cables; }

  // ─── 2. Initialisierung ─────────────────────────────────────────────────────

  init(THREE, scene, opts = {}) {
    this.THREE      = THREE;
    this.scene      = scene;
    this.upAxis     = opts.upAxis     || 'y';
    this.viewerOnly = opts.viewerOnly || false;
    this._sGeoSm    = new THREE.SphereGeometry(7,  14, 10);
    this._sGeoLg    = new THREE.SphereGeometry(11, 16, 12);

    if (!this.viewerOnly) {
      // Standard-Kabel: A1 → TCP (folgt dem Roboter sobald setTrackObjects aufgerufen wird)
      this._cables = [{
        thickness: 6,
        anchors: [
          { x:0, y:0, z:0, ox:0, oy:0, oz:0, segLen:1200, track:'a1' },
          { x:0, y:0, z:0, ox:0, oy:0, oz:0, segLen:1200, track:'tcp' },
        ]
      }];
      this._buildPanel();
    } else {
      this._buildErrorBanner();
    }
  }

  // ─── 3. Track-Objekte registrieren ──────────────────────────────────────────
  // Muss nach jedem rebuildRobotKinematics() aufgerufen werden,
  // da die Pivot-Gruppen bei jedem Rebuild neu erstellt werden.

  setTrackObjects(map) {
    this._trackMap = {};
    for (const [k, v] of Object.entries(map)) {
      if (v) this._trackMap[k] = v;
    }
    this._updateTrackDropdown();
  }

  // ─── 4. Laden aus JSON (viewerOnly) ─────────────────────────────────────────

  loadCables(arr) {
    if (!Array.isArray(arr)) return;
    this._cables = JSON.parse(JSON.stringify(arr));
    this.selCab  = 0;
    this.selAnch = 0;
    this.visible = arr.length > 0;
    if (!this.viewerOnly && this._panel) {
      this._panel.style.display = this.visible ? 'flex' : 'none';
      this._renderTable(); this._loadDetail();
    }
    this._updateSideCard();
    this.refresh();
  }

  // ─── 5. Ankerpunkt-Weltposition ─────────────────────────────────────────────

  _anchorWorldPos(ci, ai) {
    const T = this.THREE, a = this._cables[ci].anchors[ai];
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
    return new T.Vector3(a.x||0, a.y||0, a.z||0);
  }

  // ─── 6. Physik: Verlet PBD ──────────────────────────────────────────────────

  _resolveFloor(p) {
    if (this.upAxis==='z' && p.z<0){p.z=0;return true;}
    if (this.upAxis==='y' && p.y<0){p.y=0;return true;}
    return false;
  }

  _simCable(p1, p2, L) {
    const T=this.THREE, chord=p1.distanceTo(p2);
    if (chord>=L-0.01) return {taut:true,  ratio:Math.min(chord/L,1), pts:null};
    if (chord<0.5)      return {taut:false, ratio:0, pts:null};
    const N=this.N_PTS, rL=L/N;
    const pos=[],old=[];
    for(let i=0;i<=N;i++){const p=p1.clone().lerp(p2,i/N);pos.push(p);old.push(p.clone());}
    const GY=0.14,DM=0.85,STEPS=280,CP=8,up=this.upAxis;
    for(let s=0;s<STEPS;s++){
      for(let i=1;i<N;i++){
        const v=pos[i].clone().sub(old[i]).multiplyScalar(DM);
        old[i].copy(pos[i]);pos[i].add(v);
        if(up==='z')pos[i].z-=GY;else pos[i].y-=GY;
      }
      for(let c=0;c<CP;c++){
        for(let i=0;i<N;i++){
          const d=pos[i].distanceTo(pos[i+1]);if(d<1e-4)continue;
          const cr=new T.Vector3().subVectors(pos[i+1],pos[i]).multiplyScalar((d-rL)/(d*2));
          if(i>0)pos[i].add(cr);if(i<N)pos[i+1].sub(cr);
        }
        pos[0].copy(p1);pos[N].copy(p2);
        for(let i=1;i<N;i++)this._resolveFloor(pos[i]);
      }
    }
    return {taut:false,ratio:chord/L,pts:pos};
  }

  // ─── 7. Rendering ───────────────────────────────────────────────────────────

  _tCol(r){
    if(r>=1.0)return{color:0xff2222,emissive:0x440000,emissiveIntensity:.6};
    if(r>.92) return{color:0xff5500,emissive:0x110000,emissiveIntensity:.2};
    if(r>.75) return{color:0x885500,emissive:0,emissiveIntensity:0};
    return         {color:0xd0e0f0,emissive:0,emissiveIntensity:0};
  }

  _tube(pts,col,r){
    const T=this.THREE;
    const g=new T.TubeGeometry(new T.CatmullRomCurve3(pts,false,'centripetal'),Math.max(pts.length,5),r,8,false);
    return new T.Mesh(g,new T.MeshStandardMaterial({roughness:.8,metalness:.1,...col}));
  }

  refresh() {
    if (!this.scene) return;
    const T=this.THREE;
    this.sceneObjs.forEach(o=>{
      o.meshes.forEach(m=>{this.scene.remove(m);m.geometry.dispose();m.material.dispose();});
      o.marks.forEach(m=>{this.scene.remove(m);m.material.dispose();});
    });
    this.sceneObjs=[];this.cableMaxR=[];
    if(!this.visible||!this._cables.length){this._updateErrorBanner([]);return;}
    const broken=[];
    this._cables.forEach((cab,ci)=>{
      const obj={meshes:[],marks:[]},aC=cab.anchors.length;
      const pos=cab.anchors.map((_,ai)=>this._anchorWorldPos(ci,ai));
      let maxR=0;
      for(let ai=0;ai<aC;ai++){
        const sel=!this.viewerOnly&&ci===this.selCab&&ai===this.selAnch;
        const tracked=!!(cab.anchors[ai].track);
        const col=tracked?0xf05500:0x4488cc;
        const mat=new T.MeshStandardMaterial({color:col,emissive:col,emissiveIntensity:sel?.7:.25,roughness:.3,metalness:.5});
        const m=new T.Mesh(sel?this._sGeoLg:this._sGeoSm,mat);
        m.position.copy(pos[ai]);this.scene.add(m);obj.marks.push(m);
      }
      for(let seg=0;seg<aC-1;seg++){
        const res=this._simCable(pos[seg],pos[seg+1],cab.anchors[seg].segLen);
        maxR=Math.max(maxR,res.ratio);
        if(res.taut)broken.push({ci,seg});
        const pts=res.pts??[0,1,2,3,4].map(k=>pos[seg].clone().lerp(pos[seg+1],k/4));
        const m=this._tube(pts,this._tCol(res.ratio),cab.thickness);
        this.scene.add(m);obj.meshes.push(m);
      }
      this.sceneObjs.push(obj);this.cableMaxR.push(maxR);
    });
    if(!this.viewerOnly)this._renderTable();
    this._updateErrorBanner(broken);
    this._updateSideCard();
  }

  // ─── 8. Seitenleisten-Karte (#kabel-rows) ───────────────────────────────────

  _updateSideCard(){
    const div=document.getElementById('kabel-rows');if(!div)return;
    const b=document.getElementById('kabelBadge');if(b)b.textContent=this._cables.length;
    div.innerHTML=!this._cables.length
      ?'<div style="font-size:13px;color:var(--txt3,#6a9aaa);padding:4px 0">Keine Kabel definiert</div>'
      :this._cables.map((c,i)=>{
          const r=this.cableMaxR[i]||0,pct=Math.min(Math.round(r*100),100);
          const bc=r>=1?'#cc2222':r>.92?'#cc4400':r>.75?'#886600':'#1a6a2a';
          return `<div style="display:flex;align-items:center;gap:8px;padding:5px 0;border-bottom:1px solid var(--bdr,#1a3050)">
            <span style="font-weight:700;color:var(--acc,#f05500);font-size:14px;min-width:22px">K${i+1}</span>
            <span style="color:var(--txt3,#6a9aaa);font-size:13px">${c.anchors.length} Anker · ${c.thickness}mm</span>
            <div style="flex:1;height:4px;background:rgba(255,255,255,.08);border-radius:2px;overflow:hidden">
              <div style="width:${pct}%;height:100%;background:${bc};border-radius:2px"></div></div>
            <span style="color:var(--txt3,#6a9aaa);font-size:12px;min-width:32px;text-align:right">${pct}%</span>
            <button onclick="window.cableSystem.selCab_f(${i});window.cableSystem.setVisible(true)"
              style="background:none;border:1px solid var(--bdr,#1a3050);color:var(--txt2,#a0bfcf);cursor:pointer;font-size:13px;padding:2px 7px;border-radius:2px" title="Bearbeiten">✏</button>
          </div>`;
        }).join('');
  }

  // ─── 9. Editor-Panel aufbauen ────────────────────────────────────────────────

  _buildErrorBanner(){
    if(document.getElementById('cs-err'))return;
    const s=document.createElement('style');
    s.textContent=`#cs-err{display:none;position:fixed;top:10px;left:50%;transform:translateX(-50%);background:rgba(180,0,0,.92);border:2px solid #cc2222;color:#fff;font:bold 13px/1 monospace;letter-spacing:2px;padding:10px 20px;text-transform:uppercase;z-index:9500;border-radius:3px;animation:cs-blink .65s ease-in-out infinite}@keyframes cs-blink{0%,100%{opacity:1}50%{opacity:.6}}`;
    document.head.appendChild(s);
    const e=document.createElement('div');e.id='cs-err';document.body.appendChild(e);
  }

  _buildPanel(){
    if(this._panel)return;

    // CSS – nutzt die App-Variablen; Override nur für Range-Slider-Größe
    const sty=document.createElement('style');
    sty.textContent=`
      #cs-panel{
        position:fixed;top:0;right:0;width:280px;height:100vh;
        background:var(--bg2,#070e1a);border-left:1px solid var(--bdr,#1a3050);
        display:none;flex-direction:column;overflow-y:auto;z-index:8500;
        font-size:14px;color:var(--txt,#c8d8e8);
        scrollbar-width:thin;scrollbar-color:var(--bg4,#0f1e2e) transparent;
      }
      #cs-panel .cs-card{
        background:var(--bg3,#0a1525);border:1px solid var(--bdr,#1a3050);
        border-radius:3px;margin:6px;padding:8px 10px;
      }
      #cs-panel .cs-head{
        display:flex;align-items:center;justify-content:space-between;margin-bottom:6px;
      }
      #cs-panel h3{
        font-size:14px;font-weight:700;color:var(--txt2,#a0bfcf);
        display:flex;align-items:center;gap:6px;margin:0;
      }
      #cs-panel .cs-close{
        background:none;border:none;color:var(--txt3,#6a9aaa);
        cursor:pointer;font-size:18px;padding:0 4px;line-height:1;
      }
      #cs-panel .cs-close:hover{color:var(--txt,#c8d8e8)}
      #cs-panel .cs-btn{
        background:var(--bg2,#070e1a);border:1px solid var(--bdr,#1a3050);
        color:var(--txt2,#a0bfcf);border-radius:2px;padding:4px 10px;
        font-size:14px;cursor:pointer;font-family:inherit;
      }
      #cs-panel .cs-btn:hover{background:var(--bg4,#0f1e2e);color:var(--txt,#c8d8e8)}
      #cs-panel .cs-lbl{
        font-size:13px;color:var(--txt3,#6a9aaa);font-weight:600;
        margin-bottom:4px;display:block;
      }
      #cs-panel select{
        width:100%;background:var(--bg2,#070e1a);
        border:1px solid var(--bdr,#1a3050);border-radius:2px;
        padding:5px 7px;font-size:14px;color:var(--txt,#c8d8e8);
        font-family:inherit;cursor:pointer;
      }
      #cs-panel select:focus{outline:none;border-color:var(--acc,#f05500)}
      /* Range-Slider – größer als Browser-Standard */
      #cs-panel input[type=range]{
        -webkit-appearance:none;width:100%;height:4px;
        background:var(--bdr,#1a3050);border-radius:2px;outline:none;cursor:pointer;
      }
      #cs-panel input[type=range]::-webkit-slider-thumb{
        -webkit-appearance:none;width:16px;height:16px;border-radius:50%;
        background:var(--acc,#f05500);border:2px solid var(--bg2,#070e1a);
        box-shadow:0 0 0 1px var(--acc,#f05500);cursor:pointer;
      }
      #cs-panel input[type=range]::-moz-range-thumb{
        width:16px;height:16px;border-radius:50%;background:var(--acc,#f05500);
        border:2px solid var(--bg2,#070e1a);cursor:pointer;
      }
      #cs-panel .cs-row{
        display:flex;align-items:center;gap:8px;margin-bottom:7px;
      }
      #cs-panel .cs-axlbl{
        font-size:13px;font-weight:700;min-width:18px;text-align:right;
      }
      #cs-panel .cs-val{
        font-size:13px;color:var(--txt2,#a0bfcf);min-width:48px;text-align:right;
      }
      #cs-panel .cs-sep{height:1px;background:var(--bdr,#1a3050);margin:6px 0}
      /* Kabel-Tabelle */
      .cs-crow{
        display:grid;grid-template-columns:24px 1fr 40px 28px 18px;
        align-items:center;gap:5px;padding:6px 7px;cursor:pointer;
        border:1px solid transparent;border-radius:2px;font-size:13px;
      }
      .cs-crow:hover{background:rgba(255,255,255,.05)}
      .cs-crow.on{background:rgba(240,85,0,.1);border-color:rgba(240,85,0,.3)}
      .cs-cnum{font-weight:700;color:var(--txt3,#6a9aaa)}.cs-crow.on .cs-cnum{color:var(--acc,#f05500)}
      .cs-cinfo{color:var(--txt3,#6a9aaa);font-size:12px}
      .cs-ctbar{height:3px;background:rgba(255,255,255,.08);border-radius:2px;overflow:hidden}
      .cs-ctpct{font-size:12px;color:var(--txt3,#6a9aaa);text-align:right}
      .cs-cdel{background:none;border:none;color:rgba(255,255,255,.2);cursor:pointer;font-size:14px;padding:0}
      .cs-cdel:hover{color:#ff4444}
      /* Anker-Chips */
      #cs-achips{display:flex;flex-wrap:wrap;gap:5px;margin:6px 0}
      .cs-chip{
        padding:4px 10px;font-size:13px;font-family:inherit;cursor:pointer;
        border:1px solid var(--bdr,#1a3050);background:var(--bg2,#070e1a);
        color:var(--txt2,#a0bfcf);border-radius:2px;transition:all .12s;
      }
      .cs-chip:hover{border-color:var(--txt2);color:var(--txt,#c8d8e8)}
      .cs-chip.on{background:var(--acc,#f05500);border-color:var(--acc,#f05500);color:#fff;font-weight:700}
      .cs-dim{opacity:.3;pointer-events:none}
      .cs-tracked{opacity:.35}
      #cs-err{display:none;position:fixed;top:10px;left:50%;transform:translateX(-50%);background:rgba(180,0,0,.92);border:2px solid #cc2222;color:#fff;font:bold 13px/1 monospace;letter-spacing:2px;padding:10px 20px;text-transform:uppercase;z-index:9500;border-radius:3px;animation:cs-blink .65s ease-in-out infinite}
    `;
    document.head.appendChild(sty);

    const p=document.createElement('div');p.id='cs-panel';
    p.innerHTML=`
      <!-- Kopfzeile -->
      <div class="cs-card" style="margin-bottom:0;border-radius:0;border-left:none;border-top:none;border-right:none">
        <div class="cs-head" style="margin-bottom:0">
          <h3>🔌 Kabel-Editor</h3>
          <button class="cs-close" onclick="window.cableSystem.setVisible(false)">✕</button>
        </div>
      </div>

      <!-- Kabel-Liste -->
      <div class="cs-card">
        <div class="cs-head">
          <h3>Kabel</h3>
          <button class="cs-btn" onclick="window.cableSystem.addCable()">+ Neu</button>
        </div>
        <div id="cs-ctable"></div>
      </div>

      <!-- Ausgewähltes Kabel -->
      <div class="cs-card">
        <h3 id="cs-detlbl" style="margin-bottom:10px">Kabel 1 · 2 Anker</h3>

        <!-- Radius -->
        <label class="cs-lbl">Radius</label>
        <div class="cs-row">
          <input type="range" id="cs-slt" min="1" max="30" value="6" step="0.5" oninput="window.cableSystem.updThick()">
          <span class="cs-val" id="cs-sltv">6 mm</span>
        </div>

        <div class="cs-sep"></div>

        <!-- Ankerpunkte -->
        <div class="cs-head" style="margin-bottom:4px">
          <h3>Ankerpunkte</h3>
          <div style="display:flex;gap:5px">
            <button class="cs-btn" onclick="window.cableSystem.addAnchor()">+</button>
            <button class="cs-btn" onclick="window.cableSystem.delAnchor()">−</button>
          </div>
        </div>
        <div id="cs-achips"></div>

        <!-- Referenz / Track -->
        <label class="cs-lbl" style="margin-top:6px">Referenz (Anhängepunkt)</label>
        <select id="cs-ref" onchange="window.cableSystem.updRef()" style="margin-bottom:8px">
          <option value="">— Welt (X/Y/Z) —</option>
        </select>

        <!-- XYZ-Koordinaten (gesperrt wenn Track aktiv) -->
        <div id="cs-xyz-block">
          <div class="cs-row">
            <span class="cs-axlbl" style="color:#ff6666">X</span>
            <input type="range" id="cs-ax" min="-2000" max="2000" value="0" step="10" oninput="window.cableSystem.updA()">
            <span class="cs-val" id="cs-axv">0</span>
          </div>
          <div class="cs-row">
            <span class="cs-axlbl" style="color:#66cc88">Y</span>
            <input type="range" id="cs-ay" min="-200" max="3000" value="0" step="10" oninput="window.cableSystem.updA()">
            <span class="cs-val" id="cs-ayv">0</span>
          </div>
          <div class="cs-row">
            <span class="cs-axlbl" style="color:#6699ff">Z</span>
            <input type="range" id="cs-az" min="-2000" max="2000" value="0" step="10" oninput="window.cableSystem.updA()">
            <span class="cs-val" id="cs-azv">0</span>
          </div>
        </div>

        <!-- Länge zum nächsten Ankerpunkt -->
        <div class="cs-row" id="cs-lrow">
          <span class="cs-axlbl" style="min-width:28px;font-size:12px;color:var(--txt3,#6a9aaa)">L→</span>
          <input type="range" id="cs-al" min="50" max="3000" value="1200" step="10" oninput="window.cableSystem.updA()">
          <span class="cs-val" id="cs-alv">1200</span>
        </div>
      </div>
    `;
    document.body.appendChild(p);
    const e=document.createElement('div');e.id='cs-err';document.body.appendChild(e);
    this._panel=p;
    this._renderTable();
    this._loadDetail();
  }

  // ─── 10. Track-Dropdown aktualisieren ────────────────────────────────────────

  _updateTrackDropdown(){
    const sel=document.getElementById('cs-ref');if(!sel)return;
    const cur=sel.value;
    sel.innerHTML='<option value="">— Welt (X/Y/Z) —</option>'+
      Object.keys(this._trackMap).map(k=>{
        const lbl=this._TRACK_LABELS[k]||k;
        return `<option value="${k}"${cur===k?' selected':''}>${lbl}</option>`;
      }).join('');
  }

  // ─── 11. Kabel-Tabelle ────────────────────────────────────────────────────────

  _renderTable(){
    const div=document.getElementById('cs-ctable');if(!div)return;
    div.innerHTML=this._cables.map((c,i)=>{
      const r=this.cableMaxR[i]||0,pct=Math.min(Math.round(r*100),100);
      const bc=r>=1?'#cc2222':r>.92?'#cc4400':r>.75?'#886600':'#1a6a2a';
      return `<div class="cs-crow${i===this.selCab?' on':''}" onclick="window.cableSystem.selCab_f(${i})">
        <span class="cs-cnum">K${i+1}</span>
        <span class="cs-cinfo">${c.anchors.length}A · ${c.thickness}mm</span>
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
    this._cables.push({thickness:6,anchors:[
      {x:0,y:up==='z'?1200:500,z:up==='z'?-400:0,segLen:1200,track:null,ox:0,oy:0,oz:0},
      {x:0,y:up==='z'?1200:500,z:up==='z'? 400:0,segLen:1200,track:null,ox:0,oy:0,oz:0}]});
    this.selCab=this._cables.length-1;this.selAnch=0;this._loadDetail();this.refresh();
  }
  delCable(i){
    if(this._cables.length<=1)return;
    this._cables.splice(i,1);this.selCab=Math.min(this.selCab,this._cables.length-1);
    this.selAnch=Math.min(this.selAnch,this._cables[this.selCab].anchors.length-1);
    this._loadDetail();this.refresh();
  }

  // ─── 12. Anker-Detail ─────────────────────────────────────────────────────────

  _loadDetail(){
    const cab=this._cables[this.selCab];if(!cab)return;
    const n=document.getElementById('cs-detlbl');
    if(n)n.textContent=`Kabel ${this.selCab+1}  ·  ${cab.anchors.length} Anker`;
    const slt=document.getElementById('cs-slt');
    if(slt){slt.value=cab.thickness;document.getElementById('cs-sltv').textContent=cab.thickness+' mm';}
    this._renderChips();this._loadAnchorEdit();
  }

  _renderChips(){
    const div=document.getElementById('cs-achips');if(!div)return;
    div.innerHTML='';
    this._cables[this.selCab].anchors.forEach((a,i)=>{
      const btn=document.createElement('button');
      btn.className='cs-chip'+(i===this.selAnch?' on':'');
      btn.textContent='A'+(i+1)+(a.track?'  🔗':'');
      btn.onclick=()=>this.selA(i);
      div.appendChild(btn);
    });
  }

  _loadAnchorEdit(){
    const a=this._cables[this.selCab].anchors[this.selAnch];if(!a)return;
    const isLast=this.selAnch===this._cables[this.selCab].anchors.length-1;
    const tracked=!!(a.track);
    const sel=document.getElementById('cs-ref');
    if(sel){this._updateTrackDropdown();sel.value=a.track||'';}
    [['cs-ax',a.x||0],['cs-ay',a.y||0],['cs-az',a.z||0]].forEach(([id,v])=>{
      const el=document.getElementById(id);if(el)el.value=v;
      const vd=document.getElementById(id+'v');if(vd)vd.textContent=v;
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
      a.x=+document.getElementById('cs-ax').value;document.getElementById('cs-axv').textContent=a.x;
      a.y=+document.getElementById('cs-ay').value;document.getElementById('cs-ayv').textContent=a.y;
      a.z=+document.getElementById('cs-az').value;document.getElementById('cs-azv').textContent=a.z;
    }
    a.segLen=+document.getElementById('cs-al').value;document.getElementById('cs-alv').textContent=a.segLen;
    this.refresh();
  }

  updThick(){
    const v=+document.getElementById('cs-slt').value;
    this._cables[this.selCab].thickness=v;document.getElementById('cs-sltv').textContent=v+' mm';this.refresh();
  }

  addAnchor(){
    const cab=this._cables[this.selCab];if(cab.anchors.length>=8)return;
    const last=cab.anchors.at(-1);
    cab.anchors.push({...last,x:last.x+200,track:null});
    this.selAnch=cab.anchors.length-1;this._renderChips();this._loadAnchorEdit();this.refresh();
  }

  delAnchor(){
    const cab=this._cables[this.selCab];if(cab.anchors.length<=2)return;
    cab.anchors.splice(this.selAnch,1);this.selAnch=Math.min(this.selAnch,cab.anchors.length-1);
    this._renderChips();this._loadAnchorEdit();this.refresh();
  }

  // ─── 13. Sichtbarkeit + Fehler-Banner ────────────────────────────────────────

  setVisible(v){
    this.visible=v;
    if(!this.viewerOnly&&this._panel)this._panel.style.display=v?'flex':'none';
    this.refresh();
  }
  toggleVisible(){this.setVisible(!this.visible);}

  _updateErrorBanner(broken){
    const el=document.getElementById('cs-err');if(!el)return;
    if(broken.length){
      el.textContent='KABEL GERISSEN — '+broken.map(b=>`K${b.ci+1} SEG ${b.seg+1}→${b.seg+2}`).join(' · ');
      el.style.display='block';
    }else el.style.display='none';
  }
}

window.cableSystem = new CableSystem();
