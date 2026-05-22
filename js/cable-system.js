/**
 * cable-system.js  –  Kabel  (RobModel Editor / RobSimul Viewer)
 *
 * RobModel:
 *   init(THREE, scene, {upAxis:'z'})
 *   setTrackObjects({tcp,a1..a6})   ← Ende rebuildRobotKinematics()
 *   refresh()                        ← in applyJointRotations()
 *
 * RobSimul:
 *   init(THREE, scene, {upAxis:'y', viewerOnly:true})
 *   loadCables(arr)                  ← in applyKinematicData()
 *
 * Standard: 1 Kabel, 6 Anker an A1–A6 (track:'a1'…'a6').
 * Länge je Segment = Gelenk-Abstand + 15% Durchhang-Puffer.
 * JSON-Schlüssel "cables": [{thickness, anchors:[{x,y,z,segLen,track?}]}]
 */

class CableSystem {

  // ─── Zustand ────────────────────────────────────────────────────────────────

  constructor() {
    this.THREE      = null;
    this.scene      = null;
    this.upAxis     = 'y';
    this.viewerOnly = false;
    this.visible    = true;   // 3D-Kabel sichtbar (Menü-Button schaltet um)
    this._cables    = [];
    this.selCab     = 0;
    this.selAnch    = 0;
    this.sceneObjs  = [];
    this.cableMaxR  = [];
    this._trackMap  = {};
    this.N_PTS      = 40;
    this._sGeoSm    = null;
    this._sGeoLg    = null;
    this._LABELS    = {
      a1:'A1',a2:'A2',a3:'A3',a4:'A4',a5:'A5',a6:'A6',tcp:'TCP'
    };
    this._gizmoObj  = null;   // Object3D, an dem TransformControls hängt
    this._tc        = null;   // TransformControls-Instanz
    this._tcDragging= false;
    this._capsules  = [];     // [{obj1,obj2,r}] – Roboter-Glieder als Kapseln
    this._tv1 = null; this._tv2 = null; this._tv3 = null; // Reuse-Vektoren
  }

  get cables() { return this._cables; }

  // ─── Initialisierung ────────────────────────────────────────────────────────

  init(THREE, scene, opts={}) {
    this.THREE      = THREE;
    this.scene      = scene;
    this.upAxis     = opts.upAxis     || 'y';
    this.viewerOnly = opts.viewerOnly || false;
    this._sGeoSm    = new THREE.SphereGeometry(7,  14, 10);
    this._sGeoLg    = new THREE.SphereGeometry(11, 16, 12);
    this._tv1       = new THREE.Vector3();
    this._tv2       = new THREE.Vector3();
    this._tv3       = new THREE.Vector3();
    this._tmpDir    = new THREE.Vector3();
    this._raycaster = new THREE.Raycaster();

    if (!this.viewerOnly) {
      this._cables = [];   // Leer – Benutzer fügt Kabel manuell hinzu
      this._injectStyles();
      setTimeout(() => { this._buildInlineEditor(); this.refresh(); }, 0);
    } else {
      this._buildErrorBanner();
    }
  }

  // ─── Track-Objekte + Auto-Länge ─────────────────────────────────────────────

  setTrackObjects(map) {
    this._trackMap = {};
    for (const [k,v] of Object.entries(map)) { if(v) this._trackMap[k]=v; }

    // Roboter-Glieder als Kapseln für Kollisionserkennung
    // Radien: grob für KUKA-Arme (mm). Kann über setCapsuleRadii() angepasst werden.
    const joints  = ['a1','a2','a3','a4','a5','a6'];
    const radii   = this._capRadii || [90,80,70,60,50,40];
    this._capsules = [];
    for (let i=0; i<joints.length-1; i++) {
      const o1=this._trackMap[joints[i]], o2=this._trackMap[joints[i+1]];
      if(!o1||!o2) continue;
      // STL-Meshes dieses Glieds (Kinder von o1, aber NICHT der Unter-Pivot o2)
      const meshList=[];
      for(const child of o1.children){
        if(child===o2) continue;
        child.traverse(obj=>{ if(obj.isMesh&&obj.geometry) meshList.push(obj); });
      }
      this._capsules.push({obj1:o1, obj2:o2, r:radii[i]||60, meshes:meshList});
    }

    this._updateRefDropdown();
    this.autoLength();
    this.refresh();
  }

  /** Kapsel-Radien der Roboter-Glieder überschreiben (mm).
   *  Beispiel: cableSystem.setCapsuleRadii([100,90,80,70,60,50])  */
  setCapsuleRadii(arr) {
    this._capRadii = arr;
    // Kapseln sofort neu berechnen falls trackMap schon gesetzt
    if(Object.keys(this._trackMap).length) this.setTrackObjects(this._trackMap);
  }

  /**
   * autoLength(): Setzt segLen jedes Segments auf  Gelenk-Abstand × (1 + slack).
   * Nur wenn aktueller segLen < Abstand (Kabel wäre gespannt).
   */
  autoLength(slack=0.15) {
    this._cables.forEach((cab, ci) => {
      for (let ai=0; ai<cab.anchors.length-1; ai++) {
        const p1 = this._anchorWorldPos(ci, ai);
        const p2 = this._anchorWorldPos(ci, ai+1);
        const dist = p1.distanceTo(p2);
        if (dist > 1) {
          const minLen = Math.ceil(dist * (1+slack));
          if (cab.anchors[ai].segLen < minLen) cab.anchors[ai].segLen = minLen;
        }
      }
    });
    this._loadAnchorEdit();   // L→-Slider aktualisieren
    this._renderTable();
  }

  // ─── JSON laden (RobSimul) ──────────────────────────────────────────────────

  loadCables(arr) {
    if (!Array.isArray(arr)) return;
    this._cables = JSON.parse(JSON.stringify(arr));
    this.selCab  = 0; this.selAnch = 0;
    this.visible = arr.length > 0;
    this._updateSideCard();
    this.refresh();
  }

  // ─── Ankerpunkt-Weltposition ────────────────────────────────────────────────

  _anchorWorldPos(ci, ai) {
    const T=this.THREE, a=this._cables[ci].anchors[ai];
    if (a.track && this._trackMap[a.track]) {
      const obj=this._trackMap[a.track], wp=new T.Vector3();
      obj.getWorldPosition(wp);
      if (a.ox||a.oy||a.oz) {
        const wq=new T.Quaternion(); obj.getWorldQuaternion(wq);
        wp.add(new T.Vector3(a.ox||0,a.oy||0,a.oz||0).applyQuaternion(wq));
      }
      return wp;
    }
    return new T.Vector3(a.x||0,a.y||0,a.z||0);
  }

  // ─── Physik ─────────────────────────────────────────────────────────────────

  _resolveFloor(p) {
    if(this.upAxis==='z'&&p.z<0){p.z=0;return true;}
    if(this.upAxis==='y'&&p.y<0){p.y=0;return true;}
    return false;
  }

  /** Kapsel-Kollision (kein Heap-Alloc, nutzt pre-allozierte Vektoren).
   *  Schiebt p heraus wenn es sich innerhalb der Kapsel zwischen p1 und p2 befindet. */
  _resolveCapsule(p, p1, p2, radius) {
    this._tv1.subVectors(p2, p1);
    const len2 = this._tv1.lengthSq();
    if (len2 < 1e-8) return false;
    this._tv2.subVectors(p, p1);
    const t = Math.max(0, Math.min(1, this._tv2.dot(this._tv1) / len2));
    this._tv3.copy(p1).addScaledVector(this._tv1, t);  // nächster Punkt auf Segment
    this._tv2.subVectors(p, this._tv3);                // Richtung von Segment zu p
    const dist = this._tv2.length();
    if (dist < radius && dist > 1e-8) {
      p.copy(this._tv3).addScaledVector(this._tv2.normalize(), radius + 2);
      return true;
    }
    return false;
  }

  _simCable(p1,p2,L) {
    const T=this.THREE,chord=p1.distanceTo(p2);
    if(chord>=L-0.01)return{taut:true, ratio:Math.min(chord/L,1),pts:null};
    if(chord<0.5)     return{taut:false,ratio:0,pts:null};
    const N=this.N_PTS,rL=L/N,pos=[],old=[];
    for(let i=0;i<=N;i++){const p=p1.clone().lerp(p2,i/N);pos.push(p);old.push(p.clone());}
    const GY=0.14,DM=0.85,STEPS=180,CP=6,up=this.upAxis;

    // Kapsel-Positionen einmalig berechnen + Mesh-Listen mitführen
    const caps=this._capsules.map(c=>{
      const q1=new T.Vector3(),q2=new T.Vector3();
      c.obj1.getWorldPosition(q1); c.obj2.getWorldPosition(q2);
      return{p1:q1,p2:q2,r:c.r,meshes:c.meshes||[]};
    });

    for(let s=0;s<STEPS;s++){
      for(let i=1;i<N;i++){const v=pos[i].clone().sub(old[i]).multiplyScalar(DM);old[i].copy(pos[i]);pos[i].add(v);if(up==='z')pos[i].z-=GY;else pos[i].y-=GY;}
      for(let c=0;c<CP;c++){
        for(let i=0;i<N;i++){const d=pos[i].distanceTo(pos[i+1]);if(d<1e-4)continue;const cr=new T.Vector3().subVectors(pos[i+1],pos[i]).multiplyScalar((d-rL)/(d*2));if(i>0)pos[i].add(cr);if(i<N)pos[i+1].sub(cr);}
        pos[0].copy(p1);pos[N].copy(p2);
        for(let i=1;i<N;i++){
          this._resolveFloor(pos[i]);
          for(const cap of caps) this._resolveCapsule(pos[i],cap.p1,cap.p2,cap.r);
        }
      }
    }

    // ── Post-PBD: feinere Mesh-Kollision (läuft einmal nach PBD-Konvergenz) ──
    const hasMeshes=caps.some(c=>c.meshes.length>0);
    if(this._raycaster&&hasMeshes){
      // Alle relevanten Meshes kurz auf DoubleSide schalten (für Innen-Erkennung)
      const allM=caps.flatMap(c=>c.meshes);
      const origS=allM.map(m=>Array.isArray(m.material)?null:m.material?.side);
      allM.forEach((m,i)=>{if(origS[i]!=null)m.material.side=T.DoubleSide;});

      for(let mc=0;mc<3;mc++){
        for(let i=1;i<N;i++){
          for(const cap of caps){
            if(!cap.meshes.length)continue;
            this._resolveMeshCollision(pos[i],cap,8);
          }
        }
        // Längen-Constraints nach Mesh-Korrektur erneut anwenden
        for(let c=0;c<4;c++){
          for(let i=0;i<N;i++){const d=pos[i].distanceTo(pos[i+1]);if(d<1e-4)continue;const cr=new T.Vector3().subVectors(pos[i+1],pos[i]).multiplyScalar((d-rL)/(d*2));if(i>0)pos[i].add(cr);if(i<N)pos[i+1].sub(cr);}
          pos[0].copy(p1);pos[N].copy(p2);
        }
      }

      // Material-Side wiederherstellen
      allM.forEach((m,i)=>{if(origS[i]!=null)m.material.side=origS[i];});
    }

    return{taut:false,ratio:chord/L,pts:pos};
  }

  /**
   * _resolveMeshCollision: Schiebt einen Kabelpunkt aus einem STL-Mesh heraus.
   * Strahlt vom Punkt zur Kapsel-Achse; trifft er eine Vorderfläche (außen) oder
   * Rückfläche (innen) des Meshes, wird der Punkt auf die Oberfläche + clearance gesetzt.
   */
  _resolveMeshCollision(p, cap, clearance){
    const T=this.THREE;
    // Vorfilter: Abstand zur Kapselachse
    this._tv1.subVectors(cap.p2,cap.p1);
    const len2=this._tv1.lengthSq(); if(len2<1)return;
    this._tv2.subVectors(p,cap.p1);
    const t=Math.max(0,Math.min(1,this._tv2.dot(this._tv1)/len2));
    this._tv3.copy(cap.p1).addScaledVector(this._tv1,t);
    if(p.distanceTo(this._tv3)>cap.r*1.8)return;

    // Richtung: Punkt → Kapselachse (nach innen)
    this._tmpDir.subVectors(this._tv3,p);
    const d=this._tmpDir.length(); if(d<0.1)return;
    this._tmpDir.divideScalar(d);

    this._raycaster.set(p,this._tmpDir);
    this._raycaster.near=0;
    this._raycaster.far=d+clearance;
    const hits=this._raycaster.intersectObjects(cap.meshes,false);
    if(!hits.length)return;

    const hit=hits[0];
    const fn=new T.Vector3().copy(hit.face.normal)
      .transformDirection(hit.object.matrixWorld).normalize();
    const inside=fn.dot(this._tmpDir)>0; // Rückfläche getroffen → Punkt ist innen

    if(inside){
      // Innen: auf Oberfläche + clearance nach außen (entgegen Strahlrichtung)
      p.copy(hit.point).addScaledVector(this._tmpDir,-clearance);
    } else if(hit.distance<clearance+2){
      // Außen aber zu nah: weg von der Fläche schieben
      p.copy(hit.point).addScaledVector(fn,clearance);
    }
  }

  // ─── 3D-Rendering ───────────────────────────────────────────────────────────

  _tCol(r){
    if(r>=1.0)return{color:0xff2222,emissive:0x440000,emissiveIntensity:.6};
    if(r>.92) return{color:0xff5500,emissive:0x110000,emissiveIntensity:.2};
    if(r>.75) return{color:0x885500,emissive:0,emissiveIntensity:0};
    return         {color:0xd0e8f8,emissive:0,emissiveIntensity:0};
  }

  _tube(pts,col,r){
    const T=this.THREE;
    const g=new T.TubeGeometry(new T.CatmullRomCurve3(pts,false,'centripetal'),Math.max(pts.length,5),r,8,false);
    return new T.Mesh(g,new T.MeshStandardMaterial({roughness:.8,metalness:.1,...col}));
  }

  refresh() {
    if(!this.scene)return;
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
        const mat=new T.MeshStandardMaterial({color:col,emissive:col,emissiveIntensity:sel?.8:.25,roughness:.3,metalness:.5});
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
    if(!this.viewerOnly){this._renderTable();this._updateSideCard();}
    this._updateErrorBanner(broken);
  }

  // ─── Seitenleiste: Kurzübersicht (#kabel-rows) ──────────────────────────────

  _updateSideCard(){
    const div=document.getElementById('kabel-rows');if(!div)return;
    const b=document.getElementById('kabelBadge');if(b)b.textContent=this._cables.length;
    div.innerHTML=!this._cables.length
      ?'<div style="font-size:13px;color:var(--txt3);padding:3px 0">Kein Kabel — + drücken</div>'
      :this._cables.map((c,i)=>{
        const r=this.cableMaxR[i]||0,pct=Math.min(Math.round(r*100),100);
        const bc=r>=1?'#cc2222':r>.92?'#cc4400':r>.75?'#886600':'#1a6a2a';
        const on=i===this.selCab?'background:rgba(240,85,0,.12);border-color:rgba(240,85,0,.35)':'';
        return `<div style="display:flex;align-items:center;gap:7px;padding:5px 6px;
                  border:1px solid var(--bdr);border-radius:2px;cursor:pointer;margin-bottom:3px;${on}"
                  onclick="window.cableSystem.selCab_f(${i})">
          <span style="font-weight:700;color:${i===this.selCab?'var(--acc)':'var(--txt3)'};font-size:14px;min-width:24px">K${i+1}</span>
          <span style="color:var(--txt3);font-size:13px;flex:1">${c.anchors.length}A · ${c.thickness}mm</span>
          <div style="width:60px;height:3px;background:rgba(255,255,255,.08);border-radius:2px;overflow:hidden">
            <div style="width:${pct}%;height:100%;background:${bc};border-radius:2px"></div></div>
          <span style="color:var(--txt3);font-size:12px;min-width:34px;text-align:right">${pct}%</span>
          <button onclick="event.stopPropagation();window.cableSystem.delCable(${i})"
            style="background:none;border:none;color:rgba(255,255,255,.2);cursor:pointer;font-size:14px;padding:0 2px">×</button>
        </div>`;
      }).join('');
  }

  // ─── Inline-Editor aufbauen (#cs-inline-editor) ─────────────────────────────

  _buildInlineEditor(){
    const container=document.getElementById('cs-inline-editor');
    if(!container||container.dataset.built==='1')return;
    container.dataset.built='1';
    container.style.cssText='border-top:1px solid var(--bdr);margin-top:6px;padding-top:8px';

    container.innerHTML=`
      <!-- Ausgewähltes Kabel -->
      <div id="cs-det-head" style="font-size:13px;font-weight:700;color:var(--txt2);margin-bottom:8px;letter-spacing:.5px"></div>

      <!-- Radius -->
      <div style="margin-bottom:8px">
        <div style="font-size:12px;color:var(--txt3);margin-bottom:3px">R</div>
        <div style="display:flex;align-items:center;gap:8px">
          <input type="range" id="cs-slt" min="1" max="50" value="6" step="0.5" oninput="window.cableSystem.updThick()" style="flex:1">
          <span id="cs-sltv" style="font-size:13px;color:var(--txt2);min-width:44px;text-align:right">6 mm</span>
        </div>
      </div>

      <!-- Ankerpunkte -->
      <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:5px">
        <span style="font-size:12px;color:var(--txt3);font-weight:700;letter-spacing:.5px">ANKERPUNKTE</span>
        <div style="display:flex;gap:4px">
          <button onclick="window.cableSystem.addAnchor()" class="cs-ibtn">+</button>
          <button onclick="window.cableSystem.delAnchor()" class="cs-ibtn">−</button>
        </div>
      </div>
      <div id="cs-achips" style="display:flex;flex-wrap:wrap;gap:4px;margin-bottom:8px"></div>

      <!-- Referenz -->
      <div style="font-size:12px;color:var(--txt3);margin-bottom:3px">Ref</div>
      <select id="cs-ref" onchange="window.cableSystem.updRef()" style="width:100%;background:var(--bg2);border:1px solid var(--bdr);border-radius:2px;padding:5px 7px;font-size:14px;color:var(--txt);font-family:inherit;cursor:pointer;margin-bottom:8px">
        <option value="">— Welt (X/Y/Z) —</option>
      </select>

      <!-- XYZ (immer editierbar; bei Track = Offset vom Gelenkpunkt) -->
      <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:3px">
        <div style="font-size:12px;color:var(--txt3)">X / Y / Z</div>
        <div id="cs-xyz-hint" style="font-size:11px;color:var(--txt3);font-style:italic"></div>
      </div>
      <div id="cs-xyz-block">
        <div style="display:flex;align-items:center;gap:8px;margin-bottom:5px">
          <span style="font-size:13px;font-weight:700;color:#ff6666;min-width:14px;text-align:right">X</span>
          <input type="range" id="cs-ax" min="-2000" max="2000" value="0" step="10" oninput="window.cableSystem.updA()" style="flex:1">
          <span id="cs-axv" style="font-size:13px;color:var(--txt2);min-width:44px;text-align:right">0</span>
        </div>
        <div style="display:flex;align-items:center;gap:8px;margin-bottom:5px">
          <span style="font-size:13px;font-weight:700;color:#66cc88;min-width:14px;text-align:right">Y</span>
          <input type="range" id="cs-ay" min="-200" max="3000" value="0" step="10" oninput="window.cableSystem.updA()" style="flex:1">
          <span id="cs-ayv" style="font-size:13px;color:var(--txt2);min-width:44px;text-align:right">0</span>
        </div>
        <div style="display:flex;align-items:center;gap:8px;margin-bottom:5px">
          <span style="font-size:13px;font-weight:700;color:#6699ff;min-width:14px;text-align:right">Z</span>
          <input type="range" id="cs-az" min="-2000" max="2000" value="0" step="10" oninput="window.cableSystem.updA()" style="flex:1">
          <span id="cs-azv" style="font-size:13px;color:var(--txt2);min-width:44px;text-align:right">0</span>
        </div>
      </div>

      <!-- L→ (immer sichtbar, beim letzten Anker deaktiviert) -->
      <div id="cs-lrow">
        <div style="font-size:12px;color:var(--txt3);margin-bottom:3px">L→ <span id="cs-ldist" style="color:var(--txt3);font-size:11px"></span></div>
        <div style="display:flex;align-items:center;gap:8px">
          <input type="range" id="cs-al" min="10" max="3000" value="1200" step="10" oninput="window.cableSystem.updA()" style="flex:1">
          <span id="cs-alv" style="font-size:13px;color:var(--txt2);min-width:44px;text-align:right">1200</span>
        </div>
      </div>`;

    this._renderTable();
    this._loadDetail();
  }

  // ─── CSS (Range-Slider-Größe, Anker-Chips) ──────────────────────────────────

  _injectStyles(){
    if(document.getElementById('cs-styles'))return;
    const s=document.createElement('style');s.id='cs-styles';
    s.textContent=`
      /* Range-Slider – app-konform, großer Thumb */
      #cs-inline-editor input[type=range]{
        -webkit-appearance:none;width:100%;height:4px;
        background:var(--bdr,#1a3050);border-radius:2px;outline:none;cursor:pointer;
      }
      #cs-inline-editor input[type=range]::-webkit-slider-thumb{
        -webkit-appearance:none;width:16px;height:16px;border-radius:50%;
        background:var(--acc,#f05500);border:2px solid var(--bg2,#070e1a);
        box-shadow:0 0 0 1px var(--acc,#f05500);cursor:pointer;
      }
      #cs-inline-editor input[type=range]::-moz-range-thumb{
        width:16px;height:16px;border-radius:50%;
        background:var(--acc,#f05500);border:2px solid var(--bg2,#070e1a);cursor:pointer;
      }
      #cs-inline-editor input[type=range].dim::-webkit-slider-thumb{background:var(--bdr,#1a3050);box-shadow:none;}
      /* Kleine Inline-Buttons */
      .cs-ibtn{
        background:var(--bg2,#070e1a);border:1px solid var(--bdr,#1a3050);
        color:var(--txt2,#a0bfcf);border-radius:2px;padding:3px 10px;
        font-size:14px;cursor:pointer;font-family:inherit;
      }
      .cs-ibtn:hover{background:var(--bg4,#0f1e2e);color:var(--txt,#c8d8e8)}
      /* Anker-Chips */
      .cs-chip{
        padding:4px 10px;font-size:13px;font-family:inherit;cursor:pointer;
        border:1px solid var(--bdr,#1a3050);background:var(--bg2,#070e1a);
        color:var(--txt2,#a0bfcf);border-radius:2px;
      }
      .cs-chip:hover{border-color:var(--txt2);color:var(--txt,#c8d8e8)}
      .cs-chip.on{background:var(--acc,#f05500);border-color:var(--acc,#f05500);color:#fff;font-weight:700}
      /* Kabel-Listzeilen */
      .cs-crow{
        display:grid;grid-template-columns:24px 1fr 44px 32px 18px;
        align-items:center;gap:5px;padding:6px 7px;cursor:pointer;
        border:1px solid transparent;border-radius:2px;font-size:13px;
      }
      .cs-crow:hover{background:rgba(255,255,255,.04)}
      .cs-crow.on{background:rgba(240,85,0,.1);border-color:rgba(240,85,0,.3)!important}
      .cs-cnum{font-weight:700;color:var(--txt3)}.cs-crow.on .cs-cnum{color:var(--acc)}
      .cs-cinfo{color:var(--txt3);font-size:12px}
      .cs-ctbar{height:3px;background:rgba(255,255,255,.08);border-radius:2px;overflow:hidden}
      .cs-ctpct{font-size:12px;color:var(--txt3);text-align:right}
      .cs-cdel{background:none;border:none;color:rgba(255,255,255,.2);cursor:pointer;font-size:14px;padding:0}
      .cs-cdel:hover{color:#ff4444}
      /* Fehler-Banner */
      #cs-err{display:none;position:fixed;top:10px;left:50%;transform:translateX(-50%);
        background:rgba(180,0,0,.92);border:2px solid #cc2222;color:#fff;
        font:bold 13px/1 monospace;letter-spacing:2px;padding:10px 20px;
        text-transform:uppercase;z-index:9500;border-radius:3px;
        animation:cs-blink .65s ease-in-out infinite}
      @keyframes cs-blink{0%,100%{opacity:1}50%{opacity:.6}}
    `;
    document.head.appendChild(s);
    const e=document.createElement('div');e.id='cs-err';document.body.appendChild(e);
  }

  // ─── Editor-UI: Kabel-Tabelle ────────────────────────────────────────────────

  _renderTable(){
    // Nur wenn inline Editor existiert
    const container=document.getElementById('cs-inline-editor');if(!container)return;
    // Kurzübersicht in kabel-rows
    this._updateSideCard();
    // Detail-Header
    const dh=document.getElementById('cs-det-head');
    if(dh&&this._cables[this.selCab]){
      const cab=this._cables[this.selCab];
      dh.textContent=`KABEL ${this.selCab+1}  ·  ${cab.anchors.length} ANKER`;
    }
  }

  // ─── Editor-UI: Anker-Detail ─────────────────────────────────────────────────

  _loadDetail(){
    const cab=this._cables[this.selCab];
    const dh=document.getElementById('cs-det-head');
    if(!cab){
      if(dh)dh.textContent='';
      const achips=document.getElementById('cs-achips');if(achips)achips.innerHTML='';
      return;
    }
    if(dh)dh.textContent=`KABEL ${this.selCab+1}  ·  ${cab.anchors.length} ANKER`;
    const slt=document.getElementById('cs-slt');
    if(slt){slt.value=cab.thickness;document.getElementById('cs-sltv').textContent=cab.thickness+' mm';}
    this._renderChips();
    this._loadAnchorEdit();
  }

  _nextSLabel(){
    // Zählt vorhandene S-Anker über alle Kabel und gibt nächste freie Nummer zurück
    let max=0;
    this._cables.forEach(cab=>cab.anchors.forEach(a=>{
      if(a.label&&/^S(\d+)$/.test(a.label)) max=Math.max(max,+a.label.slice(1));
    }));
    return 'S'+(max+1);
  }

  _renderChips(){
    const div=document.getElementById('cs-achips');if(!div)return;
    div.innerHTML='';
    this._cables[this.selCab].anchors.forEach((a,i)=>{
      const btn=document.createElement('button');
      btn.className='cs-chip'+(i===this.selAnch?' on':'');
      // Priorität: eigenes Label > Track-Name > Fallback
      const lbl=a.label||(a.track?this._LABELS[a.track]||a.track:null)||`A${i+1}`;
      btn.textContent=lbl;
      btn.onclick=()=>this.selA(i);
      div.appendChild(btn);
    });
  }

  _updateRefDropdown(){
    const sel=document.getElementById('cs-ref');if(!sel)return;
    const cur=sel.value;
    const longLabels={
      a1:'A1 – Achse 1',a2:'A2 – Achse 2',a3:'A3 – Achse 3',
      a4:'A4 – Achse 4',a5:'A5 – Achse 5',a6:'A6 – Flansch (A6)',
      tcp:'TCP – Werkzeugspitze'
    };
    sel.innerHTML='<option value="">— Welt (X/Y/Z) —</option>'+
      Object.keys(this._trackMap).map(k=>
        `<option value="${k}"${cur===k?' selected':''}>${longLabels[k]||k}</option>`
      ).join('');
  }

  _loadAnchorEdit(){
    const cab=this._cables[this.selCab];if(!cab)return;
    const a=cab.anchors[this.selAnch];if(!a)return;
    const isLast=this.selAnch===cab.anchors.length-1;
    const tracked=!!(a.track);

    // Referenz-Dropdown
    const sel=document.getElementById('cs-ref');
    if(sel){this._updateRefDropdown();sel.value=a.track||'';}

    // XYZ: immer editierbar.
    // Bei Track → zeigt/editiert ox/oy/oz (Offset vom Gelenkpunkt im lokalen Raum)
    // Ohne Track → zeigt/editiert x/y/z (absolute Weltkoordinaten)
    const hint=document.getElementById('cs-xyz-hint');
    if(hint)hint.textContent=tracked?'(Offset vom Gelenkpunkt)':'(Weltkoordinaten)';
    const xyz=document.getElementById('cs-xyz-block');
    if(xyz){xyz.style.opacity='1';xyz.style.pointerEvents='auto';}
    const vals=tracked
      ?[['cs-ax',a.ox||0],['cs-ay',a.oy||0],['cs-az',a.oz||0]]
      :[['cs-ax',a.x||0], ['cs-ay',a.y||0], ['cs-az',a.z||0]];
    vals.forEach(([id,v])=>{
      const el=document.getElementById(id);if(el)el.value=v;
      const vd=document.getElementById(id+'v');if(vd)vd.textContent=v;
    });

    // L→ (letzter Anker: deaktiviert)
    const lrow=document.getElementById('cs-lrow');
    if(lrow)lrow.style.opacity=isLast?'0.3':'1';
    if(lrow)lrow.style.pointerEvents=isLast?'none':'auto';
    const al=document.getElementById('cs-al');if(al)al.value=a.segLen;
    const alv=document.getElementById('cs-alv');if(alv)alv.textContent=a.segLen;

    // Aktueller Abstand zum nächsten Anker als Hinweis
    const ld=document.getElementById('cs-ldist');
    if(ld&&!isLast){
      const p1=this._anchorWorldPos(this.selCab,this.selAnch);
      const p2=this._anchorWorldPos(this.selCab,this.selAnch+1);
      const dist=Math.round(p1.distanceTo(p2));
      ld.textContent=dist>0?`(Abstand: ${dist} mm)`:'';
    }else if(ld)ld.textContent='';
  }

  // ─── Event-Handler ───────────────────────────────────────────────────────────

  selCab_f(i){
    this.selCab=i;this.selAnch=Math.min(this.selAnch,this._cables[i].anchors.length-1);
    this._loadDetail();this.refresh();
  }

  selA(i){
    this.selAnch=i;
    this._renderChips();
    this._loadAnchorEdit();
    this._positionGimbal();
    this.refresh();
  }

  addCable(){
    if(this._cables.length>=8)return;
    const jointKeys=['a1','a2','a3','a4','a5','a6'];
    const hasJoints=jointKeys.some(k=>!!this._trackMap[k]);
    let newCable;
    if(hasJoints){
      // Standard: 6 Anker an A1–A6
      newCable={
        thickness:6,
        anchors:jointKeys.map(t=>({
          x:0,y:0,z:0,ox:0,oy:0,oz:0,segLen:200,
          track:this._trackMap[t]?t:null
        }))
      };
    } else {
      // Fallback ohne Roboter: 2 Weltpunkte
      const up=this.upAxis;
      newCable={thickness:6,anchors:[
        {x:0,y:up==='z'?1200:500,z:up==='z'?-400:0,segLen:800,track:null,ox:0,oy:0,oz:0},
        {x:0,y:up==='z'?1200:500,z:up==='z'? 400:0,segLen:800,track:null,ox:0,oy:0,oz:0}]};
    }
    this._cables.push(newCable);
    this.selCab=this._cables.length-1;this.selAnch=0;
    if(hasJoints)this.autoLength();
    this._loadDetail();this.refresh();
  }

  delCable(i){
    if(this._cables.length<=1)return;
    this._cables.splice(i,1);
    this.selCab=Math.min(this.selCab,this._cables.length-1);
    this.selAnch=Math.min(this.selAnch,this._cables[this.selCab].anchors.length-1);
    this._loadDetail();this.refresh();
  }

  updRef(){
    const a=this._cables[this.selCab].anchors[this.selAnch];
    a.track=document.getElementById('cs-ref').value||null;
    // Auto-Länge für dieses Segment aktualisieren
    const p1=this._anchorWorldPos(this.selCab,this.selAnch);
    const p2=this._anchorWorldPos(this.selCab,this.selAnch+1<this._cables[this.selCab].anchors.length?this.selAnch+1:this.selAnch);
    const dist=p1.distanceTo(p2);
    if(dist>1&&a.segLen<dist*1.1)a.segLen=Math.ceil(dist*1.15);
    this._renderChips();this._loadAnchorEdit();this.refresh();
  }

  updA(){
    const a=this._cables[this.selCab].anchors[this.selAnch];
    const tracked=!!(a.track);
    // XYZ: schreibt ox/oy/oz (Offset) wenn Track aktiv, sonst x/y/z (Welt)
    const xv=+document.getElementById('cs-ax').value;
    const yv=+document.getElementById('cs-ay').value;
    const zv=+document.getElementById('cs-az').value;
    document.getElementById('cs-axv').textContent=xv;
    document.getElementById('cs-ayv').textContent=yv;
    document.getElementById('cs-azv').textContent=zv;
    if(tracked){a.ox=xv;a.oy=yv;a.oz=zv;}
    else       {a.x=xv; a.y=yv; a.z=zv;}
    a.segLen=+document.getElementById('cs-al').value;document.getElementById('cs-alv').textContent=a.segLen;
    this._loadAnchorEdit();
    this._positionGimbal();
    this.refresh();
  }

  updThick(){
    const v=+document.getElementById('cs-slt').value;
    this._cables[this.selCab].thickness=v;document.getElementById('cs-sltv').textContent=v+' mm';
    this.refresh();
  }

  addAnchor(){
    const cab=this._cables[this.selCab];if(!cab||cab.anchors.length>=8)return;
    const src=cab.anchors[this.selAnch];
    // Neuer Anker: kein Track, eigenes S-Label, nach dem selektierten einfügen
    const neu={
      x:src.x||0, y:src.y||0, z:src.z||0,
      ox:0, oy:0, oz:0,
      segLen:src.segLen||800,
      track:null,
      label:this._nextSLabel()
    };
    cab.anchors.splice(this.selAnch+1, 0, neu);
    this.selAnch=this.selAnch+1;
    this._renderChips();this._loadAnchorEdit();this.refresh();
  }

  delAnchor(){
    const cab=this._cables[this.selCab];if(cab.anchors.length<=2)return;
    cab.anchors.splice(this.selAnch,1);
    this.selAnch=Math.min(this.selAnch,cab.anchors.length-1);
    this._renderChips();this._loadAnchorEdit();this.refresh();
  }

  // ─── Sichtbarkeit (3D-Kabel, nicht Editor) ───────────────────────────────────

  setVisible(v){
    this.visible=v;
    if(this._tc){this._tc.visible=false;if(this._gizmoObj)this._gizmoObj.visible=false;}
    this.refresh();
    if(v)this._positionGimbal();
  }
  toggleVisible(){this.setVisible(!this.visible);}

  // ─── Gimbal (TransformControls) ─────────────────────────────────────────────
  // setGimbalContext() wird aus app.js nach init3d() aufgerufen:
  //   window.cableSystem.setGimbalContext({THREE, camera, renderer, orbitControls, TransformControls})
  //
  // Zeigt den Gimbal am aktuell gewählten Ankerpunkt.
  // Bei Tracked-Ankern wird die Verschiebung als lokaler Offset (ox/oy/oz) gespeichert.

  setGimbalContext({THREE: T, camera, renderer, orbitControls, TransformControls: TC}){
    if(!T||!camera||!renderer||!TC)return;

    this._gizmoObj=new T.Object3D();
    this._gizmoObj.visible=false;
    this.scene.add(this._gizmoObj);

    this._tc=new TC(camera, renderer.domElement);
    this._tc.setMode('translate');
    this._tc.attach(this._gizmoObj);
    this._tc.visible=false;
    this.scene.add(this._tc);

    // Orbit-Controls beim Ziehen deaktivieren
    this._tc.addEventListener('dragging-changed', e=>{
      if(orbitControls) orbitControls.enabled=!e.value;
      this._tcDragging=e.value;
    });

    // Ankerpunkt-Daten aus Gimbal-Position lesen
    this._tc.addEventListener('objectChange', ()=>this._gizmoToAnchor());
  }

  _positionGimbal(){
    if(!this._tc||!this._gizmoObj)return;
    if(!this.visible||!this._cables.length){
      this._tc.visible=false; this._gizmoObj.visible=false; return;
    }
    const pos=this._anchorWorldPos(this.selCab,this.selAnch);
    this._gizmoObj.position.copy(pos);
    this._gizmoObj.visible=true;
    this._tc.visible=true;
  }

  _gizmoToAnchor(){
    if(!this._cables[this.selCab])return;
    const T=this.THREE;
    const a=this._cables[this.selCab].anchors[this.selAnch];
    const newWorld=this._gizmoObj.position.clone();

    if(a.track&&this._trackMap[a.track]){
      // Welt-Position → lokaler Offset im Gelenkraum
      const obj=this._trackMap[a.track];
      const jointWP=new T.Vector3(); obj.getWorldPosition(jointWP);
      const jointWQ=new T.Quaternion(); obj.getWorldQuaternion(jointWQ);
      const localOff=newWorld.clone().sub(jointWP).applyQuaternion(jointWQ.clone().invert());
      a.ox=Math.round(localOff.x);
      a.oy=Math.round(localOff.y);
      a.oz=Math.round(localOff.z);
    } else {
      a.x=Math.round(newWorld.x);
      a.y=Math.round(newWorld.y);
      a.z=Math.round(newWorld.z);
    }
    this._loadAnchorEdit();
    this.refresh();
  }

  _buildErrorBanner(){
    if(document.getElementById('cs-err'))return;
    this._injectStyles();
  }

  _updateErrorBanner(broken){
    const el=document.getElementById('cs-err');if(!el)return;
    if(broken.length){
      el.textContent='KABEL GERISSEN — '+broken.map(b=>`K${b.ci+1} SEG ${b.seg+1}→${b.seg+2}`).join(' · ');
      el.style.display='block';
    }else el.style.display='none';
  }
}

window.cableSystem = new CableSystem();
