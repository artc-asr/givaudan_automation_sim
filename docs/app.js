// Shelf tray demo replay: plays back a run recorded by givaudan_bringup/record_run.py
// (built into data/ by tools/web_replay/build.py). Adapted from the tccc_hmlv_automation_sim
// replay. ROS frames are z-up, so the whole view is z-up (camera.up = +z). One replay per
// demo (data/<id>/), listed in data/index.json; the steps come with the run (run.phase_defs).
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { mergeGeometries, toCreasedNormals } from "three/examples/jsm/utils/BufferGeometryUtils.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader.js";
import URDFLoader from "urdf-loader";

const params = new URLSearchParams(location.search);
let DATA = "data/task1/";
const LEAD_IN = 1.5;          // s of the recording shown before the first step

const $ = (id) => document.getElementById(id);
// revalidate the data (a new recording keeps its URLs)
const getJSON = (url) => fetch(url, { cache: "no-cache" }).then((r) => {
  if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
  return r.json();
});
const fmt = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);

// ------------------------------------------------------------------ three.js setup
const canvas = $("canvas");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0a0d12);
scene.fog = new THREE.Fog(0x0a0d12, 10, 24);

const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 100);
camera.up.set(0, 0, 1);
// looking into the aisle from its end, slightly above the shelf being worked
let HOME_VIEW = { pos: new THREE.Vector3(-1.6, -3.2, 2.4), target: new THREE.Vector3(0.35, 0, 1.15) };
camera.position.copy(HOME_VIEW.pos);
const controls = new OrbitControls(camera, canvas);
controls.target.copy(HOME_VIEW.target);
controls.enableDamping = true;
controls.maxPolarAngle = Math.PI * 0.495;
controls.minDistance = 0.3;
controls.maxDistance = 15;

scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x1a1d22, 1.1));
const sun = new THREE.DirectionalLight(0xffffff, 1.5);
sun.position.set(-2.5, -3, 6);
sun.target.position.set(0, 0, 1);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -3, right: 3, top: 3, bottom: -3, near: 1, far: 15 });
sun.shadow.bias = -0.0005;
scene.add(sun, sun.target);

const floor = new THREE.Mesh(new THREE.PlaneGeometry(40, 40),
  new THREE.MeshStandardMaterial({ color: 0x151a21, roughness: 0.95 }));
floor.receiveShadow = true;
floor.position.z = -0.001;
scene.add(floor);
const grid = new THREE.GridHelper(40, 80, 0x2b3542, 0x1c232d);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

function resize() {
  const { clientWidth: w, clientHeight: h } = canvas.parentElement;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(canvas.parentElement);

// ------------------------------------------------------------------ scene (world.sdf)
const liveModels = {};        // model name -> { group }

function primGeometry(p) {
  if (p.type === "box") return new THREE.BoxGeometry(...p.size);
  if (p.type === "sphere") return new THREE.SphereGeometry(p.r, 20, 14);
  // three's cylinder runs along y; SDF's along z
  return new THREE.CylinderGeometry(p.r, p.r, p.h, 20).rotateX(Math.PI / 2);
}
const primMatrix = (p) => new THREE.Matrix4().compose(
  new THREE.Vector3(...p.pos), new THREE.Quaternion(...p.quat), new THREE.Vector3(1, 1, 1));
const material = (rgba) => new THREE.MeshStandardMaterial({
  color: new THREE.Color().setRGB(rgba[0], rgba[1], rgba[2], THREE.SRGBColorSpace),
  roughness: 0.75, metalness: 0.05,
  transparent: rgba[3] < 1, opacity: rgba[3],
});

// Mesh visuals (the tray boards: one OBJ, its parts named by material) as instances:
// one InstancedMesh per material, one instance per placement.
async function buildMeshes(prims, base) {
  const byUrl = new Map();
  for (const p of prims) {
    if (!byUrl.has(p.url)) byUrl.set(p.url, []);
    byUrl.get(p.url).push(p);
  }
  const loader = new OBJLoader();
  for (const [url, placed] of byUrl) {
    const obj = await loader.loadAsync(base + url);
    const parts = new Map();          // material name -> geometries
    obj.traverse((o) => {
      if (!o.isMesh) return;
      const mats = Array.isArray(o.material) ? o.material : [o.material];
      if (o.geometry.groups.length && mats.length > 1) {
        for (const g of o.geometry.groups) {    // OBJLoader geometry is non-indexed: slice the group's vertices
          const idx = [...Array(g.count).keys()].map((k) => g.start + k);
          const geo = new THREE.BufferGeometry();
          for (const [name, attr] of Object.entries(o.geometry.attributes)) {
            const arr = new attr.array.constructor(idx.length * attr.itemSize);
            idx.forEach((v, k) => { for (let c = 0; c < attr.itemSize; c++) arr[k * attr.itemSize + c] = attr.array[v * attr.itemSize + c]; });
            geo.setAttribute(name, new THREE.BufferAttribute(arr, attr.itemSize));
          }
          const key = mats[g.materialIndex].name;
          if (!parts.has(key)) parts.set(key, []);
          parts.get(key).push(geo);
        }
      } else {
        const key = mats[0].name;
        if (!parts.has(key)) parts.set(key, []);
        parts.get(key).push(o.geometry);
      }
    });
    for (const [name, geoms] of parts) {
      const rgba = placed[0].colors[name] ?? [0.7, 0.7, 0.7, 1];
      const geo = geoms.length > 1 ? mergeGeometries(geoms) : geoms[0];
      const inst = new THREE.InstancedMesh(geo, material(rgba), placed.length);
      placed.forEach((p, k) => inst.setMatrixAt(k, primMatrix(p)));
      inst.castShadow = inst.receiveShadow = true;
      scene.add(inst);
    }
  }
}

async function buildScene(data, base) {
  // static: one merged mesh per colour (hundreds of boxes)
  const byColour = new Map();
  const meshes = [];
  for (const p of data.prims) {
    if (p.type === "mesh") { meshes.push(p); continue; }
    if (p.live) {
      const m = (liveModels[p.model] ??= { group: new THREE.Group() });
      const mesh = new THREE.Mesh(primGeometry(p), material(p.rgba));
      mesh.applyMatrix4(primMatrix(p));
      mesh.castShadow = mesh.receiveShadow = true;
      m.group.add(mesh);
      continue;
    }
    const key = p.rgba.join(",");
    if (!byColour.has(key)) byColour.set(key, []);
    byColour.get(key).push(primGeometry(p).applyMatrix4(primMatrix(p)));
  }
  for (const [key, geoms] of byColour) {
    const mesh = new THREE.Mesh(mergeGeometries(geoms.map((g) => g.toNonIndexed())),
      material(key.split(",").map(Number)));
    mesh.castShadow = mesh.receiveShadow = true;
    scene.add(mesh);
  }
  for (const m of Object.values(liveModels)) {
    m.group.matrixAutoUpdate = false;
    scene.add(m.group);
  }
  await buildMeshes(meshes, base);
}

// ------------------------------------------------------------------ robot (URDF)
let rig = null;               // {model, rootToBaseInv, jointIdx: [[urdf joint, column]]}

function loadRobot(url, onProgress) {
  return new Promise((resolve, reject) => {
    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    loader.parseCollision = false;
    // STL is flat-shaded; the meshes are decimated, so smooth all but the real edges
    const stl = new STLLoader(manager);
    loader.loadMeshCb = (path, _manager, done) => stl.load(path, (geom) => {
      const g = toCreasedNormals(geom, Math.PI / 6);
      done(new THREE.Mesh(g, new THREE.MeshPhongMaterial()));
    }, undefined, (err) => done(null, err));
    let model = null;
    manager.onProgress = (_url, done, total) => onProgress(done, total);
    manager.onLoad = () => (model ? resolve(model) : null);
    manager.onError = (url) => reject(new Error(`could not load ${url}`));
    loader.load(url, (m) => {
      model = m;
      if (manager.itemsLoaded >= manager.itemsTotal) resolve(model);
    });
  });
}

function prepareRobot(model, baseLink) {
  model.traverse((o) => {
    if (!o.isMesh) return;
    o.castShadow = o.receiveShadow = true;
    const mats = Array.isArray(o.material) ? o.material : [o.material];
    for (const mat of mats) {
      // URDF colours only; meshes without one get a light alloy
      if (mat.color && mat.color.getHex() === 0xffffff && !mat.map) mat.color.set(0xc9ced6);
      mat.shininess = 30;
    }
  });
  // the recording has the base frame in the world; the URDF may hang it below another root
  model.updateMatrixWorld(true);
  const base = model.links[baseLink];
  const rootToBaseInv = base && base !== model ? base.matrixWorld.clone().invert() : new THREE.Matrix4();
  model.matrixAutoUpdate = false;
  scene.add(model);
  return { model, rootToBaseInv };
}

// ------------------------------------------------------------------ recording
let run = null;
let t0 = 0, t1 = 0;           // shown time range (recording time)
let group = null;             // {defs, segs, list}

function index(t) {           // last sample at or before t
  const ts = run.t;
  let lo = 0, hi = ts.length - 1;
  if (t <= ts[0]) return 0;
  if (t >= ts[hi]) return hi;
  while (hi - lo > 1) { const mid = (lo + hi) >> 1; (ts[mid] <= t ? (lo = mid) : (hi = mid)); }
  return lo;
}

const _p = new THREE.Vector3(), _q = new THREE.Quaternion(), _q2 = new THREE.Quaternion();
const _s = new THREE.Vector3(1, 1, 1);
function poseAt(track, i, f, out) {
  const a = track[i], b = track[Math.min(i + 1, track.length - 1)];
  _p.set(a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f);
  _q.set(a[3], a[4], a[5], a[6]);
  _q2.set(b[3], b[4], b[5], b[6]);
  _q.slerp(_q2, f);
  return out.compose(_p, _q, _s);
}

function buildGroup() {
  t1 = run.t[run.t.length - 1];
  const defs = run.phase_defs;
  const keys = new Set(defs.map(([k]) => k));
  const reached = run.phases.filter((p) => keys.has(p.phase));
  const first = reached[0];
  t0 = Math.max(run.t[0], (first?.t ?? run.t[0]) - LEAD_IN);
  const failed = run.phases.some((p) => p.phase === "failed");
  const segs = defs.map(([key, label, desc]) => ({ key, label, desc, t: null, end: null, failed: false }));
  reached.forEach((p, i) => {
    const s = segs.find((x) => x.key === p.phase);
    if (s.t !== null) return;
    s.t = p.t;
    s.end = reached[i + 1]?.t ?? t1;
  });
  if (failed && reached.length) segs.find((x) => x.key === reached.at(-1).phase).failed = true;
  const done = segs.at(-1);   // the last step ("done") is instantaneous: give it the tail
  if (done.key === "done" && done.t !== null) done.end = t1;
  group = { segs, list: $("phases") };
}

function renderGroup() {
  const g = group;
  g.list.innerHTML = "";
  g.segs.forEach((s, i) => {
    const li = document.createElement("li");
    li.className = "phase";
    li.innerHTML = `<span class="dot">${i + 1}</span>
      <div><div class="name">${esc(s.label)}</div><div class="desc">${esc(s.desc)}</div></div>
      <span class="dur">${s.t === null ? "" : fmt(s.end - s.t)}</span>
      <span class="bar-fill"></span>`;
    if (s.t !== null) li.addEventListener("click", () => seek(s.t + 0.01));
    g.list.appendChild(li);
    s.el = li;
  });
  const bar = $("segments");
  bar.innerHTML = "";
  for (const s of g.segs) {
    if (s.t === null) continue;
    const d = document.createElement("div");
    d.style.flex = `${Math.max(s.end - s.t, 0.5)} 0 0`;
    d.title = s.label;
    d.addEventListener("click", () => seek(s.t + 0.01));
    bar.appendChild(d);
    s.segEl = d;
  }
}

function applyGroup() {
  const g = group;
  const cur = g.segs.reduce((acc, s) => (s.t !== null && s.t <= t ? s : acc), null);
  for (const s of g.segs) {
    const isCur = s === cur;
    const done = s.t !== null && s.end <= t && !isCur;
    s.el.classList.toggle("active", isCur && !s.failed);
    s.el.classList.toggle("failed", isCur && s.failed && t >= s.end - 0.05);
    s.el.classList.toggle("done", done || (isCur && s.key === "done"));
    s.el.querySelector(".bar-fill").style.width =
      isCur && s.end > s.t ? `${Math.min(100, ((t - s.t) / (s.end - s.t)) * 100)}%` : "0";
    if (s.segEl) {
      s.segEl.classList.toggle("active", isCur);
      s.segEl.classList.toggle("done", done);
      s.segEl.classList.toggle("failed", s.failed && t >= s.end - 0.05);
    }
  }
  if (cur && cur.el.dataset.shown !== "1") {
    g.segs.forEach((s) => (s.el.dataset.shown = ""));
    cur.el.dataset.shown = "1";
    cur.el.scrollIntoView({ block: "nearest", behavior: "smooth" });
  }
}

let logCount = -1;
function renderLog(time) {
  let n = 0;
  while (n < run.logs.length && run.logs[n].t <= time) n++;
  if (n === logCount) return;
  logCount = n;
  const box = $("log");
  box.innerHTML = run.logs.slice(Math.max(0, n - 60), n).filter((l) => l.t >= t0).map((l) => {
    const cls = l.level >= 40 ? "err" : l.level >= 30 ? "warn" : "";
    return `<div class="${cls}"><span class="t">${fmt(Math.max(0, l.t - t0))}</span>${esc(l.msg)}</div>`;
  }).join("");
  box.scrollTop = box.scrollHeight;
}

function renderFacts(facts) {
  $("facts").innerHTML = facts.map(([k, v]) =>
    `<div class="f"><div class="k">${esc(k)}</div><div class="v">${esc(v)}</div></div>`).join("");
}

// ------------------------------------------------------------------ playback
let t = 0;                    // recording time
let playing = false, started = false, speed = 2;
let follow = false;
const _m = new THREE.Matrix4();

function apply(time) {
  t = Math.min(Math.max(time, t0), t1);
  const i = index(t);
  const ta = run.t[i], tb = run.t[Math.min(i + 1, run.t.length - 1)];
  const f = tb > ta ? Math.min(1, (t - ta) / (tb - ta)) : 0;

  if (rig) {
    const a = run.joints[i], b = run.joints[Math.min(i + 1, run.joints.length - 1)];
    for (const [joint, c] of rig.jointIdx) joint.setJointValue(a[c] + (b[c] - a[c]) * f);
    rig.model.matrix.multiplyMatrices(poseAt(run.base, i, f, _m), rig.rootToBaseInv);
    rig.model.matrixWorldNeedsUpdate = true;
  }
  for (const [name, m] of Object.entries(liveModels)) {
    if (run.objects[name]) poseAt(run.objects[name], i, f, m.group.matrix);
    else if (run.objects_still[name]) poseAt([run.objects_still[name]], 0, 0, m.group.matrix);
    m.group.matrixWorldNeedsUpdate = true;
  }

  applyGroup();
  renderLog(t);
  $("clock").textContent = `${fmt(t - t0)} / ${fmt(t1 - t0)}`;
  if (!scrubbing) $("scrub").value = String(Math.round(((t - t0) / (t1 - t0)) * 1000));
}

function setButton() {
  const label = !started ? "Start" : playing ? "Pause" : t >= t1 ? "Replay" : "Resume";
  $("start-label").textContent = label;
  $("start-icon").setAttribute("d", playing ? "M7 5h4v14H7zM13 5h4v14h-4z"
    : label === "Replay" ? "M12 5V2L7 6l5 4V7a6 6 0 1 1-6 6H4a8 8 0 1 0 8-8z" : "M8 5v14l11-7z");
}

function seek(time) {
  started = true;
  apply(time);
  setButton();
}

$("start").addEventListener("click", () => {
  if (!started || t >= t1) { started = true; apply(t0); playing = true; }
  else playing = !playing;
  setButton();
});
document.querySelectorAll(".speed button").forEach((b) => b.addEventListener("click", () => {
  speed = Number(b.dataset.speed);
  document.querySelectorAll(".speed button").forEach((x) => x.classList.toggle("on", x === b));
}));
let scrubbing = false;
const scrub = $("scrub");
scrub.addEventListener("input", () => { scrubbing = true; seek(t0 + (scrub.value / 1000) * (t1 - t0)); });
scrub.addEventListener("change", () => { scrubbing = false; });
$("follow").addEventListener("click", (e) => {
  follow = !follow;
  e.currentTarget.setAttribute("aria-pressed", String(follow));
});
$("reset").addEventListener("click", () => {
  follow = false;
  $("follow").setAttribute("aria-pressed", "false");
  camera.position.copy(HOME_VIEW.pos);
  controls.target.copy(HOME_VIEW.target);
});
document.addEventListener("keydown", (e) => {
  if (e.code === "Space" && !$("start").disabled && e.target === document.body) {
    e.preventDefault();
    $("start").click();
  }
});

// ------------------------------------------------------------------ loop
const clock = new THREE.Clock();
const _basePos = new THREE.Vector3(), _delta = new THREE.Vector3();
function frame() {
  const dt = Math.min(clock.getDelta(), 0.1);
  if (run && playing) {
    apply(t + dt * speed);
    if (t >= t1) { playing = false; setButton(); }
  }
  if (follow && rig) {
    _basePos.setFromMatrixPosition(rig.model.matrix);
    _basePos.z = 1.1;
    _delta.subVectors(_basePos, controls.target).multiplyScalar(Math.min(1, dt * 4));
    controls.target.add(_delta);
    camera.position.add(_delta);
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
resize();
frame();

// ------------------------------------------------------------------ demo switch
function renderSwitch(items, current) {
  const box = $("robots");
  box.innerHTML = "";
  for (const r of items) {
    const b = document.createElement("button");
    b.textContent = r.label;
    b.classList.toggle("on", r.id === current.id);
    b.setAttribute("aria-pressed", String(r.id === current.id));
    // a fresh page per demo: its own scene and run (keeps the speed)
    b.addEventListener("click", () => {
      if (r.id === current.id) return;
      const q = new URLSearchParams({ demo: r.id });
      if (speed !== 2) q.set("speed", speed);
      location.search = q.toString();
    });
    box.appendChild(b);
  }
}
const askedSpeed = Number(params.get("speed"));
if ([1, 2, 4, 8].includes(askedSpeed)) {
  speed = askedSpeed;
  document.querySelectorAll(".speed button").forEach((x) =>
    x.classList.toggle("on", Number(x.dataset.speed) === speed));
}

// ------------------------------------------------------------------ load
(async () => {
  try {
    const index = await getJSON("data/index.json");
    const demos = index.demos;
    const demo = demos.find((r) => r.id === params.get("demo")) ?? demos[0];
    DATA = `data/${demo.id}/`;
    renderSwitch(demos, demo);
    const modelP = loadRobot(`${DATA}robot/robot.urdf`, (done, total) => {
      $("loading-text").textContent = `Loading robot model… ${done}/${total}`;
    });
    const [runData, sceneData] = await Promise.all([getJSON(`${DATA}run.json`), getJSON(`${DATA}scene.json`)]);
    run = runData;
    if (run.view) HOME_VIEW = { pos: new THREE.Vector3(...run.view.pos), target: new THREE.Vector3(...run.view.target) };
    camera.position.copy(HOME_VIEW.pos);
    controls.target.copy(HOME_VIEW.target);
    renderFacts(run.facts ?? []);
    await buildScene(sceneData, "data/");
    buildGroup();
    renderGroup();
    const model = await modelP;
    rig = { ...prepareRobot(model, run.base_frame), jointIdx: [] };
    run.joint_names.forEach((n, c) => {
      if (model.joints[n]) rig.jointIdx.push([model.joints[n], c]);
    });
    // ?t=<seconds> opens the replay at that moment
    const at = Number(params.get("t"));
    if (at > 0) { started = true; apply(t0 + at); } else apply(t0);
    $("loading").classList.add("gone");
    $("start").disabled = false;
    setButton();
  } catch (err) {
    $("loading-text").textContent = `Could not load the recording: ${err.message}`;
    console.error(err);
  }
})();
