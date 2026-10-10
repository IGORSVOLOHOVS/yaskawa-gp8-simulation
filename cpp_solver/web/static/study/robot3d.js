/* robot3d.js - the GP8 in three.js, shared by every panel.
 *
 * The STL loading, the ROS Z-up to three.js Y-up rotation, the URDF joint
 * origins, the camera and the orbit controls are the same ones the existing
 * dashboard uses. On top of that it draws whatever the selected op produced:
 * mat4 frames as RGB axis triads and points as a point cloud. The view never
 * disappears - an op with no 3D meaning simply leaves the robot standing.
 */
(function (global) {
  'use strict';

  var Robot3D = {};

  var MESHES = [
    { file: 'gp8_base_link.stl', group: 'base', material: 'blue' },
    { file: 'gp8_link_1_s.stl', group: 'j1', material: 'blue' },
    { file: 'gp8_link_2_l.stl', group: 'j2', material: 'silver' },
    { file: 'gp8_link_3_u.stl', group: 'j3', material: 'blue' },
    { file: 'gp8_link_4_r.stl', group: 'j4', material: 'silver' },
    { file: 'gp8_link_5_b.stl', group: 'j5', material: 'blue' },
    { file: 'gp8_link_6_t.stl', group: 'j6', material: 'dark' }
  ];

  var state = {
    ready: false,
    scene: null,
    camera: null,
    renderer: null,
    controls: null,
    root: null,
    groups: {},
    overlay: null,
    container: null,
    loaded: 0,
    joints: [0, 0, 0, 0, 0, 0]
  };

  function disposeChildren(node) {
    while (node.children.length) {
      var child = node.children.pop();
      if (child.geometry && child.geometry.dispose) { child.geometry.dispose(); }
      if (child.material && child.material.dispose) { child.material.dispose(); }
    }
  }

  Robot3D.available = function () {
    return typeof global.THREE !== 'undefined' && !!global.THREE.WebGLRenderer;
  };

  Robot3D.init = function (container) {
    if (state.ready || !Robot3D.available()) { return Robot3D.available(); }
    var THREE = global.THREE;
    state.container = container;

    var scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b1220);

    var width = container.clientWidth || 640;
    var height = container.clientHeight || 420;
    var camera = new THREE.PerspectiveCamera(45, width / height, 0.01, 100);
    camera.position.set(1.3, 1.0, 1.3);

    var renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(global.devicePixelRatio || 1);
    renderer.setSize(width, height);
    container.appendChild(renderer.domElement);

    var controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0.4, 0);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.update();

    scene.add(new THREE.AmbientLight(0xffffff, 0.7));
    var dir = new THREE.DirectionalLight(0xffffff, 0.9);
    dir.position.set(3, 5, 3);
    scene.add(dir);

    scene.add(new THREE.GridHelper(4, 40, 0x38bdf8, 0x334155));

    // ROS Z-up to three.js Y-up, exactly as the existing dashboard does it
    var root = new THREE.Group();
    root.rotation.x = -Math.PI / 2;
    scene.add(root);

    var groups = {
      base: new THREE.Group(),
      j1: new THREE.Group(),
      j2: new THREE.Group(),
      j3: new THREE.Group(),
      j4: new THREE.Group(),
      j5: new THREE.Group(),
      j6: new THREE.Group()
    };
    root.add(groups.base);
    groups.j1.position.set(0, 0, 0.33);
    groups.base.add(groups.j1);
    groups.j2.position.set(0.04, 0, 0);
    groups.j1.add(groups.j2);
    groups.j3.position.set(0, 0, 0.345);
    groups.j2.add(groups.j3);
    groups.j4.position.set(0.34, 0, 0.04);
    groups.j3.add(groups.j4);
    groups.j5.position.set(0, 0, 0);
    groups.j4.add(groups.j5);
    groups.j6.position.set(0, 0, 0);
    groups.j5.add(groups.j6);

    // base frame of the workcell
    var baseAxes = new THREE.AxesHelper(0.25);
    root.add(baseAxes);

    var overlay = new THREE.Group();
    root.add(overlay);

    var materials = {
      blue: new THREE.MeshStandardMaterial({ color: 0x1462c4, metalness: 0.4, roughness: 0.3 }),
      silver: new THREE.MeshStandardMaterial({ color: 0xd1d5db, metalness: 0.6, roughness: 0.2 }),
      dark: new THREE.MeshStandardMaterial({ color: 0x262626, metalness: 0.8, roughness: 0.2 })
    };

    var loader = new THREE.STLLoader();
    MESHES.forEach(function (spec) {
      loader.load('/meshes/' + spec.file, function (geo) {
        var mesh = new THREE.Mesh(geo, materials[spec.material]);
        groups[spec.group].add(mesh);
        state.loaded += 1;
      }, undefined, function () {
        state.loaded += 1;        // a missing mesh must not stop the view
      });
    });

    state.scene = scene;
    state.camera = camera;
    state.renderer = renderer;
    state.controls = controls;
    state.root = root;
    state.groups = groups;
    state.overlay = overlay;
    state.ready = true;

    function animate() {
      global.requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    }
    animate();

    Robot3D.resize();
    global.addEventListener('resize', Robot3D.resize);
    if (global.ResizeObserver) {
      new global.ResizeObserver(Robot3D.resize).observe(container);
    }
    return true;
  };

  Robot3D.resize = function () {
    if (!state.ready) { return; }
    var w = state.container.clientWidth || 640;
    var h = state.container.clientHeight || 420;
    state.camera.aspect = w / h;
    state.camera.updateProjectionMatrix();
    state.renderer.setSize(w, h);
  };

  /** Home configuration is all joints at zero. */
  Robot3D.setJoints = function (q) {
    if (!state.ready) { return; }
    var v = (q && q.length === 6) ? q.map(Number) : [0, 0, 0, 0, 0, 0];
    for (var i = 0; i < 6; i += 1) { if (!isFinite(v[i])) { v[i] = 0; } }
    state.joints = v;
    var g = state.groups;
    g.j1.rotation.z = v[0];
    g.j2.rotation.y = v[1];
    g.j3.rotation.y = v[2];
    g.j4.rotation.x = v[3];
    g.j5.rotation.y = v[4];
    g.j6.rotation.x = v[5];
  };

  function matrixFromRows(rows) {
    var THREE = global.THREE;
    var m = new THREE.Matrix4();
    var r = rows;
    m.set(
      Number(r[0][0]), Number(r[0][1]), Number(r[0][2]), Number(r[0][3]),
      Number(r[1][0]), Number(r[1][1]), Number(r[1][2]), Number(r[1][3]),
      Number(r[2][0]), Number(r[2][1]), Number(r[2][2]), Number(r[2][3]),
      Number(r[3][0]), Number(r[3][1]), Number(r[3][2]), Number(r[3][3])
    );
    return m;
  }

  /** Draw mat4 frames as RGB triads and points as a cloud, in robot space. */
  Robot3D.showScene = function (payload) {
    if (!state.ready) { return; }
    var THREE = global.THREE;
    disposeChildren(state.overlay);

    var frames = (payload && payload.frames) || [];
    frames.forEach(function (rows) {
      if (!Array.isArray(rows) || rows.length !== 4) { return; }
      var axes = new THREE.AxesHelper(0.12);
      axes.matrixAutoUpdate = false;
      axes.matrix = matrixFromRows(rows);
      state.overlay.add(axes);
    });

    var points = (payload && payload.points) || [];
    if (points.length) {
      var coords = [];
      points.forEach(function (p) {
        if (!Array.isArray(p) || p.length < 3) { return; }
        coords.push(Number(p[0]), Number(p[1]), Number(p[2]));
      });
      if (coords.length) {
        var geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(coords, 3));
        var mat = new THREE.PointsMaterial({ color: 0x38bdf8, size: 0.012 });
        state.overlay.add(new THREE.Points(geo, mat));
        if (coords.length <= 3 * 64) {
          var lineGeo = new THREE.BufferGeometry();
          lineGeo.setAttribute('position', new THREE.Float32BufferAttribute(coords, 3));
          state.overlay.add(new THREE.Line(lineGeo, new THREE.LineBasicMaterial({ color: 0x4ade80 })));
        }
      }
    }
  };

  Robot3D.status = function () {
    return { ready: state.ready, meshes: state.loaded, joints: state.joints.slice() };
  };

  global.Robot3D = Robot3D;
}(window));
