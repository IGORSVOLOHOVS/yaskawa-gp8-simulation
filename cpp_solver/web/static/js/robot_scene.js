// Yaskawa GP8 3D WebGL Scene Engine
let scene, camera, renderer, controls;
let robotRootGroup, baseGroup, j1Group, j2Group, j3Group, j4Group, j5Group, j6Group, cabinetGroup, pendantGroup;
let blueMat, silverMat, darkMat, copperMat, magnetMat, goldMat, pcbMat, chipMat, glassMat, fastenerMat, rubberMat, redMat, highlightMat;
let isXRay = false;
let isExploded = false;
let selectedPartId = 'root';
let lastHighlightedMesh = null;
const originalMatMap = new Map();
const comp3DObjects = {};

function initRobotScene() {
    const container = document.getElementById('canvas-container');
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b1120);

    camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.01, 100);
    camera.position.set(1.2, 1.0, 1.2);

    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.target.set(0, 0, 0.4);

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.65);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.95);
    dirLight.position.set(4, 8, 5);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    scene.add(dirLight);

    const fillLight = new THREE.DirectionalLight(0x38bdf8, 0.35);
    fillLight.position.set(-4, 2, -4);
    scene.add(fillLight);

    const grid = new THREE.GridHelper(4, 40, 0x38bdf8, 0x1e293b);
    scene.add(grid);

    // Materials System (Default X-Ray Semi-Transparency enabled for immediate 1000+ part visibility)
    blueMat = new THREE.MeshStandardMaterial({ color: 0x1462c4, metalness: 0.4, roughness: 0.3, transparent: true, opacity: 0.35 });
    silverMat = new THREE.MeshStandardMaterial({ color: 0xd1d5db, metalness: 0.6, roughness: 0.2, transparent: true, opacity: 0.35 });
    darkMat = new THREE.MeshStandardMaterial({ color: 0x262626, metalness: 0.8, roughness: 0.2, transparent: true, opacity: 0.35 });

    copperMat = new THREE.MeshStandardMaterial({ color: 0xd97706, metalness: 0.85, roughness: 0.15 });
    magnetMat = new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.9, roughness: 0.1 });
    goldMat = new THREE.MeshStandardMaterial({ color: 0xeab308, metalness: 0.95, roughness: 0.05 });
    pcbMat = new THREE.MeshStandardMaterial({ color: 0x059669, metalness: 0.2, roughness: 0.4 });
    chipMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, metalness: 0.7, roughness: 0.2 });
    glassMat = new THREE.MeshPhysicalMaterial({ color: 0x38bdf8, transmission: 0.9, opacity: 1, transparent: true, roughness: 0.05 });
    fastenerMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.8, roughness: 0.2 });
    rubberMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.1, roughness: 0.8 });
    redMat = new THREE.MeshStandardMaterial({ color: 0xef4444, metalness: 0.4, roughness: 0.3 });

    highlightMat = new THREE.MeshStandardMaterial({ color: 0x00ffff, emissive: 0x00aaaa, emissiveIntensity: 0.8, metalness: 0.5, roughness: 0.2 });

    // Kinematic Joint Hierarchy Setup
    robotRootGroup = new THREE.Group();
    robotRootGroup.rotation.x = -Math.PI / 2;
    scene.add(robotRootGroup);

    baseGroup = new THREE.Group();
    j1Group = new THREE.Group();
    j2Group = new THREE.Group();
    j3Group = new THREE.Group();
    j4Group = new THREE.Group();
    j5Group = new THREE.Group();
    j6Group = new THREE.Group();
    cabinetGroup = new THREE.Group();
    pendantGroup = new THREE.Group();

    robotRootGroup.add(baseGroup);
    j1Group.position.set(0, 0, 0.33); baseGroup.add(j1Group);
    j2Group.position.set(0.04, 0, 0); j1Group.add(j2Group);
    j3Group.position.set(0, 0, 0.345); j2Group.add(j3Group);
    j4Group.position.set(0.34, 0, 0.04); j3Group.add(j4Group);
    j5Group.position.set(0, 0, 0); j4Group.add(j5Group);
    j6Group.position.set(0, 0, 0); j5Group.add(j6Group);

    // Co-locate Cabinet & Pendant neatly beside base
    cabinetGroup.position.set(-0.30, -0.20, 0.12); robotRootGroup.add(cabinetGroup);
    pendantGroup.position.set(-0.30, -0.20, 0.32); robotRootGroup.add(pendantGroup);

    // Load STL Outer Link Shells
    const loader = new THREE.STLLoader();
    const meshFiles = [
        { id: 'sec_1_casting', path: '/meshes/gp8_base_link.stl', mat: blueMat, grp: baseGroup },
        { id: 'sec_1', path: '/meshes/gp8_link_1_s.stl', mat: blueMat, grp: j1Group },
        { id: 'sec_2', path: '/meshes/gp8_link_2_l.stl', mat: silverMat, grp: j2Group },
        { id: 'sec_3', path: '/meshes/gp8_link_3_u.stl', mat: blueMat, grp: j3Group },
        { id: 'sec_4', path: '/meshes/gp8_link_4_r.stl', mat: silverMat, grp: j4Group },
        { id: 'sec_5', path: '/meshes/gp8_link_5_b.stl', mat: blueMat, grp: j5Group },
        { id: 'sec_6', path: '/meshes/gp8_link_6_t.stl', mat: darkMat, grp: j6Group }
    ];

    meshFiles.forEach(item => {
        loader.load(item.path, (geo) => {
            const mesh = new THREE.Mesh(geo, item.mat);
            mesh.castShadow = true;
            mesh.receiveShadow = true;
            mesh.userData.partId = item.id;
            item.grp.add(mesh);
            comp3DObjects[item.id] = mesh;
            originalMatMap.set(mesh, item.mat);
        });
    });

    // Load Robotiq 2F-85 Industrial Gripper CAD Models onto J6 Tool Flange
    const gripperMeshFiles = [
        { id: 'robotiq_base', path: '/meshes/gripper/base.stl', mat: darkMat, scale: 0.001 },
        { id: 'robotiq_coupler', path: '/meshes/gripper/coupler.stl', mat: silverMat, scale: 0.001 },
        { id: 'robotiq_driver', path: '/meshes/gripper/driver.stl', mat: darkMat, scale: 0.001 },
        { id: 'robotiq_follower', path: '/meshes/gripper/follower.stl', mat: darkMat, scale: 0.001 }
    ];

    gripperMeshFiles.forEach(item => {
        loader.load(item.path, (geo) => {
            if (item.scale) geo.scale(item.scale, item.scale, item.scale);
            const mesh = new THREE.Mesh(geo, item.mat);
            mesh.castShadow = true;
            mesh.receiveShadow = true;
            mesh.userData.partId = item.id;
            mesh.position.set(0, 0, 0.015);
            j6Group.add(mesh);
            comp3DObjects[item.id] = mesh;
            originalMatMap.set(mesh, item.mat);
        });
    });

    // Window Resize Handler
    window.addEventListener('resize', onWindowResize);

    // Raycaster Click Handler
    const raycaster = new THREE.Raycaster();
    const mouse = new THREE.Vector2();
    window.addEventListener('pointerdown', (e) => {
        if (e.target.tagName !== 'CANVAS') return;
        mouse.x = (e.clientX / window.innerWidth) * 2 - 1;
        mouse.y = -(e.clientY / window.innerHeight) * 2 + 1;
        raycaster.setFromCamera(mouse, camera);
        const intersects = raycaster.intersectObjects(Object.values(comp3DObjects), true);
        if (intersects.length > 0) {
            const hitMesh = intersects[0].object;
            if (hitMesh.userData && hitMesh.userData.partId) {
                selectPart(hitMesh.userData.partId);
            }
        }
    });

    animate();
}

function create3DMesh(partId, geometry, material, parentGroup, position, rotation, scale) {
    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.userData.partId = partId;
    if (position) {
        if (position.isVector3) mesh.position.copy(position);
        else if (Array.isArray(position)) mesh.position.set(position[0], position[1], position[2]);
    }
    if (rotation) {
        if (rotation.isEuler) mesh.rotation.copy(rotation);
        else if (Array.isArray(rotation)) mesh.rotation.set(rotation[0], rotation[1], rotation[2]);
    }
    if (scale) {
        if (scale.isVector3) mesh.scale.copy(scale);
        else if (Array.isArray(scale)) mesh.scale.set(scale[0], scale[1], scale[2]);
    }
    if (parentGroup) parentGroup.add(mesh);
    comp3DObjects[partId] = mesh;
    originalMatMap.set(mesh, material);
    return mesh;
}

let activeBoxHelper = null;
let activeTargetMesh = null;

function spotlightSelectedMesh(partId, comp) {
    if (activeBoxHelper) {
        scene.remove(activeBoxHelper);
        activeBoxHelper.geometry.dispose();
        activeBoxHelper = null;
    }

    const targetMesh = comp3DObjects[partId];
    if (!targetMesh) {
        focusCameraOnSystem(comp ? comp.system_category_id : 1, comp ? comp.level : 1);
        hideCalloutBadge();
        return;
    }

    targetMesh.visible = true;

    // Create Glowing 3D Bounding Box Helper
    activeBoxHelper = new THREE.BoxHelper(targetMesh, 0x00ffff);
    scene.add(activeBoxHelper);

    // Calculate Bounding Center & Radius for Adaptive Micro-Macro Zoom
    const box = new THREE.Box3().setFromObject(targetMesh);
    const center = new THREE.Vector3();
    box.getCenter(center);

    const sphere = new THREE.Sphere();
    box.getBoundingSphere(sphere);
    const radius = Math.max(sphere.radius, 0.002);

    // Adaptive micro-zoom camera offset (Zoom in to 4-5cm for tiny 2mm parts!)
    const camOffset = Math.max(radius * 4.2, 0.045);
    const targetCamPos = [center.x + camOffset, center.y + camOffset * 0.75, center.z + camOffset];

    smoothAnimateCamera([center.x, center.y, center.z], targetCamPos, 700);

    // Highlight Target Mesh with Cyan Emissive Material
    if (lastHighlightedMesh && originalMatMap.has(lastHighlightedMesh)) {
        lastHighlightedMesh.material = originalMatMap.get(lastHighlightedMesh);
    }
    targetMesh.material = highlightMat;
    lastHighlightedMesh = targetMesh;
    activeTargetMesh = targetMesh;

    // Display Floating 3D Callout Badge
    if (comp) {
        showCalloutBadge(comp);
    }
}

function updateCalloutBadgePosition() {
    if (!activeTargetMesh) return;
    const badge = document.getElementById('spatial-callout-badge');
    if (!badge || badge.style.display === 'none') return;

    const box = new THREE.Box3().setFromObject(activeTargetMesh);
    const center = new THREE.Vector3();
    box.getCenter(center);

    const vector = center.clone();
    vector.project(camera);

    const x = (vector.x * 0.5 + 0.5) * window.innerWidth;
    const y = (-(vector.y * 0.5) + 0.5) * window.innerHeight;

    badge.style.left = `${x}px`;
    badge.style.top = `${y}px`;
}

function showCalloutBadge(comp) {
    const badge = document.getElementById('spatial-callout-badge');
    if (!badge) return;
    badge.style.display = 'flex';
    document.getElementById('callout-title').textContent = comp.name;
    document.getElementById('callout-sub').textContent = `ID: ${comp.id} | #${comp.part_number}`;
}

function hideCalloutBadge() {
    const badge = document.getElementById('spatial-callout-badge');
    if (badge) badge.style.display = 'none';
}

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    updateCalloutBadgePosition();
    renderer.render(scene, camera);
}

function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

// Smooth Camera LERP Controls
function smoothAnimateCamera(targetPos, cameraPos, duration = 800) {
    const startTarget = controls.target.clone();
    const startCamPos = camera.position.clone();
    const endTarget = new THREE.Vector3(...targetPos);
    const endCamPos = new THREE.Vector3(...cameraPos);
    const startTime = performance.now();

    function step() {
        const elapsed = performance.now() - startTime;
        const progress = Math.min(elapsed / duration, 1.0);
        const ease = 1 - Math.pow(1 - progress, 3);
        controls.target.lerpVectors(startTarget, endTarget, ease);
        camera.position.lerpVectors(startCamPos, endCamPos, ease);
        if (progress < 1.0) requestAnimationFrame(step);
    }
    step();
}

function focusCameraOnSystem(catId, level) {
    let t = [0, 0, 0.4];
    let c = [1.2, 1.0, 1.2];
    if (catId === 1) { t = [0, 0, 0.15]; c = [0.45, 0.35, 0.45]; }
    else if (catId === 2) { t = [0.04, 0, 0.45]; c = [0.55, 0.45, 0.70]; }
    else if (catId === 3) { t = [0.20, 0, 0.70]; c = [0.65, 0.50, 0.90]; }
    else if (catId === 4) { t = [0.38, 0, 0.72]; c = [0.68, 0.30, 0.85]; }
    else if (catId === 5) { t = [0.38, 0, 0.72]; c = [0.62, 0.25, 0.80]; }
    else if (catId === 6) { t = [0.38, 0, 0.72]; c = [0.58, 0.20, 0.76]; }
    else if (catId === 7) { t = [0.20, 0, 0.45]; c = [0.60, 0.40, 0.75]; }
    else if (catId === 8 || catId === 9) { t = [-0.30, -0.20, 0.12]; c = [-0.10, 0.10, 0.40]; }
    else if (catId === 10) { t = [-0.30, -0.20, 0.32]; c = [-0.15, 0.05, 0.55]; }
    else if (catId === 11) { t = [0.38, 0, 0.78]; c = [0.55, 0.15, 0.85]; }
    else if (catId === 12) { t = [0, 0, -0.40]; c = [1.20, 0.80, 0.50]; }

    if (level >= 3) {
        c = [t[0] + 0.18, t[1] + 0.15, t[2] + 0.18];
    }
    smoothAnimateCamera(t, c);
}

function toggleXRayMode(forceState) {
    isXRay = (forceState !== undefined) ? forceState : !isXRay;
    [blueMat, silverMat, darkMat].forEach(mat => {
        mat.transparent = isXRay;
        mat.opacity = isXRay ? 0.35 : 1.0;
    });
    const status = document.getElementById('console-status');
    if (status) status.textContent = `Status: X-Ray Mode ${isXRay ? 'ENABLED (1353 Physical Components Visible)' : 'DISABLED'} (${Object.keys(comp3DObjects).length} 3D Models Active)`;
}

function setPresetViewMode(mode) {
    if (mode === 'xray') {
        isXRay = true;
        toggleXRayMode(true);
    } else if (mode === 'solid') {
        isXRay = false;
        toggleXRayMode(false);
    } else if (mode === 'exploded') {
        toggleExplodedView();
    }
}

function toggleExplodedView() {
    isExploded = !isExploded;
    const offset = isExploded ? 0.18 : 0.0;
    j1Group.position.set(0, 0, 0.33 + offset);
    j2Group.position.set(0.04 + offset, 0, 0);
    j3Group.position.set(0, 0, 0.345 + offset);
    j4Group.position.set(0.34 + offset, 0, 0.04);
    const status = document.getElementById('console-status');
    if (status) status.textContent = `Status: Exploded Assembly View ${isExploded ? 'ACTIVE' : 'RESET'}`;
}

function resetCameraView() {
    smoothAnimateCamera([0, 0, 0.4], [1.2, 1.0, 1.2]);
}
