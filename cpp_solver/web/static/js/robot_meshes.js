// Yaskawa GP8 Procedural 3D Mesh Generator for 1353 Physical Components
function buildAllDetailed3DComponents() {
    // Category 1: Base & S-Axis (Ultra-compact radius <= 0.035m)
    const boltRadius = 0.032;
    for (let i = 1; i <= 4; i++) {
        const a = (i / 4) * Math.PI * 2;
        create3DMesh(`base_anchor_bolt_${i}`, new THREE.CylinderGeometry(0.002, 0.002, 0.015, 12), fastenerMat, baseGroup, new THREE.Vector3(Math.cos(a)*boltRadius, Math.sin(a)*boltRadius, 0.015));
    }
    for (let i = 1; i <= 8; i++) {
        const a = (i / 8) * Math.PI * 2;
        create3DMesh(`base_m12_bolt_${i}`, new THREE.CylinderGeometry(0.002, 0.002, 0.012, 12), fastenerMat, baseGroup, new THREE.Vector3(Math.cos(a)*0.028, Math.sin(a)*0.028, 0.02));
    }
    for (let i = 1; i <= 4; i++) {
        const a = (i / 4) * Math.PI * 2 + Math.PI/4;
        create3DMesh(`base_dowel_pin_${i}`, new THREE.CylinderGeometry(0.0015, 0.0015, 0.01, 12), silverMat, baseGroup, new THREE.Vector3(Math.cos(a)*0.025, Math.sin(a)*0.025, 0.015));
    }
    create3DMesh('base_ground_lug', new THREE.BoxGeometry(0.008, 0.008, 0.006), copperMat, baseGroup, new THREE.Vector3(-0.02, -0.02, 0.015));
    create3DMesh('base_cable_gland_plate', new THREE.BoxGeometry(0.02, 0.025, 0.004), silverMat, baseGroup, new THREE.Vector3(0.02, -0.02, 0.02));
    create3DMesh('base_gland_o_ring', new THREE.TorusGeometry(0.012, 0.0015, 12, 24), rubberMat, baseGroup, new THREE.Vector3(0.02, -0.02, 0.023));

    // S-Axis Motor & Tamagawa Encoder
    const motorPos = new THREE.Vector3(0, 0, 0.04);
    create3DMesh('s_axis_motor', new THREE.CylinderGeometry(0.025, 0.025, 0.05, 24), darkMat, j1Group, motorPos);
    create3DMesh('sgmsv_stator_frame', new THREE.CylinderGeometry(0.024, 0.024, 0.048, 24), darkMat, j1Group, motorPos);
    create3DMesh('sgmsv_stator_core', new THREE.CylinderGeometry(0.022, 0.022, 0.045, 24), silverMat, j1Group, motorPos);
    for (let i = 1; i <= 12; i++) {
        const a = (i / 12) * Math.PI * 2;
        create3DMesh(`sgmsv_coil_${i}`, new THREE.BoxGeometry(0.005, 0.005, 0.04), copperMat, j1Group, new THREE.Vector3(Math.cos(a)*0.018, Math.sin(a)*0.018, 0.04));
    }
    create3DMesh('sgmsv_rotor_shaft', new THREE.CylinderGeometry(0.006, 0.006, 0.06, 24), silverMat, j1Group, motorPos);
    for (let i = 1; i <= 8; i++) {
        const a = (i / 8) * Math.PI * 2;
        create3DMesh(`sgmsv_magnet_${i}`, new THREE.BoxGeometry(0.003, 0.005, 0.035), magnetMat, j1Group, new THREE.Vector3(Math.cos(a)*0.011, Math.sin(a)*0.011, 0.04));
    }
    create3DMesh('sgmsv_encoder', new THREE.CylinderGeometry(0.018, 0.018, 0.01, 24), darkMat, j1Group, new THREE.Vector3(0, 0, 0.075));
    create3DMesh('enc_glass_disc', new THREE.CylinderGeometry(0.016, 0.016, 0.0015, 32), glassMat, j1Group, new THREE.Vector3(0, 0, 0.075));
    create3DMesh('enc_led_emitter', new THREE.BoxGeometry(0.003, 0.003, 0.003), goldMat, j1Group, new THREE.Vector3(0.008, 0.003, 0.077));
    create3DMesh('enc_photodiode_array', new THREE.BoxGeometry(0.005, 0.005, 0.002), chipMat, j1Group, new THREE.Vector3(-0.008, 0.003, 0.077));
    create3DMesh('enc_pcb', new THREE.BoxGeometry(0.026, 0.026, 0.0015), pcbMat, j1Group, new THREE.Vector3(0, 0, 0.080));
    for (let i = 1; i <= 10; i++) {
        create3DMesh(`enc_pcb_resistor_${i}`, new THREE.BoxGeometry(0.002, 0.001, 0.001), darkMat, j1Group, new THREE.Vector3((i % 5 - 2) * 0.004, (Math.floor(i / 5) - 0.5) * 0.004, 0.082));
    }

    // Nabtesco RV-50E Reducer
    create3DMesh('s_axis_reducer', new THREE.CylinderGeometry(0.028, 0.028, 0.025, 32), darkMat, baseGroup, new THREE.Vector3(0, 0, 0.12));
    create3DMesh('rv50_planet_shaft_1', new THREE.CylinderGeometry(0.005, 0.005, 0.02, 16), silverMat, baseGroup, new THREE.Vector3(0.012, 0, 0.12));
    create3DMesh('rv50_planet_shaft_2', new THREE.CylinderGeometry(0.005, 0.005, 0.02, 16), silverMat, baseGroup, new THREE.Vector3(-0.006, 0.01, 0.12));
    create3DMesh('rv50_planet_shaft_3', new THREE.CylinderGeometry(0.005, 0.005, 0.02, 16), silverMat, baseGroup, new THREE.Vector3(-0.006, -0.01, 0.12));
    create3DMesh('rv50_cyc_disc_a', new THREE.CylinderGeometry(0.026, 0.026, 0.006, 32), silverMat, baseGroup, new THREE.Vector3(0, 0, 0.115));
    create3DMesh('rv50_cyc_disc_b', new THREE.CylinderGeometry(0.026, 0.026, 0.006, 32), silverMat, baseGroup, new THREE.Vector3(0, 0, 0.125));
    for (let i = 1; i <= 20; i++) {
        const a = (i / 20) * Math.PI * 2;
        create3DMesh(`rv50_pin_roller_${i}`, new THREE.CylinderGeometry(0.0015, 0.0015, 0.018, 12), silverMat, baseGroup, new THREE.Vector3(Math.cos(a)*0.027, Math.sin(a)*0.027, 0.12));
    }
    create3DMesh('s_axis_main_bearing', new THREE.TorusGeometry(0.028, 0.0025, 16, 32), goldMat, baseGroup, new THREE.Vector3(0, 0, 0.14));
    create3DMesh('s_axis_oil_seal', new THREE.TorusGeometry(0.026, 0.0015, 12, 32), rubberMat, baseGroup, new THREE.Vector3(0, 0, 0.145));
    for (let i = 1; i <= 16; i++) {
        const a = (i / 16) * Math.PI * 2;
        create3DMesh(`s_axis_flange_bolt_${i}`, new THREE.CylinderGeometry(0.0012, 0.0012, 0.01, 12), fastenerMat, baseGroup, new THREE.Vector3(Math.cos(a)*0.027, Math.sin(a)*0.027, 0.15));
    }

    // Category 2: L-Axis Lower Arm Components
    create3DMesh('l_axis_motor', new THREE.CylinderGeometry(0.020, 0.020, 0.035, 24), darkMat, j2Group, new THREE.Vector3(0, 0.015, 0.03));
    create3DMesh('l_axis_reducer', new THREE.CylinderGeometry(0.024, 0.024, 0.025, 32), darkMat, j2Group, new THREE.Vector3(0, -0.015, 0.03));
    create3DMesh('l_axis_brake', new THREE.CylinderGeometry(0.016, 0.016, 0.012, 24), copperMat, j2Group, new THREE.Vector3(0, 0.025, 0.03));

    // Category 3: U-Axis Upper Arm Components
    create3DMesh('u_axis_harmonic', new THREE.CylinderGeometry(0.016, 0.016, 0.018, 24), silverMat, j3Group, new THREE.Vector3(0.02, 0, 0.01));
    create3DMesh('csg32_wave_gen', new THREE.CylinderGeometry(0.011, 0.009, 0.008, 24), goldMat, j3Group, new THREE.Vector3(0.02, 0, 0.005));
    create3DMesh('csg32_flexspline', new THREE.CylinderGeometry(0.015, 0.015, 0.015, 24), silverMat, j3Group, new THREE.Vector3(0.02, 0, 0.012));
    create3DMesh('csg32_circular_spline', new THREE.CylinderGeometry(0.017, 0.017, 0.008, 24), darkMat, j3Group, new THREE.Vector3(0.02, 0, 0.018));

    // Category 4: R-Axis Wrist
    create3DMesh('r_axis_harmonic', new THREE.CylinderGeometry(0.012, 0.012, 0.015, 24), silverMat, j4Group, new THREE.Vector3(0.01, 0, 0));

    // Category 5: B-Axis Wrist
    create3DMesh('b_axis_harmonic', new THREE.CylinderGeometry(0.010, 0.010, 0.012, 24), silverMat, j5Group, new THREE.Vector3(0, 0, 0));

    // Category 6: T-Axis Tool Flange
    create3DMesh('t_axis_harmonic', new THREE.CylinderGeometry(0.008, 0.008, 0.010, 24), silverMat, j6Group, new THREE.Vector3(0, 0, 0.005));
    create3DMesh('tool_flange_plate', new THREE.CylinderGeometry(0.018, 0.018, 0.004, 24), darkMat, j6Group, new THREE.Vector3(0, 0, 0.012));
    create3DMesh('tool_m12_connector', new THREE.CylinderGeometry(0.003, 0.003, 0.008, 12), goldMat, j6Group, new THREE.Vector3(0.008, 0, 0.014));

    // Category 8: YRC1000 Power Cabinet (Glass-windowed workstation cabinet)
    create3DMesh('sec_8', new THREE.BoxGeometry(0.20, 0.28, 0.18), glassMat, cabinetGroup, new THREE.Vector3(0, 0, 0));
    create3DMesh('yrc_main_breaker', new THREE.BoxGeometry(0.03, 0.04, 0.02), darkMat, cabinetGroup, new THREE.Vector3(-0.04, 0.08, 0.03));
    create3DMesh('yrc_emc_filter', new THREE.BoxGeometry(0.04, 0.06, 0.02), silverMat, cabinetGroup, new THREE.Vector3(0.04, 0.07, 0.03));
    create3DMesh('yrc_igbt_module', new THREE.BoxGeometry(0.06, 0.08, 0.02), goldMat, cabinetGroup, new THREE.Vector3(0, -0.03, 0.03));

    // Category 9: YRC1000 Logic Boards (Visible inside glass workstation)
    create3DMesh('sec_9', new THREE.BoxGeometry(0.14, 0.003, 0.10), pcbMat, cabinetGroup, new THREE.Vector3(0, 0.05, 0.03));
    create3DMesh('yrc_main_cpu_board', new THREE.BoxGeometry(0.12, 0.003, 0.08), pcbMat, cabinetGroup, new THREE.Vector3(0, 0.05, 0.03));
    create3DMesh('yrc_cpu_chip', new THREE.BoxGeometry(0.018, 0.004, 0.018), chipMat, cabinetGroup, new THREE.Vector3(-0.02, 0.054, 0.04));
    create3DMesh('yrc_fpga_chip', new THREE.BoxGeometry(0.015, 0.004, 0.015), goldMat, cabinetGroup, new THREE.Vector3(0.02, 0.054, 0.04));
    create3DMesh('yrc_safety_board', new THREE.BoxGeometry(0.10, 0.003, 0.07), pcbMat, cabinetGroup, new THREE.Vector3(0, 0.05, -0.03));

    // Category 10: Teach Pendant (Glass frame pendant)
    create3DMesh('sec_10', new THREE.BoxGeometry(0.12, 0.16, 0.018), glassMat, pendantGroup, new THREE.Vector3(0, 0, 0));
    create3DMesh('pendant_lcd', new THREE.BoxGeometry(0.09, 0.12, 0.003), glassMat, pendantGroup, new THREE.Vector3(0, 0.008, 0.003));
    create3DMesh('pendant_deadman_switch', new THREE.CylinderGeometry(0.006, 0.006, 0.015, 12), darkMat, pendantGroup, new THREE.Vector3(0, -0.06, -0.01));
    create3DMesh('pendant_estop_button', new THREE.CylinderGeometry(0.008, 0.008, 0.01, 16), redMat, pendantGroup, new THREE.Vector3(0.035, 0.06, 0.01));

    // Category 11: End-Effector Parallel Gripper & Tooling Assembly (J6 Transformation Child)
    create3DMesh('sec_11', new THREE.BoxGeometry(0.08, 0.06, 0.08), glassMat, j6Group, new THREE.Vector3(0, 0, 0.04));
    create3DMesh('gripper_coupler_plate', new THREE.CylinderGeometry(0.025, 0.025, 0.008, 24), silverMat, j6Group, new THREE.Vector3(0, 0, 0.018));
    create3DMesh('gripper_base_body', new THREE.BoxGeometry(0.06, 0.04, 0.04), darkMat, j6Group, new THREE.Vector3(0, 0, 0.045));
    create3DMesh('gripper_cylinder_left', new THREE.CylinderGeometry(0.008, 0.008, 0.03, 16), silverMat, j6Group, new THREE.Vector3(-0.015, 0, 0.045));
    create3DMesh('gripper_cylinder_right', new THREE.CylinderGeometry(0.008, 0.008, 0.03, 16), silverMat, j6Group, new THREE.Vector3(0.015, 0, 0.045));
    create3DMesh('gripper_finger_left', new THREE.BoxGeometry(0.01, 0.015, 0.035), silverMat, j6Group, new THREE.Vector3(-0.02, 0, 0.075));
    create3DMesh('gripper_finger_right', new THREE.BoxGeometry(0.01, 0.015, 0.035), silverMat, j6Group, new THREE.Vector3(0.02, 0, 0.075));
    create3DMesh('gripper_pad_left', new THREE.BoxGeometry(0.004, 0.012, 0.02), rubberMat, j6Group, new THREE.Vector3(-0.014, 0, 0.08));
    create3DMesh('gripper_pad_right', new THREE.BoxGeometry(0.004, 0.012, 0.02), rubberMat, j6Group, new THREE.Vector3(0.014, 0, 0.08));

    // Category 12: Workcell Pedestal, Mounting Table & Safety Enclosure Assembly (Base Reference Frame)
    create3DMesh('sec_12', new THREE.BoxGeometry(1.2, 1.2, 1.0), glassMat, baseGroup, new THREE.Vector3(0, 0, -0.5));
    create3DMesh('pedestal_main_column', new THREE.BoxGeometry(0.20, 0.20, 0.95), darkMat, baseGroup, new THREE.Vector3(0, 0, -0.485));
    create3DMesh('pedestal_top_plate', new THREE.BoxGeometry(0.30, 0.30, 0.025), silverMat, baseGroup, new THREE.Vector3(0, 0, -0.0125));
    create3DMesh('pedestal_base_flange', new THREE.BoxGeometry(0.40, 0.40, 0.03), darkMat, baseGroup, new THREE.Vector3(0, 0, -0.985));
}

function determineComponentMaterial(comp) {
    const name = (comp.name || '').toLowerCase();
    const matStr = (comp.material || '').toLowerCase();
    const subStr = (comp.subsystem || '').toLowerCase();

    if (name.includes('magnet') || matStr.includes('ndfeb')) {
        return magnetMat;
    }
    if (matStr.includes('glass') || matStr.includes('polycarbonate') || matStr.includes('acrylic') || matStr.includes('transparent') || name.includes('lcd') || name.includes('optical') || name.includes('disc') || name.includes('window') || name.includes('guard panel') || subStr.includes('optics') || subStr.includes('guarding')) {
        return glassMat;
    }
    if (name.includes('pcb') || name.includes('board') || matStr.includes('fr4')) {
        return pcbMat;
    }
    if (name.includes('ic') || name.includes('chip') || name.includes('resistor') || name.includes('processor') || matStr.includes('semiconductor') || name.includes('transducer') || name.includes('optocoupler') || name.includes('asic') || name.includes('diode') || name.includes('emitter') || name.includes('memory') || name.includes('fpga') || name.includes('ram') || name.includes('flash') || name.includes('microcontroller') || name.includes('regulator')) {
        return chipMat;
    }
    if (matStr.includes('gold') || name.includes('pin contact') || name.includes('contact') || name.includes('connector') || name.includes('terminal pin') || name.includes('inverter') || name.includes('power module')) {
        return goldMat;
    }
    if (matStr.includes('copper') || matStr.includes('cu-') || name.includes('coil') || name.includes('lug') || name.includes('busbar') || name.includes('winding') || name.includes('braid') || name.includes('terminal')) {
        return copperMat;
    }
    if (matStr.includes('rubber') || matStr.includes('silicone') || name.includes('seal') || name.includes('gasket') || name.includes('o-ring') || name.includes('bumper') || matStr.includes('fkm') || matStr.includes('elastomer') || matStr.includes('pur') || matStr.includes('polyurethane') || name.includes('hose') || name.includes('conduit') || name.includes('sleeve') || name.includes('cable') || name.includes('wire') || name.includes('harness') || name.includes('duct') || name.includes('pad') || name.includes('tubing')) {
        return rubberMat;
    }
    if (name.includes('bolt') || name.includes('screw') || name.includes('pin') || name.includes('rivet') || name.includes('fastener') || name.includes('nut') || name.includes('washer') || name.includes('insert') || name.includes('tie') || name.includes('clip') || name.includes('thread') || name.includes('clamp') || name.includes('t-nut') || name.includes('anchor')) {
        return fastenerMat;
    }
    if (name.includes('dark') || matStr.includes('black') || matStr.includes('anodized') || name.includes('brake') || name.includes('motor') || name.includes('reducer') || name.includes('breaker') || name.includes('housing') || name.includes('casting') || name.includes('filter') || name.includes('pedestal') || name.includes('column') || name.includes('frame')) {
        return darkMat;
    }
    return silverMat;
}

function determineComponentGeometry(comp) {
    const name = (comp.name || '').toLowerCase();

    if (name.includes('o-ring') || name.includes('seal') || name.includes('washer') || name.includes('gasket') || name.includes('torus')) {
        return new THREE.TorusGeometry(0.003, 0.0008, 12, 24);
    } else if (name.includes('bolt') || name.includes('screw') || name.includes('pin') || name.includes('shaft') || name.includes('roller') || name.includes('cylinder') || name.includes('cable') || name.includes('hose') || name.includes('conduit') || name.includes('plug') || name.includes('nipple') || name.includes('rivet') || name.includes('insert') || name.includes('bearing') || name.includes('button') || name.includes('switch') || name.includes('tubing') || name.includes('anchor')) {
        return new THREE.CylinderGeometry(0.0015, 0.0015, 0.006, 12);
    } else if (name.includes('panel') || name.includes('plate') || name.includes('board') || name.includes('pcb')) {
        return new THREE.BoxGeometry(0.008, 0.008, 0.0015);
    } else {
        return new THREE.BoxGeometry(0.003, 0.003, 0.003);
    }
}

// Ensure all 1,353 physical components have 3D procedural meshes registered
function ensureAll1093ComponentsHave3DMeshes(spatialManifestData) {
    buildAllDetailed3DComponents();

    const groupMap = {
        'baseGroup': baseGroup,
        'j1Group': j1Group,
        'j2Group': j2Group,
        'j3Group': j3Group,
        'j4Group': j4Group,
        'j5Group': j5Group,
        'j6Group': j6Group,
        'cabinetGroup': cabinetGroup,
        'pendantGroup': pendantGroup
    };

    const manifestMap = spatialManifestData || (window.spatialManifest || {});

    allComponents.forEach((comp, index) => {
        if (!comp3DObjects[comp.id]) {
            const manifestItem = manifestMap[comp.id];
            let parentGrp = baseGroup;
            let pos = [0, 0, 0];
            let rot = [0, 0, 0];
            let scl = [1, 1, 1];

            if (manifestItem) {
                if (manifestItem.parent_group && groupMap[manifestItem.parent_group]) {
                    parentGrp = groupMap[manifestItem.parent_group];
                }
                if (manifestItem.position) pos = manifestItem.position;
                if (manifestItem.rotation) rot = manifestItem.rotation;
                if (manifestItem.scale) scl = manifestItem.scale;
            } else {
                if (comp.system_category_id === 1) {
                    parentGrp = (comp.level >= 3) ? j1Group : baseGroup;
                    const a = (index * 0.35); const r = 0.008 + (index % 6) * 0.0025;
                    pos = [Math.cos(a)*r, Math.sin(a)*r, 0.01 + (index % 10) * 0.006];
                } else if (comp.system_category_id === 2) {
                    parentGrp = j2Group;
                    pos = [(index % 12) * 0.006, ((index % 3) - 1) * 0.003, (index % 6) * 0.008];
                } else if (comp.system_category_id === 3) {
                    parentGrp = j3Group;
                    pos = [(index % 10) * 0.008, ((index % 3) - 1) * 0.003, ((index % 3) - 1) * 0.003];
                } else if (comp.system_category_id === 4) {
                    parentGrp = j4Group;
                    pos = [0.003 + (index % 6) * 0.004, ((index % 3) - 1) * 0.0025, ((index % 3) - 1) * 0.0025];
                } else if (comp.system_category_id === 5) {
                    parentGrp = j5Group;
                    pos = [0, ((index % 3) - 1) * 0.0025, (index % 4) * 0.0025];
                } else if (comp.system_category_id === 6) {
                    parentGrp = j6Group;
                    pos = [((index % 3) - 1) * 0.002, ((index % 3) - 1) * 0.002, 0.003 + (index % 3) * 0.002];
                } else if (comp.system_category_id === 7) {
                    parentGrp = j2Group;
                    pos = [(index % 15) * 0.005, 0.01, 0.01];
                } else if (comp.system_category_id === 8 || comp.system_category_id === 9) {
                    parentGrp = cabinetGroup;
                    pos = [(index % 6) * 0.015 - 0.045, (index % 5) * 0.018 - 0.045, 0.015];
                } else if (comp.system_category_id === 10) {
                    parentGrp = pendantGroup;
                    pos = [(index % 5) * 0.010 - 0.025, (index % 5) * 0.012 - 0.03, 0.003];
                } else if (comp.system_category_id === 11) {
                    parentGrp = j6Group;
                    pos = [(index % 5) * 0.005 - 0.01, (index % 5) * 0.005 - 0.01, 0.04 + (index % 4) * 0.01];
                } else if (comp.system_category_id === 12) {
                    parentGrp = baseGroup;
                    pos = [(index % 7) * 0.1 - 0.3, (index % 7) * 0.1 - 0.3, -0.8 + (index % 5) * 0.2];
                }
            }

            const mat = determineComponentMaterial(comp);
            const geo = determineComponentGeometry(comp);

            create3DMesh(comp.id, geo, mat, parentGrp, pos, rot, scl);
        }
    });
}

// Alias for 1,353 component specification
const ensureAll1353ComponentsHave3DMeshes = ensureAll1093ComponentsHave3DMeshes;

