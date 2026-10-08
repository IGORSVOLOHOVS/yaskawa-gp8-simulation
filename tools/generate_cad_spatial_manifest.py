#!/usr/bin/env python3
import os
import sys
import json
import math

# Add web dir to path to load database
script_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.abspath(os.path.join(script_dir, '..'))
sys.path.insert(0, os.path.join(root_dir, 'cpp_solver', 'web'))

from tree_database import get_robot_physical_tree

def generate_spatial_manifest():
    components = get_robot_physical_tree()
    manifest = {}

    for idx, comp in enumerate(components):
        cid = comp["id"]
        cat_id = comp["system_category_id"]
        level = comp["level"]

        # Default parent frame group
        parent_group = "baseGroup"
        pos = [0.0, 0.0, 0.0]
        rot = [0.0, 0.0, 0.0]
        scale = [1.0, 1.0, 1.0]

        # Base & Axis 1 (S-Axis) - Cat 1
        if cat_id == 1:
            if cid == "sec_1":
                parent_group = "j1Group"
                pos = [0.0, 0.0, 0.05]
            elif cid == "sec_1_casting":
                parent_group = "baseGroup"
                pos = [0.0, 0.0, 0.0]
            elif cid.startswith("base_anchor_bolt_"):
                parent_group = "baseGroup"
                b_idx = int(cid.split("_")[-1])
                angle = (b_idx - 1) * (math.pi / 2.0)
                pos = [round(math.cos(angle) * 0.125, 4), round(math.sin(angle) * 0.125, 4), 0.015]
            elif cid.startswith("base_m12_bolt_"):
                parent_group = "baseGroup"
                b_idx = int(cid.split("_")[-1])
                angle = (b_idx - 1) * (2.0 * math.pi / 8.0)
                pos = [round(math.cos(angle) * 0.105, 4), round(math.sin(angle) * 0.105, 4), 0.025]
            elif cid.startswith("base_dowel_pin_"):
                parent_group = "baseGroup"
                b_idx = int(cid.split("_")[-1])
                angle = (b_idx - 1) * (math.pi / 2.0) + math.pi / 4.0
                pos = [round(math.cos(angle) * 0.115, 4), round(math.sin(angle) * 0.115, 4), 0.02]
            elif cid == "base_ground_lug":
                parent_group = "baseGroup"
                pos = [-0.09, -0.09, 0.02]
            elif cid == "base_cable_gland_plate":
                parent_group = "baseGroup"
                pos = [0.08, -0.07, 0.04]
            elif cid == "base_gland_o_ring":
                parent_group = "baseGroup"
                pos = [0.08, -0.07, 0.045]
            elif cid in ("s_axis_motor", "sgmsv_stator_frame", "sgmsv_stator_core"):
                parent_group = "j1Group"
                offset_z = 0.058 if cid == "sgmsv_stator_frame" else (0.062 if cid == "sgmsv_stator_core" else 0.060)
                pos = [0.0, 0.0, offset_z]
            elif cid.startswith("sgmsv_coil_"):
                parent_group = "j1Group"
                c_idx = int(cid.split("_")[-1])
                angle = (c_idx - 1) * (2.0 * math.pi / 12.0)
                pos = [round(math.cos(angle) * 0.042, 4), round(math.sin(angle) * 0.042, 4), 0.06]
            elif cid == "sgmsv_rotor_shaft":
                parent_group = "j1Group"
                pos = [0.0, 0.0, 0.065]
            elif cid.startswith("sgmsv_magnet_"):
                parent_group = "j1Group"
                m_idx = int(cid.split("_")[-1])
                angle = (m_idx - 1) * (2.0 * math.pi / 8.0)
                pos = [round(math.cos(angle) * 0.024, 4), round(math.sin(angle) * 0.024, 4), 0.06]
            elif cid == "sgmsv_encoder":
                parent_group = "j1Group"
                pos = [0.0, 0.0, 0.125]
            elif cid == "enc_glass_disc":
                parent_group = "j1Group"
                pos = [0.0, 0.0, 0.126]
            elif cid == "enc_led_emitter":
                parent_group = "j1Group"
                pos = [0.02, 0.0, 0.128]
            elif cid == "enc_photodiode_array":
                parent_group = "j1Group"
                pos = [-0.02, 0.0, 0.128]
            elif cid == "enc_pcb":
                parent_group = "j1Group"
                pos = [0.0, 0.0, 0.132]
            elif cid.startswith("enc_pcb_resistor_"):
                parent_group = "j1Group"
                r_idx = int(cid.split("_")[-1])
                pos = [round(((r_idx - 1) % 5 - 2) * 0.008, 4), round(((r_idx - 1) // 5 - 0.5) * 0.008, 4), 0.135]
            elif cid == "s_axis_reducer":
                parent_group = "baseGroup"
                pos = [0.0, 0.0, 0.18]
            elif cid.startswith("rv50_planet_shaft_"):
                parent_group = "baseGroup"
                p_idx = int(cid.split("_")[-1])
                angle = (p_idx - 1) * (2.0 * math.pi / 3.0)
                pos = [round(math.cos(angle) * 0.045, 4), round(math.sin(angle) * 0.045, 4), 0.18]
            elif cid == "rv50_cyc_disc_a":
                parent_group = "baseGroup"
                pos = [0.0, 0.0, 0.172]
            elif cid == "rv50_cyc_disc_b":
                parent_group = "baseGroup"
                pos = [0.0, 0.0, 0.188]
            elif cid.startswith("rv50_pin_roller_"):
                parent_group = "baseGroup"
                pr_idx = int(cid.split("_")[-1])
                angle = (pr_idx - 1) * (2.0 * math.pi / 20.0)
                pos = [round(math.cos(angle) * 0.082, 4), round(math.sin(angle) * 0.082, 4), 0.18]
            elif cid == "s_axis_main_bearing":
                parent_group = "baseGroup"
                pos = [0.0, 0.0, 0.215]
            elif cid == "s_axis_oil_seal":
                parent_group = "baseGroup"
                pos = [0.0, 0.0, 0.228]
            elif cid.startswith("s_axis_flange_bolt_"):
                parent_group = "baseGroup"
                fb_idx = int(cid.split("_")[-1])
                angle = (fb_idx - 1) * (2.0 * math.pi / 16.0)
                pos = [round(math.cos(angle) * 0.078, 4), round(math.sin(angle) * 0.078, 4), 0.24]
            else:
                parent_group = "j1Group" if level >= 3 else "baseGroup"
                p_sub = int(cid.split("_")[-1]) if cid.split("_")[-1].isdigit() else idx
                angle = p_sub * 0.42
                r = 0.02 + (p_sub % 8) * 0.009
                pos = [round(math.cos(angle) * r, 4), round(math.sin(angle) * r, 4), round(0.01 + p_sub * 0.005, 4)]

        # Lower Arm (L-Axis) - Cat 2
        elif cat_id == 2:
            parent_group = "j2Group"
            if cid == "sec_2":
                pos = [0.0, 0.0, 0.0]
            elif cid == "l_arm_casting":
                pos = [0.05, 0.0, 0.15]
            elif cid == "l_axis_motor":
                pos = [0.0, 0.065, 0.0]
            elif cid == "l_axis_reducer":
                pos = [0.0, -0.065, 0.0]
            elif cid == "l_axis_brake":
                pos = [0.0, 0.095, 0.0]
            else:
                p_sub = int(cid.split("_")[-1]) if cid.split("_")[-1].isdigit() else idx
                x_off = round(0.01 + (p_sub * 0.0022), 4)
                y_off = round(((p_sub % 7) - 3) * 0.012, 4)
                z_off = round((p_sub % 11) * 0.022, 4)
                pos = [x_off, y_off, z_off]

        # Upper Arm (U-Axis) - Cat 3
        elif cat_id == 3:
            parent_group = "j3Group"
            if cid == "sec_3":
                pos = [0.0, 0.0, 0.0]
            elif cid == "u_arm_casting":
                pos = [0.15, 0.0, 0.02]
            elif cid == "u_axis_harmonic":
                pos = [0.0, 0.0, 0.02]
            elif cid == "csg32_wave_gen":
                pos = [0.0, 0.0, 0.01]
            elif cid == "csg32_flexspline":
                pos = [0.0, 0.0, 0.025]
            elif cid == "csg32_circular_spline":
                pos = [0.0, 0.0, 0.035]
            else:
                p_sub = int(cid.split("_")[-1]) if cid.split("_")[-1].isdigit() else idx
                x_off = round(0.015 + (p_sub * 0.0022), 4)
                y_off = round(((p_sub % 5) - 2) * 0.008, 4)
                z_off = round((p_sub % 7) * 0.010, 4)
                pos = [x_off, y_off, z_off]

        # Wrist R-Axis - Cat 4
        elif cat_id == 4:
            parent_group = "j4Group"
            if cid == "sec_4":
                pos = [0.0, 0.0, 0.0]
            elif cid == "r_axis_harmonic":
                pos = [0.02, 0.0, 0.0]
            else:
                p_sub = int(cid.split("_")[-1]) if cid.split("_")[-1].isdigit() else idx
                x_off = round(0.008 + (p_sub * 0.0012), 4)
                y_off = round(((p_sub % 5) - 2) * 0.005, 4)
                z_off = round(((p_sub % 7) - 3) * 0.004, 4)
                pos = [x_off, y_off, z_off]

        # Wrist B-Axis - Cat 5
        elif cat_id == 5:
            parent_group = "j5Group"
            if cid == "sec_5":
                pos = [0.0, 0.0, 0.0]
            elif cid == "b_axis_harmonic":
                pos = [0.0, 0.0, 0.012]
            else:
                p_sub = int(cid.split("_")[-1]) if cid.split("_")[-1].isdigit() else idx
                x_off = round(((p_sub % 7) - 3) * 0.004, 4)
                y_off = round(((p_sub % 5) - 2) * 0.005, 4)
                z_off = round(0.002 + (p_sub * 0.0004), 4)
                pos = [x_off, y_off, z_off]

        # Tool Flange T-Axis - Cat 6
        elif cat_id == 6:
            parent_group = "j6Group"
            if cid == "sec_6":
                pos = [0.0, 0.0, 0.0]
            elif cid == "tool_flange_plate":
                pos = [0.0, 0.0, 0.025]
            elif cid == "t_axis_harmonic":
                pos = [0.0, 0.0, 0.01]
            elif cid == "tool_m12_connector":
                pos = [0.02, 0.0, 0.028]
            else:
                p_sub = int(cid.split("_")[-1]) if cid.split("_")[-1].isdigit() else idx
                angle = p_sub * 0.38
                r = round(0.003 + (p_sub % 5) * 0.0035, 4)
                x_off = round(math.cos(angle) * r, 4)
                y_off = round(math.sin(angle) * r, 4)
                z_off = round(0.005 + (p_sub * 0.0003), 4)
                pos = [x_off, y_off, z_off]

        # Dress Pack Harnesses - Cat 7
        elif cat_id == 7:
            parent_group = "j2Group"
            if cid == "sec_7":
                pos = [0.0, 0.04, 0.05]
            else:
                p_sub = int(cid.split("_")[-1]) if cid.split("_")[-1].isdigit() else idx
                x_off = round(0.01 + (p_sub * 0.0042), 4)
                y_off = round(0.038 + (p_sub % 3) * 0.002, 4)
                z_off = round(0.048 + (p_sub % 5) * 0.002, 4)
                pos = [x_off, y_off, z_off]

        # YRC1000 Power Cabinet - Cat 8
        elif cat_id == 8:
            parent_group = "cabinetGroup"
            if cid == "sec_8":
                pos = [0.0, 0.0, 0.0]
            elif cid == "yrc_main_breaker":
                pos = [-0.05, 0.15, 0.05]
            elif cid == "yrc_emc_filter":
                pos = [0.08, 0.12, 0.05]
            elif cid == "yrc_igbt_module":
                pos = [0.0, -0.05, 0.05]
            else:
                p_sub = int(cid.split("_")[-1]) if cid.split("_")[-1].isdigit() else idx
                x_off = round(((p_sub % 11) - 5) * 0.022, 4)
                y_off = round(((p_sub % 9) - 4) * 0.025, 4)
                z_off = round(-0.06 + (p_sub * 0.0012), 4)
                pos = [x_off, y_off, z_off]

        # YRC1000 Logic Boards - Cat 9
        elif cat_id == 9:
            parent_group = "cabinetGroup"
            if cid == "sec_9":
                pos = [0.0, 0.10, 0.05]
            elif cid == "yrc_main_cpu_board":
                pos = [0.0, 0.10, 0.051]
            elif cid == "yrc_cpu_chip":
                pos = [-0.04, 0.108, 0.07]
            elif cid == "yrc_fpga_chip":
                pos = [0.04, 0.108, 0.07]
            elif cid == "yrc_safety_board":
                pos = [0.0, 0.10, -0.05]
            else:
                p_sub = int(cid.split("_")[-1]) if cid.split("_")[-1].isdigit() else idx
                x_off = round(((p_sub % 13) - 6) * 0.015, 4)
                y_off = round(0.095 + (p_sub % 3) * 0.005, 4)
                z_off = round(-0.07 + (p_sub * 0.0015), 4)
                pos = [x_off, y_off, z_off]

        # Teach Pendant - Cat 10
        elif cat_id == 10:
            parent_group = "pendantGroup"
            if cid == "sec_10":
                pos = [0.0, 0.0, 0.0]
            elif cid == "pendant_lcd":
                pos = [0.0, 0.01, 0.005]
            elif cid == "pendant_deadman_switch":
                pos = [0.0, -0.14, -0.02]
            elif cid == "pendant_estop_button":
                pos = [0.08, 0.12, 0.02]
            else:
                p_sub = int(cid.split("_")[-1]) if cid.split("_")[-1].isdigit() else idx
                x_off = round(((p_sub % 7) - 3) * 0.018, 4)
                y_off = round(((p_sub % 9) - 4) * 0.022, 4)
                z_off = round(-0.015 + (p_sub * 0.0005), 4)
                pos = [x_off, y_off, z_off]

        # End-Effector Gripper & Tooling - Cat 11
        elif cat_id == 11:
            parent_group = "j6Group"
            if cid == "sec_11":
                pos = [0.0, 0.0, 0.03]
            elif cid == "gripper_coupler_plate":
                pos = [0.0, 0.0, 0.035]
            elif cid == "gripper_base_body":
                pos = [0.0, 0.0, 0.055]
            elif cid == "gripper_cylinder_left":
                pos = [-0.015, 0.0, 0.055]
            elif cid == "gripper_cylinder_right":
                pos = [0.015, 0.0, 0.055]
            elif cid == "gripper_finger_left":
                pos = [-0.025, 0.0, 0.085]
            elif cid == "gripper_finger_right":
                pos = [0.025, 0.0, 0.085]
            elif cid == "gripper_pad_left":
                pos = [-0.022, 0.0, 0.105]
            elif cid == "gripper_pad_right":
                pos = [0.022, 0.0, 0.105]
            else:
                p_sub = int(cid.split("_")[-1]) if cid.split("_")[-1].isdigit() else idx
                x_off = round(((p_sub % 7) - 3) * 0.006, 4)
                y_off = round(((p_sub % 5) - 2) * 0.005, 4)
                z_off = round(0.040 + (p_sub * 0.0006), 4)
                pos = [x_off, y_off, z_off]

        # Workcell Pedestal, Table & Safety Enclosure - Cat 12
        elif cat_id == 12:
            parent_group = "baseGroup"
            if cid == "sec_12":
                pos = [0.0, 0.0, -1.0]
            elif cid == "pedestal_main_column":
                pos = [0.0, 0.0, -0.50]
            elif cid == "pedestal_top_plate":
                pos = [0.0, 0.0, -0.01]
            elif cid == "pedestal_base_flange":
                pos = [0.0, 0.0, -1.00]
            else:
                p_sub = int(cid.split("_")[-1]) if cid.split("_")[-1].isdigit() else idx
                angle = p_sub * 0.25
                r = round(0.35 + (p_sub % 9) * 0.08, 4)
                x_off = round(math.cos(angle) * r, 4)
                y_off = round(math.sin(angle) * r, 4)
                z_off = round(-0.95 + (p_sub % 11) * 0.18, 4)
                pos = [x_off, y_off, z_off]

        manifest[cid] = {
            "id": cid,
            "parent_group": parent_group,
            "position": pos,
            "rotation": rot,
            "scale": scale
        }

    output_path = os.path.join(root_dir, 'cpp_solver', 'web', 'spatial_manifest.json')
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(manifest, f, indent=2)

    print(f"✅ Generated CAD Spatial 3D Manifest for {len(manifest)} components at {output_path}")
    return manifest

if __name__ == '__main__':
    generate_spatial_manifest()
