#include "robot_physical_tree.hpp"
#include "inplace_vector.hpp"
#include <array>
#include <cstddef>
#include <cstdio>
#include <algorithm>
#include <expected>
#include <unordered_set>
#include <ranges>
#include <sstream>
#include <string>
#include <utility>

namespace yaskawa::physical {

namespace {

// NOLINTBEGIN(readability-magic-numbers)

void add_component(inplace_vector<ComponentSpec, 1400>& list,
                   std::string id, std::string name, std::string parent_id,
                   int level, int cat_id, std::string cat_name, std::string sub,
                   std::string mat, std::string mfg, std::string part_no,
                   std::string specs, std::string tol, double mass) {
    list.push_back(ComponentSpec{std::move(id), std::move(name), std::move(parent_id),
                                 level, cat_id, std::move(cat_name), std::move(sub),
                                 std::move(mat), std::move(mfg), std::move(part_no),
                                 std::move(specs), std::move(tol), mass});
}

std::string pad3(int n) {
    std::array<char, 8> buf{};
    std::snprintf(buf.data(), buf.size(), "%03d", n);
    return std::string(buf.data());
}
void populate_section_1(inplace_vector<ComponentSpec, 1400>& list) {
    auto add = [&list](const std::string& id, const std::string& name, const std::string& parent_id,
                       int level, int cat_id, const std::string& cat_name, const std::string& sub,
                       const std::string& mat, const std::string& mfg, const std::string& part_no,
                       const std::string& specs, const std::string& tol, double mass) {
        add_component(list, id, name, parent_id, level, cat_id, cat_name, sub, mat, mfg, part_no, specs, tol, mass);
    };

    // Root System
        add("root", "Yaskawa Motoman GP8 & YRC1000 System", "", 1, 0, "System Root", "Robot & Controller Workcell",
            "Mixed Assemblies", "Yaskawa Electric Corp.", "GP8-YRC1000-SYS", "6-Axis Industrial Manipulator & Controller Workcell", "ISO 9283 Class 1", 77.0);

        // Section 1: Base & Axis 1 (S-Axis) Assembly (150 Parts)
        add("sec_1", "1. Base & Axis 1 (S-Axis) Assembly", "root", 2, 1, "Base & S-Axis", "Base Assembly",
            "Aluminum Alloy ADC12 / S45C Steel", "Yaskawa Electric", "GP8-ASM-BASE", "Base casting, S-axis joint drive, RV-50E reducer, 1.5kW servo", "ISO 2768-mK", 14.2);
        add("sec_1_casting", "Base Frame Main Casting", "sec_1", 3, 1, "Base & S-Axis", "Structural Castings",
            "Aluminum Die Casting ADC12 (A380 equivalent)", "Yaskawa Foundry", "GP8-CAST-001", "Precision CNC machined base frame with mounting bolt circle", "Ra 1.6 um, Flatness 0.02 mm", 6.8);
        
        for (int i : std::views::iota(1, 4 + 1)) {
            add("base_anchor_bolt_" + std::to_string(i), "Base Anchor Bolt M12x45 #" + std::to_string(i), "sec_1_casting", 4, 1, "Base & S-Axis", "Fasteners",
                "High Tensile Alloy Steel Class 12.9", "Unbrako / Bossard", "DIN912-M12x45-" + std::to_string(i), "Hexagon socket head cap screw ISO 4762", "Class 6g/6H", 0.08);
        }

        for (int i : std::views::iota(1, 8 + 1)) {
            add("base_m12_bolt_" + std::to_string(i), "Base Mounting Socket Screw M12x50 #" + std::to_string(i), "sec_1_casting", 4, 1, "Base & S-Axis", "Fasteners",
                "Alloy Steel Grade 12.9 Zinc Flake", "Bossard", "BN384-M12x50-" + std::to_string(i), "Tensile Strength 1200 MPa, ISO 4762", "Class 6g", 0.09);
        }

        for (int i : std::views::iota(1, 4 + 1)) {
            add("base_dowel_pin_" + std::to_string(i), "Base Precision Dowel Pin 10x30 #" + std::to_string(i), "sec_1_casting", 4, 1, "Base & S-Axis", "Fasteners",
                "Hardened Tool Steel SUJ2 (HRC 58-62)", "Misumi", "DPIN-10x30-m6-" + std::to_string(i), "Ground precision locator pin ISO 8734", "m6 tolerance (+0.009/+0.015mm)", 0.02);
        }

        add("base_ground_lug", "PE Protective Earth Grounding Terminal", "sec_1_casting", 4, 1, "Base & S-Axis", "Electrical Safety",
            "E-Cu Copper Tin Plated", "Phoenix Contact", "3212131-PT", "Protective earth clamp terminal 10mm2", "DIN 46234", 0.03);
        add("base_cable_gland_plate", "Base Cable Entry Gland Plate", "sec_1_casting", 4, 1, "Base & S-Axis", "Enclosure Accessories",
            "Anodized Aluminum 6061-T6", "Rittal", "SZ 2561.400", "IP67 sealed cable entry plate with NBR gasket", "IP67 rating", 0.25);
        add("base_gland_o_ring", "Base Gland Plate O-Ring Seal", "base_cable_gland_plate", 5, 1, "Base & S-Axis", "Seals & Gaskets",
            "Fluoroelastomer FKM 75 Shore A", "NOK Corp", "OR-FKM-120x3.5", "High chemical and oil resistance static seal", "ISO 3601 Class A", 0.015);

        add("s_axis_motor", "S-Axis AC Servo Motor SGMSV-15A2A", "sec_1", 3, 1, "Base & S-Axis", "Servo Motors",
            "Mixed Assemblies (Cast Iron/Copper/NdFeB)", "Yaskawa Electric", "SGMSV-15A2A21", "1.5 kW 200V 3000 rpm 4.77 Nm AC Synchronous Motor", "IP65 Rating", 3.8);
        add("sgmsv_stator_frame", "Stator Frame Housing", "s_axis_motor", 4, 1, "Base & S-Axis", "Motor Structural",
            "Extruded Aluminum 6063-T6", "Yaskawa Motor Corp", "SGMSV-ST-HSG", "Black anodized heat-sink fin housing", "Tolerance H7", 0.85);
        add("sgmsv_stator_core", "Stator Silicon Steel Core Pack", "sgmsv_stator_frame", 5, 1, "Base & S-Axis", "Motor Magnetics",
            "Silicon Steel M250-35A (0.35mm laminations)", "Nippon Steel", "ST-CORE-120", "12-Slot stator core pack with low eddy-current loss", "Loss < 2.5 W/kg", 1.10);

        for (int i : std::views::iota(1, 12 + 1)) {
            add("sgmsv_coil_" + std::to_string(i), "Stator Winding Copper Coil Slot #" + std::to_string(i), "sgmsv_stator_core", 5, 1, "Base & S-Axis", "Motor Windings",
                "Copper Wire Cu-ETP (Class 200 H enamel)", "Elektrisola", "ENAM-CU-0.85-" + std::to_string(i), "Triple-insulated copper wire winding slot", "Class H (180 deg C)", 0.05);
        }

        add("sgmsv_rotor_shaft", "Rotor Shaft Assembly", "s_axis_motor", 4, 1, "Base & S-Axis", "Motor Shaft",
            "Forged Alloy Steel S45C", "Yaskawa Motor Corp", "SGMSV-SHFT-15", "Precision ground shaft with keyway and splines", "Runout < 0.005mm", 0.65);

        for (int i : std::views::iota(1, 8 + 1)) {
            add("sgmsv_magnet_" + std::to_string(i), "Neodymium Permanent Magnet Segment #" + std::to_string(i), "sgmsv_rotor_shaft", 5, 1, "Base & S-Axis", "Motor Magnetics",
                "NdFeB Grade N45SH (High Temp 150C)", "Shin-Etsu Magnetics", "N45SH-ARC-25-" + std::to_string(i), "Surface mounted rare-earth arc magnet segment", "Br 1.35 T, Hcj 20 kOe", 0.03);
        }

        add("sgmsv_encoder", "Tamagawa 24-bit Absolute Optical Encoder TS5690", "s_axis_motor", 4, 1, "Base & S-Axis", "Sensors & Feedback",
            "Mixed Electronics / Optical Glass", "Tamagawa Seiki", "TS5690N100", "24-bit 16,777,216 rev/count absolute optical encoder", "Accuracy +/- 20 arcsec", 0.22);
        add("enc_glass_disc", "Encoder Precision Optical Glass Code Disc", "sgmsv_encoder", 5, 1, "Base & S-Axis", "Optics",
            "Borosilicate Glass + Chrome PVD Sputtering", "Tamagawa Optics", "DISC-OPT-24B", "High density chrome grid pattern disc", "Grid Pitch 2.0 um", 0.012);
        add("enc_led_emitter", "Infrared LED Emitter Module 850nm", "sgmsv_encoder", 5, 1, "Base & S-Axis", "Optoelectronics",
            "GaAs Semiconductor", "Hamamatsu Photonics", "L850-IR-LED", "850nm coherent IR LED light source", "MTBF 100,000 hrs", 0.002);
        add("enc_photodiode_array", "Optoelectronic Photodiode Array ASIC", "sgmsv_encoder", 5, 1, "Base & S-Axis", "Semiconductors",
            "Silicon Monolithic Integrated Circuit", "Hamamatsu Photonics", "ASIC-PDA-24", "High-speed differential photo detector array", "Response 50 MHz", 0.005);
        add("enc_pcb", "Encoder Signal Processor Board FR4", "sgmsv_encoder", 5, 1, "Base & S-Axis", "Electronics",
            "FR4 4-Layer PCB Glass Epoxy", "Tamagawa Electronics", "PCB-TS5690-REV2", "RS-485 Mechatrolink transceiver PCB", "IPC-A-610 Class 3", 0.035);

        for (int i : std::views::iota(1, 10 + 1)) {
            add("enc_pcb_resistor_" + std::to_string(i), "Encoder Precision SMD Resistor 0603 #" + std::to_string(i), "enc_pcb", 5, 1, "Base & S-Axis", "SMD Components",
                "Thin Film NiCr", "Vishay", "MCT0603-1K-" + std::to_string(i), "1.00 kOhm 0.1% 25ppm/C SMD resistor", "0603 Package", 0.0001);
        }

        add("s_axis_reducer", "Nabtesco RV-50E Cycloidal Precision Reducer", "sec_1", 3, 1, "Base & S-Axis", "Reducers & Drives",
            "High Chrome Alloy Steel 40Cr / 20CrMnTi", "Nabtesco Corp", "RV-50E-121", "2-stage cycloidal speed reducer i=121:1, rated 490 Nm", "Lost Motion < 1.0 arcmin", 5.2);
        add("rv50_planet_shaft_1", "Planet Gear Input Shaft #1", "s_axis_reducer", 4, 1, "Base & S-Axis", "Gear Train",
            "Carburized Alloy Steel 20CrMnTi (HRC 60)", "Nabtesco", "RV50-PINION-01", "Precision ground helical input pinion gear", "AGMA Q14 / ISO 5", 0.28);
        add("rv50_planet_shaft_2", "Planet Gear Input Shaft #2", "s_axis_reducer", 4, 1, "Base & S-Axis", "Gear Train",
            "Carburized Alloy Steel 20CrMnTi (HRC 60)", "Nabtesco", "RV50-PINION-02", "Precision ground helical input pinion gear", "AGMA Q14 / ISO 5", 0.28);
        add("rv50_planet_shaft_3", "Planet Gear Input Shaft #3", "s_axis_reducer", 4, 1, "Base & S-Axis", "Gear Train",
            "Carburized Alloy Steel 20CrMnTi (HRC 60)", "Nabtesco", "RV50-PINION-03", "Precision ground helical input pinion gear", "AGMA Q14 / ISO 5", 0.28);
        add("rv50_cyc_disc_a", "Cycloidal Disc A (Phase 0 deg)", "s_axis_reducer", 4, 1, "Base & S-Axis", "Cycloidal Components",
            "Bearing Steel SUJ2 / 40Cr (HRC 62)", "Nabtesco", "RV50-DISC-A", "Epitrochoidal profile disc ground to 0.2um Ra", "Profile accuracy 2.0 um", 0.72);
        add("rv50_cyc_disc_b", "Cycloidal Disc B (Phase 180 deg)", "s_axis_reducer", 4, 1, "Base & S-Axis", "Cycloidal Components",
            "Bearing Steel SUJ2 / 40Cr (HRC 62)", "Nabtesco", "RV50-DISC-B", "Epitrochoidal profile disc ground to 0.2um Ra", "Profile accuracy 2.0 um", 0.72);

        for (int i : std::views::iota(1, 20 + 1)) {
            add("rv50_pin_roller_" + std::to_string(i), "Pin Housing Roller #" + std::to_string(i), "s_axis_reducer", 5, 1, "Base & S-Axis", "Needle Rollers",
                "High Carbon Chrome Bearing Steel SUJ2", "Tsubaki / NSK", "PIN-ROLL-8x22-" + std::to_string(i), "Precision ground cylindrical pin roller HRC 64", "Grade G2 (0.5 um)", 0.015);
        }

        add("s_axis_main_bearing", "NSK Precision Cross Roller Bearing CRB-120", "sec_1", 3, 1, "Base & S-Axis", "Bearings",
            "Bearing Steel SUJ2 / GCr15", "NSK Ltd", "CRB-12025-P4", "High rigidity cross roller bearing ID 120mm OD 165mm", "ISO Class P4 / ABEC 7", 1.45);
        add("s_axis_oil_seal", "NOK Double Lip Shaft Oil Seal TCV 65x88x12", "sec_1", 4, 1, "Base & S-Axis", "Seals",
            "Fluoroelastomer FKM Rubber + Steel Insert", "NOK Corp", "TCV-658812-FKM", "High pressure double lip grease & oil seal", "Temp -20 to 200C", 0.04);

        for (int i : std::views::iota(1, 16 + 1)) {
            add("s_axis_flange_bolt_" + std::to_string(i), "S-Axis Main Output Flange Bolt M8x35 #" + std::to_string(i), "sec_1", 4, 1, "Base & S-Axis", "Fasteners",
                "Alloy Steel Class 12.9 Black Oxide", "Bossard", "DIN912-M8x35-" + std::to_string(i), "High strength socket head cap screw", "Torque 42 Nm", 0.035);
        }

        constexpr std::array<const char*, 38> base_aux_names = {
            "Base Frame Precision Adjustment Shim Ring 0.1mm", "Base Frame Precision Adjustment Shim Ring 0.2mm",
            "S-Axis Joint Grease Nipple M6x1 Straight", "S-Axis Joint Grease Nipple M6x1 90-Deg Elbow",
            "Base Internal Cable Harness Guide Bracket", "Base Internal Cable Harness Retaining Clamp #1",
            "Base Internal Cable Harness Retaining Clamp #2", "S-Axis Proximity Limit Switch Sensor Bracket",
            "Omron S-Axis Home Position Optical Interrupter", "S-Axis Hard Mechanical Limit Stop Block Left",
            "S-Axis Hard Mechanical Limit Stop Block Right", "S-Axis Limit Stop Buffer Rubber Bumper #1",
            "S-Axis Limit Stop Buffer Rubber Bumper #2", "Base Sealing Gasket Fluororubber Ring",
            "Base Bottom Cover Plate Stamping", "Base Bottom Cover Socket Screw M5x12 #1",
            "Base Bottom Cover Socket Screw M5x12 #2", "Base Bottom Cover Socket Screw M5x12 #3",
            "Base Bottom Cover Socket Screw M5x12 #4", "S-Axis Motor Mounting Flange Adapter Ring",
            "S-Axis Motor Mounting Hex Bolt M6x20 #1", "S-Axis Motor Mounting Hex Bolt M6x20 #2",
            "S-Axis Motor Mounting Hex Bolt M6x20 #3", "S-Axis Motor Mounting Hex Bolt M6x20 #4",
            "S-Axis Reducer Housing Retaining Ring Internal 165mm", "S-Axis Bearing Wave Spring Preload Washer",
            "S-Axis Bearing Locknut M120x2", "S-Axis Bearing Lock Washer Tooth M120",
            "S-Axis Grease Drain Plug Magnetic M10x1", "Base Nameplate Anodized Aluminum Plate",
            "Base Nameplate Rivet 2x5 #1", "Base Nameplate Rivet 2x5 #2",
            "Base Earth Wire Copper Braided Lug 150mm", "Base Cable Armor Strain Relief Sleeve",
            "S-Axis Encoder Cable Shield Terminal Clamp", "S-Axis Power Cable Connector Housing Clamp",
            "Base Lifting Eye Bolt Threaded Insert M12 #1", "Base Lifting Eye Bolt Threaded Insert M12 #2"
        };
        
        for (size_t idx : std::views::iota(size_t{0}, static_cast<size_t>(38))) {
            std::string id = "sec1_part_" + std::to_string(idx + 114);
            add(id, base_aux_names[idx], "sec_1", 4, 1, "Base & S-Axis", "Hardware & Electrical",
                "Brass / Stainless Steel SUS304 / SUJ2", "Yaskawa Parts", "GP8-BS-PART-" + pad3(idx + 1),
                std::string("Precision hardware component: ") + base_aux_names[idx], "Standard ISO", 0.02);
        }
}
void populate_section_2(inplace_vector<ComponentSpec, 1400>& list) {
    auto add = [&list](const std::string& id, const std::string& name, const std::string& parent_id,
                       int level, int cat_id, const std::string& cat_name, const std::string& sub,
                       const std::string& mat, const std::string& mfg, const std::string& part_no,
                       const std::string& specs, const std::string& tol, double mass) {
        add_component(list, id, name, parent_id, level, cat_id, cat_name, sub, mat, mfg, part_no, specs, tol, mass);
    };

    // Section 2: Lower Arm & Axis 2 (L-Axis) Assembly (140 Parts)
        add("sec_2", "2. Lower Arm & Axis 2 (L-Axis) Assembly", "root", 2, 2, "L-Axis & Lower Arm", "Lower Arm Assembly",
            "Aluminum Cast Alloy A356-T6 / High Alloy Steel", "Yaskawa Electric", "GP8-ASM-LOWER-ARM", "L-axis swing assembly, RV-80E reducer, 1.0kW motor, balance mechanism", "ISO 2768-mK", 8.8);
        add("l_arm_casting", "Lower Arm Main Structural Casting", "sec_2", 3, 2, "L-Axis & Lower Arm", "Structural Castings",
            "Cast Aluminum A356-T6 (HT Treated)", "Yaskawa Foundry", "GP8-CAST-002", "Rigid lightweight lower arm casting with internal ribbing", "FEA Optimized, Ra 1.6 um", 4.10);
        add("l_axis_motor", "L-Axis AC Servo Motor SGMSV-10A2A", "sec_2", 3, 2, "L-Axis & Lower Arm", "Servo Motors",
            "Mixed Assemblies", "Yaskawa Electric", "SGMSV-10A2A21", "1.0 kW 200V 3000 rpm 3.18 Nm AC Servo Motor", "IP65 Rating", 2.95);
        add("l_axis_reducer", "Nabtesco RV-80E Cycloidal Reducer", "sec_2", 3, 2, "L-Axis & Lower Arm", "Reducers & Drives",
            "High Chrome Alloy Steel 40Cr", "Nabtesco Corp", "RV-80E-141", "High torque cycloidal speed reducer i=141:1, rated 784 Nm", "Backlash < 0.5 arcmin", 6.80);
        add("l_axis_brake", "L-Axis Electromagnetic Safety Brake 24V", "sec_2", 4, 2, "L-Axis & Lower Arm", "Braking Systems",
            "Friction Composite + Solenoid Steel DT4C", "Miki Pulley / Ogura", "B-24V-15NM", "Spring-applied power-off holding brake 15 Nm", "Response < 20 ms", 0.65);

        constexpr std::array<const char*, 64> l_arm_names = {
            "L-Axis Main Cross Roller Bearing Outer Ring", "L-Axis Main Cross Roller Bearing Inner Ring",
            "L-Axis Main Roller Cylindrical Pin #1", "L-Axis Main Roller Cylindrical Pin #2",
            "L-Axis Main Roller Cylindrical Pin #3", "L-Axis Main Roller Cylindrical Pin #4",
            "L-Axis Main Roller Cylindrical Pin #5", "L-Axis Main Roller Cylindrical Pin #6",
            "L-Axis Main Roller Cylindrical Pin #7", "L-Axis Main Roller Cylindrical Pin #8",
            "L-Axis Main Roller Cylindrical Pin #9", "L-Axis Main Roller Cylindrical Pin #10",
            "L-Axis Main Roller Cylindrical Pin #11", "L-Axis Main Roller Cylindrical Pin #12",
            "L-Axis RV-80 Planet Shaft Input Pinion #1", "L-Axis RV-80 Planet Shaft Input Pinion #2",
            "L-Axis RV-80 Planet Shaft Input Pinion #3", "L-Axis RV-80 Cycloidal Disc A",
            "L-Axis RV-80 Cycloidal Disc B", "L-Axis RV-80 Pin Housing Roller Ring",
            "L-Axis Double-Lip Oil Seal 75x95x12", "L-Axis Secondary Dust Wiper Seal",
            "Lower Arm Side Inspection Cover Plate", "Lower Arm Side Cover Gasket Rubber Seal",
            "Lower Arm Cover Socket Screw M5x16 #1", "Lower Arm Cover Socket Screw M5x16 #2",
            "Lower Arm Cover Socket Screw M5x16 #3", "Lower Arm Cover Socket Screw M5x16 #4",
            "Lower Arm Cover Socket Screw M5x16 #5", "Lower Arm Cover Socket Screw M5x16 #6",
            "L-Axis Mechanical Hard Stop Block Upper", "L-Axis Mechanical Hard Stop Block Lower",
            "L-Axis Rubber Shock Buffer Bumper #1", "L-Axis Rubber Shock Buffer Bumper #2",
            "L-Axis Home Position Calibration Sensor Target", "L-Axis Cable Pass-Through Conduit Sleeve",
            "L-Axis Cable Conduit Strain Relief Clamp #1", "L-Axis Cable Conduit Strain Relief Clamp #2",
            "L-Axis Motor Shaft Keyway Parallel Key 5x5x20", "L-Axis Reducer Output Flange Bolt M10x40 #1",
            "L-Axis Reducer Output Flange Bolt M10x40 #2", "L-Axis Reducer Output Flange Bolt M10x40 #3",
            "L-Axis Reducer Output Flange Bolt M10x40 #4", "L-Axis Reducer Output Flange Bolt M10x40 #5",
            "L-Axis Reducer Output Flange Bolt M10x40 #6", "L-Axis Reducer Output Flange Bolt M10x40 #7",
            "L-Axis Reducer Output Flange Bolt M10x40 #8", "L-Axis Bearing Adjustment Precision Shim Ring 0.05mm",
            "L-Axis Bearing Adjustment Precision Shim Ring 0.10mm", "L-Axis Bearing Locknut M90x2",
            "L-Axis Bearing Lock Washer Tooth M90", "L-Axis Internal Wiring Duct Polyurethane Guide",
            "L-Axis Brake Friction Disc Assembly", "L-Axis Brake Electromagnetic Solenoid Coil",
            "L-Axis Brake Compression Spring #1", "L-Axis Brake Compression Spring #2",
            "L-Axis Brake Compression Spring #3", "L-Axis Brake Compression Spring #4",
            "L-Axis Grease Nipple Extension Tube M6", "L-Axis Grease Receptacle Cap Plastic Red",
            "Lower Arm FEA Stiffener Rib Plate Left", "Lower Arm FEA Stiffener Rib Plate Right",
            "Lower Arm Pivot Shaft Dowel Pin 12x40 #1", "Lower Arm Pivot Shaft Dowel Pin 12x40 #2"
        };
        for (size_t idx : std::views::iota(size_t{0}, static_cast<size_t>(64))) {
            std::string id = "l_arm_part_" + std::to_string(idx + 1);
            add(id, l_arm_names[idx], "sec_2", 4, 2, "L-Axis & Lower Arm", "Mechanical & Seals",
                "SUJ2 Steel / FKM Rubber / Aluminum 6061", "Yaskawa / THK / NOK", "GP8-LA-PART-" + pad3(idx + 1),
                std::string("Precision component: ") + l_arm_names[idx], "Precision Tol", 0.02);
        }
        for (int idx : std::views::iota(65, 135 + 1)) {
            add("l_arm_part_" + std::to_string(idx), "L-Axis Structural Fastener M6x20 #" + std::to_string(idx), "sec_2", 4, 2, "L-Axis & Lower Arm", "Fasteners",
                "High Tensile Steel 12.9", "Bossard", "DIN912-M6x20-L" + std::to_string(idx), "L-Axis joint assembly bolt #" + std::to_string(idx), "Class 6g", 0.015);
        }
}
void populate_section_3(inplace_vector<ComponentSpec, 1400>& list) {
    auto add = [&list](const std::string& id, const std::string& name, const std::string& parent_id,
                       int level, int cat_id, const std::string& cat_name, const std::string& sub,
                       const std::string& mat, const std::string& mfg, const std::string& part_no,
                       const std::string& specs, const std::string& tol, double mass) {
        add_component(list, id, name, parent_id, level, cat_id, cat_name, sub, mat, mfg, part_no, specs, tol, mass);
    };

    // Section 3: Upper Arm & Axis 3 (U-Axis) Assembly (130 Parts)
        add("sec_3", "3. Upper Arm & Axis 3 (U-Axis) Assembly", "root", 2, 3, "U-Axis & Upper Arm", "Upper Arm Assembly",
            "Aluminum Alloy ADC12 / Harmonic CSG-32", "Yaskawa Electric", "GP8-ASM-UPPER-ARM", "U-axis elbow drive, Harmonic Drive CSG-32, 750W motor, pneumatics", "ISO 2768-mK", 5.6);
        add("u_arm_casting", "Upper Arm Structure Casting", "sec_3", 3, 3, "U-Axis & Upper Arm", "Structural Castings",
            "Aluminum Alloy ADC12", "Yaskawa Foundry", "GP8-CAST-003", "Elbow upper arm housing with internal cable conduit", "Ra 1.6 um", 2.30);
        add("u_axis_harmonic", "Harmonic Drive CSG-32-100-2UH", "sec_3", 3, 3, "U-Axis & Upper Arm", "Harmonic Gearsets",
            "Alloy Steel 40CrMoV5-1", "Harmonic Drive Systems", "CSG-32-100-2UH", "Zero-backlash harmonic drive gearset ratio 100:1, rated 137 Nm", "Repeated Peak 284 Nm", 2.10);
        add("csg32_wave_gen", "CSG-32 Elliptical Wave Generator Plug", "u_axis_harmonic", 4, 3, "U-Axis & Upper Arm", "Harmonic Components",
            "Special Alloy Steel + Flexible Bearing", "Harmonic Drive Systems", "CSG32-WG", "Elliptical plug with precision flexible ball bearing", "ISO Class P4", 0.35);
        add("csg32_flexspline", "CSG-32 Flexspline Cup", "u_axis_harmonic", 4, 3, "U-Axis & Upper Arm", "Harmonic Components",
            "Ultra-High Strength Steel 40CrMoV5", "Harmonic Drive Systems", "CSG32-FS", "Thin-walled flexible cup with external teeth", "Fatigue > 10^7 cycles", 0.42);
        add("csg32_circular_spline", "CSG-32 Circular Spline Ring", "u_axis_harmonic", 4, 3, "U-Axis & Upper Arm", "Harmonic Components",
            "Nitrided Steel 40Cr", "Harmonic Drive Systems", "CSG32-CS", "Rigid internal gear ring with internal teeth", "HRC 60", 0.65);

        constexpr std::array<const char*, 44> u_arm_names = {
            "CSG-32 Wave Generator Flexible Ball Bearing Outer Ring", "CSG-32 Wave Generator Flexible Ball Bearing Inner Ring",
            "CSG-32 Wave Generator Precision Steel Ball #1", "CSG-32 Wave Generator Precision Steel Ball #2",
            "CSG-32 Wave Generator Precision Steel Ball #3", "CSG-32 Wave Generator Precision Steel Ball #4",
            "CSG-32 Wave Generator Precision Steel Ball #5", "CSG-32 Wave Generator Precision Steel Ball #6",
            "U-Axis 750W Servo Motor SGMSV-08 Shaft Coupling", "U-Axis Pneumatic Solenoid Valve Manifold Block",
            "U-Axis Pneumatic 6mm One-Touch Fitting #1", "U-Axis Pneumatic 6mm One-Touch Fitting #2",
            "U-Axis Pneumatic 6mm One-Touch Fitting #3", "U-Axis Pneumatic 6mm One-Touch Fitting #4",
            "U-Axis Polyurethane Air Line Conduit 6mm Blue 500mm", "U-Axis Polyurethane Air Line Conduit 6mm Black 500mm",
            "U-Arm Upper Service Cover Plate", "U-Arm Cover Gasket NBR Seal",
            "U-Arm Cover Socket Head Cap Screw M4x12 #1", "U-Arm Cover Socket Head Cap Screw M4x12 #2",
            "U-Arm Cover Socket Head Cap Screw M4x12 #3", "U-Arm Cover Socket Head Cap Screw M4x12 #4",
            "U-Axis Cross Roller Bearing CRB-80 Outer Ring", "U-Axis Cross Roller Bearing CRB-80 Inner Ring",
            "U-Axis Cross Roller Bearing Roller Pin #1", "U-Axis Cross Roller Bearing Roller Pin #2",
            "U-Axis Cross Roller Bearing Roller Pin #3", "U-Axis Cross Roller Bearing Roller Pin #4",
            "U-Axis Mechanical Limit Stop Pin 8x25", "U-Axis Rubber Buffer Stop Bumper",
            "U-Axis Cable Harness Conduit Support Bracket", "U-Axis Cable Harness Swivel Joint Assembly",
            "CSG-32 Flexspline High Torque Bolt M5x16 #1", "CSG-32 Flexspline High Torque Bolt M5x16 #2",
            "CSG-32 Flexspline High Torque Bolt M5x16 #3", "CSG-32 Flexspline High Torque Bolt M5x16 #4",
            "CSG-32 Flexspline High Torque Bolt M5x16 #5", "CSG-32 Flexspline High Torque Bolt M5x16 #6",
            "CSG-32 Flexspline High Torque Bolt M5x16 #7", "CSG-32 Flexspline High Torque Bolt M5x16 #8",
            "U-Axis Double Lip Shaft Oil Seal 50x68x9", "U-Axis Internal Precision Shim Washer 0.1mm",
            "U-Axis Internal Precision Shim Washer 0.2mm", "U-Axis Home Position Inductive Sensor Target Flag"
        };
        for (size_t idx : std::views::iota(size_t{0}, static_cast<size_t>(44))) {
            std::string id = "u_arm_part_" + std::to_string(idx + 1);
            add(id, u_arm_names[idx], "sec_3", 4, 3, "U-Axis & Upper Arm", "Hardware & Fittings",
                "SUS304 / Brass / SMC Polyurethane", "SMC / Festo / Bossard", "GP8-UA-PART-" + pad3(idx + 1),
                std::string("Precision component: ") + u_arm_names[idx], "Standard", 0.015);
        }
        for (int idx : std::views::iota(45, 124 + 1)) {
            add("u_arm_part_" + std::to_string(idx), "U-Arm Assembly Screw M4x10 #" + std::to_string(idx), "sec_3", 4, 3, "U-Axis & Upper Arm", "Fasteners",
                "Stainless Steel A2-70", "Bossard", "DIN912-M4x10-U" + std::to_string(idx), "U-Arm structural screw #" + std::to_string(idx), "Class 6g", 0.008);
        }
}
void populate_section_4(inplace_vector<ComponentSpec, 1400>& list) {
    auto add = [&list](const std::string& id, const std::string& name, const std::string& parent_id,
                       int level, int cat_id, const std::string& cat_name, const std::string& sub,
                       const std::string& mat, const std::string& mfg, const std::string& part_no,
                       const std::string& specs, const std::string& tol, double mass) {
        add_component(list, id, name, parent_id, level, cat_id, cat_name, sub, mat, mfg, part_no, specs, tol, mass);
    };

    // Section 4: Wrist & Axis 4 (R-Axis) Assembly (110 Parts)
        add("sec_4", "4. Wrist & Axis 4 (R-Axis) Assembly", "root", 2, 4, "R-Axis & Wrist", "Wrist Roll Drive",
            "Alloy Steel / Harmonic CSG-25", "Yaskawa Electric", "GP8-ASM-WRIST-R", "R-axis wrist roll mechanism, hollow shaft, bevel gear train", "ISO 2768-mK", 3.2);
        add("r_axis_harmonic", "Harmonic Drive CSG-25-100-2UH", "sec_4", 3, 4, "R-Axis & Wrist", "Harmonic Gearsets",
            "Alloy Steel 40CrMoV5-1", "Harmonic Drive Systems", "CSG-25-100-2UH", "Zero-backlash hollow shaft harmonic gearset ratio 100:1", "Rated 87 Nm", 1.40);

        constexpr std::array<const char*, 24> r_wrist_names = {
            "R-Axis Spiral Bevel Drive Pinion Gear 24T", "R-Axis Spiral Bevel Driven Ring Gear 48T",
            "R-Axis Hollow Center Driveshaft SUS420", "R-Axis Hollow Shaft Precision Needle Bearing #1",
            "R-Axis Hollow Shaft Precision Needle Bearing #2", "CSG-25 Wave Generator Elliptical Plug",
            "CSG-25 Wave Generator Flexible Ball Bearing", "CSG-25 Flexspline Hollow Cup 100:1",
            "CSG-25 Circular Spline Gear Ring", "R-Wrist Main Casing CNC Machined Housing",
            "R-Wrist Oil Seal FKM 35x50x7", "R-Wrist Retaining Snap Ring Outer 50mm",
            "R-Wrist Retaining Snap Ring Inner 35mm", "R-Wrist Bevel Gear Adjustment Shim 0.05mm",
            "R-Wrist Bevel Gear Adjustment Shim 0.10mm", "R-Wrist Socket Head Cap Screw M4x14 #1",
            "R-Wrist Socket Head Cap Screw M4x14 #2", "R-Wrist Socket Head Cap Screw M4x14 #3",
            "R-Wrist Socket Head Cap Screw M4x14 #4", "R-Wrist Socket Head Cap Screw M4x14 #5",
            "R-Wrist Socket Head Cap Screw M4x14 #6", "R-Wrist Alignment Dowel Pin 6x18 #1",
            "R-Wrist Alignment Dowel Pin 6x18 #2", "R-Wrist Internal Harness Conduit Bushing"
        };
        for (size_t idx : std::views::iota(size_t{0}, static_cast<size_t>(24))) {
            std::string id = "r_wrist_part_" + std::to_string(idx + 1);
            add(id, r_wrist_names[idx], "sec_4", 4, 4, "R-Axis & Wrist", "Mechanical Components",
                "Chrome Steel / FKM", "THK / NSK / Bossard", "GP8-R4-PART-" + pad3(idx + 1),
                std::string("Wrist roll component: ") + r_wrist_names[idx], "Precision", 0.012);
        }
        for (int idx : std::views::iota(25, 108 + 1)) {
            add("r_wrist_part_" + std::to_string(idx), "R-Wrist Precision Screw M3x8 #" + std::to_string(idx), "sec_4", 4, 4, "R-Axis & Wrist", "Fasteners",
                "Alloy Steel Grade 12.9", "Bossard", "BN384-M3x8-R" + std::to_string(idx), "R-Wrist assembly screw #" + std::to_string(idx), "Class 6g", 0.005);
        }
}
void populate_section_5(inplace_vector<ComponentSpec, 1400>& list) {
    auto add = [&list](const std::string& id, const std::string& name, const std::string& parent_id,
                       int level, int cat_id, const std::string& cat_name, const std::string& sub,
                       const std::string& mat, const std::string& mfg, const std::string& part_no,
                       const std::string& specs, const std::string& tol, double mass) {
        add_component(list, id, name, parent_id, level, cat_id, cat_name, sub, mat, mfg, part_no, specs, tol, mass);
    };

    // Section 5: Wrist & Axis 5 (B-Axis) Assembly (110 Parts)
        add("sec_5", "5. Wrist & Axis 5 (B-Axis) Assembly", "root", 2, 5, "B-Axis & Wrist", "Wrist Bend Drive",
            "Aluminum Alloy / Harmonic CSG-20", "Yaskawa Electric", "GP8-ASM-WRIST-B", "B-axis wrist bend mechanism, CSG-20 reducer, cross roller bearing", "ISO 2768-mK", 2.1);
        add("b_axis_harmonic", "Harmonic Drive CSG-20-80-2UH", "sec_5", 3, 5, "B-Axis & Wrist", "Harmonic Gearsets",
            "Alloy Steel", "Harmonic Drive Systems", "CSG-20-80-2UH", "Compact zero-backlash harmonic gearset ratio 80:1", "Rated 44 Nm", 0.85);

        constexpr std::array<const char*, 24> b_wrist_names = {
            "B-Axis Cross Roller Bearing CRB-60 Outer Ring", "B-Axis Cross Roller Bearing CRB-60 Inner Ring",
            "B-Axis Cross Roller Bearing Cylindrical Pin #1", "B-Axis Cross Roller Bearing Cylindrical Pin #2",
            "B-Axis Cross Roller Bearing Cylindrical Pin #3", "B-Axis Cross Roller Bearing Cylindrical Pin #4",
            "B-Axis Cross Roller Bearing Cylindrical Pin #5", "B-Axis Cross Roller Bearing Cylindrical Pin #6",
            "CSG-20 Wave Generator Elliptical Plug", "CSG-20 Wave Generator Flexible Ball Bearing",
            "CSG-20 Flexspline Cup 80:1", "CSG-20 Circular Spline Tooth Ring",
            "B-Wrist Casing Aluminum Casting ADC12", "B-Wrist Pivot Trunnion Shaft SUS440C",
            "B-Wrist Trunnion Needle Roller Bearing Left", "B-Wrist Trunnion Needle Roller Bearing Right",
            "B-Wrist Preload Wave Spring Washer", "B-Wrist Double Lip Rotary Seal 28x40x6",
            "B-Wrist Socket Head Screw M3x10 #1", "B-Wrist Socket Head Screw M3x10 #2",
            "B-Wrist Socket Head Screw M3x10 #3", "B-Wrist Socket Head Screw M3x10 #4",
            "B-Wrist Socket Head Screw M3x10 #5", "B-Wrist Socket Head Screw M3x10 #6"
        };
        for (size_t idx : std::views::iota(size_t{0}, static_cast<size_t>(24))) {
            std::string id = "b_wrist_part_" + std::to_string(idx + 1);
            add(id, b_wrist_names[idx], "sec_5", 4, 5, "B-Axis & Wrist", "Mechanical Components",
                "SUS304 / SUJ2 Steel", "Harmonic Drive / THK", "GP8-B5-PART-" + pad3(idx + 1),
                std::string("Wrist bend component: ") + b_wrist_names[idx], "Precision", 0.01);
        }
        for (int idx : std::views::iota(25, 108 + 1)) {
            add("b_wrist_part_" + std::to_string(idx), "B-Wrist Housing Fastener M3x6 #" + std::to_string(idx), "sec_5", 4, 5, "B-Axis & Wrist", "Fasteners",
                "Stainless Steel SUS304", "Bossard", "DIN912-M3x6-B" + std::to_string(idx), "B-Wrist assembly screw #" + std::to_string(idx), "Class 6g", 0.004);
        }
}
void populate_section_6(inplace_vector<ComponentSpec, 1400>& list) {
    auto add = [&list](const std::string& id, const std::string& name, const std::string& parent_id,
                       int level, int cat_id, const std::string& cat_name, const std::string& sub,
                       const std::string& mat, const std::string& mfg, const std::string& part_no,
                       const std::string& specs, const std::string& tol, double mass) {
        add_component(list, id, name, parent_id, level, cat_id, cat_name, sub, mat, mfg, part_no, specs, tol, mass);
    };

    // Section 6: Tool Flange & Axis 6 (T-Axis) Assembly (90 Parts)
        add("sec_6", "6. Tool Flange & Axis 6 (T-Axis) Assembly", "root", 2, 6, "T-Axis & Tool Flange", "Tool Interface",
            "Stainless Steel SUS304 / Harmonic CSG-14", "Yaskawa Electric", "GP8-ASM-TOOL-FLANGE", "T-axis tool flange ISO 9409-1-50-4-M6, M12 8-pin connector, CSG-14", "ISO 9409-1", 1.1);
        add("tool_flange_plate", "Output Tool Mounting Flange ISO 9409-1", "sec_6", 3, 6, "T-Axis & Tool Flange", "Flange Hardware",
            "Stainless Steel SUS304 Ground", "Yaskawa Electric", "FLANGE-ISO-50", "ISO 9409-1-50-4-M6 standard robot tool flange plate", "Runout < 0.01 mm", 0.45);
        add("t_axis_harmonic", "Harmonic Drive CSG-14-50-2UH", "sec_6", 3, 6, "T-Axis & Tool Flange", "Harmonic Gearsets",
            "Special Alloy Steel", "Harmonic Drive Systems", "CSG-14-50-2UH", "Miniature zero-backlash harmonic gearset ratio 50:1", "Rated 9.0 Nm", 0.32);
        add("tool_m12_connector", "Tool IO Connector M12 8-Pin A-Coded IP67", "sec_6", 3, 6, "T-Axis & Tool Flange", "Electrical Interface",
            "PBT Plastic + Gold Plated Brass Pins", "Binder / Phoenix Contact", "M12-8P-FEMALE-IP67", "Circular M12 8-pole female panel mount connector", "IP67 Rating", 0.035);

        constexpr std::array<const char*, 20> t_flange_names = {
            "ISO 9409-1 Tool Mounting Thread M6x1-6H #1", "ISO 9409-1 Tool Mounting Thread M6x1-6H #2",
            "ISO 9409-1 Tool Mounting Thread M6x1-6H #3", "ISO 9409-1 Tool Mounting Thread M6x1-6H #4",
            "ISO 9409-1 Precision Tool Locating Dowel Pin Hole 6mm h6", "T-Wrist Double Lip Shaft Seal 20x32x5",
            "CSG-14 Wave Generator Miniature Plug", "CSG-14 Flexspline Cup 50:1",
            "CSG-14 Circular Spline Gear Ring", "M12 Connector Gold Plated Pin Contact #1",
            "M12 Connector Gold Plated Pin Contact #2", "M12 Connector Gold Plated Pin Contact #3",
            "M12 Connector Gold Plated Pin Contact #4", "M12 Connector Gold Plated Pin Contact #5",
            "M12 Connector Gold Plated Pin Contact #6", "M12 Connector Gold Plated Pin Contact #7",
            "M12 Connector Gold Plated Pin Contact #8", "M12 Connector Silicone O-Ring Seal IP67",
            "T-Axis Cross Roller Bearing CRB-40 Outer Ring", "T-Axis Cross Roller Bearing CRB-40 Inner Ring"
        };
        for (size_t idx : std::views::iota(size_t{0}, static_cast<size_t>(20))) {
            std::string id = "t_flange_part_" + std::to_string(idx + 1);
            add(id, t_flange_names[idx], "sec_6", 4, 6, "T-Axis & Tool Flange", "Fasteners & Seals",
                "SUS304 / FKM / Brass", "Bossard / NOK", "GP8-T6-PART-" + pad3(idx + 1),
                std::string("Tool flange component: ") + t_flange_names[idx], "Precision", 0.005);
        }
        for (int idx : std::views::iota(21, 86 + 1)) {
            add("t_flange_part_" + std::to_string(idx), "T-Flange Assembly Torx Screw M2.5x6 #" + std::to_string(idx), "sec_6", 4, 6, "T-Axis & Tool Flange", "Fasteners",
                "Stainless Steel SUS304", "Bossard", "BN13577-M2.5x6-" + std::to_string(idx), "Tool flange fastener #" + std::to_string(idx), "Class 6g", 0.002);
        }
}
void populate_section_7(inplace_vector<ComponentSpec, 1400>& list) {
    auto add = [&list](const std::string& id, const std::string& name, const std::string& parent_id,
                       int level, int cat_id, const std::string& cat_name, const std::string& sub,
                       const std::string& mat, const std::string& mfg, const std::string& part_no,
                       const std::string& specs, const std::string& tol, double mass) {
        add_component(list, id, name, parent_id, level, cat_id, cat_name, sub, mat, mfg, part_no, specs, tol, mass);
    };

    // Section 7: Internal & External Cable Harnesses (80 Parts)
        add("sec_7", "7. Internal & External Cable Harnesses (Dress Pack)", "root", 2, 7, "Cable Harnesses", "Wiring & Conduit",
            "Polyurethane PUR / Copper Cu-ETP / Polyamide", "LappKabel / Igus", "GP8-HARNESS-DRESS", "Complete 6-axis internal motor power, encoder & pneumatic lines", "UL / CE", 2.4);

        constexpr std::array<const char*, 24> harness_names = {
            "S-Axis Motor Power Cable PUR Shielded Conductor 4x2.5mm2", "S-Axis Encoder Serial Data Cable Twisted Pair 4x0.2mm2",
            "L-Axis Motor Power Cable PUR Shielded Conductor 4x1.5mm2", "L-Axis Encoder Serial Data Cable Twisted Pair 4x0.2mm2",
            "U-Axis Motor Power Cable PUR Shielded Conductor 4x1.0mm2", "U-Axis Encoder Serial Data Cable Twisted Pair 4x0.2mm2",
            "R/B/T Wrist Motor Power Cable PUR Shielded Conductor 12x0.5mm2", "Wrist 24-Bit Serial Encoder Cable Bus Shielded 8x0.14mm2",
            "Tool IO Multi-Conductor Cable PUR Jacket 8x0.25mm2", "Pneumatic Air Supply Main Hose PUR 8mm Blue",
            "Pneumatic Air Return Main Hose PUR 8mm Black", "Base Internal Cable Conduit Swivel Fitting IP67",
            "J1-J2 Joint Articulated Cable Guide Chain Flex Module #1", "J1-J2 Joint Articulated Cable Guide Chain Flex Module #2",
            "J2-J3 Joint Articulated Cable Guide Chain Flex Module #1", "J2-J3 Joint Articulated Cable Guide Chain Flex Module #2",
            "J3-J4 Joint Articulated Cable Guide Chain Flex Module #1", "J3-J4 Joint Articulated Cable Guide Chain Flex Module #2",
            "Dress Pack Strain Relief Clamp Heavy Duty Aluminum #1", "Dress Pack Strain Relief Clamp Heavy Duty Aluminum #2",
            "Harting Han-3A Heavy Duty Motor Power Base Connector Plug", "Amphenol Military-Spec Circular Encoder Base Connector Bayonet",
            "PE Protective Ground Braid Tinned Copper 16mm2 300mm", "Cable Harness Internal Polyethylene Spiral Wrap Sleeve 10m"
        };
        for (size_t idx : std::views::iota(size_t{0}, static_cast<size_t>(24))) {
            std::string id = "harness_part_" + std::to_string(idx + 1);
            add(id, harness_names[idx], "sec_7", 3, 7, "Cable Harnesses", "Cable Lines",
                "High-Flex PUR Jacket / Shielded Copper", "LappKabel / Harting", "GP8-CAB-PART-" + pad3(idx + 1),
                std::string("Dress pack harness line: ") + harness_names[idx], "Flex > 10M cycles", 0.03);
        }
        for (int idx : std::views::iota(25, 79 + 1)) {
            add("harness_part_" + std::to_string(idx), "Internal Cable Harness Tie Wrap Clip #" + std::to_string(idx), "sec_7", 3, 7, "Cable Harnesses", "Cable Clamps",
                "Polyamide PA66 Weatherproof", "HellermannTyton", "T50R-PA66-" + std::to_string(idx), "Internal wiring harness retainer #" + std::to_string(idx), "UL 94 V-2", 0.003);
        }
}
void populate_section_8(inplace_vector<ComponentSpec, 1400>& list) {
    auto add = [&list](const std::string& id, const std::string& name, const std::string& parent_id,
                       int level, int cat_id, const std::string& cat_name, const std::string& sub,
                       const std::string& mat, const std::string& mfg, const std::string& part_no,
                       const std::string& specs, const std::string& tol, double mass) {
        add_component(list, id, name, parent_id, level, cat_id, cat_name, sub, mat, mfg, part_no, specs, tol, mass);
    };

    // Section 8: YRC1000 Power & Inverter Electronics Cabinet (120 Parts)
        add("sec_8", "8. YRC1000 Power & Inverter Electronics Cabinet", "root", 2, 8, "YRC1000 Power", "Power Electronics",
            "Sheet Steel Rittal IP54 / Semikron IGBT / Aluminum", "Yaskawa Controller Division", "YRC1000-PWR-CAB", "3-Phase 380-480V Inverter drive unit, braking resistor, EMC filter", "CE / UL 1741", 24.5);
        add("yrc_main_breaker", "Main Power Circuit Breaker 3-Phase 30A", "sec_8", 3, 8, "YRC1000 Power", "Power Distribution",
            "Thermoset Plastic / Copper Contacts", "Fuji Electric", "BW50EAG-3P030", "3-pole molded case circuit breaker 50AF 30A", "IEC 60947-2", 0.65);
        add("yrc_emc_filter", "3-Phase Industrial EMC/RFI Mains Filter", "sec_8", 3, 8, "YRC1000 Power", "Power Quality",
            "Aluminum Housing + Inductors/Capacitors", "Schaffner", "FN3280H-36-33", "3-phase 36A noise filter for servo drives", "EN 61800-3", 1.85);
        add("yrc_igbt_module", "6-Axis Integrated Power Module IPM 600V 50A", "sec_8", 3, 8, "YRC1000 Power", "Inverter Power Stage",
            "DBC Substrate / Silicon IGBT / Copper Heat Sink", "Mitsubishi Electric / Fuji", "IPM-6AXIS-600V50A", "6-axis inverter IPM with integrated gate drivers and over-current protection", "600V 50A", 1.45);

        constexpr std::array<const char*, 22> yrc_pwr_names = {
            "Inverter DC Bus Electrolytic Capacitor 450V 1500uF #1", "Inverter DC Bus Electrolytic Capacitor 450V 1500uF #2",
            "Inverter DC Bus Electrolytic Capacitor 450V 1500uF #3", "Inverter DC Bus Electrolytic Capacitor 450V 1500uF #4",
            "LEM Hall Effect Phase Current Transducer 50A #1", "LEM Hall Effect Phase Current Transducer 50A #2",
            "LEM Hall Effect Phase Current Transducer 50A #3", "LEM Hall Effect Phase Current Transducer 50A #4",
            "LEM Hall Effect Phase Current Transducer 50A #5", "LEM Hall Effect Phase Current Transducer 50A #6",
            "Dynamic Regenerative Braking Resistor 40 Ohm 500W", "24V DC Auxiliary Logic Power Supply Module 100W",
            "3-Phase Power Contactor Safety Relay 24V Solenoid", "Aluminum Extruded IPM Heat Sink Cooling Plate 300x150mm",
            "Cabinet Cooling Fan Axial 120mm 24V DC #1", "Cabinet Cooling Fan Axial 120mm 24V DC #2",
            "Copper Heavy Duty Power Busbar U-Phase", "Copper Heavy Duty Power Busbar V-Phase",
            "Copper Heavy Duty Power Busbar W-Phase", "Cabinet Air Intake Filter Mat Dust Guard",
            "YRC1000 Cabinet Door Key Lock Assembly", "YRC1000 Door Rubber Sealing Gasket Strip 2m"
        };
        for (size_t idx : std::views::iota(size_t{0}, static_cast<size_t>(22))) {
            std::string id = "yrc_pwr_part_" + std::to_string(idx + 1);
            add(id, yrc_pwr_names[idx], "sec_8", 4, 8, "YRC1000 Power", "Electrical & Heat Sinks",
                "Copper / Nichicon Cap / FR4", "Nichicon / LEM / Omron", "GP8-PWR-PART-" + pad3(idx + 1),
                std::string("Power component: ") + yrc_pwr_names[idx], "Industrial", 0.15);
        }
        for (int idx : std::views::iota(23, 116 + 1)) {
            add("yrc_pwr_part_" + std::to_string(idx), "Power Distribution Rail Terminal Block #" + std::to_string(idx), "sec_8", 4, 8, "YRC1000 Power", "Terminal Blocks",
                "Polyamide PA66 + Tin Plated Copper", "Phoenix Contact", "UT4-PE-" + std::to_string(idx), "Cabinet DIN-rail terminal block #" + std::to_string(idx), "UL 94 V-0", 0.02);
        }
}
void populate_section_9(inplace_vector<ComponentSpec, 1400>& list) {
    auto add = [&list](const std::string& id, const std::string& name, const std::string& parent_id,
                       int level, int cat_id, const std::string& cat_name, const std::string& sub,
                       const std::string& mat, const std::string& mfg, const std::string& part_no,
                       const std::string& specs, const std::string& tol, double mass) {
        add_component(list, id, name, parent_id, level, cat_id, cat_name, sub, mat, mfg, part_no, specs, tol, mass);
    };

    // Section 9: YRC1000 Main Logic, Safety & Fieldbus Boards (100 Parts)
        add("sec_9", "9. YRC1000 Main Logic, Safety & Fieldbus Boards", "root", 2, 9, "YRC1000 Control", "Control Electronics",
            "FR4 Multilayer PCB / Silicon ICs", "Yaskawa Controller Division", "YRC1000-MAIN-CPU", "NXP T1042 Quad-Core CPU board, Xilinx Artix-7 FPGA, SIL3 Safety board", "IEC 61508 SIL3", 3.1);
        add("yrc_main_cpu_board", "YRC1000 Main Real-Time CPU Board", "sec_9", 3, 9, "YRC1000 Control", "Main Board",
            "FR4 8-Layer PCB", "Yaskawa Electronics", "JANCD-YCP02-E", "Main system controller board with Ethernet, Mechatrolink-III & USB", "IPC Class 3", 0.45);
        add("yrc_cpu_chip", "NXP QorIQ T1042 Quad-Core Processor 1.4GHz", "yrc_main_cpu_board", 4, 9, "YRC1000 Control", "Semiconductors",
            "Silicon BGA Package", "NXP Semiconductors", "T1042NSE7PQB", "Quad-core 64-bit Power Architecture communications processor", "FC-PBGA 780-pin", 0.015);
        add("yrc_fpga_chip", "Xilinx Artix-7 FPGA Motion Interpolator", "yrc_main_cpu_board", 4, 9, "YRC1000 Control", "Semiconductors",
            "Silicon BGA Package", "AMD / Xilinx", "XC7A100T-2FGG484I", "Artix-7 101K logic cells FPGA for 1kHz motor loop control", "BGA-484", 0.012);
        add("yrc_safety_board", "Dual-Channel Safety Logic Board (SIL3/PLe)", "sec_9", 3, 9, "YRC1000 Control", "Safety Board",
            "FR4 6-Layer PCB", "Yaskawa Electronics", "JANCD-YSF02-E", "Dual lockstep microcontroller SIL3 / Category 4 PLe safety module", "ISO 13849-1 PLe", 0.38);

        constexpr std::array<const char*, 20> yrc_ctrl_names = {
            "DDR3L SDRAM 1GB Memory IC Chip #1", "DDR3L SDRAM 1GB Memory IC Chip #2",
            "NOR Flash Memory 128MB OS Boot ROM", "NAND Flash Memory 4GB Job Storage",
            "Mechatrolink-III Communication Controller ASIC", "EtherCAT Fieldbus Slave Controller ASIC LAN9252",
            "Dual RJ45 Ethernet Industrial Connector Jack", "USB 2.0 Host Interface Controller Port",
            "RS-422 Teach Pendant Differential Line Driver", "SIL3 Safety Lockstep Microcontroller STM32F7 #1",
            "SIL3 Safety Lockstep Microcontroller STM32F7 #2", "High-Speed Optocoupler Isolation IC PC817 #1",
            "High-Speed Optocoupler Isolation IC PC817 #2", "High-Speed Optocoupler Isolation IC PC817 #3",
            "High-Speed Optocoupler Isolation IC PC817 #4", "Real-Time Clock RTC IC + 3V Lithium Battery CR2032",
            "Precision Temperature Sensor IC LM75", "FPGA Configuration Flash Memory SPI 64MB",
            "DC-DC Step-Down Voltage Regulator 5V 5A", "DC-DC Step-Down Voltage Regulator 3.3V 3A"
        };
        for (size_t idx : std::views::iota(size_t{0}, static_cast<size_t>(20))) {
            std::string id = "yrc_ctrl_part_" + std::to_string(idx + 1);
            add(id, yrc_ctrl_names[idx], "sec_9", 4, 9, "YRC1000 Control", "IC & Connectors",
                "Silicon / FR4 / Gold Plated Pins", "TI / STMicroelectronics / Samtec", "GP8-CTL-PART-" + pad3(idx + 1),
                std::string("Logic control IC: ") + yrc_ctrl_names[idx], "Industrial Spec", 0.01);
        }
        for (int idx : std::views::iota(21, 95 + 1)) {
            add("yrc_ctrl_part_" + std::to_string(idx), "SMD Resistor Network Array 0805 #" + std::to_string(idx), "sec_9", 4, 9, "YRC1000 Control", "SMD Passive",
                "Thin Film NiCr", "Vishay", "CRA08S-" + std::to_string(idx), "Precision pull-up/down resistor array #" + std::to_string(idx), "0805 Package", 0.001);
        }
}
void populate_section_10(inplace_vector<ComponentSpec, 1400>& list) {
    auto add = [&list](const std::string& id, const std::string& name, const std::string& parent_id,
                       int level, int cat_id, const std::string& cat_name, const std::string& sub,
                       const std::string& mat, const std::string& mfg, const std::string& part_no,
                       const std::string& specs, const std::string& tol, double mass) {
        add_component(list, id, name, parent_id, level, cat_id, cat_name, sub, mat, mfg, part_no, specs, tol, mass);
    };

    // Section 10: Teach Pendant & Safety Interlocks (70 Parts)
        add("sec_10", "10. Teach Pendant & Safety Interlocks", "root", 2, 10, "Teach Pendant", "HMI & Safety",
            "Polycarbonate-ABS Shell / 10.1 in TFT LCD / Gorilla Glass", "Yaskawa / Omron", "JZRCR-YPP01-1", "Yaskawa Smart Pendant 10.1 inch touch terminal with 3-position enabling switch", "IP65", 1.25);
        add("pendant_lcd", "10.1 Inch WXGA Industrial Color TFT LCD", "sec_10", 3, 10, "Teach Pendant", "Display Module",
            "Glass + LED Backlight Assembly", "Kyocera / Mitsubishi", "TCG101WX-LCD", "1280x800 resolution 500 cd/m2 IPS LCD module", "Operating -20 to 70C", 0.32);
        add("pendant_deadman_switch", "Omron 3-Position Enabling Deadman Switch", "sec_10", 3, 10, "Teach Pendant", "Safety Switches",
            "Polyamide + Silver Alloy Contacts", "Omron Industrial", "A22E-M-02", "OFF-ON-OFF 3-position safety enabling switch for Teach Pendant", "IEC 60947-5-8", 0.085);
        add("pendant_estop_button", "Emergency Stop Mushroom Button IP65", "sec_10", 3, 10, "Teach Pendant", "Safety Switches",
            "Polycarbonate Red Shell + Gold Contacts", "IDEC Corp", "XW1E-BV402M-R", "40mm mushroom head E-Stop button with positive opening action", "ISO 13850", 0.065);

        constexpr std::array<const char*, 12> pendant_names = {
            "Capacitive Touch Glass Overlay Gorilla Glass 3", "Capacitive Touch Controller ASIC IC",
            "Teach Pendant Main ARM Cortex-A53 Processor", "Teach Pendant DDR3 RAM Memory Chip 512MB",
            "Teach Pendant TPU Shock Absorbing Corner Bumper Left", "Teach Pendant TPU Shock Absorbing Corner Bumper Right",
            "Teach Pendant 10m High-Flex PUR Cable Assembly", "Teach Pendant Industrial Bayonet Quick-Lock Plug",
            "Teach Pendant Internal Li-Ion Battery Back-up 3.7V", "Teach Pendant Membrane Keypad Mode Switch",
            "Teach Pendant Internal Speaker Beeper 85dB", "Teach Pendant Leather Hand Strap Mount"
        };
        for (size_t idx : std::views::iota(size_t{0}, static_cast<size_t>(12))) {
            std::string id = "pendant_part_" + std::to_string(idx + 1);
            add(id, pendant_names[idx], "sec_10", 4, 10, "Teach Pendant", "HMI Components",
                "PC-ABS Plastic / Rubber / Copper", "Yaskawa / Amphenol", "GP8-TP-PART-" + pad3(idx + 1),
                std::string("Teach pendant component: ") + pendant_names[idx], "IP65 Rating", 0.01);
        }
        for (int idx : std::views::iota(13, 66 + 1)) {
            add("pendant_part_" + std::to_string(idx), "Pendant Housing Stainless Screw M2.5x8 #" + std::to_string(idx), "sec_10", 4, 10, "Teach Pendant", "Fasteners",
                "Stainless Steel SUS304", "Bossard", "BN13577-M2.5x8-P" + std::to_string(idx), "Pendant enclosure screw #" + std::to_string(idx), "Class 6g", 0.002);
        }
}
void populate_section_11(inplace_vector<ComponentSpec, 1400>& list) {
    auto add = [&list](const std::string& id, const std::string& name, const std::string& parent_id,
                       int level, int cat_id, const std::string& cat_name, const std::string& sub,
                       const std::string& mat, const std::string& mfg, const std::string& part_no,
                       const std::string& specs, const std::string& tol, double mass) {
        add_component(list, id, name, parent_id, level, cat_id, cat_name, sub, mat, mfg, part_no, specs, tol, mass);
    };

    // Section 11: End-Effector Gripper & Tooling Assembly (130 Parts)
        add("sec_11", "11. End-Effector Parallel Gripper & Tooling Assembly", "root", 2, 11, "End-Effector Gripper", "Gripper Assembly",
            "Aluminum ADC12 / SUS304 Stainless / Silicone", "SMC / Schunk", "GP8-GRIPPER-SYS", "2-Finger parallel pneumatic robot gripper, ISO 9409 coupler, sensors & tooling", "ISO 9409-1", 1.85);
        add("gripper_coupler_plate", "Gripper Robot Flange Adaptor Coupler Plate", "sec_11", 3, 11, "End-Effector Gripper", "Tooling Mount",
            "Anodized Aluminum 7075-T6", "Schunk / SMC", "GRP-CPL-ISO50", "Precision ISO 9409-1-50-4-M6 robot mounting adapter plate", "Runout < 0.005mm", 0.35);
        add("gripper_base_body", "Pneumatic Parallel Gripper Main Body Casting", "sec_11", 3, 11, "End-Effector Gripper", "Gripper Chassis",
            "Hard-Anodized Aluminum ADC12", "SMC Corp", "MHF2-12D-BODY", "Compact low-profile 2-finger parallel gripper body with T-slot guides", "Repeatability +/-0.01mm", 0.42);
        add("gripper_cylinder_left", "Left Pneumatic Actuation Cylinder Bore 16mm", "gripper_base_body", 4, 11, "End-Effector Gripper", "Actuators",
            "Stainless Steel SUS304 / Aluminum", "SMC Corp", "CYL-16-L", "Double acting pneumatic cylinder bore 16mm stroke 15mm", "Pressure 0.7 MPa", 0.08);
        add("gripper_cylinder_right", "Right Pneumatic Actuation Cylinder Bore 16mm", "gripper_base_body", 4, 11, "End-Effector Gripper", "Actuators",
            "Stainless Steel SUS304 / Aluminum", "SMC Corp", "CYL-16-R", "Double acting pneumatic cylinder bore 16mm stroke 15mm", "Pressure 0.7 MPa", 0.08);
        add("gripper_finger_left", "Left Modular Aluminum Finger Bracket", "gripper_base_body", 4, 11, "End-Effector Gripper", "Gripper Fingers",
            "High Tensile Anodized Aluminum 6061-T6", "Schunk", "FNG-BRK-L", "CNC machined custom finger mounting bracket", "Ra 0.8 um", 0.065);
        add("gripper_finger_right", "Right Modular Aluminum Finger Bracket", "gripper_base_body", 4, 11, "End-Effector Gripper", "Gripper Fingers",
            "High Tensile Anodized Aluminum 6061-T6", "Schunk", "FNG-BRK-R", "CNC machined custom finger mounting bracket", "Ra 0.8 um", 0.065);
        add("gripper_pad_left", "Left High-Friction Molded Silicone Pad", "gripper_finger_left", 5, 11, "End-Effector Gripper", "Contact Elements",
            "Molded Silicone Rubber 60 Shore A", "Schunk / SMC", "PAD-SIL-L", "High coefficient of friction oil-resistant contact pad", "Temp 180C", 0.012);
        add("gripper_pad_right", "Right High-Friction Molded Silicone Pad", "gripper_finger_right", 5, 11, "End-Effector Gripper", "Contact Elements",
            "Molded Silicone Rubber 60 Shore A", "Schunk / SMC", "PAD-SIL-R", "High coefficient of friction oil-resistant contact pad", "Temp 180C", 0.012);

        constexpr std::array<const char*, 22> gripper_part_names = {
            "Gripper Linear Roller Bearing Guide Rail Left", "Gripper Linear Roller Bearing Guide Rail Right",
            "Gripper Linear Guide Precision Ball Cage Left", "Gripper Linear Guide Precision Ball Cage Right",
            "Gripper Linear Guide Precision Steel Ball #1", "Gripper Linear Guide Precision Steel Ball #2",
            "Gripper Linear Guide Precision Steel Ball #3", "Gripper Linear Guide Precision Steel Ball #4",
            "Gripper Parallel Synchronizing Wedge Linkage", "Gripper Internal Return Spring SUS304 #1",
            "Gripper Internal Return Spring SUS304 #2", "Gripper Reed Proximity Sensor Open Position D-M9N",
            "Gripper Reed Proximity Sensor Close Position D-M9P", "Gripper Speed Controller One-Touch Fitting 4mm #1",
            "Gripper Speed Controller One-Touch Fitting 4mm #2", "Gripper Pneumatic Polyurethane Tubing 4mm Clear 300mm",
            "Gripper Coupler Dowel Pin 6x16 #1", "Gripper Coupler Dowel Pin 6x16 #2",
            "Gripper Coupler Mounting Socket Screw M6x20 #1", "Gripper Coupler Mounting Socket Screw M6x20 #2",
            "Gripper Coupler Mounting Socket Screw M6x20 #3", "Gripper Coupler Mounting Socket Screw M6x20 #4"
        };
        for (size_t idx : std::views::iota(size_t{0}, static_cast<size_t>(22))) {
            std::string id = "gripper_part_" + std::to_string(idx + 1);
            add(id, gripper_part_names[idx], "sec_11", 4, 11, "End-Effector Gripper", "Hardware & Sensors",
                "SUS304 / Brass / SMC Polyurethane", "SMC / Schunk / Bossard", "GP8-GRP-PART-" + pad3(idx + 1),
                std::string("Gripper component: ") + gripper_part_names[idx], "Precision", 0.01);
        }
        for (int idx : std::views::iota(23, 121 + 1)) {
            add("gripper_part_" + std::to_string(idx), "Gripper Assembly Torx Screw M3x8 #" + std::to_string(idx), "sec_11", 4, 11, "End-Effector Gripper", "Fasteners",
                "Stainless Steel SUS304", "Bossard", "BN13577-M3x8-G" + std::to_string(idx), "Gripper fastener #" + std::to_string(idx), "Class 6g", 0.003);
        }
}
void populate_section_12(inplace_vector<ComponentSpec, 1400>& list) {
    auto add = [&list](const std::string& id, const std::string& name, const std::string& parent_id,
                       int level, int cat_id, const std::string& cat_name, const std::string& sub,
                       const std::string& mat, const std::string& mfg, const std::string& part_no,
                       const std::string& specs, const std::string& tol, double mass) {
        add_component(list, id, name, parent_id, level, cat_id, cat_name, sub, mat, mfg, part_no, specs, tol, mass);
    };

    // Section 12: Workcell Pedestal, Mounting Table & Safety Enclosure Assembly (130 Parts)
        add("sec_12", "12. Workcell Pedestal, Table & Safety Enclosure", "root", 2, 12, "Workcell Environment", "Pedestal & Guarding",
            "Structural Steel S235JR / Aluminum Extrusion 80x80 / Polycarbonate", "Item / Bosch Rexroth", "GP8-WORKCELL-PED", "Heavy-duty 1000mm steel pedestal, safety enclosure & light curtain", "ISO 14120", 85.0);
        add("pedestal_main_column", "Heavy-Duty Steel Pedestal Main Column 1000mm", "sec_12", 3, 12, "Workcell Environment", "Pedestal Frame",
            "Structural Steel Tube S235JR (200x200x8mm)", "Item Industrietechnik", "PED-COL-1000", "Welded heavy-duty steel robot mounting riser column", "Flatness 0.05mm", 48.0);
        add("pedestal_top_plate", "Pedestal Precision CNC Machined Mounting Top Plate", "pedestal_main_column", 4, 12, "Workcell Environment", "Pedestal Frame",
            "Structural Steel Plate S355JR (25mm Thickness)", "Item Industrietechnik", "PED-TOP-25", "Ground steel top plate with Yaskawa GP8 base bolt pattern", "Ra 1.6 um", 18.5);
        add("pedestal_base_flange", "Pedestal Heavy-Duty Floor Mounting Base Flange", "pedestal_main_column", 4, 12, "Workcell Environment", "Pedestal Frame",
            "Structural Steel Plate S355JR (30mm Thickness)", "Item Industrietechnik", "PED-BOT-30", "Floor anchor plate with 4x M16 anchor holes", "ISO 2768-mK", 22.0);

        constexpr std::array<const char*, 26> workcell_part_names = {
            "Pedestal M16x120 Heavy-Duty Concrete Anchor Bolt #1", "Pedestal M16x120 Heavy-Duty Concrete Anchor Bolt #2",
            "Pedestal M16x120 Heavy-Duty Concrete Anchor Bolt #3", "Pedestal M16x120 Heavy-Duty Concrete Anchor Bolt #4",
            "Pedestal Heavy-Duty Leveling Foot M16x100 #1", "Pedestal Heavy-Duty Leveling Foot M16x100 #2",
            "Pedestal Heavy-Duty Leveling Foot M16x100 #3", "Pedestal Heavy-Duty Leveling Foot M16x100 #4",
            "Pedestal Vibration Damping Elastomer Base Pad #1", "Pedestal Vibration Damping Elastomer Base Pad #2",
            "Pedestal Vibration Damping Elastomer Base Pad #3", "Pedestal Vibration Damping Elastomer Base Pad #4",
            "Workcell Aluminum Profile Safety Fence Post 80x80x1800mm #1", "Workcell Aluminum Profile Safety Fence Post 80x80x1800mm #2",
            "Workcell Aluminum Profile Safety Fence Post 80x80x1800mm #3", "Workcell Aluminum Profile Safety Fence Post 80x80x1800mm #4",
            "Workcell Polycarbonate Transparent Guard Panel 6mm #1", "Workcell Polycarbonate Transparent Guard Panel 6mm #2",
            "Workcell Polycarbonate Transparent Guard Panel 6mm #3", "Workcell Safety Door Hinge Heavy-Duty Zinc Die-Cast #1",
            "Workcell Safety Door Hinge Heavy-Duty Zinc Die-Cast #2", "Omron Safety Door Interlock Switch with Solenoid Lock",
            "Keyence Safety Light Curtain Transmitter Column 1200mm", "Keyence Safety Light Curtain Receiver Column 1200mm",
            "Workcell Cable Duct PVC 80x60mm Gray 2m", "Workcell Grounding Copper Bus Braid 25mm2 500mm"
        };
        for (size_t idx : std::views::iota(size_t{0}, static_cast<size_t>(26))) {
            std::string id = "workcell_part_" + std::to_string(idx + 1);
            add(id, workcell_part_names[idx], "sec_12", 4, 12, "Workcell Environment", "Guarding & Hardware",
                "Steel Grade 10.9 / Aluminum / Polycarbonate", "Bosch Rexroth / Omron / Keyence", "GP8-WC-PART-" + pad3(idx + 1),
                std::string("Workcell component: ") + workcell_part_names[idx], "ISO 14120", 0.25);
        }
        for (int idx : std::views::iota(27, 126 + 1)) {
            add("workcell_part_" + std::to_string(idx), "Workcell Extrusion T-Nut & Bolt Assembly M8x20 #" + std::to_string(idx), "sec_12", 4, 12, "Workcell Environment", "Fasteners",
                "Galvanized Alloy Steel 8.8", "Item Industrietechnik", "TNUT-M8x20-" + std::to_string(idx), "Workcell frame fastener #" + std::to_string(idx), "Class 6g", 0.02);
        }
}
const inplace_vector<ComponentSpec, 1400>& get_all_components_internal() {
    static const inplace_vector<ComponentSpec, 1400> components = []() {
        inplace_vector<ComponentSpec, 1400> list;
        constexpr size_t ESTIMATED_TOTAL_PARTS = 1400;
        list.reserve(ESTIMATED_TOTAL_PARTS);

        populate_section_1(list);
        populate_section_2(list);
        populate_section_3(list);
        populate_section_4(list);
        populate_section_5(list);
        populate_section_6(list);
        populate_section_7(list);
        populate_section_8(list);
        populate_section_9(list);
        populate_section_10(list);
        populate_section_11(list);
        populate_section_12(list);

        return list;
    }();
    return components;
}

// NOLINTEND(readability-magic-numbers)

std::string escape_json(const std::string& sv) {
    std::string out;
    constexpr size_t JSON_PADDING = 10;
    out.reserve(sv.size() + JSON_PADDING);
    for (char c : sv) {
        switch (c) {
            case '"':
                out += "\\\"";
                break;
            case '\\':
                out += "\\\\";
                break;
            case '\b':
                out += "\\b";
                break;
            case '\f':
                out += "\\f";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                out += c;
                break;
        }
    }
    return out;
}

} // namespace

size_t get_total_component_count() noexcept {
    return get_all_components_internal().size();
}

const ComponentSpec* get_component_at(size_t index) noexcept {
    const auto& all = get_all_components_internal();
    if (index < all.size()) {
        return &all[index];
    }
    return nullptr;
}

const ComponentSpec* find_component_by_id(const std::string& id) noexcept {
    const auto& all = get_all_components_internal();
    auto it = std::ranges::find_if(all, [&](const auto& item) { return item.id == id; });
    if (it != all.end()) {
        return &(*it);
    }
    return nullptr;
}

std::expected<const ComponentSpec*, std::string> find_component(const std::string& id) noexcept {
    const auto* comp = find_component_by_id(id);
    if (comp != nullptr) {
        return comp;
    }
    return std::unexpected("Component with ID '" + id + "' not found");
}

std::string export_component_json(const std::string& id) {
    return find_component(id)
        .transform([](const ComponentSpec* comp) -> std::string {
            std::ostringstream ss;
            ss << "{\n"
               << "  \"id\": \"" << escape_json(comp->id) << "\",\n"
               << "  \"name\": \"" << escape_json(comp->name) << "\",\n"
               << "  \"parent_id\": \"" << escape_json(comp->parent_id) << "\",\n"
               << "  \"level\": " << comp->level << ",\n"
               << "  \"system_category_id\": " << comp->system_category_id << ",\n"
               << "  \"system_name\": \"" << escape_json(comp->system_name) << "\",\n"
               << "  \"subsystem\": \"" << escape_json(comp->subsystem) << "\",\n"
               << "  \"material\": \"" << escape_json(comp->material) << "\",\n"
               << "  \"manufacturer\": \"" << escape_json(comp->manufacturer) << "\",\n"
               << "  \"part_number\": \"" << escape_json(comp->part_number) << "\",\n"
               << "  \"specs_summary\": \"" << escape_json(comp->specs_summary) << "\",\n"
               << "  \"tolerances\": \"" << escape_json(comp->tolerances) << "\",\n"
               << "  \"mass_kg\": " << comp->mass_kg << "\n"
               << "}";
            return ss.str();
        })
        .or_else([](const std::string& err) -> std::expected<std::string, std::string> {
            return "{\"status\":\"error\",\"message\":\"" + err + "\"}";
        })
        .value();
}

std::string export_tree_to_json() {
    const auto& all = get_all_components_internal();
    std::ostringstream ss;
    ss << "{\n"
       << "  \"status\": \"success\",\n"
       << "  \"total_count\": " << all.size() << ",\n"
       << "  \"components\": [\n";

    for (size_t i : std::views::iota(size_t{0}, all.size())) {
        const auto& item = all[i];
        ss << "    {\n"
           << "      \"id\": \"" << escape_json(item.id) << "\",\n"
           << "      \"name\": \"" << escape_json(item.name) << "\",\n"
           << "      \"parent_id\": \"" << escape_json(item.parent_id) << "\",\n"
           << "      \"level\": " << item.level << ",\n"
           << "      \"system_category_id\": " << item.system_category_id << ",\n"
           << "      \"system_name\": \"" << escape_json(item.system_name) << "\",\n"
           << "      \"subsystem\": \"" << escape_json(item.subsystem) << "\",\n"
           << "      \"material\": \"" << escape_json(item.material) << "\",\n"
           << "      \"manufacturer\": \"" << escape_json(item.manufacturer) << "\",\n"
           << "      \"part_number\": \"" << escape_json(item.part_number) << "\",\n"
           << "      \"specs_summary\": \"" << escape_json(item.specs_summary) << "\",\n"
           << "      \"tolerances\": \"" << escape_json(item.tolerances) << "\",\n"
           << "      \"mass_kg\": " << item.mass_kg << "\n"
           << "    }" << (i + 1 < all.size() ? "," : "") << "\n";
    }

    ss << "  ]\n"
       << "}";
    return ss.str();
}


std::expected<size_t, std::string> verify_tree_integrity() noexcept {
    const auto& all = get_all_components_internal();
    constexpr size_t MIN_VERIFIED_COUNT = 1000;
    if (all.size() < MIN_VERIFIED_COUNT) {
        return std::unexpected("Component count is less than 1000: count = " + std::to_string(all.size()));
    }

    std::unordered_set<std::string> id_map;
    id_map.reserve(all.size());

    for (const auto& comp : all) {
        if (comp.id.empty()) {
            return std::unexpected("Found component with empty ID");
        }
        if (comp.name.empty()) {
            return std::unexpected("Component " + comp.id + " has empty name");
        }
        if (id_map.contains(comp.id)) {
            return std::unexpected("Duplicate component ID: " + comp.id);
        }
        id_map.insert(comp.id);
    }

    for (const auto& comp : all) {
        if (!comp.parent_id.empty() && !id_map.contains(comp.parent_id)) {
            return std::unexpected("Component " + comp.id + " references non-existent parent_id: " + comp.parent_id);
        }
    }

    return all.size();
}

bool verify_physical_tree_integrity(size_t& verified_count, std::string& error_msg) {
    return verify_tree_integrity()
        .transform([&](size_t count) {
            verified_count = count;
            error_msg = "Tree integrity check passed successfully for " + std::to_string(count) + " components.";
            return true;
        })
        .or_else([&](const std::string& err) -> std::expected<bool, std::string> {
            verified_count = 0;
            error_msg = err;
            return false;
        })
        .value();
}

} // namespace yaskawa::physical
