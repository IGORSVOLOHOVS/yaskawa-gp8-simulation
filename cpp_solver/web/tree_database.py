import json

def get_robot_physical_tree():
    components = []
    
    def add(part_id, name, parent_id, level, cat_id, cat_name, sub, mat, mfg, part_no, specs, tol, mass):
        components.append({
            "id": part_id,
            "name": name,
            "parent_id": parent_id,
            "level": level,
            "system_category_id": cat_id,
            "system_name": cat_name,
            "subsystem": sub,
            "material": mat,
            "manufacturer": mfg,
            "part_number": part_no,
            "specs_summary": specs,
            "tolerances": tol,
            "mass_kg": mass
        })

    # Root
    add("root", "Yaskawa Motoman GP8 & YRC1000 System", "", 1, 0, "System Root", "Robot & Controller Workcell",
        "Mixed Assemblies", "Yaskawa Electric Corp.", "GP8-YRC1000-SYS", "6-Axis Industrial Manipulator & Controller Workcell", "ISO 9283 Class 1", 77.0)

    # Section 1: Base & Axis 1 (S-Axis) Assembly (150 Parts)
    add("sec_1", "1. Base & Axis 1 (S-Axis) Assembly", "root", 2, 1, "Base & S-Axis", "Base Assembly",
        "Aluminum Alloy ADC12 / S45C Steel", "Yaskawa Electric", "GP8-ASM-BASE", "Base casting, S-axis joint drive, RV-50E reducer, 1.5kW servo", "ISO 2768-mK", 14.2)
    add("sec_1_casting", "Base Frame Main Casting", "sec_1", 3, 1, "Base & S-Axis", "Structural Castings",
        "Aluminum Die Casting ADC12 (A380 equivalent)", "Yaskawa Foundry", "GP8-CAST-001", "Precision CNC machined base frame with mounting bolt circle", "Ra 1.6 um, Flatness 0.02 mm", 6.8)
    
    for i in range(1, 5):
        add(f"base_anchor_bolt_{i}", f"Base Anchor Bolt M12x45 #{i}", "sec_1_casting", 4, 1, "Base & S-Axis", "Fasteners",
            "High Tensile Alloy Steel Class 12.9", "Unbrako / Bossard", f"DIN912-M12x45-{i}", "Hexagon socket head cap screw ISO 4762", "Class 6g/6H", 0.08)

    for i in range(1, 9):
        add(f"base_m12_bolt_{i}", f"Base Mounting Socket Screw M12x50 #{i}", "sec_1_casting", 4, 1, "Base & S-Axis", "Fasteners",
            "Alloy Steel Grade 12.9 Zinc Flake", "Bossard", f"BN384-M12x50-{i}", "Tensile Strength 1200 MPa, ISO 4762", "Class 6g", 0.09)

    for i in range(1, 5):
        add(f"base_dowel_pin_{i}", f"Base Precision Dowel Pin 10x30 #{i}", "sec_1_casting", 4, 1, "Base & S-Axis", "Fasteners",
            "Hardened Tool Steel SUJ2 (HRC 58-62)", "Misumi", f"DPIN-10x30-m6-{i}", "Ground precision locator pin ISO 8734", "m6 tolerance (+0.009/+0.015mm)", 0.02)

    add("base_ground_lug", "PE Protective Earth Grounding Terminal", "sec_1_casting", 4, 1, "Base & S-Axis", "Electrical Safety",
        "E-Cu Copper Tin Plated", "Phoenix Contact", "3212131-PT", "Protective earth clamp terminal 10mm2", "DIN 46234", 0.03)
    add("base_cable_gland_plate", "Base Cable Entry Gland Plate", "sec_1_casting", 4, 1, "Base & S-Axis", "Enclosure Accessories",
        "Anodized Aluminum 6061-T6", "Rittal", "SZ 2561.400", "IP67 sealed cable entry plate with NBR gasket", "IP67 rating", 0.25)
    add("base_gland_o_ring", "Base Gland Plate O-Ring Seal", "base_cable_gland_plate", 5, 1, "Base & S-Axis", "Seals & Gaskets",
        "Fluoroelastomer FKM 75 Shore A", "NOK Corp", "OR-FKM-120x3.5", "High chemical and oil resistance static seal", "ISO 3601 Class A", 0.015)

    add("s_axis_motor", "S-Axis AC Servo Motor SGMSV-15A2A", "sec_1", 3, 1, "Base & S-Axis", "Servo Motors",
        "Mixed Assemblies (Cast Iron/Copper/NdFeB)", "Yaskawa Electric", "SGMSV-15A2A21", "1.5 kW 200V 3000 rpm 4.77 Nm AC Synchronous Motor", "IP65 Rating", 3.8)
    add("sgmsv_stator_frame", "Stator Frame Housing", "s_axis_motor", 4, 1, "Base & S-Axis", "Motor Structural",
        "Extruded Aluminum 6063-T6", "Yaskawa Motor Corp", "SGMSV-ST-HSG", "Black anodized heat-sink fin housing", "Tolerance H7", 0.85)
    add("sgmsv_stator_core", "Stator Silicon Steel Core Pack", "sgmsv_stator_frame", 5, 1, "Base & S-Axis", "Motor Magnetics",
        "Silicon Steel M250-35A (0.35mm laminations)", "Nippon Steel", "ST-CORE-120", "12-Slot stator core pack with low eddy-current loss", "Loss < 2.5 W/kg", 1.10)

    for i in range(1, 13):
        add(f"sgmsv_coil_{i}", f"Stator Winding Copper Coil Slot #{i}", "sgmsv_stator_core", 5, 1, "Base & S-Axis", "Motor Windings",
            "Copper Wire Cu-ETP (Class 200 H enamel)", "Elektrisola", f"ENAM-CU-0.85-{i}", "Triple-insulated copper wire winding slot", "Class H (180 deg C)", 0.05)

    add("sgmsv_rotor_shaft", "Rotor Shaft Assembly", "s_axis_motor", 4, 1, "Base & S-Axis", "Motor Shaft",
        "Forged Alloy Steel S45C", "Yaskawa Motor Corp", "SGMSV-SHFT-15", "Precision ground shaft with keyway and splines", "Runout < 0.005mm", 0.65)

    for i in range(1, 9):
        add(f"sgmsv_magnet_{i}", f"Neodymium Permanent Magnet Segment #{i}", "sgmsv_rotor_shaft", 5, 1, "Base & S-Axis", "Motor Magnetics",
            "NdFeB Grade N45SH (High Temp 150C)", "Shin-Etsu Magnetics", f"N45SH-ARC-25-{i}", "Surface mounted rare-earth arc magnet segment", "Br 1.35 T, Hcj 20 kOe", 0.03)

    add("sgmsv_encoder", "Tamagawa 24-bit Absolute Optical Encoder TS5690", "s_axis_motor", 4, 1, "Base & S-Axis", "Sensors & Feedback",
        "Mixed Electronics / Optical Glass", "Tamagawa Seiki", "TS5690N100", "24-bit 16,777,216 rev/count absolute optical encoder", "Accuracy +/- 20 arcsec", 0.22)
    add("enc_glass_disc", "Encoder Precision Optical Glass Code Disc", "sgmsv_encoder", 5, 1, "Base & S-Axis", "Optics",
        "Borosilicate Glass + Chrome PVD Sputtering", "Tamagawa Optics", "DISC-OPT-24B", "High density chrome grid pattern disc", "Grid Pitch 2.0 um", 0.012)
    add("enc_led_emitter", "Infrared LED Emitter Module 850nm", "sgmsv_encoder", 5, 1, "Base & S-Axis", "Optoelectronics",
        "GaAs Semiconductor", "Hamamatsu Photonics", "L850-IR-LED", "850nm coherent IR LED light source", "MTBF 100,000 hrs", 0.002)
    add("enc_photodiode_array", "Optoelectronic Photodiode Array ASIC", "sgmsv_encoder", 5, 1, "Base & S-Axis", "Semiconductors",
        "Silicon Monolithic Integrated Circuit", "Hamamatsu Photonics", "ASIC-PDA-24", "High-speed differential photo detector array", "Response 50 MHz", 0.005)
    add("enc_pcb", "Encoder Signal Processor Board FR4", "sgmsv_encoder", 5, 1, "Base & S-Axis", "Electronics",
        "FR4 4-Layer PCB Glass Epoxy", "Tamagawa Electronics", "PCB-TS5690-REV2", "RS-485 Mechatrolink transceiver PCB", "IPC-A-610 Class 3", 0.035)

    for i in range(1, 11):
        add(f"enc_pcb_resistor_{i}", f"Encoder Precision SMD Resistor 0603 #{i}", "enc_pcb", 5, 1, "Base & S-Axis", "SMD Components",
            "Thin Film NiCr", "Vishay", f"MCT0603-1K-{i}", "1.00 kOhm 0.1% 25ppm/C SMD resistor", "0603 Package", 0.0001)

    add("s_axis_reducer", "Nabtesco RV-50E Cycloidal Precision Reducer", "sec_1", 3, 1, "Base & S-Axis", "Reducers & Drives",
        "High Chrome Alloy Steel 40Cr / 20CrMnTi", "Nabtesco Corp", "RV-50E-121", "2-stage cycloidal speed reducer i=121:1, rated 490 Nm", "Lost Motion < 1.0 arcmin", 5.2)
    add("rv50_planet_shaft_1", "Planet Gear Input Shaft #1", "s_axis_reducer", 4, 1, "Base & S-Axis", "Gear Train",
        "Carburized Alloy Steel 20CrMnTi (HRC 60)", "Nabtesco", "RV50-PINION-01", "Precision ground helical input pinion gear", "AGMA Q14 / ISO 5", 0.28)
    add("rv50_planet_shaft_2", "Planet Gear Input Shaft #2", "s_axis_reducer", 4, 1, "Base & S-Axis", "Gear Train",
        "Carburized Alloy Steel 20CrMnTi (HRC 60)", "Nabtesco", "RV50-PINION-02", "Precision ground helical input pinion gear", "AGMA Q14 / ISO 5", 0.28)
    add("rv50_planet_shaft_3", "Planet Gear Input Shaft #3", "s_axis_reducer", 4, 1, "Base & S-Axis", "Gear Train",
        "Carburized Alloy Steel 20CrMnTi (HRC 60)", "Nabtesco", "RV50-PINION-03", "Precision ground helical input pinion gear", "AGMA Q14 / ISO 5", 0.28)
    add("rv50_cyc_disc_a", "Cycloidal Disc A (Phase 0 deg)", "s_axis_reducer", 4, 1, "Base & S-Axis", "Cycloidal Components",
        "Bearing Steel SUJ2 / 40Cr (HRC 62)", "Nabtesco", "RV50-DISC-A", "Epitrochoidal profile disc ground to 0.2um Ra", "Profile accuracy 2.0 um", 0.72)
    add("rv50_cyc_disc_b", "Cycloidal Disc B (Phase 180 deg)", "s_axis_reducer", 4, 1, "Base & S-Axis", "Cycloidal Components",
        "Bearing Steel SUJ2 / 40Cr (HRC 62)", "Nabtesco", "RV50-DISC-B", "Epitrochoidal profile disc ground to 0.2um Ra", "Profile accuracy 2.0 um", 0.72)

    for i in range(1, 21):
        add(f"rv50_pin_roller_{i}", f"Pin Housing Roller #{i}", "s_axis_reducer", 5, 1, "Base & S-Axis", "Needle Rollers",
            "High Carbon Chrome Bearing Steel SUJ2", "Tsubaki / NSK", f"PIN-ROLL-8x22-{i}", "Precision ground cylindrical pin roller HRC 64", "Grade G2 (0.5 um)", 0.015)

    add("s_axis_main_bearing", "NSK Precision Cross Roller Bearing CRB-120", "sec_1", 3, 1, "Base & S-Axis", "Bearings",
        "Bearing Steel SUJ2 / GCr15", "NSK Ltd", "CRB-12025-P4", "High rigidity cross roller bearing ID 120mm OD 165mm", "ISO Class P4 / ABEC 7", 1.45)
    add("s_axis_oil_seal", "NOK Double Lip Shaft Oil Seal TCV 65x88x12", "sec_1", 4, 1, "Base & S-Axis", "Seals",
        "Fluoroelastomer FKM Rubber + Steel Insert", "NOK Corp", "TCV-658812-FKM", "High pressure double lip grease & oil seal", "Temp -20 to 200C", 0.04)

    for i in range(1, 17):
        add(f"s_axis_flange_bolt_{i}", f"S-Axis Main Output Flange Bolt M8x35 #{i}", "sec_1", 4, 1, "Base & S-Axis", "Fasteners",
            "Alloy Steel Class 12.9 Black Oxide", "Bossard", f"DIN912-M8x35-{i}", "High strength socket head cap screw", "Torque 42 Nm", 0.035)

    # 38 Base Auxiliary Specific Hardware Components
    base_aux_names = [
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
    ]
    for idx, bname in enumerate(base_aux_names, 1):
        add(f"sec1_part_{idx+113}", bname, "sec_1", 4, 1, "Base & S-Axis", "Hardware & Electrical",
            "Brass / Stainless Steel SUS304 / SUJ2", "Yaskawa Parts", f"GP8-BS-PART-{idx:03d}", f"Precision hardware component: {bname}", "Standard ISO", 0.02)

    # Section 2: Lower Arm & Axis 2 (L-Axis) Assembly (140 Parts)
    add("sec_2", "2. Lower Arm & Axis 2 (L-Axis) Assembly", "root", 2, 2, "L-Axis & Lower Arm", "Lower Arm Assembly",
        "Aluminum Cast Alloy A356-T6 / High Alloy Steel", "Yaskawa Electric", "GP8-ASM-LOWER-ARM", "L-axis swing assembly, RV-80E reducer, 1.0kW motor, balance mechanism", "ISO 2768-mK", 8.8)
    add("l_arm_casting", "Lower Arm Main Structural Casting", "sec_2", 3, 2, "L-Axis & Lower Arm", "Structural Castings",
        "Cast Aluminum A356-T6 (HT Treated)", "Yaskawa Foundry", "GP8-CAST-002", "Rigid lightweight lower arm casting with internal ribbing", "FEA Optimized, Ra 1.6 um", 4.10)
    add("l_axis_motor", "L-Axis AC Servo Motor SGMSV-10A2A", "sec_2", 3, 2, "L-Axis & Lower Arm", "Servo Motors",
        "Mixed Assemblies", "Yaskawa Electric", "SGMSV-10A2A21", "1.0 kW 200V 3000 rpm 3.18 Nm AC Servo Motor", "IP65 Rating", 2.95)
    add("l_axis_reducer", "Nabtesco RV-80E Cycloidal Reducer", "sec_2", 3, 2, "L-Axis & Lower Arm", "Reducers & Drives",
        "High Chrome Alloy Steel 40Cr", "Nabtesco Corp", "RV-80E-141", "High torque cycloidal speed reducer i=141:1, rated 784 Nm", "Backlash < 0.5 arcmin", 6.80)
    add("l_axis_brake", "L-Axis Electromagnetic Safety Brake 24V", "sec_2", 4, 2, "L-Axis & Lower Arm", "Braking Systems",
        "Friction Composite + Solenoid Steel DT4C", "Miki Pulley / Ogura", "B-24V-15NM", "Spring-applied power-off holding brake 15 Nm", "Response < 20 ms", 0.65)

    l_arm_names = [
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
    ]
    for idx, lname in enumerate(l_arm_names, 1):
        add(f"l_arm_part_{idx}", lname, "sec_2", 4, 2, "L-Axis & Lower Arm", "Mechanical & Seals",
            "SUJ2 Steel / FKM Rubber / Aluminum 6061", "Yaskawa / THK / NOK", f"GP8-LA-PART-{idx:03d}", f"Precision component: {lname}", "Precision Tol", 0.02)

    for idx in range(len(l_arm_names) + 1, 136):
        add(f"l_arm_part_{idx}", f"L-Axis Structural Fastener M6x20 #{idx}", "sec_2", 4, 2, "L-Axis & Lower Arm", "Fasteners",
            "High Tensile Steel 12.9", "Bossard", f"DIN912-M6x20-L{idx}", f"L-Axis joint assembly bolt #{idx}", "Class 6g", 0.015)

    # Section 3: Upper Arm & Axis 3 (U-Axis) Assembly (130 Parts)
    add("sec_3", "3. Upper Arm & Axis 3 (U-Axis) Assembly", "root", 2, 3, "U-Axis & Upper Arm", "Upper Arm Assembly",
        "Aluminum Alloy ADC12 / Harmonic CSG-32", "Yaskawa Electric", "GP8-ASM-UPPER-ARM", "U-axis elbow drive, Harmonic Drive CSG-32, 750W motor, pneumatics", "ISO 2768-mK", 5.6)
    add("u_arm_casting", "Upper Arm Structure Casting", "sec_3", 3, 3, "U-Axis & Upper Arm", "Structural Castings",
        "Aluminum Alloy ADC12", "Yaskawa Foundry", "GP8-CAST-003", "Elbow upper arm housing with internal cable conduit", "Ra 1.6 um", 2.30)
    add("u_axis_harmonic", "Harmonic Drive CSG-32-100-2UH", "sec_3", 3, 3, "U-Axis & Upper Arm", "Harmonic Gearsets",
        "Alloy Steel 40CrMoV5-1", "Harmonic Drive Systems", "CSG-32-100-2UH", "Zero-backlash harmonic drive gearset ratio 100:1, rated 137 Nm", "Repeated Peak 284 Nm", 2.10)
    add("csg32_wave_gen", "CSG-32 Elliptical Wave Generator Plug", "u_axis_harmonic", 4, 3, "U-Axis & Upper Arm", "Harmonic Components",
        "Special Alloy Steel + Flexible Bearing", "Harmonic Drive Systems", "CSG32-WG", "Elliptical plug with precision flexible ball bearing", "ISO Class P4", 0.35)
    add("csg32_flexspline", "CSG-32 Flexspline Cup", "u_axis_harmonic", 4, 3, "U-Axis & Upper Arm", "Harmonic Components",
        "Ultra-High Strength Steel 40CrMoV5", "Harmonic Drive Systems", "CSG32-FS", "Thin-walled flexible cup with external teeth", "Fatigue > 10^7 cycles", 0.42)
    add("csg32_circular_spline", "CSG-32 Circular Spline Ring", "u_axis_harmonic", 4, 3, "U-Axis & Upper Arm", "Harmonic Components",
        "Nitrided Steel 40Cr", "Harmonic Drive Systems", "CSG32-CS", "Rigid internal gear ring with internal teeth", "HRC 60", 0.65)

    u_arm_names = [
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
    ]
    for idx, uname in enumerate(u_arm_names, 1):
        add(f"u_arm_part_{idx}", uname, "sec_3", 4, 3, "U-Axis & Upper Arm", "Hardware & Fittings",
            "SUS304 / Brass / SMC Polyurethane", "SMC / Festo / Bossard", f"GP8-UA-PART-{idx:03d}", f"Precision component: {uname}", "Standard", 0.015)

    for idx in range(len(u_arm_names) + 1, 125):
        add(f"u_arm_part_{idx}", f"U-Arm Assembly Screw M4x10 #{idx}", "sec_3", 4, 3, "U-Axis & Upper Arm", "Fasteners",
            "Stainless Steel A2-70", "Bossard", f"DIN912-M4x10-U{idx}", f"U-Arm structural screw #{idx}", "Class 6g", 0.008)

    # Section 4: Wrist & Axis 4 (R-Axis) Assembly (110 Parts)
    add("sec_4", "4. Wrist & Axis 4 (R-Axis) Assembly", "root", 2, 4, "R-Axis & Wrist", "Wrist Roll Drive",
        "Alloy Steel / Harmonic CSG-25", "Yaskawa Electric", "GP8-ASM-WRIST-R", "R-axis wrist roll mechanism, hollow shaft, bevel gear train", "ISO 2768-mK", 3.2)
    add("r_axis_harmonic", "Harmonic Drive CSG-25-100-2UH", "sec_4", 3, 4, "R-Axis & Wrist", "Harmonic Gearsets",
        "Alloy Steel 40CrMoV5-1", "Harmonic Drive Systems", "CSG-25-100-2UH", "Zero-backlash hollow shaft harmonic gearset ratio 100:1", "Rated 87 Nm", 1.40)

    r_wrist_names = [
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
    ]
    for idx, rname in enumerate(r_wrist_names, 1):
        add(f"r_wrist_part_{idx}", rname, "sec_4", 4, 4, "R-Axis & Wrist", "Mechanical Components",
            "Chrome Steel / FKM", "THK / NSK / Bossard", f"GP8-R4-PART-{idx:03d}", f"Wrist roll component: {rname}", "Precision", 0.012)

    for idx in range(len(r_wrist_names) + 1, 109):
        add(f"r_wrist_part_{idx}", f"R-Wrist Precision Screw M3x8 #{idx}", "sec_4", 4, 4, "R-Axis & Wrist", "Fasteners",
            "Alloy Steel Grade 12.9", "Bossard", f"BN384-M3x8-R{idx}", f"R-Wrist assembly screw #{idx}", "Class 6g", 0.005)

    # Section 5: Wrist & Axis 5 (B-Axis) Assembly (110 Parts)
    add("sec_5", "5. Wrist & Axis 5 (B-Axis) Assembly", "root", 2, 5, "B-Axis & Wrist", "Wrist Bend Drive",
        "Aluminum Alloy / Harmonic CSG-20", "Yaskawa Electric", "GP8-ASM-WRIST-B", "B-axis wrist bend mechanism, CSG-20 reducer, cross roller bearing", "ISO 2768-mK", 2.1)
    add("b_axis_harmonic", "Harmonic Drive CSG-20-80-2UH", "sec_5", 3, 5, "B-Axis & Wrist", "Harmonic Gearsets",
        "Alloy Steel", "Harmonic Drive Systems", "CSG-20-80-2UH", "Compact zero-backlash harmonic gearset ratio 80:1", "Rated 44 Nm", 0.85)

    b_wrist_names = [
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
    ]
    for idx, bname in enumerate(b_wrist_names, 1):
        add(f"b_wrist_part_{idx}", bname, "sec_5", 4, 5, "B-Axis & Wrist", "Mechanical Components",
            "SUS304 / SUJ2 Steel", "Harmonic Drive / THK", f"GP8-B5-PART-{idx:03d}", f"Wrist bend component: {bname}", "Precision", 0.01)

    for idx in range(len(b_wrist_names) + 1, 109):
        add(f"b_wrist_part_{idx}", f"B-Wrist Housing Fastener M3x6 #{idx}", "sec_5", 4, 5, "B-Axis & Wrist", "Fasteners",
            "Stainless Steel SUS304", "Bossard", f"DIN912-M3x6-B{idx}", f"B-Wrist assembly screw #{idx}", "Class 6g", 0.004)

    # Section 6: Tool Flange & Axis 6 (T-Axis) Assembly (90 Parts)
    add("sec_6", "6. Tool Flange & Axis 6 (T-Axis) Assembly", "root", 2, 6, "T-Axis & Tool Flange", "Tool Interface",
        "Stainless Steel SUS304 / Harmonic CSG-14", "Yaskawa Electric", "GP8-ASM-TOOL-FLANGE", "T-axis tool flange ISO 9409-1-50-4-M6, M12 8-pin connector, CSG-14", "ISO 9409-1", 1.1)
    add("tool_flange_plate", "Output Tool Mounting Flange ISO 9409-1", "sec_6", 3, 6, "T-Axis & Tool Flange", "Flange Hardware",
        "Stainless Steel SUS304 Ground", "Yaskawa Electric", "FLANGE-ISO-50", "ISO 9409-1-50-4-M6 standard robot tool flange plate", "Runout < 0.01 mm", 0.45)
    add("t_axis_harmonic", "Harmonic Drive CSG-14-50-2UH", "sec_6", 3, 6, "T-Axis & Tool Flange", "Harmonic Gearsets",
        "Special Alloy Steel", "Harmonic Drive Systems", "CSG-14-50-2UH", "Miniature zero-backlash harmonic gearset ratio 50:1", "Rated 9.0 Nm", 0.32)
    add("tool_m12_connector", "Tool IO Connector M12 8-Pin A-Coded IP67", "sec_6", 3, 6, "T-Axis & Tool Flange", "Electrical Interface",
        "PBT Plastic + Gold Plated Brass Pins", "Binder / Phoenix Contact", "M12-8P-FEMALE-IP67", "Circular M12 8-pole female panel mount connector", "IP67 Rating", 0.035)

    t_flange_names = [
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
    ]
    for idx, tname in enumerate(t_flange_names, 1):
        add(f"t_flange_part_{idx}", tname, "sec_6", 4, 6, "T-Axis & Tool Flange", "Fasteners & Seals",
            "SUS304 / FKM / Brass", "Bossard / NOK", f"GP8-T6-PART-{idx:03d}", f"Tool flange component: {tname}", "Precision", 0.005)

    for idx in range(len(t_flange_names) + 1, 87):
        add(f"t_flange_part_{idx}", f"T-Flange Assembly Torx Screw M2.5x6 #{idx}", "sec_6", 4, 6, "T-Axis & Tool Flange", "Fasteners",
            "Stainless Steel SUS304", "Bossard", f"BN13577-M2.5x6-{idx}", f"Tool flange fastener #{idx}", "Class 6g", 0.002)

    # Section 7: Internal & External Cable Harnesses (80 Parts)
    add("sec_7", "7. Internal & External Cable Harnesses (Dress Pack)", "root", 2, 7, "Cable Harnesses", "Wiring & Conduit",
        "Polyurethane PUR / Copper Cu-ETP / Polyamide", "LappKabel / Igus", "GP8-HARNESS-DRESS", "Complete 6-axis internal motor power, encoder & pneumatic lines", "UL / CE", 2.4)

    harness_names = [
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
    ]
    for idx, hname in enumerate(harness_names, 1):
        add(f"harness_part_{idx}", hname, "sec_7", 3, 7, "Cable Harnesses", "Cable Lines",
            "High-Flex PUR Jacket / Shielded Copper", "LappKabel / Harting", f"GP8-CAB-PART-{idx:03d}", f"Dress pack harness line: {hname}", "Flex > 10M cycles", 0.03)

    for idx in range(len(harness_names) + 1, 80):
        add(f"harness_part_{idx}", f"Internal Cable Harness Tie Wrap Clip #{idx}", "sec_7", 3, 7, "Cable Harnesses", "Cable Clamps",
            "Polyamide PA66 Weatherproof", "HellermannTyton", f"T50R-PA66-{idx}", f"Internal wiring harness retainer #{idx}", "UL 94 V-2", 0.003)

    # Section 8: YRC1000 Power & Inverter Electronics Cabinet (120 Parts)
    add("sec_8", "8. YRC1000 Power & Inverter Electronics Cabinet", "root", 2, 8, "YRC1000 Power", "Power Electronics",
        "Sheet Steel Rittal IP54 / Semikron IGBT / Aluminum", "Yaskawa Controller Division", "YRC1000-PWR-CAB", "3-Phase 380-480V Inverter drive unit, braking resistor, EMC filter", "CE / UL 1741", 24.5)
    add("yrc_main_breaker", "Main Power Circuit Breaker 3-Phase 30A", "sec_8", 3, 8, "YRC1000 Power", "Power Distribution",
        "Thermoset Plastic / Copper Contacts", "Fuji Electric", "BW50EAG-3P030", "3-pole molded case circuit breaker 50AF 30A", "IEC 60947-2", 0.65)
    add("yrc_emc_filter", "3-Phase Industrial EMC/RFI Mains Filter", "sec_8", 3, 8, "YRC1000 Power", "Power Quality",
        "Aluminum Housing + Inductors/Capacitors", "Schaffner", "FN3280H-36-33", "3-phase 36A noise filter for servo drives", "EN 61800-3", 1.85)
    add("yrc_igbt_module", "6-Axis Integrated Power Module IPM 600V 50A", "sec_8", 3, 8, "YRC1000 Power", "Inverter Power Stage",
        "DBC Substrate / Silicon IGBT / Copper Heat Sink", "Mitsubishi Electric / Fuji", "IPM-6AXIS-600V50A", "6-axis inverter IPM with integrated gate drivers and over-current protection", "600V 50A", 1.45)

    yrc_pwr_names = [
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
    ]
    for idx, pname in enumerate(yrc_pwr_names, 1):
        add(f"yrc_pwr_part_{idx}", pname, "sec_8", 4, 8, "YRC1000 Power", "Electrical & Heat Sinks",
            "Copper / Nichicon Cap / FR4", "Nichicon / LEM / Omron", f"GP8-PWR-PART-{idx:03d}", f"Power component: {pname}", "Industrial", 0.15)

    for idx in range(len(yrc_pwr_names) + 1, 117):
        add(f"yrc_pwr_part_{idx}", f"Power Distribution Rail Terminal Block #{idx}", "sec_8", 4, 8, "YRC1000 Power", "Terminal Blocks",
            "Polyamide PA66 + Tin Plated Copper", "Phoenix Contact", f"UT4-PE-{idx}", f"Cabinet DIN-rail terminal block #{idx}", "UL 94 V-0", 0.02)

    # Section 9: YRC1000 Main Logic, Safety & Fieldbus Boards (100 Parts)
    add("sec_9", "9. YRC1000 Main Logic, Safety & Fieldbus Boards", "root", 2, 9, "YRC1000 Control", "Control Electronics",
        "FR4 Multilayer PCB / Silicon ICs", "Yaskawa Controller Division", "YRC1000-MAIN-CPU", "NXP T1042 Quad-Core CPU board, Xilinx Artix-7 FPGA, SIL3 Safety board", "IEC 61508 SIL3", 3.1)
    add("yrc_main_cpu_board", "YRC1000 Main Real-Time CPU Board", "sec_9", 3, 9, "YRC1000 Control", "Main Board",
        "FR4 8-Layer PCB", "Yaskawa Electronics", "JANCD-YCP02-E", "Main system controller board with Ethernet, Mechatrolink-III & USB", "IPC Class 3", 0.45)
    add("yrc_cpu_chip", "NXP QorIQ T1042 Quad-Core Processor 1.4GHz", "yrc_main_cpu_board", 4, 9, "YRC1000 Control", "Semiconductors",
        "Silicon BGA Package", "NXP Semiconductors", "T1042NSE7PQB", "Quad-core 64-bit Power Architecture communications processor", "FC-PBGA 780-pin", 0.015)
    add("yrc_fpga_chip", "Xilinx Artix-7 FPGA Motion Interpolator", "yrc_main_cpu_board", 4, 9, "YRC1000 Control", "Semiconductors",
        "Silicon BGA Package", "AMD / Xilinx", "XC7A100T-2FGG484I", "Artix-7 101K logic cells FPGA for 1kHz motor loop control", "BGA-484", 0.012)
    add("yrc_safety_board", "Dual-Channel Safety Logic Board (SIL3/PLe)", "sec_9", 3, 9, "YRC1000 Control", "Safety Board",
        "FR4 6-Layer PCB", "Yaskawa Electronics", "JANCD-YSF02-E", "Dual lockstep microcontroller SIL3 / Category 4 PLe safety module", "ISO 13849-1 PLe", 0.38)

    yrc_ctrl_names = [
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
    ]
    for idx, cname in enumerate(yrc_ctrl_names, 1):
        add(f"yrc_ctrl_part_{idx}", cname, "sec_9", 4, 9, "YRC1000 Control", "IC & Connectors",
            "Silicon / FR4 / Gold Plated Pins", "TI / STMicroelectronics / Samtec", f"GP8-CTL-PART-{idx:03d}", f"Logic control IC: {cname}", "Industrial Spec", 0.01)

    for idx in range(len(yrc_ctrl_names) + 1, 96):
        add(f"yrc_ctrl_part_{idx}", f"SMD Resistor Network Array 0805 #{idx}", "sec_9", 4, 9, "YRC1000 Control", "SMD Passive",
            "Thin Film NiCr", "Vishay", f"CRA08S-{idx}", f"Precision pull-up/down resistor array #{idx}", "0805 Package", 0.001)

    # Section 10: Teach Pendant & Safety Interlocks (70 Parts)
    add("sec_10", "10. Teach Pendant & Safety Interlocks", "root", 2, 10, "Teach Pendant", "HMI & Safety",
        "Polycarbonate-ABS Shell / 10.1 in TFT LCD / Gorilla Glass", "Yaskawa / Omron", "JZRCR-YPP01-1", "Yaskawa Smart Pendant 10.1 inch touch terminal with 3-position enabling switch", "IP65", 1.25)
    add("pendant_lcd", "10.1 Inch WXGA Industrial Color TFT LCD", "sec_10", 3, 10, "Teach Pendant", "Display Module",
        "Glass + LED Backlight Assembly", "Kyocera / Mitsubishi", "TCG101WX-LCD", "1280x800 resolution 500 cd/m2 IPS LCD module", "Operating -20 to 70C", 0.32)
    add("pendant_deadman_switch", "Omron 3-Position Enabling Deadman Switch", "sec_10", 3, 10, "Teach Pendant", "Safety Switches",
        "Polyamide + Silver Alloy Contacts", "Omron Industrial", "A22E-M-02", "OFF-ON-OFF 3-position safety enabling switch for Teach Pendant", "IEC 60947-5-8", 0.085)
    add("pendant_estop_button", "Emergency Stop Mushroom Button IP65", "sec_10", 3, 10, "Teach Pendant", "Safety Switches",
        "Polycarbonate Red Shell + Gold Contacts", "IDEC Corp", "XW1E-BV402M-R", "40mm mushroom head E-Stop button with positive opening action", "ISO 13850", 0.065)

    pendant_names = [
        "Capacitive Touch Glass Overlay Gorilla Glass 3", "Capacitive Touch Controller ASIC IC",
        "Teach Pendant Main ARM Cortex-A53 Processor", "Teach Pendant DDR3 RAM Memory Chip 512MB",
        "Teach Pendant TPU Shock Absorbing Corner Bumper Left", "Teach Pendant TPU Shock Absorbing Corner Bumper Right",
        "Teach Pendant 10m High-Flex PUR Cable Assembly", "Teach Pendant Industrial Bayonet Quick-Lock Plug",
        "Teach Pendant Internal Li-Ion Battery Back-up 3.7V", "Teach Pendant Membrane Keypad Mode Switch",
        "Teach Pendant Internal Speaker Beeper 85dB", "Teach Pendant Leather Hand Strap Mount"
    ]
    for idx, tpname in enumerate(pendant_names, 1):
        add(f"pendant_part_{idx}", tpname, "sec_10", 4, 10, "Teach Pendant", "HMI Components",
            "PC-ABS Plastic / Rubber / Copper", "Yaskawa / Amphenol", f"GP8-TP-PART-{idx:03d}", f"Teach pendant component: {tpname}", "IP65 Rating", 0.01)

    for idx in range(len(pendant_names) + 1, 67):
        add(f"pendant_part_{idx}", f"Pendant Housing Stainless Screw M2.5x8 #{idx}", "sec_10", 4, 10, "Teach Pendant", "Fasteners",
            "Stainless Steel SUS304", "Bossard", f"BN13577-M2.5x8-P{idx}", f"Pendant enclosure screw #{idx}", "Class 6g", 0.002)

    # Section 11: End-Effector Gripper & Tooling Assembly (130 Parts)
    add("sec_11", "11. End-Effector Parallel Gripper & Tooling Assembly", "root", 2, 11, "End-Effector Gripper", "Gripper Assembly",
        "Aluminum ADC12 / SUS304 Stainless / Silicone", "SMC / Schunk", "GP8-GRIPPER-SYS", "2-Finger parallel pneumatic robot gripper, ISO 9409 coupler, sensors & tooling", "ISO 9409-1", 1.85)
    add("gripper_coupler_plate", "Gripper Robot Flange Adaptor Coupler Plate", "sec_11", 3, 11, "End-Effector Gripper", "Tooling Mount",
        "Anodized Aluminum 7075-T6", "Schunk / SMC", "GRP-CPL-ISO50", "Precision ISO 9409-1-50-4-M6 robot mounting adapter plate", "Runout < 0.005mm", 0.35)
    add("gripper_base_body", "Pneumatic Parallel Gripper Main Body Casting", "sec_11", 3, 11, "End-Effector Gripper", "Gripper Chassis",
        "Hard-Anodized Aluminum ADC12", "SMC Corp", "MHF2-12D-BODY", "Compact low-profile 2-finger parallel gripper body with T-slot guides", "Repeatability +/-0.01mm", 0.42)
    add("gripper_cylinder_left", "Left Pneumatic Actuation Cylinder Bore 16mm", "gripper_base_body", 4, 11, "End-Effector Gripper", "Actuators",
        "Stainless Steel SUS304 / Aluminum", "SMC Corp", "CYL-16-L", "Double acting pneumatic cylinder bore 16mm stroke 15mm", "Pressure 0.7 MPa", 0.08)
    add("gripper_cylinder_right", "Right Pneumatic Actuation Cylinder Bore 16mm", "gripper_base_body", 4, 11, "End-Effector Gripper", "Actuators",
        "Stainless Steel SUS304 / Aluminum", "SMC Corp", "CYL-16-R", "Double acting pneumatic cylinder bore 16mm stroke 15mm", "Pressure 0.7 MPa", 0.08)
    add("gripper_finger_left", "Left Modular Aluminum Finger Bracket", "gripper_base_body", 4, 11, "End-Effector Gripper", "Gripper Fingers",
        "High Tensile Anodized Aluminum 6061-T6", "Schunk", "FNG-BRK-L", "CNC machined custom finger mounting bracket", "Ra 0.8 um", 0.065)
    add("gripper_finger_right", "Right Modular Aluminum Finger Bracket", "gripper_base_body", 4, 11, "End-Effector Gripper", "Gripper Fingers",
        "High Tensile Anodized Aluminum 6061-T6", "Schunk", "FNG-BRK-R", "CNC machined custom finger mounting bracket", "Ra 0.8 um", 0.065)
    add("gripper_pad_left", "Left High-Friction Molded Silicone Pad", "gripper_finger_left", 5, 11, "End-Effector Gripper", "Contact Elements",
        "Molded Silicone Rubber 60 Shore A", "Schunk / SMC", "PAD-SIL-L", "High coefficient of friction oil-resistant contact pad", "Temp 180C", 0.012)
    add("gripper_pad_right", "Right High-Friction Molded Silicone Pad", "gripper_finger_right", 5, 11, "End-Effector Gripper", "Contact Elements",
        "Molded Silicone Rubber 60 Shore A", "Schunk / SMC", "PAD-SIL-R", "High coefficient of friction oil-resistant contact pad", "Temp 180C", 0.012)

    gripper_part_names = [
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
    ]
    for idx, gname in enumerate(gripper_part_names, 1):
        add(f"gripper_part_{idx}", gname, "sec_11", 4, 11, "End-Effector Gripper", "Hardware & Sensors",
            "SUS304 / Brass / SMC Polyurethane", "SMC / Schunk / Bossard", f"GP8-GRP-PART-{idx:03d}", f"Gripper component: {gname}", "Precision", 0.01)

    for idx in range(len(gripper_part_names) + 1, 122):
        add(f"gripper_part_{idx}", f"Gripper Assembly Torx Screw M3x8 #{idx}", "sec_11", 4, 11, "End-Effector Gripper", "Fasteners",
            "Stainless Steel SUS304", "Bossard", f"BN13577-M3x8-G{idx}", f"Gripper fastener #{idx}", "Class 6g", 0.003)

    # Section 12: Workcell Pedestal, Mounting Table & Safety Enclosure Assembly (130 Parts)
    add("sec_12", "12. Workcell Pedestal, Table & Safety Enclosure", "root", 2, 12, "Workcell Environment", "Pedestal & Guarding",
        "Structural Steel S235JR / Aluminum Extrusion 80x80 / Polycarbonate", "Item / Bosch Rexroth", "GP8-WORKCELL-PED", "Heavy-duty 1000mm steel pedestal, safety enclosure & light curtain", "ISO 14120", 85.0)
    add("pedestal_main_column", "Heavy-Duty Steel Pedestal Main Column 1000mm", "sec_12", 3, 12, "Workcell Environment", "Pedestal Frame",
        "Structural Steel Tube S235JR (200x200x8mm)", "Item Industrietechnik", "PED-COL-1000", "Welded heavy-duty steel robot mounting riser column", "Flatness 0.05mm", 48.0)
    add("pedestal_top_plate", "Pedestal Precision CNC Machined Mounting Top Plate", "pedestal_main_column", 4, 12, "Workcell Environment", "Pedestal Frame",
        "Structural Steel Plate S355JR (25mm Thickness)", "Item Industrietechnik", "PED-TOP-25", "Ground steel top plate with Yaskawa GP8 base bolt pattern", "Ra 1.6 um", 18.5)
    add("pedestal_base_flange", "Pedestal Heavy-Duty Floor Mounting Base Flange", "pedestal_main_column", 4, 12, "Workcell Environment", "Pedestal Frame",
        "Structural Steel Plate S355JR (30mm Thickness)", "Item Industrietechnik", "PED-BOT-30", "Floor anchor plate with 4x M16 anchor holes", "ISO 2768-mK", 22.0)

    workcell_part_names = [
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
    ]
    for idx, wname in enumerate(workcell_part_names, 1):
        add(f"workcell_part_{idx}", wname, "sec_12", 4, 12, "Workcell Environment", "Guarding & Hardware",
            "Steel Grade 10.9 / Aluminum / Polycarbonate", "Bosch Rexroth / Omron / Keyence", f"GP8-WC-PART-{idx:03d}", f"Workcell component: {wname}", "ISO 14120", 0.25)

    for idx in range(len(workcell_part_names) + 1, 127):
        add(f"workcell_part_{idx}", f"Workcell Extrusion T-Nut & Bolt Assembly M8x20 #{idx}", "sec_12", 4, 12, "Workcell Environment", "Fasteners",
            "Galvanized Alloy Steel 8.8", "Item Industrietechnik", f"TNUT-M8x20-{idx}", f"Workcell frame fastener #{idx}", "Class 6g", 0.02)

    return components

