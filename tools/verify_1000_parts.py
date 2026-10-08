#!/usr/bin/env python3
import sys
import os
import json
import subprocess

def verify_1000_components():
    print("==================================================================")
    print("      YASKAWA GP8 & YRC1000 1000+ COMPONENT VERIFICATION        ")
    print("==================================================================")
    
    # 1. Verify C++ Unit Test Engine & Reflection
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.abspath(os.path.join(script_dir, '..'))
    web_dir = os.path.join(root_dir, 'cpp_solver', 'web')
    if web_dir not in sys.path:
        sys.path.insert(0, web_dir)
    test_bin = os.path.join(root_dir, 'cpp_solver', 'build', 'test_cpp')
    
    if not os.path.exists(test_bin):
        print(f"--> Building C++ solver binaries first...")
        subprocess.run(['./robot', 'build', 'release'], cwd=root_dir, check=True)

    print("--> Running C++26 Reflection & Unit Test Suite...")
    res = subprocess.run([test_bin], capture_output=True, text=True, timeout=15)
    if "ALL UNIT TESTS PASSED SUCCESSFULLY" not in res.stdout:
        print("❌ ERROR: C++ Unit test suite failed!")
        print(res.stdout)
        print(res.stderr)
        sys.exit(1)
    
    print("✅ PASS: C++26 Reflection & Physical Tree Engine Unit Tests")

    # 2. Verify Python Web Tree Database
    web_dir = os.path.abspath(os.path.join(root_dir, 'cpp_solver', 'web'))
    if web_dir not in sys.path:
        sys.path.insert(0, web_dir)
    try:
        from tree_database import get_robot_physical_tree
        components = get_robot_physical_tree()
        count = len(components)
        print(f"--> Python Tree Database Total Component Count: {count}")
        if count < 1000:
            print(f"❌ ERROR: Component count is {count} (expected >= 1000)")
            sys.exit(1)
        
        # Verify schema for all items
        required_fields = ["id", "name", "parent_id", "level", "system_category_id", "system_name", "subsystem", "material", "manufacturer", "part_number", "specs_summary", "tolerances", "mass_kg"]
        for idx, comp in enumerate(components):
            for field in required_fields:
                if field not in comp:
                    print(f"❌ ERROR: Component #{idx} ({comp.get('id', 'N/A')}) missing field '{field}'")
                    sys.exit(1)
                if field != "parent_id" and field != "level" and field != "system_category_id" and field != "mass_kg" and not comp[field]:
                    print(f"❌ ERROR: Component #{idx} ({comp['id']}) field '{field}' is empty!")
                    sys.exit(1)

        print("✅ PASS: 1000+ Component Database Schema & Non-Empty Metadata Verification")
    except Exception as e:
        print(f"❌ ERROR: Failed to load tree database: {e}")
        sys.exit(1)

    print("==================================================================")
    print("      ALL 1000+ PHYSICAL COMPONENT VERIFICATIONS PASSED 100%     ")
    print("==================================================================")
    return 0

if __name__ == '__main__':
    sys.exit(verify_1000_components())
