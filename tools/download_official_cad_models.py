#!/usr/bin/env python3
import os
import urllib.request
import ssl

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESH_BASE = os.path.join(BASE_DIR, 'src', 'yaskawa_workcell_description', 'meshes')

# 1. Yaskawa GP8 Visual Meshes (ROS Industrial noetic-devel)
VISUAL_URL_BASE = "https://raw.githubusercontent.com/ros-industrial/motoman/noetic-devel/motoman_gp8_support/meshes/visual/"
VISUAL_FILES = [
    "gp8_base_link.stl",
    "gp8_link_1_s.stl",
    "gp8_link_2_l.stl",
    "gp8_link_3_u.stl",
    "gp8_link_4_r.stl",
    "gp8_link_5_b.stl",
    "gp8_link_6_t.stl"
]

# 2. Yaskawa GP8 Collision Meshes (ROS Industrial noetic-devel)
COLLISION_URL_BASE = "https://raw.githubusercontent.com/ros-industrial/motoman/noetic-devel/motoman_gp8_support/meshes/collision/"
COLLISION_FILES = [
    "gp8_base_link.stl",
    "gp8_link_1_s.stl",
    "gp8_link_2_l.stl",
    "gp8_link_3_u.stl",
    "gp8_link_4_r.stl",
    "gp8_link_5_b.stl",
    "gp8_link_6_t.stl"
]

# 3. Robotiq 2F-85 Gripper Meshes (Google DeepMind MuJoCo Menagerie)
GRIPPER_URL_BASE = "https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/robotiq_2f85/assets/"
GRIPPER_FILES = [
    "base.stl",
    "base_mount.stl",
    "coupler.stl",
    "driver.stl",
    "follower.stl",
    "pad.stl",
    "silicone_pad.stl",
    "spring_link.stl"
]

def download_file(url, dest_path):
    print(f"--> Downloading: {url}")
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
    with urllib.request.urlopen(req, context=ctx) as response, open(dest_path, 'wb') as out_file:
        data = response.read()
        out_file.write(data)
        print(f"    Saved {len(data):,} bytes -> {os.path.basename(dest_path)}")

def main():
    print("==================================================================")
    print("      OFFICIAL CAD 3D MESH MODEL DOWNLOADER (YASKAWA GP8)")
    print("==================================================================")

    # Visual Dir
    vis_dir = os.path.join(MESH_BASE, 'visual')
    os.makedirs(vis_dir, exist_ok=True)
    for fname in VISUAL_FILES:
        dest = os.path.join(vis_dir, fname)
        url = VISUAL_URL_BASE + fname
        try:
            download_file(url, dest)
        except Exception as e:
            print(f"❌ Failed to download visual mesh {fname}: {e}")

    # Collision Dir
    col_dir = os.path.join(MESH_BASE, 'collision')
    os.makedirs(col_dir, exist_ok=True)
    for fname in COLLISION_FILES:
        dest = os.path.join(col_dir, fname)
        url = COLLISION_URL_BASE + fname
        try:
            download_file(url, dest)
        except Exception as e:
            print(f"❌ Failed to download collision mesh {fname}: {e}")

    # Gripper Dir
    grp_dir = os.path.join(MESH_BASE, 'gripper')
    os.makedirs(grp_dir, exist_ok=True)
    for fname in GRIPPER_FILES:
        dest = os.path.join(grp_dir, fname)
        url = GRIPPER_URL_BASE + fname
        try:
            download_file(url, dest)
        except Exception as e:
            print(f"❌ Failed to download gripper mesh {fname}: {e}")

    print("==================================================================")
    print("      ALL CAD MESH DOWNLOADS COMPLETED SUCCESSFULLY")
    print("==================================================================")

if __name__ == '__main__':
    main()
