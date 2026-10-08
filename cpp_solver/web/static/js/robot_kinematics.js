// Yaskawa GP8 6-DOF Joint Kinematics Engine & Slider Controller
const currentJoints = { j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0 };
let isKinematicAnimating = false;

function initKinematics() {
    const jointIds = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'];
    jointIds.forEach(id => {
        const slider = document.getElementById(`${id}-slider`);
        if (slider) {
            slider.addEventListener('input', (e) => {
                currentJoints[id] = parseFloat(e.target.value);
                applyJointAngles();
            });
        }
    });
    applyJointAngles();
}

function applyJointAngles() {
    const deg2rad = Math.PI / 180;

    // Apply 6-Axis Rotations to Three.js Joint Groups
    if (j1Group) j1Group.rotation.z = currentJoints.j1 * deg2rad;
    if (j2Group) j2Group.rotation.y = currentJoints.j2 * deg2rad;
    if (j3Group) j3Group.rotation.y = currentJoints.j3 * deg2rad;
    if (j4Group) j4Group.rotation.x = currentJoints.j4 * deg2rad;
    if (j5Group) j5Group.rotation.y = currentJoints.j5 * deg2rad;
    if (j6Group) j6Group.rotation.x = currentJoints.j6 * deg2rad;

    // Update Slider Value Readouts
    const jointIds = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'];
    jointIds.forEach(id => {
        const valElem = document.getElementById(`${id}-val`);
        const slider = document.getElementById(`${id}-slider`);
        if (valElem) {
            valElem.textContent = `${currentJoints[id] >= 0 ? '+' : ''}${currentJoints[id].toFixed(1)}°`;
        }
        if (slider && parseFloat(slider.value) !== currentJoints[id]) {
            slider.value = currentJoints[id];
        }
    });
}

function resetHomePose() {
    isKinematicAnimating = false;
    currentJoints.j1 = 0;
    currentJoints.j2 = 0;
    currentJoints.j3 = 0;
    currentJoints.j4 = 0;
    currentJoints.j5 = 0;
    currentJoints.j6 = 0;
    applyJointAngles();
    const status = document.getElementById('console-status');
    if (status) status.textContent = "Status: Robot Kinematic Pose Reset to HOME (0°, 0°, 0°, 0°, 0°, 0°)";
}

function animatePoseSequence() {
    if (isKinematicAnimating) return;
    isKinematicAnimating = true;

    const keyframes = [
        { j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0, duration: 800 },
        { j1: 45, j2: 30, j3: -20, j4: 60, j5: -45, j6: 90, duration: 1200 },
        { j1: -60, j2: -20, j3: 45, j4: -90, j5: 60, j6: -180, duration: 1500 },
        { j1: 90, j2: 45, j3: -30, j4: 120, j5: -75, j6: 270, duration: 1500 },
        { j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0, duration: 1000 }
    ];

    let frameIdx = 0;
    function stepToKeyframe() {
        if (!isKinematicAnimating || frameIdx >= keyframes.length) {
            isKinematicAnimating = false;
            return;
        }

        const startPose = { ...currentJoints };
        const targetPose = keyframes[frameIdx];
        const startTime = performance.now();

        function interpolate() {
            if (!isKinematicAnimating) return;
            const elapsed = performance.now() - startTime;
            const progress = Math.min(elapsed / targetPose.duration, 1.0);
            const ease = 0.5 - Math.cos(progress * Math.PI) / 2;

            currentJoints.j1 = startPose.j1 + (targetPose.j1 - startPose.j1) * ease;
            currentJoints.j2 = startPose.j2 + (targetPose.j2 - startPose.j2) * ease;
            currentJoints.j3 = startPose.j3 + (targetPose.j3 - startPose.j3) * ease;
            currentJoints.j4 = startPose.j4 + (targetPose.j4 - startPose.j4) * ease;
            currentJoints.j5 = startPose.j5 + (targetPose.j5 - startPose.j5) * ease;
            currentJoints.j6 = startPose.j6 + (targetPose.j6 - startPose.j6) * ease;

            applyJointAngles();

            if (progress < 1.0) {
                requestAnimationFrame(interpolate);
            } else {
                frameIdx++;
                setTimeout(stepToKeyframe, 200);
            }
        }
        interpolate();
    }

    const status = document.getElementById('console-status');
    if (status) status.textContent = "Status: Executing 6-DOF Industrial Motion Trajectory Animation...";
    stepToKeyframe();
}
