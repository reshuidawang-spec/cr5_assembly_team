"""Quick R1 IK — can R1 reach new targets with native R1T gripper?"""
import math
from sim_bridge.coppelia_client import SimBridge
from sim_bridge.scene_objects import SCENE_ROOT, ROBOT_TIPS

bridge = SimBridge()
bridge.connect(port=23000)
sim = bridge.sim

client = getattr(bridge, "_client", None)
if client is None:
    print("ERROR: no ZMQ client"); exit(1)
sim_ik = client.require("simIK")

# --- Discover R1 ---
r1 = sim.getObject("/R1")
joint_names = [f"joint{i}" for i in range(1, 7)]
joints = []
for h in sim.getObjectsInTree(r1, sim.object_joint_type, 0):
    a = sim.getObjectAlias(h)
    if a in joint_names:
        joints.append((a, h))
joints.sort(key=lambda x: x[0])
joint_handles = [h for _, h in joints]

# --- Native tip ---
tip_alias = ROBOT_TIPS["R1"]
tips = [h for h in sim.getObjectsInTree(r1, sim.handle_all, 0) if sim.getObjectAlias(h) == tip_alias]
tip = tips[0]
base = r1  # robot root as base
seed = [0.0] * 6

print(f"R1 tip={tip_alias}  pos={[f'{v:.3f}' for v in sim.getObjectPosition(tip, -1)]}")

# --- Test targets ---
targets = ["R1_BOX_PICK_APP", "R1_BOX_PICK_TCP", "R1_BOX_PLACE_APP", "R1_BOX_PLACE_TCP"]

for name in targets:
    target = sim.getObject(f"{SCENE_ROOT}/Targets/R1_Targets/{name}")
    target_pos = sim.getObjectPosition(target, -1)

    env = sim_ik.createEnvironment()
    group = sim_ik.createGroup(env)
    try:
        element, scene_to_ik, _ = sim_ik.addElementFromScene(
            env, group, base, tip, target, sim_ik.constraint_pose
        )
        for jh, s in zip(joint_handles, seed):
            sim_ik.setJointPosition(env, scene_to_ik[jh], s)
        sim_ik.setGroupCalculation(env, group, sim_ik.method_damped_least_squares, 0.1, 200)
        sim_ik.setElementPrecision(env, group, element, [0.001, math.radians(1.0)])

        result, flags, precision = sim_ik.handleGroup(env, group)
        success = result == sim_ik.result_success

        final_q = [sim_ik.getJointPosition(env, scene_to_ik[jh]) for jh in joint_handles]
        degs = [f"{math.degrees(v):.1f}" for v in final_q]
        status = "OK" if success else "FAILED"
        err_mm = precision[0]*1000 if isinstance(precision, list) else precision*1000
        print(f"{name}: {status}  err={err_mm:.1f}mm  q=[{', '.join(degs)}]")
    finally:
        sim_ik.eraseEnvironment(env)
    seed = final_q if success else [0.0]*6

bridge.disconnect()
print("Done.")
