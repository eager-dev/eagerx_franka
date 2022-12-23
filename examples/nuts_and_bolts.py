import eagerx
import eagerx_franka
from eagerx_utility.solid.solid import Solid
from eagerx_franka.franka_arm.franka_arm import FrankaArm

import os
from typing import Dict, Tuple


cam_intrinsics = {
    "image_width": 640,
    "image_height": 480,
    "camera_name": "logitech_c170",
    "camera_matrix": {
        "rows": 3,
        "cols": 3,
        "data": [744.854391488828, 0, 327.2505862760107, 0, 742.523670731623, 207.0294448122543, 0, 0, 1],
    },
    "distortion_model": "plumb_bob",
    "distortion_coefficients": {
        "rows": 1,
        "cols": 5,
        "data": [0.1164823030284325, -0.7022013182646298, -0.01409335811907957, 0.001216661775149573, 0],
    },
    "rectification_matrix": {"rows": 3, "cols": 3, "data": [1, 0, 0, 0, 1, 0, 0, 0, 1]},
    "projection_matrix": {
        "rows": 3,
        "cols": 4,
        "data": [742.5888671875, 0, 327.7492634819646, 0, 0, 741.0902709960938, 202.927556117038, 0, 0, 0, 1, 0],
    },
}



class NutsBoltsEnv(eagerx.BaseEnv):
    def __init__(self, name: str, rate: float, graph: eagerx.Graph, engine: eagerx.specs.EngineSpec, backend: eagerx.specs.BackendSpec):
        super().__init__(name, rate=rate, graph=graph, engine=engine, backend=backend)

    def step(self, action: Dict) -> Tuple[Dict, float, bool, Dict]:
        obs = self._step(action)
        reward = 0
        done = False
        info = {}
        return obs, reward, done, info

    def reset(self) -> Dict:
        states = self.state_space.sample()
        obs = self._reset(states)
        return obs


if __name__ == "__main__":
    eagerx.set_log_level(eagerx.WARN)

    urdf_path = os.path.dirname(eagerx_franka.__file__) + "/solid/assets/"

    sync = True
    rate = 10

    graph = eagerx.Graph.create()

    bolt = Solid.make("bolt", urdf=urdf_path + "bolt.urdf", cam_intrinsics=cam_intrinsics, rate=rate)
    nut = Solid.make("nut", urdf=urdf_path + "nut.urdf", cam_intrinsics=cam_intrinsics, rate=rate)
    arm = FrankaArm.make("franka_arm", robot_type="panda", arm_name="panda", sensors=["position", "velocity", "ee_pos", "ee_orn"])
    bolt.states.position.space.update(low=[-1, -1, 0.1])
    nut.states.position.space.update(low=[-1, -1, 0.1])

    graph.add([arm, nut, bolt])
    graph.connect(action="joint_pos", target=arm.actuators.pos_control)
    graph.connect(action="gripper_pos", target=arm.actuators.gripper_control)
    graph.connect(source=arm.sensors.position, observation="joint_pos")
    graph.connect(source=arm.sensors.velocity, observation="joint_vel")
    graph.connect(source=arm.sensors.ee_pos, observation="ee_pos")
    graph.connect(source=arm.sensors.ee_orn, observation="ee_orn")

    graph.gui()

    from eagerx_pybullet.engine import PybulletEngine

    engine = PybulletEngine.make(rate=rate, gui=True, egl=True, sync=True, real_time_factor=0.0)

    from eagerx.backends.single_process import SingleProcess

    backend = SingleProcess.make()

    env = NutsBoltsEnv("env", rate=rate, graph=graph, engine=engine, backend=backend)


    # Evaluate
    eps = 0
    for i in range(10):
        step = 0
        _, done, frames = env.reset(), False, []
        while not done:
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
