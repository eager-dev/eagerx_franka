import eagerx
from typing import Any, List
from eagerx import Space
from eagerx.core.specs import NodeSpec
import eagerx.core.register as register
from eagerx.utils.utils import Msg
import numpy as np
from eagerx_franka.panda_ros.panda import Panda as Client


class FrankaSensor(eagerx.EngineNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        arm_name: str,
        robot_type: str,
        joints: List[str],
        color: str = "cyan",
        mode: str = "position",
    ) -> NodeSpec:
        # TODO: Use arm_name and robot_type to get appropriate robot.
        """Make the parameter specification for an Franka joint sensor.

        :param name: Node name
        :param rate: Rate of the node [Hz].
        :param arm_name: Name of the arm.
        :param robot_type: Manipulator type.
        :param joints: Joint names.
        :param color: Color of logged messages.
        :param mode: Types supported measurements:
                     - position: Joint positions [rad]. Should be limited by the joint limits.
                     - velocity: Angular velocity [rad/s]. Should be limited by the velocity limits.
                     - ee_position: End-effector position [m].
                     - ee_orientation: End-effector orientation [quaternion].
                     - ee_pose: End-effector pose [ee_position, ee_quaternion].
        :return:
        """
        spec = cls.get_specification()

        # Modify default node params
        spec.config.update(name=name, rate=rate, process=eagerx.ENGINE, color=color)
        spec.config.update(arm_name=arm_name, robot_type=robot_type)
        spec.config.inputs = ["tick"]
        spec.config.outputs = ["obs"]

        # Set parameters, defined by the signature of cls.initialize(...)
        spec.config.mode = mode
        spec.config.joints = joints

        # Update space definition based on mode.
        if mode == "ee_position":
            spec.outputs.obs.space.update(shape=[3])
        elif mode == "ee_orientation":
            spec.outputs.obs.space.update(low=-1, high=1, shape=[4])
        elif mode == "ee_pose":
            spec.outputs.obs.space.update(shape=[7])
        elif mode == "gripper_position":
            spec.outputs.obs.space.update(shape=[1])
        else:
            spec.outputs.obs.space.update(shape=[len(joints)])

        return spec

    def initialize(self, spec: NodeSpec, simulator: Any):
        self.mode = spec.config.mode
        if self.mode not in ["position", "velocity", "ee_position", "ee_orientation", "ee_pose", "gripper_position"]:
            raise NotImplementedError(f"This mode is not implemented: {spec.config.mode}")

        # Get arm client
        if "client" not in simulator:
            simulator["client"] = Client()
        self.arm = simulator["client"]

        # Remap joint measurements & commands according to ordering in spec.config.joints.
        # self.arm.set_joint_remapping(spec.config.joints)

    @register.states()
    def reset(self):
        pass

    @register.inputs(tick=Space(shape=(), dtype="int64"))
    @register.outputs(obs=Space(dtype="float32"))
    def callback(self, t_n: float, tick: Msg):
        # Select based on mode of node.
        if self.mode == "position":
            position = self.arm.curr_joint
            obs = np.array(position, dtype="float32")
        elif self.mode == "velocity":
            velocity = self.arm.curr_joint_vel
            obs = np.array(velocity, dtype="float32")
        elif self.mode == "ee_position":
            ee_position = self.arm.curr_pos
            obs = np.array(ee_position, dtype="float32")
        elif self.mode == "ee_orientation":
            ori = self.arm.curr_ori
            # w should be the last element instead of the first element
            ori = np.roll(ori, -1)
            obs = np.array(ori, dtype="float32")
        elif self.mode == "ee_pose":
            pos = self.arm.curr_pos
            ori = self.arm.curr_ori
            # w should be the last element instead of the first element
            ori = np.roll(ori, -1)
            obs = np.concatenate([pos, ori])
        elif self.mode == "gripper_position":
            gripper_pos = self.arm.gripper_width
            obs = np.array([gripper_pos], dtype="float32")
        else:
            raise NotImplementedError(f"This mode is not implemented: {self.mode}")
        return dict(obs=obs)

    def close(self):
        pass


class FrankaGripper(eagerx.EngineNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        arm_name: str,
        robot_type: str,
        color: str = "green",
    ) -> NodeSpec:
        """FrankaGripper spec

        :param name: Name of the node.
        :param rate: Rate of the node.
        :param arm_name: Name of the arm.
        :param robot_type: Type of the robot.
        :param color: Color of the node.
        """
        spec = cls.get_specification()

        # Modify default node params
        spec.config.update(name=name, rate=rate, process=eagerx.ENGINE, color=color)
        spec.config.update(arm_name=arm_name, robot_type=robot_type)
        spec.config.inputs = ["tick", "action"]
        spec.config.outputs = ["action_applied"]
        return spec

    def initialize(self, spec: NodeSpec, simulator: Any):
        # Get arm client
        if "client" not in simulator:
            simulator["client"] = Client()
        self.arm = simulator["client"]

    @register.states()
    def reset(self):
        pass

    @register.inputs(tick=Space(shape=(), dtype="int64"), action=Space(low=[0], high=[1], dtype="float32"))
    @register.outputs(action_applied=Space(low=[0], high=[1], dtype="float32"))
    def callback(self, t_n: float, tick: Msg, action: Msg):
        self.arm.grasp_gripper(action.msgs[-1][0] * 0.08)
        # Send action that has been applied.
        return dict(action_applied=action.msgs[-1])

    def shutdown(self):
        self.arm.stop_gripper()


class DummySensor(eagerx.EngineNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        color: str = "green",
    ) -> NodeSpec:
        """DummySensor spec"""
        spec = cls.get_specification()

        # Modify default node params
        spec.config.update(name=name, rate=rate, process=eagerx.ENGINE, color=color)
        spec.config.inputs = ["tick"]
        spec.config.outputs = ["obs"]
        return spec

    def initialize(self, spec: NodeSpec, simulator: Any):
        pass

    @register.states()
    def reset(self):
        pass

    @register.inputs(tick=Space(shape=(), dtype="int64"))
    @register.outputs(obs=Space(dtype="float32"))
    def callback(self, t_n: float, tick: Msg):
        return dict(obs=np.asarray([], dtype="float32"))

    def shutdown(self):
        pass


class TaskSpaceControl(eagerx.EngineNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        arm_name: str,
        robot_type: str,
        color: str = "green",
    ) -> NodeSpec:
        """FrankaTaskSpaceControl spec

        :param name: Name of the node.
        :param rate: Rate of the node.
        :param arm_name: Name of the arm.
        :param robot_type: Type of the robot.
        :param color: Color of the node.
        """
        spec = cls.get_specification()

        # Modify default node params
        spec.config.update(name=name, rate=rate, process=eagerx.ENGINE, color=color)
        spec.config.update(arm_name=arm_name, robot_type=robot_type)
        spec.config.inputs = ["tick", "ee_pose"]
        spec.config.outputs = ["goal"]
        return spec

    def initialize(self, spec: NodeSpec, simulator: Any):
        # Get arm client
        if "client" not in simulator:
            simulator["client"] = Client()
        self.arm = simulator["client"]

    @register.states()
    def reset(self):
        pass

    @register.inputs(
        tick=Space(shape=(), dtype="int64"),
        ee_pose=Space(low=[-2, -2, 0, -1, -1, -1, -1], high=[2, 2, 2, 1, 1, 1, 1], dtype="float32"),
    )
    @register.outputs(goal=Space(low=[-2, -2, 0, -1, -1, -1, -1], high=[2, 2, 2, 1, 1, 1, 1], dtype="float32"))
    def callback(self, t_n: float, tick: Msg, ee_pose: Msg):
        goal_pose = ee_pose.msgs[-1]
        goal_pose[3:] = np.roll(goal_pose[3:], 1)
        self.arm.go_to_pose_array(np.squeeze([goal_pose]))
        return dict(goal=ee_pose.msgs[-1])

    def shutdown(self):
        curr_pos = self.arm.curr_pos
        curr_ori = self.arm.curr_ori
        self.arm.go_to_pose_array(np.concatenate([curr_pos, curr_ori]))
