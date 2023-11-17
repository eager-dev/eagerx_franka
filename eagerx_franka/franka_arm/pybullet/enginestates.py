from dataclasses import dataclass
import numpy as np
import eagerx
from typing import Any, List, Dict
from eagerx.core.specs import EngineStateSpec
import pybullet


class FrankaGripper(eagerx.EngineState):
    @classmethod
    def make(cls, joints, constant, scale, fixed=False) -> EngineStateSpec:
        spec = cls.get_specification()
        spec.config.update(joints=joints, constant=constant, scale=scale, fixed=fixed)
        return spec

    def initialize(self, spec: EngineStateSpec, simulator: Any):
        self.joints = spec.config.joints
        self.constant = spec.config.constant
        self.scale = spec.config.scale
        self.fixed = spec.config.fixed
        self.robot = simulator["object"]
        self._p = simulator["client"]
        self.physics_client_id = self._p._client

        self.bodyUniqueId = []
        self.jointIndices = []
        for _idx, pb_name in enumerate(spec.config.joints):
            bodyid, jointindex = self.robot.jdict[pb_name].get_bodyid_jointindex()
            self.bodyUniqueId.append(bodyid), self.jointIndices.append(jointindex)
        self.gripper_cb = self._gripper_reset(
            self._p, self.bodyUniqueId[0], self.jointIndices, self.constant, self.scale, self.fixed
        )

    def reset(self, state: Any):
        self.gripper_cb(state)

    @staticmethod
    def _gripper_reset(p, bodyUniqueId, jointIndices, constant, scale, fixed):
        def cb(state):
            # Mirror & scale gripper position
            pos = scale * state[0] + constant
            gripper_pos = [pos, pos]

            # Only 1-dof joints are supported here.
            # https://github.com/bulletphysics/bullet3/issues/2803
            velocities = []
            states = p.getJointStates(bodyUniqueId=bodyUniqueId, jointIndices=jointIndices, physicsClientId=p._client)
            for _i, (_, vel, _, _) in enumerate(states):
                velocities.append([vel])
            p.resetJointStatesMultiDof(
                targetValues=[[pos] for pos in gripper_pos],
                targetVelocities=velocities,
                bodyUniqueId=bodyUniqueId,
                jointIndices=jointIndices,
                physicsClientId=p._client,
            )
            # If we are not performing control with the gripper, fix the position.
            if fixed:
                p.setJointMotorControlArray(
                    bodyUniqueId=bodyUniqueId,
                    jointIndices=jointIndices,
                    controlMode=pybullet.VELOCITY_CONTROL,
                    forces=len(jointIndices) * [10**9],
                    physicsClientId=p._client,
                )

        return cb


class PoseState(eagerx.EngineState):
    @classmethod
    def make(
        cls,
        joints: List[int],
        upper: List[float],
        lower: List[float],
        ee_link: str,
        rest_poses: List[float],
    ) -> EngineStateSpec:
        spec = cls.get_specification()
        spec.config.joints = joints
        spec.config.upper = upper
        spec.config.lower = lower
        spec.config.ee_link = ee_link
        spec.config.rest_poses = rest_poses
        return spec

    def initialize(self, spec: EngineStateSpec, simulator: Dict):
        """Initializes the engine state according to the spec."""
        self.joints = spec.config.joints
        self.upper = np.array(spec.config.upper, dtype="float")
        self.lower = np.array(spec.config.lower, dtype="float")
        self.ee_link = spec.config.ee_link
        self.rest_poses = np.array(spec.config.rest_poses, dtype="float")

        self.robot = simulator["object"]
        self._p = simulator["client"]._client
        self.physics_client_id = self._p

        # Setup physics server for ik solver
        self.pb = pybullet

        self.arm = self.robot.robot_objectid[0]
        self.indexed_joints, self.index_ee_link = index_joints_and_ee_link(
            self.pb, self._p, self.arm, self.joints, self.ee_link
        )

        # Reset to sleep position
        for i, joint in enumerate(self.indexed_joints):
            self.pb.resetJointState(self.arm, joint.joint_uid, self.rest_poses[i], physicsClientId=self._p)

    def reset(self, state):
        ee_pose_goal = state
        # self.robot.parts[self.ee_link].get_pose()
        # ee_pose_goal = np.array([.40, 0.,  0.50,  1, 0.,  0.,  0])
        ee_pos_goal = ee_pose_goal[:3]
        ee_orn_goal = ee_pose_goal[3:]

        # Get inverse kinematics solution
        goal = self.pb.calculateInverseKinematics(
            bodyUniqueId=self.arm,
            endEffectorLinkIndex=self.index_ee_link,
            targetPosition=ee_pos_goal,
            targetOrientation=ee_orn_goal,
            lowerLimits=self.lower,
            upperLimits=self.upper,
            jointRanges=self.upper - self.lower,
            restPoses=self.rest_poses,
            # currentPositions=current.tolist(),
            maxNumIterations=100,
            residualThreshold=1e-5,
            physicsClientId=self._p,
        )

        # Set joints to goal
        for i, joint in enumerate(self.indexed_joints):
            self.pb.resetJointState(self.arm, joint.joint_uid, goal[i], physicsClientId=self._p)


@dataclass
class IndexedJointObject:
    """Index of a robot joint and its name."""

    joint_name: str
    joint_uid: int


def index_joints_and_ee_link(pb, physics_uid, robot_id, joints, ee_link):
    """Map a list of joint names to indexed joints.
    In other words, map named joints to the index used by
    PyBullet to facilitate setting the configuration.
    Parameters:
      physics_uid: Index of the PyBullet physics server to use.
      joints: list with joint name keys
      ee_link: end effector link name
    Returns: a list of IndexedJointObject
    """
    indexed_joints = []
    indexed_ee_link = None
    n = pb.getNumJoints(robot_id, physics_uid)
    for joint_name in joints:
        for i in range(n):
            info = pb.getJointInfo(robot_id, i, physics_uid)
            if info[12].decode("utf8") == ee_link:
                indexed_ee_link = i
            if joint_name == info[1].decode("utf-8"):
                indexed_joints.append(IndexedJointObject(joint_name, i))
                continue
    assert len(joints) == len(indexed_joints), "Not all joints were found in the provided urdf."
    assert indexed_ee_link is not None, "End effector link not found in the provided urdf."
    return indexed_joints, indexed_ee_link
