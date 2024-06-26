import rospy
import math
import numpy as np
from threading import Thread
import quaternion  # pip install numpy-quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float32MultiArray
import dynamic_reconfigure.client
from eagerx_franka.panda_ros.franka_gripper.msg import GraspActionGoal, HomingActionGoal, StopActionGoal, MoveActionGoal
from eagerx_franka.panda_ros.pose_transform_functions import (
    array_quat_2_pose,
    list_2_quaternion,
    position_2_array,
    orientation_2_quaternion,
    array_array_2_pose,
)


def inner(q1, q2):
    result = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w
    return result


def change_sign(q):
    q.w = -q.w
    q.x = -q.x
    q.y = -q.y
    q.z = -q.z
    return q


def quaternion_divide(q1, q2):
    """Divide quaternions q1/q2 = q1 * q2.inverse"""
    if inner(q1, q2) < 0:
        q2 = change_sign(q2)
    q2norm = q2.w**2 + q2.x**2 + q2.y**2 + q2.z**2
    a = (q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z) / q2norm
    b = (-q1.w * q2.x + q1.x * q2.w - q1.y * q2.z + q1.z * q2.y) / q2norm
    c = (-q1.w * q2.y + q1.x * q2.z + q1.y * q2.w - q1.z * q2.x) / q2norm
    d = (-q1.w * q2.z - q1.x * q2.y + q1.y * q2.x + q1.z * q2.w) / q2norm
    qout = np.quaternion(0, 0, 0, 0)
    qout.w = a
    qout.x = b
    qout.y = c
    qout.z = d
    return qout


def quaternion_product(q1, q2):
    """Multiply quaternions q1*q2"""
    a = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
    b = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
    c = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
    d = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    qout = np.quaternion(0, 0, 0, 0)
    qout.w = a
    qout.x = b
    qout.y = c
    qout.z = d
    return qout


class Panda:
    def __init__(self):
        super(Panda, self).__init__()

        self.K_pos = 1000
        self.K_ori = 30
        self.K_ns = 10  ##### not being used
        self.curr_pos = None
        self.curr_ori = None
        self.curr_pos_goal = None
        self.curr_ori_goal = None
        self.attractor_distance_threshold = 0.05
        self.max_force = 2  # [N]
        self.force_min_exit = 1  # [N]
        self._from = None
        self.goal_pose = None
        self.go_to_pose_thread = None
        self.goal_updated = False

        self.gripper_goal = None

        self.pos_sub = rospy.Subscriber("/cartesian_pose", PoseStamped, self.ee_pos_callback)

        self.force_feedback_sub = rospy.Subscriber("/force_torque_ext", WrenchStamped, self.force_feedback_callback)
        self.goal_sub = rospy.Subscriber("/equilibrium_pose", PoseStamped, self.ee_pos_goal_callback)
        self.goal_pub = rospy.Publisher("/equilibrium_pose", PoseStamped, queue_size=0)
        self.configuration_pub = rospy.Publisher("/equilibrium_configuration", Float32MultiArray, queue_size=0)
        self.grasp_pub = rospy.Publisher("/franka_gripper/grasp/goal", GraspActionGoal, queue_size=0)
        self.move_pub = rospy.Publisher("/franka_gripper/move/goal", MoveActionGoal, queue_size=0)
        self.homing_pub = rospy.Publisher("/franka_gripper/homing/goal", HomingActionGoal, queue_size=0)
        self.stop_pub = rospy.Publisher("/franka_gripper/stop/goal", StopActionGoal, queue_size=0)

        self.force_feedback = 0.0
        self.set_K = dynamic_reconfigure.client.Client("/dynamic_reconfigure_compliance_param_node", config_callback=None)
        self.joint_states_sub = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)

        self.move_command = MoveActionGoal()
        self.grasp_command = GraspActionGoal()
        self.home_command = HomingActionGoal()
        self.stop_command = StopActionGoal()
        self.gripper_width = 0
        self.move_command.goal.speed = 1
        self.grasp_command.goal.epsilon.inner = 0.3
        self.grasp_command.goal.epsilon.outer = 0.3
        self.grasp_command.goal.speed = 0.1
        self.grasp_command.goal.force = 50
        self.grasp_command.goal.width = 1
        joint_states = rospy.wait_for_message("/joint_states", JointState)
        self.joint_names = joint_states.name

        rospy.sleep(1)

    def ee_pos_goal_callback(self, goal_conf):
        self.curr_pos_goal = np.array([goal_conf.pose.position.x, goal_conf.pose.position.y, goal_conf.pose.position.z])
        self.curr_ori_goal = np.array(
            [
                goal_conf.pose.orientation.w,
                goal_conf.pose.orientation.x,
                goal_conf.pose.orientation.y,
                goal_conf.pose.orientation.z,
            ]
        )

    def ee_pos_callback(self, curr_conf):
        self.curr_pos = np.array([curr_conf.pose.position.x, curr_conf.pose.position.y, curr_conf.pose.position.z])
        self.curr_ori = np.array(
            [
                curr_conf.pose.orientation.w,
                curr_conf.pose.orientation.x,
                curr_conf.pose.orientation.y,
                curr_conf.pose.orientation.z,
            ]
        )

    def move_gripper(self, width):
        self.move_command.goal.width = width
        self.move_pub.publish(self.move_command)

    def grasp_gripper(self, width):
        if self.gripper_goal == width:
            return
        self.gripper_goal = width
        self.grasp_command.goal.width = width
        self.grasp_pub.publish(self.grasp_command)

    def home(self):
        pos_array = np.array([0.4, -0.05, 0.25])  # home position
        quat = np.quaternion(0, 1, 0, 0)  # home orientation
        goal = array_quat_2_pose(pos_array, quat)
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()

        ns_msg = [0, 0, 0, -2.4, 0, 2.4, 0]  # home joint configuration
        self.go_to_pose(goal, do_spiral_search=False)
        self.set_configuration(ns_msg)
        self.set_K.update_configuration({"nullspace_stiffness": 10})

        self.offset_compensator(10)

        rospy.sleep(rospy.Duration(secs=5))

        self.set_K.update_configuration({"nullspace_stiffness": 0})

    def home_gripper(self):
        self.homing_pub.publish(self.home_command)

    def stop_gripper(self):
        self.stop_pub.publish(self.stop_command)

    def force_feedback_callback(self, feedback):
        self.force = feedback.wrench.force
        self.force_feedback = np.linalg.norm(np.array([self.force.x, self.force.y, self.force.z]))

    def joint_states_callback(self, data):
        if self._from is not None:
            indices = self._from
        else:
            indices = range(7)
        self.curr_joint = np.array([data.position[i] for i in indices])
        self.curr_joint_vel = np.array([data.velocity[i] for i in indices])
        self.gripper_width = data.position[7] + data.position[8]

    def set_stiffness(self, k_t1, k_t2, k_t3, k_r1, k_r2, k_r3, k_ns):

        self.set_K.update_configuration({"translational_stiffness_X": k_t1})
        self.set_K.update_configuration({"translational_stiffness_Y": k_t2})
        self.set_K.update_configuration({"translational_stiffness_Z": k_t3})
        self.set_K.update_configuration({"rotational_stiffness_X": k_r1})
        self.set_K.update_configuration({"rotational_stiffness_Y": k_r2})
        self.set_K.update_configuration({"rotational_stiffness_Z": k_r3})
        self.set_K.update_configuration({"nullspace_stiffness": k_ns})

    def set_stiffness_key(self):
        self.set_stiffness(4000, 4000, 4000, 50, 50, 30, 0)

    def set_configuration(self, joint):
        joint_des = Float32MultiArray()
        joint_des.data = np.array(joint).astype(np.float32)
        self.configuration_pub.publish(joint_des)

    def go_to_pose_array(self, goal_pose, do_spiral_search=True, interp_dist=0.001, interp_dist_polar=0.001):
        if len(goal_pose) != 7:
            print("goal pose has wrong length")
            print(f"goal pose is {goal_pose}")
        goal_pose = array_array_2_pose(goal_pose[0:3], goal_pose[3:7])
        self.go_to_pose(goal_pose, do_spiral_search, interp_dist, interp_dist_polar)

    def go_to_pose(self, goal_pose, do_spiral_search=True, interp_dist=0.001, interp_dist_polar=0.001):
        if self.goal_pose is not None:
            if (
                self.goal_pose.pose.position.x == goal_pose.pose.position.x
                and self.goal_pose.pose.position.y == goal_pose.pose.position.y
                and self.goal_pose.pose.position.z == goal_pose.pose.position.z
            ):
                if (
                    self.goal_pose.pose.orientation.x == goal_pose.pose.orientation.x
                    and self.goal_pose.pose.orientation.y == goal_pose.pose.orientation.y
                    and self.goal_pose.pose.orientation.z == goal_pose.pose.orientation.z
                    and self.goal_pose.pose.orientation.w == goal_pose.pose.orientation.w
                ):
                    return
        self.goal_updated = True
        self.goal_pose = goal_pose
        if self.go_to_pose_thread is not None:
            self.go_to_pose_thread.join()
        self.goal_updated = False
        self.go_to_pose_thread = Thread(
            target=self._go_to_pose, args=(goal_pose, do_spiral_search, interp_dist, interp_dist_polar)
        )
        self.go_to_pose_thread.start()

    # control robot to desired goal position
    def _go_to_pose(self, goal_pose, do_spiral_search=True, interp_dist=0.001, interp_dist_polar=0.001):
        # the goal pose should be of type PoseStamped. E.g. goal_pose=PoseStampled()
        control_rate = 100
        r = rospy.Rate(control_rate)
        start = self.curr_pos
        start_ori = self.curr_ori
        goal_array = np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z])

        # interpolate from start to goal with attractor distance of approx 1 cm
        dist = np.sqrt(np.sum(np.subtract(start, goal_array) ** 2, axis=0))

        step_num_lin = math.floor(dist / interp_dist)

        q_start = np.quaternion(start_ori[0], start_ori[1], start_ori[2], start_ori[3])

        q_goal = np.quaternion(
            goal_pose.pose.orientation.w,
            goal_pose.pose.orientation.x,
            goal_pose.pose.orientation.y,
            goal_pose.pose.orientation.z,
        )

        inner_prod = q_start.x * q_goal.x + q_start.y * q_goal.y + q_start.z * q_goal.z + q_start.w * q_goal.w
        if inner_prod < 0:
            q_start.x = -q_start.x
            q_start.y = -q_start.y
            q_start.z = -q_start.z
            q_start.w = -q_start.w
        inner_prod = q_start.x * q_goal.x + q_start.y * q_goal.y + q_start.z * q_goal.z + q_start.w * q_goal.w
        theta = np.arccos(np.abs(inner_prod))
        step_num_polar = math.floor(theta / interp_dist_polar)

        step_num = np.max([step_num_polar, step_num_lin])

        x = np.linspace(start[0], goal_pose.pose.position.x, step_num)
        y = np.linspace(start[1], goal_pose.pose.position.y, step_num)
        z = np.linspace(start[2], goal_pose.pose.position.z, step_num)

        goal = PoseStamped()
        self.set_stiffness(4000, 4000, 4000, 50, 50, 30, 0)

        for i in range(step_num):
            if self.goal_updated:
                break
            quat = np.slerp_vectorized(q_start, q_goal, (i + 1) / step_num)
            pos_array = np.array([x[i], y[i], z[i]])
            goal = array_quat_2_pose(pos_array, quat)

            if self.force.z > self.max_force and do_spiral_search:
                spiral_success, offset_correction = self.spiral_search(goal)
                self.spiralling_occured = True
                if spiral_success:
                    x[i:] += offset_correction[0]
                    y[i:] += offset_correction[1]

            self.goal_pub.publish(goal)
            r.sleep()
        if self.goal_updated:
            return
        self.goal_pub.publish(goal_pose)
        self.offset_compensator(3)

    def spiral_search(self, goal, control_rate=20):
        r = rospy.Rate(control_rate)
        max_spiral_time = 30  # seconds
        increase_radius_per_second = 0.0005  # meters, distance from center of the spiral after 1 second
        rounds_per_second = 1  # how many rounds does the spiral every second
        dt = 1.0 / control_rate
        goal_init = position_2_array(goal.pose.position)
        pos_init = self.curr_pos
        ori_quat = orientation_2_quaternion(goal.pose.orientation)
        goal_pose = array_quat_2_pose(goal_init, ori_quat)
        time = 0
        spiral_success = False
        self.set_stiffness(4000, 4000, 1000, 50, 50, 30, 0)  # get more compliant in z direction
        for _ in range(max_spiral_time * control_rate):
            goal_pose.pose.position.x = (
                pos_init[0] + np.cos(2 * np.pi * rounds_per_second * time) * increase_radius_per_second * time
            )
            goal_pose.pose.position.y = (
                pos_init[1] + np.sin(2 * np.pi * rounds_per_second * time) * increase_radius_per_second * time
            )
            self.goal_pub.publish(goal_pose)
            if self.force.z <= self.force_min_exit:
                spiral_success = True
                break
            time += dt
            r.sleep()
        self.set_stiffness(4000, 4000, 4000, 50, 50, 30, 0)
        offset_correction = self.curr_pos - goal_init

        return spiral_success, offset_correction

    def offset_compensator(self, steps):
        curr_quat_desired = list_2_quaternion(np.copy(self.curr_ori_goal))
        curr_pos_desired = np.copy(self.curr_pos_goal)
        for _ in range(steps):
            curr_quat_goal = list_2_quaternion(self.curr_ori_goal)
            curr_pos_goal = self.curr_pos_goal
            curr_quat = list_2_quaternion(self.curr_ori)

            quat_diff = quaternion_divide(curr_quat_desired, curr_quat)
            lin_diff = curr_pos_desired - self.curr_pos

            quat_goal_new = quaternion_product(quat_diff, curr_quat_goal)
            goal_pos = curr_pos_goal + lin_diff

            goal_pose = array_quat_2_pose(goal_pos, quat_goal_new)
            self.goal_pub.publish(goal_pose)
            rospy.sleep(0.2)

    def set_joint_remapping(self, joint_names):
        """Remap joint measurement/commands based on this index mapping

        :param joint_names: joint names.
        :return:
        """
        assert len(joint_names) == 7, "The number of provided joint_names should be 7."
        self._from = []
        for n in joint_names:
            assert n in self.joint_names, f"`{n}` is not a registered joint name."
            i = self.joint_names.index(n)
            self._from.append(i)

        self._to = []
        for n in self.joint_names:
            assert n in joint_names, f"`{n}` was not provided in the joint name remapping."
            i = joint_names.index(n)
            self._to.append(i)
