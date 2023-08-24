import os
import eagerx_franka
import yaml
from eagerx_utility.utils import launch_node
import gymnasium as gym


def get_joint_limits(robot_model: str, joint_limits: str):

    config_path = os.path.dirname(eagerx_franka.__file__) + f"/assets/robots/{robot_model}"
    try:
        joint_limits = joint_limits if isinstance(joint_limits, str) else f"{config_path}/joint_limits.yaml"
        with open(joint_limits, "r") as yamlfile:
            joint_limits = yaml.safe_load(yamlfile)
    except IOError:
        print(f"Joint Limits File was not found in: {config_path}")
        raise
    return joint_limits


def generate_urdf(
    robot: str,
    ns="",
    use_gripper=True,
):
    import rospy
    import rosparam

    module_path = os.path.dirname(eagerx_franka.__file__) + "/assets/"
    launch_file = module_path + "franka_description.launch"
    cli_args = [
        f"module_path:={module_path}",
        f"robot:={robot}",
        f"ns:={ns}",
        f"use_gripper:={use_gripper}",
    ]
    launch = launch_node(launch_file, cli_args)
    launch.start()

    # Replace mesh urls
    urdf_key = f"{ns}/{robot}/robot_description"
    urdf: str = rospy.get_param(urdf_key)
    urdf_sbtd = urdf.replace("package://franka_description/", module_path)
    rosparam.upload_params(urdf_key, urdf_sbtd)

    return urdf_key


def RescaleAction(env, min_action, max_action):
    env = gym.wrappers.rescale_action.RescaleAction(env, min_action, max_action)
    env.action_space = gym.spaces.Box(
        low=min_action,
        high=max_action,
        shape=env.action_space.shape,
        dtype=env.action_space.dtype,
    )
    return env
