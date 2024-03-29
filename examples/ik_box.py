import eagerx
import eagerx_franka
from eagerx_franka import utils
import numpy as np
import stable_baselines3 as sb
from datetime import datetime
import os
import eagerx_utility


def position_control(_graph, _arm, source_goal, safe_rate):
    # Add position control actuator
    if "pos_control" not in _arm.config.actuators:
        _arm.config.actuators.append("pos_control")

    # Create safety node
    from eagerx_utility.safety.node import SafePositionControl


    c = _arm.config
    urdf = c.urdf
    urdf_sbtd = urdf.replace("package://", PACKAGE_PATH)
    collision = dict(
        workspace="eagerx_utility.safety.workspaces/exclude_ground",
        # workspace="eagerx_franka.safety.workspaces/exclude_ground_minus_2m",
        margin=0.01,  # [cm]
        gui=False,
        robot=dict(urdf=urdf_sbtd, basePosition=c.base_pos, baseOrientation=c.base_or),
    )
    safe = SafePositionControl.make(
        "safety",
        safe_rate,
        c.joint_names,
        c.joint_upper,
        c.joint_lower,
        [0.2 * vl for vl in c.vel_limit],
        checks=3,
        collision=collision,
    )
    _graph.add(safe)

    # Connecting safety filter to arm
    _graph.connect(**source_goal, target=safe.inputs.goal)
    _graph.connect(source=_arm.sensors.position, target=safe.inputs.current)
    _graph.connect(source=safe.outputs.filtered, target=_arm.actuators.pos_control)

    return safe


def velocity_control(_graph, _arm, source_goal, safe_rate):
    # Add velocity control actuator
    if "vel_control" not in _arm.config.actuators:
        _arm.config.actuators.append("vel_control")

    # Create safety node
    from eagerx_utility.safety.node import SafeVelocityControl

    c = _arm.config
    urdf = c.urdf
    urdf_sbtd = urdf.replace("package://", PACKAGE_PATH)
    collision = dict(
        workspace="eagerx_utility.safety.workspaces/exclude_ground",
        # workspace="eagerx_franka.safety.workspaces/exclude_ground_minus_2m",
        margin=0.01,  # [cm]
        gui=False,
        robot=dict(urdf=urdf_sbtd, basePosition=c.base_pos, baseOrientation=c.base_or),
    )
    safe = SafeVelocityControl.make(
        "safety",
        safe_rate,
        c.joint_names,
        c.joint_upper,
        c.joint_lower,
        [0.2 * vl for vl in c.vel_limit],
        checks=3,
        collision=collision,
    )
    _graph.add(safe)

    # Connecting goal
    _graph.connect(**source_goal, target=safe.inputs.goal)
    # Connecting safety filter to arm
    _graph.connect(source=_arm.sensors.position, target=safe.inputs.position)
    _graph.connect(source=_arm.sensors.velocity, target=safe.inputs.velocity)
    _graph.connect(source=safe.outputs.filtered, target=_arm.actuators.vel_control)

    return safe


NAME = "HER_force_torque"
LOG_DIR = os.path.dirname(eagerx_franka.__file__) + f"/../logs/{NAME}_{datetime.today().strftime('%Y-%m-%d-%H%M')}"
PACKAGE_PATH = os.path.dirname(eagerx_franka.__file__) + "/assets/franka_panda/"


if __name__ == "__main__":
    eagerx.set_log_level(eagerx.WARN)

    n_procs = 1  # sb HER does not support multiprocessing
    rate = 10  # 20
    safe_rate = 20
    T_max = 10.0  # [sec]
    add_bias = False
    excl_z = False
    USE_POS_CONTROL = False
    MUST_LOG = True
    MUST_TEST = True

    # Initialize empty graph
    graph = eagerx.Graph.create()

    # Create solid object
    from eagerx_utility.solid.solid import Solid
    import yaml

    urdf_path = os.path.dirname(eagerx_franka.__file__) + "/solid/assets/"
    cam_path = os.path.dirname(eagerx_franka.__file__) + "/assets/calibrations"
    cam_name = "logitech_c170"
    with open(f"{cam_path}/{cam_name}.yaml", "r") as f:
        cam_intrinsics = yaml.safe_load(f)
    cam_translation = [0.811, 0.527, 0.43]
    cam_rotation = [0.321, 0.801, -0.466, -0.197]

    solid = Solid.make(
        "solid",
        urdf=urdf_path + "box.urdf",
        rate=rate,
        cam_translation=cam_translation,
        cam_rotation=cam_rotation,
        cam_index=2,
        cam_intrinsics=cam_intrinsics,
        # sensors=["position", "yaw", "robot_view"],  # select robot_view to render.
        sensors=["position", "yaw"],  # select robot_view to render.
        states=["position", "velocity", "orientation", "angular_vel", "lateral_friction"],
    )

    solid.sensors.position.space.update(low=[-1, -1, 0], high=[1, 1, 0.13])
    graph.add(solid)

    # Create solid goal
    from eagerx_utility.solid.goal import Goal

    goal = Goal.make(
        "goal",
        urdf=urdf_path + "box_goal.urdf",
        rate=rate,
        sensors=["position", "yaw"],
        states=["position", "orientation"],
    )
    goal.sensors.position.space.update(low=[0, -1, 0], high=[1, 1, 0])
    graph.add(goal)

    # Circular goal
    x, y, z = 0.45, 0.0, 0.05
    dx, dy = 0.1, 0.20
    solid.states.lateral_friction.space.update(low=0.1, high=0.4)
    solid.states.orientation.space.update(low=[-1, -1, 0, 0], high=[1, 1, 0, 0])
    solid.states.position.space.update(low=[x, -y - dy, z], high=[x + dx, y + dy, z])
    goal.states.orientation.space.update(low=[-1, -1, 0, 0], high=[1, 1, 0, 0])
    goal.states.position.space.update(low=[x, -y - dy, 0], high=[x + dx, y + dy, 0])

    # Create arm
    from eagerx_franka.franka_arm.franka_arm import FrankaArm

    robot_type = "panda"
    arm = FrankaArm.make(
        name=robot_type,
        robot_type=robot_type,
        sensors=["position", "velocity", "force_torque", "ee_pos", "ee_orn"],
        actuators=[],
        states=["position", "velocity", "gripper"],
        rate=rate,
    )
    arm.states.gripper.space.update(low=[0.0], high=[0.0])  # Set gripper to closed position
    arm.states.position.space.low[3] = -np.pi / 2
    arm.states.position.space.high[3] = -np.pi / 2
    arm.states.position.space.low[5] = np.pi / 2
    arm.states.position.space.high[5] = np.pi / 2
    graph.add(arm)

    # Create IK node
    from eagerx_franka.ik.node import EndEffectorDownward
    import eagerx_franka.franka_arm.mr_descriptions as mrd

    robot_des = getattr(mrd, robot_type)
    ik = EndEffectorDownward.make(
        "ik",
        rate,
        robot_des.Slist.tolist(),
        robot_des.M.tolist(),
        -np.pi,
        np.pi,
        max_dxyz=[0.2, 0.2, 0.2],
        max_dyaw=2 * np.pi / 2,
        eomg=0.01,
        ev=0.01,
    )
    graph.add(ik)

    if USE_POS_CONTROL:
        safe = position_control(graph, arm, dict(source=ik.outputs.target), safe_rate)
    else:
        safe = velocity_control(graph, arm, dict(source=ik.outputs.dtarget), safe_rate)

    # Connecting observations
    graph.connect(source=arm.sensors.position, observation="joints")
    graph.connect(source=arm.sensors.velocity, observation="velocity")
    graph.connect(source=arm.sensors.force_torque, observation="force_torque")
    graph.connect(source=arm.sensors.ee_pos, observation="ee_position")
    graph.connect(source=solid.sensors.position, observation="pos")
    graph.connect(source=solid.sensors.yaw, observation="yaw")
    graph.connect(source=goal.sensors.position, observation="pos_desired")

    # Connect IK
    graph.connect(source=arm.sensors.position, target=ik.inputs.current)
    graph.connect(source=arm.sensors.ee_pos, target=ik.inputs.xyz)
    graph.connect(source=arm.sensors.ee_orn, target=ik.inputs.orn)
    # Connecting actions
    graph.connect(action="dxyz", target=ik.inputs.dxyz)
    graph.connect(action="dyaw", target=ik.inputs.dyaw)

    # Add rendering
    if "robot_view" in solid.config.sensors:
        # Create camera
        from eagerx_utility.camera.objects import Camera

        cam = Camera.make(
            "cam",
            rate=rate,
            sensors=["image"],
            urdf=os.path.dirname(eagerx_utility.__file__) + "/camera/assets/realsense2_d435.urdf",
            optical_link="camera_color_optical_frame",
            calibration_link="camera_bottom_screw_frame",
            camera_index=0,  # todo: set correct index
        )
        graph.add(cam)
        # Create overlay
        from eagerx_utility.overlay.node import Overlay

        overlay = Overlay.make("overlay", rate=20, resolution=[480, 480], caption="robot view")
        graph.add(overlay)
        # Connect
        graph.connect(source=solid.sensors.robot_view, target=overlay.inputs.main)
        graph.connect(source=cam.sensors.image, target=overlay.inputs.thumbnail)
        graph.render(source=overlay.outputs.image, rate=20, encoding="bgr")

    graph.gui()

    # Define environment
    from eagerx_franka.env import ArmEnv
    from eagerx_franka.goal_env import GoalArmEnv

    def make_env(rank: int, use_ros: bool = True):
        gui = True if rank == 0 else False
        if rank == 0 and use_ros:
            from eagerx.backends.ros1 import Ros1

            backend = Ros1.make()
        else:
            from eagerx.backends.single_process import SingleProcess

            backend = SingleProcess.make()

        # Define engines
        from eagerx_pybullet.engine import PybulletEngine

        engine = PybulletEngine.make(rate=safe_rate, gui=gui, egl=True, sync=True, real_time_factor=0)

        def _init():
            graph.reload()
            env = ArmEnv(
                name=f"ArmEnv_{rank}",
                rate=rate,
                graph=graph,
                engine=engine,
                backend=backend,
                exclude_z=excl_z,
                max_steps=int(T_max * rate),
            )
            goal_env = GoalArmEnv(env, add_bias=add_bias)
            env = utils.RescaleAction(goal_env, min_action=-1.0, max_action=1.0)
            return env

        return _init

    # Use multi-processing
    if n_procs > 1:
        from stable_baselines3.common.vec_env import SubprocVecEnv

        train_env = SubprocVecEnv([make_env(i) for i in range(n_procs)], start_method="spawn")
    else:
        train_env = make_env(rank=0, use_ros=False)()

    # Initialize model
    if MUST_LOG:
        os.mkdir(LOG_DIR)
        graph.save(f"{LOG_DIR}/graph.yaml")
        from stable_baselines3.common.callbacks import CheckpointCallback

        checkpoint_callback = CheckpointCallback(save_freq=25_000, save_path=LOG_DIR, name_prefix="rl_model")
    else:
        LOG_DIR = None
        checkpoint_callback = None

    # First train in simulation
    # train_env.render("human")
    obs_space = train_env.observation_space

    # Evaluate
    if MUST_TEST:
        for eps in range(5000):
            print(f"Episode {eps}")
            _, done = train_env.reset(), False
            done = np.array([done], dtype="bool") if isinstance(done, bool) else done
            while not done.all():
                action = train_env.action_space.sample()
                obs, reward, terminated, truncated, info = train_env.step(action)

    # Create experiment directory
    total_steps = 2_500_000
    goal_selection_strategy = "future"  # equivalent to GoalSelectionStrategy.FUTURE
    model = sb.SAC(
        "MultiInputPolicy",
        train_env,
        replay_buffer_class=sb.HerReplayBuffer,
        # Parameters for HER
        replay_buffer_kwargs=dict(
            n_sampled_goal=4,
            goal_selection_strategy=goal_selection_strategy,
        ),
        verbose=1,
        tensorboard_log=LOG_DIR,
    )
    model.learn(total_steps, callback=checkpoint_callback)
