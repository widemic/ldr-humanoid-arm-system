🧩 2️⃣ Xacro integration

In arm_description/urdf/arm.xacro, add Gazebo plugin support but don’t bake it into the CAD URDF.

Example addition:

<!-- Only loaded in simulation -->
<xacro:if value="$(arg use_gazebo)">
  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <ros2_control>
        <parameters>$(find arm_control)/config/ros2_control_sim.yaml</parameters>
      </ros2_control>
    </plugin>
  </gazebo>
</xacro:if>


And define an argument at the top:

<xacro:arg name="use_gazebo" default="false"/>


So your robot works both in RViz and in Gazebo, depending on launch file arguments.

⚙️ 3️⃣ Gazebo configuration files
------------simulation/arm_gazebo/config/gazebo_ros2_control.yaml

This file connects joints to controllers and plugins.

controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: position_controllers/JointTrajectoryController
      joints:
        - base_rotation_joint
        - shoulder_pitch_joint
        - elbow_pitch_joint
        - wrist_pitch_joint
        - wrist_roll_joint
      gains:
        base_rotation_joint: {p: 100.0, d: 1.0}
        shoulder_pitch_joint: {p: 100.0, d: 1.0}
        elbow_pitch_joint: {p: 100.0, d: 1.0}
        wrist_pitch_joint: {p: 50.0, d: 0.5}
        wrist_roll_joint: {p: 50.0, d: 0.5}

---------simulation/arm_gazebo/config/controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
      joints:
        - base_rotation_joint
        - shoulder_pitch_joint
        - elbow_pitch_joint
        - wrist_pitch_joint
        - wrist_roll_joint
      command_interfaces:
        - position
      state_interfaces:
        - position
        - velocity

🌍 4️⃣ World file (example)

simulation/arm_gazebo/worlds/lab.world

<sdf version="1.6">
  <world name="lab">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <physics type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>
  </world>
</sdf>

🚀 5️⃣ Launch files
simulation/arm_gazebo/launch/spawn_arm.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_arm_description = FindPackageShare('arm_description')
    pkg_arm_gazebo = FindPackageShare('arm_gazebo')

    use_gazebo = LaunchConfiguration('use_gazebo', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf = PathJoinSubstitution([pkg_arm_description, 'urdf', 'arm.xacro'])

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'arm',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_gazebo', default_value='true'),
        robot_state_publisher,
        spawn_entity
    ])

simulation/arm_gazebo/launch/arm_world.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_arm_gazebo = FindPackageShare('arm_gazebo')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'world': PathJoinSubstitution([pkg_arm_gazebo, 'worlds', 'lab.world'])}.items()
    )

    spawn_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_arm_gazebo, 'launch', 'spawn_arm.launch.py'])
        )
    )

    return LaunchDescription([gazebo, spawn_arm])

✅ 6️⃣ Run everything

To bring up Gazebo simulation:

ros2 launch arm_gazebo arm_world.launch.py


You should see:

Gazebo open with your robot on the ground

/joint_states publishing

/arm_controller/follow_joint_trajectory available

Then, you can control it via:

ros2 control list_controllers
ros2 topic pub /arm_controller/joint_trajectory ...

💡 7️⃣ Key design benefits
Principle	Benefit
Description is isolated	CAD / Xacro independent of sim config
Gazebo files modular	Easy to change physics or world
ros2_control YAML split	Reuse in simulation and hardware
Wrapper launch hierarchy	Extendable to dual-arm or gripper setups later

If you want, I can show you how to extend this same simulation setup to include the gripper or even connect it with arm_moveit_config for trajectory execution in Gazebo.

Would you like me to extend it to include the gripper too?