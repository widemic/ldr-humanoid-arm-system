# LDR Humanoid Arm System - Architecture Diagrams

This document provides visual architecture diagrams using Mermaid syntax that can be rendered in GitHub, VS Code, or online tools.

---

## 1. System Overview Diagram

```mermaid
graph TB
    subgraph "User Interface Layer"
        RViz2[RViz2 Visualization]
        GUI[GUI Tools]
        Keyboard[Keyboard Teleop]
        Joystick[Joystick Teleop]
    end

    subgraph "Application Layer"
        MoveIt[MoveIt2 Planning]
        MotionPlanner[Simple Motion Planner]
        Teleop[Teleop Nodes]

        subgraph "Perception Pipeline"
            OctoMap[OctoMap Server]
            YOLO[YOLO Detection]
            Tracker[Object Tracker]
        end
    end

    subgraph "Control Layer"
        subgraph "ros2_control Framework"
            CtrlMgr[Controller Manager]
            ArmCtrl[arm_controller<br/>6 DOF]
            HandCtrl[hand_controller<br/>gripper]
            JSB[joint_state_broadcaster]
        end
    end

    subgraph "Hardware Abstraction Layer"
        GazeboPlugin[GazeboSimROS2ControlPlugin]
        HardwareIF[Hardware Interface<br/>Placeholder]
    end

    subgraph "Physical Layer"
        Gazebo[Gazebo Harmonic<br/>Simulation]
        Robot[Physical Robot<br/>Future]
    end

    RViz2 --> MoveIt
    GUI --> MotionPlanner
    Keyboard --> Teleop
    Joystick --> Teleop

    MoveIt --> ArmCtrl
    MotionPlanner --> ArmCtrl
    Teleop --> ArmCtrl

    OctoMap --> MoveIt
    YOLO --> Tracker
    Tracker --> MoveIt

    ArmCtrl --> CtrlMgr
    HandCtrl --> CtrlMgr
    JSB --> CtrlMgr

    CtrlMgr --> GazeboPlugin
    CtrlMgr -.-> HardwareIF

    GazeboPlugin --> Gazebo
    HardwareIF -.-> Robot

    style MoveIt fill:#4CAF50
    style MotionPlanner fill:#4CAF50
    style GazeboPlugin fill:#2196F3
    style HardwareIF fill:#9E9E9E
    style Robot fill:#9E9E9E
```

---

## 2. Package Dependency Graph

```mermaid
graph TD
    AD[arm_description<br/>Robot Model]

    AG[arm_gazebo<br/>Simulation]
    AC[arm_control<br/>Simple Control]
    AMC[arm_moveit_config<br/>MoveIt Planning]

    ASB[arm_system_bringup<br/>Integration]

    AP[arm_perception<br/>3D Perception]
    ADemos[arm_demos<br/>Applications]
    AGT[arm_gui_tools<br/>GUI Tools]
    AT[arm_teleop<br/>Teleoperation]

    AD --> AG
    AD --> AC
    AD --> AMC

    AG --> ASB
    AC --> ASB
    AMC --> ASB

    AD --> AP
    AD --> AT

    ASB --> ADemos
    ASB --> AGT

    AC --> AT
    AMC --> ADemos

    style AD fill:#FF5722,color:#fff
    style ASB fill:#4CAF50,color:#fff
    style AG fill:#2196F3,color:#fff
    style AC fill:#2196F3,color:#fff
    style AMC fill:#2196F3,color:#fff
    style AP fill:#9C27B0,color:#fff
```

---

## 3. Control Architecture - Dual Path

```mermaid
graph TB
    subgraph "User Code"
        Simple[Simple Control<br/>motion_planner.py<br/><br/>- Direct joint control<br/>- Fast execution<br/>- Known trajectories]

        Advanced[MoveIt Control<br/>moveit_py<br/><br/>- IK solving<br/>- Collision avoidance<br/>- Cartesian planning]
    end

    Action[/arm_controller/<br/>follow_joint_trajectory<br/><br/>FollowJointTrajectory Action]

    subgraph "ros2_control"
        CM[Controller Manager]

        subgraph "Controllers"
            JSB[joint_state_broadcaster<br/>50 Hz]
            ArmC[arm_controller<br/>JointTrajectoryController<br/>6 arm joints]
            HandC[hand_controller<br/>GripperActionController<br/>2 gripper fingers]
        end
    end

    HW[Hardware Abstraction<br/>GazeboSimROS2ControlPlugin<br/>OR<br/>HardwareInterface]

    Phys[Physical Layer<br/>Gazebo Harmonic<br/>OR<br/>Real Robot Motors]

    Simple --> Action
    Advanced --> Action

    Action --> ArmC

    ArmC --> CM
    HandC --> CM
    JSB --> CM

    CM --> HW
    HW --> Phys

    style Simple fill:#8BC34A
    style Advanced fill:#03A9F4
    style Action fill:#FF9800
    style CM fill:#9C27B0,color:#fff
```

---

## 4. Data Flow - Topics and Actions

```mermaid
graph LR
    subgraph "Sensors"
        Camera[RGBD Camera<br/>Gazebo Plugin]
    end

    subgraph "Topics Published"
        RGB[/camera/color/image_raw<br/>sensor_msgs/Image]
        Depth[/camera/depth/image_raw<br/>sensor_msgs/Image]
        PCL[/camera/depth/points<br/>sensor_msgs/PointCloud2]
        JS[/joint_states<br/>sensor_msgs/JointState]
        TF[/tf, /tf_static<br/>tf2_msgs/TFMessage]
    end

    subgraph "Perception"
        PN[perception_node<br/>Point cloud processing]
        ObjDet[/detected_objects<br/>MarkerArray]
    end

    subgraph "MoveIt"
        PSU[planning_scene_updater]
        PS[/planning_scene<br/>PlanningScene]
        MG[move_group<br/>Planning Server]
    end

    subgraph "Controllers"
        RSP[robot_state_publisher]
        ArmCtrl[arm_controller]
    end

    Camera --> RGB
    Camera --> Depth
    Camera --> PCL

    PCL --> PN
    PN --> ObjDet
    ObjDet --> PSU
    PSU --> PS
    PS --> MG

    ArmCtrl --> JS
    JS --> RSP
    RSP --> TF

    TF --> MG

    style Camera fill:#4CAF50
    style PN fill:#9C27B0,color:#fff
    style MG fill:#2196F3,color:#fff
    style ArmCtrl fill:#FF5722,color:#fff
```

---

## 5. Launch File Hierarchy

```mermaid
graph TD
    Full[full_system.launch.py<br/>arm_system_bringup]

    MG[moveit_gazebo.launch.py<br/>arm_system_bringup]
    MGO[moveit_gazebo_with_octomap.launch.py<br/>arm_system_bringup]

    AW[arm_world.launch.py<br/>arm_gazebo]
    SA[spawn_arm.launch.py<br/>arm_gazebo]

    Demo[demo.launch.py<br/>arm_moveit_config]
    MoveG[move_group.launch.py<br/>arm_moveit_config]

    Perc[perception.launch.py<br/>arm_perception]
    Octo[octomap_server.launch.py<br/>arm_system_bringup]

    Sim[sim.launch.py<br/>arm_control]

    Full --> MGO
    MGO --> MG
    MGO --> Perc
    MGO --> Octo

    MG --> AW
    MG --> MoveG

    AW --> SA

    Sim --> AW

    Demo --> MoveG

    style Full fill:#4CAF50,color:#fff
    style MG fill:#2196F3,color:#fff
    style MGO fill:#2196F3,color:#fff
    style AW fill:#FF9800
    style Demo fill:#9C27B0,color:#fff
```

---

## 6. Perception Pipeline Flow

```mermaid
graph TB
    Input[Point Cloud Input<br/>/camera/depth/points<br/>640x480 @ 30Hz]

    subgraph "Preprocessing"
        PF[PassThrough Filter<br/>Z: 0-1.5m]
        VG[Voxel Grid<br/>1cm leaf size]
    end

    subgraph "Segmentation"
        PS[Plane Segmentation<br/>RANSAC 1cm threshold<br/>Remove floor/walls]
    end

    subgraph "Clustering"
        EC[Euclidean Clustering<br/>5cm tolerance<br/>200-10K points]
    end

    subgraph "Shape Detection"
        BB[Bounding Box<br/>AABB computation]
    end

    subgraph "Recognition Branch"
        YOLO[YOLO Detection<br/>YOLOv8<br/>80 COCO classes]
    end

    subgraph "Tracking Branch"
        KF[Kalman Filter<br/>Position prediction<br/>Velocity estimation]
    end

    Merge[Detected Objects<br/>/detected_objects<br/>MarkerArray]

    PSU[Planning Scene Updater<br/>Sync to MoveIt]

    MoveIt[MoveIt Planning Scene<br/>Collision Avoidance]

    Input --> PF
    PF --> VG
    VG --> PS
    PS --> EC
    EC --> BB
    BB --> YOLO
    BB --> KF
    YOLO --> Merge
    KF --> Merge
    Merge --> PSU
    PSU --> MoveIt

    style Input fill:#4CAF50
    style PS fill:#FF9800
    style EC fill:#2196F3,color:#fff
    style YOLO fill:#9C27B0,color:#fff
    style KF fill:#9C27B0,color:#fff
    style MoveIt fill:#F44336,color:#fff
```

---

## 7. Robot Model Structure (URDF)

```mermaid
graph TB
    Main[arm.urdf.xacro<br/>Main Entry Point<br/><br/>use_sim argument]

    Base[arm_base.xacro<br/>Base Structure]

    SimBranch{use_sim?}

    GazeboX[arm_gazebo.xacro<br/>Simulation Mode<br/><br/>- GazeboSimROS2ControlPlugin<br/>- Gazebo properties<br/>- Camera sensor]

    RealX[arm_real.xacro<br/>Real Hardware Mode<br/><br/>- Hardware ros2_control<br/>- Real motor interfaces]

    Links[arm_links.xacro<br/>14 Links<br/><br/>- Visual meshes STL<br/>- Collision meshes STL<br/>- Inertial properties]

    Joints[arm_joints.xacro<br/>8 Actuated Joints<br/><br/>6 arm + 2 gripper<br/>Limits, dynamics]

    RC[ros2_control.xacro<br/>Control Interfaces<br/><br/>position/velocity<br/>command/state]

    Cam[depth_camera.xacro<br/>RGBD Camera<br/><br/>640x480 @ 30Hz<br/>RGB + Depth + PCL]

    Main --> Base
    Main --> SimBranch

    SimBranch -->|true| GazeboX
    SimBranch -->|false| RealX

    Base --> Links
    Base --> Joints

    GazeboX --> RC
    GazeboX --> Cam

    RealX --> RC

    style Main fill:#4CAF50,color:#fff
    style GazeboX fill:#2196F3,color:#fff
    style RealX fill:#9E9E9E,color:#fff
    style Links fill:#FF9800
    style Joints fill:#FF9800
```

---

## 8. TF Transform Tree

```mermaid
graph TB
    Map[map<br/>World frame]

    Base[base_link<br/>Robot base]

    Camera[camera_link]
    CamRGB[camera_rgb_frame]
    CamDepth[camera_depth_frame]
    CamOpt[camera_optical_frame]

    SP[left_shoulder_pitch_link]
    SR[left_shoulder_roll_link]
    SY[left_shoulder_yaw_link]

    Elbow[left_elbow_link]
    Wrist[left_wrist_link]
    Hand[left_hand_link]

    Palm[left_palm_link]
    FR[left_finger_right_link]
    FL[left_finger_left_link]

    Map --> Base

    Base --> Camera
    Base --> SP

    Camera --> CamRGB
    Camera --> CamDepth
    Camera --> CamOpt

    SP --> SR
    SR --> SY
    SY --> Elbow
    Elbow --> Wrist
    Wrist --> Hand
    Hand --> Palm
    Palm --> FR
    Palm --> FL

    style Map fill:#4CAF50
    style Base fill:#FF5722,color:#fff
    style Camera fill:#2196F3,color:#fff
    style Palm fill:#9C27B0,color:#fff
```

---

## 9. Controller Lifecycle State Machine

```mermaid
stateDiagram-v2
    [*] --> Unconfigured: Load controller

    Unconfigured --> Inactive: configure()
    Unconfigured --> [*]: unload()

    Inactive --> Active: activate()
    Inactive --> Unconfigured: cleanup()

    Active --> Inactive: deactivate()

    note right of Unconfigured
        Controller loaded
        No resources allocated
    end note

    note right of Inactive
        Resources allocated
        Not processing commands
    end note

    note right of Active
        Processing commands
        Publishing states
    end note
```

---

## 10. MoveIt Planning Pipeline

```mermaid
graph TB
    Start[Planning Request<br/>Goal pose/joints]

    subgraph "Planning Scene"
        PS[Get Current State<br/>/joint_states]
        Coll[Load Collision Objects<br/>OctoMap + static]
    end

    IK{Goal type?}

    IKSolve[IK Solver KDL<br/>Cartesian → Joint]

    subgraph "Motion Planning"
        OMPL[OMPL Planner<br/>RRT*/PRM/etc.]
        Pilz[Pilz Planner<br/>LIN/PTP/CIRC]
        CHOMP[CHOMP<br/>Gradient-based]
    end

    Valid{Valid plan?}

    subgraph "Trajectory Processing"
        Smooth[Smoothing<br/>Time parameterization]
        Limit[Apply Limits<br/>Velocity/acceleration]
    end

    Execute[Execute<br/>Send to arm_controller]

    Monitor[Execution Monitor<br/>Track progress]

    Success{Success?}

    Done[Plan Complete]
    Fail[Plan Failed]

    Start --> PS
    PS --> Coll
    Coll --> IK

    IK -->|Cartesian| IKSolve
    IK -->|Joint| OMPL
    IKSolve --> OMPL

    OMPL --> Valid
    Pilz --> Valid
    CHOMP --> Valid

    Valid -->|Yes| Smooth
    Valid -->|No| Fail

    Smooth --> Limit
    Limit --> Execute
    Execute --> Monitor
    Monitor --> Success

    Success -->|Yes| Done
    Success -->|No| Fail

    style Start fill:#4CAF50
    style Done fill:#8BC34A
    style Fail fill:#F44336,color:#fff
    style Execute fill:#FF9800
```

---

## 11. Deployment Architecture - Simulation

```mermaid
graph TB
    subgraph "Single Computer - Ubuntu 24.04"
        subgraph "ROS 2 Jazzy Workspace"
            Gazebo[Gazebo Harmonic<br/>Physics simulation]
            MoveIt[MoveIt2<br/>Motion planning]
            RViz[RViz2<br/>Visualization]
            Perc[Perception<br/>Point cloud processing]
            Ctrl[Controllers<br/>ros2_control]
        end
    end

    User[User]

    User --> RViz
    User --> MoveIt

    MoveIt --> Ctrl
    Perc --> MoveIt
    Ctrl --> Gazebo
    Gazebo --> Perc

    style Gazebo fill:#2196F3,color:#fff
    style MoveIt fill:#4CAF50,color:#fff
```

---

## 12. Deployment Architecture - Real Hardware (Future)

```mermaid
graph TB
    subgraph "Control Computer"
        subgraph "ROS 2 Jazzy"
            MoveIt[MoveIt2<br/>Planning]
            Perc[Perception<br/>3D vision]
            RViz[RViz2<br/>Monitor]
            Ctrl[Controllers<br/>ros2_control]
        end
    end

    Network[Ethernet/USB/CAN<br/>Communication]

    subgraph "Hardware Interface Computer"
        HW[arm_hardware<br/>ros2_control plugin]
        Driver[Motor Drivers<br/>CAN/Ethernet]
    end

    subgraph "Physical Robot"
        Motors[6 Arm Motors<br/>2 Gripper Motors]
        Cam[RGBD Camera]
        Enc[Joint Encoders]
    end

    User[User]

    User --> RViz
    User --> MoveIt

    MoveIt --> Ctrl
    Perc --> MoveIt
    Ctrl --> Network
    Network --> HW
    HW --> Driver
    Driver --> Motors

    Cam --> Perc
    Enc --> HW

    style HW fill:#9E9E9E,color:#fff
    style Motors fill:#9E9E9E,color:#fff
    style Network fill:#FF9800
```

---

## 13. Build System Flow

```mermaid
graph LR
    subgraph "Source Code"
        Src[src/ packages]
        PkgXML[package.xml<br/>Dependencies]
        CMake[CMakeLists.txt<br/>Build rules]
    end

    subgraph "Colcon Build"
        Parse[Parse dependencies]
        Order[Build order]
        Compile[Compile C++]
        Install[Install files]
    end

    subgraph "Outputs"
        Build[build/<br/>Artifacts]
        Inst[install/<br/>Packages]
        Log[log/<br/>Build logs]
    end

    Setup[install/setup.bash<br/>Environment]

    Run[ros2 launch<br/>Run system]

    Src --> Parse
    PkgXML --> Parse
    CMake --> Compile

    Parse --> Order
    Order --> Compile
    Compile --> Install

    Install --> Build
    Install --> Inst
    Install --> Log

    Inst --> Setup
    Setup --> Run

    style Parse fill:#4CAF50
    style Compile fill:#FF9800
    style Inst fill:#2196F3,color:#fff
```

---

## 14. Modular vs All-in-One Launch Strategy

```mermaid
graph TB
    subgraph "Strategy 1: Modular Separate Terminals"
        T1[Terminal 1<br/>ros2 launch arm_control sim.launch.py<br/><br/>Gazebo + Controllers]
        T2[Terminal 2 wait 20s<br/>ros2 launch arm_moveit_config demo.launch.py<br/><br/>MoveIt + RViz]

        T1 -.->|Ready| T2
    end

    subgraph "Strategy 2: All-in-One Single Terminal"
        Single[Single Terminal<br/>ros2 launch arm_system_bringup moveit_gazebo.launch.py<br/><br/>Everything with automatic timing]
    end

    User[User Choice]

    User -->|Debug/Development| T1
    User -->|Demo/Production| Single

    style T1 fill:#8BC34A
    style T2 fill:#4CAF50
    style Single fill:#2196F3,color:#fff
```

---

## 15. Timing Sequence Diagram - System Startup

```mermaid
sequenceDiagram
    participant User
    participant Gazebo
    participant ControllerMgr as Controller Manager
    participant Controllers
    participant MoveGroup as move_group
    participant RViz

    User->>Gazebo: Launch Gazebo (t=0s)
    activate Gazebo
    Note over Gazebo: World loading...<br/>Physics initialization

    Gazebo-->>ControllerMgr: Plugin loaded (t=3s)
    activate ControllerMgr

    User->>Controllers: Spawn joint_state_broadcaster (t=4s)
    activate Controllers
    Controllers-->>ControllerMgr: Register controller

    User->>Controllers: Spawn arm_controller (t=5s)
    Controllers-->>ControllerMgr: Register controller
    Note over Controllers: /joint_states publishing

    User->>MoveGroup: Launch move_group (t=6s)
    activate MoveGroup
    MoveGroup->>Controllers: Subscribe /joint_states
    MoveGroup->>ControllerMgr: Query controllers
    Note over MoveGroup: Planning ready

    User->>RViz: Launch RViz (t=8s)
    activate RViz
    RViz->>MoveGroup: Connect to planning
    RViz->>Controllers: Subscribe /joint_states
    Note over RViz: Visualization ready

    Note over User,RViz: System Ready (t=10s)
```

---

**End of Architecture Diagrams**

## Usage

### Viewing Diagrams

**GitHub:** These diagrams will render automatically when viewing this file on GitHub.

**VS Code:** Install the "Markdown Preview Mermaid Support" extension to see diagrams in preview.

**Online:** Copy diagram code blocks to https://mermaid.live for interactive editing.

**Export:** Use mermaid-cli to export diagrams to PNG/SVG:
```bash
npm install -g @mermaid-js/mermaid-cli
mmdc -i ARCHITECTURE_DIAGRAMS.md -o architecture_diagrams.pdf
```

### Diagram Legend

- **Green**: Active/functional components
- **Blue**: Core system components
- **Gray**: Placeholder/future components
- **Orange**: Configuration/intermediate
- **Purple**: Perception/advanced features
- **Red**: Critical/important components
