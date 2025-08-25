# Admittance Control for Haptic Surface Exploration

This repository contains a comprehensive admittance control system for haptic exploration of unknown surfaces using a Franka Emika FR3 robot arm. The system integrates force/torque sensing, contact detection, and hybrid force-position control to systematically scan and map object surfaces.

## Features

### Core Capabilities
- **Hybrid Force-Position Control**: Simultaneous control of contact forces and end-effector positioning
- **Contact Point Detection**: Real-time detection and recording of surface contact points
- **Surface Scanning**: Systematic lateral and axial scanning of object surfaces
- **Force Feedback**: Adaptive force control with configurable thresholds
- **Data Recording**: Automatic logging of contact points with 3D visualization support

### Control Modes
- **Position Control**: Precise end-effector positioning for initialization
- **Force Control**: Maintains desired contact force during surface exploration
- **Hybrid Control**: Combines position and force control for optimal surface tracking
- **Orientation Control**: Maintains proper tool orientation relative to surface normals

## System Architecture

### Package Structure
```
src/
├── fr3_control_publish/           # Main control and sensing package
│   ├── src/
│   │   ├── contact_point_recorder.cpp      # Contact detection and logging
│   │   ├── ee_pose_bridge.cpp              # End-effector pose publisher
│   │   ├── fr3_jacobian_publisher.cpp      # Robot jacobian calculations
│   │   └── hybridforcetangentialcontroller.cpp  # Main hybrid controller
│   ├── msg/
│   │   └── TouchTipWrench.msg              # Custom force/torque message
│   └── launch/
│       └── contact_scanning.launch.py     # Contact scanning launch file
└── mujoco_ros2_control/           # MuJoCo simulation integration
    └── mujoco_ros2_control_demos/
        ├── launch/
        │   └── fr3_hzx.launch.py           # Main system launch file
        ├── config/
        │   └── fr3_controllers.yaml        # Controller configurations
        ├── urdf/
        │   └── fr3_nohand.urdf             # Robot description
        └── mujoco_models/
            ├── fr3.xml                     # MuJoCo robot model
            └── scene.xml                   # Simulation environment
```

### System Components

1. **MuJoCo-ROS2 Control Node**
   - Provides physics simulation and robot control interface
   - Handles joint commands and state feedback

2. **Robot State Publisher**
   - Publishes robot kinematic transformations
   - Maintains TF tree for coordinate frame relationships

3. **Jacobian Publisher** (`jacobian_publisher`)
   - Computes and publishes robot Jacobian matrices
   - Enables Cartesian space control calculations

4. **End-Effector Pose Bridge** (`ee_pose_bridge`)
   - Publishes end-effector pose at high frequency (100 Hz)
   - Transforms between base frame (`fr3_link0`) and tool frame (`touch_tip`)

5. **Contact Point Recorder** (`contact_point_recorder`)
   - Detects contact events based on force threshold
   - Records contact points with timestamps and force data
   - Exports data in CSV and PLY formats for analysis

6. **Hybrid Force-Tangential Controller** (`hybridforcetangentialcontroller`)
   - Main control algorithm implementing admittance control
   - Manages scanning trajectory and force regulation
   - Handles surface approach, contact maintenance, and exploration phases

## Configuration Parameters

### Force Control Parameters
- `f_high`: 25.0 N - Contact detection threshold
- `f_low`: 0.5 N - Contact loss threshold  
- `desired_contact_force`: 25.0 N - Target force during surface contact

### Scanning Parameters
- `step_length`: 0.003 m - Axial advancement step size
- `lateral_step`: 0.005 m - Lateral scanning step size
- `axis_speed`: 0.02 m/s - Axial advancement velocity
- `lateral_speed`: 0.1 m/s - Lateral scanning velocity

### Object Parameters
- `object_center_x/y`: Object center coordinates for scanning boundary
- `object_width`: 0.2 m - Object width for scanning limits
- `object_height`: 0.3 m - Object height for scanning limits

### Force Measurement Parameters
- `T_pause`: 0.15 s - Static force measurement window
- `N_min`: 10 - Minimum samples in measurement window
- `f_eps`: 0.2 N - Force fluctuation threshold for steady-state detection

## Usage Instructions

### Prerequisites
- ROS 2 (tested with Humble)
- MuJoCo physics engine
- Franka robot description packages

### Building the System
```bash
# Navigate to workspace
cd /path/to/Haptic-Exploration-of-Unknown-Surfaces/Admittance\ Control

# Build packages
colcon build

# Source environment
source install/setup.bash
```

### Running the System

#### Launch Complete System
The main launch file starts all necessary components:
```bash
ros2 launch mujoco_ros2_control_demos fr3_hzx.launch.py
```

This launch file will:
1. Start MuJoCo simulation with FR3 robot model
2. Initialize robot state publisher
3. Launch joint controllers (with 3s delay)
4. Start contact detection and recording system (with 5s delay)
5. Activate hybrid force controller for surface exploration

#### Monitor System Status
```bash
# Check active topics
ros2 topic list

# Monitor contact points
ros2 topic echo /contact_points

# View end-effector pose
ros2 topic echo /ee_pose

# Check force/torque data
ros2 topic echo /touch_tip/wrench
```

### Data Output
Contact data is automatically saved to the `contact_data/` directory:
- `contact_points.csv` - Contact point coordinates and force data
- `contact_points.ply` - 3D point cloud for visualization

## Control Algorithm

The system implements a multi-phase control strategy:

1. **Initialization Phase**: Move to starting position above object
2. **Approach Phase**: Controlled descent until contact detection
3. **Contact Phase**: Regulate contact force while maintaining position
4. **Scanning Phase**: Systematic lateral and axial surface exploration
5. **Data Recording**: Continuous logging of contact points and forces

The admittance control law adapts end-effector motion based on measured forces, enabling compliant interaction with unknown surface geometries while maintaining consistent contact forces.

## Troubleshooting

### Common Issues
- **Controller startup delays**: The system uses timed delays to ensure proper initialization order
- **Force sensor calibration**: Verify force/torque sensor is properly zeroed before operation  
- **Simulation stability**: Ensure MuJoCo timestep is appropriate for control frequency

### Parameter Tuning
- Adjust `f_high` and `f_low` thresholds based on surface properties
- Modify scanning speeds for different material types
- Tune force control gains for stability vs. responsiveness

## Contributing

When modifying the system:
1. Update parameter documentation in launch files
2. Test with both simulation and real robot hardware
3. Validate data recording functionality
4. Ensure proper error handling and safety limits

## Dependencies

- `mujoco_ros2_control` - MuJoCo-ROS2 integration
- `controller_manager` - ROS2 control framework
- `robot_state_publisher` - TF publishing
- `fr3_control_publish` - Custom control and sensing nodes
