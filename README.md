[//]: # (Image References)

[image1]: ./assets/skid1.png "Starter package"
[image2]: ./assets/skid2.png "Adding a camera"
[image3]: ./assets/skid3.png "Adding a camera"
[image4]: ./assets/odom1.jpg "Adding a camera"
[image5]: ./assets/camera1.png "Adding a camera"
[image6]: ./assets/cam_rqt.png "rqt reconfigure"
[image7]: ./assets/compressed_rqt.png "Wide angle camera"
[image8]: ./assets/relay_node_compressed.png "Wide angle camera"
[image9]: ./assets/rqt_reconfigure.png "IMU"
[image10]: ./assets/rgbd2.png "TF Tree"
[image11]: ./assets/rgbd1.png "Odometry"
[image12]: ./assets/depth-image.png "Navsat"
[image13]: ./assets/lidar1.png "New York - Madrid"
[image14]: ./assets/visualize_lidar.png "RViz GPS"
[image15]: ./assets/lidar2.png "RViz GPS"
[image16]: ./assets/3d_lidar.png "Lidar"
[image17]: ./assets/3d_mapping.png "Lidar"
[image18]: ./assets/mapping_rgbd.png "Lidar"
[image19]: ./assets/ekf1.png "Lidar"
[image20]: ./assets/imu1.png "Lidar"
[image21]: ./assets/opencv1.png "RGBD Camera"
[image22]: ./assets/opencv2.png "RGBD Camera"
[image23]: ./assets/opencv3.png "RGBD Camera"
[image24]: ./assets/opencv4.png "Depth image"
[image25]: ./assets/yolo1.png "OpenCV"
[image26]: ./assets/yolo2.png "Red ball in Gazebo"
[image27]: ./assets/opencv.png "Red ball in Gazebo"

# Session 4 : Sensors and Perception Guide

A robot without sensors is like a human with their eyes closed.

It can move.
It can turn.
But it has no idea what's happening around it.

In this session, we'll give our robot the ability to see, measure, and understand its surroundings.

Starting from a basic skid-steer robot, we'll add cameras, RGBD sensing, lidar, and an IMU, learn how robots estimate their position using sensor fusion, and finally build vision applications using OpenCV and YOLOv8.

Welcome to the world of robot perception.


## Table of Contents
1. [Introduction](#introduction)
2. [Skid-Steer](#skid-steer)
3. [Friction](#friction)
4. [Odometry](#odometry)
5. [Sensors](#sensors)
   - [Camera](#camera)
     - [Image Transport](#image-transport)
     - [rqt_reconfigure](#rqt_reconfigure)
   - [RGBD Camera](#rgbd-camera)
   - [Lidar](#lidar)
     - [2D Lidar](#2d-lidar)
     - [3D Lidar](#3d-lidar)
   - [IMU](#imu)
     - [Sensor Fusion with EKF](#sensor-fusion-with-ekf)
6. [Perception](#perception)
   - [OpenCV](#opencv)
   - [YOLOv8](#yolov8)

# Introduction

## Getting Started

The starting package for this session is very similar to where we left off in the previous lesson. However, every session comes with its own starter package containing the required files and project structure.

Clone the starter branch into your ROS 2 workspace:

```bash
git clone -b initial https://github.com/sachinmandal3580-rgb/sensors_and_perception.git
```

After downloading the package,move the gazebo models folder to your home directory, then rebuild your workspace and source it so ROS 2 can discover the newly added packages:

```bash
colcon build
source install/setup.bash
```

### Test the Starter Package

Before adding sensors and perception capabilities, let's verify that everything works correctly.

Launch the robot simulation:

```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

In a separate terminal, start keyboard teleoperation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

If the robot spawns successfully and responds to your keyboard commands, you're ready to begin building its sensing and perception stack.

## Skid-Steer

> **In short:** Skid-steer is the drive system used by tank-like robots. Wheels on the same side always spin together, and the robot turns by spinning the left and right sides at different speeds (or opposite directions for a sharp turn) — there's no separate steering mechanism. Here we convert the starter robot from a wobbly single caster-wheel design into a stable 4-wheel skid-steer robot.

**Goal:** Convert the starter caster-wheel robot into a 4-wheel skid-steer differential-drive robot.

You're given with the caster wheel bot:

![alt text][image1]

To convert it to a skid-steer 4-wheeled diff-drive bot, we need to give it 4 wheels — 2 on the left and 2 on the right. Currently there are 2 wheels placed close to the center (as with a caster wheel, for stability), but since we now have four wheels, we'll move them slightly away from center.

First, let's replace the caster wheel with the front-left wheel.

**Step 1 — Remove the caster wheel** from `erc_bot.urdf`:

```xml
   <joint type="fixed" name="front_caster_wheel_joint">
    <origin xyz="0.11 0 -0.05" rpy="0 0 0"/>
    <child link="front_caster_wheel"/>
    <parent link="base_link"/>
   </joint>

   <link name='front_caster_wheel'>
     <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
          ixx="0.002" ixy="0" ixz="0"
          iyy="0.002" iyz="0"
          izz="0.002"
      />
    </inertial>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/> 
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>

    <visual name='front_caster_wheel_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
        <material name="white"/>
    </visual>
  </link>
```

**Step 2 — Add the front-left wheel.**

Attach the wheel to the robot's base:
```xml
  <joint type="continuous" name="front_left_wheel_joint">
    <origin xyz="0.15 0.15 0" rpy="0 0 0"/>
    <child link="front_left_wheel"/>
    <parent link="base_link"/>
    <axis xyz="0 1 0" rpy="0 0 0"/>
    <limit effort="100" velocity="10"/>
    <dynamics damping="1.0" friction="1.0"/>
  </joint>
```

Define mass and inertial properties:
```xml
  <link name='front_left_wheel'>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <inertia
          ixx="0.014" ixy="0" ixz="0"
          iyy="0.014" iyz="0"
          izz="0.025"
      />
    </inertial>
```

Define the collision model:
```xml
    <collision>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707"/> 
      <geometry>
        <cylinder radius=".1" length=".05"/>
      </geometry>
    </collision>
```

Define the visual model:
```xml
   <visual name='front_left_wheel_visual'>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <geometry>
        <cylinder radius=".1" length=".05"/>
      </geometry>
       <material name="green"/>
    </visual>
  </link>
```

Do the same for the right wheel, but don't forget to change the origin (it should mirror the front-left wheel — only one coordinate needs to flip).

**Step 3 — Test the model in RViz:**
```bash
ros2 launch erc_gazebo_sensors check_urdf.launch.py
```

![alt text][image2]

You'll notice the rear wheels are closer to the base center in RViz — mirror the front wheels for both rear wheels to fix this.

**Step 4 — Update the Diff-Drive plugin** in `erc_bot.gazebo` to include all four wheel joints:

```xml
<?xml version="1.0"?>
<robot>
  <gazebo>
    <plugin
        filename="gz-sim-diff-drive-system"
        name="gz::sim::systems::DiffDrive">
        <!-- Topic for the command input -->
        <topic>/cmd_vel</topic>

        <!-- Wheel joints -->
        <left_joint>rear_left_wheel_joint</left_joint>
        <right_joint>rear_right_wheel_joint</right_joint>
        <left_joint>front_left_wheel_joint</left_joint>
        <right_joint>front_right_wheel_joint</right_joint>

        <!-- Wheel parameters -->
        <wheel_separation>0.3</wheel_separation>
        <wheel_radius>0.1</wheel_radius> 

        <!-- Control gains and limits (optional) -->
        <max_velocity>3.0</max_velocity> 
        <max_linear_acceleration>1</max_linear_acceleration>
        <min_linear_acceleration>-1</min_linear_acceleration>
        <max_angular_acceleration>2</max_angular_acceleration>
        <min_angular_acceleration>-2</min_angular_acceleration>
        <max_linear_velocity>0.5</max_linear_velocity>
        <min_linear_velocity>-0.5</min_linear_velocity>
        <max_angular_velocity>1</max_angular_velocity>
        <min_angular_velocity>-1</min_angular_velocity>
        
        <!-- Other parameters (optional) -->
        <odom_topic>odom</odom_topic> 
        <tf_topic>tf</tf_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_footprint</child_frame_id>
        <odom_publish_frequency>30</odom_publish_frequency>
    </plugin>

    <plugin
        filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher">
        <topic>joint_states</topic>
        <joint_name>rear_left_wheel_joint</joint_name>
        <joint_name>rear_right_wheel_joint</joint_name>
        <joint_name>front_left_wheel_joint</joint_name>
        <joint_name>front_right_wheel_joint</joint_name>

    </plugin>
  </gazebo>
  
</robot>
```

**Step 5 — Build, source, and launch:**

```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

![alt text][image3]

Drive it with teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> You can also test bot motion directly from the Gazebo teleop GUI: open the three-dot menu → type "teleop" → set wheel speed and yaw → move around.

---

## Friction

> **In short:** Friction settings control how much the wheels grip the floor versus slide. Get this wrong and the robot either won't move (too much friction/resistance) or slides around unrealistically (too little) — tuning it makes the simulated physics behave like a real robot.

Friction parameters control how the wheels grip the ground and how the base slides.

For all 4 wheels:
```xml
  <gazebo reference="front_left_wheel">
    <mu1>1.5</mu1>
    <mu2>0.7</mu2>
    <kp>200000.0</kp>
    <kd>5000.0</kd>
    <minDepth>0.002</minDepth>
    <maxVel>0.3</maxVel>
    <fdir1>0 1 0</fdir1>
  </gazebo>
```

For the base link:
```xml
    <gazebo reference="base_link">
    <mu1>0.000002</mu1>
    <mu2>0.000002</mu2>
  </gazebo>
```

---

## Odometry

> **In short:** Odometry estimates the robot's position and path over time by tracking wheel rotations. It's useful but drifts with error (e.g. wheel slip), which is why later we fuse it with IMU data via the EKF.

Let's view the robot's trajectory using the `trajectory_server` package.

**Terminal 1:**
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

**Terminal 2:**
```bash
ros2 run trajectory_server trajectory_server
```

Move around with teleop and watch the path drawn in green:

![alt text][image4]

---

## Sensors

> **In short:** This section covers how to add the four core sensor types — camera, RGBD camera, lidar, and IMU — to a simulated robot. Every sensor needs two pieces: a URDF entry (where it's mounted) and a Gazebo plugin entry (how it behaves/what it publishes).

To add any sensor — camera, lidar, IMU, etc. — two files need to be edited:

1. **`erc_bot.urdf`** — defines the sensor's position, orientation, and physical (link/joint) properties.
2. **`erc_bot.gazebo`** — defines the simulated sensor's real-world behavior (resolution, FOV, noise, etc.).

### Camera

> **In short:** Adds a basic RGB camera to the robot so it can "see" in color. We also bridge the image stream from Gazebo into ROS, and compress it (JPEG) so it doesn't hog bandwidth — important for wireless robots.

Attach the camera to the front of the robot's base:
```xml
  <joint type="fixed" name="camera_joint">
    <origin xyz="0.225 0 0.075" rpy="0 0 0"/>
    <child link="camera_link"/>
    <parent link="base_link"/>
    <axis xyz="0 1 0" />
  </joint>
```

Define mass and inertial properties:
```xml
  <link name='camera_link'>
    <pose>0 0 0 0 0 0</pose>
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>
```

Add a collision model (a 3 cm cube box):
```xml
    <collision name='collision'>
      <origin xyz="0 0 0" rpy="0 0 0"/> 
      <geometry>
        <box size=".03 .03 .03"/>
      </geometry>
    </collision>
```

Add a visual model:
```xml
    <visual name='camera_link_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size=".03 .03 .03"/>
      </geometry>
    </visual>
  </link>
```

Give the camera a Gazebo color:
```xml
  <gazebo reference="camera_link">
    <material>Gazebo/Red</material>
  </gazebo>
```

ROS robot coordinate frames differ from what computer-vision algorithms expect, so we define a second link, `camera_link_optical`: `camera_link` says where the camera is mounted, `camera_link_optical` says how the camera sees the world.

```xml
  <joint type="fixed" name="camera_optical_joint">
    <origin xyz="0 0 0" rpy="-1.5707 0 -1.5707"/>
    <child link="camera_link_optical"/>
    <parent link="camera_link"/>
  </joint>

  <link name="camera_link_optical">
  </link>
```

Now add the simulated camera sensor properties to `erc_bot.gazebo`:
```xml
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>15</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <!-- Noise is sampled independently per pixel on each frame.
               That pixel's noise value is added to each of its color
               channels, which at that point lie in the range [0,1]. -->
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
        <optical_frame_id>camera_link_optical</optical_frame_id>
        <camera_info_topic>camera/camera_info</camera_info_topic>
      </camera>
      <always_on>1</always_on>
      <update_rate>20</update_rate>
      <visualize>true</visualize>
      <topic>camera/image</topic>
    </sensor>
  </gazebo>
```
> Keep this snippet between the `<robot>` tags.

**Key parameters explained:**
- `<gazebo reference="camera_link">` — refers to the `camera_link` defined in the URDF
- `<horizontal_fov>` — field of view of the simulated camera
- `width`, `height`, `format`, `update_rate` — video stream properties
- `<optical_frame_id>` — ensures correct static transforms between coordinate systems (must be `camera_link_optical`)
- `<camera_info_topic>` — required by tools like RViz; must share the same prefix as the image topic
- `<topic>` — the camera's image topic name

Gazebo publishes the camera feed, but ROS can't read it directly — the two need to be **bridged**. This is handled by `gz_bridge_node` in `spawn_robot.launch.py`. Let's extend it to forward the image topics:

```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",

        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

Rebuild and launch:
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

![alt text][image5]

> If you don't see the camera window in RViz, go to Add → By topic → image.

You can also view it via `rqt`:

![alt text][image6]

Type `rqt` in a terminal → Plugins → Visualization → Image View (refresh if needed).

#### Image Transport

> **In short:** Raw uncompressed video is too heavy for wireless robots (~20 MB/s). This subsection compresses the camera stream to JPEG using `image_bridge`, then fixes the resulting RViz `camera_info` topic-mismatch warning with a relay node.

Both `/camera/camera_info` and `/camera/image` are now forwarded, but raw, uncompressed video at 640x480 consumes ~20 MB/s — too much for a wireless mobile robot. ROS's image transport plugins can compress the stream automatically, but this doesn't work with `parameter_bridge`. Instead, we use the dedicated `image_bridge` node from `ros_gz_image`.

Comment out the raw image topic in `gz_bridge_node` (we'll forward only the compressed image separately):
```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",

        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Node to bridge camera image with image_transport and compressed_image_transport
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'camera.image.compressed.jpeg_quality': 75},
        ],
    )
```

Add the new node to the launch description:
```python
launchDescriptionObject.add_action(gz_image_bridge_node)
```

After rebuilding, `rqt` shows a big bandwidth improvement thanks to JPEG compression:

![alt text][image7]

> If compressed images aren't visible in `rqt`, install the relevant transport plugin(s):
> - `sudo apt install ros-jazzy-compressed-image-transport` — JPEG / PNG
> - `sudo apt install ros-jazzy-theora-image-transport` — Theora
> - `sudo apt install ros-jazzy-zstd-image-transport` — Zstd

There's still an issue: RViz shows a warning for the compressed stream:
```
Camera Info
Expecting Camera Info on topic [/camera/image/camera_info]. No CameraInfo received. Topic may not exist.
```

RViz always expects `image` and `camera_info` topics to share the same prefix. This works for:

`/camera/image` → `/camera/camera_info`

but not for:

`/camera/image/compressed` → `/camera/image/camera_info`

because `camera_info` isn't published under that prefix. Remapping it directly would break the uncompressed stream, so instead we use the `relay` node from the `topic_tools` package to republish it:

```python
    # Relay node to republish /camera/camera_info to /camera/image/camera_info
    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

Add it to the launch description:
```python
launchDescriptionObject.add_action(relay_camera_info_node)
```

> If `topic_tools` isn't installed: `sudo apt install ros-jazzy-topic-tools`

Rebuild and test:
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```
![alt text][image8]

#### rqt_reconfigure

> **In short:** `rqt_reconfigure` is a GUI tool that lets you tweak sensor/node parameters (like JPEG quality) live, instead of editing code and rebuilding every time.

We set the JPEG quality manually via:
```python
'camera.image.compressed.jpeg_quality': 75
```
To discover other available parameters interactively, use `rqt_reconfigure`.

**Terminal 1:**
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

**Terminal 2:**
```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

![alt text][image9]

Adjust compression settings or algorithms here and monitor their effect live in `rqt`.

### RGBD Camera

> **In short:** An RGBD camera adds a depth (distance) channel on top of regular color, like a Kinect. It gives the robot both "what does this look like" and "how far away is it," letting it generate 3D point clouds and depth maps of its surroundings.

An RGBD camera adds depth perception on top of color. Replace the camera sensor definition in `erc_bot.gazebo` with:
```xml
  <gazebo reference="camera_link">
    <sensor name="rgbd_camera" type="rgbd_camera">
      <camera>
        <horizontal_fov>1.25</horizontal_fov>
        <image>
          <width>320</width>
          <height>240</height>
        </image>
        <clip>
          <near>0.3</near>
          <far>15</far>
        </clip>
        <optical_frame_id>camera_link_optical</optical_frame_id>
      </camera>
      <always_on>1</always_on>
      <update_rate>20</update_rate>
      <visualize>true</visualize>
      <topic>camera</topic>
      <gz_frame_id>camera_link</gz_frame_id>
    </sensor>
  </gazebo>
```

Two extra topics need forwarding:

- **`/camera/depth_image`** — a grayscale stream where pixel value = distance. RViz can render this together with the color image as a depth cloud.

![alt text][image12]

- **`/camera/points`** — a 3D point cloud (same type as the 3D lidar's cloud), visualized like any other point cloud.

```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

Rebuild and launch:
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

![alt text][image10]

The point cloud's orientation may look wrong because it's interpreted in the `camera_link_optical` frame — fix by changing it to: `<optical_frame_id>camera_link</optical_frame_id>`

![alt text][image11]

For a basic environment map, increase the **decay time** under the `Pointcloud2 camera` display option (~30 seconds) and move around:

![alt text][image18]

### Lidar

> **In short:** Lidar sends out laser beams and measures how long they take to bounce back, building a map of distances around the robot. 2D lidar scans a flat ring (used for basic obstacle avoidance/mapping); 3D lidar adds vertical scan layers for a full 3D point cloud of the environment.

Like the camera, lidar requires a URDF link/joint plus a Gazebo sensor plugin.

#### 2D Lidar

> **In short:** Scans 360° around the robot on a single horizontal plane, producing a `LaserScan` — great for cheap, fast obstacle detection and basic 2D mapping.

Add the link/joint to `erc_bot.urdf`:
```xml
  <joint type="fixed" name="scan_joint">
    <origin xyz="0.0 0 0.15" rpy="0 0 0"/>
    <child link="scan_link"/>
    <parent link="base_link"/>
    <axis xyz="0 1 0" rpy="0 0 0"/>
  </joint>

  <link name='scan_link'>
    <inertial>
      <mass value="1e-5"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>
    <collision name='collision'>
      <origin xyz="0 0 0" rpy="0 0 0"/> 
      <geometry>
        <box size=".06 .06 .06"/>
      </geometry>
    </collision>

    <visual name='scan_link_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
       <box size=".06 .06 .06"/>
      </geometry>
    </visual>
  </link>
```

Add the sensor plugin to `erc_bot.gazebo`:
```xml
  <gazebo reference="scan_link">
    <sensor name="gpu_lidar" type="gpu_lidar">
      <update_rate>10</update_rate>
      <topic>scan</topic>
      <gz_frame_id>scan_link</gz_frame_id>
      <lidar>
        <scan>
          <horizontal>
            <samples>720</samples>
            <!--(max_angle-min_angle)/samples * resolution -->
            <resolution>1</resolution>
            <min_angle>-3.14156</min_angle>
            <max_angle>3.14156</max_angle>
          </horizontal>
          <!-- Dirty hack for fake lidar detections with ogre 1 rendering in VM -->
        </scan>
        <range>
          <min>0.05</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
        </noise>
        <frame_id>scan_link</frame_id>
      </lidar>
      <always_on>1</always_on>
      <visualize>true</visualize>
    </sensor>
  </gazebo>
```

Update `parameter_bridge` to forward the scan topic:
```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

Test it:
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

![alt text][image13]

The red laser-scan points appear in RViz. You can also verify rendering inside Gazebo (since `visualize` is `true`) with the **Visualize Lidar** tool:

![alt text][image14]

Increase decay time here too for a simple map of the surroundings:

![alt text][image15]

#### 3D Lidar

> **In short:** Same idea as 2D lidar, but adds multiple vertical scan layers (a `<vertical>` block with samples/angle range), producing a full 3D point cloud instead of a flat scan — useful for 3D obstacle/terrain mapping.

To simulate a 3D lidar, add vertical samples (with min/max angles) alongside the horizontal scan parameters in `erc_bot.gazebo`:

```xml
          <vertical>
              <samples>32</samples>
              <min_angle>-0.5353</min_angle>
              <max_angle>0.1862</max_angle>
          </vertical>
```

Forward one more topic for the 3D point cloud:
```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

Launch:
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

![alt text][image16]

Increase decay time for a 3D map of the surroundings (same approach as the RGBD camera and 2D lidar):

![alt text][image17]

### IMU

> **In short:** An IMU measures the robot's own motion — acceleration, rotation rate, and sometimes heading — without looking at the outside world at all. It's the robot's "inner ear," and it's especially useful for sensing things wheels can't, like slipping or tipping.

An Inertial Measurement Unit (IMU) typically combines a 3-axis accelerometer, 3-axis gyroscope, and sometimes a 3-axis magnetometer — measuring linear acceleration, angular velocity, and (optionally) magnetic heading.

Add a simple link and fixed joint at the center of the base link, in the `urdf`:
```xml
  <joint name="imu_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="base_link"/>
    <child link="imu_link" />
  </joint>

  <link name="imu_link">
  </link>
```

Add the sensor plugin in `erc_bot.gazebo`:
```xml
  <gazebo reference="imu_link">
    <sensor name="imu" type="imu">
      <always_on>1</always_on>
      <update_rate>50</update_rate>
      <visualize>true</visualize>
      <topic>imu</topic>
      <enable_metrics>true</enable_metrics>
      <gz_frame_id>imu_link</gz_frame_id>
    </sensor>
  </gazebo>
```

The simulated world also needs the IMU system plugin. Add this inside the `<world>` tag of `home.sdf`:
```xml
    <plugin
      filename="gz-sim-imu-system"
      name="gz::sim::systems::Imu">
    </plugin>
```

Bridge the `imu` topic, rebuild, and test:
```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

**Terminal 1:**
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

**Terminal 2:**
```bash
rqt
```

Go to Plugins → Topics → Topic Monitor, check IMU, and move the bot to watch angular velocity, linear acceleration, and orientation change in real time.

![alt text][image20]

#### Sensor Fusion with EKF

> **In short:** No single sensor is perfectly reliable — wheel odometry drifts (e.g. wheel slip), IMU drifts too. The Extended Kalman Filter (EKF) blends multiple noisy sensor readings (odometry + IMU) into one smarter, more accurate position estimate. This is "sensor fusion."

**Sensor fusion** combines data from multiple sensors to get a more accurate, complete picture than any single sensor alone.

A **Kalman Filter (KF)** estimates a system's internal state (position, velocity, etc.) from noisy measurements using a predictive model, assuming linear dynamics. Real-world systems — especially involving orientation or IMU/odometry/GPS fusion — are often non-linear. The **Extended Kalman Filter (EKF)** handles this by locally linearizing the non-linear models. (We'll cover EKF theory in depth during the localization session — here we just see the filtered odometry output.)

**Step 1 — Create a config directory** inside `erc_gazebo_sensors`:
```bash
mkdir config
```

Add it to `CMakeLists.txt`:
```txt
cmake_minimum_required(VERSION 3.8)
project(erc_gazebo_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  config
  launch
  worlds
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

**Step 2 — Create `ekf.yaml`:**
```bash
touch ekf.yaml
```

**Step 3 — Install `robot_localization`:**
```bash
sudo apt install ros-jazzy-robot-localization
```

**Step 4 — Configure the EKF** in `ekf.yaml`:
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    publish_acceleration: false
    publish_tf: true


    map_frame: map
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom

    odom0: odom
    odom0_config: [false, false, false,
                  false, false, false,
                  true, true, false,
                  false, false, true,
                  false, false, false]

    imu0: imu
    imu0_config: [false, false, false,
                  true, true, true,
                  false, false, false,
                  false, false, true,
                  true, false, false]

    imu0_differential: false

    process_noise_covariance: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.015]

    initial_estimate_covariance: [1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9]
```

**Step 5 — Stop bridging raw `tf`**, since the EKF node will publish the filtered `tf` instead:
```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            #"/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

**Step 6 — Add the EKF node** and two trajectory servers (one for raw `odom`, one for `/odometry/filtered`):

```python
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_erc_gazebo_sensors, 'config', 'ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
             ]
    )
```

```python
    trajectory_odom_topic_node = Node(
        package='trajectory_server',
        executable='trajectory_server',
        name='trajectory_server_odom_topic',
        parameters=[{'trajectory_topic': 'trajectory_raw'},
                    {'odometry_topic': 'odom'}]
    )
```

```python
    trajectory_filtered_topic_node = Node(
    package='trajectory_server',
    executable='trajectory_server',
    name='trajectory_server_filtered',
    parameters=[
        {'trajectory_topic': 'trajectory'},
        {'odometry_topic': '/odometry/filtered'}
    ]
)
```

Add all new nodes to the launch description:
```python
launchDescriptionObject.add_action(ekf_node)

launchDescriptionObject.add_action(trajectory_odom_topic_node)

launchDescriptionObject.add_action(trajectory_filtered_topic_node)
```

Rebuild and run:
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

![alt text][image19]

**What you'll observe:** The yellow (raw) odometry trajectory drifts away from the corrected one quickly, especially in tricky situations — e.g. driving on a curve and hitting a wall, causing wheel slip. The raw odometry (trusting wheel encoders) thinks the robot keeps turning; the EKF-fused odometry correctly infers the robot's orientation isn't actually changing, since neither sensor alone can distinguish "moving uniformly" from "stuck."

---

## Perception

> **In short:** Perception is about making sense of sensor data — not just collecting it. Here we use vision algorithms (OpenCV, YOLOv8) to turn raw camera pixels into useful understanding: "where's the ball?" or "what objects are in this scene?"

### OpenCV

> **In short:** OpenCV is a classic computer-vision library. Here it's used for color-based detection: isolate red pixels, find the largest red blob (the ball), compute its center, and steer the robot toward it — a simple but effective "ball-chasing" behavior using thresholding and contour detection.

[OpenCV](https://opencv.org/) is an open-source computer vision library with hundreds of ready-to-use algorithms. Here we use it to make the robot chase a red ball.

**Step 1 — Create a package** for the Python nodes:
```bash
ros2 pkg create --build-type ament_python erc_gazebo_sensors_py
```

**Step 2 — Create the node:**
```bash
touch chase_the_ball.py

chmod +x chase_the_ball.py
```

Register it in `setup.py`:
```bash
entry_points={
     'console_scripts': [
         'chase_the_ball = erc_gazebo_sensors_py.chase_the_ball:main'
     ],
 },
```

**Step 3 — Install OpenCV:**
```bash
pip --version
```
Make sure `pip` is tied to your `python3.xx` (use `pip3` if needed).

```bash
pip install opencv-contrib-python
#or
pip install opencv-python
```

Test the install:
```bash
python3
```
```python
import cv2 as cv
print(cv.__version__)
```

> If `cv2` conflicts with `numpy`, downgrade numpy (the newest version breaks `cv_bridge`):
> ```bash
> pip install "numpy<2"
> ```

**Step 4 — Build the base subscriber node.** It subscribes to `/camera/image`, converts to an OpenCV frame, and displays it:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import threading

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Create a subscriber with a queue size of 1 to only keep the last frame
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            1  # Queue size of 1
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Variable to store the latest frame
        self.latest_frame = None       

        # Flag to control the display loop
        self.running = True 

    def image_callback(self, msg):
        """Callback function to receive and store the latest frame."""
        # Convert ROS Image message to OpenCV format and store it
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def display_image(self):
        """Main loop to process and display the latest frame."""
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while rclpy.ok():
            # Check if there is a new frame available
            if self.latest_frame is not None:

                # Process the current image
                self.process_image(self.latest_frame)

                # Show the latest frame
                cv2.imshow("frame", self.latest_frame)
                self.latest_frame = None  # Clear the frame after displaying

            # Check for quit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

            rclpy.spin_once(self, timeout_sec=0.05)

        # Close OpenCV window after quitting
        cv2.destroyAllWindows()
        self.running = False

def main(args=None):

    print("OpenCV version: %s" % cv2.__version__)

    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        node.display_image()  # Run the display loop
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Build, source, then run:
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```
```bash
ros2 run erc_gazebo_sensors_py chase_the_ball
```

![alt text][image27]

You should see the live camera feed in an OpenCV window.

**Step 5 — Process the image.** Next we add `process_image()` to produce three more views: a binary mask, an outlined contour, and a tracking crosshair. Since heavier processing could block the spin loop that feeds `image_callback()`, we move spinning to its own thread.

Update `__init__()`:
```python
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Create a subscriber with a queue size of 1 to only keep the last frame
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            1  # Queue size of 1
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Variable to store the latest frame
        self.latest_frame = None
        self.frame_lock = threading.Lock()  # Lock to ensure thread safety
        
        # Flag to control the display loop
        self.running = True

        # Start a separate thread for spinning (to ensure image_callback keeps receiving new frames)
        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()
```

Add `spin_thread_func()` and make `image_callback()` thread-safe:
```python
    def spin_thread_func(self):
        """Separate thread function for rclpy spinning."""
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def image_callback(self, msg):
        """Callback function to receive and store the latest frame."""
        # Convert ROS Image message to OpenCV format and store it
        with self.frame_lock:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
```

`display_image()` no longer needs `rclpy.spin_once(self, timeout_sec=0.05)` directly.

Add a `stop()` method to join the thread cleanly:
```python
    def stop(self):
        """Stop the node and the spin thread."""
        self.running = False
        self.spin_thread.join()
```

Call it from `main()`:
```python
def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        node.display_image()  # Run the display loop
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()  # Ensure the spin thread and node stop properly
        node.destroy_node()
        rclpy.shutdown()
```

Now add `process_image()` (after `display_image`) — this detects the red ball via color thresholding and contour detection:
```python
    def process_image(self, img):
        """Image processing task."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        rows,cols = img.shape[:2]

        R,G,B = self.convert2rgb(img)

        redMask = self.threshold_binary(R, (220, 255))
        stackedMask = np.dstack((redMask, redMask, redMask))
        contourMask = stackedMask.copy()
        crosshairMask = stackedMask.copy()

        # return value of findContours depends on OpenCV version
        (contours, hierarchy) = cv2.findContours(redMask.copy(), 1, cv2.CHAIN_APPROX_NONE)

        # Find the biggest contour (if detected)
        if len(contours) > 0:
            
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)

            # Make sure that "m00" won't cause ZeroDivisionError: float division by zero
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0

            # Show contour and centroid
            cv2.drawContours(contourMask, contours, -1, (0,255,0), 10)
            cv2.circle(contourMask, (cx, cy), 5, (0, 255, 0), -1)

            # Show crosshair and difference from middle point
            cv2.line(crosshairMask,(cx,0),(cx,rows),(0,0,255),10)
            cv2.line(crosshairMask,(0,cy),(cols,cy),(0,0,255),10)
            cv2.line(crosshairMask,(int(cols/2),0),(int(cols/2),rows),(255,0,0),10)

        # Return processed frames
        return redMask, contourMask, crosshairMask

    # Convert to RGB channels
    def convert2rgb(self, img):
        R = img[:, :, 2]
        G = img[:, :, 1]
        B = img[:, :, 0]

        return R, G, B

    # Apply threshold and result a binary image
    def threshold_binary(self, img, thresh=(200, 255)):
        binary = np.zeros_like(img)
        binary[(img >= thresh[0]) & (img <= thresh[1])] = 1

        return binary*255
```
And update the `display image` func to show those windows:
```bash
    def display_image(self):
        """Main loop to process and display the latest frame."""
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while rclpy.ok():
            # Check if there is a new frame available
            if self.latest_frame is not None:

                # Process the current image
                mask, contour, crosshair = self.process_image(self.latest_frame)

                # Show the latest frame
                cv2.imshow("frame", self.latest_frame)
                cv2.imshow("mask", mask)
                cv2.imshow("contour", contour)
                cv2.imshow("crosshair", crosshair)
                self.latest_frame = None  # Clear the frame after displaying

            # Check for quit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        # Close OpenCV window after quitting
        cv2.destroyAllWindows()
        self.running = False
```
This opens 4 OpenCV windows and tries to find the red ball. Spawn a red ball in the simulation using the Gazebo Resource Spawner plugin first:

![alt text][image21]

The new windows look like this:

![alt text][image22]

**Step 6 — Combine the views.** Managing 4 separate windows is awkward, so let's overlay the mask/contour/crosshair views onto the main camera frame.

Add `add_small_pictures()`:
```python
    # Add small images to the top row of the main image
    def add_small_pictures(self, img, small_images, size=(160, 120)):

        x_base_offset = 40
        y_base_offset = 10

        x_offset = x_base_offset
        y_offset = y_base_offset

        for small in small_images:
            small = cv2.resize(small, size)
            if len(small.shape) == 2:
                small = np.dstack((small, small, small))

            img[y_offset: y_offset + size[1], x_offset: x_offset + size[0]] = small

            x_offset += size[0] + x_base_offset

        return img
```

Use it in `display_image()` and show only the combined `result` window:
```python
    def display_image(self):
        """Main loop to process and display the latest frame."""
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while rclpy.ok():
            # Check if there is a new frame available
            if self.latest_frame is not None:

                # Process the current image
                mask, contour, crosshair = self.process_image(self.latest_frame)

                # Add processed images as small images on top of main image
                result = self.add_small_pictures(self.latest_frame, [mask, contour, crosshair])

                # Show the latest frame
                cv2.imshow("frame", result)
                self.latest_frame = None  # Clear the frame after displaying

            # Check for quit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        # Close OpenCV window after quitting
        cv2.destroyAllWindows()
        self.running = False
```

**Step 7 — Drive toward the ball.** Add this to `process_image()`, just before the `return`:
```python
...
            # Chase the ball
            if abs(cols/2 - cx) > 20:
                msg.linear.x = 0.0
                if cols/2 > cx:
                    msg.angular.z = 0.2
                else:
                    msg.angular.z = -0.2

            else:
                msg.linear.x = 0.2
                msg.angular.z = 0.0

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        # Publish cmd_vel
        self.publisher.publish(msg)
...
```

The robot now follows the red ball in the Gazebo simulation:

![alt text][image24]

### YOLOv8

> **In short:** YOLOv8 ("You Only Look Once") is a deep-learning object detector that recognizes many different object classes (people, fire hydrants, etc.) in real time, drawing boxes and confidence scores around them — far more general than the color-based OpenCV approach above, since it understands actual object shapes/features instead of just color.

[YOLOv8](https://github.com/ultralytics/ultralytics) brings real-time, multi-class object detection on top of the camera feed.

**Step 1 — Create the node** `yolo_detection_node.py` in `erc_gazebo_sensors_py` (same way as `chase_the_ball.py`), and register it in `setup.py`.

**Step 2 — Install Ultralytics:**
```bash
pip install ultralytics
```

**Step 3 — Full node code** (subscribes to the camera, runs YOLOv8s, draws bounding boxes/labels, and shows a live detection dashboard with FPS and object count):

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import threading
import time

from ultralytics import YOLO


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        # Better model than yolov8n
        self.model = YOLO("yolov8s.pt")

        self.get_logger().info("YOLO model loaded")

        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            1
        )

        self.bridge = CvBridge()

        self.latest_frame = None
        self.frame_lock = threading.Lock()

        self.running = True

        self.spin_thread = threading.Thread(
            target=self.spin_thread_func,
            daemon=True
        )
        self.spin_thread.start()

        self.prev_time = time.time()

    def spin_thread_func(self):

        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        with self.frame_lock:
            self.latest_frame = frame

    def stop(self):

        self.running = False

        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1)

    def display_image(self):

        cv2.namedWindow(
            "YOLO Detection",
            cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO
        )

        cv2.resizeWindow("YOLO Detection", 1600, 900)

        while rclpy.ok() and self.running:

            with self.frame_lock:
                frame = None if self.latest_frame is None else self.latest_frame.copy()

            if frame is not None:

                result = self.run_yolo(frame)

                cv2.imshow("YOLO Detection", result)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:
                self.running = False
                break

        cv2.destroyAllWindows()

    def run_yolo(self, frame):

        CONF_THRESHOLD = 0.35
        results = self.model(
            frame,
            conf=CONF_THRESHOLD,
            imgsz=640,
            verbose=False
        )

        detections = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                class_name = self.model.names[class_id]
                detections.append(
                    f"{class_name} ({confidence:.2f})"
                )
                color = self.class_color(class_id)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2
                )
                label = f"{class_name} {confidence:.2f}"

                (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
                )
                text_y = max(y1 - 10, th + 10)

                cv2.rectangle(frame, (x1, text_y - th - baseline), (x1 + tw + 10, text_y + baseline), color, -1
                )
                cv2.putText(frame, label, (x1 + 5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
                )
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                cv2.circle(frame, (cx, cy), 5, color, -1
                )
                
        current_time = time.time()
        fps = 1.0 / max(current_time - self.prev_time, 1e-6)
        self.prev_time = current_time
        dashboard_width = 350
        dashboard = np.zeros(
            (frame.shape[0], dashboard_width, 3),
            dtype=np.uint8
        )

        cv2.putText(
            dashboard, "Detections", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2
        )

        cv2.putText(dashboard,f"FPS : {fps:.1f}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )

        cv2.putText(dashboard, f"Objects : {len(detections)}", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )

        y = 170

        for det in detections[:25]:

            cv2.putText(dashboard, det, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1
            )

            y += 30

        combined = np.hstack((frame, dashboard))

        return combined

    def class_color(self, class_id):

        np.random.seed(class_id)

        return tuple(
            int(c)
            for c in np.random.randint(100, 255, 3)
        )


def main(args=None):

    print("OpenCV Version:", cv2.__version__)

    rclpy.init(args=args)

    node = YoloDetectorNode()

    try:
        node.display_image()

    except KeyboardInterrupt:
        pass

    finally:

        node.stop()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Now add different Gazebo models with the Resource Spawner and watch YOLOv8 detect them live (shown below for a fire hydrant and a person):

![alt text][image25]

![alt text][image26]

# Conclusion

A robot's journey toward autonomy begins with perception.

Today, we gave our robot eyes through cameras, depth perception through RGBD sensing, environmental awareness through lidar, motion awareness through an IMU, and a more reliable estimate of reality through sensor fusion.

We even taught it to recognize objects and react to what it sees.

The robot can now observe the world.

The next challenge is helping it understand where it is within that world and how to navigate through it.

Because seeing is only the first step.

Knowing where you are is what truly unlocks autonomy.

## Till then Stay Tuned !
