## Setup

Our project is within the `unitree_ws` folder. All installation commands will use this path.

### Ros Unitree
Install from [this repo](https://github.com/macc-n/ros_unitree) for ROS Noetic. Follow the basic instructions from there.

### RealSense D435 (top and down cameras)

Since the Aliengo robot has two RealSense D435 cameras at the front, one pointing to the front and the other to down, we will use the following two repos: 
- https://github.com/m-tartari/realsense_gazebo_plugin
- https://github.com/issaiass/realsense2_description
Both shoul be cloned in your workspace:
```
cd ~/unitree_ws/src
git clone https://github.com/issaiass/realsense2_description
git clone https://github.com/issaiass/realsense_gazebo_plugin
```
and compile with `catkin b`.

### Gazebo worls for testing

You can desing your own environments with your favorite software. In my case I just need it to be like a solid maze for locomotion testing, for this reason I can export it as a unique `.stl` file. If you need more complex things like a door that needs to be open, etc. your will need to work a little bit more. 

#### How to import `.stl` files to Gazebo?
The idea is to import the `.stl` file in any possible way and then export the world to later be launched directly from a launch file. Ok, step by step:

1) In your `home` folder look for the `.gazebo/models` folder. 
2) There, create a folder with your desired name (i.e., `maze_1`). And create the rest of the following files:
    ```
    home
    |- .gazebo/models
        |- meshes
        |  |- maze_1.stl
        |- model.config
        |- model.sdf
    ```
3) Edit the following files including the proper content.
    
    The content of the `model.config` file is the following:
    ``` xml
    <?xml version="1.0"?>
    <model>
    <name>Maze 1</name>
    <version>1.0</version>
    <sdf version="1.6">model.sdf</sdf>
    <author>
        <name>Your Name</name>
        <email>yourmail@mail.com</email>
    </author>
    <description>
        A meaningful description.
    </description>
    </model>
    ``` 
    And the content fo the `model.sdf` is this:
    ``` xml
    <?xml version="1.0"?>
    <sdf version="1.6">
    <model name="maze_1">
        <static>true</static>
        <link name="link">
        <visual name="visual">
            <geometry>
            <mesh>
                <uri>model://maze_1/meshes/maze_1.stl</uri>
                <scale>0.001 0.001 0.001</scale>
            </mesh>
            </geometry>
        </visual>
        <collision name="collision">
            <geometry>
            <mesh>
                <uri>model://maze_1/meshes/maze_1.stl</uri>
                <scale>0.001 0.001 0.001</scale>
            </mesh>
            </geometry>
        </collision>
        </link>
    </model>
    </sdf>
    ``` 
beware of the `<scale>` and `<static>` parameters, among others. This is a basic example.

#### Other environments
If you prefer to import other prebuild environments [this](https://github.com/macc-n/gazebo_worlds) is a good repository where to find them. We have selected two possible enviroments where to test locomotion:
- Easy: https://github.com/macc-n/gazebo_worlds/blob/master/gazebo/screenshots/office_cpr_construction.jpg
- Hard: https://github.com/macc-n/gazebo_worlds/blob/master/gazebo/screenshots/rubble.jpg

### Xacro

We have installed the Intel RealSense cameras and designed a custom Gazebo maze. Now, let’s integrate the cameras into the `urdf.xacro` and update the `gazebo.xacro` files accordingly.

In the `unitree_ros` package look for the aliengo `robot.xacro` description. There add the following lines:
```
<!-- ######## Camera Intel RealSense 435 (down) ######### -->
<xacro:property name="M_PI" value="3.1415926535897931" />
<xacro:include filename="$(find realsense2_description)/urdf/_d435.urdf.xacro"/>
<xacro:sensor_d435  parent="trunk" name="D435_camera_down" topics_ns="D435_camera_down">
    <origin rpy="0 ${M_PI/15} 0" xyz="0.312 0.013 -0.035"/>
</xacro:sensor_d435>
<!-- ######### Camera Intel RealSense 435 (top) ######### -->
<xacro:include filename="$(find realsense2_description)/urdf/_d435.urdf.xacro"/>
<xacro:sensor_d435  parent="trunk" name="D435_camera_top" topics_ns="D435_camera_top">
    <origin rpy="0 ${-M_PI/11} 0" xyz="0.325 0.013 0.024"/>
</xacro:sensor_d435>
<!-- #################################################### -->
```
Then in the `ros_unitree/unitree_ros/unitree_gazebo/launch` edit the `robot_simulation.launch` adding the mazes as worlds. Then, when spawning the robots modify the service as follows:
```
<!-- Set trunk and joint positions at startup -->
<node pkg="gazebo_ros" type="spawn_model" name="urdf_spawner" respawn="false" output="screen"
        args="-urdf -z 0.6 -x -0.2 -Y 1.57 -model $(arg rname)_gazebo -param robot_description -unpause"/>
```

In order to test the `urdf.xacro` and `gazebo.xacro` modifications for the Aliengo use the following launcher:
``` 
roslaunch unitree_gazebo robot_simulation.launch rname:=aliengo wname:=maze_1
``` 

### Elevation Mapping
Based on this repo: https://github.com/ANYbotics/elevation_mapping

1) Intall [Grid Map](https://github.com/anybotics/grid_map):
    ```
    sudo apt-get install ros-$ROS_DISTRO-grid-map
    ```
2) Install [`kindr`](https://github.com/anybotics/kindr) (kinematics and dynamics library for robotics):
    
    With `catkin`:
    ```
    cd ~/unitree_ws/src
    git clone git@github.com:anybotics/kindr.git
    catkin build -w ~/unitree_ws kindr
    ``` 
    `kindr` can be included in your catkin project with: 
    
    Add the following to your CMakeLists.txt:
    ```
    find_package(catkin COMPONENTS kindr)
    include_directories(${catkin_INCLUDE_DIRS})
    ``` 
    And to your package.xml:
    ```xml
    <package>
        <build_depend>kindr</build_depend>
    </package>
    ```
3) Clone in your ws and build the [`kindr_ros`](https://github.com/anybotics/kindr_ros) interface:
    ```
    git clone https://github.com/ANYbotics/kindr_ros.git
    catkin build kindr_ros
    ```
4) Install [Point Cloud Library](https://pointclouds.org/downloads/) (PCL):
    ```
    sudo apt install libpcl-dev
    ```
5) Install the [Eigen](http://eigen.tuxfamily.org/) library:
    ```
    git clone https://gitlab.com/libeigen/eigen.git
    ```
    Then compile using CMake:
    ```
    cd ~/unitree_ws/src/eigen/buil_dir
    cmake ..
    make install
    ```
    (*The "make install" step may require administrator privileges.*)

Now you are ready to use the `elevation-mapping`:
```
cd unitree_ws/src
git clone https://github.com/anybotics/elevation_mapping.git
cd ..
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
``` 


