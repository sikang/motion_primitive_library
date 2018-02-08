# MRSL Motion Primitive Library for quadrotor v0.4
[![wercker status](https://app.wercker.com/status/ae575e9b5218e1f02684065445c6de66/s/master "wercker status")](https://app.wercker.com/project/byKey/ae575e9b5218e1f02684065445c6de66)
- - -
## New Feature
  - Add incremental trajectory planning
  - Fix many small bugs in previous version

## Compilation
#### Prerequisite:
  - `Boost`
  - [`PCL`](http://pointclouds.org/)
  - [`Eigen`](http://eigen.tuxfamily.org/index.php?title=Main_Page)

##### A) Simple cmake
```sh
mkdir build && cd build && cmake .. && make
```

##### B) Using Catkin (not recognizable by catkin\_make)
```sh
$ mv motion_primitive_library ~/catkin_ws/src
$ cd ~/catkin_ws & catkin_make_isolated -DCMAKE_BUILD_TYPE=Release
```

##### Include in other projects:
To link this lib properly, add following in the `CMakeLists.txt` 
```
find_package(motion_primitive_library REQUIRED)
include_directories(${MOTION_PRIMITIVE_LIBRARY_INCLUDE_DIRS})
...
add_executable(test_xxx src/test_xxx.cpp)
target_link_libraries(test_xxx ${MOTION_PRIMITIVE_LIBRARY_LIBRARIES})

```



## Example Usage
The simple API are provided in the base planner class, please look up in Doxygen. 

Before planning, the start and goal states must be set up accordingly as:
```c++
// Initialize planning mission 
Waypoint start, goal;
start.pos = Vec3f(2.5, -3.5, 0.0);
start.use_pos = true;
start.use_vel = true;
start.use_acc = false; 
start.use_jrk = false; 
goal.pos = Vec3f(35, 2.5, 0.0);
goal.use_pos = start.use_pos;
goal.use_vel = start.use_vel;
goal.use_acc = start.use_acc;
goal.use_jrk = start.use_jrk;
```

The flags `use_xxx` will adapt the planning in different control space. For example, the one above is control in acc space. Four options are provided by setting those flags as below:

Vel | Acc | Jrk | Snp
:-- | :-- | :-- | :--
`use_pos = true` | `use_pos = true` | `use_pos = true` | `use_pos = true`
`use_vel = false` | `use_vel = true` | `use_vel = true` | `use_vel = true`
`use_acc = false` | `use_acc = false` | `use_acc = true` | `use_acc = true`
`use_jrk = false` | `use_jrk = false` | `use_jrk = false` | `use_jrk = true`


After setting up start and goal states, a planning thread can be started as:
```
std::unique_ptr<MPMapUtil> planner(new MPMapUtil(true)); // Declare a mp planner using voxel map
planner->setMapUtil(map_util); // Set collision checking function
planner->setEpsilon(1.0); // Set greedy param (default equal to 1)
planner->setVmax(1.0); // Set max velocity
planner->setAmax(1.0); // Set max acceleration 
planner->setJmax(1.0); // Set max jerk
planner->setUmax(0.5); // Set max control input
planner->setDt(1.0); // Set dt for each primitive
planner->setW(10); // Set weight of time 
planner->setMaxNum(-1); // Set maximum allowed number of expanded nodes (-1 means no limitation)
planner->setU(1, false);// 2D discretization with 1
planner->setTol(0.2, 0.1, 1); // Tolerance for goal region (pos=0.2, vel=0.1, acc=1 accordingly)

bool valid = planner->plan(start, goal); // Plan from start to goal
```

## Test
#### Example1 (simple planning):
Run following command for test a 2D planning in a given map:
```sh
$ ./build/devel/lib/test_planner_2d ../data/corridor.yaml
```
or with ROS
```sh
$ rosrun motion_primitive_library test_planner_2d /PATH/TO/MPL/data/corridor.yaml
```
You should see following messages if it works properly:
```sh
[MPPlanner] PLANNER VERBOSE ON
[MPBaseUtil] set epsilon: 1.000000
[MPBaseUtil] set v_max: 1.000000
[MPBaseUtil] set a_max: 1.000000
[MPBaseUtil] set j_max: 1.000000
[MPBaseUtil] set u_max: 0.500000
[MPBaseUtil] set dt: 1.000000
[MPBaseUtil] set w: 10.000000
[MPBaseUtil] set max num: -1
[MPBaseUtil] set tol_dis: 0.200000
[MPBaseUtil] set tol_vel: 0.100000
[MPBaseUtil] set tol_acc: 1.000000
start pos: [2.500000, -3.500000, 0.000000], vel: [0.000000, 0.000000, 0.000000], acc: [0.000000, 0.000000, 0.000000]
goal pos: [37.000000, 2.500000, 0.000000], vel: [0.000000, 0.000000, 0.000000], acc: [0.000000, 0.000000, 0.000000]
[MPBaseUtil] set effort in acc
Start from new node!
goalNode fval: 373.000000, g: 373.000000!
Expand [6901] nodes!
Reached Goal !!!!!!

...

MP Planner takes: 183.000000 ms
MP Planner expanded states: 6901
Total time T: 37.000000
Total J:  J(0) = 40.250000, J(1) = 3.000000, J(2) = 0.000000, J(3) = 0.000000
```

The output image is stored as `output.svg` in the current folder (blue dots show the expended states, red circles indicate start and goal):
![Visualization](./data/example1.png). 

It is recommended to visualize the svg in web browser: for example, `firefox output.svg`.

#### Example2 (planning with a prior trajectory):
Run following command for test a 2D planning, it first finds a trajector in low dimensional space (acceleration-driven), then it uses the planned trajectory to refine for a trajectory in high dimensional space (jerk-driven):
```sh
$ ./build/devel/lib/test_planner_2d_prior_traj ../data/corridor.yaml
```
or with ROS
```sh
$ rosrun motion_primitive_library test_planner_2d_prior_traj /PATH/TO/MPL/data/corridor.yaml
```
The output image is stored as `output.svg` in the current folder (blue dots show the expended states, red circles indicate start and goal, the black curve is the prior trajectory):
![Visualization](./data/example2.png). 

It is recommended to visualize the svg in web browser: for example, `firefox output.svg`.




## Doxygen
For more details, please refer to [Doxygen](https://sikang.github.io/motion_primitive_library).

## ROS Wrapper
The interface with ROS for planning using this library can be found in [`mpl_ros`](https://github.com/sikang/mpl_ros.git).
