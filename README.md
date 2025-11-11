Clone repo: git clone https://github.com/your-username/mowito-assignments.git && cd mowito-assignments

Install deps: sudo apt update && sudo apt install ros-humble-desktop libeigen3-dev cmake build-essential

Build Task 1: cd task1_ros2 && colcon build --packages-select ros_comms && source install/setup.bash

Build Tasks 3/4: cd ../task3_euler_quat && mkdir build && cd build && cmake .. && make (repeat for task4_fk)

Run demos: See below for each task.

Total setup: 10 mins. Ready for demos.

What I Did:-
Task 1 - ROS2 Communication
Built custom package with:

Publisher: Sends "Hello, ROS2!" to /chatter topic at 1Hz.

Subscriber: Echoes messages in real-time.

Service: AddTwoInts.srv - adds two ints (e.g., 5+3=8).

Run: ros2 run ros_comms publisher (new terminal for subscriber/server). Test service: ros2 service call /add_two_ints ros_comms/srv/AddTwoInts "{a: 5, b: 3}". Full launch: ros2 launch ros_comms basic.launch.py.
Handles reliable messaging for robotics (like sensor data).

Task 3 - Euler to Quaternion
Converts roll/pitch/yaw (degrees) to quaternions using ZYX order. Uses Eigen: AngleAxis method + matrix extraction for gimbal lock (e.g., pitch=90°).

Math: Rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll). Quat from trace(t) where qw = sqrt(t+1)/2, etc. Norm=1.

Run: cd task3_euler_quat/build && ./euler_to_quat
Sample (10/20/30°): w=0.9285 x=0.0750 y=0.1906 z=0.3192. Matches ROS tf2. Edit angles in main.cpp and rebuild.

Task 4 - Forward Kinematics
Computes arm end pose from joint angles using DH params. Sample: 3-DOF planar (links=1m each).

Math: DH matrix Ai with cos/sin theta, a, alpha, d. Total T = A1 * A2 * A3. Extract position from translation, quat from rotation submatrix.

Run: cd task4_fk/build && ./forward_kinematics
Sample joints [0.5,0.3,0.2] rad: x=1.237 y=1.559 z=0, w=0.8776 z=0.4794. Home (all 0): x=3 y=0.

Edit DH params/joints in main.cpp; rebuild with make. Extensible to 6DOF arms.

Structure
task1_ros2/: ROS2 package (src, srv, launch).

task3_euler_quat/: C++ Euler/Quat (main.cpp, CMakeLists.txt).

task4_fk/: C++ FK (main.cpp, CMakeLists.txt).

.gitignore: Ignores builds.

Verification
All build without errors. Outputs match math (quat norm=1, service sums correct). Edge cases: Gimbal lock, singular joints. Clean: rm -rf */build task1_ros2/{install,log}.



