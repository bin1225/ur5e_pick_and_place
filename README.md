## 실행 환경
* ROS2-jazzy
* gazebo-harmonic

### 실행
```sql
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py
```

```sql
	ros2 run ur_robot_driver example_move.py
```

```bash
ros2 topic pub /forward_position_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.1]}"
```

```bash
ros2 run move_test_cpp move_test_cpp_node
```
