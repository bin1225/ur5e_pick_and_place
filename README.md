## 개요

https://docs.google.com/document/d/1CtW869otya4qrUMVrVcnpTfO5YerwII2hoBX2MgaZgM/edit?tab=t.0

Ros2 기반 UR5 로봇 Pick and Place Simulation 환경 구축

## 실행 환경
* ROS2-jazzy
* gazebo-harmonic
* moveit2


### 실행
- 로봇 한대 
```sql
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py
```

```bash
	ros2 run move_test_cpp demo_controller.cpp 
```

- 다중 로봇
```bash
ros2 launch ur_simulation_gz dual_sim_moveit.launch.py
```

- gripper 제어
```bash
ros2 topic pub /forward_position_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.1]}"
```

### 패키지 구조
* ur_simulation_gz
	* worlds : gazebo world
	* ur_simulation_gz : gazebo launch, urdf 
* ur_moveit_config 
	* launch : moveit + rviz launch
	* config : moveit 관련 설정
	* srdf
* move_test_cpp : moveit 실행 및 로봇 제어 코드

### 실행 
<img width="432" height="480" alt="image" src="https://github.com/user-attachments/assets/654df866-174f-450d-a510-3b9536eafe94" />

### 현재 구현 완료
- 로봇 1대 사용한 Pick and Place 
- Rviz 기반 다중 로봇 Plan, execute 성공
- Action msg를 사용하여 gripper 제어

### 미완성
- moveit 이용한 다중로봇 제어
