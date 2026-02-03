#安装sdk
sudo apt install esptool
pip install pyserial
pip install aero-open-sdk --no-deps

#安装ros2包
sudo apt install libncursesw5-dev
sudo apt-get update
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
colcon build

#BASH查看串口拔插
while true; do clear; ls /dev/tty* 2>/dev/null; sleep 1; done

#启动通讯节点#修改端口"/dev/ttyACM0"
ros2 run aero_hand_open aero_hand_node
ros2 run aero_hand_open aero_hand_node --ros-args -p control_space:=actuator -p right_port:=/dev/ttyACM0
ros2 run aero_hand_open aero_hand_node --ros-args -p control_space:=joint -p right_port:=/dev/ttyACM0
ros2 run aero_hand_open aero_hand_node --ros-args -p left_port:=/dev/ttyACM0 -p right_port:=/dev/ttyACM1
#手动添加aero_hand_node.py端口
class AeroHandNode(Node):
    def __init__(self):
        super().__init__("aero_hand_node")
        ## Parameters
        self.declare_parameter("right_port", "/dev/ttyACM0")
        self.declare_parameter("left_port", "")
        self.declare_parameter("baudrate", 921600)
        self.declare_parameter("feedback_frequency", 100.0)
        #self.declare_parameter("control_space", "joint")
        self.declare_parameter("control_space", "actuator")

        
#发布位置zero
ros2 topic pub /right/joint_control aero_hand_open_msgs/JointControl "{header: {stamp: {sec: 0, nanosec: 0}}, target_positions: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]}" --once


#抓握1
ros2 topic pub /right/joint_control aero_hand_open_msgs/JointControl "{header: {stamp: {sec: 0, nanosec: 0}}, target_positions: [1.745,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]}" --once

#抓握2
ros2 topic pub /right/joint_control aero_hand_open_msgs/JointControl "{header: {stamp: {sec: 0, nanosec: 0}}, target_positions: [1.745,0.0,1.571,0.662,1.571,1.571,1.571,1.571,1.571,1.571,1.571,1.571,1.571,1.571,1.571,1.571]}" --once

#发布位置大拇指
ros2 topic pub /right/joint_control aero_hand_open_msgs/JointControl "{header: {stamp: {sec: 0, nanosec: 0}}, target_positions: [1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]}" --once

#ok
ros2 topic pub /right/joint_control aero_hand_open_msgs/JointControl "{header: {stamp: {sec: 0, nanosec: 0}}, target_positions: [1.217,0.415,1.356,0.0,1.571,1.477,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]}" --once

#中指
ros2 topic pub /right/joint_control aero_hand_open_msgs/JointControl "{header: {stamp: {sec: 0, nanosec: 0}}, target_positions: [1.745,0.488,1.571,0.0,1.571,1.571,1.571,0.0,0.0,0.0,1.571,1.571,1.571,1.571,1.571,1.571]}" --once

#rockit
ros2 topic pub /right/joint_control aero_hand_open_msgs/JointControl "{header: {stamp: {sec: 0, nanosec: 0}}, target_positions: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.571,1.571,1.571,1.571,1.571,1.571,0.0,0.0,0.0]}" --once

#rl切换actuator模式
#安装mujoco_playgroud and jax
ros2 run aero_hand_open aero_hand_node --ros-args -p control_space:=actuator -p right_port:=/dev/ttyACM0
ros2 run aero_hand_open_rl rl_z_rotation_deploy






