# 半醒灵巧手Ros2驱动

## 快速开始

### 1.克隆此仓库至Ros的工作区

```bash
git clone https://github.com/1326898804/BxiHandRos2.git
```

### 2.编译工作区

```bash
colcon build
source install/setup.bash
```

###3.启动控制节点

#### usb模式
```bash
ros2 run aero_hand_open aero_hand_node --ros-args -p left_port:="/dev/ttyACM0" -p right_port:="/dev/ttyACM1" -p bluetooth:=False
```
```python
Node(
    package="aero_hand_open",
    executable="aero_hand_node",
    name="aero_hand_open",
    output="screen",
    parameters=[
        {'left_port':'/dev/ttyACM0'},
        {'right_port':'/dev/ttyACM1'},
        {'bluetooth':False}
    ]
)
```
#### 蓝牙模式
在终端中执行
```bash
ros2 run aero_hand_open aero_hand_node --ros-args -p left_port:="10:B4:1D:E9:9A:2A" -p right_port:="90:70:69:12:73:26" -p bluetooth:=True
```
或在launch文件中
```python
Node(
    package="aero_hand_open",
    executable="aero_hand_node",
    name="aero_hand_open",
    output="screen",
    parameters=[
        {'left_port':'10:B4:1D:E9:9A:2A'},
        {'right_port':'90:70:69:12:73:26'},
        {'bluetooth':True}
    ]
)
```
接下来你应该可以看见几个话题
```
ros2 topic list
/left/actuator_states #左手电机反馈
/left/joint_control #左手关节控制
/right/actuator_states #右手电机反馈
/right/joint_control #右手电机控制
```

### 4.测试

回到初始状态
```bash
ros2 topic pub /right/joint_control aero_hand_open_msgs/JointControl "{header: {stamp: {sec: 0, nanosec: 0}}, target_positions: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]}" --once
```

握拳

```bash
ros2 topic pub /right/joint_control aero_hand_open_msgs/JointControl "{header: {stamp: {sec: 0, nanosec: 0}}, target_positions: [1.745,0.0,1.571,0.662,1.571,1.571,1.571,1.571,1.571,1.571,1.571,1.571,1.571,1.571,1.571,1.571]}" --once
```

#### python example
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from aero_hand_open_msgs.msg import JointControl

class Hand:
    """封装单个手掌的控制逻辑"""
    def __init__(self, open_pos, close_pos, timing):
        self.open_pos = open_pos
        self.close_pos = close_pos
        self.timing = timing

    def get_interpolated_positions(self, trigger_val):
        """核心计算：根据触发值和时序计算当前所有关节位置"""
        # 限制输入范围 0~1
        trigger_val = max(0.0, min(1.0, trigger_val))
        
        target_positions = []
        for i in range(16):
            start_t, end_t = self.timing[i]
            
            # 计算局部进度 (Local Fraction)
            if trigger_val <= start_t:
                local_f = 0.0
            elif trigger_val >= end_t:
                local_f = 1.0
            else:
                # 线性映射: 将 [start_t, end_t] 映射到 [0, 1]
                local_f = (trigger_val - start_t) / (end_t - start_t)
            
            # 关节线性插值
            val = self.open_pos[i] + (self.close_pos[i] - self.open_pos[i]) * local_f
            target_positions.append(val)
        
        return target_positions

class PicoDualHandNode(Node):
    def __init__(self):
        super().__init__('pico_dual_hand_controller')

        # --- 1. 左手独立配置 ---
        left_open = [1.2]*1+[0.2]*3+[0.3]*14
        left_close = [1.745, 1.571, 1.571, 1.571, 1.571, 1.571, 1.571, 1.571, 1.571, 1.571, 1.571, 1.571, 1.571, 1.571, 1.571, 1.571]
        # 左手时序：拇指和食指先扣住，其他手指后收紧
        left_timing = [
            # --- 第一阶段：拇指和食指扣住物体 (0.0-0.4) ---
            
            # 大拇指 (位置 0-3) - 先扣住
            (0.0, 0.2), (0.4, 0.8),     # 0,1 虎口与指根：首先接触物体
            (0.1, 0.9), (0.2, 1.0),     # 2,3 指中与指尖：逐步扣紧
            
            # 食指 (位置 4-6) - 同步拇指扣住
            (0.0, 0.4),                 # 4 指根：最早接触
            (0.1, 0.8),                 # 5 指中
            (0.2, 0.9),                 # 6 指尖：完成扣住动作
            
            # --- 第二阶段：其他手指收紧 (0.4-1.0) ---
            
            # 中指 (位置 7-9) - 稍后收紧
            (0.1, 0.3),                 # 7 指根：开始动作
            (0.2, 0.6),                 # 8 指中
            (0.3, 0.9),                 # 9 指尖：最后收紧
            
            # 无名指 (位置 10-12) - 更晚收紧
            (0.2, 0.4),                 # 10 指根
            (0.3, 0.7),                 # 11 指中
            (0.4, 0.9),                 # 12 指尖
            
            # 小拇指 (位置 13-15) - 最后收紧
            (0.4, 0.5),                 # 13 指根
            (0.5, 0.8),                 # 14 指中
            (0.6, 1.0),                 # 15 指尖：最后完成
        ]
        self.left_hand = Hand(left_open, left_close, left_timing)

        # --- 2. 右手独立配置 ---
        right_open = left_open
        right_close = left_close
        # 5. 大拇指 (0,1,2,3)：0.6 以后再收紧虎口。

        right_timing = left_timing
        # right_timing = [((0.0,1.0) if i==15 else (1.0,1.0) )for i,j in enumerate(right_open)]
        print(right_timing)
        self.right_hand = Hand(right_open, right_close, right_timing)

        # 订阅者
        self.create_subscription(Float32, '/pico/left_trigger', self.left_cb, 10)
        self.create_subscription(Float32, '/pico/right_trigger', self.right_cb, 10)

        # 发布者
        self.left_pub = self.create_publisher(JointControl, '/left/joint_control', 10)
        self.right_pub = self.create_publisher(JointControl, '/right/joint_control', 10)

        self.get_logger().info('Dual Hand Controller with Independent Timing Logic Started.')

    def left_cb(self, msg):
        joint_msg = JointControl()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.target_positions = self.left_hand.get_interpolated_positions(msg.data)
        self.left_pub.publish(joint_msg)

    def right_cb(self, msg):
        joint_msg = JointControl()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.target_positions = self.right_hand.get_interpolated_positions(msg.data)
        self.right_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PicoDualHandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```