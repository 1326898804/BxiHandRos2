#!/usr/bin/env python3
# Copyright 2025 TetherIA, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time 
import struct
from typing import Iterator
from serial import Serial, SerialTimeoutException
from aero_open_sdk.aero_hand_constants import AeroHandConstants
from aero_open_sdk.joints_to_actuations import MOTOR_PULLEY_RADIUS, JointsToActuationsModel
from aero_open_sdk.actuations_to_joints import ActuationsToJointsModelCompact

## Setup Modes
HOMING_MODE = 0x01
SET_ID_MODE = 0x02
TRIM_MODE = 0x03

## Command Modes
CTRL_POS = 0x11
CTRL_TOR = 0x12

## Request Modes
GET_ALL = 0x21
GET_POS = 0x22
GET_VEL = 0x23
GET_CURR = 0x24
GET_TEMP = 0x25

## Setting Modes
SET_SPE = 0x31
SET_TOR = 0x32

_UINT16_MAX = 65535

_RAD_TO_DEG = 180.0 / 3.141592653589793
_DEG_TO_RAD = 3.141592653589793 / 180.0
import threading
import queue
import asyncio
from bleak import BleakClient
class FakeSerial:
    def __init__(self, port_or_mac, baudrate=921600, bluetooth=False, debug=True):
        self.bluetooth = bluetooth
        self.debug = debug
        self.running = True
        
        # 队列定义
        self.payload_queue = queue.Queue()  # 标准 16 字节返回包 (Opcode 回复)
        self.state_queue = queue.Queue()    # 新增：61 字节电机状态包 (JointState)
        
        self.buffer = bytearray()
        
        # 协议定义
        self.HEADER = b'\xA1\xA2\xA3\xA4'
        self.CMD_FRAME_SIZE = 21   # 4(Header) + 16(Payload) + 1(CRC)
        self.STATE_FRAME_SIZE = 61 # 4(Header) + 56(28*uint16) + 1(CRC)

        # 蓝牙配置常量
        self.SVC_UUID = "0000abb0-0000-1000-8000-00805f9b34fb" 
        self.CHAR_STATE_UUID = "0000abb1-0000-1000-8000-00805f9b34fb" # 新增：订阅电机状态
        self.CHAR_WRITE_UUID = "0000abb2-0000-1000-8000-00805f9b34fb" 
        self.CHAR_READ_UUID = "0000abb3-0000-1000-8000-00805f9b34fb" # 用于命令回复

        if not bluetooth:
            self._log(f"[INIT] 正在打开串口: {port_or_mac}")
            self.ser = Serial(port_or_mac, baudrate, timeout=0.01, write_timeout=0.01)
        else:
            self._log(f"[INIT] 蓝牙模式初始化, MAC: {port_or_mac}")
            self.mac = port_or_mac
            self.ble_write_queue = asyncio.Queue() 
            self._loop_ref = None 
            
            self.bt_loop_thread = threading.Thread(target=self._start_bt_loop, daemon=True)
            self.bt_loop_thread.start()
            
            while self._loop_ref is None:
                time.sleep(0.1)
        
        self.parse_thread = threading.Thread(target=self._read_and_parse_loop, daemon=True)
        self.parse_thread.start()

    def _log(self, msg):
        if self.debug: print(msg)

    def _start_bt_loop(self):
        self._loop_ref = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop_ref)
        self._loop_ref.run_until_complete(self._bt_manager_task())

    async def _bt_manager_task(self):
        while self.running:
            try:
                self._log(f"[BLE] 尝试连接 {self.mac}...")
                async with BleakClient(self.mac, timeout=10.0, mtu_size=517) as client:
                    self._log(f"[BLE] 已连接. MTU={client.mtu_size}")
                    
                    # 订阅特征 1：标准命令回复 (0xABB3)
                    await client.start_notify(self.CHAR_READ_UUID, self._ble_data_handler)
                    # 订阅特征 2：电机状态推送 (0xABB1)
                    await client.start_notify(self.CHAR_STATE_UUID, self._ble_data_handler)
                    
                    while client.is_connected and self.running:
                        try:
                            data = await asyncio.wait_for(self.ble_write_queue.get(), timeout=0.01)
                            if data is None:
                                print("none")
                                break 
                            await client.write_gatt_char(self.CHAR_WRITE_UUID, data, response=False)
                        except asyncio.TimeoutError:
                            continue
            except Exception as e:
                if self.running:
                    self._log(f"[BLE_ERR] {e}. 5秒后重连...")
                    await asyncio.sleep(5)

    def _ble_data_handler(self, sender, data):
        self.buffer.extend(data)

    def _calc_crc8(self, data: bytes):
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

    def _read_and_parse_loop(self):
        """解析引擎：根据帧头 A1-A4 (命令) 和 A5-A8 (状态) 进行分流"""
        # 定义帧头常量
        HEADER_CMD = b'\xA1\xA2\xA3\xA4'
        HEADER_STATE = b'\xA5\xA6\xA7\xA8'
        
        while self.running:
            if not self.bluetooth:
                if self.ser.in_waiting > 0:
                    self.buffer.extend(self.ser.read(self.ser.in_waiting))
                else:
                    time.sleep(0.001)
            else:
                time.sleep(0.001)

            # 只要缓冲区足够判断一个头部
            while len(self.buffer) >= 4:
                # 寻找两个可能的帧头位置
                pos_cmd = self.buffer.find(HEADER_CMD)
                pos_state = self.buffer.find(HEADER_STATE)

                # 确定哪一个头更靠前
                positions = [p for p in [pos_cmd, pos_state] if p != -1]
                if not positions:
                    # 没找到任何头，保留末尾 3 字节防止头断裂，其余删除
                    if len(self.buffer) > 3 :
                        del self.buffer[:-3]
                    break
                
                first_header_pos = min(positions)
                if first_header_pos > 0:
                    del self.buffer[:first_header_pos]

                # 再次确认当前 buffer 开头是什么包
                if self.buffer.startswith(HEADER_STATE):
                    # --- 处理 61 字节状态包 ---
                    if len(self.buffer) < self.STATE_FRAME_SIZE:
                        break # 数据不够一包，等下次
                    
                    frame = self.buffer[:self.STATE_FRAME_SIZE]
                    if self._calc_crc8(frame[:-1]) == frame[-1]:
                        # payload 长度为 56 字节 (28个 uint16)
                        # < 表示小端，28H 表示 28 个无符号短整型
                        payload = struct.unpack("<28H", frame[4:60])
                        self.state_queue.put(payload)
                        del self.buffer[:self.STATE_FRAME_SIZE]
                    else:
                        self._log("[CRC_ERR] 状态包校验失败")
                        del self.buffer[:1]

                elif self.buffer.startswith(HEADER_CMD):
                    # --- 处理 21 字节命令回复包 ---
                    if len(self.buffer) < self.CMD_FRAME_SIZE:
                        break
                    
                    frame = self.buffer[:self.CMD_FRAME_SIZE]
                    if self._calc_crc8(frame[:-1]) == frame[-1]:
                        self.payload_queue.put(frame[4:20])
                        del self.buffer[:self.CMD_FRAME_SIZE]
                    else:
                        self._log("[CRC_ERR] 命令包校验失败")
                        del self.buffer[:1]
    def get_latest_state(self):
        """获取最新的电机状态包（非阻塞）"""
        latest = None
        while not self.state_queue.empty():
            try:
                latest = self.state_queue.get_nowait()
            except queue.Empty:
                break
        return latest # 返回 tuple (pos0, pos1, ..., temp6) 或 None

    def read(self, size=16):
        try:
            return self.payload_queue.get(block=True, timeout=1.0)
        except queue.Empty:
            return b''

    def write(self, msg):
        if len(msg) != 16: raise ValueError("msg must be 16 bytes")
        full_frame = self.HEADER + msg
        final_packet = full_frame + struct.pack("B", self._calc_crc8(full_frame))
        
        if self.bluetooth:
            if self._loop_ref and self._loop_ref.is_running():
                try:
                    while not self.ble_write_queue.empty():
                        self.ble_write_queue.get_nowait()
                except: 
                    pass
                
                self._loop_ref.call_soon_threadsafe(self.ble_write_queue.put_nowait, final_packet)
        else:
            self.ser.write(final_packet)
        return len(final_packet)
    
    def reset_input_buffer(self):
        self._log("[RESET] 输入缓冲已清理")
        if not self.bluetooth: self.ser.reset_input_buffer()
        self.buffer.clear()
        while not self.payload_queue.empty():
            try: self.payload_queue.get_nowait()
            except queue.Empty: break

    def reset_output_buffer(self):
        if not self.bluetooth: self.ser.reset_output_buffer()

    def flush(self):
        if not self.bluetooth: self.ser.flush()

    async def _async_close_ble(self):
        """蓝牙异步清理逻辑"""
        try:
            await self.ble_write_queue.put(None) # 唤醒并终止队列等待
        except Exception:
            pass

    def close(self):
        self._log("[CLOSE] 正在启动关闭程序...")
        self.running = False 

        if self.bluetooth:
            if self._loop_ref and self._loop_ref.is_running():
                # 通知异步逻辑结束
                asyncio.run_coroutine_threadsafe(self._async_close_ble(), self._loop_ref)
                time.sleep(0.5) # 给一点时间处理断开
                self._loop_ref.call_soon_threadsafe(self._loop_ref.stop)
        else:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
                self._log("[CLOSE] 串口已关闭")

        self._log("[CLOSE] 资源已释放")

class AeroHand:
    def __init__(self, port=None, baudrate=921600,bluetooth=False):
        ## Connect to serial port
        self.bluetooth=bluetooth
        self.ser = FakeSerial(port,baudrate,bluetooth)
        self._last_state = None

        ## Clean Buffers before starting
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        aero_hand_constants = AeroHandConstants()

        self.joint_names = aero_hand_constants.joint_names
        self.joint_lower_limits = aero_hand_constants.joint_lower_limits
        self.joint_upper_limits = aero_hand_constants.joint_upper_limits

        self.actuation_names = aero_hand_constants.actuation_names
        self.actuation_lower_limits = aero_hand_constants.actuation_lower_limits
        self.actuation_upper_limits = aero_hand_constants.actuation_upper_limits

        self.joints_to_actuations_model = JointsToActuationsModel()
        self.actuations_to_joints_model = ActuationsToJointsModelCompact()

    def create_trajectory(self, trajectory: list[tuple[list[float], float]]) -> Iterator[list[float]]:
        rate = 100  # Hz

        def _interp_keypoints(start, end, t):
            return [start[i] + t * (end[i] - start[i]) for i in range(len(start))]

        for i in range(1, len(trajectory)):
            prev_keypoint, _ = trajectory[i - 1]
            curr_keypoint, duration = trajectory[i]

            num_steps = int(duration * rate)

            for step in range(1, num_steps + 1):
                t = step / num_steps
                yield _interp_keypoints(prev_keypoint, curr_keypoint, t)

    def run_trajectory(self, trajectory: list):
        ## Linerly interpolate between trajectory points
        interpolated_traj = self.create_trajectory(trajectory)
        for waypoint in interpolated_traj:
            self.set_joint_positions(waypoint)
            time.sleep(0.001)
        return
    
    def convert_seven_joints_to_sixteen(self, positions: list) -> list:
        return [
            positions[0], positions[1], positions[2], positions[2],
            positions[3], positions[3], positions[3],
            positions[4], positions[4], positions[4],
            positions[5], positions[5], positions[5],
            positions[6], positions[6], positions[6],
        ]

    def set_joint_positions(self, positions: list):
        """
        Set the joint positions of the Aero Hand.

        Args:
            positions (list): A list of 16 joint positions. (degrees)
        """
        assert len(positions) in (16, 7), "Expected 16 or 7 Joint Positions"
        if len(positions) == 7:
            positions = self.convert_seven_joints_to_sixteen(positions)
        ## Clamp the positions to the joint limits.
        positions = [
            max(
                self.joint_lower_limits[i],
                min(positions[i], self.joint_upper_limits[i]),
            )
            for i in range(16)
        ]

        ## Convert to actuations
        actuations = self.joints_to_actuations_model.hand_actuations(positions)

        ## Normalize actuation to uint16 range. (0-65535)
        actuations = [
            (actuations[i] - self.actuation_lower_limits[i])
            / (self.actuation_upper_limits[i] - self.actuation_lower_limits[i])
            * _UINT16_MAX
            for i in range(7)
        ]
        try:
            self._send_data(CTRL_POS, [int(a) for a in actuations])
        except SerialTimeoutException as e:
            print(f"Serial Timeout while sending joint positions: {e}")
            return

    def tendon_to_actuations(self, tendon_extension: float) -> float:
        """
        Convert tendon extension (mm) to actuator actuations (degrees).
        Args:
            tendon_extension (float): Tendon extension in mm.
        Returns:
            float: actuator actuations in degrees.
        """

        return (tendon_extension / MOTOR_PULLEY_RADIUS) * _RAD_TO_DEG
    
    def actuations_to_tendon(self, actuation: float) -> float:
        """
        Convert actuator actuations (degrees) to tendon extension (mm).
        Args:
            actuation (float): actuator actuations in degrees.
        Returns:
            float: Tendon extension in mm.
        """

        return (actuation * MOTOR_PULLEY_RADIUS) * _DEG_TO_RAD

    def set_actuations(self, actuations: list):
        """
        This function is used to set the actuations of the hand directly.
        Use this with caution as Thumb actuations are not independent i.e. setting one
        actuation requires changes in other actuations. We use the joint to 
        actuations model to handle this. But this function give you direct access.
        If the actuations are not coupled correctly, it will cause Thumb tendons to
        derail.
        Args:
            actuations (list): A list of 7 actuations in degrees
            actuator actuations sequence being:
            (thumb_cmc_abd_act, thumb_cmc_flex_act, thumb_tendon, index_tendon, middle_tendon, ring_tendon, pinky_tendon)
        """
        assert len(actuations) == 7, "Expected 7 Actuations"

        ## Clamp the actuations to the limits.
        actuations = [
            max(
                self.actuation_lower_limits[i],
                min(actuations[i], self.actuation_upper_limits[i]),
            )
            for i in range(7)
        ]

        ## Normalize actuation to uint16 range. (0-65535)
        actuations = [
            (actuations[i] - self.actuation_lower_limits[i])
            / (self.actuation_upper_limits[i] - self.actuation_lower_limits[i])
            * _UINT16_MAX
            for i in range(7)
        ]

        try:
            self._send_data(CTRL_POS, [int(a) for a in actuations])
        except SerialTimeoutException as e:
            print(f"Error while writing to serial port: {e}")
            return

    def _wait_for_ack(self, opcode: int, timeout_s: float) -> bytes:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame = self.ser.read(16)
            if len(frame) != 16:
                continue 
            if frame[0] == (opcode & 0xFF) and frame[1] == 0x00:
                return frame[2:]
        raise TimeoutError(f"ACK (opcode 0x{opcode:02X}) not received within {timeout_s}s")
    
    def set_id(self, id: int, current_limit: int):
        """This fn is used by the GUI to set actuator IDs and current limits for the first time."""
        if not (0 <= id <= 253):
            raise ValueError("new_id must be 0..253")
        if not (0 <= current_limit <= 1023):
            raise ValueError("current_limit must be in between 0..1023")
        
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

        payload = [0] * 7
        payload[0] = id & 0xFF   # stored in low byte of word0
        payload[1] = current_limit & 0x03FF
        self._send_data(SET_ID_MODE, payload)
        payload = self._wait_for_ack(SET_ID_MODE, 5.0)
        old_id, new_id, cur_limit = struct.unpack_from("<HHH", payload, 0)
        return {"Old_id": old_id, "New_id": new_id, "Current_limit": cur_limit}
    
    def set_speed(self, id: int, speed: int):
        """ 
        Set the speed of a specific actuator.This speed setting is max by default when the motor moves.
        This is different from speed control mode. It only affect the dynamic of motion execution during position control.
        Args:
            id (int): Actuator ID (0..6)
            speed (int): Speed value (0..32766)
        """
        if not (0 <= id <= 6):
            raise ValueError("id must be 0..6")
        if not (0 <= speed <= 32766):
            raise ValueError("speed must be in range 0..32766")
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        payload = [0] * 7
        payload[0] = id & 0xFFFF
        payload[1] = speed & 0xFFFF
        self._send_data(SET_SPE, payload)
        payload = self._wait_for_ack(SET_SPE, 2.0)
        id, speed_val = struct.unpack_from("<HH", payload, 0)
        return {"Servo ID": id, "Speed": speed_val}

    def set_torque(self, id: int, torque: int):
        """ 
         Set the torque of a specific actuator. This torque setting is max by default when the motor moves.
         This is different from torque control mode. It only affect the dynamic of motion execution during position control.
         Args:
            id (int): Actuator ID (0..6)
            torque (int): Torque value (0..1000)
        """
        if not (0 <= id <= 6):
            raise ValueError("id must be 0..6")
        if not (0 <= torque <= 1000):
            raise ValueError("torque must be in range 0..1000")
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        payload = [0] * 7
        payload[0] = id & 0xFFFF
        payload[1] = torque & 0xFFFF
        self._send_data(SET_TOR, payload)
        payload = self._wait_for_ack(SET_TOR, 2.0)
        id, torque_val = struct.unpack_from("<HH", payload, 0)
        return {"Servo ID": id, "Torque": torque_val}

    def trim_servo(self, id: int, degrees: int):
        """This fn is used by the GUI to fine tune the actuator positions."""
        if not (0 <= id <= 6):
            raise ValueError("id must be 0..6")
        if not (-360 <= degrees <= 360):
            raise ValueError("degrees out of range")
        
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

        payload = [0] * 7
        payload[0] = id & 0xFFFF
        payload[1] = degrees & 0xFFFF  
        self._send_data(TRIM_MODE, payload)
        payload = self._wait_for_ack(TRIM_MODE, 2.0)
        id, extend = struct.unpack_from("<HH", payload, 0)
        return {"Servo ID": id, "Extend Count": extend}
    
    def ctrl_torque(self, torque: list[int]):
        """
        Set the same torque value for all 7 servos using the CTRL_TOR command.
        Args:
            torque (list[int]): Torque values (0..1000)
        """
        if not all(0 <= t <= 1000 for t in torque):
            raise ValueError("torque must be in range 0..1000")
        payload = [t & 0xFFFF for t in torque]
        self._send_data(CTRL_TOR, payload)

    def _send_data(self, header: int, payload: list[int] = [0] * 7):
        assert self.ser is not None, "Serial port is not initialized"
        assert len(payload) == 7, "Payload must be a list of 7 integers in Range 0-65535"
        assert all(0 <= v <= 65535 for v in payload), "Payload values must be in Range 0-65535"
        msg = struct.pack("<2B7H", header & 0xFF, 0x00, *(v & 0xFFFF for v in payload))
        self.ser.write(msg)
        self.ser.flush()

    def send_homing(self, timeout_s: float = 175.0):
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._send_data(HOMING_MODE) 
        payload = self._wait_for_ack(HOMING_MODE, timeout_s)
        if all(b == 0 for b in payload):
            return True
        else:
            raise ValueError(f"Unexpected HOMING payload: {payload.hex()}")

    def get_forward_kinematics(self):
        raise NotImplementedError("This method is not yet implemented")

    def get_joint_positions(self):
        raise NotImplementedError("This method is not yet implemented")
    
    def get_joint_positions_compact(self):
        """
        Get the joint positions from the hand in the compact 7 joint representation.
        Returns:
            list: A list of 7 joint positions. (degrees)
        """
        actuations = self.get_actuations()
        ## If there was an error getting actuations, return None
        if actuations is None:
            return None
        ## Convert to radians
        actuations = [act * _DEG_TO_RAD for act in actuations]

        ## Get Joint Positions
        joint_positions = self.actuations_to_joints_model.hand_joints(actuations)

        ## Convert to degrees
        joint_positions = [pos * _RAD_TO_DEG for pos in joint_positions]

        return joint_positions
    def update_internal_state(self):
        """
        从 FakeSerial 队列中获取最新解析的 28 个 uint16
        映射关系：
        0-6: 位置, 7-13: 速度, 14-20: 电流, 21-27: 温度
        """
        new_packet = self.ser.get_latest_state()
        if new_packet is not None:
            self._last_state = new_packet
        return self._last_state
    
    def get_actuations(self):
        """获取 7 个电机的位置 (角度)"""
        state = self.update_internal_state()
        if state is None: return None
        
        # 提取 payload[0:7]
        positions_uint16 = state[0:7]
        return [
            self.actuation_lower_limits[i]
            + (positions_uint16[i] / 65535.0)
            * (self.actuation_upper_limits[i] - self.actuation_lower_limits[i])
            for i in range(7)
        ]

    def get_actuator_speeds(self):
        """获取 7 个电机的速度 (RPM)"""
        state = self.update_internal_state()
        if state is None: return None
        
        # 提取 payload[7:14]，根据 Feetech 协议 uint16 转有符号 int16
        # 1 unit = 0.732 RPM
        raw_speeds = state[7:14]
        speeds_rpm = []
        for val in raw_speeds:
            # 处理补码转有符号数
            s = struct.unpack('h', struct.pack('H', val))[0]
            speeds_rpm.append(s * 0.732)
        return speeds_rpm

    def get_actuator_currents(self):
        """获取 7 个电机的电流 (mA)"""
        state = self.update_internal_state()
        if state is None: return None
        
        # 提取 payload[14:21]
        # 1 unit = 6.5 mA
        raw_currents = state[14:21]
        currents_ma = []
        for val in raw_currents:
            c = struct.unpack('h', struct.pack('H', val))[0]
            currents_ma.append(c * 6.5)
        return currents_ma

    def get_actuator_temperatures(self):
        """获取 7 个电机的温度 (Celsius)"""
        state = self.update_internal_state()
        if state is None: return None
        
        # 提取 payload[21:28]
        return [float(val) for val in state[21:28]]

    def close(self):
        self.ser.close()
