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
import struct
import time
from typing import Optional, Tuple, Dict, Any, List
from dataclasses import dataclass, field
from bleak import BleakClient
from collections import deque

try:
    from serial import Serial
except ImportError:
    Serial = None


@dataclass
class FrameConfig:
    """帧配置"""
    header: bytes
    frame_size: int
    payload_format: str
    queue_name: str
    description: str


@dataclass
class Statistics:
    """通信统计信息"""
    # 接收统计
    rx_bytes: int = 0              # 接收总字节数
    rx_frames: int = 0             # 接收总帧数
    rx_frames_ok: int = 0          # 接收成功帧数
    rx_frames_crc_err: int = 0     # CRC错误帧数
    rx_frames_parse_err: int = 0   # 解析错误帧数
    
    # 发送统计
    tx_bytes: int = 0              # 发送总字节数
    tx_frames: int = 0             # 发送总帧数
    tx_frames_failed: int = 0      # 发送失败帧数
    
    # 速率统计（使用滑动窗口，最近1秒）
    rx_rate_window: deque = field(default_factory=lambda: deque(maxlen=100))
    tx_rate_window: deque = field(default_factory=lambda: deque(maxlen=100))
    
    # 按帧类型统计
    frame_type_stats: Dict[str, int] = field(default_factory=dict)
    
    # 连接统计
    connect_time: Optional[float] = None
    disconnect_count: int = 0
    reconnect_count: int = 0
    
    # 延迟统计（需要时间戳）
    latency_samples: deque = field(default_factory=lambda: deque(maxlen=100))
    
    def get_rx_rate(self) -> float:
        """获取接收速率（字节/秒）"""
        if len(self.rx_rate_window) < 2:
            return 0.0
        
        time_span = self.rx_rate_window[-1][0] - self.rx_rate_window[0][0]
        if time_span <= 0:
            return 0.0
        
        total_bytes = sum(size for _, size in self.rx_rate_window)
        return total_bytes / time_span
    
    def get_tx_rate(self) -> float:
        """获取发送速率（字节/秒）"""
        if len(self.tx_rate_window) < 2:
            return 0.0
        
        time_span = self.tx_rate_window[-1][0] - self.tx_rate_window[0][0]
        if time_span <= 0:
            return 0.0
        
        total_bytes = sum(size for _, size in self.tx_rate_window)
        return total_bytes / time_span
    
    def get_frame_rate(self, frame_type: str = None) -> float:
        """获取帧速率（帧/秒）"""
        if frame_type:
            count = self.frame_type_stats.get(frame_type, 0)
            uptime = self.get_uptime()
            return count / uptime if uptime > 0 else 0.0
        else:
            uptime = self.get_uptime()
            return self.rx_frames_ok / uptime if uptime > 0 else 0.0
    
    def get_error_rate(self) -> float:
        """获取错误率"""
        total = self.rx_frames
        if total == 0:
            return 0.0
        errors = self.rx_frames_crc_err + self.rx_frames_parse_err
        return errors / total
    
    def get_success_rate(self) -> float:
        """获取成功率"""
        total = self.rx_frames
        if total == 0:
            return 0.0
        return self.rx_frames_ok / total
    
    def get_uptime(self) -> float:
        """获取运行时间（秒）"""
        if self.connect_time is None:
            return 0.0
        return time.time() - self.connect_time
    
    def get_avg_latency(self) -> float:
        """获取平均延迟（毫秒）"""
        if not self.latency_samples:
            return 0.0
        return sum(self.latency_samples) / len(self.latency_samples)
    
    def get_signal_quality(self) -> float:
        """获取信号质量（0-100）"""
        # 基于成功率和错误率计算信号质量
        success_rate = self.get_success_rate()
        error_rate = self.get_error_rate()
        
        # 信号质量 = 成功率 * (1 - 错误率) * 100
        quality = success_rate * (1 - error_rate) * 100
        return min(100.0, max(0.0, quality))
    
    def record_rx(self, size: int):
        """记录接收"""
        self.rx_bytes += size
        self.rx_rate_window.append((time.time(), size))
    
    def record_tx(self, size: int):
        """记录发送"""
        self.tx_bytes += size
        self.tx_rate_window.append((time.time(), size))
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            'uptime': self.get_uptime(),
            'rx_bytes': self.rx_bytes,
            'rx_frames': self.rx_frames,
            'rx_frames_ok': self.rx_frames_ok,
            'rx_crc_errors': self.rx_frames_crc_err,
            'rx_parse_errors': self.rx_frames_parse_err,
            'rx_rate_bps': self.get_rx_rate(),
            'tx_bytes': self.tx_bytes,
            'tx_frames': self.tx_frames,
            'tx_failed': self.tx_frames_failed,
            'tx_rate_bps': self.get_tx_rate(),
            'success_rate': self.get_success_rate(),
            'error_rate': self.get_error_rate(),
            'signal_quality': self.get_signal_quality(),
            'avg_latency_ms': self.get_avg_latency(),
            'disconnect_count': self.disconnect_count,
            'reconnect_count': self.reconnect_count,
            'frame_types': dict(self.frame_type_stats),
        }


class ProtocolConfig:
    """协议配置类"""
    
    HEADER_CMD = b'\xA1\xA2\xA3\xA4'
    HEADER_STATE = b'\xA5\xA6\xA7\xA8'
    
    SEND_HEADER = HEADER_CMD
    CRC_POLYNOMIAL = 0x07
    
    BLE_SERVICE_UUID = "0000abb0-0000-1000-8000-00805f9b34fb"
    BLE_CHAR_STATE_UUID = "0000abb1-0000-1000-8000-00805f9b34fb"
    BLE_CHAR_WRITE_UUID = "0000abb2-0000-1000-8000-00805f9b34fb"
    BLE_CHAR_READ_UUID = "0000abb3-0000-1000-8000-00805f9b34fb"
    
    FRAME_CONFIGS = [
        FrameConfig(
            header=HEADER_CMD,
            frame_size=21,
            payload_format="16s",
            queue_name="cmd_response",
            description="命令响应帧"
        ),
        FrameConfig(
            header=HEADER_STATE,
            frame_size=61,
            payload_format="<28H",
            queue_name="motor_state",
            description="电机状态帧"
        ),
    ]
    
    SEND_PAYLOAD_SIZE = 16
    SEND_PAYLOAD_FORMAT = "16s"
    
    @classmethod
    def get_frame_by_header(cls, header: bytes) -> Optional[FrameConfig]:
        for config in cls.FRAME_CONFIGS:
            if config.header == header:
                return config
        return None
    
    @classmethod
    def get_all_headers(cls) -> List[bytes]:
        return [config.header for config in cls.FRAME_CONFIGS]


class FakeSerial:
    """支持蓝牙和串口的双模通信"""
    
    def __init__(self, port_or_mac: str, baudrate: int = 921600, 
                 bluetooth: bool = False, debug: bool = True,
                 protocol_config: type = ProtocolConfig,
                 enable_stats: bool = True):
        """
        初始化通信类
        
        Args:
            port_or_mac: 串口名称或蓝牙MAC地址
            baudrate: 波特率（仅串口模式）
            bluetooth: 是否使用蓝牙模式
            debug: 是否输出调试信息
            protocol_config: 协议配置类
            enable_stats: 是否启用统计功能
        """
        self.bluetooth = bluetooth
        self.debug = debug
        self.running = True
        self.protocol = protocol_config
        self.enable_stats = enable_stats
        
        # 统计对象
        self.stats = Statistics() if enable_stats else None
        self._stats_lock = threading.Lock()
        
        # 动态创建队列
        self.queues: Dict[str, queue.Queue] = {}
        for config in self.protocol.FRAME_CONFIGS:
            self.queues[config.queue_name] = queue.Queue(maxsize=100)
        
        # 接收缓冲区
        self.buffer = bytearray()
        self._buffer_lock = threading.Lock()
        
        # 创建帧头到配置的映射
        self._header_map: Dict[bytes, FrameConfig] = {
            config.header: config for config in self.protocol.FRAME_CONFIGS
        }
        
        # 初始化传输层
        if not bluetooth:
            self._init_serial(port_or_mac, baudrate)
        else:
            self._init_bluetooth(port_or_mac)
        
        # 启动解析线程
        self.parse_thread = threading.Thread(
            target=self._read_and_parse_loop, 
            daemon=True, 
            name="ParseThread"
        )
        self.parse_thread.start()
        
        # 启动统计显示线程
        if enable_stats:
            self.stats_thread = threading.Thread(
                target=self._stats_display_loop,
                daemon=True,
                name="StatsThread"
            )
            # self.stats_thread.start()  # 取消注释以启用自动打印统计
        
        self._log(f"[INIT] 协议配置加载完成，支持 {len(self.protocol.FRAME_CONFIGS)} 种帧类型")

    def _init_serial(self, port: str, baudrate: int):
        """初始化串口连接"""
        if Serial is None:
            raise ImportError("pyserial not installed. Run: pip install pyserial")
        
        self._log(f"[INIT] 正在打开串口: {port}")
        self.ser = Serial(port, baudrate, timeout=0.01, write_timeout=0.01)
        self._log(f"[INIT] 串口已打开: {port} @ {baudrate}")
        
        if self.enable_stats:
            self.stats.connect_time = time.time()

    def _init_bluetooth(self, mac: str):
        """初始化蓝牙连接"""
        self._log(f"[INIT] 蓝牙模式初始化, MAC: {mac}")
        self.mac = mac
        self.ble_write_queue = asyncio.Queue()
        self._loop_ref = None
        self._client_ref = None
        
        self.bt_loop_thread = threading.Thread(
            target=self._start_bt_loop, 
            daemon=True, 
            name="BLEThread"
        )
        self.bt_loop_thread.start()
        
        timeout = 5
        start_time = time.time()
        while self._loop_ref is None:
            if time.time() - start_time > timeout:
                raise TimeoutError("蓝牙事件循环初始化超时")
            time.sleep(0.1)
        
        self._log("[INIT] 蓝牙事件循环已启动")

    def _log(self, msg: str):
        """日志输出"""
        if self.debug:
            print(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def _start_bt_loop(self):
        """启动蓝牙异步事件循环"""
        self._loop_ref = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop_ref)
        try:
            self._loop_ref.run_until_complete(self._bt_manager_task())
        except Exception as e:
            self._log(f"[BLE_FATAL] 事件循环异常退出: {e}")
        finally:
            self._loop_ref.close()

    async def _bt_manager_task(self):
        """蓝牙连接管理任务（自动重连）"""
        retry_count = 0
        max_retries = 3
        
        while self.running:
            try:
                self._log(f"[BLE] 尝试连接 {self.mac}... (尝试 {retry_count + 1})")
                
                async with BleakClient(
                    self.mac, 
                    timeout=15.0,
                    disconnected_callback=self._on_disconnect
                ) as client:
                    self._client_ref = client
                    self._log(f"[BLE] 已连接 {self.mac}")
                    
                    # 统计：连接成功
                    if self.enable_stats:
                        with self._stats_lock:
                            if self.stats.connect_time is None:
                                self.stats.connect_time = time.time()
                            else:
                                self.stats.reconnect_count += 1
                    
                    retry_count = 0
                    
                    # 订阅通知特征
                    if hasattr(self.protocol, 'BLE_CHAR_READ_UUID'):
                        await client.start_notify(
                            self.protocol.BLE_CHAR_READ_UUID, 
                            self._ble_data_handler
                        )
                    
                    if hasattr(self.protocol, 'BLE_CHAR_STATE_UUID'):
                        await client.start_notify(
                            self.protocol.BLE_CHAR_STATE_UUID, 
                            self._ble_data_handler
                        )
                    
                    self._log("[BLE] 已订阅所有通知特征")
                    
                    # 写入循环
                    while client.is_connected and self.running:
                        try:
                            data = await asyncio.wait_for(
                                self.ble_write_queue.get(), 
                                timeout=0.1
                            )
                            
                            if data is None:
                                self._log("[BLE] 收到退出信号")
                                break
                            
                            await client.write_gatt_char(
                                self.protocol.BLE_CHAR_WRITE_UUID, 
                                data, 
                                response=False
                            )
                            
                            # 统计：发送成功
                            if self.enable_stats:
                                with self._stats_lock:
                                    self.stats.record_tx(len(data))
                            
                        except asyncio.TimeoutError:
                            continue
                        except Exception as e:
                            self._log(f"[BLE_WRITE_ERR] {e}")
                            # 统计：发送失败
                            if self.enable_stats:
                                with self._stats_lock:
                                    self.stats.tx_frames_failed += 1
                            break
                            
            except Exception as e:
                if not self.running:
                    break
                
                # 统计：断开连接
                if self.enable_stats:
                    with self._stats_lock:
                        self.stats.disconnect_count += 1
                
                retry_count += 1
                if retry_count >= max_retries:
                    self._log(f"[BLE_ERR] 达到最大重试次数，等待30秒...")
                    await asyncio.sleep(30)
                    retry_count = 0
                else:
                    self._log(f"[BLE_ERR] {e}. 5秒后重连...")
                    await asyncio.sleep(5)
            finally:
                self._client_ref = None

    def _on_disconnect(self, client):
        """蓝牙断开回调"""
        self._log(f"[BLE] 设备已断开: {client.address}")
        if self.enable_stats:
            with self._stats_lock:
                self.stats.disconnect_count += 1

    def _ble_data_handler(self, sender, data: bytearray):
        """蓝牙数据接收回调"""
        with self._buffer_lock:
            self.buffer.extend(data)
        
        # 统计：接收字节
        if self.enable_stats:
            with self._stats_lock:
                self.stats.record_rx(len(data))

    def _calc_crc8(self, data: bytes) -> int:
        """CRC8校验计算"""
        crc = 0x00
        polynomial = self.protocol.CRC_POLYNOMIAL
        
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ polynomial
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

    def _read_and_parse_loop(self):
        """数据解析主循环"""
        while self.running:
            # 从传输层读取数据
            if not self.bluetooth:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    with self._buffer_lock:
                        self.buffer.extend(data)
                    
                    # 统计：接收字节
                    if self.enable_stats:
                        with self._stats_lock:
                            self.stats.record_rx(len(data))
                else:
                    time.sleep(0.001)
            else:
                time.sleep(0.001)

            # 解析缓冲区
            with self._buffer_lock:
                self._parse_buffer()

    def _parse_buffer(self):
        """解析缓冲区中的数据帧"""
        all_headers = self.protocol.get_all_headers()
        
        while len(self.buffer) >= 4:
            # 查找所有可能的帧头位置
            header_positions = {}
            for header in all_headers:
                pos = self.buffer.find(header)
                if pos != -1:
                    header_positions[pos] = header
            
            if not header_positions:
                if len(self.buffer) > 3:
                    del self.buffer[:-3]
                break
            
            first_pos = min(header_positions.keys())
            if first_pos > 0:
                del self.buffer[:first_pos]
            
            current_header = header_positions[first_pos]
            frame_config = self._header_map.get(current_header)
            
            if frame_config and not self._parse_frame(frame_config):
                break

    def _parse_frame(self, config: FrameConfig) -> bool:
        """通用帧解析方法"""
        if len(self.buffer) < config.frame_size:
            return False
        
        frame = bytes(self.buffer[:config.frame_size])
        
        # 统计：接收到一帧
        if self.enable_stats:
            with self._stats_lock:
                self.stats.rx_frames += 1
        
        # CRC校验
        if self._calc_crc8(frame[:-1]) != frame[-1]:
            self._log(f"[CRC_ERR] {config.description} 校验失败")
            
            # 统计：CRC错误
            if self.enable_stats:
                with self._stats_lock:
                    self.stats.rx_frames_crc_err += 1
            
            del self.buffer[:1]
            return True
        
        # 提取payload
        header_len = len(config.header)
        payload_bytes = frame[header_len:-1]
        
        # 解析payload
        try:
            if config.payload_format == "raw":
                parsed_data = payload_bytes
            else:
                parsed_data = struct.unpack(config.payload_format, payload_bytes)
                if len(parsed_data) == 1:
                    parsed_data = parsed_data[0]
            
            # 放入对应队列
            target_queue = self.queues.get(config.queue_name)
            if target_queue:
                try:
                    target_queue.put_nowait(parsed_data)
                except queue.Full:
                    try:
                        target_queue.get_nowait()
                        target_queue.put_nowait(parsed_data)
                    except queue.Empty:
                        pass
            
            # 统计：解析成功
            if self.enable_stats:
                with self._stats_lock:
                    self.stats.rx_frames_ok += 1
                    frame_type = config.queue_name
                    self.stats.frame_type_stats[frame_type] = \
                        self.stats.frame_type_stats.get(frame_type, 0) + 1
            
            del self.buffer[:config.frame_size]
            
        except struct.error as e:
            self._log(f"[PARSE_ERR] {config.description} 解析失败: {e}")
            
            # 统计：解析错误
            if self.enable_stats:
                with self._stats_lock:
                    self.stats.rx_frames_parse_err += 1
            
            del self.buffer[:1]
        
        return True

    def _stats_display_loop(self):
        """统计显示循环（每5秒打印一次）"""
        while self.running:
            time.sleep(5)
            if self.enable_stats:
                self.print_stats()

    # ========== 统计相关API ==========
    
    def get_stats(self) -> Dict[str, Any]:
        """获取统计信息"""
        if not self.enable_stats:
            return {}
        
        with self._stats_lock:
            return self.stats.to_dict()
    
    def print_stats(self, detailed: bool = False):
        """打印统计信息"""
        if not self.enable_stats:
            print("统计功能未启用")
            return
        
        stats = self.get_stats()
        
        print("\n" + "="*60)
        print(f"通信统计 (运行时间: {stats['uptime']:.1f}s)")
        print("="*60)
        print(f"信号质量: {stats['signal_quality']:.1f}%")
        print(f"成功率:   {stats['success_rate']*100:.2f}%")
        print(f"错误率:   {stats['error_rate']*100:.2f}%")
        print("-"*60)
        print(f"接收: {stats['rx_bytes']} 字节, {stats['rx_frames_ok']} 帧")
        print(f"      速率: {stats['rx_rate_bps']:.1f} B/s")
        print(f"      CRC错误: {stats['rx_crc_errors']}, 解析错误: {stats['rx_parse_errors']}")
        print(f"发送: {stats['tx_bytes']} 字节, {stats['tx_frames']} 帧")
        print(f"      速率: {stats['tx_rate_bps']:.1f} B/s")
        print(f"      失败: {stats['tx_failed']}")
        
        if self.bluetooth:
            print(f"连接: 断开 {stats['disconnect_count']} 次, 重连 {stats['reconnect_count']} 次")
        
        if detailed and stats['frame_types']:
            print("-"*60)
            print("帧类型统计:")
            for frame_type, count in stats['frame_types'].items():
                rate = count / stats['uptime'] if stats['uptime'] > 0 else 0
                print(f"  {frame_type}: {count} 帧 ({rate:.1f} 帧/秒)")
        
        print("="*60 + "\n")
    
    def reset_stats(self):
        """重置统计信息"""
        if self.enable_stats:
            with self._stats_lock:
                self.stats = Statistics()
                self.stats.connect_time = time.time()

    # ========== 高层API ==========
    
    def get_data(self, queue_name: str, timeout: float = 1.0) -> Any:
        """从指定队列读取数据（阻塞）"""
        target_queue = self.queues.get(queue_name)
        if not target_queue:
            raise ValueError(f"Unknown queue name: {queue_name}")
        
        try:
            return target_queue.get(block=True, timeout=timeout)
        except queue.Empty:
            return None

    def get_latest_data(self, queue_name: str) -> Any:
        """获取指定队列的最新数据（非阻塞）"""
        target_queue = self.queues.get(queue_name)
        if not target_queue:
            raise ValueError(f"Unknown queue name: {queue_name}")
        
        latest = None
        while True:
            try:
                latest = target_queue.get_nowait()
            except queue.Empty:
                break
        return latest

    def read(self, size: int = 16, timeout: float = 1.0) -> bytes:
        """读取命令响应包（兼容旧代码）"""
        data = self.get_data('cmd_response', timeout)
        return data if data is not None else b''

    def get_latest_state(self) -> Optional[Tuple]:
        """获取最新的电机状态包（兼容旧代码）"""
        return self.get_latest_data('motor_state')

    def write(self, payload: bytes) -> int:
        """发送命令"""
        expected_size = self.protocol.SEND_PAYLOAD_SIZE
        if len(payload) != expected_size:
            raise ValueError(f"payload must be {expected_size} bytes, got {len(payload)}")
        
        # 构造完整帧
        full_frame = self.protocol.SEND_HEADER + payload
        crc = self._calc_crc8(full_frame)
        final_packet = full_frame + struct.pack("B", crc)
        
        # 统计：发送帧
        if self.enable_stats:
            with self._stats_lock:
                self.stats.tx_frames += 1
        
        if self.bluetooth:
            if not self._loop_ref or not self._loop_ref.is_running():
                raise RuntimeError("蓝牙事件循环未运行")
            
            # 清空写队列
            while True:
                try:
                    self.ble_write_queue.get_nowait()
                except asyncio.QueueEmpty:
                    break
            
            # 线程安全地放入队列
            future = asyncio.run_coroutine_threadsafe(
                self.ble_write_queue.put(final_packet),
                self._loop_ref
            )
            
            try:
                future.result(timeout=1.0)
            except Exception as e:
                self._log(f"[WRITE_ERR] 蓝牙写入失败: {e}")
                if self.enable_stats:
                    with self._stats_lock:
                        self.stats.tx_frames_failed += 1
                return 0
        else:
            try:
                self.ser.write(final_packet)
                
                # 统计：发送字节（串口模式）
                if self.enable_stats:
                    with self._stats_lock:
                        self.stats.record_tx(len(final_packet))
            except Exception as e:
                self._log(f"[WRITE_ERR] 串口写入失败: {e}")
                if self.enable_stats:
                    with self._stats_lock:
                        self.stats.tx_frames_failed += 1
                return 0
        
        return len(final_packet)

    def write_structured(self, format_str: str, *values) -> int:
        """发送结构化数据"""
        payload = struct.pack(format_str, *values)
        
        expected_size = self.protocol.SEND_PAYLOAD_SIZE
        if len(payload) < expected_size:
            payload += b'\x00' * (expected_size - len(payload))
        elif len(payload) > expected_size:
            raise ValueError(f"Packed data too large: {len(payload)} > {expected_size}")
        
        return self.write(payload)

    def reset_input_buffer(self):
        """清空输入缓冲"""
        self._log("[RESET] 清空输入缓冲")
        
        with self._buffer_lock:
            self.buffer.clear()
        
        for q in self.queues.values():
            while True:
                try:
                    q.get_nowait()
                except queue.Empty:
                    break
        
        if not self.bluetooth:
            self.ser.reset_input_buffer()

    def reset_output_buffer(self):
        """清空输出缓冲"""
        if not self.bluetooth:
            self.ser.reset_output_buffer()

    def flush(self):
        """刷新输出缓冲"""
        if not self.bluetooth:
            self.ser.flush()

    def is_connected(self) -> bool:
        """检查连接状态"""
        if self.bluetooth:
            return (self._client_ref is not None and 
                    self._client_ref.is_connected)
        else:
            return hasattr(self, 'ser') and self.ser.is_open

    def close(self):
        """关闭连接并释放资源"""
        self._log("[CLOSE] 正在关闭连接...")
        self.running = False
        
        if self.bluetooth:
            if self._loop_ref and self._loop_ref.is_running():
                try:
                    future = asyncio.run_coroutine_threadsafe(
                        self.ble_write_queue.put(None),
                        self._loop_ref
                    )
                    future.result(timeout=1.0)
                except Exception:
                    pass
                
                time.sleep(0.5)
                self._loop_ref.call_soon_threadsafe(self._loop_ref.stop)
                
                if self.bt_loop_thread.is_alive():
                    self.bt_loop_thread.join(timeout=2.0)
        else:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
                self._log("[CLOSE] 串口已关闭")
        
        if self.parse_thread.is_alive():
            self.parse_thread.join(timeout=2.0)
        
        self._log("[CLOSE] 资源已释放")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False
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
