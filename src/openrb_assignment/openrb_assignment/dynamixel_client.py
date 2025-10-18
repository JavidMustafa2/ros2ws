# Copyright 2019 The ROBEL Authors.
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

"""Communication using the DynamixelSDK."""

import atexit
import logging
import time
from enum import IntFlag
import re
from typing import Optional, Sequence, Union, Tuple, List, Dict
import threading
import numpy as np
from .utils import Rate, NamedArrayElementDescriptor, find_tty_port
from .dynamixel_registers import *

def dynamixel_cleanup_handler():
    """Cleanup function to ensure Dynamixels are disconnected properly."""
    open_clients = list(DynamixelClient.OPEN_CLIENTS)
    for open_client in open_clients:
        if open_client.port_handler.is_using:
            logging.warning("Forcing client to close.")
        open_client.port_handler.is_using = False
        open_client.disconnect()


def signed_to_unsigned(value: int, size: int) -> int:
    """Converts the given value to its unsigned representation."""
    if value < 0:
        bit_size = 8 * size
        max_value = (1 << bit_size) - 1
        value = max_value + value
    return value


def unsigned_to_signed(value: int, size: int) -> int:
    """Converts the given value from its unsigned representation."""
    bit_size = 8 * size
    if (value & (1 << (bit_size - 1))) != 0:
        value = -((1 << bit_size) - value)
    return value


class HardwareErrorStatus(IntFlag):
    NONE = 0
    INPUT_VOLTAGE_ERROR = 1 << 0
    OVERHEATING_ERROR = 1 << 2
    ELECTRICAL_SHOCK_ERROR = 1 << 4
    OVERLOAD_ERROR = 1 << 5

class OperatingMode(IntFlag):
    CURRENT_CONTROL = 0
    VELOCITY_CONTROL = 1
    POSITION_CONTROL = 3
    MULTI_TURN_POSITION_CONTROL = 4
    CURRENT_BASED_POSITION_CONTROL = 5
    PWM_CONTROL = 16

class DummyDynamixelClient:
    """Dummy client for testing."""

    def __init__(self, motor_ids: Sequence[int], port="Torgtuga", baudrate=1000000, lazy_connect=False):
        self.motor_ids = motor_ids
        self.motor_positions = np.zeros(len(motor_ids))
        self.torque_enabled = np.zeros(len(motor_ids), dtype=bool)
        self.lazy_connect = lazy_connect

    def write_desired_pos(self, motor_ids, motor_positions_rad):
        if len(motor_ids) != len(motor_positions_rad):
            raise ValueError(
                "motor_ids and motor_positions_rad must be the same length"
            )
        self.motor_positions[:] = motor_positions_rad

    def write_desired_current(self, motor_ids, motor_currents_mA):
        pass

    def connect(self):
        print(f"Connected to DUMMY Dynamixel motors: {self.motor_ids}")

    def disconnect(self):
        print(f"Disconnected from DUMMY Dynamixel motors: {self.motor_ids}")

    def set_operating_mode(self, motor_ids, mode: OperatingMode):
        print(
            f"Set operating mode for DUMMY dynamixels with IDs {self.motor_ids}: {mode}"
        )

    def read_pos_vel_cur(self):
        return (
            self.motor_positions.copy(),
            np.zeros(len(self.motor_ids)),
            np.zeros(len(self.motor_ids)),
        )

    def read_status_is_done_moving(self):
        return True

    def set_torque_enabled(self):
        self.torque_enabled[:] = True

    def read_pos(self):
        return self.motor_positions.copy()

class DynamixelClient:
    """Client for communicating with Dynamixel motors.
    NOTE: This only supports Protocol 2.
    """

    # The currently open clients.
    OPEN_CLIENTS = set()

    def __init__(
        self,
        motor_ids: Sequence[int],
        port: str = "",
        baudrate: int = 1000000,
        lazy_connect: bool = False,
        pos_scale: Optional[float] = None,
        vel_scale: Optional[float] = None,
        cur_scale: Optional[float] = None,
        **kwargs,
    ):
        """Initializes a new client.
        Args:
            motor_ids: All motor IDs being used by the client.
            port: The Dynamixel device to talk to.
            baudrate: The Dynamixel baudrate to communicate with.
            lazy_connect: If True, automatically connects when calling a method
                that requires a connection, if not already connected.
            pos_scale: The scaling factor for the positions. This is
                motor-dependent. If not provided, uses the default scale.
            vel_scale: The scaling factor for the velocities. This is
                motor-dependent. If not provided uses the default scale.
            cur_scale: The scaling factor for the currents. This is
                motor-dependent. If not provided uses the default scale.
        """
        import dynamixel_sdk

        self.dxl = dynamixel_sdk

        if port == "":
            port = find_tty_port("dynamixel")

        self.port_name = port
        self.baudrate = baudrate
        self.lazy_connect = lazy_connect

        self.port_rlock = threading.RLock()
        self.port_handler = self.dxl.PortHandler(port)
        self.packet_handler = self.dxl.PacketHandler(PROTOCOL_VERSION)
        available_motor_ids = self.scan((min(motor_ids), max(motor_ids)))
        if not set(motor_ids).issubset(set(available_motor_ids)):
            missing_ids = set(motor_ids) - set(available_motor_ids)
            err_str = f"Not all motor IDs are connected. Missing: {[int(m) for m in missing_ids]}"
            raise ValueError(err_str)
        self.motor_ids = motor_ids

        self._pos_reader = DynamixelPositionReader(
            self,
            self.motor_ids,
            pos_scale=pos_scale if pos_scale is not None else DEFAULT_POS_SCALE,
        )

        self._pos_vel_cur_reader = DynamixelPosVelCurReader(
            self,
            self.motor_ids,
            pos_scale=pos_scale if pos_scale is not None else DEFAULT_POS_SCALE,
            vel_scale=vel_scale if vel_scale is not None else DEFAULT_VEL_SCALE,
            cur_scale=cur_scale if cur_scale is not None else DEFAULT_CUR_SCALE,
        )
        self._moving_status_reader = DynamixelReader(
            self, self.motor_ids, REG_MOVING_STATUS.addr, REG_MOVING_STATUS.len
        )

        self._sync_readers = {}
        self._sync_writers = {}

        self.OPEN_CLIENTS.add(self)
        
        # Start with all motors enabled.
        self.set_torque_enabled(self.motor_ids, True)

    @property
    def is_connected(self) -> bool:
        with self.port_rlock:
            return self.port_handler.is_open

    def scan(self, id_range=(0, 252), timeout=0.01):
        """
        Scan for available Dynamixel motors and return their IDs in sorted order.
        
        Args:
        id_range (tuple): Range of IDs to scan (min, max). Default is (0, 252).
        timeout (float): Timeout for each ping attempt in seconds. Default is 0.05.
        
        Returns:
        list: Sorted list of available Dynamixel motor IDs.
        """
        available_ids = []
        
        # Ensure we're connected
        self.check_connected()
        
        for motor_id in range(id_range[0], id_range[1] + 1):
            with self.port_rlock:
                # Ping the motor
                _, comm_result, error = self.packet_handler.ping(self.port_handler, motor_id)
            
            if comm_result != self.dxl.COMM_SUCCESS:
                # If there's a communication error, we just continue to the next ID
                continue
            
            if error != 0:
                # If there's a protocol error, we just continue to the next ID
                continue
            
            # If we've reached here, the motor responded successfully
            available_ids.append(motor_id)
            
            # Small delay to prevent overwhelming the bus
            time.sleep(timeout)
        
        available_ids = sorted(available_ids)
        print(f"Done scanning. Found: {available_ids}")
        return available_ids

    def connect(self):
        """Connects to the Dynamixel motors.
        NOTE: This should be called after all DynamixelClients on the same
            process are created.
        """
        
        if self.is_connected:
            print("Client is already connected.")
            raise OSError("Client is already connected.")
        
        else:
            print("Connecting to Dynamixel motors...")
        with self.port_rlock:
            if self.port_handler.openPort():
                logging.info("Succeeded to open port: %s", self.port_name)
            else:
                raise OSError(
                    (
                        "Failed to open port at {} (Check that the device is powered "
                        "on and connected to your computer)."
                    ).format(self.port_name)
                )

            if self.port_handler.setBaudRate(self.baudrate):
                logging.info("Succeeded to set baudrate to %d", self.baudrate)
            else:
                raise OSError(
                    (
                        "Failed to set the baudrate to {} (Ensure that the device was "
                        "configured for this baudrate)."
                    ).format(self.baudrate)
                )

    def disconnect(self):
        """Disconnects from the Dynamixel device."""
        if not self.is_connected:
            return
        with self.port_rlock:
            if self.port_handler.is_using:
                logging.error("Port handler in use; cannot disconnect.")
                return
        # Ensure motors are disabled at the end.
        # self.set_torque_enabled(self.motor_ids, False, retries=0)
        with self.port_rlock:
            self.port_handler.closePort()
        if self in self.OPEN_CLIENTS:
            self.OPEN_CLIENTS.remove(self)

    def set_torque_enabled(
        self,
        motor_ids: Sequence[int],
        enabled: bool,
        retries: int = -1,
        retry_interval: float = 0.25,
    ):
        """Sets whether torque is enabled for the motors.
        Args:
            motor_ids: The motor IDs to configure.
            enabled: Whether to engage or disengage the motors.
            retries: The number of times to retry. If this is <0, will retry
                forever.
            retry_interval: The number of seconds to wait between retries.
        """
        remaining_ids = list(motor_ids)
        while remaining_ids:
            with self.port_rlock:
                remaining_ids = self.write_byte(
                    remaining_ids,
                    int(enabled),
                    REG_TORQUE_ENABLE.addr,
                )
            if remaining_ids:
                logging.error(
                    "Could not set torque %s for IDs: %s",
                    "enabled" if enabled else "disabled",
                    str(remaining_ids),
                )
            if retries == 0:
                break
            time.sleep(retry_interval)
            retries -= 1

    def reboot(self, motor_ids: Sequence[int], retries: int = -1, retry_interval: float = 0.25):
        print(f"Rebooting motor IDs: {motor_ids}")
        remaining_ids = list(motor_ids)
        while remaining_ids:
            for motor_id in remaining_ids[:]:
                with self.port_rlock:
                    comm_result, dxl_error = self.packet_handler.reboot(self.port_handler, motor_id)
                    if self.handle_packet_result(comm_result, dxl_error, motor_id, context="reboot"):
                        remaining_ids.remove(motor_id)
            if remaining_ids:
                logging.error(f"Could not reboot IDs: {remaining_ids}")
            if retries == 0:
                break
            time.sleep(retry_interval)
            retries -= 1

    def set_operating_mode(self, motor_ids: Sequence[int], mode: OperatingMode):
        """
        Sets the operating mode for the motors.
        """
        # data in EEPROM area can only be written when torque is disabled
        self.set_torque_enabled(motor_ids, False)
        self.sync_write(
            motor_ids,
            [int(mode)] * len(motor_ids),
            EEREG_OPERATING_MODE.addr,
            EEREG_OPERATING_MODE.len,
        )
        self.set_torque_enabled(motor_ids, True)

    def get_model_and_firmware(self) -> Tuple[List[str], List[int]]:
        """
        Reads the Model Number and Firmware Version from all motors and returns lists of model names and firmware versions for each motor ID.
        
        Returns:
            Tuple[List[str], List[int]]: A tuple containing (model_names, firmware_versions)
        """
        model_ids = self.sync_read(self.motor_ids, EEREG_MODEL_NUMBER.addr, EEREG_MODEL_NUMBER.len)
        firmware_versions = self.sync_read(self.motor_ids, EEREG_FIRMWARE_VER.addr, EEREG_FIRMWARE_VER.len)
        
        id2name_map = {
            1220: "xc330-t288",
            1080: "xc430-t240bb",
            1020: "xm430-w350",
            1120: "xm540-w270",
        }
        model_names = []
        fw_versions = []
        
        for motor_id in self.motor_ids:
            model_number = model_ids[motor_id]
            fw_version = firmware_versions[motor_id]
            
            try:
                model_name = id2name_map[model_number]
            except KeyError:
                print(f"Unknown model number {model_number} for motor ID {motor_id}.\nRegistered models: {id2name_map.values()}")
                raise NotImplementedError()
            
            model_names.append(model_name)
            fw_versions.append(fw_version)
        
        return model_names, fw_versions

    def write_homing_offset(self, motor_ids: Sequence[int], homing_offset: np.ndarray):
        """Writes homing offsets to EEPROM. offset = -1 * position after calibration"""
        assert len(motor_ids) == len(homing_offset)

        self.set_torque_enabled(motor_ids, False)
        # Convert to Dynamixel position space.
        homing_offset = homing_offset / self._pos_vel_cur_reader.pos_scale
        self.sync_write(motor_ids, homing_offset,
                        EEREG_HOMING_OFFSET.addr, EEREG_HOMING_OFFSET.len)
        self.set_torque_enabled(motor_ids, True)

    def read_pos_vel_cur(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Returns the positions, velocities, and currents."""
        return self._pos_vel_cur_reader.read()

    def read_pos(self) -> np.ndarray:
        """Returns the positions, velocities, and currents."""
        return self._pos_reader.read()

    def read_status_is_done_moving(self) -> bool:
        """Returns the last bit of moving status"""
        moving_status = self._moving_status_reader.read().astype(np.int8)
        return np.bitwise_and(
            moving_status, np.array([0x01] * len(moving_status)).astype(np.int8)
        )

    def write_desired_pos(self, motor_ids: Sequence[int], positions: np.ndarray):
        """Writes the given desired positions.
        Args:
            motor_ids: The motor IDs to write to.
            positions: The joint angles in radians to write.
        """
        assert len(motor_ids) == len(positions)
        # Convert to Dynamixel position space.
        positions = positions / self._pos_vel_cur_reader.pos_scale
        self.sync_write(motor_ids, positions, REG_GOAL_POS.addr, REG_GOAL_POS.len)

    def write_desired_current(self, motor_ids: Sequence[int], current: np.ndarray):
        assert len(motor_ids) == len(current)
        self.sync_write(motor_ids, current, REG_GOAL_CURR.addr, REG_GOAL_CURR.len)

    def write_profile_velocity(
        self, motor_ids: Sequence[int], profile_velocity: np.ndarray
    ):
        assert len(motor_ids) == len(profile_velocity)

        self.sync_write(
            motor_ids, profile_velocity, REG_PROF_VEL.addr, REG_PROF_VEL.len
        )

    def write_byte(
        self,
        motor_ids: Sequence[int],
        value: int,
        address: int,
    ) -> Sequence[int]:
        """Writes a value to the motors.
        Args:
            motor_ids: The motor IDs to write to.
            value: The value to write to the control table.
            address: The control table address to write to.
        Returns:
            A list of IDs that were unsuccessful.
        """
        self.check_connected()
        errored_ids = []
        for motor_id in motor_ids:
            with self.port_rlock:
                comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                    self.port_handler, motor_id, address, value
                )
                success = self.handle_packet_result(
                    comm_result, dxl_error, motor_id, context="write_byte"
                )
            if not success:
                errored_ids.append(motor_id)
        return errored_ids

    def write_2byte(
        self,
        motor_ids: Sequence[int],
        value: int,
        address: int,
    ) -> Sequence[int]:
        """Writes a value to the motors.
        Args:
            motor_ids: The motor IDs to write to.
            value: The value to write to the control table.
            address: The control table address to write to.
        Returns:
            A list of IDs that were unsuccessful.
        """
        self.check_connected()
        errored_ids = []
        for motor_id in motor_ids:
            with self.port_rlock:
                comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                    self.port_handler, motor_id, address, value
                )
                success = self.handle_packet_result(
                    comm_result, dxl_error, motor_id, context="write_byte"
                )
            if not success:
                errored_ids.append(motor_id)
        return errored_ids

    def sync_read(self, motor_ids: Sequence[int], address: int, size: int, retries: int = 1) -> Dict[int, int]:
        """
        Reads values from multiple motors simultaneously.

        Args:
            motor_ids: The motor IDs to read from.
            address: The control table address to read from.
            size: The size of the data to read.

        Returns:
            A dictionary mapping motor IDs to their read values.
        """
        with self.port_rlock:
            self.check_connected()
            key = (address, size)
            if key not in self._sync_readers:
                self._sync_readers[key] = self.dxl.GroupSyncRead(
                    self.port_handler, self.packet_handler, address, size
                )
            sync_reader = self._sync_readers[key]

            # Clear any existing parameters
            sync_reader.clearParam()

            # Add parameters for each motor
            for motor_id in motor_ids:
                if not sync_reader.addParam(motor_id):
                    raise OSError(f"Failed to add parameter for motor ID {motor_id}")

            # Perform the sync read
            success = False
            while not success and retries >= 0:
                with self.port_rlock:
                    # if you use the https://github.com/gavincangan/DynamixelSDK/tree/labview_fast_sync branch you can use the fastSyncRead method which is faster
                    # comm_result = sync_reader.fastSyncRead()
                    comm_result = sync_reader.txRxPacket()
                    success = self.handle_packet_result(comm_result, context="syncRead")
                retries -= 1

            # Collect the results
            results = {}
            for motor_id in motor_ids:
                if sync_reader.isAvailable(motor_id, address, size):
                    if sync_reader.data_length < 5:
                        results[motor_id] = sync_reader.getData(motor_id, address, size)
                    else:
                        results[motor_id] = sync_reader.data_dict[motor_id]
                else:
                    results[motor_id] = None
            return results

    def sync_write(
        self,
        motor_ids: Sequence[int],
        values: Sequence[Union[int, float]],
        address: int,
        size: int,
    ):
        """Writes values to a group of motors.
        Args:
            motor_ids: The motor IDs to write to.
            values: The values to write.
            address: The control table address to write to.
            size: The size of the control table value being written to.
        """
        assert size <= 28, "sync_write could fail silently on some motors (e.g. XC330) for larger data sizes. Consider chunking your writes."
        self.check_connected()
        key = (address, size)
        if key not in self._sync_writers:
            self._sync_writers[key] = self.dxl.GroupSyncWrite(
                self.port_handler, self.packet_handler, address, size
            )
        sync_writer = self._sync_writers[key]

        errored_ids = []
        for motor_id, value in zip(motor_ids, values):
            if not isinstance(value, list):
                # Ensure value is a native Python int
                if isinstance(value, float) or isinstance(value, np.floating):
                    value = int(value)
                elif isinstance(value, np.integer):
                    value = int(value)

                if isinstance(value, int):
                    if value < 0:
                        value = signed_to_unsigned(value, size=size)
                    value = value.to_bytes(size, byteorder="little")

            success = sync_writer.addParam(motor_id, value)
            if not success:
                errored_ids.append(motor_id)

        if errored_ids:
            logging.error("Sync write failed for: %s", str(errored_ids))

        with self.port_rlock:
            comm_result = sync_writer.txPacket()
            self.handle_packet_result(comm_result, context="sync_write")

        sync_writer.clearParam()

    def bulk_write(
        self,
        motor_ids: Sequence[int],
        values: Sequence[Union[int, bytes, bytearray, float]],
        addresses: Sequence[int],
        sizes: Sequence[int],
    ):
        """Write values to multiple motors with potentially different addresses."""

        self.check_connected()
        if not hasattr(self, "_bulk_writer"):
            self._bulk_writer = self.dxl.GroupBulkWrite(
                self.port_handler, self.packet_handler
            )
        bulk_writer = self._bulk_writer
        bulk_writer.clearParam()

        errored_ids = []
        for motor_id, value, addr, size in zip(motor_ids, values, addresses, sizes):
            if not isinstance(value, (bytes, bytearray, list)):
                if isinstance(value, float) or isinstance(value, np.floating):
                    value = int(value)
                elif isinstance(value, np.integer):
                    value = int(value)
                if isinstance(value, int):
                    if value < 0:
                        value = signed_to_unsigned(value, size=size)
                    value = value.to_bytes(size, byteorder="little")

            success = bulk_writer.addParam(motor_id, addr, size, value)
            if not success:
                errored_ids.append(motor_id)

        if errored_ids:
            logging.error("Bulk write failed for: %s", str(errored_ids))

        with self.port_rlock:
            comm_result = bulk_writer.txPacket()
            self.handle_packet_result(comm_result, context="bulk_write")

        bulk_writer.clearParam()


    def read_hardware_error_status(self) -> Dict[int, HardwareErrorStatus]:
        """
        Reads the Hardware Error Status from all motors.
        
        Returns:
            A dictionary mapping motor IDs to their HardwareErrorStatus.
        """
        return self.sync_read(self.motor_ids, REG_HARDWARE_ERROR.addr, REG_HARDWARE_ERROR.len)

    def check_connected(self):
        """Ensures the robot is connected."""
        with self.port_rlock:
            if self.lazy_connect and not self.is_connected:
                self.connect()
            if not self.is_connected:
                raise OSError("Must call connect() first.")

    def handle_packet_result(
        self,
        comm_result: int,
        dxl_error: Optional[int] = None,
        dxl_id: Optional[int] = None,
        context: Optional[str] = None,
    ):
        """Handles the result from a communication request."""
        error_message = None
        if comm_result != self.dxl.COMM_SUCCESS:
            error_message = self.packet_handler.getTxRxResult(comm_result)
        elif dxl_error is not None:
            error_message = self.packet_handler.getRxPacketError(dxl_error)
        if error_message:
            if dxl_id is not None:
                error_message = "[Motor ID: {}] {}".format(dxl_id, error_message)
            if context is not None:
                error_message = "> {}: {}".format(context, error_message)
            logging.error(error_message)
            return False
        return True

    def convert_to_unsigned(self, value: int, size: int) -> int:
        """Converts the given value to its unsigned representation."""
        if value < 0:
            max_value = (1 << (8 * size)) - 1
            value = max_value + value
        return value

    def __enter__(self):
        """Enables use as a context manager."""
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, *args):
        """Enables use as a context manager."""
        self.disconnect()

    def __del__(self):
        """Automatically disconnect on destruction."""
        self.disconnect()


class DynamixelReader:
    """Reads data from Dynamixel motors.
    This wraps a GroupBulkRead from the DynamixelSDK.
    """

    def __init__(
        self, client: DynamixelClient, motor_ids: Sequence[int], address: int, size: int
    ):
        """Initializes a new reader."""
        self.client = client
        self.motor_ids = motor_ids
        self.address = address
        self.size = size
        self._initialize_data()

        self.operation = self.client.dxl.GroupBulkRead(
            client.port_handler, client.packet_handler
        )

        for motor_id in motor_ids:
            success = self.operation.addParam(motor_id, address, size)
            if not success:
                raise OSError(
                    "[Motor ID: {}] Could not add parameter to bulk read.".format(
                        motor_id
                    )
                )

    def read(self, retries: int = 1):
        """Reads data from the motors."""
        self.client.check_connected()
        success = False
        while not success and retries >= 0:
            with self.client.port_rlock:
                comm_result = self.operation.txRxPacket()
                success = self.client.handle_packet_result(comm_result, context="read")
            retries -= 1

        # If we failed, send a copy of the previous data.
        if not success:
            return self._get_data()

        errored_ids = []
        for i, motor_id in enumerate(self.motor_ids):
            # Check if the data is available.
            available = self.operation.isAvailable(motor_id, self.address, self.size)
            if not available:
                errored_ids.append(motor_id)
                continue

            self._update_data(i, motor_id)

        if errored_ids:
            logging.error("Bulk read data is unavailable for: %s", str(errored_ids))

        return self._get_data()

    def _initialize_data(self):
        """Initializes the cached data."""
        self._data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        """Updates the data index for the given motor ID."""
        self._data[index] = self.operation.getData(motor_id, self.address, self.size)

    def _get_data(self):
        """Returns a copy of the data."""
        return self._data.copy()


class DynamixelPositionReader(DynamixelReader):
    """Reads motor positions."""

    def __init__(
        self, client: DynamixelClient, motor_ids: Sequence[int], pos_scale: float = 1.0
    ):
        super().__init__(
            client,
            motor_ids,
            address=REG_PRESENT_POS.addr,
            size=REG_PRESENT_POS.len,
        )
        self.pos_scale = pos_scale

    def _initialize_data(self):
        """Initializes the cached data."""
        self._pos_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        """Updates the data index for the given motor ID."""
        pos = self.operation.getData(
            motor_id, REG_PRESENT_POS.addr, REG_PRESENT_POS.len
        )
        pos = unsigned_to_signed(pos, size=4)
        self._pos_data[index] = float(pos) * self.pos_scale

    def _get_data(self):
        """Returns a copy of the data."""
        return self._pos_data.copy()


class DynamixelPosVelCurReader(DynamixelReader):
    """Reads positions and velocities."""

    def __init__(
        self,
        client: DynamixelClient,
        motor_ids: Sequence[int],
        pos_scale: float = 1.0,
        vel_scale: float = 1.0,
        cur_scale: float = 1.0,
    ):
        super().__init__(
            client,
            motor_ids,
            address=SPL_REG_PRESENT_CURR_VEL_POS.addr,
            size=SPL_REG_PRESENT_CURR_VEL_POS.len,
        )
        self.pos_scale = pos_scale
        self.vel_scale = vel_scale
        self.cur_scale = cur_scale

    def _initialize_data(self):
        """Initializes the cached data."""
        self._pos_data = np.zeros(len(self.motor_ids), dtype=np.float32)
        self._vel_data = np.zeros(len(self.motor_ids), dtype=np.float32)
        self._cur_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        """Updates the data index for the given motor ID."""
        cur = self.operation.getData(
            motor_id, REG_PRESENT_CURR.addr, REG_PRESENT_CURR.len
        )
        vel = self.operation.getData(
            motor_id, REG_PRESENT_VEL.addr, REG_PRESENT_VEL.len
        )
        pos = self.operation.getData(
            motor_id, REG_PRESENT_POS.addr, REG_PRESENT_POS.len
        )
        cur = unsigned_to_signed(cur, size=2)
        vel = unsigned_to_signed(vel, size=4)
        pos = unsigned_to_signed(pos, size=4)
        self._pos_data[index] = float(pos) * self.pos_scale
        self._vel_data[index] = float(vel) * self.vel_scale
        self._cur_data[index] = float(cur) * self.cur_scale

    def _get_data(self):
        """Returns a copy of the data."""
        return (self._pos_data.copy(), self._vel_data.copy(), self._cur_data.copy())


class DynamixelIndirectClient(DynamixelClient):

    def __init__(
        self,
        start_update_thread=True,
        ctrl_hz=40,
        *args,
        **kwargs,
    ):
        """In Linux, install setserial and run 
        `sudo setserial /dev/ttyUSB0 low_latency` to reduce latency.
        """
        super().__init__(*args, **kwargs)
        self.start_update_thread = start_update_thread
        self.ctrl_hz = ctrl_hz
        self.registers = None
        self.rw_indices = None
        
        # Set this to True to read RW registers from the motors
        # and overwrite the local state. Otherwise the local state
        # will be preserved in RW registers so they can be written to motors.
        self.do_read_rw_registers = True
        self.standard_setup()

    def standard_setup(self):
        """
        Set up read/write for control table registers.
        """

        reg_names = ["gpos", "ppos",
                     "gvel", "pvel",
                     "gcurr", "pcurr",
                     "gpwm", "ppwm",
                     "posPgain",
                     "ptemp",
                     "pvolt"]

        self.registers = [Reg.NAME_TO_REG[reg_name] for reg_name in reg_names]

        # Sort such that the RW registers move to the end
        self.registers.sort(key=lambda reg: reg.access == "RW")

        self.n_motors = len(self.motor_ids)
        self.n_registers = len(self.registers)
        self.n_readonly_registers = sum(1 for reg in self.registers if reg.access == "R")
        self.n_rw_registers = sum(1 for reg in self.registers if reg.access == "RW")
        print(f"Read-only registers: {self.n_readonly_registers}, Read-write registers: {self.n_rw_registers}")

        self.state_updated = threading.Event()
        self.state_vec = np.zeros((self.n_motors, self.n_registers))
        self.state_lock = threading.RLock()
        self.keep_running = True

        self.rw_registers = [reg for reg in self.registers if reg.access == "RW"]

        self.len_readonly_data = sum(reg.len for reg in self.registers if reg.access == "R")
        self.len_readwrite_data = sum(reg.len for reg in self.registers if reg.access == "RW")

        self.rw_indices = [i for i, reg in enumerate(self.registers) if reg.access == "RW"]
        self.n_rw_registers = len(self.rw_indices)

        for idx, reg in enumerate(self.registers):
            setattr(self.__class__, reg.name, NamedArrayElementDescriptor(reg.name, "state_vec", (slice(None), idx)))

        # Prepare indirect addressing for faster writes. Map the RW registers to
        # the contiguous INDIRECT_DATA region so that all targets can be sent
        # with a single sync_write call.
        self.indirect_rw_addr = SPL_REG_INDIRECT_ADDR.addr
        # Determine INDIRECT_DATA addresses for each motor based on model as it is different for XC330 and XC430 models.
        self.indirect_rw_data_addrs = []
        motor_models, firmware_versions = self.get_model_and_firmware()
        print(f"connected motor models: {motor_models}")
        print(f"firmware versions: {firmware_versions}")
        for model, firmware in zip(motor_models, firmware_versions):
            m = model.lower()
            if "xc330" in m and firmware <= 52:
                # older firmware of XC330 has different INDIRECT_DATA address from other motors
                # was recently modified so it's the same as the other motors
                # https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/
                addr = 208
            else:
                addr = 224
            self.indirect_rw_data_addrs.append(addr)
        self.len_rw_bytes = sum(reg.len for reg in self.rw_registers)

        # Build the address table bytes per motor. Each INDIRECT_ADDR entry
        # consists of a little-endian uint16 representing the source address for
        # one byte of INDIRECT_DATA. If a register is not supported by a motor
        # model, map the address back to the corresponding INDIRECT_DATA byte so
        # that writes are ignored.
        # Important for RW registers, since if data is written to a group of 
        # addresses including unsupported registers, the whole write will fail.
        addr_bytes_per_motor = []
        for motor_idx, model in enumerate(motor_models):
            model_l = model.lower()
            base_addr = self.indirect_rw_data_addrs[motor_idx]
            addr_bytes = b""
            offset_counter = 0
            for reg in self.rw_registers:
                for offset in range(reg.len):
                    if any(
                        re.fullmatch(p.replace("*", ".*"), model_l, re.IGNORECASE)
                        for p in reg.unsupported_motors
                    ):
                        addr = base_addr + offset_counter
                    else:
                        addr = reg.addr + offset
                    addr_bytes += int(addr).to_bytes(2, "little")
                    offset_counter += 1
            addr_bytes_per_motor.append(addr_bytes)
        
        # check that all motors have the same number of RW registers
        assert len(set(len(addr_bytes) for addr_bytes in addr_bytes_per_motor)) == 1

        # Write the address table to all motors once during setup. This allows
        # subsequent writes to use the contiguous INDIRECT_DATA region.
        self.sync_write(
            self.motor_ids,
            addr_bytes_per_motor,
            self.indirect_rw_addr,
            len(addr_bytes_per_motor[0]),
        )

        self.update_thread = threading.Thread(target=self.motor_update_loop, args=(1.0 / self.ctrl_hz,))
        self.update_thread.daemon = True  # Ensure the thread exits when the main program exits
        
        if self.start_update_thread:
            self.update_thread.start()

    def read_state_from_motors(self, overwrite_rw=False) -> np.ndarray:
        """
        Read all indirect data for all Dynamixels using sync_read.
        
        Returns:
            np.ndarray: Array of shape (n_motors, n_registers) with converted values
        """
        data = self.read_control_table(overwrite_rw=overwrite_rw)
        n_reg = self.n_registers if overwrite_rw else self.n_readonly_registers
        with self.state_lock:
            self.state_vec[:, :n_reg] = data
        self.state_updated.set()

    def write_targets_to_motors(self):
        """
        Write to RW registers using sync_write.
        
        Args:
            data: np.ndarray of shape (n_motors, n_rw_registers) with physical values
        """
        with self.state_lock:
            rw_data = self.state_vec[:, self.rw_indices].copy()
        self.write_to_control_table(rw_data)

    def motor_update_loop(self, period_sec: float):
        """
        Continuously read and write motor data at a fixed interval.
        
        Args:
            period_sec: Time in seconds between updates
        """
        self.rate_sleep = Rate(period_sec, context=f"motor_update_loop (consider running `sudo setserial {self.port_name} low_latency`)", warn_probability=0.1)
        while self.keep_running:
            # Read motor data. If it's the first run, read all data,
            # otherwise only read RW data. This is to prevent writing zeros
            # to RW registers when the client is first started.
            self.read_state_from_motors(overwrite_rw=self.do_read_rw_registers)
            self.do_read_rw_registers = False

            # Sleep while other threads potentially update motor targets
            self.rate_sleep.sleep()
            
            # Write motor data
            self.write_targets_to_motors()

            if np.random.rand() < 0.1:
                self.check_for_hw_errors()
    

    def set_operating_mode(self, motor_ids: Sequence[int], mode: OperatingMode):
        """Sets the operating mode for the motors.
        This function needs to be overloaded for the indirect addressing setup
        because we need to reset the local RW registers after setting the operating mode.
        """
        # data in EEPROM area can only be written when torque is disabled
        self.set_torque_enabled(motor_ids, False)
        self.sync_write(
            motor_ids, [int(mode)] * len(motor_ids), EEREG_OPERATING_MODE.addr, EEREG_OPERATING_MODE.len
        )

        # Reset the local RW registers to avoid writing incorrect values.
        self.do_read_rw_registers = True
        wait_counter = 0
        while self.do_read_rw_registers and wait_counter < 10:
            time.sleep(0.1)
            wait_counter += 1
            print("Waiting for RW registers to be updated from motors")

        self.set_torque_enabled(motor_ids, True)

    def write_homing_offset(self, motor_ids: Sequence[int], homing_offset: np.ndarray):
        """Writes homing offsets to EEPROM. offset = -1 * position after calibration. This function
        needs to be overloaded for the indirect addressing setup because we need to reset the local RW registers.
        after homing offset is written to EEPROM to avoid moving the motors to the wrong position.
        """
        assert len(motor_ids) == len(homing_offset)

        self.set_torque_enabled(motor_ids, False)
        print(f"Writing homing offset to EEPROM: {homing_offset}")
        # Convert to Dynamixel position space.
        homing_offset = (homing_offset / self._pos_vel_cur_reader.pos_scale).astype(int).tolist()

        self.sync_write(motor_ids, homing_offset,
                        EEREG_HOMING_OFFSET.addr, EEREG_HOMING_OFFSET.len)

        self.do_read_rw_registers = True
        wait_counter = 0
        while self.do_read_rw_registers and wait_counter < 10:
            time.sleep(0.1)
            wait_counter += 1
            print("Waiting for RW registers to be updated from motors")

        self.set_torque_enabled(motor_ids, True)

    def write_dynamixel_ids(self, motor_ids: Sequence[int], new_ids: Sequence[int]):
        """Writes new dynamixel IDs to EEPROM. IDs are in the range [0, 253]. This function
        needs to be overloaded for the indirect addressing setup because we need to reset the local RW registers.
        after homing offset is written to EEPROM to avoid moving the motors to the wrong position.
        """

        all_new_ids = set(new_ids)
        assert all(0 <= new_id <= 253 for new_id in all_new_ids), "New IDs must be in the range [0, 253]"
        assert len(all_new_ids) == len(new_ids), "New IDs must be unique"
        assert len(all_new_ids) == len(motor_ids), "New IDs must have the same length as motor IDs"

        self.set_torque_enabled(motor_ids, False)

        print_str = "Writing new IDs to EEPROM: \n\tOld ID-> New ID\n"
        for (motor_id, new_id) in zip(motor_ids, new_ids):
            print_str += f"\t{motor_id} -> {new_id}\n"
        print(print_str)

        self.sync_write(motor_ids, new_ids, EEREG_ID.addr, EEREG_ID.len)
        self.motor_ids = new_ids

        self.do_read_rw_registers = True
        wait_counter = 0
        while self.do_read_rw_registers and wait_counter < 10:
            time.sleep(0.1)
            wait_counter += 1
            print("Waiting for RW registers to be updated from motors")

        self.set_torque_enabled(motor_ids, True)

    def stop(self):
        """
        Stop the motor update loop.
        """
        self.keep_running = False
        if self.update_thread.is_alive():
            self.update_thread.join()

    def read_control_table(self, overwrite_rw=False) -> np.ndarray:
        """
        Read all indirect data for all Dynamixels using sync_read.
        
        Returns:
            np.ndarray: Array of shape (n_motors, n_registers) with converted values
        """
        # read everything between start_addr and end_addr
        start_addr = REG_VEL_I_GAIN.addr
        end_addr = REG_PRESENT_TEMP.addr
        data = self.sync_read(self.motor_ids, start_addr, end_addr-start_addr+1)
        # data = self.sync_read(self.motor_ids, SPL_REG_CONTROL_TABLE.addr, SPL_REG_CONTROL_TABLE.len)

        n_reg = self.n_readonly_registers if not overwrite_rw else self.n_registers
        result = np.zeros((self.n_motors, n_reg))

        for i, motor_id in enumerate(self.motor_ids):
            motor_data = data.get(motor_id)
            if motor_data is None:
                continue

            for j, reg in enumerate(self.registers):
                if not overwrite_rw and reg.access == "RW":
                    continue

                start_idx = reg.addr - start_addr
                end_idx = start_idx + reg.len
                value = int.from_bytes(motor_data[start_idx:end_idx], 'little', signed=True)
                if reg.addr == REG_HARDWARE_ERROR.addr:
                    value = HardwareErrorStatus(value)
                else:
                    value = value * reg.conv_factor
                result[i, j] = value
        return result

    def check_for_hw_errors(self):
        """
        Read the hardware error status for all motors and print the status.
        """
        error_statuses = self.read_hardware_error_status()
        errors = []
        for motor_id, error_status in error_statuses.items():
            if error_status:
                errors = (motor_id, error_status)
        if errors:
            print(f"Hardware errors detected: {errors}")

    def write_to_control_table(self, data: np.ndarray):
        """
        Write to RW registers using sync_write.
        
        Args:
            data: np.ndarray of shape (n_motors, n_rw_registers) with physical values
        """
        if not self.registers or self.rw_indices is None:
            raise RuntimeError("Control table not set up. Call standard_setup first.")

        n_motors, n_rw_registers = data.shape
        if n_motors != len(self.motor_ids) or n_rw_registers != len(self.rw_indices):
            raise ValueError("Data shape does not match the number of motors or RW registers")

        # Construct a contiguous byte array for each motor in the order of the
        # RW registers and perform a bulk_write to the INDIRECT_DATA region
        # for each motor. Bulk write allows different addresses per motor.
        values = []
        for i in range(n_motors):
            motor_bytes = b""
            for j, reg in enumerate(self.rw_registers):
                raw_val = int(data[i, j] / reg.conv_factor)
                motor_bytes += raw_val.to_bytes(reg.len, byteorder="little", signed=True)
            values.append(motor_bytes)

        expected_len = self.len_rw_bytes
        assert all(len(v) == expected_len for v in values), "Incorrect value length for indirect write"

        self.bulk_write(
            self.motor_ids,
            values,
            self.indirect_rw_data_addrs,
            [expected_len] * len(self.motor_ids),
        )

# Register global cleanup function.
atexit.register(dynamixel_cleanup_handler)

if __name__ == "__main__":
    import argparse
    import itertools

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-m", "--motors", required=True, help="Comma-separated list of motor IDs."
    )
    parser.add_argument(
        "-d",
        "--device",
        default="",
        help="The Dynamixel device to connect to.",
    )
    parser.add_argument(
        "-b", "--baud", default=3000000, help="The baudrate to connect with."
    )
    parsed_args = parser.parse_args()
    motors = [int(motor) for motor in parsed_args.motors.split(",")]

    with DynamixelIndirectClient(start_update_thread=True, motor_ids=motors, port=parsed_args.device, baudrate=parsed_args.baud, lazy_connect=True) as dxl_client:

        dxl_client.set_operating_mode(motors, OperatingMode.CURRENT_BASED_POSITION_CONTROL)
        dxl_client.set_torque_enabled(motors, True)
        dxl_client.write_profile_velocity(motors, [500] * len(motors))

        torque_enabled = dxl_client.sync_read(motors, REG_TORQUE_ENABLE.addr, REG_TORQUE_ENABLE.len)
        # dxl_client.gcurr = np.full(len(motors), 400.0)

        for _ in range(100):

            do_write = np.random.rand() < 0.1
            if do_write:
                dxl_client.gpos = dxl_client.ppos + np.random.rand(len(motors)) * 0.25
            # dxl_client.write_targets_to_motors()

            # dxl_client.read_state_from_motors()s
            print(f"{dxl_client.ppos}, {dxl_client.pvel}, {dxl_client.pcurr}, {dxl_client.ptemp}, {dxl_client.gpos}")
            # print(f"{dxl_client.tick}, {dxl_client.hwerror}, {dxl_client.ppos}, {dxl_client.pvel}, {dxl_client.pcurr}, {dxl_client.ptemp}, {dxl_client.gpos}")

            time.sleep(0.1)        
        dxl_client.stop()
