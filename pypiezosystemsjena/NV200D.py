import serial
import telnetlib
from enum import Enum
from typing import Union, Optional, List, Tuple


class Connection(Enum):
    usb = 1
    ethernet = 2


class ControlMode(Enum):
    pid = 0
    identification = 1
    feedforward = 2
    feedback = 3


class SensorType(Enum):
    actuator_without_position_sensor = 0
    capacitve_sensor = 1
    strain_guage_sensor = 2


class LoopMode(Enum):
    open = 0
    closed = 1


def list_status_codes(status: int) -> List[int]:
    """
    Convert status bitset to list of active status codes

    The NV200D controller maintains its status as a 16bit register of bit flags.\
    For each high bit found in the bit set we return an integer from 0-15 to \
    represent the relevent status code found.

    Args:
        status (int): result of NV200D.status

    Returns:
        (List[int]): list of active codes
    """
    bit_string = "{0:016b}".format(status)
    codes = [i for i in range(15, -1, -1) if bit_string[2::] == "1"]
    codes.reverse()
    return codes


def actuator_connected(status_codes: List[int]) -> bool:
    return 0 in status_codes


def sensor_type(status_codes: List[int]) -> SensorType:
    if 1 in status_codes:
        return SensorType(1)
    elif 2 in status_codes:
        return SensorType(2)
    else:
        return SensorType(0)


def loop_mode(status_codes: List[int]) -> LoopMode:
    if 3 in status_codes:
        return LoopMode(1)
    return LoopMode(0)


def notch_filter_active(status_codes: List[int]) -> bool:
    return 5 in status_codes


def signal_processing_active(status_codes: List[int]) -> bool:
    return 7 in status_codes


def amplifier_channels_bridged(status_codes: List[int]) -> bool:
    return 8 in status_codes


def temperature_too_high(status_codes: List[int]) -> bool:
    return 10 in status_codes


def actuator_error(status_codes: List[int]) -> bool:
    return 11 in status_codes


def hardware_error(status_codes: List[int]) -> bool:
    return 12 in status_codes


def i2c_error(status_codes: List[int]) -> bool:
    return 13 in status_codes


def lower_control_value_limit_reached(status_codes: List[int]) -> bool:
    return 14 in status_codes


def upper_control_value_limit_reached(status_codes: List[int]) -> bool:
    return 14 in status_codes


def current_status(status: int) -> dict:
    """
    Parse NV200D statuse to dictionary of active statuses

    Convert the result of NV200D.status to a dictionary of all status codes \
    that could be available

    Args:
        status (int): result of NV200D.status

    Returns:
        (dict): active statuses

    """
    codes = list_status_codes(status)
    status_dict = {
        "actuator connected": actuator_connected(codes),
        "sensor type": sensor_type(codes),
        "loop mode": loop_mode(codes),
        "notch filter active": notch_filter_active(codes),
        "signal processing active": signal_processing_active(codes),
        "amplifier channels bridged": amplifier_channels_bridged(codes),
        "temperature too high": temperature_too_high(codes),
        "actuator error": actuator_error(codes),
        "hardware error": hardware_error(codes),
        "i2c error": i2c_error(codes),
        "lower control value limit reached": lower_control_value_limit_reached(codes),
        "upper control value limit reached": upper_control_value_limit_reached(codes),
    }

    return status_dict


class NV200D:
    def __init__(self, connection: Connection,
                 port: str = None,
                 ip_address: str = None,
                 mac_address: str = None):

        self.timeout = 0.25

        if connection is Connection.usb:
            if port is None:
                raise ValueError
            self.port = port
            self.baudrate = 115200
            self.connection = serial.Serial(self.port, self.baudrate,
                                            timeout=self.timeout, xonxoff=True)

        if connection is Connection.ethernet:
            if (ip_address is None) or (mac_address is None):
                raise ValueError

            if ip_address is not None:
                self.ip_address = ip_address
            elif mac_address is not None:
                self.mac_address = mac_address

            self.network_port = 23
            self.connection = telnetlib.Telnet(self.ip_address, self.network_port)

        self._status = self.status

    def _socket_write(self, command: str, args: Optional[str] = None):
        cmd = command + "," + args + "\r"
        n: int = self.connection.write(cmd.encode("ascii"))

    def _socket_read(self, command: str) -> str:
        self._socket_write(command)
        return self.connection.read_until(b"\n", timeout=self.timeout).decode()

    def hardware_reset(self) -> None:
        self._socket_write("reset")

    @property
    def fenable(self) -> bool:
        """
        Enable cycling through entire piezo voltage range during startup
        """
        res: int = int(self._socket_read("fenable"))
        if res is 0:
            return False
        return True

    @fenable.setter
    def fenable(self, enable: bool):
        arg = "0"
        if enable:
            arg = "1"
        self._socket_write("fenable", arg)

    @property
    def initial_actuator_position(self) -> int:
        """
        Initial actuator position after power-up
        """
        pos: int = int(self._socket_read("sinit"))
        return pos

    @initial_actuator_position.setter
    def initial_actuator_position(self, position: int):
        if not ((position >= 0) and (position <= 100)):
            raise ValueError("position is in percentage, must be 0 <= position <= 100")
        self._socket_write("sinit", f"{position}")

    def measure(self):
        result = float(self._socket_read("meas"))

    @property
    def measured_current(self) -> Tuple[float, float]:
        """Measured piezo current on each channel"""
        self._socket_write("imeas", "0")
        i0 = self._socket_read("imeas")
        self._socket_write("imeas", "1")
        i1 = self._socket_read("imeas")
        return (i0, i1)

    @property
    def control_mode(self) -> ControlMode:
        """Access control mode of NV200D"""
        result: int = int(self._socket_read("ctrlmode"))
        return Connection(result)

    @control_mode.setter
    def control_mode(self, mode: ControlMode):
        self._socket_write("ctrlmode", f"{mode.value}")

    @property
    def temperature(self) -> float:
        """Heat sink temperature"""
        return float(self._socket_read("temp"))

    @property
    def status(self) -> dict:
        """
        Get the current status of the controller
        """
        self._status = self._socket_read("stat")
        return current_status(self._status)
