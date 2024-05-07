import serial
import telnetlib
from enum import Enum
from typing import Union, Optional


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
    bit_string = bin(status)
    codes = [i for i in range(15, -1, -1) if bit_string[2::] == "1"]
    return codes.reverse()


def actuator_connected(status_codes: List[int]) -> bool:
    return 0 in codes


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


def socket_write(socket: Union[serial.Serial, telnetlib.Telnet],
                 command: str, args: Optional[str] = None) -> None:
    cmd = command + "," + args + "\r"
    socket.write(cmd.encode("ascii"))


def socket_read(socket: Union[serial.Serial, telnetlib.Telnet],
                command: str, timeout: int) -> str:
    socket_write(socket, command)
    return socket.read_until(b"\n", timeout=timeout).decode()


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

    @property
    def measured_current(self) -> Tuple[float, float]:
        """Measured piezo current on each channel"""
        socket_write(self.connection, "imeas", "0")
        i0 = socket_read(self.connection, "imeas", self.timeout)
        socket_write(self.connection, "imeas", "1")
        i1 = socket_read(self.connection, "imeas", self.timeout)
        return (i0, i1)

    @property
    def control_mode(self) -> ControlMode:
        """Access control mode of NV200D"""
        result: int = int(socket_read(self.connection, "ctrlmode", self.timeout))
        return Connection(result)

    @control_mode.setter
    def control_mode(self, mode: ControlMode):
        socket_write(self.connection, "ctrlmode", f"{mode.value}")

    @property
    def temperature(self) -> float:
        """Heat sink temperature"""
        return float(socket_read(self.connection, "temp", self.timeout))

    @property
    def status(self) -> Status:
        self._status = socket_read(self.connection, "stat", self.timeout)
