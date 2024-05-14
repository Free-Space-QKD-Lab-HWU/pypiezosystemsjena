import serial
import telnetlib
from enum import Enum
from typing import Union, Optional, List, Tuple

__all__ = [
    "Connection", "ControlMode", "SensorType", "LoopMode", "ModulationSource",
    "MonitorSource", "ArbitraryWaveformGeneratorRun", "DataRecorderBuffer",
    "DataRecorderSource", "DataRecorderStartMode", "TriggerInputFunction",
    "TriggerEdge", "SPIMonitor", "SPIControlLoopInterupt", "SPISetpoint",
    "NV200D"
]


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


class ModulationSource(Enum):
    """
    Source of control signal for device setpoint
    """
    usb_or_ethernet = 0
    analogue_in = 1
    spi = 2
    awg = 3


class MonitorSource(Enum):
    position_close_loop = 0
    setpoint = 1
    piezo_voltage = 2
    position_error = 3
    absolute_position_error = 4
    position_open_loop = 5
    piezo_current_1 = 6
    piezo_current_2 = 7


class ArbitraryWaveformGeneratorRun(Enum):
    stop = 0
    start = 1


class DataRecorderBuffer(Enum):
    a = 0
    b = 1


class DataRecorderSource(Enum):
    piezo_position = 0
    setpoint = 1
    piezo_voltage = 2
    position_error = 3
    abs_position_error = 4
    piezo_position_2 = 5
    piezo_current_1 = 6
    piezo_current_2 = 7


class DataRecorderStartMode(Enum):
    off = 0
    set = 1
    grun = 2


class TriggerInputFunction(Enum):
    none = 0
    awg_start = 1
    awg_step = 2
    awg_sync = 3
    ilc_sync = 4
    start_data_recorder = 5


class TriggerEdge(Enum):
    off = 0
    rising = 1
    falling = 2
    both = 3


class TriggerOutSource(Enum):
    position = 0
    set_point = 1


class SPIMonitor(Enum):
    zero = 0
    position_closed_loop = 1
    setpoint = 2
    piezo_voltage = 3
    position_error = 4
    abs_position_error = 5
    position_open_loop = 6
    piezo_current_1 = 7
    piezo_current_2 = 8


class SPIControlLoopInterupt(Enum):
    interal = 0
    spi = 1


class SPISetpoint(Enum):
    hex = 0
    decimal = 1
    stroke_or_voltage = 2


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
    codes = [15 - i for i in range(len(bit_string)) if bit_string[i] == "1"]
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


def check_error(msg: str) -> Optional[str]:
    errors: dict = {
        1: "Error not specified",
        2: "Unknown command",
        3: "Parameter missing",
        4: "Admissible parameter range exceeded",
        5: "Command’s parameter count exceeded",
        6: "Parameter is locked or read only",
        7: "Underload",
        8: "Overload",
        9: "Parameter too low",
        10: "Parameter too high",
    }

    if "error" not in msg:
        return None

    error_idx: int = int(msg.split(",")[1])
    return errors[error_idx]


class NV200D:

    """
    A wrapper for class for the Piezosystems Jena NV200D (Net) piezo controller

    Args:
        connection (Connection): type of connection with device
        port (Optional[str]): serial port
        ip_address (Optional[str]): i.p. address to find device
        mac_address (Optional[str]): mac address to search for

    Examples:
        Connection to an NV200 can be done via either serial or telnet.
        === "Serial Port"
            ``` python
            nv = NV200D(psj.Connection.usb, port="/dev/ttyUSB0")
            ```
        === "I.P. Address"
            ``` python
            nv = NV200D(psj.Connection.ethernet, port="/dev/ttyUSB0")
            ```
        === "Search by MAC address"
            ``` python
            nv = NV200D(psj.Connection.ethernet, mac_address="aa:bb:cc:dd:ee:ff:gg:hh")
            ```
    """

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

    def __del__(self):
        self.connection.close()

    def __repr__(self):
        status = self.status
        longest_key = max([len(k) for k in status.keys()])
        result: str = f"NV200D attached on {self.connection.port}\n"
        result += "Device status:\n"
        for k, v in status.items():
            result += f"{k.rjust(longest_key)} : {v}\n"

        result += "Temperature (heat sink)".rjust(longest_key)
        result += f" : {self.temperature} C\n"

        return result

    def _socket_write(self, command: str, args: Optional[str] = None):

        cmd = command + "\r"

        if args is not None:
            cmd = command + "," + args + "\r"

        n: int = self.connection.write(cmd.encode("ascii"))

    def _socket_read(self, command: str) -> Union[str, list[str]]:
        self._socket_write(command)
        msg: str = self.connection.read_until(b"\n").decode()

        error_message: str = check_error(msg)
        if error_message is not None:
            raise RuntimeError(error_message)

        elements: List[str] = msg.split(",")

        assert command == elements[0]

        if len(elements) == 2:
            return elements[1]

        return elements[1:]

    def hardware_reset(self) -> None:
        self._socket_write("reset")

    @property
    def fenable(self) -> bool:
        """
        Enable cycling through entire piezo voltage range during startup
        """
        res: int = int(self._socket_read("fenable"))
        if res == 0:
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
        result: str = self._socket_read("stat")
        self._status: int = int(result)
        return current_status(self._status)

    @property
    def lower_motion_range_limit(self) -> float:
        """
        Lower motion range limit of piezo
        """
        result: float = float(self._socket_read("posmin"))
        return result

    @property
    def upper_motion_range_limit(self) -> float:
        """
        Upper motion range limit of piezo
        """
        result: float = float(self._socket_read("posmax"))
        return result

    @property
    def lower_voltage_range_limit(self) -> float:
        """
        Lower voltage range limit of piezo
        """
        result: float = float(self._socket_read("avmin"))
        return result

    @property
    def upper_voltage_range_limit(self) -> float:
        """
        Upper voltage range limit of piezo
        """
        result: float = float(self._socket_read("avmax"))
        return result

    @property
    def modulation_source(self) -> ModulationSource:
        """
        Source of signal for setpoint
        """
        result: int = int(self._socket_read("modsrc"))
        return ModulationSource(result)

    @modulation_source.setter
    def modulation_source(self, modulation: ModulationSource):
        self._socket_write("modsrc", f"{modulation.value}")

    @property
    def monitor_source(self) -> MonitorSource:
        """
        Source of data for analogue output
        """
        result: int = int(self._socket_read("monsrc"))
        return MonitorSource(result)

    @monitor_source.setter
    def monitor_source(self, monitor: MonitorSource):
        self._socket_write("monsrc", f"{monitor.value}")

    @property
    def PID_mode(self) -> LoopMode:
        """PID open or closed loop mode"""
        result: int = int(self._socket_read("cl"))
        return LoopMode(result)

    @PID_mode.setter
    def PID_mode(self, mode: LoopMode):
        self._socket_write("cl", f"{mode.value}")

    @property
    def slew_rate(self) -> float:
        """
        Maximum slew rate
        """
        return float(self._socket_read("sr"))

    @slew_rate.setter
    def slew_rate(self, rate: float):
        if (rate < 0.0000008) or (rate > 2000):
            raise ValueError(f"Slew rate must be between 0.0000008 and 2000 %/ms")
        self._socket_write("sr", f"{rate}")

    @property
    def PID_proportional_term(self) -> float:
        return float(self._socket_read("kp"))

    @PID_proportional_term.setter
    def PID_proportional_term(self, prop: float):
        self._socket_write("kp", f"{prop}")

    @property
    def PID_integral_term(self) -> float:
        return float(self._socket_read("ki"))

    @PID_integral_term.setter
    def PID_integral_term(self, prop: float):
        self._socket_write("ki", f"{prop}")

    @property
    def PID_differential_amplification_term(self) -> float:
        return float(self._socket_read("kd"))

    @PID_differential_amplification_term.setter
    def PID_differential_amplification_term(self, prop: float):
        self._socket_write("kd", f"{prop}")

    @property
    def PID_differential_term(self) -> float:
        return float(self._socket_read("tf"))

    @property
    def PID_feedforward_control_amplification(self) -> Tuple[float, float, float]:
        """
        Feed forward control amplification for position, velocity and acceleration

        Returns:
            (Tuple[float, float, float]): A tuple of PID factors in the order (\
            position, velocity, acceleration)
        """
        [x, v, a] = self._socket_read("pcf")
        return (float(x), float(v), float(a))

    @PID_feedforward_control_amplification.setter
    def PID_feedforward_control_amplification(self, factors: Tuple[float, float, float]):
        """
        Feed forward control amplification for position, velocity and acceleration

        Args:
            factors (Tuple[float, float, float]): A tuple of PID factors in the order (\
            position, velocity, acceleration)
        """
        (x, v, a) = factors
        self._socket_write("pcf", f"{x},{v},{a}")

    @property
    def low_pass_filter(self) -> bool:
        result: int = int(self._socket_read("setlpon"))
        if result == 0:
            return False
        return True

    @low_pass_filter.setter
    def low_pass_filter(self, value: bool):
        case: int = 0
        if value:
            case = 1

        self._socket_write("setlpon", f"{case}")

    @property
    def low_pass_filter_frequency(self) -> float:
        self._socket_read("setlpf")

    @low_pass_filter_frequency.setter
    def low_pass_filter_frequency(self, freq: float):
        if (freq < 1) or (freq > 10000):
            raise ValueError("Cut off frequency must be range 1 - 10000Hz")
        self._socket_write("setlpf", f"{freq}")

    @property
    def notch_filter(self) -> bool:
        result: int = int(self._socket_read("notchon"))
        if result == 0:
            return False
        return True

    @notch_filter.setter
    def notch_filter(self, value: bool):
        case: int = 0
        if value:
            case = 1
        self._socket_write("notchon", f"{case}")

    @property
    def notch_filter_frequency(self) -> float:
        return float(self._socket_read("notchf"))

    @notch_filter_frequency.setter
    def notch_filter_frequency(self, value: float):
        if (freq < 1) or (freq > 10000):
            raise ValueError("Notch filter frequency must be range 1 - 10000Hz")
        self._socket_write("notchf", f"{freq}")

    @property
    def notch_filter_bandwidth(self) -> float:
        return float(self._socket_read("notchb"))

    @notch_filter_bandwidth.setter
    def notch_filter_bandwidth(self, value: float):
        if (freq < 1) or (freq > 10000):
            raise ValueError("Notch filter frequency must be range 1 - 10000Hz")
        self._socket_write("notchb", f"{freq}")

    @property
    def measured_position_low_pass_filter(self) -> bool:
        result: int = int(self._socket_read("poslpon"))
        if result == 0:
            return False
        return True

    @measured_position_low_pass_filter.setter
    def measured_position_low_pass_filter(self, value: bool):
        case: int = 0
        if value:
            case = 1
        self._socket_write("poslpon", f"{case}")

    @property
    def measured_position_filter_frequency(self) -> float:
        return float(self._socket_read("poslpf"))

    @measured_position_filter_frequency.setter
    def measured_position_filter_frequency(self, value: float):
        if (freq < 1) or (freq > 10000):
            raise ValueError("Measured position filter frequency must be range 1 - 10000Hz")
        self._socket_write("poslpf", f"{freq}")

    @property
    def awg_run(self) -> ArbitraryWaveformGeneratorRun:
        result: int = int(self._socket_read("grun"))
        return ArbitraryWaveformGeneratorRun(result)

    @awg_run.setter
    def awg_run(self, run: ArbitraryWaveformGeneratorRun):
        self._socket_write("grun", f"{run.value}")

    @property
    def awg_output_start_index(self) -> int:
        return int(self._socket_read("gsarb"))

    @awg_output_start_index.setter
    def awg_output_start_index(self, value: int):
        if (value < 0) or (value > 1023):
            raise ValueError("Must be in range 0 - 1023")
        self._socket_write("gsarb", f"{value}")

    @property
    def awg_output_end_index(self) -> int:
        return int(self._socket_read("gearb"))

    @awg_output_end_index.setter
    def awg_output_end_index(self, value: int):
        if (value < 0) or (value > 1023):
            raise ValueError("Must be in range 0 - 1023")
        self._socket_write("gearb", f"{value}")

    @property
    def awg_cycles(self) -> int:
        self._socket_read("gcarb")

    @awg_cycles.setter
    def awg_cycles(self, cycles: int):
        if (value < 0) or (value > 65535):
            raise ValueError("Must be in range 0 - 65535")
        self._socket_write("gcarb", f"{cycles}")

    @property
    def awg_output_offset_index(self) -> int:
        return int(self._socket_read("goarb"))

    @awg_output_offset_index.setter
    def awg_output_offset_index(self, value: int):
        if (value < 0) or (value > 1023):
            raise ValueError("Must be in range 0 - 1023")
        self._socket_write("goarb", f"{value}")

    @property
    def awg_output_current_index(self) -> int:
        return int(self._socket_read("giarb"))

    @property
    def sampling_factor(self) -> int:
        self._socket_read("gtarb")

    @sampling_factor.setter
    def sampling_factor(self, cycles: int):
        if (value < 0) or (value > 65535):
            raise ValueError("Must be in range 0 - 65535")
        self._socket_write("gtarb", f"{cycles}")

    @property
    def data_recorder_source(self) -> Tuple[DataRecorderBuffer, DataRecorderSource]:
        """doc"""
        a: int = int(self._socket_read("recsrc", "0"))
        b: int = int(self._socket_read("recsrc", "1"))
        return (DataRecorderSource(a), DataRecorderSource(b))

    @data_recorder_source.setter
    def data_recorder_source(self, value: Tuple[DataRecorderBuffer, DataRecorderSource]):
        self._socket_write("recsrc", f"{value[0].value},{value[1].value}")

    @property
    def data_recorder_autostart(self) -> DataRecorderStartMode:
        """doc"""
        result: int = int(self._socket_read("recast"))
        return DataRecorderStartMode(result)

    @data_recorder_autostart.setter
    def data_recorder_autostart(self, mode: DataRecorderStartMode):
        self._socket_write("recast", f"{mode.value}")

    @property
    def data_recorder_stride(self) -> int:
        """doc"""
        result: int = int(self._socket_read("recstr"))
        return result

    @data_recorder_stride.setter
    def data_recorder_buffer_length(self, buffer_length: int):
        if (buffer_length < 0) or (buffer_length > 6144):
            raise ValueError("buffer_length must be in range 0, 6144 ")
        self._socket_write("reclen", f"{buffer_length}")

    @property
    def data_recorder_buffer_length(self) -> int:
        """doc"""
        result: int = int(self._socket_read("reclen"))
        return result

    @data_recorder_stride.setter
    def data_recorder_stride(self, stride: int):
        if (stride < 1) or (stride > 655355):
            raise ValueError("Stride must be in range 1, 65535")
        self._socket_write("recstr", f"{stride}")

    @property
    def recording(self) -> bool:
        """doc"""
        result: int = int(self._socket_read("recrun"))
        if result == 0:
            return False
        return True

    @recording.setter
    def recording(self, value: bool):
        case: int = 0
        if value:
            case = 1
        self._socket_write("recrun", f"{case}")

    def read_data_recorder(self, channel: DataRecorderBuffer,
                           index: Optional[int] = None,
                           length: Optional[int] = None):
        """
        Read the data from the buffer on chosen channel.

        If index and length are not supplied the entire contents of the buffer\
        will read out for the chosen buffer.

        Args:
            channel (DataRecorderBuffer): data buffer
            index (Optional[int]): first value to read out of buffer
            length (Optional[int): number of values to read out of buffer

        Returns:
            (List[float]): list of recorded data
        """

        if index is None and length is None:
            result = self._socket_read("recoutf", f"{channel.value}")
            return result

        if length is None:
            length = 1

        result = self._socket_read("recout", f"{channel.value},{index},{length}")
        return result

    @property
    def trigger_input_function(self) -> TriggerInputFunction:
        """
        Function to run on trigger input
        """
        result: int = int(self._socket_read("trgfkt"))
        return TriggerInputFunction(result)

    @trigger_input_function.setter
    def trigger_input_function(self, trigger_func: TriggerInputFunction):
        self._socket_write("trgfkt", f"{trigger_func.value}")

    @property
    def triggering_edge(self) -> TriggerEdge:
        """
        Which edge to trigger on
        """
        result: int = int(self._socket_read("trgedg"))
        return TriggerEdge(result)

    @triggering_edge.setter
    def triggering_edge(self, edge: TriggerEdge):
        self._socket_write("trgedg", f"{edge.value}")

    @property
    def trigger_out_source(self) -> TriggerOutSource:
        result: int = int(self._socket_read("trgsrc"))
        return TriggerOutSource(result)

    @trigger_out_source.setter
    def trigger_out_source(self, trigger_out: TriggerOutSource):
        self._socket_write("trgsrc", f"{trigger_out.value}")

    @property
    def trigger_output_start(self) -> float:
        result: float = float(self._socket_read("trgss"))
        return result

    @trigger_output_start.setter
    def trigger_output_start(self, value: float):
        self._socket_write("trgss", f"{value.value}")

    @property
    def trigger_output_stop(self) -> float:
        result: float = float(self._socket_read("trgse"))
        return result

    @trigger_output_stop.setter
    def trigger_output_stop(self, value: float):
        self._socket_write("trgse", f"{value.value}")

    @property
    def trigger_output_step(self) -> float:
        result: float = float(self._socket_read("trgsi"))
        return result

    @trigger_output_step.setter
    def trigger_output_step(self, value: float):
        self._socket_write("trgsi", f"{value.value}")

    @property
    def trigger_output_length(self) -> float:
        result: float = float(self._socket_read("trglen"))
        return result

    @trigger_output_length.setter
    def trigger_output_length(self, value: float):
        self._socket_write("trglen", f"{value.value}")

    @property
    def spi_monitor(self) -> SPIMonitor:
        result: int = int(self._socket_read("spisrc"))
        return SPIMonitor(result)

    @spi_monitor.setter
    def spi_monitor(self, value: SPIMonitor):
        self._socket_write("spisrc", f"{value.value}")

    @property
    def spi_control_loop_interupt_source(self) -> SPIControlLoopInterupt:
        """doc"""
        result: int = int(self._socket_read("spitrg"))
        return SPIControlLoopInterupt(result)

    @spi_control_loop_interupt_source.setter
    def spi_control_loop_interupt_source(self, value: SPIControlLoopInterupt):
        self._socket_write("spitrg", f"{value.value}")

    @property
    def spi_read_last_setpoint(self):
        return self._socket_read("spis")

    @spi_read_last_setpoint.setter
    def spi_read_last_setpoint(self, value: SPISetpoint):
        self._socket_write("spis", f"{value.value}")



