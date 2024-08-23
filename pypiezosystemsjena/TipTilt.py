from .NV200D import NV200D
from .NV200D import ControllerID, Connection, LoopMode, ModulationSource
from typing import Dict

__all__ = [
    "TipTilt",
]


class TipTilt:

    def __init__(self,
                 axis_x_connection: ControllerID,
                 axis_y_connection: ControllerID):

        self.axis_x_details = axis_x_connection
        self.axis_y_details = axis_y_connection

        if axis_x_connection.connection_type == Connection.ethernet:
            self.axis_x = NV200D(
                axis_x_connection.connection_type,
                ip_address=axis_x_connection.connection_details.ip_address)
        else:
            self.axis_x = NV200D(
                axis_x_connection.connection_type,
                ip_address=axis_x_connection.connection_details)

        if axis_y_connection.connection_type == Connection.ethernet:
            self.axis_y = NV200D(
                axis_y_connection.connection_type,
                ip_address=axis_y_connection.connection_details.ip_address)
        else:
            self.axis_y = NV200D(
                axis_y_connection.connection_type,
                ip_address=axis_y_connection.connection_details)

    def __del__(self):
        self.axis_x.__del__()
        self.axis_y.__del__()

    def __repr__(self):
        result: str = f"X-Axis {str(self.axis_x)}\n"
        result += f"Y-Axis {str(self.axis_y)}"

        return result

    @property
    def loop_mode(self) -> Dict[str, LoopMode]:
        return {
            self.axis_x_details.label: self.axis_x.PID_mode,
            self.axis_y_details.label: self.axis_y.PID_mode,
        }

    @loop_mode.setter
    def loop_mode(self, mode: LoopMode):
        self.axis_x.PID_mode = mode
        self.axis_y.PID_mode = mode

    @property
    def modulation_source(self) -> ModulationSource:
        return {
            self.axis_x_details.label: self.axis_x.modulation_source,
            self.axis_y_details.label: self.axis_y.modulation_source,
        }

    @modulation_source.setter
    def modulation_source(self, source: ModulationSource):
        self.axis_x.modulation_source = source
        self.axis_y.modulation_source = source

    def go_to_position(self, position_x: float, position_y: float):
        self.axis_x.go_to_position(position_x)
        self.axis_y.go_to_position(position_y)

    def set_position(self, position_x: float, position_y: float):
        self.axis_x.set_position(position_x)
        self.axis_y.set_position(position_y)
