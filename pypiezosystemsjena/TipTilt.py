from .NV200D import NV200D
from .NV200D import ControllerID, Connection, LoopMode, ModulationSource
from typing import Dict

__all__ = [
    "TipTilt",
]


class TipTilt:

    """
    Combines a pair of NV200D (Net) piezo controllers, providing joint control
    over a both x and y axes.

    Args:
        axis_x_connection (ControllerID): connection details for x-axis
        axis_y_connection (ControllerID): connection details for y-axis

    Examples:
        Connect to two network attached NV200D driving a single mirror mount.
        ``` python
        from pypiezosystemsjena import find_device,  TipTilt

        # i.p. address of network adapter controllers are attached to
        base_ip = "169.254.1.1"

        # MAC addresses of controllers connected to mirror
        mac_addresses = {
            "x": "00:00:00:00:00:0A,
            "y": "00:00:00:00:00:0B,
        }

        # get pairs of matching ip and mac address
        controller_ids = find_device(mac_addresses, base_ip)

        mirror = TipTilt(controller_ids["x"], controller_ids["y"])
        ```

        Set closed-loop control
        ``` python
        from pypiezosystemsjena import LoopMode
        mirror.loop_mod = LoopMode.closed
        ```

        Set a position to move the mirror to. This will depend on the type of 
        control loop that has been chosen, as open loop control (default)
        expects set points to defined as voltages whereas closed-loop control
        allows the user to define set points in terms of positions with units
        defined by the type of piezo device attached to the controllers.
        Examples for both are given below:

        === "Closed-loop control (mrad or µm)"
            ``` python
            from pypiezosystemsjena import LoopMode, ModulationSource

            # positions in mrad or µm require closed loop operation
            mirror.loop_mod = LoopMode.closed
            mirror.modulation_source = ModulationSource.usb_or_ethernet

            pos_x = 0.5
            pos_y = -0.1

            mirror.go_to_position(pos_x, pos_y)
            ```
        === "Open-loop control (V)"
            ``` python
            from pypiezosystemsjena import LoopMode, ModulationSource

            # positions in volts require open loop operation
            mirror.loop_mod = LoopMode.open
            mirror.modulation_source = ModulationSource.usb_or_ethernet

            voltage_x = 10
            voltage_y = 50

            mirror.apply_voltage(voltage_x, voltage_y)
            ```
    """

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
        """
        Control mode both axes are set to
        """
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
        """
        Modulation source for both axes
        """
        return {
            self.axis_x_details.label: self.axis_x.modulation_source,
            self.axis_y_details.label: self.axis_y.modulation_source,
        }

    @modulation_source.setter
    def modulation_source(self, source: ModulationSource):
        self.axis_x.modulation_source = source
        self.axis_y.modulation_source = source

    def go_to_position(self, position_x: float, position_y: float):
        """
        Position to set both axes to
        Args:
            position_x (float): set point for x-axis
            position_y (float): set point for y-axis
        """
        self.axis_x.go_to_position(position_x)
        self.axis_y.go_to_position(position_y)

    def apply_voltage(self, voltage_x: float, voltage_y: float):
        """
        Set voltage applied to both axes
        Args:
            voltage_x (float): set point for x-axis
            voltage_y (float): set point for y-axis
        """
        self.axis_x.set_voltage(voltage_x)
        self.axis_y.set_voltage(voltage_y)
