# Cookbook

## Connecting to controllers
Communication with the NV200D controller can be handled over either serial (via
a usb connection) or over telnet (via ethernet). The `NV200D` class in this
library provides both methods to connect to these controllers along with a
means to find the correct i.p. address of a controller attached to the same
network your computer is on by searching for the controllers MAC address.
In each of these examples the user is requried to to use the `Connection`
enum class in order to specifiy the correct type of connection that is to be
established.

### Connecting over serial (USB)
As an example, connecting over serial can be done as follows. Here 
`/dev/ttyUSB1` corresponds to serial-over-usb port 1 when using Linux, for 
Windows this is this instead might be `COM1`.

!!! example
    ``` python
    from pypiezosystemsjena import NV200D, Connection

    nv200d = NV200D(Connection.usb, port="/dev/ttyUSB1")
    
    print(nv200d)
        
    ```

### Connecting over the network
Alternatively if the NV200D is attached to the network and its i.p. address is
known then a connection can be made like so.

!!! example
    ``` python
    from pypiezosystemsjena import NV200D, Connection

    nv200d = NV200D(Connection.ethernet, ip_address="192.168.188.71")
    
    print(nv200d)
        
    ```

### Searching via MAC address
As a final option, if the controller is attached to the network but you can
meet the following conditions:

1. You know the MAC address of the controller (found on a sticker on the
underside fo the controller)
2. You know the address of the network that both your computer and the
controller are connected to

If this is the case then you can use the i.p. address of the network and the
MAC address of the controller to establish a connection.

!!! example
    ``` python
    from pypiezosystemsjena import NV200D, Connection

    # i.p. address of network the controller is attached to
    base_ip_address = "192.168.188.71"

    MAC_address = "00:00:00:00:00:0A"

    nv200d = NV200D(
                Connection.ethernet,
                ip_address=base_ip_address,
                mac_address=MAC_address)
    
    print(nv200d)
        
    ```

## Finding controllers on the network

A typical use case of these controllers would involve having multiple of them
connected to the same network. For example the network layout below might look
familiar to you scenario where you have perhaps a pair of mirrors to control
requiring four independent controllers that are all then connected to a router
or an ethernet switch.

``` mermaid
graph TB
    subgraph Network
    B[Router or Switch]
    end
    
    subgraph Mirror 1
    C[X-axis] <-.-> B;
    D[Y-axis] <-.-> B;
    end

    subgraph Mirror 2
    E[X-axis] <-.-> B;
    F[Y-axis] <-.-> B;
    end

    A[My Computer] <---> B[Router or Switch];
```

The problem we face in using the controllers like this is finding the correct
i.p. address of each of them to connect to. Fortunately each controller has a
unique identifier in the form of a MAC address, for the NV200D/Net modules 
you should be able to find this on a sticker on the controllers underside.

This library comes with a utility to match each MAC address with its
corresponding i.p. address on the network. For convenience it also requires
that you supply a label for each of the MAC address you search for to help with
identifying and distibguishing bewtween them later.

This functionality can be used like so:

!!! example
    ``` python

    from pypiezosystemsjena import find_device

    # dictionary of labels and MAC address
    mac_addresses = {
        "x1": "00:00:00:00:00:0A",
        "y1": "00:00:00:00:00:0B",
        "x2": "00:00:00:00:00:0C",
        "y2": "00:00:00:00:00:0D",
    }

    # i.p. address of your network, in this case we have are connected to a
    # switch, if you were instead connected to a router this might instead be    
    # "home" i.e. 192.168.0.1
    base_ip = "169.254.166.227"
    
    controller_ids = find_device(mac_addresses, base_ip)
    print(controller_ids)
    ```

If this was successful you will see the contents of a dictionary, much like the
one shown below.
!!! success
    ```
    {
        'x1':ControllerID(
            label='x1',
            connection_type=<Connection.ethernet: 2>,
            connection_details=NetworkConnection(
                MAC_address='00:00:00:00:00:0A',
                ip_address='169.254.2.2
                )
            ),
        'y1':ControllerID(
            label='y1',
            connection_type=<Connection.ethernet: 2>,
            connection_details=NetworkConnection(
                MAC_address='00:00:00:00:00:0B',
                ip_address='169.254.2.3
                )
            ),
        'x2':ControllerID(
            label='x2',
            connection_type=<Connection.ethernet: 2>,
            connection_details=NetworkConnection(
                MAC_address='00:00:00:00:00:0C',
                ip_address='169.254.2.4
                )
            ),
        'y2':ControllerID(
            label='y2',
            connection_type=<Connection.ethernet: 2>,
            connection_details=NetworkConnection(
                MAC_address='00:00:00:00:00:0D',
                ip_address='169.254.2.5
                )
            ),
    }
    ```


## Tip-Tilt stages
Tip-Tilt stages are a common use for these controllers and require that a pair
of them are used in tandem, one for each axis. Configuration and use of devices
used in this way can be made simpler by use of the `TipTilt` class.
Creating a `TipTilt` object can be done as shown below by searching using the
i.p. addresses of the target devices, here those are found by searching for
their respective MAC addresses.

!!! example
    ``` python

    from pypiezosystemsjena import find_device, LoopMode, ModulationSource, TipTilt

    # dictionary of labels and MAC address
    mac_addresses = {
        "x": "00:00:00:00:00:0A",
        "y": "00:00:00:00:00:0B",
    }

    # i.p. address of your network, in this case we have are connected to a
    # switch, if you were instead connected to a router this might instead be    
    # "home" i.e. 192.168.0.1
    base_ip = "169.254.166.227"
    
    # find our controllers
    controller_ids = find_device(mac_addresses, base_ip)

    # using our "x" and "y" labels we can conveniently construct a TipTilt
    # mirror with correctly aligned axes
    tip_tilt_mirror = TipTilt(controller_ids["x"], controller_ids["y"])
    
    # closed loop so we can set positions in mrad
    tip_tilt_mirror.loop_mode = LoopMode.closed

    # set modulation source as "usb_or_ethernet" to allow serial control of position
    tip_tilt_mirror.modulation_source = ModulationSource.usb_or_ethernet
    ```

The two arguments of the `TipTilt` class are identifiers containing a label,
a connection type and details of the connection. From this it is then made
convenient to for example connect one axis over serial and the other over
ethernet should that be required. The identifier has the following structure:

``` python
@dataclass(frozen=True)
class ControllerID:
    label: str
    connection_type: Connection
    connection_details: Union[str, NetworkConnection]
```

As an example, the `TipTilt` object can be made to move in a circular motion.
For this the motion (or voltage in open-loop operation) ranges are needed for
the axes. In the case of closed-loop operation this is relatively straight
forward as the motion range is symmetric about its centre. Here we define our
angles from 0 to 2$\pi$ and then scale them accordingly. We can then iterate
through each position and send them those acuator positions to the controller.

!!! example
    ``` python
    import numpy as np

    angles = np.linspace(0, 2*np.pi, 1000)
    
    sine = np.sin(angles)
    sine = sine / np.max(sine)
    sine = sine * tip_tilt_mirror.axis_x.upper_motion_range_limit
    
    cosine = np.cos(angles)
    cosine = cosine / np.max(cosine)
    cosine = cosine * tip_tilt_mirror.axis_y.upper_motion_range_limit

    for i in range(angles.size):
        tip_tilt_mirror.go_to_position(sine[i], cosine[i])

    # Alternatively you could use zip instead of tracking an index
    for s, c in zip(sine, cosine): 
        tip_tilt_mirror.go_to_position(s, c)
    ```
