# PyPiezosystemsJena

## About
PyPiezosystemsJena is a Python library to interface with NV200D/Net controllers
from Piezosystems Jena. This has been developed as part of the [Hub Optical 
Ground Station](https://www.quantumcommshub.net/research-community/about-the-hub/phase-2/work-package-5/the-hub-optical-ground-station-hogs/), *HOGS*, at the Free-Space QKD
Lab in Heriot-Watt University. The development of this library is for the
design and control of a fine-steering system for incoming light collected by
the ground-station.

The library wraps the various serial commands required to communicate with the
NV200D controllers and provide a convenient Python class to operate the device.
Full details of the functionality covered and how to use it can be found in the
[api reference](api.md), examples are also included in the api reference.
For a more general usage guide please see the [guides section](cookbook.md).

## Installation
This library is currently not available on PyPi but you can still install it
via pip with the following: 

``` shell
pip install git+https://github.com/Free-Space-QKD-Lab-HWU/pypiezosystemsjena.git
```


## Developing
For developers the following should suffice, clone the repo and install the
library as a development build, this way you can write new features and have
them available the next time you import the library.

Clone the repository:
``` shell
git clone https://github.com/Free-Space-QKD-Lab-HWU/pypiezosystemsjena.git
cd pypiezosystemsjena
```

And then run a development build:
``` shell
python3 -m pip install -e .
```



## Quick Start
A connection to a Piezosystems Jena NV200D/Net can be opened as follows:

``` python
import pypiezosystemsjena as psj

nv = psj.NV200D(psj.Connection.usb, port="/dev/ttyUSB1")

print(nv)
```

If the above is successful you should see the following, good luck!
```
NV200D attached on /dev/ttyUSB0
Device status:
               actuator connected : True
                      sensor type : SensorType.capacitve_sensor
                        loop mode : LoopMode.closed
              notch filter active : False
         signal processing active : True
       amplifier channels bridged : True
             temperature too high : False
                   actuator error : False
                   hardware error : False
                        i2c error : False
lower control value limit reached : False
upper control value limit reached : False
          Temperature (heat sink) : 28.677 C
```
