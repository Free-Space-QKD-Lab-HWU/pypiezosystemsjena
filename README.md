# PyPiezosystemsJena

Wrapper/interface to control PSJ NV200D/Net piezo drivers, [documentation available here](https://free-space-qkd-lab-hwu.github.io/pypiezosystemsjena/)


``` python
import pypiezosystemsjena as psj

nv = psj.NV200D(psj.Connection.usb, port="/dev/ttyUSB1")

print(nv)

```

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
