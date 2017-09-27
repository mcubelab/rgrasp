# Loadstar Weight Sensor

Code repo for interfacing to the [Loadstar](http://www.loadstarsensors.com/ "Loadstar's Homepage") weight sensor. This code is primarily composed of a ROS publisher and source code for function calls to read the sensor values. Below, we detail hardware set up and usage. 

## Hardware Setup

The hardware is composed of one DI-40U, a four channel USB load-cell interface, and 4 attached sensors. The interface connects to a computer via USB. Each of the four sensors is a number, printed on the connection point to the interface box. The numbers are four sequential five-digit numbers. 

> For use in the Amazon Picking Challenge we assume that the sensors are laid out in a particular z-shaped order. 

> **Robot Here**

> | Left | Right |
> | ------------- | ------------- |
> | 2nd  | 4th  |
> | 1st  | 3rd |

## Utilizing Hardware

Since the sensors interface via USB, they can be plugged in and read on a personal computer. (*Note the following instructions are for Linux*). 

Plug in the sensors to any USB port. In `/dev` they should appear as `ttyUSB#` for numbers `0-3`. First, your computer needs to be added to their group:

```$ sudo adduser $COMPUTER_NAME dialout```

Some have noted the need to reboot your computer after this command. You will then need to change the permissions to enable you to read from the sensor: 

```$ sudo chown $COMPUTER_NAME /dev/ttyUSB#```

for each of the 4 USBs. You should now be able to read from the sensor, running the publisher (inside the repo) directly via:

```$ python src/ws_publisher.py```

Expected output is a continously printing stream of the sensor output. You can visualize this output using `rqt` plot with the following command: 

```$ rqt_plot rqt_plot```

Which will launch a line plotting interface. To utilize this, you will need a roscore running (`roscore` in a separate terminal). Subscribe to the topics `/ws_stream0` and `/ws_stream1`. 

## Testing Weight Sensor with APC

Note that these instructions are specific to testing the weight sensor system in the APC system. There are two ways to test this. The first is to run the publisher as described above. Alternatively, we can test the weight sensor function calls through the script `src/example_usage.py` which makes calls to `src/ws_prob.py` in a similar way to the APC codebase. 

First, launch `pman`. You will need to launch `roscore`, `1-robotconfig-real` (used to read the weights of objects), and `3-weight sensor`. The output of the weight sensor is printed to pman and can also be read via `rostopic echo -c /ws_stream#` for any of the streams (0-3). The test script can then be run by:

```$ python src/example_usage.py```

Which provides instructions and readout. Additionally this script provides (as the title suggests) example usage of the weight sensor.

## Example Software Usage

Below provides an example of using the weight sensor interface. Note that this can be seen in use in `src/scripts/example_usage.py`.

```
from ws_prob import WeightSensor
weight = WeightSensor()
weight.calibrateWeights(withSensor=True)
ws_data = weight.readWeightSensor(["Duct_Tape"], withSensor=True, binNum=0, givenWeights=sim_weight)
```

We first import and then create an instance of the class. The third line calibrates all of the sensors. The fourth line reads the weight information from the given bin number. For more detailed documentation about the function parameters, see the docstrings of the functions in `src/ws_prob.py`. 
