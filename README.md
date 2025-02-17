# Luff-UMB Weather Sensor ROS 2 driver


## Description
<!-- Let people know what your project can do specifically. Provide context and add a link to any reference visitors might be unfamiliar with. A list of Features or a Background subsection can also be added here. If there are alternatives to your project, this is a good place to list differentiating factors. -->

This is a [ROS 2](https://www.github.com/ros2) driver for communicating with Lufft UMB weather sensors via serial port.

## Requirements

The tested ROS distribution is Humble, but it should work with other ROS 2 distributions as well. Any Lufft sensor that supports UMB should work. For connection a RS482 to USB adapter was used. If a weather parameter is missing it can be added in [LufftWSXXX.msg](/lufft_wsx_interfaces/msg/LufftWSXXX.msg).

The driver was successfully tested with the following devices:

- WS501-UMB
- WS600-UMB
- WS100 Radar Precipitation Sensor

Additional resources:

- [Lufft UMB protocol](https://www.lufft.com/download/manual-lufft-umb-protocol-en/)
- [WS series weather sensors manual - from February 2024](https://www.lufft.com/download/operatinal-manual-lufft-wsxxx-weather-sensors/)
- [WS series weather sensors manual - before February 2024](https://www.lufft.com/download/lufft-intelligent-weather-sensors-manual/)


## Installation

Install like any other ROS 2 package. Clone the repository into the `src` folder in your workspace and build it with `colcon build`.

## Usage

Adapt the config file at `lufft_wsx_launch/param/lufft_wsx.yaml` to your needs and run:

```bash
ros2 launch lufft_wsx_launch lufft_wsx.launch.py
```

For a multi-sensor setup check out `lufft_wsx_launch/launch/lufft_wsx_multi.launch.py`.



## Configuration

The driver is configured via a YAML file. The default configuration file is `config/lufft_umb_ws501.yaml`. The following parameters can be set:

- **query_interval** (float): Defines the interval between two queries to the sensor in seconds. Messages will be published at the same interval (i.e.: 10.0).
- **umb_channels** (list): List of the UMB channels to querry separated by a comma (i.e.: [113, 620]). For all available channels of your sensor see weather sensors manual above.
- **device** (str): The port where the sensor is connected (i.e.: "/dev/rs485_adapter_0").
- **baudrate** (int): # Baudrate of that port (i.e.: 19200).
- **device_id** (int): # The device ID of the sensor as configured in the Lufft software (i.e.: 1).



## Contributing

Always open for PRs :)

## Authors and acknowledgment

This project is based on the work of [Pascal Deneaux](https://github.com/Tasm-Devil/lufft-python) who did the heavy lifting regarding the binary protocol translation and [Aleksi Narkilahti](https://github.com/AleksiNarkilahti/lufft-python) who added multichannel queries and an English translation.
