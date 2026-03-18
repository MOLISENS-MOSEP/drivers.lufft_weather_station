# Lufft UMB Weather Sensor ROS 2 Driver

ROS 2 driver for communicating with Lufft UMB weather sensors via serial port (RS485). Publishes structured weather measurements as custom ROS messages.

## Packages

| Package                | Description                                                                   |
| ---------------------- | ----------------------------------------------------------------------------- |
| `lufft_wsx`            | Python ROS 2 node for sensor communication via UMB protocol                   |
| `lufft_wsx_interfaces` | Custom message definitions (temperature, wind, humidity, precipitation, etc.) |
| `lufft_wsx_launch`     | Launch files and parameter configurations                                     |

## Supported Hardware

Any Lufft sensor supporting the UMB protocol should work. A RS485-to-USB adapter is required.

## Requirements

- ROS 2 Humble (should work with other ROS 2 distributions)
- RS485-to-USB adapter

## Installation

Clone into your workspace and build:

```bash
cd ~/molisens_ws/src
git clone https://github.com/MOLISENS-MOSEP/drivers.lufft_weather_station.git drivers/lufft_weather_station
cd ~/molisens_ws
colcon build --packages-select lufft_wsx lufft_wsx_interfaces lufft_wsx_launch
```

## Usage

Single sensor:

```bash
ros2 launch lufft_wsx_launch lufft_wsx.launch.py
```

Multi-sensor setup:

```bash
ros2 launch lufft_wsx_launch lufft_wsx_multi.launch.py
```

## Configuration

Edit `lufft_wsx_launch/param/lufft_wsx.yaml` (single) or `lufft_wsx_launch/param/lufft_wsx_multi.yaml` (multi):

| Parameter        | Example                  | Description                                                           |
| ---------------- | ------------------------ | --------------------------------------------------------------------- |
| `query_interval` | `1.0`                    | Interval between sensor queries (seconds)                             |
| `umb_channels`   | `[113, 620, 825]`        | UMB channel IDs to query (see sensor manual)                          |
| `device`         | `"/dev/rs485_adapter_0"` | Serial port                                                           |
| `baudrate`       | `19200`                  | Serial baudrate                                                       |
| `device_id`      | `1`                      | Sensor address on the RS485 bus (as configured in the Lufft software) |

For available UMB channels, refer to the [WS series weather sensors manual](https://www.lufft.com/download/operatinal-manual-lufft-wsxxx-weather-sensors/).

## Published Topics

| Topic                | Type                              | Description                           |
| -------------------- | --------------------------------- | ------------------------------------- |
| `wsxxx_measurements` | `lufft_wsx_interfaces/LufftWSXXX` | Composite weather measurement message |

The `LufftWSXXX` message contains nested sub-messages: `Temperature`, `Wind`, `Humidity`, `Precipitation`, `AirPressure`, `AirDensity`, `Radiation`, `Enthalpy`. Missing/unqueried fields have their `_valid` flag set to `false`.

## Resources

- [Lufft UMB Protocol](https://www.lufft.com/download/manual-lufft-umb-protocol-en/)
- [WS Series Weather Sensors Manual (Feb 2024+)](https://www.lufft.com/download/operatinal-manual-lufft-wsxxx-weather-sensors/)
- [WS Series Weather Sensors Manual (pre-Feb 2024)](https://www.lufft.com/download/lufft-intelligent-weather-sensors-manual/)

## License

Apache 2.0

## Acknowledgments

Based on work by [Pascal Deneaux](https://github.com/Tasm-Devil/lufft-python) (UMB protocol implementation) and [Aleksi Narkilahti](https://github.com/AleksiNarkilahti/lufft-python) (multichannel queries, English translation).
