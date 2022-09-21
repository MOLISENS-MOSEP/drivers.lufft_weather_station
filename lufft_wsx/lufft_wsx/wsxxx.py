#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import functools

from lufft_wsx_interfaces.msg import LufftWSXXX
from .WS_UMB import WS_UMB
from .sensor_config import CHANNELS_LUT


# Multilevel setattr and getattr functions
def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)

def rgetattr(obj, attr, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)
    return functools.reduce(_getattr, [obj] + attr.split('.'))


class WSXXXNode(Node):
    def __init__(self, channels_lut: dict):
        super().__init__("wsxxx")

        self.channels_lut = channels_lut

        self.declare_parameter("publish_frequency", 1.0)
        self.declare_parameter("umb_channels", list(self.channels_lut.keys())) #[620, 625, 700, 780, 820, 825])
        self.declare_parameter("device", "/dev/rs485_adapter_2")
        self.declare_parameter("baudrate", 19200)
        self.declare_parameter("device_id", 6)

        self.publish_frequency_ = self.get_parameter("publish_frequency").value
        self.umb_channels_ = self.get_parameter("umb_channels").value
        self.device_ = self.get_parameter("device").value
        self.baudrate_ = self.get_parameter("baudrate").value
        self.device_id_ = self.get_parameter("device_id").value

        # Check if umb_channels are declared
        self.check_channel_config()

        self.measurement_publisher_ = self.create_publisher(
            LufftWSXXX, "wsxxx_measurements", 10 
        )  # 10 is que size, like a buffer. If massages are late up to 10 msg will be kept.
        self.create_timer(1 / self.publish_frequency_, self.publish_measurement)
        
        self.get_logger().info(f"WSXXX Node started with UMB channels: {self.umb_channels_} at {self.device_}({self.baudrate_}).")


    def check_channel_config(self):
        for c in self.umb_channels_:
            if c not in self.channels_lut:
                raise ValueError(f"Undefined UMB channel [{c}]. Check if channel is listed in sensor_config.py")


    def publish_measurement(self):

        
        with WS_UMB(device=self.device_, baudrate=self.baudrate_) as umb: 
            # onlineDataQueryMultiOneCall seems to be faster than onlineDataQueryMulti.
            values, statuses = umb.onlineDataQueryMultiOneCall(self.umb_channels_, self.device_id_)

        self.get_logger().debug(f'{values=} {statuses}')
                    
        # Set header fields of message
        msg = LufftWSXXX()
        msg.header.stamp = self.get_clock().now().to_msg() # rospy.Time.now()
        msg.header.frame_id = "LufftWSX"
        
        # Set value fields of message
        for channel, value, status,  in zip(self.umb_channels_, values, statuses):
            if status != 0:
                rsetattr(msg, f'{self.channels_lut[channel]}_valid', False)
                
                # If status other than 36 log the errror. Status 36 means invalid channel: Channel not existing for this 
                # sensor.
                if status != 36:
                    self.get_logger().info(
                        f'None 0 status return from {self.channels_lut[channel]}({channel=}, {value=}, {status=}):'
                    )
                    self.get_logger().info(umb.checkStatus(status))
            else:
                rsetattr(msg, f'{self.channels_lut[channel]}_valid', True)
                rsetattr(msg, self.channels_lut[channel], value)

        self.measurement_publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = WSXXXNode(CHANNELS_LUT)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
