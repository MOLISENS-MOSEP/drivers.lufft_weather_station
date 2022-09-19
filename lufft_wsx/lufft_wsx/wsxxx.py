#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import functools

from lufft_wsx_interfaces.msg import LufftWSXXX
from .WS_UMB import WS_UMB
from .sensor_config import CHANNELS


# Multilevel setattr and getattr functions
def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)

def rgetattr(obj, attr, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)
    return functools.reduce(_getattr, [obj] + attr.split('.'))


class WSXXXNode(Node):
    def __init__(self):
        super().__init__("wsxxx")

        self.declare_parameter("publish_frequency", 1.0)
        self.declare_parameter("umb_channels", [113]) #[620, 625, 700, 780, 820, 825])
        self.declare_parameter("device", "/dev/ttyUSB2")
        self.declare_parameter("baudrate", 19200)
        self.declare_parameter("device_id", 6)

        self.publish_frequency_ = self.get_parameter("publish_frequency").value
        self.umb_channels_ = self.get_parameter("umb_channels").value
        self.device_ = self.get_parameter("device").value
        self.baudrate_ = self.get_parameter("baudrate").value
        self.device_id_ = self.get_parameter("device_id").value

        self.measurement_publisher_ = self.create_publisher(
            LufftWSXXX, "wsxxx_measurements", 10 
        )  # 10 is que size, like a buffer. If massages are late up to 10 msg will be kept.
        self.create_timer(1 / self.publish_frequency_, self.publish_measurement)
        
        self.get_logger().info(f"WSXXX Node started with UMB channels: {self.umb_channels_} at {self.device_}({self.baudrate_}).")


    def publish_measurement(self):

        # TODO: Change to WS_UMB lib
        with WS_UMB(device=self.device_, baudrate=self.baudrate_) as umb: 
            values, statuses = umb.onlineDataQueryMulti(self.umb_channels_, self.device_id_) # TODO: check if  onlineDataQueryMultiOneCall makes a difference
        #! Dummy values:
        # values, statuses = ([1., 2., 3, 4, 5., 6.], [0, 0, 0, 0, 1, 0])

        self.get_logger().debug(f'{values=} {statuses}')
                    
        # Set header fields of message
        msg = LufftWSXXX()
        msg.header.stamp = self.get_clock().now().to_msg() # rospy.Time.now()
        msg.header.frame_id = "LufftWSX"
        
        # Set value fields of message
        for channel, value, status,  in zip(self.umb_channels_, values, statuses):
            if status != 0:
                self.get_logger().info(f'None 0 status return from {CHANNELS[channel]}(channel: {channel}):')
                # TODO: Change to WS_UMB lib
                self.get_logger().info(umb.checkStatus(status))
                valid = False
            else:
                valid = True

            rsetattr(msg, f'{CHANNELS[channel]}_valid', valid)  # Check if channel is defined in device config at beginning
            rsetattr(msg, CHANNELS[channel], value)

        self.measurement_publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = WSXXXNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
