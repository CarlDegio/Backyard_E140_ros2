import rclpy
from rclpy.node import Node
from bye140_sdk import BackYardGripper
from bye140_msg.srv import CalibrateGripper, MoveTo, GetCalibrated, ShutdownGripper, RestartGripper
from bye140_msg.msg import ByStatus


class BYE140Node(Node):
    def __init__(self):
        super().__init__('bye140_node')
        self.declare_parameter('ip_host', '192.168.137.9')
        self.declare_parameter('ip_port', 9999)

        self.gripper = BackYardGripper(self.get_parameter('ip_host').value, self.get_parameter('ip_port').value)

        self.calibrate_srv = self.create_service(CalibrateGripper, 'CalibrateGripper', self.calibrate_gripper_cb_)
        self.get_calibrated_srv = self.create_service(GetCalibrated, 'GetCalibrated', self.get_calibrated_cb_)
        self.restart_srv = self.create_service(RestartGripper, 'RestartGripper', self.restart_gripper)
        self.shutdown_srv = self.create_service(ShutdownGripper, 'ShutdownGripper', self.shutdown_gripper)

    def calibrate_gripper_cb_(self, request, response):
        self.get_logger().info('calibrating...')
        success = bool(self.gripper.calibrate_gripper())
        self.get_logger().info('calibrated?: %s' % success)
        return response

    def get_calibrated_cb_(self, request, response):
        calibrated = bool(self.gripper.get_calibrated())
        self.get_logger().info('calibrated?: %s' % calibrated)
        return response

    def restart_gripper(self, request, response):
        success = self.gripper.restart()
        self.get_logger().info('restarted: %s' % ('success' if success == 0 else 'failed'))
        return response

    def shutdown_gripper(self, request, response):
        success = self.gripper.shutdown()
        self.get_logger().info('shutdown: %s' % ('success' if success == 0 else 'failed'))
        return response


def main(args=None):
    rclpy.init(args=args)
    minimal_service = BYE140Node()
    rclpy.spin(minimal_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
