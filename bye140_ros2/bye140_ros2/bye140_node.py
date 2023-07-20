import re
import rclpy
from rclpy.node import Node
from .bye140_sdk import BackYardGripper
from bye140_msg.srv import CalibrateGripper, MoveTo, GetCalibrated, ShutdownGripper, RestartGripper
from bye140_msg.msg import ByStatus


class BYE140Node(Node):
    def __init__(self):
        super().__init__('bye140_node')
        self.declare_parameter('ip_host', '172.31.1.2')
        self.declare_parameter('ip_port', 9999)
        self.declare_parameter('status_frequency',1.0)

        self.gripper = BackYardGripper(self.get_parameter('ip_host').value, self.get_parameter('ip_port').value)

        self.calibrate_srv = self.create_service(CalibrateGripper, 'calibrate_gripper', self.calibrate_gripper_cb_)
        self.get_calibrated_srv = self.create_service(GetCalibrated, 'get_calibrated', self.get_calibrated_cb_)
        self.restart_srv = self.create_service(RestartGripper, 'restart_gripper', self.restart_gripper_cb_)
        self.shutdown_srv = self.create_service(ShutdownGripper, 'shutdown_gripper', self.shutdown_gripper_cb_)
        self.moveto_srv=self.create_service(MoveTo,'moveto',self.moveto_cb_)
        self.status_publisher=self.create_publisher(ByStatus,'by_status',10)
        self.timer=self.create_timer(1/self.get_parameter('status_frequency').value,self.get_status_cb_)

        self.pattern="\[(.*),(.*),(?P<pos>.*),(?P<spd>.*),(?P<force>.*),(.*),(.*),(.*),(.*)\]\n"


    def calibrate_gripper_cb_(self, request, response):
        self.get_logger().info('calibrating...')
        success = bool(self.gripper.calibrate_gripper())
        self.get_logger().info('calibrated?: %s' % success)
        response.success=success
        return response

    def get_calibrated_cb_(self, request, response):
        calibrated = bool(self.gripper.get_calibrated())
        self.get_logger().info('calibrated?: %s' % calibrated)
        response.calibrated = calibrated
        return response

    def restart_gripper_cb_(self, request, response):
        success = self.gripper.restart()
        self.get_logger().info('restarted: %s' % ('success' if success == 0 else 'failed'))
        response.success=True if success==0 else False
        return response

    def shutdown_gripper_cb_(self, request, response):
        success = self.gripper.shutdown()
        self.get_logger().info('shutdown: %s' % ('success' if success == 0 else 'failed'))
        response.success=True if success==0 else False
        return response

    def moveto_cb_(self, request, response):
        status = self.gripper.moveto(request.position,request.speed,request.acceleration,
                                     request.torque,request.tolerance,request.waitflag)
        response.success=True if status==0 else False
        return response

    def get_status_cb_(self):
        status_str=self.gripper.get_status()
        match=re.match(self.pattern,status_str)
        msg=ByStatus()
        msg.position=float(match.group('pos'))
        msg.speed=float(match.group('spd'))
        msg.force=float(match.group('force'))
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_service = BYE140Node()
    rclpy.spin(minimal_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
