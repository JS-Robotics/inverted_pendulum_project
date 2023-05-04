import time

import rclpy.publisher
from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Float32
from std_msgs.msg import UInt8

# from ivp_cart.odrive_interface import CartControl
from ivp_cart.odrive_interface import CartControl

from enum import IntEnum


class States(IntEnum):
    CONFIGURED = 0
    ACTIVE = 1
    HEARTBEAT_ERROR = 2


class Control(Node):

    def __init__(self):
        super().__init__('cart_control_node')
        self.cart_control: CartControl = CartControl()
        self._torque_subscriber: rclpy.node.Subscription = None
        self._turn_publisher: rclpy.node.Publisher = None
        self._status_publisher: rclpy.node.Publisher = None
        self.turn_message: Float32 = Float32()
        self.torque = 0
        self.start_time = None
        self.end_time = None
        self.loop_time = 0.01
        self.heart_beat = None
        self.effective_radius = 19.184/(2*1000)  # d/2 = r [mm], r/1000 --> r[m]
        self.pi = 3.14159265359
        self.heart_beat_threshold = 0.1  # 100ms before it brakes
        self.status_qos = qos.QoSProfile(depth=1,
                                         reliability=qos.QoSReliabilityPolicy.RELIABLE,
                                         history=qos.QoSHistoryPolicy.KEEP_LAST,
                                         durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.state: int = 0
        self.publish_state = False
        self.soft_end_limit = 5.0  # 6

    def run_once(self):
        self.start_time = time.time()
        # self.turn_message.data = 1.23

        self.turn_message.data = -1 * self.cart_control.get_estimated_pos() * 2*self.effective_radius*self.pi  # -1 to make right positive direction, (Is also converted from turns to distance traveled: distance = pi*2r*turns) TODO Turn back for motor
        self._turn_publisher.publish(self.turn_message)
        duration = time.time() - self.heart_beat_threshold
        if duration > self.heart_beat:
            self.torque = 0
            self.set_state(States.HEARTBEAT_ERROR)
            if self.publish_state:
                message = UInt8()
                message.data = States.HEARTBEAT_ERROR
                self._status_publisher.publish(message)
                self.publish_state = False
        else:
            self.set_state(States.ACTIVE)
            if self.publish_state:
                message = UInt8()
                message.data = States.ACTIVE
                self._status_publisher.publish(message)
                self.publish_state = False

        if abs(self.turn_message.data) > self.soft_end_limit:  # Perform soft emergency stop check
            self.torque = 0

        self.cart_control.set_torque(self.torque)

    def sleep(self):
        self.end_time = time.time()
        duration = self.end_time - self.start_time
        if duration < self.loop_time:
            time.sleep(self.loop_time - duration)
        else:
            self.get_logger().warn(f"Time overflow: "
                                   f"Target time: {round(1000 * self.loop_time, 4)}[ms],"
                                   f" Loop time: {round(1000 * duration, 4)}[ms], "
                                   f"Overflow: {round(1000 * (duration - self.loop_time), 4)}[ms]")

    def configure(self):
        self._torque_subscriber = self.create_subscription(Float32,
                                                           'force_setpoint',
                                                           self.torque_callback,
                                                           qos.qos_profile_sensor_data)

        self._turn_publisher = self.create_publisher(Float32, 'cart_position', qos.qos_profile_sensor_data)
        self._status_publisher = self.create_publisher(UInt8, 'status_cart', self.status_qos)
        self.heart_beat = time.time()  # Init timer on init in order to avoid None type error
        self.state = States.CONFIGURED

    def prepare_motor(self):
        self.cart_control.connect()
        self.cart_control.set_pid_parameters()
        self.cart_control.calibrate()
        self.cart_control.homing()
        self.cart_control.set_torque_control()

    def torque_callback(self, force: Float32):
        self.torque = -1 * force.data * self.effective_radius  # -1 to make right direction positive
        self.get_logger().info(f"Received force input: {round(force.data,4)}, converted to Moment: {round(self.torque, 4)}")
        self.heart_beat = time.time()

    def set_state_idle(self):
        self.cart_control.set_state_idle()

    def set_state(self, new_state):
        if new_state is not self.state:
            self.state = new_state
            self.publish_state = True
