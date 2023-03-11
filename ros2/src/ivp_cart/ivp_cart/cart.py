import rclpy
from rclpy.node import Node
from rclpy import qos
from geometry_msgs.msg import Vector3

from ivp_cart.odrive_interface import CartControl


class Control(Node):

    def __init__(self):
        super().__init__('cart_control_node')
        self.cart_control: CartControl = CartControl()
        self.subscriber_ = None
        self.pendulum_pos: float = 0.0
        self.pendulum_vel: float = 0.0

    def __del__(self):
        # self.cart_control.set_state_idle()
        print("Drive was set to idle")

    def run_once(self):
        self.get_logger().info(
            f'Pendulum: pos[rad], vel[rad/s]: "{round(self.pendulum_pos, 4)}", "{round(self.pendulum_vel, 4)}"')

    def configure(self):
        self.subscriber_ = self.create_subscription(
            Vector3,
            '/ivp/pendulum_state',
            self.listener_callback,
            qos.qos_profile_sensor_data)

    def prepare_motor(self):
        self.cart_control.connect()
        self.cart_control.calibrate()
        self.cart_control.homing()

    def listener_callback(self, pendulum_state):
        self.pendulum_pos = pendulum_state.x
        self.pendulum_vel = pendulum_state.y
