from ivp_cart.cart import Control
import rclpy
import time


def main(args=None):
    rclpy.init(args=args)

    cart_control_node = Control()
    cart_control_node.prepare_motor()  # TODO Turn back for motor

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(cart_control_node)

    cart_control_node.configure()

    try:
        while rclpy.ok():
            cart_control_node.run_once()
            executor.spin_once(0.001)
            cart_control_node.sleep()
    except KeyboardInterrupt:
        print("Keyboard interrupt detected")
    finally:
        cart_control_node.set_state_idle()  # TODO Turn back for motor # TODO Program freezes if this is called in the node's destructor (__del__)
        print("ODrive set to state: IDLE")

    cart_control_node.destroy_node()
    # rclpy.shutdown() # Causes problems


if __name__ == '__main__':
    main()
