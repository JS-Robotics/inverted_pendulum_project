from ivp_control.control import Control
import rclpy
import time


def main(args=None):
    rclpy.init(args=args)

    control_node = Control()
    control_node.prepare_motor()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(control_node)

    control_node.configure()

    try:
        while rclpy.ok():
            control_node.run_once()
            executor.spin_once()
            time.sleep(0.001)

    except KeyboardInterrupt:
        print('interrupted')

    finally:
        print("Finally executed")

    control_node.destroy_node()
    # rclpy.shutdown() # Causes problems


if __name__ == '__main__':
    main()
