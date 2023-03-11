import time
import odrive
from odrive.enums import *
from odrive.utils import *


class CartControl:

    def __init__(self):
        self.serial = '209034615333'
        self.drive: odrive = None
        self.isConnected: bool = False

    def connect(self):
        """
        Connects to the drive unit on the static serial number: 209034615333
        """
        self.drive = odrive.find_any(serial_number=self.serial)
        self.isConnected = True
        time.sleep(0.25)

    def calibrate(self):
        """
        Performs axis0 calibration sequence. First the encoder index search is performed.
        Then, the encoder offset calibration is performed. The axis0 is set to idle state after calibration.
        """
        self.drive.axis0.requested_state = AxisState.ENCODER_INDEX_SEARCH
        time.sleep(0.25)
        while self.drive.axis0.current_state != AxisState.IDLE:
            time.sleep(0.25)
        time.sleep(0.25)
        self.drive.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
        time.sleep(0.25)
        while self.drive.axis0.current_state != AxisState.IDLE:
            time.sleep(0.25)
        time.sleep(0.25)

    def homing(self):
        """
        Performs axis0 homing sequence. Axis0 has to perform calibrate(self) before homing can be performed.
        The axis0 is set to idle state after homing
        """
        self.drive.axis0.requested_state = AxisState.HOMING
        time.sleep(0.25)
        while self.drive.axis0.current_state != AxisState.IDLE:
            time.sleep(0.25)

    def set_state_idle(self):
        """
        Set axis0 into idle state
        """
        self.drive.axis0.requested_state = AxisState.IDLE
        time.sleep(0.25)

    def set_position_control(self):
        """
        Set axis0 into closed loop position control mode
        """
        self.drive.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
        time.sleep(0.25)
        self.drive.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        time.sleep(0.25)

    def set_velocity_control(self):
        """
        Set axis0 into closed loop velocity control mode
        """
        self.drive.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        time.sleep(0.25)
        self.drive.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        time.sleep(0.25)

    def set_torque_control(self):
        """
        Set axis0 into closed loop torque control mode
        """
        self.drive.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        time.sleep(0.25)
        self.drive.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        time.sleep(0.25)

    def set_position(self, position, rotation_count: int = 8192, step_size: int = 10):
        """
        Set the target position in number of turns
        :param position: Input position [turn]
        :param rotation_count: Encoder counts per revolution (prc). Defaults to 8192
        :param step_size: Iteration step size. Defaults to 10
        """
        start_pos = self.drive.axis0.encoder.pos_estimate
        for i in range(0, rotation_count + step_size, step_size):
            cal_pos = start_pos + i / rotation_count * (position - start_pos)
            self.drive.axis0.controller.input_pos = cal_pos
            time.sleep(0.0001)

    def set_velocity(self, velocity):
        """
        Set the target velocity in turn/s
        :param velocity: Input velocity [turn/s]
        """
        self.drive.axis0.controller.input_vel = velocity

    def set_torque(self, torque):
        """
        Set the target torque in Nm
        :param torque: input torque [Nm]
        """
        self.drive.axis0.controller.input_torque = torque

    def dump_errors(self, clear_errors: bool = False):
        """
        Dumps the current drive, axis0 and axis1 error states in the terminal
        :param clear_errors: (bool) Set true to clear errors in addition
        """
        odrive.utils.dump_errors(self.drive, clear_errors)

    def clear_errors(self):
        """
        Clear all errors on drive and axis
        """
        self.drive.clear_errors()

    def get_drive_error(self):
        """
        Get the current error code on drive
        :return: 0 if no error, else not 0
        """
        return self.drive.error

    def get_axis0_error(self):
        """
        Get the current error code on axis0
        :return: 0 if no error, else not 0
        """
        return self.drive.axis0.error

    def is_connected(self):
        """
        Check if the device is connected to the drive.
        :return: True if the device is connected.
        """
        return self.isConnected
