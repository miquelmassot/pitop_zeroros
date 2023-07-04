from math import floor

from pitop import BrakingType, EncoderMotor, ForwardDirection
from pitop.pma.common.encoder_motor_registers import MotorSyncBits, MotorSyncRegisters
from pitop.pma.plate_interface import PlateInterface
from pitop.common.firmware_device import FirmwareDevice, FirmwareDeviceID


class Pitop:
    def __init__(
        self,
        wheel_separation,
        wheel_diameter,
        left_port="M0",
        right_port="M1",
        invert_left=False,
        invert_right=True,
    ):
        # chassis setup
        self.wheel_separation = wheel_separation
        self.wheel_diameter = wheel_diameter

        # Round down to ensure no speed value ever goes above maximum due to rounding issues
        # (resulting in error)
        self.max_motor_speed = (
            floor(min(self.left_motor.max_speed, self.right_motor.max_speed) * 1000)
            / 1000
        )
        self.max_robot_angular_speed = self.max_motor_speed / (
            self.wheel_separation / 2
        )

        left_direction = ForwardDirection.COUNTER_CLOCKWISE
        right_direction = ForwardDirection.CLOCKWISE
        if invert_left:
            left_direction = ForwardDirection.CLOCKWISE
        if invert_right:
            right_direction = ForwardDirection.COUNTER_CLOCKWISE

        # Setup the motors
        self.left_motor = EncoderMotor(
            left_port,
            forward_direction=left_direction,
            braking_type=BrakingType.COAST,
            wheel_diameter=self.wheel_diameter,
            name="Left Motor",
        )

        self.right_motor = EncoderMotor(
            right_port,
            forward_direction=right_direction,
            braking_type=BrakingType.COAST,
            wheel_diameter=self.wheel_diameter,
            name="Right Motor",
        )

        # Motor syncing
        self.__mcu_device = PlateInterface().get_device_mcu()
        self._set_synchronous_motor_movement_mode()

        # make sure the Expansion Plate is attached to the pi-top
        device = FirmwareDevice(FirmwareDeviceID.pt4_expansion_plate)
        version = device.get_sch_hardware_version_major()

    def _set_synchronous_motor_movement_mode(self):
        sync_config = (
            MotorSyncBits[self.left_motor_port].value
            | MotorSyncBits[self.right_motor_port].value
        )
        self.__mcu_device.write_byte(MotorSyncRegisters.CONFIG.value, sync_config)

    def _start_synchronous_motor_movement(self):
        self.__mcu_device.write_byte(MotorSyncRegisters.START.value, 1)

    def _calculate_motor_speeds(self, linear_speed, angular_speed, turn_radius):
        # if angular_speed is positive, then rotation is anti-clockwise in this coordinate frame
        speed_right = (
            linear_speed + (turn_radius + self.wheel_separation / 2) * angular_speed
        )
        speed_left = (
            linear_speed + (turn_radius - self.wheel_separation / 2) * angular_speed
        )

        if (
            abs(speed_right) > self.max_motor_speed
            or abs(speed_left) > self.max_motor_speed
        ):
            factor = self.max_motor_speed / max(abs(speed_left), abs(speed_right))
            speed_right = speed_right * factor
            speed_left = speed_left * factor

        return speed_left, speed_right

    def current_rpm(self):
        return self.left_motor.current_rpm(), self.right_motor.current_rpm()

    def robot_move_rpm(self, rpm_left, rpm_right):
        self.left_motor.set_target_speed(target_speed=rpm_left)
        self.right_motor.set_target_speed(target_speed=rpm_right)
        self._start_synchronous_motor_movement()

    def robot_move(self, linear_speed, angular_speed, turn_radius=0.0):
        # TODO: turn_radius will introduce a hidden linear speed component to the robot,
        # so params are syntactically misleading
        speed_left, speed_right = self._calculate_motor_speeds(
            linear_speed, angular_speed, turn_radius
        )
        self.left_motor.set_target_speed(target_speed=speed_left)
        self.right_motor.set_target_speed(target_speed=speed_right)
        self._start_synchronous_motor_movement()
