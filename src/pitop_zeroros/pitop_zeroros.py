import zeroros
from zeroros.messages import Twist, Float64
from zeroros.message_broker import MessageBroker

from .pitop_controller import Pitop


class PitopZeroROS:
    def __init__(self):
        # chassis setup
        self.wheel_separation = 0.163
        self.wheel_diameter = 0.065
        self.timer_period = 0.01  # seconds

        # Create ZeroROS broker
        self.broker = MessageBroker()

        # Instance the PiTop driver
        self.controller = Pitop(
            wheel_diameter=self.wheel_diameter, wheel_separation=self.wheel_separation
        )

        # Create twist subscriber
        self.twist_sub = zeroros.Subscriber("/cmd_vel", Twist, self.twist_callback)
        self.left_wheel_rpm_pub = zeroros.Publisher("/left_wheel_rpm", Float64)
        self.right_wheel_rpm_pub = zeroros.Publisher("/right_wheel_rpm", Float64)
        self.timer = zeroros.Timer(self.timer_period, self.timer_callback)

    def twist_callback(self, msg):
        self.get_logger().info(
            'I heard: "%s"' % msg.linear.x
            + ' "%s"' % msg.linear.y
            + ' "%s"' % msg.linear.z
            + ' "%s"' % msg.angular.x
            + ' "%s"' % msg.angular.y
            + ' "%s"' % msg.angular.z
        )
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z
        self.controller.robot_move(linear_speed, angular_speed)

    def timer_callback(self):
        # Publish wheel RPM
        left_wheel_rpm, right_wheel_rpm = self.controller.current_rpm()
        self.left_wheel_rpm_pub.publish(Float64(data=left_wheel_rpm))
        self.right_wheel_rpm_pub.publish(Float64(data=right_wheel_rpm))

    def stop(self):
        self.twist_sub.stop()
        self.timer.stop()


def main():
    print("Starting PitopZeroROS")
    ptzr = PitopZeroROS()

    try:
        while True:
            pass
    except KeyboardInterrupt:
        ptzr.stop()
        print("PitopZeroROS stopped")


if __name__ == "__main__":
    main()
