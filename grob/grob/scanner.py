import rclpy
from rclpy.node import Node

from grob_interfaces.msg import RobotStates, Pose, Path, States

from grob.definitions import States_E

class scanner(Node):
    def __init__(self):
        super().__init__("scanner")

        # To request a state change once done scanning
        self.request_state_publisher = self.create_publisher(States, "/requested_state", qos_profile=10)

        arbitrary_time = 5 # seconds
        self.scan_timer = self.create_timer(arbitrary_time, self.scanComplete)

        # Update system state info
        self.state_subscriber = self.create_subscription(States, "/state", self.updateState, qos_profile=10)
        self.state = States_E.SCANNING

    def updateState(self, state_msg: States):
        self.state = States_E(state_msg.state)

    def scanComplete(self):

        if (self.state == States_E.SCANNING):
            print("Completed Scan - Requesting Planning")
            
            newStateMsg = States()
            newStateMsg.state = int(States_E.PLANNING)

            self.request_state_publisher.publish(newStateMsg)


def main(args=None):
    rclpy.init()

    scanner_node = scanner()

    rclpy.spin(scanner_node)


if __name__ == "__main__":
    main()