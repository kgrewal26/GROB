import rclpy
from rclpy.node import Node

from grob_interfaces.msg import Pose, Path, States

from grob.definitions import States_E

class planner(Node):
    def __init__(self):
        super().__init__("planner")

        # To publish a new path to the controller
        self.path_publisher = self.create_publisher(Path, "/desired_path", qos_profile=10)

        # To request a state change once done planning
        self.request_state_publisher = self.create_publisher(States, "/requested_state", qos_profile=10)

        arbitrary_time = 10 # seconds
        self.plan_timer = self.create_timer(arbitrary_time, self.planComplete)

        # Update system state info
        self.state_subscriber = self.create_subscription(States, "/state", self.updateState, qos_profile=10)
        self.state = States_E.SCANNING

    def updateState(self, state_msg: States):
        self.state = States_E(state_msg.state)

    def planComplete(self):

        if (self.state == States_E.PLANNING):
            data = Path()

            desired_pose = Pose()
            desired_pose.x = 1.0
            desired_pose.y = 1.0
            desired_pose.theta = 1000.0

            data.desired_pose = [desired_pose]
            data.num_pose = 1

            self.path_publisher.publish(data)

            print("Published Path")

            newStateMsg = States()
            newStateMsg.state = int(States_E.NAVIGATING)

            self.request_state_publisher.publish(newStateMsg)
            print("Requested State to Navigation")


def main(args=None):
    rclpy.init()

    planner_node = planner()

    rclpy.spin(planner_node)


if __name__ == "__main__":
    main()