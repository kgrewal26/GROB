import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from grob_interfaces.msg import RobotStates, Pose, Path, States
from grob_interfaces.srv import SavedPath
from grob.definitions import States_E


class decision_maker(Node):
    def __init__(self):
        super().__init__("decision_maker")

        self.state_publisher = self.create_publisher(States, "/state", qos_profile=10)
        self.requested_state_subscriber = self.create_subscription(States, "/requested_state", self.requestedStateCallback, qos_profile=10)

        self.state_time = self.create_timer(1, self.publishCurrentState)

        # Track current state (scanning is default)
        self.state = States_E.SCANNING
        self.requested_state = States_E.SCANNING

        # For checking if controller is ready to path follow
        self.path_ready_client = self.create_client(SavedPath, 'check_if_saved_path')
        while not self.path_ready_client.wait_for_service(timeout_sec=1.0):
            print("Path Ready Client not available ... is controller running?")
        self.path_ready_req = SavedPath.Request()

    def publishCurrentState(self):
        state_msg = States()
        state_msg.state = int(self.state)

        self.state_publisher.publish(state_msg)

        
    def requestedStateCallback(self, requested_state_msg: States):
        self.requested_state = States_E(requested_state_msg.state)

    def handleStateTransitions(self):
        requested_state = self.requested_state

        valid_transition = False

        if self.state == States_E.SCANNING:
            # Only way to exit scanning is through a planned path
            valid_transition |= (requested_state == States_E.PLANNING)

        elif self.state == States_E.PLANNING:

            # Planning can go back to scanning
            valid_transition |= (requested_state == States_E.SCANNING)

            # Planning ideally goes to navigating - but requires some conditions
            if (requested_state == States_E.NAVIGATING):
                # TODO: check that a path is saved in the controller
                valid_transition |= True

        elif self.state == States_E.NAVIGATING:
            # Navigating can go into replanning (if new obstacle detected), waiting, emptying
            valid_transition |= (requested_state == States_E.PLANNING)
            valid_transition |= (requested_state == States_E.WAITING)
            valid_transition |= (requested_state == States_E.EMPTYING)

        elif self.state == States_E.WAITING:
            # Can return to collecting garbage, or plan path home
            valid_transition |= (requested_state == States_E.SCANNING)
            valid_transition |= (requested_state == States_E.PLANNING)

        elif self.state == States_E.EMPTYING:
            # After emptying, it will start to scan for more garbage
            valid_transition |= (requested_state == States_E.SCANNING)
        
        if (valid_transition):
            self.state = requested_state

        print("In Callback")
        self.path_ready_req.request = True
        future = self.path_ready_client.call_async(self.path_ready_req)
        rclpy.spin_until_future_complete(self, future=future)

        print("Got Response")
        print(future)

def main(args=None):
    rclpy.init()
    DM = decision_maker()

    # Decision maker needs to manager complex state transitions, while pulling data from different services across nodes
    # Makes more sense to just manage it in the main loop, so we have complete control over calling spins().
    try:
        # Main execution loop (handle state transitions here)
        while rclpy.ok():
            DM.handleStateTransitions()
            rclpy.spin_once(DM)
            
    except KeyboardInterrupt:
        print("Shutting Down Node")
    finally:
        DM.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()