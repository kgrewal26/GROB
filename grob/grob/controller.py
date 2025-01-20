
import rclpy
from rclpy.node import Node

import numpy as np

from grob_interfaces.msg import Pose, Path, RobotStates
from grob_interfaces.srv import SavedPath

from grob.PID import PID_Control
from grob.helpers import euler_from_quaternion, calculate_angular_error, calculate_linear_error

from geometry_msgs.msg import Twist

class controller(Node):
    def __init__(self, klp = 0.2, klv = 0.2, kli = 0.2, kap = 0.2, kav = 0.2, kai = 0.2):
        super().__init__("controller")
        
        # PID instances to generate velocities based on error
        self.linear_PID = PID_Control(klp, klv, kli, filename_="linear.csv")
        self.angular_PID = PID_Control(kap, kav, kai, filename_="angular.csv")

        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel", qos_profile = 10)

        # Register callback for path and state data
        self.path_subscriber = self.create_subscription(Path, '/desired_path', self.desiredPathCallback, qos_profile=10)
        self.robot_states_subscriber = self.create_subscription(RobotStates, '/robot_states', self.robotStatesCallback, qos_profile=10)

        # Callback for service to return if theres a currently saved path
        self.valid_path_server = self.create_service(SavedPath, 'check_if_saved_path', self.hasValidPath)

        self.desired_path = None
        self.num_waypoints = 0

    def desiredPathCallback(self, path_msg: Path):

        print("Received Path")
        self.desired_path = path_msg.desired_pose
        self.num_waypoints = path_msg.num_pose

    def hasValidPath(self, request: SavedPath.Request, response: SavedPath.Response):
        if ((self.desired_path is not None) and (self.num_waypoints > 0)):
            response.is_path_saved = True
        else:
            response.is_path_saved = False

        print("Received Path Validity Request, Returning:", response.is_path_saved)
        return response

    def robotStatesCallback(self, states_msg: RobotStates):

        num_waypoints = self.num_waypoints
        current_pose = Pose(x = states_msg.x, y = states_msg.y, theta = states_msg.theta)

        if ((num_waypoints > 0) and (self.desired_path != None)):
            print("Following Path")
            # Linear error uses the final goal
            final_pose = self.desired_path[num_waypoints - 1]
            
            # Angular error uses an intermediate waypoint (which may end up just being the final goal)
            goal_pose = self.lookFarFor(current_pose, self.desired_path, num_waypoints)

            e_linear = calculate_linear_error(current_pose, final_pose)
            e_angular = calculate_angular_error(current_pose, goal_pose)

            vel_linear = self.linear_PID.update([e_linear, states_msg.stamp], True)
            vel_angular = self.angular_PID.update([e_angular, states_msg.stamp], True)

            vel = Twist()
            vel.linear.x = vel_linear
            vel.angular.z = vel_angular
            self.vel_publisher.publish(vel)
            

    def lookFarFor(self, pose, listGoals, numGoals):
    
        poseArray=np.array([pose.x, pose.y])
        listGoalsArray=np.zeros((numGoals, 2))

        for goalIndex in range(0, numGoals):
            listGoalsArray[goalIndex] = [listGoals[goalIndex].x, listGoals[goalIndex].y]

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                                axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 1, len(listGoals) - 1) ]

def main(args=None):
    rclpy.init()

    pidController = controller()

    rclpy.spin(pidController)


if __name__ == "__main__":
    main()
