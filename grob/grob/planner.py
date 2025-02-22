import rclpy
import numpy as np
import matplotlib.pyplot as plt

from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

from grob_interfaces.msg import Pose, Path, States, RobotStates

from grob.definitions import States_E

from grob.map import Map

from grob.rrt_star import RRT_Path

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
        self.state = States_E.PLANNING # TODO: switch back to scanning

        # Read in map data to get occupancy grid
        self.map_subscriber = self.create_subscription(OccupancyGrid, "/map", self.updateMap, qos_profile=10)
        self.map = Map()

        # For keeping track of the current robot states (for our start position in the path)
        self.robot_states_subscriber = self.create_subscription(RobotStates, '/robot_states', self.updateRobotStatesCallback, qos_profile=10)
        self.currentPose = Pose()


    def updateState(self, state_msg: States):
        self.state = States_E(state_msg.state)

    def updateRobotStatesCallback(self, robot_state_msg: RobotStates):
        self.currentPose = Pose(x = robot_state_msg.x, y = robot_state_msg.y, theta = robot_state_msg.theta)

    # Goal points are expressed as cartersian coordinates
    def planPath(self, goalX, goalY):

        startX = self.map.cartesian_to_index_x(self.currentPose.x)
        startY = self.map.cartesian_to_index_y(self.currentPose.y)

        goalX = self.map.cartesian_to_index_x(goalX)
        goalY = self.map.cartesian_to_index_y(goalY)

        path = RRT_Path(startX, startY, goalX, goalY, 100, 100, self.map.resolution, self.map.obstacle_list)
        generated_path = path.createPath()
        
        return generated_path

    def planComplete(self):
        if (self.state == States_E.PLANNING):
            path_indices = self.planPath(-0.5, -1.5) #TODO: Feed in desired coordinates here

            # Convert the path to cartesian coordiantes
            generated_path = []
            for vertex in path_indices:
                point = self.map.index_to_cartersian([vertex.x, vertex.y])
                generated_path.append(point)

            data = Path()
            path_array = []
            for waypoint in generated_path:
                desired_pose = Pose()
                desired_pose.x = float(waypoint[0])
                desired_pose.y = float(waypoint[1])
                desired_pose.theta = 1000.0

                path_array.append(desired_pose)
            
            data.desired_pose = path_array
            data.num_pose = len(path_array)

            self.path_publisher.publish(data)

            print("Published Path")

            newStateMsg = States()
            newStateMsg.state = int(States_E.NAVIGATING)

            self.request_state_publisher.publish(newStateMsg)
            print("Requested State to Navigation")

        

    def updateMap(self, map_msg: OccupancyGrid):
        self.map.update_map(map_msg)

        # obstacleX = self.map.obstacle_list[0]
        # obstacleY = self.map.obstacle_list[1]

        # obstacleXCart = list(map(self.map.index_to_catersian_x, obstacleX))
        # obstacleYCart = list(map(self.map.index_to_catersian_y, obstacleY))

        # print(obstacleX)
        # print(obstacleY)

        # plt.scatter(obstacleXCart, obstacleYCart)
        # plt.show()


def main(args=None):
    rclpy.init()

    planner_node = planner()

    rclpy.spin(planner_node)


if __name__ == "__main__":
    main()