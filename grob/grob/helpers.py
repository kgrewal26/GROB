from math import atan2, asin, sqrt

M_PI=3.1415926535

import numpy as np

from grob_interfaces.msg import States
from grob.definitions import States_E

class Logger:
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        self.filename = filename

        with open(self.filename, 'w') as file:
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            vals_str=""
            for value in values_list:
                vals_str+=f"{value}, "
            
            vals_str+="\n"
            
            file.write(vals_str)
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:
            # Skip the header line
            

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table

def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    # just unpack yaw for tb
    return yaw

def calculate_linear_error(current_pose, goal_pose):
        
    return sqrt( (current_pose.x - goal_pose.x)**2 +
                (current_pose.y - goal_pose.y)**2 )


def calculate_angular_error(current_pose, goal_pose):

    # If its an invalid angle, treat it as an unspecified goal angle, and calculate
    invalid_angle = (goal_pose.theta > 2 * M_PI) or (goal_pose.theta < 0)

    if (invalid_angle):
        error_angular= atan2(goal_pose.y-current_pose.y,
                            goal_pose.x-current_pose.x) - current_pose.theta
        
        if error_angular <= -M_PI:
            error_angular += 2*M_PI
        
        
        elif error_angular >= M_PI:
            error_angular -= 2*M_PI
    else:
        error_angular = goal_pose.theta - current_pose.theta
    
    return error_angular

# Helper to request a new state
def request_new_state(request_state_publisher, new_state: States_E):

    newStateMsg = States()
    newStateMsg.state = int(new_state)

    request_state_publisher.publish(newStateMsg)