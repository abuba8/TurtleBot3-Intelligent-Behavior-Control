import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist

from controllers.fuzzy_logic import Fuzzifier
from controllers.input_space import InputSpace
from controllers.output_space import OutputSpace
from controllers.coord_layer import CoordLayer
from utils.rules_loader import loadRules

mynode_ = None
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
twstmsg_ = None

# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if ( twstmsg_ != None ):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_
    
    regions_ = {
        #LIDAR readings are anti-clockwise
        'brs':  find_nearest(msg.ranges[180:220]),
        # 'fright': find_nearest (msg.ranges[130:140]),
        # 'front':  find_nearest (msg.ranges[175:185]),
        # 'fleft':  find_nearest (msg.ranges[220:230]),
        # 'left':   find_nearest (msg.ranges[265:275]),
        'fls':  find_nearest (msg.ranges[0:50]),
        'frs':  find_nearest (msg.ranges[280:360]),
        
    }       
    twstmsg_= movement()

    
# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)



FRS = InputSpace('FRS')
BRS = InputSpace('BRS')
FLS = InputSpace('FLS')

obj_re = CoordLayer(0.25, 1.01, 0.30)
obj_oa = CoordLayer(0.0, 0.20, 0.25)

#Basic movement method
def movement():
    global regions_, mynode_, FRS, BRS, FLS, obj_re, obj_oa
    regions = regions_
    msg = Twist()
    
    # get sensor values
    x1 = regions['frs']
    x2 = regions['brs']
    x3 = regions['fls']

    # fuzzify inputs for both behaviours
    lin_x_re, ang_z_re, mapper_re, fr_re = Fuzzifier('right_edge', x1, x2, 'min', FRS, BRS)
    lin_x_oa, ang_z_oa, mapper_oa, fr_oa = Fuzzifier('obstacle_avoid', x3, x1, 'min', FLS, FRS)
    
    # take the min of each behaviour
    d1 = min(x1, x2)
    d2 = min(x1, x3)

    '''
    # For Subsumption architecture uncomment this chunk of code, and comment the code below
    # assign speed
    if d2 < 0.4:
        msg.linear.x = lin_x_oa
        msg.angular.z = ang_z_oa
    else:
        msg.linear.x = lin_x_re
        msg.angular.z = ang_z_re

    return msg
    '''

    # get membership value for each behaviour from the coordination layer
    # right edge has the goal setting behaviour/rising edge/right shoulder
    # obstacle avoidance has the falling edge/left shoulder
    obj_re.getMembershipHigh(d1)
    obj_oa.getMembershipLow(d2)

    # calculate the values
    val_x = ((obj_oa.m_value * lin_x_oa) + (obj_re.m_value * lin_x_re)) / (obj_oa.m_value + obj_re.m_value)
    val_z = ((obj_oa.m_value * ang_z_oa) + (obj_re.m_value * ang_z_re)) / (obj_oa.m_value + obj_re.m_value)
    
    print(f'\nSensor values: {x1}, {x2}, {x3}\n')
    print(f'normal vals right edge: {lin_x_re}, {ang_z_re}')
    print(f'normal vals OA: {lin_x_oa}, {ang_z_oa}')
    print(f'FLC output: {val_x}, {val_z}')

    # set the speed
    msg.linear.x = val_x
    msg.angular.z = val_z

    return msg
    
#used to stop the rosbot
def stop():
    global pub_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub_.publish(msg)


def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')

    # define qos profile (the subscriber default 'reliability' is not compatible with robot publisher 'best effort')
    qos = QoSProfile(
        depth=10,
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    )

    # publisher for twist velocity messages (default qos depth 10)
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # subscribe to laser topic (with our qos)
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Configure timer
    timer_period = 0.2  # seconds 
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Run and handle keyboard interrupt (ctrl-c)
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()  # stop the robot
    except:
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
