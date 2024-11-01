'''
For Simulator
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

For robot
ros2 launch turtlebot3_bringup robot.launch.py
and run the python script on the other terminal
'''


#TESTED ON CSEEBURGER03


import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import numpy as np
# import pandas as pd
from itertools import product

class InputSpace():
    '''
    Step 1a:
    Initializing the input membership functions where the values of close, medium, and far are defined using range
    Then I've calculated a threshold called centroid for each of the membership function
    Also, I've set a attribute name, so it can be checked which sensor input is being used/to maintain dictionaries 
    '''
    def __init__(self, name):
        self.name = name
        # setting close range and threshold
        self.c_range = np.arange(0.0, 0.41, 0.1)
        self.close_centroid = np.round(np.mean(self.c_range), decimals=2)

        # setting medium range and threshold
        self.m_range = np.arange(self.close_centroid, 0.651, 0.1)
        self.medium_centroid = np.round(np.mean(self.m_range), decimals=2)

        # setting far range and threshold
        self.f_range = np.arange(self.medium_centroid, 0.81, 0.1)
        self.far_centroid = np.round(np.mean(self.f_range), decimals=2)
        
class OutputSpace():
    '''
    Step 1b:
    Initializing the output membership functions where the values of linear speed x and angular velocity z are defined
    '''
    def __init__(self, out):
        # check if the out receives x or z and set the speeds
        if out == 'x':
            self.slow = 0.06
            self.medium = 0.09
            self.fast = 0.12
        elif out == 'z':
            self.right = -0.5
            self.forward = 0.0
            self.left = 0.5
        else:
            return 'pass the correct argument'
        

def loadRules(path):
    '''
    Step 1c:
    Loading the pre-defined rules in a csv file 
    '''
    return np.genfromtxt(path, delimiter=',', dtype=str)


def crispInputs(obj, name):
    '''
    Step 3:
    Firstly, I'm checking if the length of membership function is greater than 1
    it means the inputs lies between falling and rising edge 
    As, mentioned in determine_vals function the first index is the falling edge, and second index is rising edge
    I've calculated the edges and made another key in the dictionary to store the values
    For the left, right shoulder and value = 0.5, I'm adding the value of 1
    '''
    if len(obj[name]['membership_func']) > 1:
        falling_edge = round((obj[name]['centroids'][1] - obj[name]['input']) \
            / (obj[name]['centroids'][1] - obj[name]['centroids'][0]), 2)
        
        rising_edge = round((obj[name]['input'] - obj[name]['centroids'][0]) \
            / (obj[name]['centroids'][1] - obj[name]['centroids'][0]), 2)
        
        obj[name]['crisp_inputs'] = [falling_edge, rising_edge]
        
    else:
        obj[name]['crisp_inputs'] = [1]
    return obj


def ruleBase(obj, inputs):
    '''
    Step 2+3:
    This is the crux of the whole assignment
    combs, cents are 2 lists used to note down the membership function and its centroid values respectively
    combs: Also keeps track of left shoulder, right shoulder, falling edge and rising edge
           Whenever there is a falling or rising edge condition, I've appended the list, 
           which means the first index is falling edge, and second index is rising
    mapper: keeps track of sensor's (object), inputs, membership function, and centroid values 
            which can be used to calculate the crisp values
    '''
    combs = []
    cents = []
    mapper = {}
    if inputs == 0.5:
        # print(f'exactly in middle {inputs}, medium -> 1')
        combs.extend(['medium'])
        cents.extend([obj.medium_centroid])
    elif inputs <= obj.close_centroid:
        # print(f'left shoulder: {inputs}, close -> 1')
        combs.extend(['close'])
        cents.extend([obj.close_centroid])
    elif inputs >= obj.far_centroid:
        # print(f'right shoulder {inputs}, far -> 1')
        combs.extend(['far'])
        cents.extend([obj.far_centroid])
    elif inputs > obj.close_centroid and inputs < obj.medium_centroid:
        # print(f'{inputs}, falling edge for close, rising edge for medium')
        combs.extend(['close', 'medium'])  
        cents.extend([obj.close_centroid, obj.medium_centroid])  
    elif inputs > obj.medium_centroid and inputs < obj.far_centroid:
        # print(f'{inputs}, falling edge for medium, rising edge for far')
        combs.extend(['medium', 'far'])
        cents.extend([obj.medium_centroid, obj.far_centroid])
    else:
        print('Do nothing')
        
    mapper[obj.name] = {'input': inputs, 'membership_func': combs, 'centroids': cents}
    mapper = crispInputs(mapper, obj.name)
    return mapper



def getCombos(obj1, obj2, sensor1, sensor2, key):
    return list(product(obj1[sensor1][key], obj2[sensor2][key]))    


def firingStrength(obj1, obj2, path, n_sens_1, n_sens_2, cond='min'):
    '''
    Step 4:
    In this function, I'm getting all the combinations for FRS, and BRS sensors also with there crisp_inputs using the dictionary
    Creating a new dictionary, and setting the key values as rule number and calculating the firing rate value 
    For firing rate, there are two options implemented, to get minimum value or get product
    Also get the output Space and its respective values for c/m/f and r/f/l for both sensors and map them in dataframe
    Afterwards, append them to dictionary, so it can be used to calculate the weighted sum for x and z
    '''
    mapper = dict()
    combinations = getCombos(obj1, obj2, n_sens_1, n_sens_2, 'membership_func')
    combination_vals = getCombos(obj1, obj2, n_sens_1, n_sens_2, 'crisp_inputs')
    
    
    # Here we're reading the rules from a csv file and mapping the input combinations to the rules file
    # and getting the complete rows
    rules = loadRules(path)
    combinations = np.array(combinations)
    
    subset_rules = rules[1:, :2] # getting only first two columns of each row of rules (apart from header FRS BRS x z)  

    matches = np.all(subset_rules[:, np.newaxis, :] == combinations, axis=2)
    matching_indices = np.where(matches)
    _results = rules[matching_indices[0] + 1]
        
    
    linear_x = OutputSpace('x')
    angular_z = OutputSpace('z')

   
    x = [vars(linear_x)[value] for value in _results[:,2]]
    z = [vars(angular_z)[value] for value in _results[:,3]]
    

    # this is the implementation for fuzzification of min and prod
    for i in range(len(combinations)):
        n = 'rule'+str(i+1)
        if cond == 'min':
            mapper[n] = {'combination': combinations[i].tolist(), 'inputs': combination_vals[i], \
                        'firing_rate': np.min(combination_vals[i]),\
                        'rule_x': _results[i,2], 'rule_z': _results[i,3], '.x': x[i], '.z': z[i]}
            
        elif cond == 'prod':
            result = 1
            for value in combination_vals[i]:
                result *= value
            mapper[n] = {'combination': combinations[i].tolist(), 'inputs': combination_vals[i], \
                        'firing_rate': result,\
                        'rule_x': _results[i,2], 'rule_z': _results[i,3], '.x': x[i], '.z': z[i]}
    
    return mapper




def defuzzification(obj, col, firing_rate):
    '''
    Step 5: In defuzzification, we calculated the membership function values * firing rates divided by the firing rates
    '''
    weights = [rule[col] * rule[firing_rate] for rule in obj.values()]
    firing_rates = [rule[firing_rate] for rule in obj.values()]
    return sum(weights) / sum(firing_rates)



def Fuzzifier(logic, input_1, input_2, firing_option, sensor_1, sensor_2):
    '''
    Function for fuzzification of defining the both behaviours right edge and obstacle avoidance
    - get the rule base
    - calculate firing strength
    - defuzzify
    '''
    if logic == 'right_edge':
        # sensor_1 = InputSpace('FRS')
        # sensor_2 = InputSpace('BRS')
        dict1 = ruleBase(sensor_1, input_1)
        dict2 = ruleBase(sensor_2, input_2)
        firing_rules = firingStrength(dict1, dict2, 'rule_base_right_edge.csv', sensor_1.name, sensor_2.name, firing_option)
        
    elif logic == 'obstacle_avoid':
        # sensor_1 = InputSpace('LFS')
        # sensor_2 = InputSpace('BFS')
        dict1 = ruleBase(sensor_1, input_1)
        dict2 = ruleBase(sensor_2, input_2)
        firing_rules = firingStrength(dict1, dict2, 'rule_base_OA.csv', sensor_1.name, sensor_2.name, firing_option)
        
    
    lin_x = defuzzification(firing_rules, '.x', 'firing_rate')
    ang_z = defuzzification(firing_rules, '.z', 'firing_rate')
    return lin_x, ang_z, (dict1, dict2), firing_rules


class CoordLayer:
    def __init__(self, start_threshold, end_threshold, centroid):
        self.end = end_threshold
        self.start = start_threshold
        self.m_range = np.arange(self.start, self.end, 0.1)
        self.member_centroid = centroid
        self.m_value = None

    def getMembershipLow(self, d_value):
        if d_value <= self.member_centroid:
            self.m_value = 1
            print('Left shoulder, value = 1')
        elif d_value > self.member_centroid and d_value < self.end:
            self.m_value = (self.end - d_value)/(self.end - self.member_centroid)
            print(f'{d_value}, falling edge for close, rising edge for medium')
        elif d_value > self.end:
            self.m_value = 0.0001
            print('Greater than, value = 0')  
        else:
            print('Value Exceeded')

    def getMembershipHigh(self, d_value):
        if d_value >= self.member_centroid:
            self.m_value = 1
            print('Right shoulder, value = 1')
        elif d_value < self.member_centroid and d_value > self.start:
            self.m_value = (self.end - d_value)/(self.end - self.member_centroid)
            print(f'{d_value}, falling edge for close, rising edge for medium')
        elif d_value < self.start:
            self.m_value = 0.0001
            print('Greater than, value = 0')  
        else:
            print('Value Exceeded')


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
