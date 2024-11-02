from utils.fuzzification import ruleBase, firingStrength
from utils.defuzzification import defuzzification

def Fuzzifier(logic, input_1, input_2, firing_option, sensor_1, sensor_2):
    if logic == 'right_edge':
        dict1 = ruleBase(sensor_1, input_1)
        dict2 = ruleBase(sensor_2, input_2)
        firing_rules = firingStrength(dict1, dict2, 'rule_base_right_edge.csv', sensor_1.name, sensor_2.name, firing_option)
    elif logic == 'obstacle_avoid':
        dict1 = ruleBase(sensor_1, input_1)
        dict2 = ruleBase(sensor_2, input_2)
        firing_rules = firingStrength(dict1, dict2, 'rule_base_OA.csv', sensor_1.name, sensor_2.name, firing_option)
    
    lin_x = defuzzification(firing_rules, '.x', 'firing_rate')
    ang_z = defuzzification(firing_rules, '.z', 'firing_rate')
    return lin_x, ang_z, (dict1, dict2), firing_rules
