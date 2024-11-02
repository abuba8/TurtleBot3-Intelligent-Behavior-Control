from itertools import product
import numpy as np
from utils.rules_loader import loadRules
from controllers.output_space import OutputSpace

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
