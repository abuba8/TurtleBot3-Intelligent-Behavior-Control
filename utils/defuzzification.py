def defuzzification(obj, col, firing_rate):
    weights = [rule[col] * rule[firing_rate] for rule in obj.values()]
    firing_rates = [rule[firing_rate] for rule in obj.values()]
    return sum(weights) / sum(firing_rates)