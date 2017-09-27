#!/usr/bin/python
import numpy as np
import json

# sensor specs
sensor_acc = 0.03 # sensor accuracy - precent of the full scale range
sensor_fsr = 10   # full scale range - kg
sensor_res = sensor_fsr*sensor_acc*10
lambda_scale = 0.15 # tuned by hand depending on the accuracy of the sensor in the system

def prob_gen(measured,obj_weight):
    return np.exp(-np.abs(np.floor((measured-obj_weight))/sensor_res)*lambda_scale)

# test item list
item_list = ['Expo_Eraser','Bath_Sponge','Duct_Tape']
measured_weights_list = [23.0,43.0]

with open('object_deetz.json') as data_file:
    content = json.load(data_file)

ind_list = []
obj_weights = []
for x in range(0,len(item_list)):
    for y in range(0,len(content['obj'])):
        if content['obj'][y]['id'] == item_list[x]:
            ind_list.append(y)
            obj_weights.append(float(content['obj'][y]['weight']))
            break

print '[WS] Measured weight in tote 1 is %f' % measured_weights_list[0]
print ''
print '[WS] Weights of the objects in the tote:'
print obj_weights
print ''
print '[WS] Probability assignment to each object:'
print [prob_gen(measured_weights_list[0],float(x)) for x in obj_weights]



