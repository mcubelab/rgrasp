#!/usr/bin/env python

#~ import rospy
import numpy as np
import json
from distutils.dir_util import copy_tree
from numpy import linalg as la
#~ from pr_msgs.msg import *
#~ from manual_fit.srv import *
from copy import deepcopy
import os
import time
import random
import sys
#~ from ik.helper import deleteMarkers, plotPickPoints
    
def check_format_json(jsonfilepath):
    is_correct = False
    try:
        with open(jsonfilepath) as json_data:
            d = json.load(json_data)
        print '[JSON format check] Correct format'
        is_correct = True
    except ValueError:
        print '[JSON format check] Incorrect format'
    
    return is_correct
        
def get_json(jsonfilepath):
    try:
        with open(jsonfilepath) as json_data:
            d = json.load(json_data)
        #~ print '[JSON format check] Correct format'
    except ValueError:
        print '[JSON get file] Incorrect format'
    return d
    
def get_random_object(filepath, num_objects = 1):
    json_obj = get_json(filepath)
    
    obj_list = []
    index_list = []
    for i in range(0,num_objects):
        index = random.randint(0, len(json_obj["objects"])-1) 
        obj_list.append(json_obj["objects"][index])
        index_list.append(index)
        
    return obj_list, index_list
    
def get_random_object_single(object_list_total, num_objects = 1):
   
    random.shuffle(object_list_total) #random.(0, len(json_obj["objects"])-1)
    obj_list_reduced = object_list_total[0:num_objects]
    
    return obj_list_reduced

def get_tote_object(jsonfilepath):
    try:
        with open(jsonfilepath) as json_data:
            d = json.load(json_data)
            json_data.close()
        print '[JSON format check] Correct format'
    except ValueError:
        print '[JSON format check] Incorrect format'
        
def write_json(filepath, data):
    with open(filepath, 'w') as f:
         json.dump(data, f, indent=4)
         
def set_num_objects_per_bin():
    #~assign order list
    box_objects_list = [5,3,2,0,0]
    box_id_list = [4,5,6,7,8]
    box_id_list_ini = deepcopy(box_id_list)
    random.shuffle(box_id_list)
    
    #~
    box_order_list = []
    num_order_list = []
    
    for i in range(5):
        box_order_list.append(box_id_list[i])
        num_order_list.append(box_objects_list[i])
        
    index = []
    num_orders_bin = []

    for i in box_id_list:
        index = box_id_list_ini.index(i)
        num_orders_bin.append(num_order_list[index])
    
    num_orders_bin4 = num_orders_bin[0]
    num_orders_bin5 = num_orders_bin[1]
    num_orders_bin6 = num_orders_bin[2]
    num_orders_bin7 = num_orders_bin[3]
    num_orders_bin8 = num_orders_bin[4]
    
    return num_orders_bin4, num_orders_bin5, num_orders_bin6, num_orders_bin7, num_orders_bin8
         
def write_json_file_task(num_objects_total=0, 
                    num_objects_bin0=0, 
                    num_objects_bin1=0, 
                    num_objects_bin2=0, 
                    num_objects_bin3=0,
                    num_orders_total=0, 
                    num_orders_bin4=0, 
                    num_orders_bin5=0, 
                    num_orders_bin6=0, 
                    num_orders_bin7=0, 
                    num_orders_bin8=0,
                    json_id = 0,
                    num_json_files = 1,
                    task_name = 'stow_pick_item_location'
                    ):

    filepath_all_objects = os.environ['ARC_BASE'] + '/input/all_objects.json'
    
    for j in range(0,num_json_files):
        filepath_location = os.environ['ARC_BASE'] + '/input/' + task_name +'_item_location_'+str(num_objects_total)+'_objects_'+str(json_id) + str(j)+'.json'
        filepath_order = os.environ['ARC_BASE'] + '/input/' + task_name +'_order_'+ str(num_objects_total)+'_objects_'+str(json_id) + str(j)+'.json'
        
        #~get random objects
        data = get_json(filepath_all_objects)
        object_list, index_list = get_random_object(filepath_all_objects, num_objects_total)
        
        ###################
        ## Location list ##
        ###################
        data_location = {}
        #~tote
        tote_objects = object_list[0:num_objects_bin0]
        tote_dic = {'contents': []}
        tote_dic['contents'].extend(tote_objects)
        
        #~ data_location['tote'] = []  
        data_location['tote']=tote_dic
        #~bins
        bin_list = ['A', 'B', 'C']
        bin_objects=[]
        bin_dic = []
        data_location['bins'] = []  
        bin_objects.append(object_list[num_objects_bin0:num_objects_bin0+num_objects_bin1])
        bin_objects.append(object_list[num_objects_bin0+num_objects_bin1:num_objects_bin0+num_objects_bin1+num_objects_bin2])
        bin_objects.append(object_list[num_objects_bin0+num_objects_bin1+num_objects_bin2:num_objects_total])
        bin_dic = []
        
        for i in range(0,3):
            bin_dic.append({'bin_id': "A" , 'contents': []})
            bin_dic[i]['bin_id']=bin_list[i]
            bin_dic[i]['contents'].extend(bin_objects[i])
            data_location['bins'].append(bin_dic[i])
            
        #~add missing bins 
        missing_list = ["A","B","C","D","E","F","G","H","I","J"]
        bin_dic = []
        for i in range(0,6):
            bin_dic.append({'bin_id': missing_list[i] , 'contents': []})
            data_location['bins'].append(bin_dic[i])
        
        #~Write json file 
        write_json(filepath_location, data_location)
        
        ################
        ## Order List ##
        ################
        data_order = {}
        order_index_list = random.sample(range(0,num_objects_total),  num_orders_total) 
        order_box_list = [0,1,2,3,4]
        map_bin_name_to_id = {"1A5":0,"1AD":1,"A1":2 ,"1B2":3,"K3":4}
        map_bin_order_to_name = {0:"A1",1:"1AD",2:"1A5" ,3:"1B2",4:"K3"}
        
        box_objects = []
        #~ print 'order_index_list', order_index_list
        box_objects.append(order_index_list[0:num_orders_bin4])
        box_objects.append(order_index_list[num_orders_bin4:num_orders_bin4+num_orders_bin5])
        box_objects.append(order_index_list[num_orders_bin4+num_orders_bin5:num_orders_bin4+num_orders_bin5+num_orders_bin6])
        box_objects.append(order_index_list[num_orders_bin4+num_orders_bin5+num_orders_bin6:num_orders_bin4+num_orders_bin5+num_orders_bin6+num_orders_bin7])
        box_objects.append(order_index_list[num_orders_bin4+num_orders_bin5+num_orders_bin6+num_orders_bin7:num_orders_bin4+num_orders_bin5+num_orders_bin6+num_orders_bin7+num_orders_bin8])
        
        data_order['orders'] = []  
        bin_list = ['A1', '1AD', '1A5', '1B2', 'K3']
        
        if (task_name == 'stow_pick') or (task_name == 'pick'):
            #~build empty dictionaries 
            bin_dic = []
            for i in range(0,5):
                bin_dic.append({'size_id': bin_list[i] , 'contents': []})
                data_order['orders'].append(bin_dic[i])
                
            #~edit dictionaries with objects
            bin_dic = []
            counter = 0
            for box in range(0,5):
                bin_json_name = map_bin_order_to_name[counter]
                bin_json_id = map_bin_name_to_id[bin_json_name]
                #~ print 'bin_json_id', bin_json_id
                for term in box_objects[bin_json_id]:
                    data_order['orders'][box]['contents'].append(object_list[term])
                counter+=1
                
            #~Write json file 
            write_json(filepath_order, data_order)
            
        print '[JSON File Generation] New JSON file added to:', os.environ['ARC_BASE'] + '/input/'
        
def write_json_file_task_competition(num_objects_total=0, 
                    num_objects_bin0=0, 
                    num_objects_bin1=0, 
                    num_objects_bin2=0, 
                    num_objects_bin3=0,
                    num_orders_total=0, 
                    json_id = 0,
                    num_json_files = 1,
                    task_name = 'stow_pick_item_location'
                    ):

    
    #~get list of objects from json file TODO
    filepath_all_objects = os.environ['ARCDATA_BASE'] + '/back_original_itemdata' #os.environ['ARC_BASE'] + '/input/all_objects.json'
     
    for j in range(0, num_json_files):
        filepath_location = os.environ['ARCDATA_BASE'] + '/input/' + task_name +'_item_location_'+str(num_objects_total)+'_objects_'+str(json_id) + str(j)+'.json'
        filepath_order = os.environ['ARCDATA_BASE'] + '/input/' + task_name +'_order_'+ str(num_objects_total)+'_objects_'+str(json_id) + str(j)+'.json'
        
        #~get random objects
        object_names_list_upper = [name for name in os.listdir(filepath_all_objects) if os.path.isdir(os.path.join(filepath_all_objects, name))]
        #~ object_names_list_lower = [i.lower() for i in object_names_list_upper]

        object_list_upper = get_random_object_single(object_names_list_upper, num_objects_total)
        object_list = [i.lower() for i in object_list_upper]
        
        num_orders_bin4, num_orders_bin5, num_orders_bin6, num_orders_bin7, num_orders_bin8 = set_num_objects_per_bin()
        
        ###################
        ## Location list ##
        ###################
        data_location = {}
        #~tote
        tote_objects = object_list[0:num_objects_bin0]
        tote_dic = {'contents': []}
        tote_dic['contents'].extend(tote_objects)
        
        #~ data_location['tote'] = []  
        data_location['tote']=tote_dic
        #~bins
        bin_list = ['A', 'B', 'C']
        bin_objects=[]
        bin_dic = []
        data_location['bins'] = []  
        bin_objects.append(object_list[num_objects_bin0:num_objects_bin0+num_objects_bin1])
        bin_objects.append(object_list[num_objects_bin0+num_objects_bin1:num_objects_bin0+num_objects_bin1+num_objects_bin2])
        bin_objects.append(object_list[num_objects_bin0+num_objects_bin1+num_objects_bin2:num_objects_total])
        bin_dic = []
        
        for i in range(0,3):
            bin_dic.append({'bin_id': "A" , 'contents': []})
            bin_dic[i]['bin_id']=bin_list[i]
            bin_dic[i]['contents'].extend(bin_objects[i])
            data_location['bins'].append(bin_dic[i])
            
        #~add missing bins 
        missing_list = ["A","B","C","D","E","F","G","H","I","J"]
        bin_dic = []
        for i in range(0,6):
            bin_dic.append({'bin_id': missing_list[i] , 'contents': []})
            data_location['bins'].append(bin_dic[i])
        
        #~Write json file 
        write_json(filepath_location, data_location)
        ################
        ## Order List ##
        ################
        data_order = {}
        order_index_list = random.sample(range(0,num_objects_total),  num_orders_total) 
        order_box_list = [0,1,2,3,4]
        map_bin_name_to_id = {"1A5":0,"1AD":1,"A1":2 ,"1B2":3,"K3":4}
        map_bin_order_to_name = {0:"A1",1:"1AD",2:"1A5" ,3:"1B2",4:"K3"}
        
        box_objects = []
        #~ print 'order_index_list', order_index_list
        box_objects.append(order_index_list[0:num_orders_bin4])
        box_objects.append(order_index_list[num_orders_bin4:num_orders_bin4+num_orders_bin5])
        box_objects.append(order_index_list[num_orders_bin4+num_orders_bin5:num_orders_bin4+num_orders_bin5+num_orders_bin6])
        box_objects.append(order_index_list[num_orders_bin4+num_orders_bin5+num_orders_bin6:num_orders_bin4+num_orders_bin5+num_orders_bin6+num_orders_bin7])
        box_objects.append(order_index_list[num_orders_bin4+num_orders_bin5+num_orders_bin6+num_orders_bin7:num_orders_bin4+num_orders_bin5+num_orders_bin6+num_orders_bin7+num_orders_bin8])
        
        data_order['orders'] = []  
        bin_list = ['A1', '1AD', '1A5', '1B2', 'K3']
        
        if (task_name == 'stow_pick') or (task_name == 'pick'):
            #~build empty dictionaries 
            bin_dic = []
            for i in range(0,5):
                bin_dic.append({'size_id': bin_list[i] , 'contents': []})
                data_order['orders'].append(bin_dic[i])
                
            #~edit dictionaries with objects
            bin_dic = []
            counter = 0
            for box in range(0,5):
                bin_json_name = map_bin_order_to_name[counter]
                bin_json_id = map_bin_name_to_id[bin_json_name]
                #~ print 'bin_json_id', bin_json_id
                for term in box_objects[bin_json_id]:
                    data_order['orders'][box]['contents'].append(object_list[term])
                counter+=1
                
            #~Write json file 
            write_json(filepath_order, data_order)
            
        print '[JSON File Generation] New JSON file added to:', os.environ['ARCDATA_BASE'] + '/input/'
        
        toDirectory = os.environ['ARCDATA_BASE'] + '/input/' + task_name + '_itemdata' +str(json_id) + str(j)
        try:
            os.stat(toDirectory)
        except:
            os.mkdir(toDirectory)
        for i in object_list_upper:
            fromDirectory = filepath_all_objects + '/' + i
            try:
                os.stat(toDirectory + '/' + i)
            except:
                os.mkdir(toDirectory + '/' + i)
            copy_tree(fromDirectory, toDirectory + '/' + i)

# To test the function
if __name__=='__main__':
    pass
    
    
    #################################
    ## Create Pick Task json files ##
    #################################
    
    #~ write_json_file_task_competition(num_objects_total=32, 
                        #~ num_objects_bin1=16, 
                        #~ num_objects_bin2=12, 
                        #~ num_objects_bin3=8,
                        #~ num_orders_total=10, 
                        #~ num_orders_bin4=2, 
                        #~ num_orders_bin5=3, 
                        #~ num_orders_bin6=0, 
                        #~ num_orders_bin7=0, 
                        #~ num_orders_bin8=5,
                        #~ num_json_files =5, #~how many files do you want generated?
                        #~ task_name = 'pick'
                        #~ )
                        
    #################################
    ## Create Stow Task json files ##
    #################################
    
    #~ write_json_file_task_competition(num_objects_total=20, 
                        #~ num_objects_bin0=20, 
                        #~ num_objects_bin1=0, 
                        #~ num_objects_bin2=0, 
                        #~ num_objects_bin3=0,
                        #~ num_json_files = 5, #~how many files do you want generated?
                        #~ task_name = 'stow'
                        #~ )
                        
    ######################################
    ## Create Stow/Pick Task json files ##
    ######################################
    
    write_json_file_task_competition(num_objects_total=32, 
                        num_objects_bin0=16, 
                        num_objects_bin1=3, 
                        num_objects_bin2=5, 
                        num_objects_bin3=8,
                        num_orders_total=10,
                        num_json_files = 5, #~how many files do you want generated?
                        task_name = 'stow_pick'
                        )



    
