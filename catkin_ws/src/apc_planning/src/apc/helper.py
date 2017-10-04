
from openpyxl import load_workbook
import copy
import os
import json
import yaml
import hashlib
from shutil import copyfile

class ObjectNameConverter:
    def __init__(self):
        self.lower2upper = dict()
        self.upper2lower = dict()
        path_old = os.environ['RGRASPDATA_BASE'] + '/itemdata/'
        path_new = os.environ['RGRASPDATA_BASE'] + '/newitemdata/'
        for x in next(os.walk(path_old))[1]:
            if x == 'Empty':
                continue
            self.read_file(x, path_old)
        for x in next(os.walk(path_new))[1]:
            if x == 'Empty':
                continue
            self.read_file(x, path_new)

    def read_file(self, x, path):
        # print x
        with open(path + x + '/' + x.lower() + '.json', 'r') as infile:
            js_data = json.load(infile)
        corrected_data = yaml.load(yaml.safe_dump(js_data))
        self.upper2lower[x] = corrected_data['name']
        self.lower2upper[corrected_data['name']] = x

    def to_upper(self, name):
        return self.lower2upper[name]
    def to_lower(self, name):
        return self.upper2lower[name]


def toExlInd(row, col):
    return "%s%d" % (chr(ord('A')+col-1), row)

def bin_id2num(bin_id):
    return ord(bin_id[4:])-ord('A')  # bin_id is of the format 'bin_A'

def num2binId(num):
    return str('bin_'+chr(ord('A')+num))

def getValue(table, row, col):
    return table[toExlInd(row, col)].value

def loadHeuristic(filename, offset, endrow, max_strategy_num=6):
    wb = load_workbook(filename = filename)

    h = wb['Heuristic']
    hdict = {}

    item_id = ''
    for i in range(offset[0], endrow):
        if h[toExlInd(i, 2)].value:
            obj_id = getValue(h, i, 2)
            #print '\nloading object:', obj_id
            hdict[obj_id] = {}
        pose_type = getValue(h, i, 4)
        strategy_list = []
        for j in range(max_strategy_num):
            tmp = getValue(h, i, 5+j)
            if tmp:
                strategy_list.append(tmp)

        hdict[obj_id][pose_type] = strategy_list
        #print '\t', pose_type, ':', strategy_list

    return hdict

class UpdateCommand(object):
    def __init__(self, bin_pre, rm_command, funct):
        self.funct = funct
        self.bin_pre = bin_pre
        self.rm_command = rm_command
    def execute(self):
        self.funct('update hm state sg', self.rm_command, self.bin_pre)

def isHardThing(order):
    return hardbadRate[order["item"]]

def isHardThingMulti(order):
    return hardbadRateMulti[order["item"]]

def sortOrder(work_order, bin_contents_all):
    norder = len(work_order)
    # a selection sort
    for i in range(norder):
        for j in range(i+1, norder):
            order_i = work_order[i]
            order_j = work_order[j]
            ncontents_i = len(bin_contents_all[order_i["bin"]])
            ncontents_j = len(bin_contents_all[order_j["bin"]])

            if isHardThing(order_i) > isHardThing(order_j) or \
               (isHardThing(order_i) == isHardThing(order_j) and isHardThingMulti(order_i) > isHardThingMulti(order_j)) or \
               (isHardThing(order_i) == isHardThing(order_j) and isHardThingMulti(order_i) == isHardThingMulti(order_j) and ncontents_i > ncontents_j) or \
               (isHardThing(order_i) == isHardThing(order_j) and isHardThingMulti(order_i) == isHardThingMulti(order_j) and ncontents_i == ncontents_j and succRate[order_i["item"]] < succRate[order_j["item"]]):
                tmp_order = copy.deepcopy(work_order[i])
                work_order[i] = copy.deepcopy(work_order[j])
                work_order[j] = copy.deepcopy(tmp_order)
    return work_order

def estimate_difficulty_bin(order, bin_contents_all):
    return 1000.*len(bin_contents_all[order["bin"]]) + bin_id2num(order["bin"])

def sortOrder_2016(work_order, bin_contents_all):
    #TODO: include scores from visualization
    norder = len(work_order)
    # a selection sort
    for i in range(norder):
        for j in range(i+1, norder):
            order_i = work_order[i]
            order_j = work_order[j]

            if estimate_difficulty_bin(order_j,bin_contents_all) < estimate_difficulty_bin(order_i,bin_contents_all):
                tmp_order = copy.deepcopy(work_order[i])
                work_order[i] = copy.deepcopy(work_order[j])
                work_order[j] = copy.deepcopy(tmp_order)
    return work_order


