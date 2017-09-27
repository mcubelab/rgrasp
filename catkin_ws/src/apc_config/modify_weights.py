#!/usr/bin/python

import os
import json

if __name__ == '__main__':
    data = dict()
    d = dict()
    path_old = os.environ['ARCDATA_BASE'] + '/itemdata/'

    i = 1
    list_of_dirs = next(os.walk(path_old))[1]
    list_of_dirs.sort()
    for x in list_of_dirs:
        if x == 'Empty':
            continue
        print '\nObject num. ' + str(i)
        i += 1
        with open(path_old + x + '/' + x.lower() + '.json', 'r') as infile:
            js_data = json.load(infile)

        print x + '. Old weight is: ' + str(js_data['weight']*1000.0) + ' g'
        weight_str = raw_input("Please enter new weight in g. Keep it empty to skip: ")
        if weight_str:
            weight = float(weight_str)
            js_data['weight'] = weight / 1000.0

        slow_move_str = raw_input("Should this object be moved carefully? y/n: ")
        slow_move = False
        if slow_move_str == 'y':
            slow_move = True
        js_data['slow_move'] = slow_move

        with open(path_old + x + '/' + x.lower() + '.json', 'w') as outfile:
            json.dump(js_data, outfile, indent=4)