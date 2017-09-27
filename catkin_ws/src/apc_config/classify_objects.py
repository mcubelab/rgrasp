#!/usr/bin/python

import yaml
import os
import json
from subprocess import call

if __name__ == '__main__':
    data = dict()
    names = list()
    d = dict()
    path_old = os.environ['ARCDATA_BASE'] + '/itemdata/'
    if not os.path.exists(path_old + 'Empty'):
        os.symlink(os.environ['ARC_BASE'] + '/catkin_ws/src/passive_vision/empty_object/Empty', path_old + 'Empty')
    elif not os.path.islink(path_old + 'Empty'):
        os.rmdir(path_old + 'Empty')
        os.symlink(os.environ['ARC_BASE'] + '/catkin_ws/src/passive_vision/empty_object/Empty', path_old + 'Empty')
    path_output = os.environ['ARC_BASE'] + '/catkin_ws/src/apc_config/object_properties.yaml'
    path_object_list = os.environ['ARCDATA_BASE'] + '/input/list_provided.yaml'
    for x in next(os.walk(path_old))[1]:
        if x == 'Empty':
            continue
        print x
        with open(path_old + x + '/' + x.lower() + '.json', 'r') as infile:
            js_data = json.load(infile)
        js_data['dimensions'].sort(reverse = True)
        names.append(x.lower())
        js_data.pop('description', None)
        d[x.lower()] = js_data

    data['/obj']=d
    with open(path_output, 'w') as outfile:
        yaml.safe_dump(data, outfile, width = 10000)
    with open(path_object_list, 'w') as outfile:
        json.dump(names, outfile)

    call(['rosparam', 'load', path_output])