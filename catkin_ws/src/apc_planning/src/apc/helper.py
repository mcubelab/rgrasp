
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
        path_old = os.environ['ARCDATA_BASE'] + '/itemdata/'
        path_new = os.environ['ARCDATA_BASE'] + '/newitemdata/'
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

def loadHeuristic_2016(filename, offset, endrow):
    filename_json = filename + '.json'
    need_to_recache = True

    # compute the sha of target xsl file
    with open(filename, 'r') as xslfile:
        readFile = xslfile.read()
        xsl_sha1 = str(hashlib.sha1(readFile).hexdigest())

    # compare the sha with the sha1 in cache
    try:
        with open(filename_json, 'r') as data_file:
            data_with_sha = json.load(data_file)
            ClutterFree = data_with_sha['data']
            cached_sha1 = data_with_sha['sha1']
        if cached_sha1 == xsl_sha1:
            need_to_recache = False
        else:
            print 'Cache of heurisitc is updated, need to recache'
    except:
        # cache json file does not exist
        print 'Cache of heurisitc does not exist, need to recache'


    if need_to_recache:
        ClutterFree = _loadHeuristic_2016(filename, offset, endrow)
        with open(filename_json, 'w') as outfile:
            json.dump({'sha1': xsl_sha1, 'data': ClutterFree}, outfile)

    return ClutterFree

def _loadHeuristic_2016(filename, offset, endrow):
    #Reads the table with the probability estimates.
    wb = load_workbook(filename = filename)
    primitives = {'grasping', 'grasp-tote', 'suction-tote', 'suction-down', 'suction-front', 'suction-side', 'push-front', 'push-side','scooping'} #'scoop-pinch' not filled
    hdict = {} #Map[primitive][object][pose_type]
    sub_ids = {}
    for primitive in primitives:
        #print primitive
        h = wb[primitive]
        hdict[primitive] = {}
        sub_ids[primitive] = {}
        current_object = 'none'
        #preprocess.
        mapping = {'Puffy-C' : 'other-center', 'Puffy-W':'other-wall', 'Sphere-C': 'other-center', 'Sphere-W': 'other-wall', 'Flat-C':'flat-center','Int-C':'intermediate-center','Tall-C':'upright-center', 'Flat-W':'flat-wall', 'Int':'intermediate-wall','Int-W':'intermediate-wall','Tall':'upright-wall','Tall-W':'upright-wall'}
        for i in range(offset[0], endrow):
            h.cell(row=i, column = 3).value = mapping[str(getValue(h,i,3))]
        for i in range(offset[0], endrow):
            #print primitive, i, getValue(h,i,1), getValue(h,i,2), getValue(h,i,3), getValue(h,i,4), getValue(h,i,5)
            if getValue(h,i,1)!=None and current_object!=str(getValue(h,i,1)):
                current_object = str(getValue(h,i,1))
                hdict[primitive][current_object] = {}
                sub_ids[primitive][current_object] = {}
            if getValue(h,i,5)!=None:
                #print primitive, current_object, str(getValue(h,i,3)), '->', float(getValue(h,i,5))
                hdict[primitive][current_object][str(getValue(h,i,3))] = float(getValue(h,i,5))
            else:
                if getValue(h,i,4)!=None:
                    if str(getValue(h,i,4)) not in sub_ids:
                        print '[Load Heuristic] ',str(getValue(h,i,4)),' mentioned before sub_id appeared'
                    else:
                        equiv = sub_ids[getValue(h,i,4)]
                        hdict[primitive][current_object][str(getValue(h,i,3))] = hdict[primitive][equiv[0]][equiv[1]]
                        #print primitive, current_object, str(getValue(h,i,3)) ,'->', hdict[primitive][equiv[0]][equiv[1]]
                else:
                    print getValue(h,i,1),getValue(h,i,2),getValue(h,i,3),getValue(h,i,4),getValue(h,i,5),getValue(h,i,6)
                    print '[Load Heuristic] no way to assign probability to ',str(getValue(h,i,2))
            sub_ids[str(getValue(h,i,2))] = [current_object, str(getValue(h,i,3))]
    return hdict

def create_categories_2016(filename, offset, endrow):
    primitive = 'grasping'
    wb = load_workbook(filename = filename)
    Cat = {}
    h = wb[primitive]
    current_object = 'none'
    #preprocess.
    mapping = {'Puffy-C' : 'other-center', 'Puffy-W':'other-wall', 'Sphere-C': 'other-center', 'Sphere-W': 'other-wall', 'Flat-C':'flat-center','Int-C':'intermediate-center','Tall-C':'upright-center', 'Flat-W':'flat-wall', 'Int':'intermediate-wall','Int-W':'intermediate-wall','Tall':'upright-wall','Tall-W':'upright-wall'}
    for i in range(offset[0], endrow):
        h.cell(row=i, column = 3).value = mapping[str(getValue(h,i,3))]
    for i in range(offset[0], endrow):
        if getValue(h,i,1)!=None and current_object!=str(getValue(h,i,1)):
            current_object = str(getValue(h,i,1))
            Cat[current_object] = []
        Cat[current_object].append(str(getValue(h,i,3)))
    print Cat
    data = Cat
    outfilename = os.environ['APC_BASE']+'/doc/strategy/categories_2016.json'
    with open(outfilename, 'w') as outfile:
        json.dump(data, outfile, sort_keys = True, indent = 4, ensure_ascii=False)

succRate = {
"oreo_mega_stuf": 1,
"crayola_64_ct": 1,
"paper_mate_12_count_mirado_black_warrior": 0.5,
"mead_index_cards": 0.5,
"rolodex_jumbo_pencil_cup": 0,
"mark_twain_huckleberry_finn": 1,
"laugh_out_loud_joke_book": 1,
"sharpie_accent_tank_style_highlighters": 0.5,
"stanley_66_052": 0.5,
"expo_dry_erase_board_eraser": 1,
"champion_copper_plus_spark_plug": 0.5,
"feline_greenies_dental_treats": 1,
"kong_air_dog_squeakair_tennis_ball": 0.3,
"dr_browns_bottle_brush": 0.5,
"kong_duck_dog_toy": 0.5,
"kong_sitting_frog_dog_toy": 0.5,
"munchkin_white_hot_duck_bath_toy": 0.4,
"mommys_helper_outlet_plugs": 0.3,
"kyjen_squeakin_eggs_plush_puppies": 0.5,
"first_years_take_and_toss_straw_cup": 0.4,
"highland_6539_self_stick_notes": 0.5,
"safety_works_safety_glasses": 0.4,
"genuine_joe_plastic_stir_sticks": 1,
"cheezit_big_original": 1,
"elmers_washable_no_run_school_glue": 1
}

hardbadRate = {
"oreo_mega_stuf": 0,
"crayola_64_ct": 0,
"paper_mate_12_count_mirado_black_warrior": 0,
"mead_index_cards": 0,
"rolodex_jumbo_pencil_cup": 1,
"mark_twain_huckleberry_finn": 0,
"laugh_out_loud_joke_book": 0,
"sharpie_accent_tank_style_highlighters": 0,
"stanley_66_052": 0,
"expo_dry_erase_board_eraser": 0,
"champion_copper_plus_spark_plug": 0,
"feline_greenies_dental_treats": 0,
"kong_air_dog_squeakair_tennis_ball": 0,
"dr_browns_bottle_brush": 0,
"kong_duck_dog_toy": 0,
"kong_sitting_frog_dog_toy": 0,
"munchkin_white_hot_duck_bath_toy": 0,
"mommys_helper_outlet_plugs": 0,
"kyjen_squeakin_eggs_plush_puppies": 0,
"first_years_take_and_toss_straw_cup": 0.9,
"highland_6539_self_stick_notes": 0,
"safety_works_safety_glasses": 0,
"genuine_joe_plastic_stir_sticks": 0,
"cheezit_big_original": 0,
"elmers_washable_no_run_school_glue": 0
}

hardbadRateMulti = {
"oreo_mega_stuf": 0,
"crayola_64_ct": 0,
"paper_mate_12_count_mirado_black_warrior": 0,
"mead_index_cards": 0,
"rolodex_jumbo_pencil_cup": 0,
"mark_twain_huckleberry_finn": 0,
"laugh_out_loud_joke_book": 0,
"sharpie_accent_tank_style_highlighters": 0.9,
"stanley_66_052": 0,
"expo_dry_erase_board_eraser": 0,
"champion_copper_plus_spark_plug": 0,
"feline_greenies_dental_treats": 0,
"kong_air_dog_squeakair_tennis_ball": 0,
"dr_browns_bottle_brush": 0,
"kong_duck_dog_toy": 0,
"kong_sitting_frog_dog_toy": 0,
"munchkin_white_hot_duck_bath_toy": 0,
"mommys_helper_outlet_plugs": 1,
"kyjen_squeakin_eggs_plush_puppies": 0,
"first_years_take_and_toss_straw_cup": 0,
"highland_6539_self_stick_notes": 0,
"safety_works_safety_glasses": 0,
"genuine_joe_plastic_stir_sticks": 0,
"cheezit_big_original": 0,
"elmers_washable_no_run_school_glue": 0
}

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

def EvaluateProbability():
    #TODO just do it.
    return 1

def sortLearnedOrder(work_order, bin_contents_all, mode='testing'):
    norder = len(work_order)
    if mode!='testing' and mode!='training':
        print '[Helper] Sorting mode not recognized. No sorting done.'
        return work_order
    # We will do selection sort in both cases.
    for i in range(norder):
        for j in range(i+1, norder):
            order_i = work_order[i]
            order_j = work_order[j]
            prob_i = EvaluateProbability()
            prob_j = EvaluateProbability()
            if mode=='training':
                #TODO: do something. Random permutation within those that are better than threshold?
                prob_i = prob_i
                prob_j = prob_j
            if prob_i<prob_j:
                tmp_order = copy.deepcopy(work_order[i])
                work_order[i] = copy.deepcopy(work_order[j])
                work_order[j] = copy.deepcopy(tmp_order)
    return work_order

def displayOrder(work_order, bin_contents_all, highlight = None):

    f = open(os.environ['HOME']+'/Desktop/'+'sorted_work_order.html', 'w')
    f.write('<HTML><HEAD><TITLE></TITLE><META HTTP-EQUIV="refresh" CONTENT="2"></HEAD><BODY>')

    norder = len(work_order)
    #print "Sorted orders: "
    for i in range(norder):
        #output = ("%d. " % (i+1)) + work_order[i]["bin"] + " " + work_order[i]["item"] + ('\n\t\t\t\t\t\tbinContentNum: %d' % len(bin_contents_all[work_order[i]["bin"]])) + (' succRate: %.1f' % succRate[work_order[i]["item"]]) + '\n'
        output = ("%d. " % (i+1)) + work_order[i]["bin"] + " " + work_order[i]["item"] + ('\n\t\t\tbinContentNum: %d' % len(bin_contents_all[work_order[i]["bin"]])) + '\n'
        #print(output)
        if highlight is not None and i == highlight:
            output = '<b>' + output + '</b>'
        f.write(output.replace('\n', '<br />').replace('\t', '&nbsp;'*20))

    f.write('</BODY></HTML>')

def td(s):
    return '<td>' + s + '</td>'
space = '&nbsp;'
def displayOrder_2016(desirability,probs,attempt_order,work_order, bin_contents_all,Seq,Seq_prim, Seq_warn, finished, threshold_for_execution):

    f = open(os.environ['HOME']+'/Desktop/'+'sorted_work_order.html', 'w')
    f.write('<HTML><HEAD><TITLE></TITLE><META HTTP-EQUIV="refresh" CONTENT="2"></HEAD><BODY>')
    f.write('<table>')
    f.write('<tr>' +
            td('Order') +
            td('Id') +
            td('Target item') +
            td('#it') +
            td('Prb') +
            td('Des') +
            td('Plan') +
            #td('Bin content') +
            '</tr>')
    norder = len(work_order)
    #print "Sorted orders: "
    for iorder, i in enumerate(attempt_order):
        binNum = bin_id2num(work_order[i]["bin"])
        output = '<tr>'
        output += td( "%d" % iorder )
        output += td( work_order[i]["bin"][4] )
        output += td( work_order[i]["item"][:15] )
        output += td( '%d' % len(bin_contents_all[work_order[i]["bin"]] ) )
        output += td( '%.2f' % probs[binNum])
        output += td( '%.2f' % desirability[binNum] )

        if desirability[i] > threshold_for_execution:
            aux_output = ''
            for j in xrange(3):
                if Seq[binNum][j] >= 0:
                    aux_output += Seq_prim[binNum][j]
                    if Seq_warn[binNum][j]:
                        aux_output += '!!!'
                    else:
                        aux_output += '?'
                    aux_output += space + bin_contents_all[work_order[i]["bin"]][Seq[binNum][j]]
                    if j < 2:
                        aux_output +='|'
                    #print 'Seq[binNum][j]' , Seq[binNum][j]
                    #print 'work_order[i]["bin"]', work_order[i]["bin"]
                    #print 'bin_contents_all[work_order[i]["bin"]]', bin_contents_all[work_order[i]['bin'] ]
            output += td( aux_output )
        else:
            if finished[i]:
                output += td( 'SUCCESS' )
            else:
                output += td( 'FAIL' )

        #output += td( str(bin_contents_all[work_order[i]["bin"]]) )
        output += '</tr>'
        f.write(output)
    f.write('</table>')

    f.write('</BODY></HTML>')
    f.close()

def displayOrder_AlphaCube_2016(explanations, table, numBin = 0):
    f = open(os.environ['HOME']+'/Desktop/'+'planner_explanation.html', 'w')
    f.write('<HTML><HEAD><TITLE></TITLE><META HTTP-EQUIV="refresh" CONTENT="2"></HEAD><BODY>')
    red = [190,0,0]
    green = [0,190,0]
    f.write('<table>')
    for i in range(len(table)):
        output = '<tr>'
        for j in range(len(table[i])):
            if j>0 and i >0:
                x = table[i][j].replace('=','*').split('*')
                for xx in x:
                    color = [int(float(xx)*green[k]+(1-float(xx))*red[k]) for k in range(3)]
                    font = '<font color="%s">' % ('#%02x%02x%02x' % tuple(color))  # IT SEEMS RGB DOES NOT WORK WELL
                    output += td( font + xx + '</font>')
            else:
                output += td(table[i][j])
        output += '</tr>'
        f.write(output)
    f.write('</table>')
    f.write('\n\n\n')
    red = [190,0,0]
    green = [0,190,0]

    f.write('<table>')
    for i in range(len(explanations)):
        output = '<tr>'
        for j in range(len(explanations[i])):
            # if (j==1 or j==2) and i >0:
            #     x = table[i][j].replace('=','*').split('*')
            #     for xx in x:
            #         color = [int(float(xx)*green[k]+(1-float(xx))*red[k]) for k in range(3)]
            #         font = '<font color="%s">' % ('#%02x%02x%02x' % tuple(color))  # IT SEEMS RGB DOES NOT WORK WELL
            #         output += td( font + xx + '</font>')
            # else:
            if type(explanations[i][j])==type([]):
                for k in range(len(explanations[i][j])):
                    # print 'WTF: ', explanations[i][j][k]
                    if explanations[i][j][k][1][0]=='-':
                        output += td(('%.2f' % -explanations[i][j][k][0]) + ' ' + str(explanations[i][j][k][1][1:]))
                    else:
                        output += td(('%.2f' % explanations[i][j][k][0]) + ' ' + str(explanations[i][j][k][1]))
            else:
                output+=td(explanations[i][j])
        output += '</tr>'
        f.write(output)
    f.write('</table>')

    f.write('</BODY></HTML>')
    f.close()
    copyfile(os.environ['HOME']+'/Desktop/'+'planner_explanation.html',os.environ['HOME']+'/Desktop/'+'planner_explanation-'+('%d' % numBin)+'.html')

def displayOrder_placing_2016(table, act_time, best_primitive, best_object, best_score):
    red = [190,0,0]
    green = [0,190,0]
    f = open(os.environ['HOME']+'/Desktop/'+'table_placing.html', 'w')
    f.write('<HTML><HEAD><TITLE></TITLE><META HTTP-EQUIV="refresh" CONTENT="2"></HEAD><BODY>')
    act_time = int(act_time)
    f.write('Time remaining: '+ str(act_time/60) + 'minutes and ' + str(act_time%60) + 'seconds.\n\n')
    f.write('NEXT MOVE: ' + best_primitive + ' on ' + best_object['label'] + ' w/ probability ' + ('%.2f' % best_score)+ '.\n\n')
    f.write('Table displays ClutterFree * VisionScore * ClutterScore * BadPairs = FinalResult\n')
    f.write('<table>')
    for i in range(len(table)):
        output = '<tr>'
        for j in range(len(table[i])):
            if (j==1 or j==2) and i >0:
                x = table[i][j].replace('=','*').split('*')
                for xx in x:
                    color = [int(float(xx)*green[k]+(1-float(xx))*red[k]) for k in range(3)]
                    font = '<font color="%s">' % ('#%02x%02x%02x' % tuple(color))  # IT SEEMS RGB DOES NOT WORK WELL
                    output += td( font + xx + '</font>')
            else:
                output
                output += td(table[i][j])
        output += '</tr>'
        f.write(output)
    f.write('</table>')

    f.write('</BODY></HTML>')
    f.close()
