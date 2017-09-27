#!/usr/bin/env python
'''
Python code to generate webpage to display user relevant content
during APC. The planner sends updates to values. The webpage updates
once every second (can be adjusted)
'''

import os, difflib, time, datetime
from item import Item

class WebDisplay(object):
    '''Status webpage code generation'''
    def __init__(self):
        '''Set up the file locations, initialize dictionary items that
        represent the data to be displayed and generate initial page'''
        base_path = os.environ['HOME']+'/arcdata/'
        self.web_location = base_path+'apc_home.html'
        self.image_location = 'itemdata/' 
        self.state_location = 'tmpdata/'
        folders = os.listdir(base_path)
        self.old_items = self.oldImageList()
        if self.image_location[0:-1] in folders:
            self.image_folders = os.listdir(base_path+self.image_location)
        else:
            self.image_folders = None
        if self.state_location[0:-1] not in folders:
            self.state_location = None

        self.printName = self.printNameDict()
        self.params = dict()
        self.params['TaskName'] = 'Stowing'
        self.params['ActiveVisionObject'] = 'N/A'
        self.params['ActiveVisionObjectSmall'] = 'N/A'
        self.params['RobotAction'] = 'N/A'
        self.params['ObjectsLeft'] = 'N/A'
        self.params['img'] = ''
        self.params['imgSmall'] = ''
        self.params['WeightDiff'] = [None]*9
        self.params['ExecutionPossible'] = 'N/A'
        self.params['ObjToBePlaced'] = 'N/A'
        self.params['IfGoal'] = 'N/A'
        self.params['PlacingLocation'] = 'N/A'
        self.params['PrimitiveID'] = 'N/A'
        self.params['NumAttempts'] = 'N/A'
        self.params['ObjInTote'] = '_'
        self.params['ObjInBins'] = ['_']*3
        self.params['ObjInBoxes'] = ['_']*5
        self.params['AllGoals'] = ['_']
        self.params['NewGoals'] = ['_']
        self.params['GoalLocations'] = []
        self.params['AreSuctionPts'] = True
        self.params['AreGraspPts'] = True

        self.params['SuctionScore'] = -1
        self.params['SuctionSideScore'] = -1
        self.params['GraspingScore'] = -1

        self.score = 0
        self.endTime = int(round(time.time() + (15*60)))
        self.generateWebpage()

    def updateValue(self, attributeName, attributeValue):
        '''Update the dictionary item with inputted value,
        then regenerate the HTML page
        @param attributeName name of data value to be updated
        @param attributeValue data value to be updated'''
        if attributeName not in self.params.keys():
            return # Silently error?

        self.params[attributeName] = attributeValue

        if (self.params['TaskName'] == ('Stowing' or 'Combined_Stowing')) and (attributeName == 'ObjInBins'):
            self.score = self.computeScore('ObjInBins')
        elif (self.params['TaskName'] == ('Picking' or 'Combined_Picking')) and (attributeName == 'ObjInBoxes'):
            self.score = self.computeScore('ObjInBoxes')

        if attributeName == 'ActiveVisionObject':
            self.params['img'] = self.findImage('ActiveVisionObject')
        if attributeName == 'ActiveVisionObjectSmall':
            self.params['imgSmall'] = self.findImage('ActiveVisionObjectSmall')
        if attributeName == 'TaskName':
            #Change Task, reset timer
            self.endTime = int(round(time.time() + (15*60)))
        self.generateWebpage()

    def generateWebpage(self):
        ''' Regenerate the HTML reading in the dictionary values'''
        f = open(self.web_location, 'w')
        f.write('<html><head><title>MIT-Princeton APC Status</title>')
        f.write('<meta http-equiv="refresh" content="1.5"></head><body>')
        f.write(self.timingScript(self.endTime))
        f.write('<h1>APC Task: {}</h1>'.format(self.params['TaskName']))
        f.write('<h2 id="time" style="position:fixed;right:10px;top:10px;"></h2>')

        if self.params['TaskName'] == ('Stowing' or 'Combined_Stowing'):
            f.write(self.stowingPage())
        else: # Picking
            f.write(self.pickingPage())
        f.write('</body></html>')

    def stowingPage(self):
        '''Populate items for the Stowing Page'''
        page = '<table><tr><th>Suction Score</th><td>{0:.2f}</td></tr>'.format(self.params['SuctionScore'])
        page += '<tr><th>Suction Side Score</th><td>{0:.2f}</td></tr>'.format(self.params['SuctionSideScore'])
        page += '<tr><th>Grasping Score</th><td>{0:.2f}</td></tr></table><br/>'.format(self.params['GraspingScore'])

        page += self.generateWeightTable(range(4))
        page += '<h2 style="position:fixed;right:10px;bottom:10px;">'
        page += '<div style="background-color:green">'
        page += 'Placing {} in bin {}</div>'.format(self.params['ObjToBePlaced'], self.params['PlacingLocation'])
        page += 'Number of Objecs Left: {}<br/>'.format(self.params['ObjectsLeft'])
        page += 'Number of Attempts: {}</h2>'.format(self.params['NumAttempts'])

        page += '<br/><table border="1">' + ('<col width="150">'*4)
        page += '<tr><th>Tote ({})</th>'.format(len(self.params['ObjInTote']))
        for i in xrange(3):
            page += '<th>Bin {} ({})</th>'.format(i+1, len(self.params['ObjInBins'][i]))
        page += '</tr><tr><td>%s</td>' % '\n '.join(map(self.name, self.params['ObjInTote']))
        for i in xrange(3):
            page += '<td>%s</td>' % '\n '.join(map(self.name, self.params['ObjInBins'][i]))
        page += '</tr></table>'

        page += self.weightPrediction()
        page += 'Execution Possible: <b>{}</b><br/>'.format(self.params['ExecutionPossible'])
        page += 'Next Action: <b>{}</b></p>'.format(self.params['RobotAction'])
        page += 'Pts? <b>{}</b>. Suction Points? <b>{}</b>. Grasp Points? <b>{}</b>'.format((self.params['AreSuctionPts'] or self.params['AreGraspPts']), self.params['AreSuctionPts'], self.params['AreGraspPts'])


        page += self.generateImageTable()
        page += self.visionStateImagesStowing()
        return page

    def pickingPage(self):
        '''Popuate items for Picking page'''
        color = 'green' if self.params['IfGoal'] else 'orange'
        page = '<table><tr><th>Suction Score</th><td>{0:.2f}</td></tr>'.format(self.params['SuctionScore'])
        page += '<tr><th>Suction Side Score</th><td>{0:.2f}</td></tr>'.format(self.params['SuctionSideScore'])
        page += '<tr><th>Grasping Score</th><td>{0:.2f}</td></tr></table><br/>'.format(self.params['GraspingScore'])
        page += self.generateWeightTable(range(9))

        page += '<h2 style="position:fixed;right:10px;bottom:10px;">'
        page += '<div style="background-color:{}">Placing {}'.format(color, self.params['ObjToBePlaced'])
        page += ' in bin {}</div>'.format(self.params['PlacingLocation'])
        page += 'Number of Goals Left: {}<br/>'.format(self.params['ObjectsLeft'])
        page += 'Number of Attempts: {}</h2>'.format(self.params['NumAttempts'])

        page += self.generateGoalTable()
        page += '<br/><table border="1">' + ('<col width="150">'*8) + '<tr>'
        counts = [len(x) for x in self.params['ObjInBins']] + [len(x) for x in self.params['ObjInBoxes']]
        page += ''.join(map(str, ['<th>Bin {} ({})</th>'.format(i+1, counts[i]) for i in xrange(8)])) + '</tr>'
        for i in xrange(3):
            page += '<td>%s</td>' % '<br/> '.join(map(self.name, self.params['ObjInBins'][i]))
        for j in xrange(5):
            page += '<td>%s</td>' % '<br/> '.join(map(self.name, self.params['ObjInBoxes'][j]))
        page += '</tr></table>'

        page += self.weightPrediction()
        page += '<p>Execution Possible: <b>{}</b><br/>'.format(self.params['ExecutionPossible'])
        page += 'Next Primitive: <b>{}</b> with ID: {}</p>'.format(self.params['RobotAction'], self.params['PrimitiveID'])
        page += self.generateImageTable()
        page += self.visionStateImagesPicking()
        return page

    def visionStateImagesPicking(self):
        '''For picking we want to display information on bins 1-3. Display
        the passive vision state (2 views), grasping (1 view), flush grasping
        (1 view), suction (2 views)'''
        page = ''
        if self.state_location is None:
            return page

        binSet = [1, 2, 3]
        pre = '{}passive-vision-'.format(self.state_location)
        size = "width='128px' height='96px'"
        page += '<table>'
        page += '<tr><th>Bin #</th><th>Input</th><th>Suction (V0)</th><th>Suction (V1)</th>'
        page += '<th>Grasp</th><th>Flush Grasp</th>'
        page += '<th>State (V0)</th><th>State (V1)</th>'
        page += '<th>Background (V0)</th><th>Background (V1)</th></tr>'
        for i in binSet:
            page += '<tr><td>Bin {}</td>'.format(i)
            page += "<td><img src='{}input.{}.0.color.png' {}/></td>".format(pre, i, size)
            page += "<td><img src='{}suction.{}.0.debug.png' {}/></td>".format(pre, i, size)
            page += "<td><img src='{}suction.{}.1.debug.png' {}/></td>".format(pre, i, size)
            page += "<td><img src='{}grasp.{}.debug.png' {}/></td>".format(pre, i, size)
            page += "<td><img src='{}flush-grasp.{}.debug.png' {}/></td>".format(pre, i, size)
            page += "<td><img src='{}state.{}.0.debug.png' {}/></td>".format(pre, i, size)
            page += "<td><img src='{}state.{}.1.debug.png' {}/></td>".format(pre, i, size)
            page += "<td><img src='{}background.{}.0.color.png' {}/></td>".format(pre, i, size)
            page += "<td><img src='{}background.{}.1.color.png' {}/></td>".format(pre, i, size)
            page += '</tr>'
        page += '</table>'
        return page

    def visionStateImagesStowing(self):
        '''In the stowing task we only display for bin 0. Display 1 view
        for grasping and flush grasping and both views for suction'''
        page = ''
        if self.state_location is None:
            return page

        pre = '{}passive-vision-'.format(self.state_location)
        size = "width='128px' height='96px'"
        page += '<table><tr><th>Suction (V0)</th><th>Suction (V1)</th><th>Grasp</th>'
        page += '<th>Flush Grasp</th><th>Bin 0</th><th>Bin 1</th><th>Bin 2</th><th>Bin 3</th></tr><tr>'
        page += "<td><img src='{}suction.0.0.debug.png' {}/></td>".format(pre, size)
        page += "<td><img src='{}suction.0.1.debug.png' {}/></td>".format(pre, size)
        page += "<td><img src='{}grasp.0.debug.png' {}/></td>".format(pre, size)
        page += "<td><img src='{}flush-grasp.0.debug.png' {}/></td>".format(pre, size)
        for i in xrange(4): 
            page += "<td><img src='{}input.{}.0.color.png' {}/></td>".format(pre, i, size)
        page += '</tr></table>'

        page += '<table><tr><th>Bin 0 (bg)</th><th>Bin 1 (bg)</th>'
        page += '<th>Bin 2 (bg)</th><th>Bin 3 (bg)</th></tr><tr>'
        for i in xrange(4):
            page += "<td><img src='{}background.{}.0.color.png' {}/></td>".format(pre, i, size)
        page += '</tr><tr>'
        for i in xrange(4):
            page += "<td><img src='{}background.{}.1.color.png' {}/></td>".format(pre, i, size)
        page += '</tr></table>'

        return page

    def generateWeightTable(self, bins):
        '''Generate HTML table to display the weight values. First row is
        bin labels, second row is weight value (N/A if not available)''' 
        table = '<table border="1">'
        table += ('<col width="75">'*len(bins))
        table += '<tr style="text-align:left">'
        for i in bins:
            table += '<th>Bin {}</th>'.format(i)
        table += '</tr><tr style="text-align:right">'
        for j in bins:
            
            if self.params['WeightDiff'][j] is None:
                val = '__'
            else:
                val = int(round(self.params['WeightDiff'][j]['weights']))
            table += '<td>{} g </td>'.format(val)
        table += '</tr></table>'
        return table

    def generateGoalTable(self):
        binNames = ['1A5', '1AD', 'A1', '1B2', 'K3']
        page = '<br/><b>Goals Left</b><br/> '
        for i in xrange(len(self.params['GoalLocations'])):
            page += 'Box {} ({}): '.format(i+4, binNames[i])
            for j in self.params['GoalLocations'][i]:
                if j in self.params['NewGoals']:
                    page += '{}, '.format(j)
                else:
                    page += '<del>{}</del>, '.format(j)
            page += '<br/>'

        return page

    def generateImageTable(self):
        page = '<table><tr><th colspan="2">Active Vision</th></tr>'
        page += '<tr><td>Full List</td><td>Small List</td><td>Bin 1 Input (V0)</td><td>Bin 1 Input (V1)</td>'
        page += '<td>Bin 2 Input (V0)</td><td>Bin 2 Input (V1)</td><td>Bin 3 Input (V0)</td><td>Bin 3 Input (V1)</td></tr>'
        page += '<tr><td>{}</td><td>{}</td></tr>'.format(self.params['ActiveVisionObject'], self.params['ActiveVisionObjectSmall'])
        page += '<tr><td><img src={} width="100px" height="100px"/></td>'.format(self.params['img'])
        page += '<td><img src={} width="100px" height="100px"/></td>'.format(self.params['imgSmall'])
        if self.state_location is not None:
            pre = '{}/active_vision/'.format(self.state_location)
            for i in xrange(1, 4):
                page += "<td><img src='{}active-vision-input.{}.0.color.png' width='128px' height='96px'/></td>".format(pre, i)
                page += "<td><img src='{}active-vision-input.{}.1.color.png' width='128px' height='96px'/></td>".format(pre, i)
        page += '</tr></table>'
        return page

    def findImage(self, name):
        '''Search through the images to display the active vision image'''
        item = self.params[name]
        if self.image_folders is None:
            return ''
        folder_name = difflib.get_close_matches(item, self.image_folders)
        if len(folder_name) != 0: # found match
            loc = '{}/{}/'.format(self.image_location, folder_name[0])
            filename = '{}{}_Top-Side_01.png'.format(loc, folder_name[0])
        else:
            filename = ''
        return filename

    def timingScript(self, endTime):
        text = '<script>var x = setInterval(function() {'
        text += 'var d = new Date();'
        text += 'var seconds =  Math.round(d.getTime() / 1000);'
        text += 'var fiveMin = {};'.format(endTime)
        text += 'var timeleft = fiveMin - seconds % fiveMin;'
        text += 'var timeElapsed = (15*60) - timeleft;'
        text += 'var remaining = "Time Left: " + '
        text += 'parseInt(timeleft / 60) + " minutes, " + timeleft % 60 + " seconds";'
        text += 'var elapsed = "<br/>Time Elapsed:   " + '
        text += 'parseInt(timeElapsed / 60) + " minutes, " + timeElapsed % 60 + " seconds";'
        text += 'var score = "<br/>Score: {}";'.format(self.score)
        text += 'document.getElementById("time").innerHTML = remaining+elapsed+score;'
        text += 'if (timeElapsed < 0) {'
        text += 'document.getElementById("time").innerHTML = "TIME EXPIRED"+score;}'
        text += '}, 1); </script>'
        return text

    def weightPrediction(self):
        text = '<p>'
        if self.params['WeightDiff'][0] is not None:
            prob = self.params['WeightDiff'][0]['probs']
            items = self.params['ObjInTote']
            wp = zip(items, prob)
            if len(wp) > 0 and len(wp) > 1:
                wp.sort(key=lambda x: x[1], reverse=True)
                text += 'Weight Predicts: <b>{}</b> with '.format(wp[0][0])
                text += '{:.2%} confidence<br/>'.format(wp[0][1])
        return text

    def computeScore(self, name):
        if name == 'ObjInBins':
            scale = 5
        else:
            scale = 10
        score = 0
        objs = [item for sublist in self.params[name] for item in sublist]
        for o in objs:
            if len(difflib.get_close_matches(o, self.old_items)) > 0:
                score += scale
            else:
                score += scale*2
        return score

    def name(self, objName):
        if objName not in self.printName.keys():
            return str(objName)
        else:
            return str(self.printName[objName])

    def printNameDict(self):
        d = dict()
        d['avery_binder'] = 'binder'
        d['band_aid_tape'] = 'band_aid'
        d['black_fashion_gloves'] = 'gloves'
        d['burts_bees_baby_wipes'] = 'baby_wipes'
        d['cherokee_easy_tee_shirt'] = 'tshirt'
        d['cloud_b_plush_bear'] = 'bear'
        d['colgate_toothbrush_4pk'] = 'toothbrush'
        d['cool_shot_glue_sticks'] = 'glue_stick'
        d['creativity_chenille_stems'] = 'chenille_stems'
        d['dove_beauty_bar'] = 'dove_bar'
        d['dr_browns_bottle_brush'] = 'bottle_brush'
        d['easter_turtle_sippy_cup'] = 'sippy_cup'
        d['elmers_washable_no_run_school_glue'] = 'elmers'
        d['fiskars_scissors'] = 'scissors'
        d['folgers_classic_roast_coffee'] = 'coffee'
        d['hanes_socks'] = 'socks'
        d['hinged_ruled_index_cards'] = 'hinged_index_cards'
        d['i_am_a_bunny_book'] = 'bunny_book'
        d['kyjen_squeakin_eggs_plush_puppies'] = 'squeakin_eggs'
        d['laugh_out_loud_jokes'] = 'lol_jokes'
        d['oral_b_toothbrush_red'] = 'oral_b'
        d['peva_shower_curtain_liner'] = 'curtain_liner'
        d['plastic_wine_glass'] = 'wine_glass'
        d['platinum_pets_dog_bowl'] = 'dog_bowl'
        d['poland_spring_water'] = 'water'
        d['tennis_ball_container'] = 'tennis_balls'
        d['ticonderoga_pencils'] = 'pencils'
        d['white_facecloth'] = 'facecloth'
        d['woods_extension_cord'] = 'extension_cord'
        return d

    def oldImageList(self):
        items = ['Scotch_Sponges', 
                 'Robots_Everywhere', 'Glue_Sticks', 'Expo_Eraser', 'Marbles', 'Tissue_Box',
                 'Hanes_Socks', 'Hinged_Ruled_Index_Cards', 'Irish_Spring_Soap', 
                 'Bath_Sponge', 'Tennis_Ball_Container', 'Robots_DVD', 
                 'White_Facecloth', 'Flashlight', 'Pie_Plates', 'Reynolds_Wrap', 'I_Am_A_Bunny_Book', 
                 'Colgate_Toothbrush_4PK', 'Balloons', 'Ice_Cube_Tray', 'Speed_Stick',
                 'Mouse_Traps', 'Ticonderoga_Pencils', 'Measuring_Spoons', 
                 'Crayons', 'Laugh_Out_Loud_Jokes', 'Plastic_Wine_Glass', 'Band_Aid_Tape', 'Mesh_Cup',
                 'Toilet_Brush', 'Hand_Weight', 'Epsom_Salts', 'Avery_Binder',
                 'Composition_Book', 'Duct_Tape', 'Windex',
                 'Burts_Bees_Baby_Wipes', 'Black_Fashion_Gloves', 'Poland_Spring_Water', 
                 'Fiskars_Scissors', 'Table_Cloth']
        return items


if __name__ == '__main__':
    W = WebDisplay()
    #W.updateValue('TaskName', 'Picking')
    W.updateValue('ActiveVisionObject', 'avery_binder')
