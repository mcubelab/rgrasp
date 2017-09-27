#!/usr/bin/env python
import time
from matplotlib import pyplot as plt
from matplotlib import cm
from matplotlib import mlab as ml
import numpy as np
import os
import vlc
from gtts import gTTS
import rospy
import sys
# Constant variables

# Levels of verbosity and themes
VERBOSITY = {"silence" : 0, "critical" : 1, "important" : 2, "normal" : 3, "verbose" : 4, "all" : 5}
THEMES = {"hardware", "planner", "manipulation", "debug"}

class Explainer(object):
    def __init__(self, verbosity = 4, current_verbosity = 4, current_theme = 'debug', can_speek = True, gTTS = True):
        try:
            verbosity = int(verbosity)
            verbosity = max(0,verbosity)
            verbosity = min(5,verbosity)
        except:
            if verbosity in VERBOSITY:
                verbosity = VERBOSITY[verbosity]
            else:
                verbosity = 0
                
        #self.logfile = file('/home/mcube/arc/catkin_ws/src/apc_planning/src/'+'planner_log'+'.' + str(rospy.get_time()), 'a')
        # Level of verbosity allowed during display
        self.verbosity = verbosity
        # Status of each theme: if filtered, it will not be displayed
        self.filtered_theme = { _ : False for _ in THEMES}
        self.current_verbosity = max(1,current_verbosity)
        self.current_theme = current_theme
        #Speech parameters
        self.can_speek = can_speek
        self.gTTS = gTTS
        self.gTTS_dir = '../../../../doc/speech/'
        self._buffered_speech = []
        self._p = vlc.Instance().media_player_new()
        self._state = 'free'
    
    # Changes the allowed verbosity
    def set_verbosity(self, level):
        try:
            level = min(5, level)
            level = max(0, level)
            self.verbosity = level
        except:
            pass
        if level in VERBOSITY:
            self.verbosity = VERBOSITY[level]
            
    # Changes the default verbosity of the upcoming messages
    def set_current_verbosity(self, level):
        try:
            level = min(5, level)
            level = max(1, level) #The minimum is at 1, cannot be silence
            self.current_verbosity = level
        except:
            pass

        if level in VERBOSITY:
            self.current_verbosity = max(1,VERBOSITY[level])
    
    # Changes the default theme of the upcoming messages
    def set_current_theme(self, theme):
        if theme in THEMES:
            self.current_theme = theme
        else:
            print '[EXPLAINER] ', theme, 'is not a valid theme'

    def playMP3(self, event = None):
        print 'Current buffer: ', self._buffered_speech
        if event != None and event.type != vlc.EventType.MediaPlayerEndReached:
            print event
            assert False
            return
        if event != None: #MediaPlayerEndReached
            self._state = "free"
        if self._buffered_speech == []: #Nothing to do here
            return
        #Play file!
        self._state = "busy"
        s = self._buffered_speech.pop(0)
        self._p = vlc.MediaPlayer(s)
        events = self._p.event_manager()
        events.event_attach(vlc.EventType.MediaPlayerEndReached, self.playMP3)
        #CMPFUNC = CFUNCTYPE(c_int)
        #p.audio_set_callbacks(CMPFUNC(self.callback),None,None,None,None,None)
        self._p.play()

    def speak(self, s = "Hi, I'm the M Cube robot, I've learned to talk!"):
        if not self.can_speek:
            return
        filename = s.replace(' ','_').replace('-','_').lower() + ".mp3"
        if s not in os.listdir(self.gTTS_dir):
            if not self.gTTS:
                return
            else:
                tts = gTTS(text = s, lang = 'en')
                tts.save(self.gTTS_dir + filename)
        self._buffered_speech.append(self.gTTS_dir + filename)
        #self._buffered_speech.append(self.gTTS_dir + filename)
        if self._state == 'free':
            self.playMP3()
        else:
            print 'cannot play'
            
    def write(self, message):
        print "I'm writing"
        sys.stdout.write(message)
        #self.logfile.write(message)
    
    def close(self):
        self.logfile.close()
        
    # Main function for creating messages and debug
    def db(self, *arr, **arr_vars):
        dont_print = ['theme', 'level']
        level = self.current_verbosity
        if 'level' in arr_vars:
            level = arr_vars['level']
            try:
                level = int(level)
            except ValueError:
                if level in VERBOSITY:
                    level = VERBOSITY[level]
                else:
                    level = 6
        if 'theme' in arr_vars:
            theme = arr_vars['theme']
            if theme not in THEMES:
                theme = self.current_theme
        else:
            theme = self.current_theme
        try:
            value = int(level)
            if max(1,level) > self.verbosity:
                return
        except ValueError:
            if level in VERBOSITY:
                if max(1,VERBOSITY[level]) > self.verbosity:
                    return
            else:
                return
        try:
            if self.filtered_theme[theme]:
                return
        except:
            return
        s = ''
        for x in arr:
            try:
                s += str(x) + ' '
            except:
                pass
        if s != '':
            print '[',theme.upper(),']', s
            #sys.stdout.write('['+theme.upper()+']'+ str(s))
        for k,v in arr_vars.iteritems():
            if k not in dont_print:
                print '[',theme.upper(),']',k,' = ',v
                
                #sys.stdout.write('['+theme.upper()+']'+str(k)+ ' = ' + str(v))

    def heatmap(self, X, Y, Z, names):
        x = y = np.linspace(-5,5,100)
        X,Y = np.meshgrid(x,y)
        z = [ml.bivariate_normal(X,Y,2,2,0,0), ml.bivariate_normal(X,Y,4,1,1,1)]
        Z = z + [z[1]-z[0]]
        name = 0
        x = X.ravel()
        y = Y.ravel()
        gridsize = 30
        if len(Z) > 1:
            print 'Multiple subplots'
            fig, axarr = plt.subplots(len(Z))
        else:
            fig, axarr = plt.subplots()
        c = 0
        cmaps = [cm.Reds, cm.Greens, cm.Blues]
        for ax in axarr.flat:
            print 'ax', ax
            z = Z[c].ravel()
            c = c + 1
            im = ax.hexbin(x, y, C=z, gridsize=gridsize, cmap=cmaps[c-1], bins=None)
        #for i in range(len(Z)):
            #z = Z[i].ravel()
            #axarr[i].hexbin(x, y, C=z, gridsize=gridsize, cmap=cm.Reds, bins=None)
            #axarr[i].axis([x.min(), x.max(), y.min(), y.max()])
            #axarr[i] = plt.colorbar()
            #axarr[i].set_label(name)
        fig.colorbar(im, ax = axarr.ravel().tolist())
        plt.show() 
        #time.sleep(10)
        fig.subplots_adjust(hspace = 0.3)
