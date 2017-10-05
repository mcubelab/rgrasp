import argparse
import cv2
import numpy as np
import os
import ctypes
import Tkinter as tk # sudo apt-get install python-tk
import tkMessageBox



class PlacingInterface(object):

    def __init__(self):

        self.eventStep = 0
        self.selectedObjInd = -1
        self.suggestedBin = 0
        self.suggestionFunction = None
        self.numObjSelected = 0

        cmptnItemDataPath = os.environ['DATA_BASE'] + '/itemdata'
        oldItemDataPath = os.environ['DATA_BASE'] + '/itemdata_amazon'

        # List all competition objects in item data
        self.cmptnObjList = []
        for dirname, cmptnObjNames, filenames in os.walk(cmptnItemDataPath):
            for cmptnObjName in cmptnObjNames:
                if cmptnObjName != 'Empty':
                    self.cmptnObjList.append(cmptnObjName)
        self.cmptnObjList.sort()

        # List all old objects
        oldObjList = []
        for dirname, oldObjNames, filenames in os.walk(oldItemDataPath):
            for oldObjName in oldObjNames:
                if oldObjName != 'Empty':
                    oldObjList.append(oldObjName)
        oldObjList.sort()

        # Rearrange competition objects by old, followed by new
        cmptnOldObjList = []
        cmptnNewObjList = []
        for cmptnObjName in self.cmptnObjList:
            if any(str.lower(cmptnObjName) in str.lower(s) for s in oldObjList):
                cmptnOldObjList.append(cmptnObjName)
            else:
                cmptnNewObjList.append(cmptnObjName)
        self.cmptnObjList = cmptnOldObjList + cmptnNewObjList

        # Compute product image grid locations and paths
        self.objGridMidX = []
        self.objGridMidY = []
        objProdImgPaths = []
        for i in range(len(self.cmptnObjList)):
            self.objGridMidX.append(int((i%10)*150+75))
            self.objGridMidY.append(int((np.floor(i/10))*170+85))
            objItemDataPath = os.path.join(cmptnItemDataPath, self.cmptnObjList[i])
            for imgPath in os.listdir(objItemDataPath):
                if imgPath.lower().endswith("top_01.png"):  # tricky amazon images, they use different cases
                    objProdImgPaths.append(os.path.join(objItemDataPath, imgPath))
                    break
            
            if len(objProdImgPaths) != i+1:  # not yet found a picture
                print 'No top img for ', self.cmptnObjList[i], ' let\'s find another'
                for imgPath in os.listdir(objItemDataPath):
                    if imgPath.endswith(".png"):
                        objProdImgPaths.append(os.path.join(objItemDataPath, imgPath))
                        break
                    

        # Create a blank canvas
        canvasHeight = 170*np.ceil(len(self.cmptnObjList)/10.0)
        canvasWidth = 150*10+200
        self.blankCanvas = np.ones((canvasHeight,canvasWidth,3), np.uint8)*255

        # Fill in blank canvas with product images
        for i in range(len(self.cmptnObjList)):
            tmpProdImg = cv2.imread(objProdImgPaths[i])
            tmpProdImg = cv2.resize(tmpProdImg,(130, 130), interpolation = cv2.INTER_CUBIC)
            print(self.objGridMidY[i])
            self.blankCanvas[(self.objGridMidY[i]-55):(self.objGridMidY[i]+75),(self.objGridMidX[i]-65):(self.objGridMidX[i]+65)] = tmpProdImg
            cv2.putText(self.blankCanvas,self.cmptnObjList[i],(self.objGridMidX[i]-65,self.objGridMidY[i]-65), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0,0,0),1)

    def placing_interface_mouse_events(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            if self.eventStep == 0:
                gridX = np.floor(x/150)
                gridY = np.floor(y/170)
                if gridX < 10:
                    self.selectedObjInd = int(gridY*10+gridX)
                    root = tk.Tk()
                    root.withdraw()
                    confirmMsg = tkMessageBox.askokcancel("Confirm","Did you choose {}?".format(self.cmptnObjList[self.selectedObjInd]))
                    if not confirmMsg:
                        return
                    self.suggestedBin = self.suggestionFunction(self.cmptnObjList[self.selectedObjInd])
                    self.selectionCanvas = self.blankCanvas.copy()
                    cv2.rectangle(self.selectionCanvas,(self.objGridMidX[self.selectedObjInd]-65,self.objGridMidY[self.selectedObjInd]-55),(self.objGridMidX[self.selectedObjInd]+65,self.objGridMidY[self.selectedObjInd]+75),(0,255,0),5)
                    self.numberCanvas = self.selectionCanvas.copy()
                    cv2.putText(self.numberCanvas,str(self.suggestedBin),(self.objGridMidX[self.selectedObjInd]-50,self.objGridMidY[self.selectedObjInd]+65), cv2.FONT_HERSHEY_SIMPLEX, 5,(0,255,0),5)
                    cv2.imshow("Interactive Placing", self.numberCanvas)
            self.eventStep = 1
        # if event == cv2.EVENT_RBUTTONDOWN:
        #     cv2.imshow("Interactive Placing", self.blankCanvas)
        #     self.eventStep = 0
        # if event == cv2.EVENT_MBUTTONDOWN:
        #     cv2.imshow("Interactive Placing", self.blankCanvas)
        #     self.eventStep = 0

    def force_select_object(self, objectName):

        for i in range(len(self.cmptnObjList)):
            if str.lower(self.cmptnObjList[i]) == str.lower(objectName):
                self.selectedObjInd = i
        self.selectionCanvas = self.blankCanvas.copy()
        cv2.rectangle(self.selectionCanvas,(self.objGridMidX[self.selectedObjInd]-65,self.objGridMidY[self.selectedObjInd]-55),(self.objGridMidX[self.selectedObjInd]+65,self.objGridMidY[self.selectedObjInd]+75),(0,255,0),5)
        self.numberCanvas = self.selectionCanvas.copy()
        cv2.putText(self.numberCanvas,str(self.suggestedBin),(self.objGridMidX[self.selectedObjInd]-50,self.objGridMidY[self.selectedObjInd]+65), cv2.FONT_HERSHEY_SIMPLEX, 5,(0,255,0),5)
        cv2.imshow("Interactive Placing", self.numberCanvas)
        self.eventStep = 1



# # Start interactive placing interface
# ipif = PlacingInterface()
# cv2.namedWindow("Interactive Placing")
# cv2.setMouseCallback("Interactive Placing", ipif.placing_interface_mouse_events)
# cv2.imshow("Interactive Placing", ipif.blankCanvas)

# # Wait for next key
# key = cv2.waitKey(1)
# while key == 255:
#     key = cv2.waitKey(1)

# obj = 'crayons'
# ipif.suggestedBin = 1
# ipif.force_select_object(obj)

# def helper_function(objectName):
#     print('called helper function: {}'.format(objectName))
#     return 1

# ipif.suggestionFunction = helper_function

# while True:
#     key = cv2.waitKey(1) & 0xFF
#     if key < 255:
#         if ipif.eventStep == 1:
#             # If number keys 1 - 3 are pressed
#             if key >= 49 and key <= 51:
#                 key = key-48
#                 ipif.numberCanvas = ipif.selectionCanvas.copy()
#                 ipif.suggestedBin = key
#                 cv2.putText(ipif.numberCanvas,str(ipif.suggestedBin),(ipif.objGridMidX[ipif.selectedObjInd]-50,ipif.objGridMidY[ipif.selectedObjInd]+65), cv2.FONT_HERSHEY_SIMPLEX, 5,(0,255,0),5)
#                 cv2.imshow("Interactive Placing", ipif.numberCanvas)

#             # If enter is pressed
#             if key == 10:
#                 cv2.imshow("Interactive Placing", ipif.blankCanvas)
#                 root = tk.Tk()
#                 root.withdraw()
#                 tkMessageBox.showwarning("Confirm","Is {} placed in bin {} and is Shuran's hand out?".format(ipif.cmptnObjList[ipif.selectedObjInd],ipif.suggestedBin))
#                 print("{} placed in bin {}".format(ipif.cmptnObjList[ipif.selectedObjInd],ipif.suggestedBin))
#                 ipif.eventStep = 0
#                 break
