#!/usr/bin/env python

# call this function with the weights in grams and you'll get the safe dropping height in
# centimeters
import rospy
import json
import numpy as np

def look_up_table(weight):
    if weight<100 and weight>0:
        return 15
    if weight<200 and weight>100:
        return 15
    if weight<250 and weight>200:
        return 10
    if weight<400 and weight>250:
        return 5
    if weight<600 and weight>400:
        return 2
    if weight<1000 and weight>600:
        return 1

def main(weight):
    if weight<1:
        print '[WS] Enter weights in grams, making the adjustment'
        weight = weight*1000.0
        
    print '[WS] Drop height in CMs is'
    print look_up_table(weight)
    return 0

if __name__=="__main__":
    sys.exit(main(sys.argv[1]))
