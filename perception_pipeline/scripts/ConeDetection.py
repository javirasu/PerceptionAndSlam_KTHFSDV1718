'''
Author:     Ajinkya
Date:       2018/05/12
Description:Takes a frame as input and outputs type of cone
Color Convention: 
0-  YELLOW
1-  BLUE
2-  ORANGE
3-  WHITE
4-  BLACK
'''

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import random
import cv2


def showImage(frame, mask):
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)
    cv2.imshow('res', res)
    #cv2.waitKey(50)
    k = cv2.waitKey(5000) & 0xFF
    #if k == 27:
    #    break

    cv2.destroyAllWindows()

def countPixels(frame, hsv, color):
    if color == 'orange':
        lower1 = np.array([ColorDict[color]['H_MIN'],ColorDict[color]['S_MIN'], ColorDict[color]['V_MIN']])
        upper1 = np.array([179, ColorDict[color]['S_MAX'], ColorDict[color]['V_MAX']])
        mask1 = cv2.inRange(hsv, lower1, upper1) 
        #showImage(frame, mask1)
        lower2 = np.array([0,ColorDict[color]['S_MIN'], ColorDict[color]['V_MIN']])
        upper2 = np.array([ColorDict[color]['H_MAX'], ColorDict[color]['S_MAX'], ColorDict[color]['V_MAX']])
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = mask1 + mask2
        #showImage(frame, mask2)
    else:
        # Threshold the HSV image to get only blue colors
        lower = np.array([ColorDict[color]['H_MIN'], ColorDict[color]['S_MIN'], ColorDict[color]['V_MIN']])
        upper = np.array([ColorDict[color]['H_MAX'], ColorDict[color]['S_MAX'], ColorDict[color]['V_MAX']])
        mask = cv2.inRange(hsv, lower, upper)
    # Optional: show images
    #showImage(frame, mask)
    per = cv2.countNonZero(mask)/float(mask.size)*100
    return per

def detectCone1(frame):
    # FUTURE: check first if image is RGB or BGR?
    # Based on that next operation has to be modified/or not
    # cv2.imshow('FRAME',frame)
    # convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    histo = []
    #Count percentage of pixels of a particular color
    histo.append(countPixels(frame, hsv, 'yellow'))
    histo.append(countPixels(frame, hsv, 'blue'))
    histo.append(countPixels(frame, hsv, 'orange'))
    histo.append(countPixels(frame, hsv, 'white'))
    histo.append(countPixels(frame, hsv, 'black'))

    w0 = 0.8*histo[0] + 0.2*histo[4]    #weight of yellow cone
    w1 = 0.8*histo[1] + 0.2*histo[3]    #weight of blue cone
    w2 = 0.8*histo[2] + 0.2*histo[3]    #weight of yellow cone

    result= np.argmax((w0, w1, w2))
    '''
    result =    0:  YELLOW CONE
                1:  BLUE CONE
                2:  ORANGE CONE
    '''
    return result
    #Plot histogram
    # plt.hist(histo)
    # plt.ylabel('Percentage of pixels of a color')

ColorDict = {'yellow': {'H_MIN': 20, 'H_MAX': 30, 'S_MIN': 100, 'S_MAX': 256, 'V_MIN': 100, 'V_MAX': 256}, 'blue': {'H_MIN': 85, 'H_MAX': 128, 'S_MIN': 70, 'S_MAX': 256, 'V_MIN': 21, 'V_MAX': 256}, 'orange': {'H_MIN': 160, 'H_MAX': 8 ,
                                                                                                                                                                                                                 'S_MIN': 40, 'S_MAX': 256, 'V_MIN': 120, 'V_MAX': 256}, 'black': {'H_MIN': 0, 'H_MAX': 180, 'S_MIN': 0, 'S_MAX': 256, 'V_MIN': 0, 'V_MAX': 50}, 'white': {'H_MIN': 0, 'H_MAX': 180, 'S_MIN': 0, 'S_MAX': 116, 'V_MIN': 190, 'V_MAX': 256}}
