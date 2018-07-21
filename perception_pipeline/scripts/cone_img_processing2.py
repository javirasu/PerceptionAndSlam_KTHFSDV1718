'''
==========================================================================
	Description
	-----------
		Pseudo Code: In main function:  
		1.	write a function, which depending on 'mode' flag, returns a 
            frame from either a stored image (mode = 0), stored video 
            (mode = 1), or zed camera (mode = 2, in this case, subscribe
             to zed camera image). mode = 0 => debug mode, i.e. images
             at all steps would be shown 
		2.	(optional) call detectCone1() function to generate region 
            proposals on image. Object detector replaces this function.
            The result is a list of ROIs (bounding boxes). This function
            is called 3 times, for orange, yellow and blue cones. steps:
            -   threshold image on chosen color
            -   aggregate color boxes into bigger boxes and resize 
                according to expected aspect ratio of a cone
		3.	Call detectCone2() on region proposals above. steps:
            -   threshold image on chosen color
            -   threshold image on strip color (eg yellow cone has
                black strip). join both thresholded images
            -   morphology to remove inconsistencies
            -   trapezoidal shape detection. 
        4.  If detected as a cone and if using object detector:
            -   Call detectColor() to inspect histogram of segmented cone
                and detect color of cone.

    Color Convention:
    ---------------- 
        0-  YELLOW
        1-  BLUE
        2-  ORANGE
        3-  WHITE
        4-  BLACK

==========================================================================
	History
	-------
		Version No.			Date			Author
		------------------------------------------
		1.x					2018/06/19		Ajinkya
		
	Revision Description
	--------------------
		1.x		---	Assume image coming from object detector, write only 
                    detectCone2 
'''
import cv2 
import numpy as np
import os
import matplotlib.pyplot as plt

class ConeColor:
    def __init__(self, minVal, maxVal, code):
        # Set H, S , V values
        # self.hmin = minVal[0]
        # self.smin = minVal[1]
        # self.vmin = minVal[2]
        self.lower = minVal

        # self.hmax = maxVal[0]
        # self.smax = maxVal[1]
        # self.vmax = maxVal[2]
        self.upper = maxVal 
        
        # Set color code
        self.colorCode = code

yellow = ConeColor(np.array([20,100,100]), np.array([30,256,256]), 0)
blue = ConeColor(np.array([85,70,21]), np.array([128,256,256]), 1)
orange = ConeColor(np.array([160,40,120]), np.array([8,256,256]), 2)
white = ConeColor(np.array([0,0,190]), np.array([180,116,256]), 3)
black = ConeColor(np.array([0,0,0]), np.array([180,256,50]), 4)

def morphOpen(image):
    # define structuring element
    # take 5% of least dimension of image as kernel size
    kernel_size = min(5, round(min(image.shape[0],image.shape[1])*0.05))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(kernel_size,kernel_size))
    #kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    opening = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    return opening

def checkTrapezoid(image):
    count = 0
    bBox = []
    '''
    CONE SHAPE DETECTION Algorithm- Take a convex hull and:
    STEP 1.	find highest and lowest points in hull, using that find vertical center
    STEP 2.	using vertical center make list of points above and below vertical center
    STEP 3.	find left most and right most points in bottom part of list 
    STEP 4.	(a)	If height/ width of convex hull >=1.25
                                AND
            (b)	if list of pts above vertical center are within left most and right most points

    => we have a cone! otherwise not
    '''
    # find contours
    _, contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for points in contours:
        if points.shape[0] < 4:
            continue
        # We don't really need convex hull here 
        #hull = cv2.convexHull(points,returnPoints = False)

        # find vertical center, 'ymid'
        ymid = (max([a[0][1] for a in points]) + min([a[0][1] for a in points]))/2

        # make two sets of points, one above and one below ymid
        temp = [a[0][1]< ymid for a in points]
        above = points[temp]
        below = points[np.invert(temp)]

        # find x coordinate of topLeft, topRight, bottom left, bottom right points
        x_tl = min([a[0][0] for a in above])
        x_tr = max([a[0][0] for a in above])
        x_bl = min([a[0][0] for a in below])
        x_br = max([a[0][0] for a in below])

        if abs(x_br - x_bl) > abs(x_tr - x_tl):
            count = count + 1
            x,y,w,h = cv2.boundingRect(points)
            bBox.append(np.array([x,y,w,h]))
    return count, bBox

def detectCone2(frame):
    bBoxMain = []

    global yellow
    global blue
    global orange
    global white
    global black

    # blur image
    frame = cv2.GaussianBlur(frame,(5,5),0)
    # convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for bodyColor in [yellow, blue]:
        if bodyColor.colorCode is 0:
            stripColor = black
        else:
            stripColor = white

        # threshold on strip color
        stripThresh = cv2.inRange(hsv, stripColor.lower, stripColor.upper)
        # Perform morphological opening
        stripMorph = morphOpen(stripThresh)
        # count number of cone strips
        countStrip, bBoxStrip = checkTrapezoid(stripMorph)
        # display image
        cv2.imshow("stripMorph", stripMorph)
        cv2.waitKey(10)

        # threshold on bodyColor
        if bodyColor.colorCode is 2:
            lower1 = bodyColor.lower
            upper1 = np.zeros((3))
            upper1[0] = 179
            upper1[1] = bodyColor.upper[1]
            upper1[2] = bodyColor.upper[2]
            
            mask1 = cv2.inRange(hsv, lower1, upper1)

            lower2 = np.zeros((3,1))
            lower2[0] = 0
            lower2[1] = bodyColor.lower[1]
            lower2[2] = bodyColor.lower[2]
            upper2 = bodyColor.upper

            mask2 = cv2.inRange(hsv, lower2, upper2)

            bodyThresh = mask1 + mask2
        else:
            bodyThresh = cv2.inRange(hsv, bodyColor.lower, bodyColor.upper)  
        
        # add bodyThresh and stripThresh
        #coneThresh = bodyThresh + stripThresh
        coneThresh = bodyThresh
        coneMorph = morphOpen(coneThresh)
        # display image
        cv2.imshow('coneMorph', coneMorph)
        cv2.waitKey(10)
        countCone, bBoxCone = checkTrapezoid(coneMorph)

        if countCone == countStrip and countCone >0:
            # there's >= 1 cone in image. 
            bBoxMain.append(bBoxCone)
        elif countCone > 0 and countStrip > 0:
            # there's >=0 cones in image but unsure.
            # reason might be that they are too close 
            # and occlude each other. If so, use 
            # bBoxStrip
            bBoxMain.append(bBoxStrip)
    return bBoxMain

def countPixels(frame, hsv, colorCode):
    global yellow
    global blue
    global orange
    global white
    global black

    # temp = ConeColor()

    if colorCode == yellow.colorCode:
        temp = yellow
    elif colorCode == blue.colorCode:
        temp = blue
    elif colorCode == orange.colorCode:
        temp = orange
    elif colorCode == white.colorCode:
        temp = white
    elif colorCode == black.colorCode:
        temp = black

    # threshold on bodyColor
    if temp.colorCode is 2:
        lower1 = temp.lower
        upper1 =  np.zeros((3,), dtype=int)
        upper1[0] = 179
        upper1[1] = temp.upper[1]
        upper1[2] = temp.upper[2]
        
        mask1 = cv2.inRange(hsv, lower1, upper1)

        lower2 = np.zeros((3,), dtype=int)
        lower2[0] = 0
        lower2[1] = temp.lower[1]
        lower2[2] = temp.lower[2]
        upper2 = temp.upper

        mask2 = cv2.inRange(hsv, lower2, upper2)

        mask = mask1 + mask2
    else:
        mask = cv2.inRange(hsv, temp.lower, temp.upper)

    # if color == 'orange':
    #     lower1 = np.array([ColorDict[color]['H_MIN'],ColorDict[color]['S_MIN'], ColorDict[color]['V_MIN']])
    #     upper1 = np.array([179, ColorDict[color]['S_MAX'], ColorDict[color]['V_MAX']])
    #     mask1 = cv2.inRange(hsv, lower1, upper1) 
    #     #showImage(frame, mask1)
    #     lower2 = np.array([0,ColorDict[color]['S_MIN'], ColorDict[color]['V_MIN']])
    #     upper2 = np.array([ColorDict[color]['H_MAX'], ColorDict[color]['S_MAX'], ColorDict[color]['V_MAX']])
    #     mask2 = cv2.inRange(hsv, lower2, upper2)
    #     mask = mask1 + mask2
    #     #showImage(frame, mask2)
    # else:
    #     # Threshold the HSV image to get only blue colors
    #     lower = np.array([ColorDict[color]['H_MIN'], ColorDict[color]['S_MIN'], ColorDict[color]['V_MIN']])
    #     upper = np.array([ColorDict[color]['H_MAX'], ColorDict[color]['S_MAX'], ColorDict[color]['V_MAX']])
    #     mask = cv2.inRange(hsv, lower, upper)
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
    #hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    histo = []
    #Count percentage of pixels of a particular color
    histo.append(countPixels(frame, hsv, 0))    #yellow
    histo.append(countPixels(frame, hsv, 1))    #blue
    histo.append(countPixels(frame, hsv, 2))    #orange 
    histo.append(countPixels(frame, hsv, 3))    #white
    histo.append(countPixels(frame, hsv, 4))    #black

    # print(np.mean(hsv[:,:,2]))
    # if np.mean(hsv[:,:,2]) < 130:
    #     # Image is dark
    #     # compare white and black histograms
    #     if histo[3] < histo[4]:
    #         result = 0
    #     else:
    #         if histo[2] > 0:
    #             result = 2
    #         else:
    #             result = 1
    # else:
    result = np.argmax(histo[0:3])
    if result == 1:
        if histo[1] > 75:
            result = 1
        elif histo[0] > 10:
            result = 0
        elif histo[2] > 10:
            result = 2
    
    if result == 0:
        if round(histo[2]) > 0:
            result = 2    


    #result= np.argmax((w0, w1, w2))
    '''
    result =    0:  YELLOW CONE
                1:  BLUE CONE
                2:  ORANGE CONE
    '''
    return result

def onkeypress(event):
    if event.key == 'q':
        plt.close()

def main(mode):
    desired_img_number = 0
    image_count = 0  
    cwd = os.getcwd()
    image_folder = './test_videos/cone_samples/'
    if mode is 0:
        for image_file in (os.listdir(image_folder)):
            img = image_file
            fig, ax = plt.subplots(1)
            image = cv2.imread(image_folder + image_file)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            ax.imshow(image)    
            #frame = cv2.imread(os.path.join(folder,image_file))

            output = detectCone1(image)

            if output == 0:
                print("Yellow Cone")
            elif output == 1:
                print('Blue Cone')
            elif output == 2:
                print('Orange Cone')
            else:
                print('!!!NO CONE!!!')

            key = plt.connect('key_press_event', onkeypress)
            plt.show()

            # cv2.imshow('original image', frame)
            # cv2.waitKey(0)
    elif mode is 1:
        #img = image_file
        fig, ax = plt.subplots(1)
        image = cv2.imread(image_folder + '114.jpg')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        ax.imshow(image)    
        #frame = cv2.imread(os.path.join(folder,image_file))

        output = detectCone1(image)

        if output == 0:
            print("Yellow Cone")
        elif output == 1:
            print('Blue Cone')
        elif output == 2:
            print('Orange Cone')
        else:
            print('!!!NO CONE!!!')

        key = plt.connect('key_press_event', onkeypress)
        plt.show()
    
if __name__ == '__main__':
    mode = 1
    main(mode)