# -*- coding: utf-8 -*-
"""
Created on Sat Nov  7 22:57:48 2020

@author: saulb
"""

import numpy as np
import cv2 as cv
import math
from matplotlib import pyplot as plt
from tensorflow.keras.preprocessing.image import img_to_array, array_to_img
from tensorflow.keras.models import load_model
import sys
import argparse
from classes import PRMController, Obstacle, Utils

def process():
    #Inicializar imagen
    img = cv.imread('..\cameraControllerPython\Image\map.jpg')
    OrigImag = img.copy()
    
    # Load image
    image_bgr = img
    # Convert to RGB
    image_rgb = cv.cvtColor(image_bgr, cv.COLOR_BGR2RGB)
    # Rectange values: start x, start y, width, height
    rectangle = (25, 25, img.shape[1]-50, img.shape[0]-50)
    # Create initial mask
    mask = np.zeros(image_rgb.shape[:2], np.uint8)
    # Create temporary arrays used by grabCut
    bgdModel = np.zeros((1, 65), np.float64)
    fgdModel = np.zeros((1, 65), np.float64)
    # Run grabCut
    cv.grabCut(image_rgb, # Our image
                mask, # The Mask
                rectangle, # Our rectangle
                bgdModel, # Temporary array for background
                fgdModel, # Temporary array for background
                5, # Number of iterations
                cv.GC_INIT_WITH_RECT) # Initiative using our rectangle
    # Create mask where sure and likely backgrounds set to 0, otherwise 1
    mask_2 = np.where((mask==2) | (mask==0), 0, 1).astype('uint8')
    # Multiply image with new mask to subtract background
    image_rgb_nobg = image_rgb * mask_2[:, :, np.newaxis]
    # Show image
    plt.imshow(image_rgb_nobg), plt.axis("off")
    plt.show()
    img=cv.cvtColor(image_rgb_nobg, cv.COLOR_RGB2BGR)
    
    
    imgray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(imgray, 5, 255, 0)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    ctrs = img.copy()
    cv.drawContours(ctrs, contours, -1, (0,255,0), 3)
    
    ###cv.imshow('Images', ctrs)
    #print(contours) 
    #print(hierarchy)
    
    
    decode_preds = {0: 'car', 1: 'destination', 2: 'house', 3: 'origin', 4: 'tree', 5: 'truck', 6: 'windmill'}
    model = load_model('drone_vision_vgg16.h5')
    
    
    xL = OrigImag.shape[0]/100
    yL = OrigImag.shape[1]/100
    current = str(round(153/xL))+", "+str(round(478/yL))
    destination = str(round(1034/xL))+", "+str(round(184/yL))
    obstacles = np.array(["empty"])
    
    divisions = img.copy()
    i=0
    #print(len(contours))
    while i < len(contours):
        #print(hierarchy[0][i][3])
        (x,y,w,h) = cv.boundingRect(contours[i])
        #print((x,y,w,h))
        if(hierarchy[0][i][3]==-1 and contours[i].shape[0]>3 and w>10 and h>10):
            #print(cont)
            print("Number: "+str(i))
            
            divisions = cv.rectangle(divisions, (x,y), (x+w,y+h), (255, 0, 0), 2)
            #imagTopredict = OrigImag[round(x+w/2-75):round(x+w/2+75), round(y+h/2-75):round(y+h/2+75), :].copy()
            imagTopredict = OrigImag[y:y+h,x:x+w, :].copy()
            #print(imagTopredict.shape)
            imagTopredict = cv.resize(imagTopredict, (150, 150))
            #imagTopredict = array_to_img(imagTopredict)
             # prepare the image for the VGG model
            imagTopredict = imagTopredict/255
            imagTopredict = np.array(imagTopredict)[:, :, 0:3]
            # convert the image pixels to a numpy array
            ###imagTopredict = img_to_array(imagTopredict)
            # reshape data for the model
    #        if(imagTopredict.shape[0]<imagTopredict.shape[1]):
    #            extension = imagTopredict.shape[0]
    #        else:
    #            extension = imagTopredict.shape[1]
           
            #imagTopredict = np.stack([imagTopredict], axis=0)
            imagTopredict = imagTopredict.reshape(-1, 150, 150, 3)
            #imagTopredict = imagTopredict.reshape((1, 150, 150, 3))
            #{'DecodeJpeg:0': imagTopredict}
            # predict the probability across all output classes
            prediction = model.predict(imagTopredict)
            if(contours[i].shape[0]==4):
                prediction[0][3] = 1
            strPred = decode_preds[np.argmax(prediction)]
            strMsg = strPred +' ('+str(round(prediction[0][np.argmax(prediction)] * 100, 2))+'%)'
            #print(strMsg)
            divisions = cv.putText(divisions, strMsg,(x-40, y), cv.FONT_HERSHEY_SIMPLEX , 1, (255, 0, 0), 2)
            print(f'{decode_preds[np.argmax(prediction)]} ({round(prediction[0][np.argmax(prediction)] * 100, 2)}%)')
            print(cv.HuMoments(cv.moments(contours[i])).flatten())
            
            if(strPred == 'origin'):
                current = str(round((x+w/2)/xL))+","+str(round((y+h/2)/yL))
            elif(strPred == 'destination'):
                destination = str(round((x+w/2)/xL))+","+str(round((y+h/2)/yL))
            else:
                temp = str(round(x/xL))+","+str(round(y/yL))+";"+str(round((x+w)/xL))+","+str(round((y+h)/yL))
                obstacles = np.concatenate((obstacles,[temp]), axis=0)
            
            i=i+1
        else:
            contours.pop(i)
            hierarchy = np.delete(hierarchy, i, 1)
    
    ctrs = img.copy()
    cv.drawContours(ctrs, contours, -1, (0,255,0), 3)
    #print
    ###cv.imshow('Images 2', ctrs)
    cv.imshow('Divisions', divisions)
    cv.imwrite("images\divisions.png", divisions)
    
    '''
    Probabilistic Road Map
    '''
    
    parser = argparse.ArgumentParser(description='PRM Path Planning Algorithm')
    parser.add_argument('--numSamples', type=int, default=500, metavar='N',
                        help='Number of sampled points')
    args = parser.parse_args()
    
    numSamples = args.numSamples
    
    current = list(map(int, current.split(",")))
    destination = list(map(int, destination.split(",")))
    
    print("Current: {} Destination: {}".format(current, destination))
    
    print("****Obstacles****")
    allObs = []
    for l in obstacles:
        if(";" in l):
            line = l.strip().split(";")
            topLeft = list(map(int, line[0].split(",")))
            bottomRight = list(map(int, line[1].split(",")))
            obs = Obstacle(topLeft, bottomRight)
            obs.printFullCords()
            allObs.append(obs)
    
    utils = Utils()
    utils.drawMap(allObs, current, destination)
    
    numSamples= 500
    prm = PRMController(numSamples, allObs, current, destination)
    # Initial random seed to try
    initialRandomSeed = 0
    xSol, ySol = prm.runPRM(initialRandomSeed)
    xSol = xL*np.array(xSol)
    ySol = yL*np.array(ySol)
    
    #Print Solution maps
    solutionImg = divisions.copy()
    j=0
    for j in range(len(ySol)-1):
        solutionImg = cv.line(solutionImg, (int(xSol[j]), int(ySol[j])), (int(xSol[j+1]), int(ySol[j+1])), (255, 0, 0), 2)
    cv.imshow('Soution', solutionImg)
    ###cv.imwrite("images\solution.png", solutionImg)
    
    solutionImgO = OrigImag.copy()
    j=0
    for j in range(len(ySol)-1):
        solutionImgO = cv.line(solutionImgO, (int(xSol[j]), int(ySol[j])), (int(xSol[j+1]), int(ySol[j+1])), (255, 0, 0), 2)
    cv.imshow('Soution Map', solutionImgO)
    ###cv.imwrite("images\solutionMap.png", solutionImgO)
    
    #xSol = -0.064637985309549*(xSol-157)
    #ySol = -0.065317919075145*(ySol-539)
    
    Sol = np.concatenate(([xSol], [ySol], np.ones((1, np.size(xSol, axis=0)))), axis=0)    
    #Sol = np.array([[-0.56648, 0.022008, -2.96859],[0.362136, 0.932125,-559.271],[0,0,1]])*Sol
    Sol = np.matmul(np.array([[-0.064717, -0.000218, 10.2782],[0.000218,-0.064717,34.8483],[0,0,1]]), Sol)
    
    cv.waitKey(0)
    return Sol[0,:], Sol[1,:]
