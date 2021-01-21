import math
import cv2 as cv
import numpy as np
import pathlib as pathlib

#Параметры
alpha_slider_max = 100
title_window = 'My Window'
numbImg=0
adaptiveThresholdParam=97 #27
adaptiveThresholdSize=24 #9
iterationsErode=1 #1
iterationsDialate=0 #1

dir=pathlib.Path.cwd().__str__()+"\img" #Путь к папке проекта
filePath = dir + '\\'.__str__() #Путь к image
src1 = cv.imread(filePath + "0.jpg", cv.IMREAD_COLOR)

cv.namedWindow(title_window)

def Val1(val):
    global adaptiveThresholdParam
    adaptiveThresholdParam = val
    Start()
def Val2(val):
    global adaptiveThresholdSize
    adaptiveThresholdSize = val
    Start()
def Val3(val):
    global numbImg
    numbImg = val
    Start()
def Val4(val):
    global iterationsErode
    iterationsErode = val
    Start()
def Val5(val):
    global iterationsDialate
    iterationsDialate = val
    Start()

cv.createTrackbar('imageNumb', title_window, 0, 4, Val3)
cv.createTrackbar('adaptiveThresholdParam', title_window , 97, 100, Val1) #51
cv.createTrackbar('adaptiveThresholdKernel', title_window , 24, 100, Val2) #11
cv.createTrackbar('iterationsErode', title_window , 1, 10, Val4)
cv.createTrackbar('iterationsDialate', title_window , 0, 10, Val5)

def Start():
    #Open image
    imageIn=cv.imread(filePath + numbImg.__str__() + ".jpg", cv.IMREAD_COLOR)
    #cv.imshow(title_window, image)  # Вывод результата
    #btn = cv.waitKey(0)


    imageGray = cv.cvtColor(imageIn, cv.COLOR_BGR2GRAY)
    #cv.imshow("Gray", imageGray)
    #btn = cv.waitKey(0)


    imageThreshold = cv.adaptiveThreshold(imageGray, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY,
                                    adaptiveThresholdParam, adaptiveThresholdSize)
    #cv.imshow(title_window, imageThreshold)
    #btn = cv.waitKey(0)


    imageMorfology = imageThreshold.copy()
    kernel = np.ones((5, 5), 'uint8')
    if (iterationsErode>0):
       imageMorfology = cv.erode(imageMorfology, kernel, iterationsErode)
    if (iterationsDialate>0):
       imageMorfology = cv.dilate(imageMorfology, kernel, iterationsDialate)
    cv.imshow("imageMorfology", imageMorfology) #Вывод результата
    #btn=cv.waitKey(0)


    contours, hierarchy = cv.findContours(imageMorfology, cv.RETR_LIST , cv.CHAIN_APPROX_SIMPLE)
    pointX = [0]
    pointY = [0]
    imageCircle = imageIn.copy()
    for contor in contours:
        (x, y), radiusIn = cv.minEnclosingCircle(contor)
        center = (int(x), int(y))
        radius = int(radiusIn)
        if radius > 8 and radius < 33:
            pointX.append(x)
            pointY.append(y)
            cv.circle(imageCircle, center, radius, (0, 0, 255), 1)
    cv.imshow(title_window, imageCircle)  # Вывод результата
    #btn = cv.waitKey(0)


    lenthY = imageCircle.shape[0]
    gistogramPorogY = 2 #px
    gistogramY = []
    for point in pointY:
        for i in range(lenthY):
            if (i+gistogramPorogY>point & i-gistogramPorogY<point):
                gistogramY[i].append(1)
                pass







#Поиск Blobs
def SearchBlobs(imageIn):
    # Параметры поиска Blob-ов
    params = cv.SimpleBlobDetector_Params()
    params.minThreshold = 0
    params.maxThreshold = 255
    params.thresholdStep = 250  # Без шагов, т.к уже бинаризованных вход
    params.filterByArea = True
    params.minArea = 1
    params.maxArea = 20000
    params.filterByCircularity = False
    params.minCircularity = 0.6  # Увеличение приводит к NoSearch 4 блобов на 11 image
    params.maxCircularity = 1
    params.filterByConvexity = False
    params.minConvexity = 0.85  # Увеличение приводит к NoSearch 4 блобов на 11 image
    params.maxConvexity = 1
    params.filterByInertia = False
    params.filterByColor = False
    params.minDistBetweenBlobs = 1  # Фильтр лишнего

    detector = cv.SimpleBlobDetector_create(params)  # Создадим детектор блобов
    keypoints = detector.detect(imageIn)  # Поиск блобов
    return keypoints
    pass


Start()
cv.waitKey()