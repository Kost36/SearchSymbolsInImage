#import cv2 as cv
import numpy as np
import pathlib as pathlib
#from math import *
#from __future__ import print_function
#from __future__ import division
import cv2 as cv
import argparse
















'''
#Переменные
dir=pathlib.Path.cwd().__str__()+"\img" #Путь к папке проекта
thresholdSizeMul=2.5 #Порог отклонения размера от среднего размера
thresholdDistToCenterMul=0.2 #Порог отклонения центра блоба от центров по осям X и Y

#Расстояние между двумя точками
def Distance(x1, y1, x2, y2):
    c = sqrt((x2-x1)**2 + (y2-y1)**2)
    return c

#Проверка позиций блобов
def CheckPozitionBlobs(keypoints, xCentr, yCentr):
    sumDistantionOfCenter=0 #Сумма растояний до центра image
    sumSize=0 #Сумма размеров blob ов

    #Считаем суммы
    for keypoint in keypoints:
        sumDistantionOfCenter+=Distance(keypoint.pt[0], keypoint.pt[1], xCentr, yCentr) #Сумма растояния до центра
        sumSize+=keypoint.size #Сумма размеров блобов

    #Среднее
    averDistantionOfCenter=sumDistantionOfCenter/4.0  #Среднее растояний до центра image
    averSize=sumSize/4.0 #Средний размер блобов

    zonesOk = np.array([False, False, False, False]) #Массив решений по зонам
    #Распределение блобов по зонам
    for keypoint in keypoints:
        if ((keypoint.pt[0]<xCentr) & (keypoint.pt[1]<yCentr)): #Если blob в зоне
            zonesOk[0]=True #В зоне есть объект
        if ((keypoint.pt[0]>=xCentr) & (keypoint.pt[1]<yCentr)): #Если blob в зоне
            zonesOk[1]=True #В зоне есть объект
        if ((keypoint.pt[0]<xCentr) & (keypoint.pt[1]>=yCentr)): #Если blob в зоне
            zonesOk[2]=True #В зоне есть объект
        if ((keypoint.pt[0]>=xCentr) & (keypoint.pt[1]<yCentr)): #Если в blob зоне
            zonesOk[3]=True #В зоне есть объект

    #Проверка заполнения всех зон блобами
    for boolResult in zonesOk:
        if (boolResult): #Если в зоне есть блоб
            pass
        else: #Если в зоне нету блоба
            return False

    #Проверка приблизительно равного размера блобов
    for keypoint in keypoints:
        if ((keypoint.size>averSize+averSize*thresholdSizeMul) | (keypoint.size<averSize-averSize*thresholdSizeMul)):
            return False #Если размер блоба выходит за порог, то вернем нет

    #Пороги для фильтра по расстоянию блоба до центра Image
    thresholdDistToCenterMin=averDistantionOfCenter-averDistantionOfCenter*thresholdDistToCenterMul
    thresholdDistToCenterMax=averDistantionOfCenter+averDistantionOfCenter*thresholdDistToCenterMul

    #Фильтр по расстоянию от центра Image до центров blobs
    for keypoint in keypoints:
        distance=Distance(keypoint.pt[0], keypoint.pt[1], xCentr, yCentr) #Рассчет дистанции до центра
        if ((distance<thresholdDistToCenterMin) | (distance>thresholdDistToCenterMax)):
            return False #Если расстоянию от центра Image до центров blob выходит за порог, то вернем нет

    return True
    pass

#Проверка блобов
def CheckBlobs(keypoints, xCentr, yCentr):
    if (len(keypoints)!=4): # Должно быть 4 шт.
        return False
    if (CheckPozitionBlobs(keypoints, xCentr, yCentr) == False): # Проверка позиций блобов.
        return False
    return True
    pass

#Поиск Blobs
def SearchBlobs(image):
    # Параметры поиска Blob-ов
    params = cv.SimpleBlobDetector_Params()
    params.minThreshold = 0
    params.maxThreshold = 255
    params.thresholdStep = 250  # Без шагов, т.к уже бинаризованных вход
    params.filterByArea = True
    params.minArea = 1
    params.maxArea = 20000
    params.filterByCircularity = True
    params.minCircularity = 0.6  # Увеличение приводит к NoSearch 4 блобов на 11 image
    params.maxCircularity = 1
    params.filterByConvexity = True
    params.minConvexity = 0.85  # Увеличение приводит к NoSearch 4 блобов на 11 image
    params.maxConvexity = 1
    params.filterByInertia = False
    params.filterByColor = False
    params.minDistBetweenBlobs = 30  # Фильтр лишнего

    detector = cv.SimpleBlobDetector_create(params)  # Создадим детектор блобов
    keypoints = detector.detect(image)  # Поиск блобов
    return keypoints
    pass

#Обработка изображения
def ProcessingImage(imageInput, numberImage):
    # Получаем центра по X и Y
    xCentr = imageInput.shape[1]/2.0 #Центр по оси X
    yCentr = imageInput.shape[0]/2.0 #Центр по оси Y

    # Бинаризация порог = 175
    # Уменьшение приводит к слиянию интересующих блобов
    # Увеличение уменьшает размер слабого блоба на 11 image
    ret, imageBinary = cv.threshold(imageInput, 175, 255, cv.THRESH_BINARY)

    # Уберем возможные шумы
    kernel = np.ones((5, 5), 'uint8')
    imageErode = cv.erode(imageBinary, kernel, iterations=1) #Убрать шумы
    imageDilate = cv.dilate(imageErode, kernel, iterations=3) #Вернем,и увеличим размер

    keypoints = SearchBlobs(imageDilate) #Поиск блобов

    # Нанесение интересующих областей(зоны)
    imageOutput = cv.rectangle(imageInput,
                               (0, 0), (int(xCentr), int(yCentr)), (0, 255, 255), 2)
    imageOutput = cv.rectangle(imageOutput,
                               (int(xCentr), 0), (int(xCentr) * 2, int(yCentr)), (0, 255, 255), 2)
    imageOutput = cv.rectangle(imageOutput,
                               (0, int(yCentr)), (int(xCentr), int(yCentr) * 2), (0, 255, 255), 2)
    imageOutput = cv.rectangle(imageOutput,
                               (int(xCentr), int(yCentr)), (int(xCentr) * 2, int(yCentr) * 2), (0, 255, 255), 2)

    # Нанесение найденных объектов
    imageWithKeypoints = cv.drawKeypoints(imageOutput, keypoints, np.array([]), (0, 0, 255),
                                          cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Нанесение результата обработки image
    if (CheckBlobs(keypoints, xCentr, yCentr)): #Если проверка пройдена
        imageWithKeypoints = cv.putText(imageWithKeypoints, 'Yes', #Image, текст
                                        (10, imageInput.shape[0]-10), #Позиция (X,Y начала текста)
                                        cv.FONT_HERSHEY_SIMPLEX, 2, #Тип фона, множитель размера
                                        (0, 255, 0), 5) #Цвет фона, толщина линий
        print("Image " + numberImage.__str__() + " Yes")
    else: #Если проверка не пройдена
        imageWithKeypoints = cv.putText(imageWithKeypoints, 'No', #Image, текст
                                        (10, imageInput.shape[0]-10), #Позиция (X,Y начала текста)
                                        cv.FONT_HERSHEY_SIMPLEX, 2, #Тип фона, множитель размера
                                        (0, 0, 255), 5) #Цвет фона, толщина линий
        print("Image " + numberImage.__str__() + " No")

    return imageWithKeypoints
    pass
'''
#Тест
def Test():
    #Open image
    image=cv.imread(filePath + numbImg.__str__() + ".jpg", cv.IMREAD_COLOR) #Грузим image в формате bmp
    #for i in range(24):
    imageGray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    imageOut = cv.adaptiveThreshold(imageGray, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY,
                                       adaptiveThresholdParam, adaptiveThresholdSize) # 51,10)

    kernel = np.ones((3, 3), 'uint8')
    if (iterationsErode>0):
       imageOut = cv.erode(imageOut, kernel, iterationsErode) #Убрать шумы
    if (iterationsDialate>0):
        imageOut = cv.dilate(imageOut, kernel, iterationsDialate) #Убрать шумы

    cv.imshow(title_window, imageOut) #Вывод результата

    #btn=cv.waitKey(0)



'''
    #ret, imageBinary = cv.threshold(image, i*10, 255, cv.THRESH_BINARY)


#Догика программы
#for i in range(5): # Пробегаемся по 14 цифрам -> номера images
    #filePath = dir + '\\'.__str__() + i.__str__() #Путь к image
    #image=cv.imread(filePath + ".jpg", cv.IMREAD_COLOR) #Грузим image в формате bmp
    #outputImage=Test(image) #Обработка image
    #cv.imshow("Image " + i.__str__() + ". For next click exit", outputImage) #Вывод результата
    #cv.imwrite(filePath + "_Result.png", outputImage)
    #btn = input()
    #btn=cv.waitKey(0)



# window
cv.namedWindow('Fenetre', cv.WINDOW_GUI_NORMAL)

# Trackbar
Slider = cv.createTrackbar('Fenetre', 'adaptiveThresholdParam', 0 , 255 , Test)
Slider = cv.createTrackbar('Fenetre', 'adaptiveThresholdKernel', 0 , 255 , Test)
Slider = cv.createTrackbar('Fenetre', 'imageNumb', 0 , 255 , Test)

# Initialise first view as the normal image
filePath = dir + '\\'.__str__() #Путь к image
image=cv.imread(filePath + "0.jpg", cv.IMREAD_COLOR) #Грузим image в формате bmp

while(1):
    cv.imshow('Fenetre', image)
    k = cv.waitKey(1) & 0xFF
    if k == 27:
        break


cv.destroyAllWindows()
'''

alpha_slider_max = 100
title_window = 'My Window'
numbImg=0
adaptiveThresholdParam=51
adaptiveThresholdSize=11
iterationsErode=0
iterationsDialate=0

#parser = argparse.ArgumentParser(description='Code for Adding a Trackbar to our applications tutorial.')
#parser.add_argument('--input1', help='Path to the first input image.', default='LinuxLogo.jpg')
#parser.add_argument('--input2', help='Path to the second input image.', default='WindowsLogo.jpg')
#args = parser.parse_args()
dir=pathlib.Path.cwd().__str__()+"\img" #Путь к папке проекта
filePath = dir + '\\'.__str__() #Путь к image
src1 = cv.imread(filePath + "0.jpg", cv.IMREAD_COLOR) #Грузим image в формате bmp
#src2 = cv.imread(cv.samples.findFile(args.input2))
#if src1 is None:
#    print('Could not open or find the image: ', args.input1)
#    exit(0)
#if src2 is None:
#    print('Could not open or find the image: ', args.input2)
#    exit(0)

cv.namedWindow(title_window)

def Val1(val):
    global adaptiveThresholdParam
    adaptiveThresholdParam = val
    Test()
def Val2(val):
    global adaptiveThresholdSize
    adaptiveThresholdSize = val
    Test()
def Val3(val):
    global numbImg
    numbImg = val
    Test()
def Val4(val):
    global iterationsErode
    iterationsErode = val
    Test()
def Val5(val):
    global iterationsDialate
    iterationsDialate = val
    Test()

cv.createTrackbar('imageNumb', title_window, 0, 4, Val3)
cv.createTrackbar('adaptiveThresholdParam', title_window , 51, 51, Val1)
cv.createTrackbar('adaptiveThresholdKernel', title_window , 11, 11, Val2)
cv.createTrackbar('iterationsErode', title_window , 0, 10, Val4)
cv.createTrackbar('iterationsDialate', title_window , 0, 10, Val5)
Test()

# Show some stuff
#on_trackbar(0)
# Wait until user press some key
cv.waitKey()