import math

import cv2 as cv
import numpy as np
import pathlib as pathlib

numbImg = 0
dir=pathlib.Path.cwd().__str__()+"\img" #Путь к папке проекта
filePath = dir + '\\'.__str__() #Путь к image
src = cv.imread(filePath + "0.jpg", cv.IMREAD_COLOR) #Грузим image в формате bmp


def Test(numbImg, adaptiveThresholdParam, adaptiveThresholdSize, iterationsErode, iterationsErodeDialate):

    #Open image
    image=cv.imread(filePath + numbImg.__str__() + ".jpg", cv.IMREAD_COLOR) #Грузим image в формате bmp
    cv.imshow("Input", image)  # Вывод результата
    btn = cv.waitKey(0)
    #for i in range(24):
    imageGray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    cv.imshow("Gray", imageGray)  # Вывод результата
    btn = cv.waitKey(0)
    #imageOut = cv.adaptiveThreshold(imageGray, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY,
    #                                   adaptiveThresholdParam, adaptiveThresholdSize) # 51,10)
    adaptiveThresholdSize = 12
    for i in range(5):
        adaptiveThresholdSize+=3
        imageOut = cv.adaptiveThreshold(imageGray, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY,
                                       adaptiveThresholdParam, adaptiveThresholdSize) # 51,10)
        cv.imshow(adaptiveThresholdSize.__str__(), imageOut)  # Вывод результата
        btn = cv.waitKey(0)

    adaptiveThresholdSize = 12
    adaptiveThresholdParam = 51
    for i in range(50):
        adaptiveThresholdParam=adaptiveThresholdParam+1
        if adaptiveThresholdParam % 2 == 1:
            imageOut = cv.adaptiveThreshold(imageGray, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY,
                                            adaptiveThresholdParam, adaptiveThresholdSize)  # 51,10)
            cv.imshow(adaptiveThresholdParam.__str__(), imageOut)  # Вывод результата
            btn = cv.waitKey(0)
    pass




    kernel = np.ones((3, 3), 'uint8')

    if (iterationsErode>0):
       imageOut = cv.erode(imageOut, kernel, iterationsErode) #Убрать шумы
    if (iterationsErodeDialate>0):
        imageOut = cv.dilate(imageOut, kernel, iterationsErodeDialate) #Убрать шумы

    cv.imshow("ImageOut", imageOut) #Вывод результата

    btn=cv.waitKey(0)



def SearchBlobs(image):
    # Параметры поиска Blob-ов
    params = cv.SimpleBlobDetector_Params()
    params.minThreshold = 0
    params.maxThreshold = 255
    params.thresholdStep = 250  # Без шагов, т.к уже бинаризованных вход
    params.filterByArea = True
    params.minArea = 1
    params.maxArea = 200000
    params.filterByCircularity = False
    params.minCircularity = 0.6  # Увеличение приводит к NoSearch 4 блобов на 11 image
    params.maxCircularity = 1
    params.filterByConvexity = False
    params.minConvexity = 0.85  # Увеличение приводит к NoSearch 4 блобов на 11 image
    params.maxConvexity = 1
    params.filterByInertia = False
    params.filterByColor = True
    params.minDistBetweenBlobs = 1  # Фильтр лишнего

    detector = cv.SimpleBlobDetector_create(params)  # Создадим детектор блобов
    keypoints = detector.detect(image)  # Поиск блобов
    return keypoints
    pass

def Search(image, imageIn):


    #image = cv.bitwise_not(image)
    #cv.imshow("bilateral", image)  # Вывод результата
    #btn = cv.waitKey(0)

    #image = cv.bilateralFilter(image,15,75,75)
    #image = cv.GaussianBlur(image,(9,9),3)
    #cv.imshow("bilateral", image)  # Вывод результата
    #btn = cv.waitKey(0)




    keypoints = SearchBlobs(image)  # Поиск блобов
    imageIn = cv.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255),
                               cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    #cv.imshow("imageIn", imageIn)  # Вывод результата
    #btn = cv.waitKey(0)
    pass

lenthPorog = 20
def angle(vector1):
    vector2 = [1, 0]  # vector of X-axis
    length1 = math.sqrt(vector1[0] * vector1[0] + vector1[1] * vector1[1])
    if length1 > lenthPorog:
        return 0
    length2 = math.sqrt(vector2[0] * vector2[0] + vector2[1] * vector2[1])
    return math.acos((vector1[0] * vector2[0] + vector1[1] * vector2[1]) / (length1 * length2))

def PreStart(numbImg):
    imageIn = cv.imread(filePath + numbImg.__str__() + ".jpg", cv.IMREAD_COLOR)  # Грузим image в формате bmp
    cv.imshow("Input", imageIn)  # Вывод результата
    btn = cv.waitKey(0)
    image = cv.cvtColor(imageIn, cv.COLOR_BGR2GRAY)
    cv.imshow("Gray", image)  # Вывод результата
    btn = cv.waitKey(0)
    #image = cv.bilateralFilter(image,15,75,75)
    image = cv.GaussianBlur(image,(9,9),100)
    cv.imshow("bilateral", image)  # Вывод результата
    btn = cv.waitKey(0)





    imageThreshold = cv.adaptiveThreshold(image, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY,
                                       adaptiveThresholdParam, adaptiveThresholdSize) # 51,21)
    cv.imshow("Adaptive", imageThreshold)  # Вывод результата
    btn = cv.waitKey(0)
    kernel = np.ones((3, 3), 'uint8')
    image = cv.erode(imageThreshold, kernel, iterations=iterationsErode)  # Убрать шумы
    cv.imshow("erode", image)  # Вывод результата
    btn = cv.waitKey(0)
    imageGray = image.copy()

    contours, hierarchy = cv.findContours(imageGray, cv.RETR_LIST , cv.CHAIN_APPROX_SIMPLE)

    number = 0
    pointX = [0]
    pointY = [0]
    pointRadius = [0]

    imageOut1 = imageIn.copy()
    for contor in contours:
        (x, y), radius1 = cv.minEnclosingCircle(contor)
        center = (int(x), int(y))
        radius = int(radius1)
        if radius>8 and radius<30:
           pointX.append(x)
           pointY.append(y)
           cv.circle(imageIn, center, radius, (0, 255, 0), 2)
    btn = cv.waitKey(0)

    #Список ближайших векторов
    #for i in range(pointX.count()):
        #vector = [pointX[i] - pointX[i+1], targetY - gunY]  # Vector of aiming of the gun at the target

    #    pass



        #if contor.size < 300 & contor.size > 100:
        #    imageIn = cv.drawContours(imageIn, contours, number, (0, 255, 0), 2, cv.LINE_AA, hierarchy, 1)
        #number = number + 1
    pass


  #  lines = cv.HoughLinesP(imageGray, 1500, np.pi/180, 90, maxLineGap=10000)
  #  for line in lines:
  #      x1, y1, x2, y2 = line[0]
  #      cv.line(imageIn, (x1, y1), (x2, y2), (0, 0, 128), 1)
  #  cv.imshow("linesEdges", imageGray)
  #  cv.imshow("linesDetected", imageIn)
  #  cv.waitKey(0)

    hueRange = 15 # how much difference from the desired color we want to include to the result If you increase this value,
    #for example a red color would detect some orange values, too.

    minSaturation = 50 #I'm not sure which value is good here...
    minValue = 10 #not sure whether 50 is a good min value here...






    #imageIn = cv.drawContours(imageIn, contours, -1, (0, 255, 0), 2, cv.LINE_AA, hierarchy, 1)
    cv.imshow('contours', imageIn)  # выводим итоговое изображение в окно
    btn = cv.waitKey(0)



    #for i in range(50):
    #   image = cv.GaussianBlur(image,(5,5),100)
    #   cv.imshow("bilateral", image)  # Вывод результата
   #    btn = cv.waitKey(0)

    #kernel = np.ones((9, 9), 'uint8')
    #image = cv.morphologyEx(image, cv.MORPH_ELLIPSE, kernel);
    #cv.imshow("filter2D", image)  # Вывод результата
    #btn = cv.waitKey(0)
    #imageOut = cv.dilate(imageOut, kernel, iterations=1)  # Убрать шумы
    #cv.imshow("dilate", imageOut)  # Вывод результата
    #btn = cv.waitKey(0)
    Search(image, imageIn)
    pass

#Парам
adaptiveThresholdParam = 51
adaptiveThresholdSize = 19
iterationsErode = 3
iterationsDilate = 0

PreStart(numbImg)
#Test(numbImg, adaptiveThresholdParam, adaptiveThresholdSize,iterationsErode,iterationsErodeDialate)