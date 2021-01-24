import math
import cv2 as cv
import numpy as np
import pathlib as pathlib


def Calc(imageRead):
    edges = cv.Canny(imageRead, 57, 76) #57 76 VS 109, 255
    cv.imshow(my_window, edges)  # Вывод результата
    cv.waitKey()

    #Инверсия
    imagem = abs(255 - edges)
    cv.imshow(my_window, imagem)  # Вывод результата
    cv.waitKey()







    pass



def KennyDetect(imageRead):
    edges = cv.Canny(imageRead, lowerthreshold, upperthreshold)

    cv.imshow(my_window, edges)  # Вывод результата

lowerthreshold = 0
upperthreshold = 0
numbImg = 0

my_window = 'My Window'
cv.namedWindow(my_window)

def TrackbarCall1(val):
    global lowerthreshold
    lowerthreshold = val
    KennyDetect(imageRead)
def TrackbarCall2(val):
    global upperthreshold
    upperthreshold = val
    KennyDetect(imageRead)
def TrackbarCall3(val):
    global numbImg, imageRead
    numbImg = val
    imageRead = cv.imread(filePath + numbImg.__str__() + ".jpg", cv.IMREAD_COLOR)
    KennyDetect(imageRead)
    Calc(imageRead)

cv.createTrackbar('lowerthreshold', my_window , 1, 255, TrackbarCall1)
cv.createTrackbar('upperthreshold', my_window , 1, 255, TrackbarCall2)
cv.createTrackbar('numberImage', my_window , 0, 4, TrackbarCall3)

dir=pathlib.Path.cwd().__str__()+"\img" #Путь к папке проекта
filePath = dir + '\\'.__str__() #Путь к image
imageRead = cv.imread(filePath + numbImg.__str__() + ".jpg", cv.IMREAD_COLOR) #Грузим image в формате bmp
cv.imshow("imageRead", imageRead)  # Вывод результата

cv.waitKey()