import cv2 as cv
import numpy as np
import pathlib as pathlib

# Параметры
title_window = 'My Window' #Окно
numbImg = 3 #Номер Image
alpha_slider_max = 100 #Макс значение для слайдера
adaptiveThresholdParam = 51 #Порог вычитаемый из среднего или взвешенного значения в зоне
adaptiveThresholdSize = 23 #Размер окрестности
iterationsErode = 1 #Эрозия
iterationsDialate = 0 #Диалатация
minPointY = 2  #Минимально Кол-во точек на линии по оси Y (Для начала контроля строки)
porogY = 1  # Допустимое отклонение от центра линии по Y
# параметры прямоугольников
minWidth = 25
maxWidth = 40
minHeigh = 65
maxHeigh = 65
averWidth = minWidth + (maxWidth - minWidth) * 0.5 #Стандартная ширина
averHeigh = minHeigh + (maxHeigh - minHeigh) * 0.5 #Стандартная высота
dir = pathlib.Path.cwd().__str__() + "\img"  #Путь к папке проекта
filePath = dir + '\\'.__str__()  #Путь к image

src1 = cv.imread(filePath + "0.jpg", cv.IMREAD_COLOR) #Грузим img
cv.namedWindow(title_window) #Создали окно

#Методы от трекбаров
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

#Трекбары
cv.createTrackbar('imageNumb', title_window, 0, 4, Val3)
cv.createTrackbar('adaptiveThresholdParam', title_window , 51, 100, Val1)
cv.createTrackbar('adaptiveThresholdKernel', title_window , 23, 100, Val2)
cv.createTrackbar('iterationsErode', title_window , 1, 10, Val4)
cv.createTrackbar('iterationsDialate', title_window , 0, 10, Val5)

#Точка
class Point(object):
    x = 0 #Координата x
    y = 0 #Координата y
    radius = 0 #Радиус

    #Конструктор
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

#прямоугольник для символа
class RectAngle(object):
    #Конструктор
    def __init__(self, x, y, x1, y1):
        self.x = x
        self.x1 = x1
        self.y = y
        self.y1 = y1
        self.pt1 = (int(x), int(y)) #Точка (левая верхняя)
        self.pt2 = (int(x1), int(y1)) #Точка (правая нижняя)

    #Значения
    x = int
    x1 = int
    y = int
    y1 = int
    pt1 = (int, int)
    pt2 = (int, int)

    #Метод обновления точек
    def CalculatePt(self):
        self.pt1 = (int(self.x), int(self.y))
        self.pt2 = (int(self.x1), int(self.y1))

#Поиск символов на линии и разметка прямоугольников
def SearchSymbolsInLine(imageMorfology, points, startLineY, endLineY):  # Поиск символов на линии
    pointsOut = [] #Точки

    # Создаем гистограмму по оси X
    gistogramX = []  # Гистограмма
    for i in range(imageMorfology.shape[1]):  #Проход по оси X
        gistogramX.append(0)  #Добавим значение в гистограмму
        pass

    # Заполняем гистограмму по оси X
    for point in points:
        if (int(point.y) > startLineY and int(point.y) < endLineY): #Собираем точки удовлетворяющие нашей строке по Y
            gistogramX[int(point.x)] = gistogramX[int(point.x)] + 1 #Добавляем на линию точку
            pointsOut.append(point)  #Собираем точки на линии

    # Вырвним центра по Y
    sumY = 0.0 #Сумма по Y
    numb = len(pointsOut) #Кол-во точек
    for point in pointsOut: #Считаем сумму по Y
        sumY = sumY + point.y  # Сумма по Y
        pass
    averY = int(sumY / numb)  # Средняя точка линии по оси Y
    for point in pointsOut:  # Смещаем точки к центру строки
        if (point.y > averY + porogY):
            point.y = int(averY + porogY)
            continue
        if (point.y < averY - porogY):
            point.y = int(averY - porogY)
            continue

    # Чистим точки
    pointsOut.sort(key=lambda Point: Point.x) #Сортируем по X
    pointsOut1 = [] #Еще массив точек (отфильтрованных) Задача выполнить т.з. на оптимизацию можно выделить время позже
    nextbit = False #Пропуск следующей точки
    for i in range(len(pointsOut) - 1): #Проходимся по точкам
        if (nextbit): #Если команда на пропуск висит
            nextbit = False #Сбросим команду
            continue #Пропускаем точку
        dist = pointsOut[i + 1].x - pointsOut[i].x #Расстояние между краями прямоугольников, рядом стоящих
        if (dist < minWidth): #Если прямоугольники находятся слишком близко
            nextbit = True #Пропустить след прямоугольник
            pass
        #Добавление существующей точки/Добавление новой точки(которая между двумя близко расположенными)
        if (dist < minWidth): #Если прямоугольники находятся слишком близко
            newX = int((pointsOut[i + 1].x + pointsOut[i].x) * 0.5) #Берем середину между ними по X
            newY = int((pointsOut[i + 1].y + pointsOut[i].y) * 0.5) #Берем середину между ними по Y
            newRadius = int((pointsOut[i + 1].radius + pointsOut[i].radius)) #Радиус уже не нужен (TODO Оптимизировать)
            newPoint = Point(newX, newY, newRadius) #Создадим новую точку
            pointsOut1.append(newPoint) #Добавим новую точку
        else: #Если прямоугольники не слишком близка
            pointsOut1.append(pointsOut[i])  #Добавляем
            pass
    pointsOut1.append(pointsOut[len(pointsOut) - 1])  #Добавляем последний прямоугольник (т.к. range(len(pointsOut)-1))

    # Собираем прямоугольник
    reactAngles = [] #Еще массив. Задача выполнить т.з. на оптимизацию можно выделить время позже
    for point in pointsOut1: #Проходимся по отфильтрованным точкам
        xStart = point.x - (averWidth * 0.5) #Задаем параметры прямоугольника
        yStart = point.y - (averHeigh * 0.5) #Задаем параметры прямоугольника
        xEnd = point.x + (averWidth * 0.5) #Задаем параметры прямоугольника
        yEnd = point.y + (averHeigh * 0.5) #Задаем параметры прямоугольника
        pt1 = (int(xStart), int(yStart)) #Задаем параметры прямоугольника
        pt2 = (int(xEnd), int(yEnd)) #Задаем параметры прямоугольника
        reactAngles.append(RectAngle(xStart, yStart, xEnd, yEnd)) #Добавляем в список прямоугольник

    # Сдвигаем границы по оси X
    for i in range(len(reactAngles) - 1): #проходимся по прямоугольникам
        dist = reactAngles[i + 1].x - reactAngles[i].x1 #Расстояние между вертикальными краями соседних прямоугольников
        value = (dist * 0.5) #Значение на которое необходимо сдвинуть края (Правый и Левый у следующего прямоугольника)
        reactAngles[i].x1 = reactAngles[i].x1 + value #Сдвинем края
        reactAngles[i + 1].x = reactAngles[i + 1].x - value #Сдвинем края
        reactAngles[i].CalculatePt() #Пересчитаем точки прямоугольника

    endRectAngle = len(reactAngles) - 1 #Конечный прямоугольник
    reactAngles[endRectAngle].CalculatePt() #Пересчитаем точки

    return reactAngles #Вернем прямоугольники
    pass

#Предобработка, извлечение строк с символами, отрисовка
def Start():
    imageIn = cv.imread(filePath + numbImg.__str__() + ".jpg", cv.IMREAD_COLOR) #Грузим img
    #cv.imshow(title_window, imageIn)
    #btn = cv.waitKey(0)

    imageGray = cv.cvtColor(imageIn, cv.COLOR_BGR2GRAY) #Перевод Gray (интенсивность яркости)
    #cv.imshow("Gray", imageGray)
    #btn = cv.waitKey(0)

    imageThreshold = cv.adaptiveThreshold(imageGray, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY,
                                          adaptiveThresholdParam, adaptiveThresholdSize) #Адаптивная бинаризация
    #cv.imshow(title_window, imageThreshold)
    #btn = cv.waitKey(0)

    imageMorfology = imageThreshold.copy() #Копия для передачи в метод
    kernel = np.ones((5, 5), 'uint8') #Ядро под морфологию
    if (iterationsErode > 0): #Эрозия
        imageMorfology = cv.erode(imageMorfology, kernel, iterationsErode)
        pass
    if (iterationsDialate > 0): #Диалатация
        imageMorfology = cv.dilate(imageMorfology, kernel, iterationsDialate)
        pass
    #cv.imshow("imageMorfology", imageMorfology)
    #btn=cv.waitKey(0)

    contours, hierarchy = cv.findContours(imageMorfology, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE) #Ищем контуры

    #фильтруем контуры по размерам и преобразуем в списки точек
    pointX = [0] #Список точек по X
    pointY = [0] #Список точек по Y
    points = [Point] #Список точек
    imageCircle = imageIn.copy() #Копия под результат
    for contor in contours: #Пробежка по контурам
        (x, y), radiusIn = cv.minEnclosingCircle(contor) #Получаем взвешенный центр, радиус контура
        radius = int(radiusIn) #Радиус к инту
        if radius > 9 and radius < 33: #Фильтруем слишком большие и маленькие контуры
            pointNew = Point(x, y, radius) #Создадим точку
            points.append(pointNew) #Добавим точку в список
            pointX.append(x) #Добавим в список X
            pointY.append(y) #Добавим в список Y
            pass

    # Заполним гистограммы по оси Y
    lenthY = imageCircle.shape[0]  #Высота гистограммы
    histogramY = [] #Гистограмма
    histogramC = [] #Гистограмма
    #Создадим гистограммы
    for i in range(lenthY):
        histogramY.append(0) #Добавим лист в гистограмму
        histogramC.append(i) #Добавим лист в гистограмму
        pass

    #Заполним гистограммы
    for point in pointY:
        histogramY[int(point)] = histogramY[int(point)] + 1 #Считаем кол-во точек, находящихся на оси Y
        pass

    #Анализ гистограммы
    startLineY = 0 #Начальная позиция линии
    countSmallBar = 0
    countSmallBarMax = 5
    for i in range(lenthY): #Проходимся по оси Y сверху вниз
        if (histogramY[i] > minPointY and startLineY == 0): #Если кол-во точек больше minPointY
            startLineY = i - 10 #Начальная позиция линии c откатом наверх
            if (startLineY < 0):
                startLineY = 0
                pass
        if (histogramY[i] < minPointY and startLineY > 0):#Если кол-во точек меньше minPointY и уже задана нач. поз.
            if (countSmallBar < countSmallBarMax):
                countSmallBar = countSmallBar + 1
                continue
            endLineY = i + 10 #Конечная позиция линии c добором вниз
            rectAngles = SearchSymbolsInLine(imageMorfology, points, startLineY, endLineY) #Поиск символов на линии
            for rectAngle in rectAngles: #Отрисовка прямоугольников
                cv.rectangle(imageCircle, rectAngle.pt1, rectAngle.pt2, (0, 0, 255), 1)
            startLineY = 0 #Обнуление
            endLineY = 0 #Обнуление
        countSmallBar = 0 #Обнулим

    cv.imshow(title_window, imageCircle) #Вывод результата

# Поиск Blobs
def SearchBlobs(imageIn):
    # Параметры поиска Blob-ов
    params = cv.SimpleBlobDetector_Params()
    params.minThreshold = 0
    params.maxThreshold = 255
    params.thresholdStep = 250 #Без шагов, т.к уже бинаризованных вход
    params.filterByArea = True
    params.minArea = 1
    params.maxArea = 20000
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = False
    params.filterByColor = False
    params.minDistBetweenBlobs = 1 #Фильтр лишнего

    detector = cv.SimpleBlobDetector_create(params) #Создадим детектор блобов
    keypoints = detector.detect(imageIn) #Поиск блобов
    return keypoints #Вернем найденные точки
    pass

Start() #Запускаем приложение
cv.waitKey()

# Тест отрисовка
#    pointsOut.sort(key=lambda Point: Point.x) #Сортируем по X
#    for point in pointsOut:
#        cv.rectangle(imageMorfology,
#                     (int(point.x - averWidth * 0.5), int(point.y - averHeigh * 0.5)),
#                     (int(point.x + averWidth * 0.5), int(point.y + averHeigh * 0.5)),
#                     (0, 0, 255), 1)
#    cv.imshow("Test", imageMorfology)
#    print(len(pointsOut))
#    btn = cv.waitKey(0)