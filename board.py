#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket #библиотека для связи между пультом и роботом
import os 
import pickle as pl
import sys 
import time
import threading
import crc16
import psutil

import cv2
import numpy as np

import transmit_thread
import feedback_thread

sys.path.append('/home/pi/RPicam-Streamer/')
import rpicam
import cv_stream

sys.path.append('/home/pi/RPiPWM')
import RPiPWM

#IP = "127.0.0.1"
IP = str(os.popen("hostname -I | cut -d\" \" -f1").readline().replace("\n",""))
PORT = 8000 #порт для управления роботом
REPLY_PORT = 9000
USER_IP = '169.254.166.251' #IP пульта (IP приемника видео)
RTP_PORT = 5000 #порт отправки RTP видео
TIMEOUT = 120 #время ожидания приема сообщения

FORMAT = rpicam.VIDEO_MJPEG #поток MJPEG
WIDTH, HEIGHT = 480, 320
RESOLUTION = (WIDTH, HEIGHT)
FRAMERATE = 30

LINE_SENSITIVITY = 1

arm1_chan = 1
arm2_chan = 2
griper_chan = 3
griper_rot_chan = 4
cam_tilt_chan = 5

left_motor_chan = 12
right_motor_chan = 13

right_motor_freq = 1460
left_motor_freq = 1460

class FrameHandlerThread(threading.Thread):
    def __init__(self, camStream, rtpStream, width, height):
        super(FrameHandlerThread, self).__init__()
        self.daemon = True
        self.rpiCamStream = camStream #получение видео с RPi камеры
        self.rtpStream = rtpStream #отправка RTP потока
        self._frame = None
        self._frameCount = 0
        self._stopped = threading.Event() #событие для остановки потока
        self._newFrameEvent = threading.Event() #событие для контроля поступления кадров
        self.state = None
        self.debug = False
        self.sensitivity = LINE_SENSITIVITY #чувствительность алгоритма определения линии (0..255)
        self.width = width
        if width > WIDTH:
            self.width = WIDTH
        self.height = height
        if height > HEIGHT:
            self.height = HEIGHT
        self._top = HEIGHT - height #верхняя точка среза
        self._left = (WIDTH - width)//2 #левая точка среза
        self._right = self._left + width #правая точка среза
        self._bottom = HEIGHT #нижняя точка среза
        
    def run(self):
        print('Frame handler started')
        while not self._stopped.is_set():
            if self.rpiCamStream.frameRequest(): #отправил запрос на новый кадр
                #print("waiting for frame")
                self._newFrameEvent.wait() #ждем появления нового кадра
                if self._frame: #если кадр есть
                    #print("new frame")
                    self._frame = np.ndarray((self.height, self.width, 3), buffer = self._frame, dtype = np.uint8)
                    self._frame = cv2.cvtColor(self._frame, cv2.COLOR_BGR2RGB)
                    
                    gray = cv2.cvtColor(self._frame, cv2.COLOR_RGB2GRAY)
                    blur = cv2.GaussianBlur(gray, (7, 7), 0)
                    ret, thresh = cv2.threshold(blur, 70, 255, cv2.THRESH_BINARY_INV)
                    contours = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
                    mainContour = max(contours, key = cv2.contourArea)    # берем максимальный
                    M = cv2.moments(mainContour)  # берем его
                    if M['m00'] != 0:   # если нет деления на ноль
                        cx = int(M['m10']/M['m00'])     # смотрим координаты центра наибольшего черного пятна
                        cy = int(M['m01']/M['m00'])     # они получаются в пикселях кадра
                        #self._frame = cv2.cvtColor(thresh, cv2.COLOR_GRAY2RGB)
                        cv2.line(self._frame, (cx, 0), (cx, self.height), (255, 0, 0), 1)    # рисуем перекрестье на контуре
                        cv2.line(self._frame, (0, cy), (self.width, cy), (255, 0, 0), 1)
                        cv2.line(self._frame, (self.width//2, 0), (self.width//2, self.height), (0, 0, 255), 3)
                        #cv2.circle(self,_frame, (self.width//2, self.height//2), 3, (0, 0, 255), -1) #отрисовываем центральную точку
                        cv2.drawContours(self._frame, mainContour, -1, (0, 255, 0), 5, cv2.FILLED) #отображаем контуры на изображении
                    
                    
                    if self.state == 'STATE_LINE':
                        # берем нижнюю часть кадра
                        crop = self._frame[self._top:self._bottom, self._left:self._right]    #обрезаем кадр

                         #вызываем функцию определения линии
                        lineFound, direction, resImg = self.detectLine(crop)

                        leftSpeed = 0
                        rightSpeed = 0
                        if lineFound: # если линия была обнаружена, задем скорости
                            leftSpeed = round(-BASE_SPEED + direction*60)
                            rightSpeed = round(BASE_SPEED + direction*60)

                        setSpeed(leftSpeed, rightSpeed)     #задаем скорости на роботе    
                        print('%.2f\t%d\t%d' % (direction, -leftSpeed, rightSpeed))
                    
                        
                        self.rtpStream.sendFrame(resImg) #помещаем кадр в поток
                        self._frameCount += 1
                        #--------------------------------------
                    else:
                        self.rtpStream.sendFrame(self._frame)
                        #self.rtpStream.sendFrame(cv2.cvtColor(thresh, cv2.COLOR_GRAY2RGB)) #помещаем кадр в поток
                        self._frameCount += 1
                #print(self._frameCount)
                self._newFrameEvent.clear() #сбрасываем событие
        print('Frame handler stopped')
        
    def detectLine(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  #преобразуем в градации серого
        blur = cv2.GaussianBlur(gray, (5, 5), 0)   # размываем изображение blur
        ret, thresh = cv2.threshold(blur, self.sensitivity, 255, cv2.THRESH_BINARY_INV) #бинаризация в ч/б (исходное изобр, порог, максимальное знач., тип бинаризации)
        #_, contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE) #нахождение контуров
        _, contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        direction = 0.0 #курс нашей посудины
        cx = self.width//2 #на случай если не удастся вычислить cx
        cy = self.height
        lineFound = False
        if len(contours) > 0:   # если есть хоть один контур
            lineFound = True
            mainContour = max(contours, key = cv2.contourArea)    # берем максимальный
            M = cv2.moments(mainContour)  # берем его
            if M['m00'] != 0:   # если нет деления на ноль
                cx = int(M['m10']/M['m00'])     # смотрим координаты центра наибольшего черного пятна
                cy = int(M['m01']/M['m00'])     # они получаются в пикселях кадра

                if self.debug:
                    cv2.line(frame, (cx, 0), (cx, self.height), (255, 0, 0), 1)    # рисуем перекрестье на контуре
                    cv2.line(frame, (0, cy), (self.width, cy), (255, 0, 0), 1)
                    
            if self.debug:
                cv2.circle(frame, (self.width//2, self.height//2), 3, (0, 0, 255), -1) #отрисовываем центральную точку
                cv2.drawContours(frame, mainContour, -1, (0, 255, 0), 2, cv2.FILLED) #отображаем контуры на изображении

        direction = cx / (self.width/2) - 1  # преобразуем координаты от 0 до ширина кадра -> от -1 до 1

        gain = cy / self.height + 1 #усиление от 1 до 2
        return lineFound, direction*gain, frame

    def stop(self): #остановка потока
        self._stopped.set()
        if not self._newFrameEvent.is_set(): #если кадр не обрабатывается
            self._frame = None
            self._newFrameEvent.set()
        self.join() #ждем завершения работы потока

    def setFrame(self, frame): #задание нового кадра для обработки
        if not self._newFrameEvent.is_set(): #если обработчик готов принять новый кадр
            self._frame = frame
            self._newFrameEvent.set() #задали событие
            return True
        return False

def onFrameCallback(frame): #обработчик события 'получен кадр'
    #print('New frame')
    frameHandlerThread.setFrame(frame) #задали новый кадр

def get_inf():
    return str(os.popen("vcgencmd measure_temp").readline())

def motor_run(motorA, motorB):
    if motorA == 0:
        right_motor.setMcs(right_motor_freq)
    else:
        right_motor.setValue(motorA)

    if motorB == 0:
        left_motor.setMcs(left_motor_freq)
    else:
        left_motor.setValue(-motorB)
        
def main():
    data = receiver.get_data()
    if data: #если данные получены
        data = pl.loads(data) #распаковываем список команд
        motorB, motorA, servo, cmd = data
        if "exit" in cmd:
            stop()
        arm1.setValue(servo[1])
        griper.setValue(servo[3])
        #motor_run(motorA, motorB)
        motor_run(motorA, motorB)
        
        print(motorA, motorB, cmd)
    else:
        motor_run(0, 0)
        
        
    voltage = adc.getVoltageFiltered()
    replyer.make_reply(voltage)
    #print(voltage)
    time.sleep(0.05)

def stop():
    global running
    running = False
    frameHandlerThread.stop()
    rtpStreamer.stop()
    receiver.stop()
    replyer.stop()
    rpiCamStreamer.stop()
    rpiCamStreamer.close()
    print("program stopped")
    print("server closed")

assert rpicam.checkCamera(), 'Raspberry Pi camera not found'
print('Raspberry Pi camera found')
print('OpenCV version: %s' % cv2.__version__)

rpiCamStreamer = rpicam.RPiCamStreamer(video = FORMAT, resolution = RESOLUTION, framerate = FRAMERATE, onFrameCallback = onFrameCallback)
rpiCamStreamer.setHost(USER_IP)
rpiCamStreamer.setPort(RTP_PORT + 1000)
time.sleep(0.2)
#robotCamStreamer.setFlip(False, True) #отражаем кадр (вертикальное отражение, горизонтальное отражение)
rpiCamStreamer.setRotation(180) #поворачиваем кадр на 180 град, доступные значения 90, 180, 270
rpiCamStreamer.start() #запускаем трансляцию

#отправка служебного cv потока
rtpStreamer = cv_stream.OpenCVRTPStreamer(resolution = RESOLUTION, framerate = FRAMERATE, host = (USER_IP, RTP_PORT))
rtpStreamer.start()

#поток обработки кадров    
frameHandlerThread = FrameHandlerThread(rpiCamStreamer, rtpStreamer, WIDTH, HEIGHT)
frameHandlerThread.start() #запускаем обработку

receiver = transmit_thread.robot()
receiver.setup(IP, PORT)
receiver.start()

replyer = feedback_thread.robot()
replyer.setup(IP, USER_IP, REPLY_PORT)
replyer.start()

arm1 = RPiPWM.Servo270(arm1_chan, extended = True)
griper = RPiPWM.Servo270(griper_chan, extended = True)
left_motor = RPiPWM.ReverseMotor(left_motor_chan)
right_motor = RPiPWM.ReverseMotor(right_motor_chan)
left_motor.setMcs(left_motor_freq)
right_motor.setMcs(right_motor_freq)
time.sleep(3)

adc = RPiPWM.Battery(vRef=3.28)
adc.start()     # запускаем измерения

running = True
first_cicle = True

while running:
    main()
    
