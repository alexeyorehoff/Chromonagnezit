#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket #библиотека для связи между пультом и роботом
import os 
import _pickle as pl
import sys 
import time
import threading
import crc16
import psutil

import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera

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
USER_IP = '192.168.42.100' #IP пульта (IP приемника видео)
RTP_PORT = 5000 #порт отправки RTP видео
TIMEOUT = 120 #время ожидания приема сообщения

FORMAT = rpicam.VIDEO_MJPEG #поток MJPEG
WIDTH, HEIGHT = 640, 480
RESOLUTION = (WIDTH, HEIGHT)
HIGH_RES = (960, 720)
FRAMERATE = 30

#каналы, на которых сидят устройства
tail_chan = 0 #задние колеса (подъем/опускание)
arm1_chan = 1 #первое сочленение манипулятора
arm2_chan = 2 #второе сочленение манипулятора 
griper_chan = 3 #захват
griper_rot_chan = 4 #поворот захвата
cam_tilt_chan = 10 #подъем/опускание курсовой камеры
right_motor_chan = 6 
left_motor_chan = 7 #правый и левый моторы

right_motor_freq = 1460 #частота ШИМ соответсвующая нулю
left_motor_freq = 1460


#/////////////////////////////////////////////////////////////////////////////////////
#///////////поток работает, но видео с курсовой камеры фризит////////////////////////
class cameraMaster(threading.Thread):
    def __init__(self, rtpStreamer, rpiCamStream, resolution = (640, 480), stream_resolution = (320, 240), framerate = 30):
        super(cameraMaster, self).__init__()
        self.daemon = True
        self._resolution = resolution
        self._stream_resolution = stream_resolution #разрешение видео, которое будет отправляться
        self._framerate = framerate 
        self._source = 1 #источник видео
        self._state = 0
        self._frame = None
        self._newFrameEvent = threading.Event()
        self._stopped = threading.Event() #событие для остановки потока
        self._rtp_streamer = rtpStreamer
        self._rpi_cam_streamer = rpiCamStream
        self._rpi_cam_streamer = rpiCamStream
        self._camera = PiCamera()
        self._camera.resolution = stream_resolution
        self._camera.framerate = framerate
        self._rawCapture = PiRGBArray(self._camera, size=(stream_resolution))

        self._cap = cv2.VideoCapture(-1)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self._cap.set(cv2.CAP_PROP_FPS, framerate)
        time.sleep(0.5)
        
    def run(self):
        print("camera master started")
        while not self._stopped.is_set():
            if self._state == 0: #просто транслируем видео с основной камеры
                if self._rpi_cam_streamer.frameRequest(): #отправил запрос на новый кадр
                #print("waiting for frame")
                    self._newFrameEvent.wait() #ждем появления нового кадра
                    if self._frame: #если кадр есть
                        self._frame = np.ndarray((self._stream_resolution[0], self._stream_resolution[1], 3), buffer = self._frame, dtype = np.uint8)
                        #self._frame = cv2.cvtColor(self._frame, cv2.COLOR_BGR2RGB)
                        #self.rtpStream.sendFrame(self._frame) #помещаем кадр в поток
                    self._newFrameEvent.clear()
                
                for frame in self._camera.capture_continuous(self._rawCapture, format="bgr", use_video_port=True):
                    self.frame = frame.array
                    self.frame = np.rot90(self.frame, 2)
                    self.frame = cv2.resize(self.frame, self._stream_resolution, interpolation = cv2.INTER_AREA)
                    self._rtp_streamer.sendFrame(self.frame)
                    self._rawCapture.truncate(0)
                    if self.state != 0 or self._stopped.is_set():
                        break
                
            elif self._state == 1: #транслируем видео с камеры на клешне
                while True:
                    ret, self._frame = self._cap.read()
                    if ret:
                        self._frame = cv2.resize(self.frame, self._stream_resolution, interpolation = cv2.INTER_AREA)
                        self._rtp_streamer.sendFrame(self.frame)
                    if self._state != 1 or self._stopped.is_set():
                        break
        print("camera master stopped")
    def change_state(self, val):
        self._state = val
        
    def setFrame(self, frame): #задание нового кадра для обработки
        if not self._newFrameEvent.is_set(): #если обработчик готов принять новый кадр
            self._frame = frame
            self._newFrameEvent.set() #задали событие
            return True
        return False
    
    def stop(self):
        self._cap.release()
        print("cap released")
        self._stopped.set()
        self.join() #ждем завершения работы потока
#///////////////////////////////////////////////////////////////////////////////
        
class FrameHandlerThread(threading.Thread):
    def __init__(self, camStream, debugStream, width, height):
        super(FrameHandlerThread, self).__init__()
        self.daemon = True
        self.rpiCamStream = camStream #поток откуда берем видео с RPi камеры
        self.debugStream = debugStream #поток для всего остального
        self._frame = None
        self._frameCount = 0
        self._stopped = threading.Event() #событие для остановки потока
        self._newFrameEvent = threading.Event() #событие для контроля поступления кадров
        self.state = None #режим работы
        # 0 - езда по линии 1 - просто передача видео с вспомогательной камеры
        self.sensitivity = 50 #чувствительность алгоритма определения линии (0..255)
        self.stopped = None #переменная для блокирования движений робота в режиме автономки
        self.base_speed = 0 #регулятор скорости, с которой едет робот в режиме автономки
        self.koof = 50 #коэффициент пропорциональности
        self._gain = 1.9 #величина усиления управляющего воздействия от близости поворота
        self._blur = 9 #величина Гауссового ядра для размытия
        self.width = width
        if width > WIDTH:
            self.width = WIDTH
        self.height = height
        if height > HEIGHT:
            self.height = HEIGHT
        self._cap = cv2.VideoCapture(-1)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
        self._cap.set(cv2.CAP_PROP_FPS, FRAMERATE)
        
    def run(self):
        print('Frame handler started')
        while not self._stopped.is_set():
            if self.state == 0: #если включен режим езды по линии
                if self.rpiCamStream.frameRequest(): #отправил запрос на новый кадр
                    #print("waiting for frame")
                    self._newFrameEvent.wait() #ждем появления нового кадра
                    if self._frame: #если кадр есть
                        self._frame = np.ndarray((self.height, self.width, 3), buffer = self._frame, dtype = np.uint8)
                        self._frame = cv2.cvtColor(self._frame, cv2.COLOR_BGR2RGB)
                        lineFound, error, resImg = self.detectLine(self._frame) #вызываем функцию определения линии
                        leftSpeed = 0
                        rightSpeed = 0
                        if lineFound: # если линия была обнаружена, задем скорости
                            leftSpeed = round(self.base_speed + error*self.koof)
                            rightSpeed = round(self.base_speed - error*self.koof)
                            self._frame = cv2.putText(resImg, "%.2f|%d|%d" % (self.base_speed, leftSpeed, rightSpeed), (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, [0, 0, 0], 2)
                        if not self.stopped:
                            if leftSpeed < 0:
                                leftSpeed = 0
                            if rightSpeed < 0:
                                rightSpeed = 0
                            motor_run(leftSpeed, rightSpeed) 
                            #print('%.2f\t%d\t%d' % (error, leftSpeed, rightSpeed))
                        self.debugStream.sendFrame(self._frame) #помещаем кадр в поток
                self._newFrameEvent.clear() #сбрасываем событие
            elif self.state == 1: #если включен режим видео с клешни
                while True:
                    ret, self._frame = self._cap.read()
                    if ret:
                        self.debugStream.sendFrame(self._frame)
                    if self.state != 0 or self._stopped.is_set():
                        break
            else:
                pass
        print("cap released")
        print('Frame handler stopped')
        
    def detectLine(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)  #преобразуем в градации серого
        blur = cv2.GaussianBlur(gray, (self._blur, self._blur), 0)   # размываем изображение blur
        ret, thresh = cv2.threshold(blur, self.sensitivity, 255, cv2.THRESH_BINARY_INV) #бинаризация в ч/б (исходное изобр, порог, максимальное знач., тип бинаризации)
        contours = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0] #нахождение контуров
        
        error = 0.0 #ошибка
        cx = self.width//2 #на случай если не получится найти линию
        cy = self.height
        lineFound = False
        if len(contours) > 0:   # если есть хоть один контур
            lineFound = True
            mainContour = max(contours, key = cv2.contourArea)  # берем самый большой
            M = cv2.moments(mainContour)  # берем его
            if M['m00'] >= 25:   # если нет деления на ноль и если пятно достаточно большое
                cx = int(M['m10']/M['m00'])     # смотрим координаты центра наибольшего черного пятна
                cy = int(M['m01']/M['m00'])
                cv2.line(frame, (cx, 0), (cx, self.height), (255, 0, 0), 1)    # рисуем перекрестие на контуре
                cv2.line(frame, (0, cy), (self.width, cy), (255, 0, 0), 1)
                    
            cv2.circle(frame, (self.width//2, self.height//2), 3, (0, 0, 255), -1) #отрисовываем центральную точку
            cv2.drawContours(frame, mainContour, -1, (0, 255, 0), 2, cv2.FILLED) #отображаем контуры на изображении
        error = cx / (self.width/2) - 1  # преобразуем координаты от 0 до ширина кадра -> от -1 до 1
        error *= cy / self.height + self._gain #усиление от 1 до 2
        
        return lineFound, error, frame
    
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

def lights_change(state):
    global lights_state
    if lights_state == True and state == False:
        for i in range(3):
            light.setValue(60)
            time.sleep(0.1)
            light.setValue(150)
            time.sleep(0.1)
        
        lights_state = False

    if lights_state == False and state == True:
        for i in range(1):
            light.setValue(60)
            time.sleep(0.1)
            light.setValue(150)
            time.sleep(0.1)
        lights_state = True
            

def stop():
    global running
    running = False
    rtpStreamer.stop()
    rpiCamStreamer.stop()
    rpiCamStreamer.close()
    receiver.stop()
    replyer.stop()
    motor_run(0, 0)
    print("program stopped")
    
def main():
    data = receiver.get_data()
    if data: #если данные получены
        motorB, motorA, servo, cmd, state, auto_mode, lights = data
        #print(data)
        
        frameHandlerThread.state = state
        frameHandlerThread.sensitivity = auto_mode[1]
        frameHandlerThread.stopped = auto_mode[2]
        frameHandlerThread.base_speed = auto_mode[3]
        
        if state == 0:
            cam_tilt.setValue(48) #установка камеры в положение для езды по линии
            lights_change(True)
        else:
            cam_tilt.setValue(90) #установка камеры в "основное" положение (для д/у)
            motor_run(motorA, motorB)
            lights_change(lights)
        tail.setValue(servo[0])
        arm1.setValue(servo[1])
        arm2.setValue(servo[2])
        griper_rot.setValue(servo[3])
        griper.setValue(servo[4])
            
        if "exit" in cmd:
            stop()
    else:
        motor_run(0, 0)
        print("no data")
        
    voltage = vmeter.getVoltageFiltered()
    replyer.make_reply(voltage)
    #print(voltage)
    time.sleep(0.01)

assert rpicam.checkCamera(), 'Raspberry Pi camera not found'
print('Raspberry Pi camera found')
print('OpenCV version: %s' % cv2.__version__)



rpiCamStreamer = rpicam.RPiCamStreamer(video = FORMAT, resolution = RESOLUTION, framerate = FRAMERATE, onFrameCallback = onFrameCallback)
rpiCamStreamer.setHost(USER_IP)
rpiCamStreamer.setPort(RTP_PORT)#поток для пустого видео с курсовой камеры

#rpiCamStreamer.setFlip(False, True)
rpiCamStreamer.setRotation(180) #поворачиваем кадр на 180 град
rpiCamStreamer.start() #запускаем трансляцию
#отправка служебного потока для всего кроме картинки с курсовой камеры
debugStreamer = cv_stream.OpenCVRTPStreamer(resolution = RESOLUTION, framerate = FRAMERATE, host = (USER_IP, RTP_PORT + 1000))
debugStreamer.start()
#поток обработки кадров    
frameHandlerThread = FrameHandlerThread(rpiCamStreamer, debugStreamer, WIDTH, HEIGHT)
frameHandlerThread.start() #запускаем обработку

receiver = transmit_thread.robot() #поток-приемник команд с ПК
receiver.setup(IP, PORT)
receiver.start()

replyer = feedback_thread.robot() #отправка телеметрии (плюс еще для измерения задержки)
replyer.setup(IP, USER_IP, REPLY_PORT)
replyer.start()

tail = RPiPWM.Servo270(tail_chan, extended = True)
arm1 = RPiPWM.Servo270(arm1_chan, extended = True)
arm2 = RPiPWM.Servo270(arm2_chan, extended = True)
griper = RPiPWM.Servo270(griper_chan, extended = True)
griper_rot =  RPiPWM.Servo180(griper_rot_chan, extended = True)
cam_tilt = RPiPWM.Servo180(cam_tilt_chan, extended = True)
light = RPiPWM.Servo270(11, extended = True)
left_motor = RPiPWM.ReverseMotor(left_motor_chan)
right_motor = RPiPWM.ReverseMotor(right_motor_chan)
left_motor.setMcs(left_motor_freq)
right_motor.setMcs(right_motor_freq)
time.sleep(3)

vmeter = RPiPWM.Battery(vRef=3.28)
vmeter.start()     # запускаем измерения

running = True
first_cicle = True
lights_state = False

while running:
    main()
    
