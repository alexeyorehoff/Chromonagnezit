#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import pygame
import os
import math
import socket
import time
import threading
import crc16
import transmit_thread
import feedback_thread
sys.path.append('/home/alexey/RPicam-Streamer/')
import receiver
import cv_stream
import cv2

IP_ROBOT = '192.168.42.10'
IP = str(os.popen("hostname -I | cut -d\" \" -f1").readline().replace("\n",""))
PORT = 8000
REPLY_PORT = 9000
RTP_PORT = 5000
TIMEOUT = 120
WIDTH, HEIGHT = 720, 480
RESOLUTION = (WIDTH, HEIGHT)
FORMAT = receiver.VIDEO_MJPEG
FPS = 30
#физические ограничения для серв
tailedp = [45, 260]
arm1edp = [-10, 145]
arm2edp = [65, 200]
griperturnedp = [20, 145]
griperedp = [10, 230]

GRIP_INCREMENT = 5 #чувствительность для управления захватом
TAIL_INCREMENT = 2 #чувствительность для управления хвостом

KOOF = 1 #чувствительность джойстика
SCALE = 3 #масштаб для элементов робота на экране
arm1 = 10 * SCALE#длины секций манипулятора робота в см
arm2 = 21 * SCALE
robot_height = 9 * SCALE

def onFrameCallback(data, width, height):
    frame = pygame.image.frombuffer(data, (width, height), 'RGB')
    print(frame)
    w, h = pygame.display.Info().current_w, pygame.display.Info().current_w
    frame = pygame.transform.scale(frame, (int(w / 1.5), int(h / 1.5)))
    screen_master.give_main_frame(frame)
    #screen.blit(frame, (0,0))
    
def onFrameCallback1(data, width, height):
    frame = pygame.image.frombuffer(data, (width, height), 'RGB')
    #print(frame)
    w, h = pygame.display.Info().current_w, pygame.display.Info().current_w
    frame = pygame.transform.scale(frame, (int(w / 1.5), int(h / 1.5)))
    screen_master.give_debug_frame(frame)
    
class screen_thread(threading.Thread):
    def __init__(self, screen_surf = None):
        super(screen_thread, self).__init__()
        self.daemon = True
        self._screen = screen_surf
        self._stopped = threading.Event()
        self._state = -1 #по умолчанию берем видео с курсовой камеры
        self._main_frame = None #кадры основного видеопотока
        self._debug_frame = None #кадры отладочного видеопотока
        self.auto_mode = False #для переключения между основным и отладочным видеопотокм
        self._widht, self._height = pygame.display.get_surface().get_size()
        self._clock = pygame.time.Clock() #для осуществления задержки
        self.a_angle = 0 #угол первой секции клешни относительно робота
        self.b_angle = 0 #угол второй секции клешни относительно робота
        
    def run(self):
        while not self._stopped.is_set():
            self._screen.fill((255, 255, 255))
            if self._main_frame and self._state == -1:
                screen.blit(self._main_frame, (0, 0))
            elif self._debug_frame and self._state in (0, 1):
                screen.blit(self._debug_frame, (0, 0))
            else: 
                screen.fill((255, 255, 255))
                text = myfont.render('No frame', False, (255, 0, 0))
                screen.blit(text,(self._widht // 2,self._height // 2))
                
            #отрисовка элементов робота    
            pygame.draw.rect(screen, (0, 0, 0), ((self._widht - 100, 100), (-70, robot_height)))
            pygame.draw.circle(screen, (0, 255, 0), (self._widht - 100, 100), int(arm1 + arm2), 1)
            pygame.draw.circle(screen, (0, 255, 0), (self._widht - 100, 100), int(low_of_cosines(arm1, arm2, 0, math.radians(40))), 1)
            pygame.draw.circle(screen, (255, 0, 0), (self._widht - 100 + x_axis , 100 + robot_height - y_axis), 3, 0)
            X1 = self._widht - 100 + int(arm1 * math.cos(self.a_angle))
            Y1 = 100 - int(arm1 * math.sin(self.a_angle))
            X2 = X1 + int(arm2 * math.cos(self.b_angle))
            Y2 = Y1 - int(arm2 * math.sin(self.b_angle))
            pygame.draw.line(screen, (255, 0, 0), (self._widht - 100, 100),(X1, Y1))
            pygame.draw.line(screen, (255, 0, 0), (X1, Y1), (X2, Y2))
            
            pygame.display.update()
            self._clock.tick(30) #задержка обеспечивающая 30 кадров в секунду

    def stop(self):
        self._stopped.set()
        self.join()
        
    def change_state(self, val):
        self._state = val
        
    def give_main_frame(self, frame):
        self._main_frame = frame
    
    def give_debug_frame(self, frame):
        self._debug_frame = frame
        
def lenght(s, e): #координаты начала вектора, координаты конца вектора
    """вычисление длины вектора по его координатам"""
    return math.sqrt((s[0] - e[0]) ** 2 + (s[1] - e[1]) ** 2)

def low_of_cosines(a, b, c, w):
    if not w:
        w = math.acos((a ** 2 + b ** 2 - c ** 2)/(2 * a * b))
        return w
    elif not c:
        c = math.sqrt(a ** 2 + b ** 2 - 2 * a * b * math.cos(w))
        return c
    else:
        pass

def map(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

def angles(x, y, L1, L2):
    """расчет углов поворота манипулятора"""
    L = math.sqrt(x ** 2 + y ** 2)
    B = low_of_cosines(L1, L2, L, 0) #угол между двумя сочленениями манипулятора
    W1 = math.atan(y/x)
    W2 = low_of_cosines(L1, L, L2, 0)
    A = W1 + W2 #угол между первым сочленением и корпусом робота
    C = A + B - math.pi #угол между вторым сочленением и уорпусом робота (для отображения на экране)
    screen_master.a_angle = A
    screen_master.b_angle = C
    return A, B

def stop():
    global running
    global cmd
    cmd.append("exit")
    time.sleep(0.5)
    print("End program")
    running = False
    recv.stop_pipeline()
    recv.null_pipeline()
    screen_master.stop()
    transmitter.stop()
    replyer.stop()
    time.sleep(0.5)
    pygame.quit() #завершаем Pygame

pygame.init() #инициализация Pygame
pygame.mixer.quit()
screen = pygame.display.set_mode([720, 480], pygame.RESIZABLE) #создаем окно программы
pygame.font.init()
myfont = pygame.font.SysFont('Comic Sans MS', 50)
pygame.display.set_caption("robot-RTC")
try:
    joy = pygame.joystick.Joystick(0) # создаем объект джойстик
    joy.init() # инициализируем джойстик
    print('Enabled joystick: ' + joy.get_name())
except pygame.error:
    print('no joystick found.')


running = True
power = 100
servo = [50, 90, 180, 90, 65]
power = 100
cmd = []
keys = []
motor_a = 0
motor_b = 0
griper_turn = 90 #угол поворота клешни
auto_mode = False
auto_mode_type = None
threshold = 50
stopped = True
base_speed = 50
lights = False
x_axis = int(low_of_cosines(arm1, arm2, 0, math.pi / 3))
y_axis = 0
vx = 0
vy = 0
v_grip = 0
v_tail = 0
state = -1 #режимы робота - -1 - работаем по курсовой камере, 0 - езда по линии, 1 - видео с камеры клешни

screen_master = screen_thread(screen_surf = screen)
screen_master.start()

transmitter = transmit_thread.pult()
transmitter.start()
transmitter.setup(IP_ROBOT, PORT)

replyer = feedback_thread.pult()
replyer.start()
replyer.setup(IP_ROBOT, REPLY_PORT, 60)

recv = receiver.StreamReceiver(video = FORMAT, host = (IP_ROBOT, RTP_PORT), onFrameCallback = onFrameCallback)
recv.play_pipeline()

recvAuto = receiver.StreamReceiver(video = FORMAT, host = (IP_ROBOT, RTP_PORT + 1000), onFrameCallback = onFrameCallback1)
recvAuto.play_pipeline()

time.sleep(1)
while running:
    for event in pygame.event.get():#пробегаемся по всем событиям pygame
        if event.type == pygame.QUIT:#если пользователь завкрывает окно pygame останавливаем программу
            stop()
        if event.type == pygame.KEYUP and event.key == pygame.K_ESCAPE:#отпускание клавиш
            stop()
        elif event.type == pygame.JOYHATMOTION:
            if event.value[0] == 0 and event.value[1] == 1:
                if not auto_mode:
                    motor_a = power
                    motor_b = power
                else:
                    if base_speed < 100:
                        base_speed += 10
            elif event.value[0] == 0 and event.value[1] == -1:
                if not auto_mode:
                    motor_a = -power
                    motor_b = -power
                else:
                    if base_speed > 0:
                        base_speed -= 10
            elif event.value[0] == 0 and event.value[1] == -1:
                motor_a = -power
                motor_b = -power
            elif event.value[0] == 1 and event.value[1] == 0:
                motor_a = -power
                motor_b = power
            elif event.value[0] == -1 and event.value[1] == 0:
                motor_a = power
                motor_b = -power
            elif event.value[0] == 1 and event.value[1] == 1:
                motor_a = 0
                motor_b = power
            elif event.value[0] == -1 and event.value[1] == 1:
                motor_a = power
                motor_b = 0
            elif event.value[0] == 1 and event.value[1] == -1:
                motor_a = 0
                motor_b = -power
            elif event.value[0] == -1 and event.value[1] == -1:
                motor_a = -power
                motor_b = 0
            elif event.value[0] == 0 and event.value[1] == 0:
                motor_a = 0
                motor_b = 0
                
        elif event.type == pygame.JOYBUTTONDOWN:
            if event.button == 5 and power + 10 <= 100:
                power += 10
            elif event.button == 4 and power - 10 >= 0:
                power -= 10
            
            if event.button == 8 or "q" in keys:
                screen_master.auto_mode = not screen_master.auto_mode
                auto_mode = not auto_mode
                if not auto_mode:
                    state = -1
                stopped = True
                if auto_mode_type:
                    auto_mode_type = None
            if auto_mode and event.button == 2: #включение редима езды по линии
                state = 0
                
            if event.button == 6: #переключение источника видео
                # True - курсовая камера False - камера с клешни
                if state == 1:
                    state = -1
                elif state == -1:
                    state = 1
            """
            #режимы пока не задействованы 
            if auto_mode and event.button == 0:
                auto_mode_type = 3
            if auto_mode and event.button == 1:
                auto_mode_type = 4
            """
            if event.button == 2: #включение огней робота
                lights = not lights
            if auto_mode and event.button == 4:#настройка порога чувствительности
                threshold -= 5
            elif auto_mode and event.button == 5:
                threshold += 5
            if auto_mode and event.button == 7:#остановка и запуск робота в режиме автономки
                stopped = not stopped
            
             
        if event.type == pygame.JOYAXISMOTION:
            if event.axis == 1:
                vy = -event.value
            if event.axis == 0:
                vx = event.value
            if event.axis == 4:
                v_tail = int(map(event.value, -1, 1, -TAIL_INCREMENT, TAIL_INCREMENT))
            if event.axis == 3:
                griper_turn = int(map(event.value, -1, 1, 180, 0))
            if event.axis == 2 :
                v_grip = -int(map(event.value, -1, 1, 0, GRIP_INCREMENT))
            elif event.axis == 5 and servo[4] < griperedp[1]:
                v_grip = int(map(event.value, -1, 1, 0, GRIP_INCREMENT))
            
    #не позволяем манипулятору уйти дальше, чем это возможно физически 
    if lenght((0, robot_height), (x_axis + int(vx * KOOF), y_axis)) <= (arm1 + arm2) and lenght((0, robot_height), (x_axis + int(vx * KOOF), y_axis)) >= low_of_cosines(arm1, arm2, 0, math.radians(40)):
        x_axis += int(vx * KOOF)
    if lenght((0, robot_height), (x_axis, y_axis + int(vy * KOOF))) <= (arm1 + arm2) and lenght((0, robot_height), (x_axis, y_axis + int(vy * KOOF))) >= low_of_cosines(arm1, arm2, 0, math.radians(40)):
        y_axis += int(vy * KOOF)
    if x_axis <= 0: #отсекаем все положения за роботом
        x_axis = 0
        
    a, b = angles(x_axis, y_axis - robot_height, arm1, arm2)
    a = math.degrees(a)
    b = math.degrees(b) + 20 #конструкционно второй сервопивод робота распложен под углом 20 градусов к первому сочленению манипулятора
    
    if a < arm1edp[0]:
        a = arm1edp[0]
    elif a > arm1edp[1]:
        a = arm1edp[1]
    if b < arm2edp[0]:
        b = arm2edp[0]
    elif b > arm2edp[1]:
        b = arm2edp[1]

    if griper_turn <= griperturnedp[0]:
        griper_turn = griperturnedp[0]
    elif griper_turn >= griperturnedp[1]:
        griper_turn = griperturnedp[1]

    if servo[0] + v_tail > tailedp[0] and servo[0] + v_tail < tailedp[1]:
        servo[0] += v_tail
    servo[1] = a
    servo[2] = b
    servo[3] = griper_turn
    if servo[4] + v_grip > griperedp[0] and servo[4] + v_grip < griperedp[1]:
        servo[4] += v_grip

    screen_master.change_state(state)
    transmitter.send_data((motor_a, motor_b, servo, cmd, state, (auto_mode_type, threshold, stopped, base_speed), lights))         
    try:
        voltage, latency = replyer.get_inf()
        print(voltage)
    except:
        pass
        #print("no reply data to print")
    time.sleep(0.01)
    
        


