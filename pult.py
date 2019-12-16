#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import pygame
import os
import socket
import pickle
#sys.path.append('/home/alexey/robot-RTC/RPicam-Streamer')
import time
import threading
import crc16
import transmit_thread
import feedback_thread
sys.path.append('/home/alexey/RPicam-Streamer/')
import receiver
import cv_stream
import cv2

IP_ROBOT = '169.254.179.225'
IP = str(os.popen("hostname -I | cut -d\" \" -f1").readline().replace("\n",""))
PORT = 8000
RTP_PORT = 5000
REPLY_PORT = 9000
TIMEOUT = 120
WIDTH, HEIGHT = 720, 480
RESOLUTION = (WIDTH, HEIGHT)
FORMAT = receiver.VIDEO_MJPEG
FPS = 30
#servo edge positions
tailedp = [0, 270]
arm1edp = []
arm2edp = [50, 170]
griperedp = [50, 270] 


def onFrameCallback(data, width, height):
    frame = pygame.image.frombuffer(data, (width, height), 'RGB')
    w, h = pygame.display.Info().current_w, pygame.display.Info().current_w
    frame = pygame.transform.scale(frame, (int(w / 1.5), int(h / 1.5)))
    screen_master.give_main_frame(frame)
    #screen.blit(frame, (0,0))
    
def onFrameCallback1(data, width, height):
    frame = pygame.image.frombuffer(data, (width, height), 'RGB')
    w, h = pygame.display.Info().current_w, pygame.display.Info().current_w
    frame = pygame.transform.scale(frame, (int(w / 1.5), int(h / 1.5)))
    screen_master.give_debug_frame(frame)
    #screen_master.give_debug_frame(frame)
    
class screen_thread(threading.Thread):
    def __init__(self, screen_surf = None):
        super(screen_thread, self).__init__()
        self.daemon = True
        self._screen = screen_surf
        self._stopped = threading.Event()
        self._main_frame = None
        self._debug_frame = None
        self.auto_mode = False
        self._clock = pygame.time.Clock() #для осуществления задержки
    def run(self):
        self._screen.fill((255, 255, 255))
        while not self._stopped.is_set():
            if not self.auto_mode and self._main_frame:
                screen.blit(self._main_frame, (0, 0))
            elif self.auto_mode and self._debug_frame:
                print("automode")
                screen.blit(self._debug_frame, (0, 0))
            else:
                print("no frame")
    
            self._clock.tick(30) #задержка обеспечивающая 30 кадров в секунду

    def stop(self):
        self._stopped.set()
        self.join()
    def give_main_frame(self, frame):
        self._main_frame = frame
    
    def give_debug_frame(self, frame):
        self._debug_frame = frame
            
def main():
    global keys
    global power
    global first_cicle
    global motor_a
    global motor_b
    global auto_mode
    
    for event in pygame.event.get():#пробегаемся по всем событиям pygame
        if event.type == pygame.QUIT:#если пользователь завкрывает окно pygame останавливаем программу 
            stop()
        if event.type == pygame.KEYDOWN:#нажатие на клавиши
            if event.key == pygame.K_q and "q" not in keys:
                keys.append("q")
            if event.key == pygame.K_s and "s" not in keys:
                keys.append("s")
            if event.key == pygame.K_a and "a" not in keys:
                keys.append("a")
            if event.key == pygame.K_d and "d" not in keys:
                keys.append("d")
            if (event.key == pygame.K_EQUALS or event.key == pygame.K_KP_PLUS) and power < 100:
                power += 10
            if (event.key == pygame.K_MINUS or event.key == pygame.K_KP_MINUS) and power > 0:
                power -= 10
                    
            if event.key == pygame.K_r and servo[1] < 270:
                servo[1] += 10
            if event.key == pygame.K_f and servo[1] > 0:
                servo[1] -= 10
                
           
        elif event.type == pygame.KEYUP:#отпускание клавиш
            if event.key == pygame.K_q:
                keys.remove("q")
            if event.key == pygame.K_s:
                keys.remove("s")
            if event.key == pygame.K_a:
                keys.remove("a")
            if event.key == pygame.K_d:
                keys.remove("d")
            if event.key == pygame.K_ESCAPE:
                stop()
        elif event.type == pygame.JOYHATMOTION:
            if event.value[0] == 0 and event.value[1] == 1:
                motor_a = power
                motor_b = power
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
            if event.button == 5 and servo[3] < griperedp[1]:
                servo[3] += 10
            elif event.button == 4 and servo[3] > griperedp[0]:
                servo[3] -= 10
            elif event.button == 8 or "q" in keys:
                if "AM" not in cmd:
                    cmd.append("AM")
                else:
                    cmd.remove("AM")
                screen_master.auto_mode = not screen_master.auto_mode
                auto_mode = not auto_mode
        
           
    if keys: #если список нажатых кнопок не пуст, то проверяем его на наличие знакомых комбинаций
        if "w" in keys and "d" in keys:
            motor_a = 0
            motor_b = power
        elif "w" in keys and "a" in keys:
            motor_a = power
            motor_b = 0
        elif "s" in keys and "a" in keys:
            motor_a = 0
            motor_b = -power
        elif "s" in keys and "d" in keys:
            motor_a = -power
            motor_b = 0
        elif "w" in keys:
            motor_a = power
            motor_b = power
        elif "s" in keys:
            motor_a = -power
            motor_b = -power
        elif "d" in keys:
            motor_a = -power
            motor_b = power
        elif "a" in keys:
            motor_a = power
            motor_b = -power
    transmitter.send_data((motor_a, motor_b, servo, cmd))
    
    pygame.display.update()         
    try:
        voltage, latency = replyer.get_inf()
        print(voltage)
    except:
        pass
        print("no reply data to print")
    time.sleep(0.02)

def stop():
    global running
    global cmd
    cmd.append("exit")
    time.sleep(0.5)
    print("End program")
    running = False
    recv.stop_pipeline()
    recv.null_pipeline()
    recvAuto.stop_pipeline()
    recvAuto.null_pipeline()
    transmitter.stop()
    replyer.stop()
    time.sleep(0.5)
    pygame.quit() #завершаем Pygame


pygame.init() #инициализация Pygame
pygame.mixer.quit()
screen = pygame.display.set_mode([720, 480], pygame.RESIZABLE) #создаем окно программы
pygame.display.set_caption("robot-RTC")
try:
    joy = pygame.joystick.Joystick(0) # создаем объект джойстик
    joy.init() # инициализируем джойстик
    print('Enabled joystick: ' + joy.get_name())
except pygame.error:
    print('no joystick found.')

running = True

power = 100
servo = [0, 90, 70, 160]
power = 100
cmd = []
keys = []
motor_a = 0
motor_b = 0
auto_mode = False

screen_master = screen_thread(screen_surf = screen)
screen_master.start()

transmitter = transmit_thread.pult()
transmitter.start()
transmitter.setup(IP_ROBOT, PORT)

replyer = feedback_thread.pult()
replyer.start()
replyer.setup(IP_ROBOT, REPLY_PORT, 60)

recv = receiver.StreamReceiver(video = FORMAT, host = (IP_ROBOT, RTP_PORT + 1000), onFrameCallback = onFrameCallback)
recv.play_pipeline()
"""
receiver = cv_stream.OpenCVRTPReciver(host = (IP_ROBOT, RTP_PORT), onFrameCallback = onFrameCallback)
receiver.start()
"""
recvAuto = receiver.StreamReceiver(video = FORMAT, host = (IP_ROBOT, RTP_PORT), onFrameCallback = onFrameCallback1)
recvAuto.play_pipeline()

while running:
    main()
    
        


