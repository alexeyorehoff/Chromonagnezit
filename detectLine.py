#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
import cv2
import numpy as np

class cv_thread(threading.Thread):
    def __init__(self):
        super(cv_thread, self).__init__()
        self.daemon = True
        self._stopped = threading.Event()
        self._frame = None
        self._newFrameEvent = threading.Event()
        self._debugFrame = None
        self._direction = 0 #направление движения робота
        self._lineFound = None #есть ли линия на кадре
        self._gain = 100
        self._direction = 0 #направление движения робота
        self._lineFound = False 
        self._cx = None
        self._cy = None
        self._leftSpeed = 0
        self._rightSpeed = 0

    def run(self):
        while not self._stopped.is_set():
            print("111")
    """    
    def run(self):
        print("thread started\n")
        hsv_min = np.array((0, 0, 0), np.uint8)
        hsv_max = np.array((100, 255, 100), np.uint8)
        while not self._stopped.is_set():
            self._newFrameEvent.wait()
            if not self._frame is None:
                self._debugFrame = self._frame
                self.detectLine()
            else:
                print("no frame")
            self._newFrameEvent.clear()
        print("thread stopped")
"""
    def detectLine(self):
        
        width = self._frame.shape[1]
        height = self._frame.shape[0]
        
        gray = cv2.cvtColor(self._frame, cv2.COLOR_BGR2GRAY)
        binary = cv2.threshold(gray, self._gain, 255, cv2.THRESH_BINARY_INV)[1]
        cnts = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
        if cnts:#если нашли контур
            #print("contours finded")
            maxContour = max(cnts, key = cv2.contourArea)
            moments = cv2.moments(maxContour)
            dArea = moments["m00"]
            sumX = moments["m10"]
            sumY = moments["m01"]
            if dArea > 50:
                
                self._cx = int(sumX/dArea)     
                self._cy = int(sumY/dArea)    

                self._lineFound = True
                cv2.line(self._frame, (self._cx, 0), (self._cx, height), (255, 0, 0), 1)
                cv2.line(self._frame, (0, self._cy), (width, self._cy), (255, 0, 0), 1)
                cv2.circle(self._frame, (self._cx, self._cy), 5, (0, 0, 255), 10, lineType=8, shift=0)
                           
            cv2.drawContours(self._debugFrame, maxContour, -1, (0, 255, 0), 3)
            
    def changeGain(self, step):
        if (self._gain <= 255 and step > 0) or (self._gain >= 0 and step < 0):
            self._gain += step
        elif self._gain < 0:
            self._gain = 0
        elif self._gain > 255:
            self._gain = 255
        
    def stop(self):
        self._stopped.set()
        self.join()

    def setframe(self, frame):
        """добавляем и пре-обрабатываем новый кадр"""
        if not self._newFrameEvent.is_set():
            self._frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            self._newFrameEvent.set()
            #print("frame is ready")
            return True
        return False
    
    def isready(self):
        return not self._newFrameEvent.is_set()
    
