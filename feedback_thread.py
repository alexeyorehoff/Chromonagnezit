#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
import pickle as pl
import socket
import crc16
import os

class pult(threading.Thread):
    def __init__(self):
        super(pult, self).__init__()
        self.daemon = True
        self._stopped = threading.Event()
        self._data = None
        self._ip = None
        self._port = None
        self._ip_robot = None
        self._reply_port = None
        self._reply = None
        self._timeout = 5
        self._latency = None
        self._waiting = True
        self._frequency = 4 #сколько раз в минуту проводить опрос
        
    def run(self):
        print("feedback thread started\n")
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #создаем сервер
        time.sleep(0.1)
        server.bind((self._ip, self._port)) #запускаем сервер
        server.settimeout(self._timeout)#утсановка времени ожидания сообщения для сервера
        
        while not self._stopped.is_set():
            if self._data: 
                data = self._data #запаковываем данные
            else:
                data = "reply request"
            msg = pl.dumps(data)
            client.sendto(msg, (self._ip_robot, self._port))
            start_time = time.clock()
            try:
                msg = server.recvfrom(1024) #пытаемся получить данные
                self._reply = pl.loads(msg[0])
            except socket.timeout: 
                print("no reply")
            self._latency = time.clock() - start_time
            self._data = None
            time.sleep(60/self._frequency)
        server.close()
                
    def make_request(self, data):
        self._data = data
            
    def setup(self, ip_robot, reply_port, freq):
        self._ip = str(os.popen("hostname -I | cut -d\" \" -f1").readline().replace("\n",""))
        self._ip_robot = ip_robot
        self._port = reply_port
        self._frequency = freq
        
    def get_inf(self):
        if self._reply and self._latency:
            return self._reply, round((self._latency * 1000), 2)
        
    def stop(self):
        self._stopped.set()
        print("feedback thread stopped")
        self.join() #ждем завершения работы потока

class robot(threading.Thread):
    def __init__(self):
        super(robot, self).__init__()
        self.daemon = True
        self._stopped = threading.Event()
        self._data = None
        self._request = None
        self._ip = None
        self._port = None
        self._ip_pult = None
        self._timeout = 60
        
    def run(self):
        print("thread started\n")
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #создаем сервер
        time.sleep(0.1)
        server.bind((self._ip, self._port)) #запускаем сервер
        server.settimeout(self._timeout)#утсановка времени ожидания сообщения для сервера
        
        while not self._stopped.is_set():
            msg = None
            try:
                msg = server.recvfrom(1024) #пытаемся получить данные
            except socket.timeout: 
                print("no feedback request")
            if msg:
                self._request = pl.loads(msg[0])
                if self._data:
                    data = self._data 
                else:
                    data = "reply"
                msg = pl.dumps(data)
                client.sendto(msg, (self._ip_pult, self._port))
            time.sleep(1)
        server.close()
        
    def make_reply(self, reply):
        self._data = reply
        
    def setup(self, ip, ip_user, port):
        self._ip = ip
        self._port = port
        self._ip_pult = ip_user
        
    def get_request(self):
        if self._request:
            return self.request
        
    def stop(self):
        self._stopped.set()
        print("feedback thread stopped")
        self.join()
