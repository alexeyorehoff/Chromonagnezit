#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
import _pickle as pl
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
        self._timeout = 1
        
    def run(self):
        print("thread started\n")
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        time.sleep(0.1)
        
        while not self._stopped.is_set():
            if not self._data is None:
                msg = pl.dumps(self._data)#запаковываем данные
                client.sendto(msg, (self._ip_robot, self._port))
            time.sleep(0.01)
        """
        while not self._stopped.is_set():
            if not self._data is None:
                crc = crc16.crc16xmodem(self._data)#вычисляем контрольную сумму пакета
                msg = pl.dumps((data, crc))#прикрепляем вычисленную контрольную сумму к пакету данных
                client.sendto(msg, (self._ip_robot, self._port))
        """
    def send_data(self, data):
        self._data = data
            
    def setup(self, ip_robot, port):
        self._ip = str(os.popen("hostname -I | cut -d\" \" -f1").readline().replace("\n",""))
        self._port = port
        self._ip_robot = ip_robot
        
    def stop(self):
        self._stopped.set()
        print("transmit stopped")
        self.join() #ждем завершения работы потока

class robot(threading.Thread):
    def __init__(self):
        super(robot, self).__init__()
        self.daemon = True
        self._stopped = threading.Event()
        self.data = None
        self._ip = None
        self._port = None
        self._timeout = 30
        
    def run(self):
        print("thread started\n")
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #создаем сервер
        server.bind((self._ip, self._port)) #запускаем сервер
        server.settimeout(self._timeout)#утсановка времени ожидания сообщения для сервера
        first_cicle = True


        while not self._stopped.is_set():
            self.data = None
            try:
                msg = server.recvfrom(1024) #пытаемся получить данные
                if first_cicle: #если первая иттерация, то записываем IP первого устройства, приславшего пакет с данными
                    self._ip_user = msg[1][0]
                    print("робот захвачен", self._ip_user)
                    first_cicle = False
                self.data = pl.loads(msg[0])
            except socket.timeout:
                print("time is out")
            except:
                print("broken data")
            time.sleep(0.01)
        """
        while not self._stopped.is_set():
            self.data = None
            try:
                msg = server.recvfrom(1024) #пытаемся получить данные
                if first_cicle: #если первая иттерация, то записываем IP первого устройства, приславшего пакет с данными
                    self._ip_user = msg[1][0]
                    print("робот захвачен", self._ip_user)
                    first_cicle = False
                data, crc = pl.loads(msg[0])
                crcNew = crc16.crc16xmodem(data)
                if crc == crcNew:
                    self.data = data
                else:
                    print("broken data")
            except socket.timeout:
                print("time is out")
            time.sleep(0.005)
            """
        server.close()
        
    def get_data(self):
        return self.data
        
    def setup(self, ip, port):
        self._ip = ip
        #self._ip = str(os.popen("hostname -I | cut -d\" \" -f1").readline().replace("\n",""))
        self._port = port
        self._ip_user = None
        
    def stop(self):
        self._stopped.set()
        print("transmit stopped")
        self.join() #ждем завершения работы потока

