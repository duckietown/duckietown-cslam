#!/usr/bin/env python

import socket
import errno
from socket import error as socket_error

class socketServer():
    def __init__(self, host='127.0.0.1', port=65432):
        while True:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.bind((host, port))
                self.sock.listen(5)
                break
            except Exception as e:
                print("Couldn't bind socket server port:  %s", str(e))

    def waitAndGetData(self):
        try:
            conn, addr = self.sock.accept()
            print('Connected by', addr)
            allData = b''
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                allData = allData + data
            return allData
        except KeyboardInterrupt:
            self.sock.close()
            raise( Exception("Exiting") )
        except:
            pass

class socketClient():
    def __init__(self, host='127.0.0.1', port=65432):
        self.host = host
        self.port = port

    def submitData(self, data):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            self.sock.sendall(data)
        except KeyboardInterrupt:
            self.sock.close()
            raise( Exception("Exiting") )
        except:
            pass
