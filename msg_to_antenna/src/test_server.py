#!/usr/bin/env python

import socket

HOST = '10.42.0.30'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
print("Waiting for connection")
s.listen(1)
(conn, addr) = s.accept()
print("Connection established")
while True:
    data = conn.recv(1024)
    if not data: break
    print(data)
conn.close()