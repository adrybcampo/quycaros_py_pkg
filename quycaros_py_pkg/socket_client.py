import socket
import sys

HOST = "localhost"  # The server's hostname or IP address
PORT = 12345  # The port used by the server

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    print("Waiting for confirmation...")
    data = s.recv(1024).decode('utf-8')
    while True:
        instr = input("Enter instruction: ")
        s.sendall(instr.encode('utf-8'))
        print("Instruction sent: ",instr)
        data = s.recv(1024).decode('utf-8')
        while (data == 'ping'):
            data = s.recv(1024).decode('utf-8')
        print("Received: ",str(data))