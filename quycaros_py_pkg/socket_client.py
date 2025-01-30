import socket

HOST = "localhost"  # The server's hostname or IP address
PORT = 12345  # The port used by the server

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(b"get cam_state")
    data = s.recv(1024)

print(f"Received {data!r}")