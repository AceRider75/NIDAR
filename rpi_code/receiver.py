import socket
import json

HOST = "127.0.0.1"
PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST, PORT))
sock.listen(1)

print("Waiting for spot tracker...")
conn, addr = sock.accept()
print("Spot tracker connected:", addr)

buffer = b""

while True:
    data = conn.recv(4096)
    if not data:
        break

    buffer += data

    while b"\n" in buffer:
        line, buffer = buffer.split(b"\n", 1)
        packet = json.loads(line.decode())

        if packet["type"] == "yellow_spots":
            # handle_spots(packet)
            print("Received yellow spots:", packet["spots"])
