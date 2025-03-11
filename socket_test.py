import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("Listening for messages...")
while True:
    data, addr = sock.recvfrom(1024)
    print(f"Received message: {data.decode()} from {addr}")