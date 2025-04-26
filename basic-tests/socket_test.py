import socket

UGV_IP = "10.42.0.120"  # Update with actual UGV IP
PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UGV_IP, PORT))

print("Listening for messages from UAV...")
while True:
    data, addr = sock.recvfrom(1024)
    print(f"Received message: {data.decode()} from {addr}")