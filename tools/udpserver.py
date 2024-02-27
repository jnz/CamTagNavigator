import socket

def parse_message(data):
    """
    Parses the incoming message and returns a dictionary with the parsed data.
    """
    # Split the received message by comma
    parts = data.decode().split(', ')
    if len(parts) != 9:
        print("Invalid message format")
        return None

    # Extract and convert each part of the message
    message = {
        'timestamp_sec': float(parts[0]),
        'udp_message_counter': int(parts[1]),
        'position': {
            'x': float(parts[2]),
            'y': float(parts[3]),
            'z': float(parts[4])
        },
        'quaternion': {
            'w': float(parts[5]),
            'x': float(parts[6]),
            'y': float(parts[7]),
            'z': float(parts[8])
        }
    }

    return message

def start_server(port=5876):
    """
    Starts a UDP server listening on the given port.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', port))
    print(f"Listening on port {port}...")

    while True:
        data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
        message = parse_message(data)
        if message:
            print("Received message:", message)

if __name__ == "__main__":
    start_server()

