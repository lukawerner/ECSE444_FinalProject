import socket
import threading
from server.message_parcer import parse_message

class TCPServer(threading.Thread):

    def __init__(self, host, port, queue):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.queue = queue

    def run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # FIX
            s.bind((self.host, self.port))
            s.listen()

            print(f"Listening on {self.host}:{self.port}")
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break

                    msg = data.decode().strip()
                    parsed = parse_message(msg)
                    print("Received:", parsed)

                    self.queue.put(parsed)
