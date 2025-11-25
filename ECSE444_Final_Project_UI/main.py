import queue
import threading
import time
import tkinter as tk
from server.tcp_server import TCPServer
from ui.radar_ui import RadarUI

HOST = "192.168.2.12" #"192.168.2.234" #'192.168.1.100' # This is PC's ip address
PORT = 8080 # Fix on both sides to be 8080


def mock_data_thread(q):
    import random
    while True:
        for i in range(120):
            q.put({'type': 'temperature', 'temperature': random.randint(15, 30)})
            if i == 60:
                q.put({'type': 'intruder', 'angle': i + 30, 'distance': 20})
            else:
                q.put({'type': 'normal', 'angle': i + 30})
            time.sleep(0.1)
        for i in range(120):
            q.put({'type': 'temperature', 'temperature': random.randint(15, 30)})
            q.put({'type': 'normal', 'angle': 120 - i + 30})
            time.sleep(0.1)


if __name__ == "__main__":
    q = queue.Queue()
    server = TCPServer(HOST, PORT, q) # TCP messages fills out the queue. Comment this out to use mock data above
    server.start()

    root = tk.Tk()

    #threading.Thread(target=mock_data_thread, args=(q,), daemon=True).start()
    app = RadarUI(root, q)
    root.mainloop()
