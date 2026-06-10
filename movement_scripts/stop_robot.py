import socket
import struct
import threading
import queue
import time
import csv
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

# Constants
PORT = 30003
HOST = "192.168.0.153"


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print("Connecting...")
        s.connect((HOST, PORT))
        print("Connected.")

        cmd = (
            "def my_program():\n"
            "  stopl(1.2)\n"
            "end\n"
            "my_program()\n"
        )
        
        print(cmd)
        s.sendall(cmd.encode('ascii'))



if __name__ == "__main__":
    main()
