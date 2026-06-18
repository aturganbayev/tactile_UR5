import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import socket
import struct
import threading
import queue
import time
import csv
import numpy as np
from scipy.spatial.transform import Rotation as R
from pose_utils import REAL_HOST

# Constants
PORT = 30003
HOST = REAL_HOST


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print("Connecting...")
        s.connect((HOST, PORT))
        print("Connected.")

        cmd = (
            "def my_program():\n"
            "  stopl(2.5)\n"
            "end\n"
            "my_program()\n"
        )
        
        print(cmd)
        s.sendall(cmd.encode('ascii'))



if __name__ == "__main__":
    main()
