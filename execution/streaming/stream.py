import subprocess
import sys
from pathlib import Path

STREAMING_DIR = Path(__file__).parent
START = STREAMING_DIR / "start_stream.sh"
STOP = STREAMING_DIR / "stop_stream.sh"


def main():
    action = input("Start or stop stream? (start/stop): ").strip().lower()
    if action == "start":
        subprocess.Popen(
            [str(START)],
            stdin=subprocess.DEVNULL, stdout=sys.stdout, stderr=sys.stderr
        )
    elif action == "stop":
        subprocess.run([str(STOP)], stdin=subprocess.DEVNULL)
    else:
        print("Invalid input. Type 'start' or 'stop'.")


if __name__ == "__main__":
    main()
