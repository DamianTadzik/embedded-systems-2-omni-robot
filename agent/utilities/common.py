import sys
import os

# ---------- Logger with Tee (console + log file) ----------
class Tee:
    def __init__(self, filepath):
        self.file = open(filepath, "w", encoding="utf8")

    def write(self, msg):
        sys.__stdout__.write(msg)
        self.file.write(msg)

    def flush(self):
        sys.__stdout__.flush()
        self.file.flush()

def setup_logger(path="agent_log.txt"):
    sys.stdout = Tee(path)

# ---------- Banner ----------
def banner(msg: str):
    print("\n" + "=" * 70)
    print(msg)
    print("=" * 70)

# ---------- File helpers ----------
def read_text(path: str) -> str:
    with open(path, "r", encoding="utf8") as f:
        return f.read()

def safe_listdir(path: str):
    try:
        return os.listdir(path)
    except:
        return []
