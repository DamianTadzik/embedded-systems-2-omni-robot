import os

TARGET_DIRS = [
    "..",      # root
    "../L1",
    "../L2",
    "../L3",
    "../LL",
    # ".",       # agent/
]

def load_readmes():
    readmes = []
    for d in TARGET_DIRS:
        full = os.path.abspath(d)
        items = os.listdir(full)

        for name in items:
            if name.lower().startswith("readme"):
                path = os.path.join(full, name)
                try:
                    with open(path, "r", encoding="utf8") as f:
                        readmes.append((path, f.read()))
                except:
                    pass
    return readmes
