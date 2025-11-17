import os

def extract_code_blocks(dirs):
    blocks = []

    for d in dirs:
        for root, _, files in os.walk(d):
            for f in files:
                if f.endswith(".py"):
                    p = os.path.join(root, f)
                    with open(p, "r") as fp:
                        blocks.append((p, fp.read()))

    return blocks
