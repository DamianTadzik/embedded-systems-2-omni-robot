import os

def extract_code_blocks(dirs):
    blocks = []

    for directory in dirs:
        # normalize path
        directory = os.path.abspath(directory)

        # get all items directly inside directory
        for name in os.listdir(directory):
            full_path = os.path.join(directory, name)

            # ignore subdirectories
            if os.path.isdir(full_path):
                continue

            # take only top-level python files
            if name.endswith(".py"):
                with open(full_path, "r", encoding="utf8") as fp:
                    code = fp.read()
                blocks.append((full_path, code))

    return blocks
