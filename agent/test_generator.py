import os

OUTPUT_DIR = "generated_tests"

def ensure_dir():
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

def save_test(origin_file, content):
    ensure_dir()
    base = os.path.basename(origin_file).replace(".py", "")
    out_path = os.path.join(OUTPUT_DIR, f"test_{base}.py")
    with open(out_path, "w") as f:
        f.write(content)
