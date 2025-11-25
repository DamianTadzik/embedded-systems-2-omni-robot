import os

OUTPUT_DIR = "generated/tests"

def ensure_dir():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

def save_test(module_name, content):
    ensure_dir()
    out_path = os.path.join(OUTPUT_DIR, f"test_{module_name}.py")
    with open(out_path, "w", encoding="utf8") as f:
        f.write(content)
