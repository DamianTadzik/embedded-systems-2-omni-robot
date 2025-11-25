import os
import sys
import time
from strip_comments import strip_comments
from code_parser import extract_code_blocks
from llm_backend import ask_llm
from test_generator import save_test
from clean_output import clean_ai_output

# TARGET_DIRS = ["../L1", "../L2"] 
TARGET_DIRS = ["../L1"]
LOG_PATH = os.path.join("generated_tests", "console.txt")


log_file = open(LOG_PATH, "w", encoding="utf8")
class Tee:
    def write(self, msg):
        sys.__stdout__.write(msg)   # normal print
        log_file.write(msg)         # write to log file
    def flush(self):
        sys.__stdout__.flush()
        log_file.flush()
sys.stdout = Tee()

def banner(msg: str):
    print("\n" + "=" * 70)
    print(msg)
    print("=" * 70)


def main():
    start_time = time.time()  

    banner("🚀 AI TEST GENERATION AGENT – STARTED")

    # 1) Locate Python files
    banner("📁 SCANNING TARGET DIRECTORIES")

    print("Target directories:")
    for d in TARGET_DIRS:
        print(f"  - {os.path.abspath(d)}")

    blocks = extract_code_blocks(TARGET_DIRS)

    if not blocks:
        print("\n❌ No Python files found. Nothing to do.")
        return

    print("\nFound Python files:")
    for fp, _ in blocks:
        print(f"  ✓ {fp}")

    # 2) Process each file
    banner("🧪 GENERATING TESTS")

    for file_path, code in blocks:
        print(f"\n→ Processing file: {file_path}")

        clean = strip_comments(code)
        print("  - Comments removed")
        print("  - Code cleaned")

        prompt = f"""
You are an expert Python test generator.

Analyze the following Python code and generate a complete PyTest file
that tests all functions found in the code.

Requirements:
- Output ONLY valid Python code.
- The test file must contain multiple test cases covering normal,
  edge-case and invalid inputs where applicable.
- Do NOT include explanations, comments or markdown.
- Use simple, direct and deterministic tests.

Code to analyze:
{clean}

Generate the full pytest file now:
"""

        print("  - Sending code to AI model...")

        try:
            out = ask_llm(prompt)
            print("  - AI response received")
            cleaned = clean_ai_output(out)
            print("  - Cleaned AI output")
        except Exception as e:
            print(f"  ❌ ERROR: AI model failed on file {file_path}")
            print(f"    Reason: {e}")
            continue

        save_test(file_path, cleaned)
        print(f"  ✓ Saved cleaned tests → test_{os.path.basename(file_path)}.py")

    # 3) Finish
    banner("✅ AI TEST GENERATION COMPLETE")

    elapsed = time.time() - start_time
    print(f"Total time: {elapsed:.2f} seconds\n")

    print("📌 Generated test files:")
    for fp, _ in blocks:
        test_name = f"test_{os.path.basename(fp).replace('.py', '')}.py"
        print(f"   - agent/generated_tests/{test_name}")

    print("\n📘 HOW TO RUN TESTS")
    print("-----------------------------------------------------------")
    print("Run all generated tests:")
    print("   python -m pytest agent/generated_tests -q\n")
    print("   pytest agent/generated_tests -q\n")
    print("Run tests with full output:")
    print("   pytest agent/generated_tests\n")
    print("Run a single test file:")
    print("   pytest agent/generated_tests/test_<filename>.py -q\n")
    print("\nDone.\n")


if __name__ == "__main__":
    main()
