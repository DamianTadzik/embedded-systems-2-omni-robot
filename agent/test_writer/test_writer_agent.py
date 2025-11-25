import json
import os

from utilities.code_parser import extract_code_blocks
from utilities.strip_comments import strip_comments
from utilities.clean_output import clean_ai_output
from llm_backend import ask_llm
from test_writer.generator_backend import build_test_prompt
from test_writer.test_generator import save_test


CONCEPT_PATH = "generated/concept.json"
TARGET_DIRS = ["../L1"]


def write_tests():
    # Load concept
    if not os.path.exists(CONCEPT_PATH):
        raise FileNotFoundError("❌ concept.json missing — run concept generator first.")

    with open(CONCEPT_PATH, "r", encoding="utf8") as f:
        concept = json.load(f)

    # Load code from L1
    blocks = extract_code_blocks(TARGET_DIRS)
    if not blocks:
        print("❌ No Python files found in L1.")
        return

    print("📦 Loaded concept and source code. Starting test generation...")

    for file_path, code in blocks:
        module_name = os.path.basename(file_path).replace(".py", "")

        layer_tests = concept.get("components", {}).get("L1", {}).get("tests", [])

        prompt = build_test_prompt(
            module_name=module_name,
            # code=strip_comments(code),
            code=code,
            concept_tests=layer_tests,
        )

        print(f"\n🧪 Generating tests for: {module_name}")

        out = ask_llm(prompt)
        cleaned = clean_ai_output(out)

        save_test(module_name, cleaned)

        print(f"   ✓ Tests saved for {module_name}")

    print("\n✅ Test generation finished.")
