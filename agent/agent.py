import os
from code_parser import extract_code_blocks
from llm_backend import ask_llm
from test_generator import save_test

TARGET_DIRS = ["../L1", "../L2"]

def main():
    code_blocks = extract_code_blocks(TARGET_DIRS)

    for file_path, code in code_blocks:
        prompt = f"""
Przeanalizuj poniższy kod Pythona i wygeneruj testy jednostkowe w formacie pytest.

Kod:
{code}

Wygeneruj JEDEN plik testowy w Pythonie.
"""
        test_code = ask_llm(prompt)
        save_test(file_path, test_code)
        print(f"[AI] Test generated for: {file_path}")

    print("=== DONE ===")
    print("Run tests: pytest agent/generated_tests -q")


if __name__ == "__main__":
    main()
