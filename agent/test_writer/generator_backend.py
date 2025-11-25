def build_test_prompt(module_name: str, code: str, concept_tests: list):
    concept_str = "\n".join(f"- {t}" for t in concept_tests)

    return f"""
You are an expert Python test engineer.

Your job is to generate a CORRECT pytest test file for module: {module_name}

Before you generate tests, you MUST analyze the code carefully.

### PHASE 1 — Extract function signatures
List ALL functions defined in the module, with:
- function name
- full parameter list (including defaults)
- whether it returns a value

You MUST NOT:
- guess hidden functions
- invent parameters
- assume behavior that is not visible

### PHASE 2 — Generate tests ONLY for functions that exist
Use the following constraints:

- Use ONLY functions found in Phase 1.
- Use ONLY arguments that match the actual signatures.
- Do NOT create dummy imports except:
    `import pytest`
    `from {module_name} import <functions>`
- Do NOT use classes, mocks, helpers, or utilities that do not exist.
- Do NOT write explanations or comments — output ONLY Python code.
- Tests must be deterministic and simple.
- Follow high-level test ideas from the concept:
{concept_str}

### CODE TO ANALYZE:
{code}

### FINAL REQUIREMENT:
Output ONLY a complete pytest file and nothing else.
"""
