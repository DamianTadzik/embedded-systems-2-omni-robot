import json
import os
from concept_creator.readme_loader import load_readmes
from utilities.clean_output import clean_ai_output
from llm_backend import ask_llm

CONCEPT_JSON = "generated/concept.json"

def generate_concept():
    readmes = load_readmes()

    combined_text = ""
    for path, content in readmes:
        combined_text += f"\n# FILE: {path}\n{content}\n"

    prompt = f"""
You are an expert in embedded systems testing.

Your job:
1. Read the system documentation below (multiple README files).
2. Extract a clean overview of the system architecture.
3. Identify the most important elements of L1 and L2 layer.
4. Propose a structured, comprehensive list of test concepts for L1 and L2 layer.

Format output STRICTLY as JSON with keys:
{{
  "system_overview": "...",
  "components": {{
    "L1": {{
      "tests": ["...", "...", "..."]
    }},
    "L2": {{
      "tests": ["...", "...", "..."]
    }}
  }}
}}

System documentation:
{combined_text}
"""

    raw = ask_llm(prompt, json_mode=True)
    cleaned = clean_ai_output(raw)

    # Parse JSON safely
    try:
        data = json.loads(cleaned)
    except Exception:
        print("⚠️ LLM output was not valid JSON. Saving raw output.")
        data = {"raw_output": cleaned}

    with open(CONCEPT_JSON, "w", encoding="utf8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"✅ Concept written to {CONCEPT_JSON}")
    return data
