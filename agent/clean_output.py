import re

def clean_ai_output(text: str) -> str:
    text = re.sub(r"```python", "", text, flags=re.IGNORECASE)
    text = re.sub(r"```", "", text)
    text = text.strip()
    lines = [line.rstrip() for line in text.splitlines() if line.strip() != ""]
    return "\n".join(lines)
