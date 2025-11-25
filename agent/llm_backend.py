import requests

def ask_llm(prompt: str, json_mode=False) -> str:
    payload = {
        "model": "qwen2.5-coder:7b",
        "stream": False,
        "messages": [
            {"role": "system", "content": """
You are a reliable and precise senior engineer.
Your job:
- think step by step
- avoid hallucinations
- produce deterministic, structured output
- follow the required output format strictly
"""},
            {"role": "user", "content": prompt}
        ],
        "options": {
            "temperature": 0,
            "top_p": 1,
            "top_k": 20,
            "repeat_penalty": 1.1,
        }
    }

    if json_mode:
        payload["format"] = "json"

    r = requests.post("http://localhost:11434/api/chat", json=payload)
    r.raise_for_status()
    data = r.json()

    # SAFE FALLBACK
    return data.get("message", {}).get("content", "")
