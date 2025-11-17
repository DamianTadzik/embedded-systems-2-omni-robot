import requests

def ask_llm(prompt: str) -> str:
    """
    Wywołuje lokalny model przez Ollama HTTP API.
    Zakładamy, że działa domyślny serwer na http://localhost:11434
    i masz model 'llama3.1' pobrany.
    """
    url = "http://localhost:11434/api/chat"
    payload = {
        "model": "qwen2.5-coder:7b",
        "messages": [
            {"role": "user", "content": prompt}
        ]
    }

    resp = requests.post(url, json=payload)
    resp.raise_for_status()
    data = resp.json()
    return data["message"]["content"]