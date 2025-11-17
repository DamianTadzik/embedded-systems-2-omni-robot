import requests

def ask_llm(prompt: str) -> str:
    url = "http://localhost:11434/api/chat"

    payload = {
        "model": "qwen2.5-coder:7b",
        "stream": False,   # <---- KLUCZOWA LINIJKA
        "messages": [
            {"role": "user", "content": prompt}
        ]
    }

    r = requests.post(url, json=payload)
    r.raise_for_status()

    data = r.json()

    # Ollama zwraca strukturę: { "message": { "role": "...", "content": "..."} }
    return data["message"]["content"]
