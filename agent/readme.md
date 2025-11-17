# 1. Wymagania wstępne

Sprzęt (sensownie):

- CPU x86_64 (normalny PC/laptop)
- RAM: min. 8 GB (12–16 GB będzie dużo przyjemniejsze)
- Opcjonalnie GPU z obsługą CUDA (NVIDIA) – przyspieszy, ale nie jest konieczne

System:

- Windows 10 lub 11 (64-bit)

# 2. Instalacja Ollama na Windows

Wejdź na stronę:  
👉 https://ollama.com/download

Pobierz instaler dla Windows (`OllamaSetup.exe`) ~1.1 GB.

Uruchom instalator:

Dalej → Dalej → Zainstaluj (standard, bez kombinacji).

Po instalacji powinno Ci się pojawić Ollama w menu Start.

Po instalacji zrestartuj terminal / PowerShell, bo instalator dodaje Ollama do `PATH`.

# 3. Sprawdzenie, czy Ollama działa

Otwórz PowerShell albo Windows Terminal.

```powershell
ollama --version
```

# 4. Pobranie modelu (np. Llama 3.1)

Polecam na start:

```bash
ollama pull llama3.1
```

To ściągnie model (kilka GB, więc chwila zejdzie).

Możesz też coś mniejszego, np.:

```bash
ollama pull llama3.1:8b
```

Albo coś do rozumienia kodu:

```bash
ollama pull qwen2.5-coder:7b
```

# 5. Test interaktywny (czy model gada)

```bash
ollama run llama3.1
```

Powinno odpalić interaktywną konsolę, gdzie możesz pisać pytania.  
Wyjście: Ctrl + C.

Jeśli to działa → masz lokalnego LLM gotowego do użycia.

# 6. Użycie Ollama z Pythona (pod AI agenta)

W twoim projekcie (tam gdzie będzie agent/), zainstaluj `requests`:

```bash
pip install requests
```

W folderze `agent/` zrób plik `llm_backend.py`:

```python
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
```

Ollama automatycznie wystawia API na `http://localhost:11434`, jak tylko wywołujesz `ollama run` lub gdy Python zrobi request – daemon się sam odpali w tle (jak jest zainstalowany poprawnie).

# 7. Prosty smoke-test z Pythona

Zrób plik `test_ollama.py` obok `llm_backend.py`:

```python
from llm_backend import ask_llm

if __name__ == "__main__":
    out = ask_llm("Napisz jedno zdanie po polsku, że działasz.")
    print(out)
```

Odpal:

```bash
python agent/test_ollama.py
```

Jeśli wypisze sensowne zdanie → AI backend działa.  
Potem tylko podpinasz to pod `agent.py`, który będzie czytał kod i prosił model o generowanie testów.
