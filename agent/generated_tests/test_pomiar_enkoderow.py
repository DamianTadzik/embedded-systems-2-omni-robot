Aby wygenerowaæ testy jednostkowe dla podanego kodu, mo¿emy u¿yæ biblioteki `pytest`. Oto przyk³adowy plik `test_pomiar_kol.py` zawieraj¹cy kilka prostych testów:

```python
import pandas as pd
from io import StringIO

# Przygotowanie danych wejœciowych do testowania
input_data = """time_ms,TL,BL
0,1.5,-1.2
100,1.6,-1.3
200,1.7,-1.4"""

# Funkcja testuj¹ca czy dane zostaj¹ poprawnie wczytane
def test_load_data():
    df = pd.read_csv(StringIO(input_data))
    assert len(df) == 3
    assert "time_ms" in df.columns
    assert "TL" in df.columns
    assert "BL" in df.columns

# Funkcja testuj¹ca czy zmiany na kó³kach s¹ poprawne
def test_change_signs():
    df = pd.read_csv(StringIO(input_data))
    df["TL"] = -df["TL"]
    df["BL"] = -df["BL"]
    assert df.loc[0, "TL"] == -1.5
    assert df.loc[0, "BL"] == 1.2

# Funkcja testuj¹ca czy wyliczenia ró¿nic s¹ poprawne
def test_calculate_differences():
    df = pd.read_csv(StringIO(input_data))
    dt = df["time_ms"].diff() / 1000
    for wheel in ["TL", "BL"]:
        assert df[f"{wheel}_vel"] == df[wheel].diff() / dt
        assert df[f"{wheel}_rpm"] == (df[f"{wheel}_vel"] / 240) * 60

# Funkcja testuj¹ca czy wykres jest poprawnie generowany
def test_plot_generation():
    # Nie mo¿emy bezpoœrednio sprawdziæ, czy wykres zosta³ wygenerowany,
    # ale mo¿emy spróbowaæ najpierw wywo³aæ funkcjê i potem przekazaæ do niej StringIO
    from io import BytesIO
    import matplotlib.pyplot as plt

    df = pd.read_csv(StringIO(input_data))
    fig, ax = plt.subplots()
    for wheel in ["TL", "TR", "BL", "BR"]:
        ax.plot(df["time_ms"] / 1000, df[f"{wheel}_rpm"], label=wheel)
    output = BytesIO()
    plt.savefig(output)
    plt.close()

if __name__ == "__main__":
    import pytest
    pytest.main(["-v"])
```

W ten sposób mo¿emy przetestowaæ ró¿ne aspecty kodu. Pamiêtaj, ¿e nie jestem w stanie dok³adnie sprawdziæ, czy wykres zosta³ wygenerowany poprawnie, poniewa¿ biblioteka `matplotlib` wymaga interfejsu graficznego do wyœwietlenia wykresów. Mo¿na jednak przekazaæ wykres do pamiêci podrêcznej i potwierdziæ, ¿e nie powstaje b³¹d podczas jego generowania.