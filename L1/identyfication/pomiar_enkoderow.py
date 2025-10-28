import pandas as pd
import matplotlib.pyplot as plt

# Wczytaj dane z pliku
df = pd.read_csv("pomiar_kol.csv")

# Zmieniamy znak dla kół TL i BL
df["TL"] = -df["TL"]
df["BL"] = -df["BL"]

# Policzenie różnic (delta impulsów) i prędkości
dt = df["time_ms"].diff() / 1000  # sekundy
for wheel in ["TL", "TR", "BL", "BR"]:
    df[f"{wheel}_vel"] = df[wheel].diff() / dt  # impulsów/s
    df[f"{wheel}_rpm"] = (df[f"{wheel}_vel"] / 240) * 60  # przeliczenie na obr./min

# Wykres prędkości w czasie
plt.figure()
for wheel in ["TL", "TR", "BL", "BR"]:
    plt.plot(df["time_ms"]/1000, df[f"{wheel}_rpm"], label=wheel)
plt.xlabel("Czas [s]")
plt.ylabel("Prędkość [obr/min]")
plt.legend()
plt.title("Charakterystyka przyspieszania kół")
plt.show()
