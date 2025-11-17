Oto przyk³ad jednego testu jednostkowego dla funkcji `omni4_inverse_kinematics`:

```python
import pytest
import numpy as np

from your_script_name import omni4_inverse_kinematics  # Zast¹p 'your_script_name' nazw¹ pliku zawieraj¹cego funkcjê

def test_omni4_inverse_kinematics():
    # Dane wejœciowe
    vx = 0.2
    vy = -0.1
    omega = 0.5
    v_max = 0.3
    omega_max = 10.0

    # Oczekiwane dane wyjœciowe (przyk³adowe, mog¹ byæ uzupe³nione na podstawie poprawnej implementacji)
    expected_wheel_cmds = [(58, 1), (64, 1), (217, 0), (163, 0)]
    expected_wheel_omegas = np.array([vx, vy, omega])

    # Wywo³anie funkcji
    wheel_cmds, wheel_omegas = omni4_inverse_kinematics(vx, vy, omega, v_max, omega_max)

    # Sprawdzenie poprawnoœci wyników
    assert len(wheel_cmds) == 4, "Funkcja powinna zwróciæ 4 pary (pwm, direction)"
    for pwm, direction in wheel_cmds:
        assert isinstance(pwm, int), "PWM powinien byæ liczb¹ ca³kowit¹"
        assert direction in [0, 1], "Kierunek powinien byæ 0 lub 1"

    np.testing.assert_almost_equal(wheel_omegas, expected_wheel_omegas, decimal=6)

if __name__ == "__main__":
    pytest.main()
```

Aby uruchomiæ ten test, zapisaæ go w pliku (np. `test_omni4_inverse_kinematics.py`) i uruchomiæ go za pomoc¹ narzêdzia `pytest`, np.:

```sh
pip install pytest
pytest test_omni4_inverse_kinematics.py
```

Ten test sprawdza, czy funkcja `omni4_inverse_kinematics` zwraca poprawne pary (pwm, direction) oraz k¹towe prêdkoœci dla danych wejœciowych. Pamiêtaj, ¿e oczekiwane dane wyjœciowe (`expected_wheel_cmds` i `expected_wheel_omegas`) mog¹ musieæ byæ dostosowane do poprawnej implementacji funkcji.