Oto jeden test jednostkowy dla klasy PIDController w formacie pytest:

```python
import pytest
from your_module import PIDController

@pytest.fixture
def pid_controller():
    return PIDController(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)

def test_compute_with_error(pid_controller):
    error = 2.0
    dt = 1.0
    expected_output = (pid_controller.Kp * error) + pid_controller.Ki * error * dt + pid_controller.Kd * (error - pid_controller.previous_error) / dt
    output = pid_controller.compute(error, dt)
    assert pytest.approx(output, abs=1e-6) == expected_output

def test_compute_with_no_error(pid_controller):
    error = 0.0
    dt = 1.0
    expected_output = 0
    output = pid_controller.compute(error, dt)
    assert pytest.approx(output, abs=1e-6) == expected_output

def test_saturation_values():
    rotate_saturation_value = rotate_saturation(2.5)
    move_forward_saturation_value = move_forawrd_saturation(1.5)
    
    assert rotate_saturation_value == SAT_TH_ANGLE
    assert move_forward_saturation_value == SAT_TH_DISTANCE

def test_mqtt_on_message(mocker):
    # Mocking the json.loads to simulate message payload
    mocker.patch('your_module.json.loads', return_value={"Cx": 100, "Cy": 200, "distance": 0.5})
    
    global distance, Cx, Cy
    mqtt_on_message(None, None, {"payload": '{"Cx": 100, "Cy": 200, "distance": 0.5}'})

    assert distance == 0.5
    assert Cx == 100
    assert Cy == 200
```

W pliku `your_module.py` powinieneœ umieœciæ kod, który ma byæ testowany (w tym przypadku klasy PIDController i funkcje rotate_saturation oraz move_forawrd_saturation). 

Ten test sprawdza:
1. Dzia³aæ poprawnie metoda `compute` klasy `PIDController`.
2. Czy wartoœci przekroczone przez próg saturacyjny s¹ ograniczane.
3. Funkcje saturation dla ruchu i obrotu.

Pamiêtaj, ¿e musisz zamieniæ `"your_module"` na rzeczywiste nazwê modu³u, w którym znajduje siê twój kod do testowania.