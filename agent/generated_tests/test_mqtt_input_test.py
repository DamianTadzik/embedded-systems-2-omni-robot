Dla przetestowania tego kodu mo¿emy u¿yæ pytest oraz mockupów dla modu³ów `paho.mqtt.client` i `time`. Poni¿ej znajduje siê przyk³adowy test jednostkowy:

```python
import time
from unittest.mock import patch, MagicMock
from your_script_name import mqttc, MQTT_PUBLISH_TOPIC, publish_data  # Zast¹p "your_script_name" na rzeczywist¹ nazwê pliku

@patch('time.time', return_value=1633072800)
@patch('json.dumps')
@patch('paho.mqtt.client.Client.publish')
def test_publish_data(mock_publish, mock_dumps, mock_time):
    # Przygotowanie danych wejœciowych
    Cx = 1.0
    Cy = 2.0
    distance = 3.0
    expected_data = {"Cx": Cx, "Cy": Cy, "distance": distance}
    
    # Wywo³anie funkcji do testowania
    publish_data(Cx, Cy, distance)
    
    # Sprawdzenie czy metoda 'publish' zosta³a wywo³ana poprawnie
    mock_publish.assert_called_once_with(MQTT_PUBLISH_TOPIC, mock_dumps.return_value, 0)
    
    # Sprawdzenie czy funkcja 'json.dumps' zosta³a wywo³ana z odpowiednimi argumentami
    mock_dumps.assert_called_once_with(expected_data, separators=(',', ':'))

# Pamiêtaj, ¿e musisz dostosowaæ nazwê pliku i nazwy funkcji do Twojego kodu.
```

W powy¿szym testzie u¿yliœmy `@patch` z biblioteki `unittest.mock`, aby zamockowaæ metody `time.time()`, `json.dumps()` oraz `paho.mqtt.client.Client.publish()`. Nastêpnie wywo³ujemy funkcjê `publish_data(Cx, Cy, distance)` i sprawdzamy, czy metoda `publish` zosta³a poprawnie wywo³ana z odpowiednimi argumentami. W przypadku zmian w Twoim kodzie, pamiêtaj o dostosowaniu nazw modu³ów, klas i funkcji do swojego programu.