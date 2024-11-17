import requests
import time

url = "http://192.168.43.245:5001/data"

while True:
    response = requests.get(url)
    data = response.json()
    print("Received data:", data)
    time.sleep(1)  # Polling setiap detik
