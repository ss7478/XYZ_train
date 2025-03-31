import requests as rq
from random import randint
import time

url = 'http://127.0.0.1:5000/data'

while True:
    resp = rq.post(url, json={"random_value:": str(randint(0, 100))})
    response = rq.get(url)
    print(response.json())
    time.sleep(0.5)
