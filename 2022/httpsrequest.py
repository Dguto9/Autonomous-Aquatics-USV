import requests
import time

#Test of the HTTP request system, later to be combined with the Google Earth plotting program

def dm(x):
    degrees = int(x) // 100
    minutes = x - 100*degrees

    return degrees, minutes

def decimal_degrees(degrees, minutes):
    return degrees + minutes/60 

while True:

    o = requests.get('http://192.168.4.1/O')

    print(decimal_degrees(*dm(float(o.text))))
    
    a = requests.get('http://192.168.4.1/A')

    print(decimal_degrees(*dm(float(a.text))))

    time.sleep(2);
