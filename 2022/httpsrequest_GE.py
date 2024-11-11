import requests
import time
import simplekml

#Combines the functionality of requesting the vehicle coordinates over HTTP and plotting them in Google Earth

kml = simplekml.Kml()
coordinates = []
cMem = (0, 0)

def dm(x):
    degrees = int(x) // 100
    minutes = x - 100*degrees

    return degrees, minutes

def decimal_degrees(degrees, minutes):
    return degrees + minutes/60

path = kml.newlinestring(name="Path")
path.style.linestyle.width = 5
path.style.linestyle.color = simplekml.Color.red
end = kml.newpoint(name="EndPoint")
end.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/shapes/target.png'
end.style.labelstyle.scale = 0

while True:

    o = requests.get('http://192.168.4.1/O')
    oCoord = -decimal_degrees(*dm(float(o.text)))

    print(oCoord)
    
    a = requests.get('http://192.168.4.1/A')

    aCoord = decimal_degrees(*dm(float(a.text)))

    print(aCoord)

    if (oCoord, aCoord) != cMem:

        coordinates.append((oCoord, aCoord))

    print(coordinates)

    cMem = (oCoord, aCoord)

    path.coords = coordinates
    end.coords = [(oCoord, aCoord)]

    kml.save("GoogleEarthPython.kml")

    time.sleep(4)
