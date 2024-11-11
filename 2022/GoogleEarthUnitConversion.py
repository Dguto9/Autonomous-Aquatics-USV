import simplekml
import math

#The navigation system of this iteration uses coordinates representing meters from an origin point; this program tests the mathematics behind converting from decimal degree to meters from an origin

earthRadius = 6366000

ref_lat = REDACTED_COORDINATE
ref_lon = REDACTED_COORDINATE

meterCoords = []
coordinates = [(REDACTED_COORDINATE, REDACTED_COORDINATE), (REDACTED_COORDINATE, REDACTED_COORDINATE), (REDACTED_COORDINATE, REDACTED_COORDINATE), (REDACTED_COORDINATE, REDACTED_COORDINATE), (REDACTED_COORDINATE, REDACTED_COORDINATE)]

 
def coordConvert(coords):
    lon = coords[0]
    lat = coords[1]

    
    dlat = lat - ref_lat
    dlon = lon - ref_lon

    k = math.cos(math.radians(ref_lat))

    z = math.radians(earthRadius)

    dx = dlon * z * k
    dy = dlat * z

    return (dx, dy)

for i in range (len(coordinates)):
    meterCoords.append(coordConvert(coordinates[i]))

    
kml = simplekml.Kml()
for i in range (len(coordinates)):
    pnt = kml.newpoint(name="Waypoint " + str(i), coords=[coordinates[i]])
    pnt.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png'
kml.save("GoogleEarthPython.kml")
    
print(meterCoords)
