import simplekml
import math

#Plots a set of coordinates in meters from a reference point in Google Earth, in order to visualize the path

earthRadius = 6366000

ref_lat = REDACTED_COORDINATE
ref_lon = REDACTED_COORDINATE

meterCoords = [(-1, 3), (5, 9), (5, 15)]
coordinates = []

def coordConvert(coords):
    dx = coords[0]
    dy = coords[1]

    k = math.cos(math.radians(ref_lat))

    z = math.radians(earthRadius)
    
    dlon = dx / z * k
    dlat = dy / z

    lon = dlon + ref_lon
    lat = dlat + ref_lat

    return (lon, lat)

for i in range (len(meterCoords)):
    coordinates.append(coordConvert(meterCoords[i]))
    print(coordinates[i])
    
kml = simplekml.Kml()
for i in range (len(coordinates)):
    pnt = kml.newpoint(name="Wpnt " + str(i), coords=[coordinates[i]])
    pnt.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png'
kml.newlinestring(name="Waypoint Line", coords=coordinates)
kml.save("GoogleEarthPython.kml")
    

