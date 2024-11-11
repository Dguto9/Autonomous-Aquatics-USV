import simplekml

#Test plotting a set of coordinates as a path in Google Earth

coordinates=[(REDACTED_COORDINATE, REDACTED_COORDINATE), (REDACTED_COORDINATE, REDACTED_COORDINATE), (REDACTED_COORDINATE, REDACTED_COORDINATE), (REDACTED_COORDINATE, REDACTED_COORDINATE)]
kml = simplekml.Kml()
for i in range (4):
    print(coordinates[i])
    pnt = kml.newpoint(name="Waypoint " + str(i), coords=[coordinates[i]])
    pnt.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png'
kml.newlinestring(name="Path", coords=coordinates)
kml.save("GoogleEarthPython.kml")

 
