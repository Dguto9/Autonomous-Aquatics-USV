import simplekml

#Test plotting decimal degree coordinates within Google Earth

ref_lat = REDACTED_COORDINATE
ref_lon = REDACTED_COORDINATE

coordinates = [(ref_lon, ref_lat)]

kml = simplekml.Kml()

waypoints = kml.newlinestring(name="waypoints", coords=coordinates)

coords = waypoints.coords

print(coords)
 
kml.save("GoogleEarthPython.kml")
