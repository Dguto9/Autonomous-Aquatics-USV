import csv
import random

#Simple program to generate random latitude and longitude points for the purpose of testing QGIS data rendering

with open('testcoords.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Latitude', 'Longitude', 'Value'])

    for i in range(10000):
        writer.writerow([28.134261 + (random.random() - 0.5), -80.593747 + (random.random() - 0.5), random.random()*14])