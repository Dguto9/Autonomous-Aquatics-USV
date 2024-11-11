import socket
import json
import csv
import tkinter as tk
import datetime

#This is the client-side python program which communicates with a Raspberry Pi, which relays messages sent from the vehicle

serverAddressPort = ("IP_ADDRESS_REDACTED", 8080)
bufferSize = 2048

#Settings packet format--specifies user so that the Pi can distinguish between client and vehicle messages
settings = {
    "type": "User",
    "speed": 50,
    "pid": [0.0, 0.0, 0.0]
} 

data = {}

def sendSettings():
    settings["speed"] = speedScale.get()
    settings["pid"][0] = pScale.get()
    settings["pid"][1] = iScale.get()
    settings["pid"][2] = dScale.get()
    UDPClientSocket.sendto(str.encode(json.dumps(settings)), serverAddressPort)

#Timestamp the CSV save
CSVout = open("C:/Users/dillo/OneDrive/Documents/Projects/Autonomous Aquatics/data-" + datetime.datetime.now().strftime("%m-%d-%Y-%H-%M-%S") + ".csv", 'w')
writer = csv.writer(CSVout)
writer.writerow(["latitude", "longitude", "temperature", "ph", "conductivity"])

# Create a UDP socket at client side
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPClientSocket.setblocking(0)
UDPClientSocket.sendto(str.encode(json.dumps(settings)), serverAddressPort)

#Create a grid-based Tkinter window to display the settings controls. The window is designed to be tall and of short width, so that it can be placed alongside a terminal
window = tk.Tk()
window.title("Autonomous Aquatics")
#window.resizable(0, 1)
control = tk.LabelFrame(window, text="Control")
speedLabel = tk.Label(control, text="Speed")
speedScale = tk.Scale(control, from_=0, to=100, orient="horizontal")
wpLabel = tk.Label(control, text="Waypoint")
wpScale = tk.Scale(control, from_=0, to=100, orient="horizontal")
tune = tk.LabelFrame(window, text="Tune")
pScale = tk.Scale(tune, from_=0, to=2, resolution=0.01)
iScale = tk.Scale(tune, from_=0, to=2, resolution=0.01)
dScale = tk.Scale(tune, from_=0, to=2, resolution=0.01)
send = tk.Button(window, text="Send",  command=sendSettings)
control.pack(side="top", anchor="nw", fill="x", expand=False)
speedLabel.pack(fill="x", expand=True)
speedScale.pack(fill="x", expand=True)
wpLabel.pack(fill="x", expand=True)
wpScale.pack(fill="x", expand=True)
tune.pack(side="top", anchor="nw", fill="both", expand=True)
pScale.pack(side="left", fill="both", expand=True)
iScale.pack(side="left", fill="both", expand=True)
dScale.pack(side="left", fill="both", expand=True)
send.pack(fill="x", expand=False)

#When data is recieved over UDP, write that data to a CSV file on the disk. This CSV is saved in three places: The vehicle SD card, the Rasperry Pi, and the Client laptop
def handleIncoming():
    try:
        bytesAddressPair = UDPClientSocket.recvfrom(bufferSize)
    except:
        window.after(50, handleIncoming)
        return
    message = bytesAddressPair[0].decode("ascii")
    data = json.loads(message)
    writer.writerow(data["coord"] + data["sensor"])
    CSVout.flush()
    print(data["coord"])
    window.after(50, handleIncoming)

handleIncoming()
window.mainloop()
