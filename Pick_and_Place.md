import socket
import threading
import time
import cri_reciver
import json

# Establishing TCP/IP connection with ReBeL-RPi
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Current and target joint positions
pos_joint_current = {"ax1": 0.0, "ax2": 0.0, "ax3": 0.0, "ax4": 0.0, "ax5": 0.0, "ax6": 0.0}
pos_joint_target = {"ax1": 0.0, "ax2": 0.0, "ax3": 0.0, "ax4": 0.0, "ax5": 0.0, "ax6": 0.0}

# Robot control parameters
ROBOT_CONTROL_ADDRESS = ("127.0.0.1", 3920)
CRI_ALIVEJOG = "CRISTART 1000 ALIVEJOG 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 CRIEND"

print(f"INFO: Connecting to host {ROBOT_CONTROL_ADDRESS[0]} at port {ROBOT_CONTROL_ADDRESS[1]}")
sock.connect(ROBOT_CONTROL_ADDRESS)
print("INFO: Connection successful!")

# Position tracking (object memory)
object_positions = {
    "pos1": True,
    "pos2": True,
    "pos3": True,
    "posa": False,
    "posb": False,
    "posc": False
}

# Joint positions for each location
positions = {
    "pos1": {"ax1": -32.2, "ax2": 65.5, "ax3": 55.9, "ax4": 0.0, "ax5": 58.6, "ax6": -32.2},
    "pos2": {"ax1": -40.1, "ax2": 49.2, "ax3": 86.1, "ax4": 0.0, "ax5": 44.7, "ax6": -40.1},
    "pos3": {"ax1": -51.8, "ax2": 38.2, "ax3": 106.5, "ax4": 0.0, "ax5": 35.3, "ax6": -51.8},
    "posa": {"ax1": 32.2, "ax2": 65.5, "ax3": 55.9, "ax4": 0.0, "ax5": 58.6, "ax6": 32.2},
    "posb": {"ax1": 40.1, "ax2": 49.2, "ax3": 86.1, "ax4": 0.0, "ax5": 44.7, "ax6": 40.1},
    "posc": {"ax1": 51.8, "ax2": 38.2, "ax3": 106.5, "ax4": 0.0, "ax5": 35.3, "ax6": 51.8}
}

# Communication functions
def read_msg(sock):
    """ Reads messages from the robot controller. """
    while True:
        response = sock.recv(1024).decode("utf-8")
        update_current_joint_position(response)

def send_msg(sock, msg, console=False):
    """ Sends messages to the robot controller. """
    if console:
        print(f"SENT: {msg}")
    sock.sendall(bytearray(msg.encode("utf-8")))

def keep_alive(sock):
    """ Keeps the connection alive. """
    while True:
        send_msg(sock, CRI_ALIVEJOG)
        time.sleep(1)

def cmd_move_joint(ax1=0.0, ax2=0.0, ax3=0.0, ax4=0.0, ax5=0.0, ax6=0.0, speed=100.0):
    """ Generates a command to move the robot joints. """
    return f"CRISTART 2000 CMD Move Joint {ax1} {ax2} {ax3} {ax4} {ax5} {ax6} 0.0 0.0 0.0 {speed} CRIEND"

def update_current_joint_position(msg):
    """ Updates the current joint position. """
    data = msg.split(" ")
    if "POSJOINTCURRENT" in data:
        idx = data.index("POSJOINTCURRENT")
        for i, key in enumerate(["ax1", "ax2", "ax3", "ax4", "ax5", "ax6"], 1):
            pos_joint_current[key] = float(data[idx + i])

# Start communication threads
read_msg_worker = threading.Thread(target=read_msg, args=(sock,))
read_msg_worker.start()

keep_alive_worker = threading.Thread(target=keep_alive, args=(sock,))
keep_alive_worker.start()

# Pick and Place Logic
def pick_and_place(source, destination):
    """ Handles pick and place operations with memory tracking. """
    if not object_positions[source]:
        print(f"ERROR: No object at {source} to pick!")
        return
    
    if object_positions[destination]:
        print(f"ERROR: Destination {destination} is already occupied!")
        return

    # Move to pick position
    print(f"Picking object from {source}...")
    send_msg(sock, cmd_move_joint(**positions[source]))
    time.sleep(5)  # Simulating pick-up delay

    print("Moving to L position...")
    send_msg(sock, cmd_move_joint(ax3=90.0, ax5=90.0))
    time.sleep(3) 
   

    # Move to place position
    print(f"Placing object at {destination}...")
    send_msg(sock, cmd_move_joint(**positions[destination]))
    time.sleep(5)  # Simulating place delay
    
    print("Moving to L position...")
    send_msg(sock, cmd_move_joint(ax3=90.0, ax5=90.0))
    time.sleep(3) 

    # Update object positions
    object_positions[source] = False
    object_positions[destination] = True
    print(f"Object moved from {source} to {destination}")

# MQTT Message Handler
def on_message(client, userdata, message):
    """ Handles incoming MQTT messages and executes the corresponding robot actions. """
    global object_positions

    topic = message.topic
    if topic.startswith("IGUS/Rebel-3121081/move/pickplace"):
        data_json = message.payload.decode("utf-8")
        data_dict = json.loads(data_json)
        source = data_dict.get("source")
        destination = data_dict.get("destination")

        if source in object_positions and destination in object_positions:
            pick_and_place(source, destination)
        else:
            print("ERROR: Invalid pick/place position")

    elif topic in ["IGUS/Rebel-3121311/move/pos1", "IGUS/Rebel-3121311/move/pos2", "IGUS/Rebel-3121311/move/pos3",
                   "IGUS/Rebel-3121311/move/posa", "IGUS/Rebel-3121311/move/posb", "IGUS/Rebel-3121311/move/posc"]:
        send_msg(sock, cmd_move_joint(**positions[topic.split("/")[-1]]))

# Connect to MQTT Broker
cri_reciver.topics = ["IGUS/Rebel-3121311/#"]
cri_reciver.client.on_message = on_message
cri_reciver.client.connect("mqtt-dashboard.com", 1883)
cri_reciver.client.loop_start()

keep_alive_worker.join()
read_msg_worker.join()

