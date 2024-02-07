import subprocess
from pathlib import Path
# from tkinter import Tk, Canvas, Button, PhotoImage, Entry, Label
# from tkinter import *
import tkinter as tk
from tkinter import ttk
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import FK_UI
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
import threading
import time
from IK import calculate_scara_angles
from FK import scara_FK
from send_move import send_cmd
import re


#Default
stored_j1 = 0.0
stored_j2 = 100.0
stored_j3 = 0.0
stored_j4 = 0.0

curr_pos = [0.0, 0.0, 21.0, 0.0]


# DEFULT PARAMETERS
default_speed = 500
default_acc = 20

joint1_new = 0.0
joint2_new = 0.0
joint3_new = 0.0
joint4_new = 0.0


def update_gui_from_file():
    global stored_j1, stored_j2, stored_j3, stored_j4, curr_pos
    while True:
        try:
            # Read values from the text file
            with open('pos_rec.txt', 'r') as file:
                stored_joints = file.read()
                values = re.findall(r'[-+]?\d*\.\d+|[-+]?\d+', stored_joints)
                
                if len(values) >= 4:
                    stored_j1 = float(values[0])
                    stored_j2 = float(values[1])
                    stored_j3 = float(values[2])
                    stored_j4 = float(values[3])
                    
                    curr_pos = scara_FK(stored_j1, stored_j2, stored_j3, stored_j4)
                else:
                    print("Insufficient values in the file. Writing default values...")
                    # Write [0.0, 0.0, 0.0, 0.0] into the file
                    with open('pos_rec.txt', 'w') as write_file:
                        write_file.write("[0.0, 0.0, 0.0, 0.0]")
                        
                    # Set default values in memory
                    stored_j1, stored_j2, stored_j3, stored_j4 = 0.0, 0.0, 0.0, 0.0
                  
        except FileNotFoundError:
            print("File not found. Writing default values...")
            # Write [0.0, 0.0, 0.0, 0.0] into the file
            with open('pos_rec.txt', 'w') as write_file:
                write_file.write("[0.0, 0.0, 0.0, 0.0]")
                
            # Set default values in memory
            stored_j1, stored_j2, stored_j3, stored_j4 = 0.0, 0.0, 0.0, 0.0
        except (ValueError, IndexError):
            print("Error reading values from the file. Writing default values...")
            # Write [0.0, 0.0, 0.0, 0.0] into the file
            with open('pos_rec.txt', 'w') as write_file:
                write_file.write("[0.0, 0.0, 0.0, 0.0]")
                
            # Set default values in memory
            stored_j1, stored_j2, stored_j3, stored_j4 = 0.0, 0.0, 0.0, 0.0
        time.sleep(1)



update_thread = threading.Thread(target=update_gui_from_file)
update_thread.daemon = True  # Set the thread as a daemon so it will exit when the main program exits
update_thread.start()


# Set the initial values from curr_joint
curr_joint = [stored_j1, stored_j2, stored_j3, stored_j4]
curr_pos = [round(value, 2) for value in curr_pos]


# UI SETUP
window = tk.Tk()
window.geometry("1400x800")
window.configure(bg="#FFFFFF")

def relative_to_assets(path: str) -> Path:
    return ASSETS_PATH / Path(path)

OUTPUT_PATH = Path(__file__).parent
ASSETS_PATH = OUTPUT_PATH / Path("./assets")


# INPUT VARIABLES
X_input = ""
Y_input = ""
Z_input = ""
G_input = ""
Speed_input = ""
Acc_input = ""
Jog_cart = ""



terminal_messages = []

# Function to update and display the terminal with the last 20 messages
def update_terminal():
    # Display the last 20 messages, if available
    if len(terminal_messages) > 20:
        displayed_messages = terminal_messages[-20:]
    else:
        displayed_messages = terminal_messages

    # Join the messages and update the teprminal
    new_text = '\n'.join(displayed_messages)
    terminal_output.config(text=new_text)

# Function to print a message to the terminal
def print_to_terminal(text):
    # Add the new message to the list
    terminal_messages.append(text)

    # Update and display the terminal
    update_terminal()

# Function to handle user input and execute the script
def handle_user_input(event):
    user_input = input_entry.get()
    # Process the user's input here
    print_to_terminal(f"CMD: {user_input}")

    # Run the send_gcode.py script and capture its output
    try:
        script_output = subprocess.check_output(
            ["python", "send_gcode.py", user_input],
            stderr=subprocess.STDOUT,
            universal_newlines=True
        )
        # Display the script output in the simulated terminal
        print_to_terminal(script_output)
    except subprocess.CalledProcessError as e:
        print_to_terminal(f"Error: {e.output}")

    # Clear the input field after processing
    input_entry.delete(0, tk.END)

# Create the terminal label widget
terminal_output = tk.Label(
    window,
    wraplength=340,  # Width of the label
    justify='left',  # Text alignment
    anchor='nw',  # Anchor text to the northwest (top-left)
    font=("Courier", 9),  # Font settings
    relief="solid",  # Border style
    bg="#000000",  # Background color
    fg="#FFFFFF",  # Text color
)
terminal_output.place(x=1050, y=0, width=340, height=750)

# Create an Entry widget for user input
input_entry = tk.Entry(window, font=("Courier", 9), bg="#FFFFFF")
input_entry.place(x=1050, y=750, width=340, height=30)

# Bind the Enter key press event to the handle_user_input function
input_entry.bind('<Return>', handle_user_input)

# Initial display of the terminal
update_terminal()

    



# SCRIPTS
def run_homing_script():
    # Runs the homing script located in the current directory
    subprocess.Popen(["python", "./homing.py"])

def run_near_mode_script():
    # Runs the near mode script located in the current directory
    subprocess.Popen(["python", "./near_mode.py"])

def go_cart():
    # global x_cart_new, y_cart_new, z_cart_new, g_cart_new
    X_input = x_entry.get()
    Y_input = y_entry.get()
    Z_input = z_entry.get()
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", X_input,
        "--y", Y_input,
        "--z", Z_input,
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def go_joint():
    J1_input = entry1.get()
    J2_input = entry2.get()
    J3_input = entry3.get()
    J4_input = entry4.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_FK_UI.py",
        "--J1", J1_input,
        "--J2", J2_input,
        "--J3", J3_input,
        "--J4", J4_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def stop_motors():
    # Runs the near mode script located in the current directory
    subprocess.Popen(["python", "./motors_off.py"])





# Calculate joints for coords
def calculate_and_update_labels(event=None):
    # Get the values from Entry fields
    x_value = float(x_entry.get())
    y_value = float(y_entry.get())
    z_value = float(z_entry.get())
    g_value = float(g_entry.get())

    # Call the function to calculate SCARA angles
    print("X_value:", x_value)
    print("Y_value:", y_value)
    print("Z_value:", z_value)
    print("g_value:", g_value)


    j1, j2, j3, j4 = calculate_scara_angles(x_value, y_value, z_value, g_value, [stored_j1, stored_j2, stored_j3, stored_j4])

    # Update the labels with the calculated values
    j1_label.config(text=f"J1: {j1:.2f}")
    j2_label.config(text=f"J2: {j2:.2f}")
    j3_label.config(text=f"J3: {j3:.2f}")
    j4_label.config(text=f"J4: {j4:.2f}")


# Function to calculate SCARA coordinates from joints and update labels
def calculate_and_update_coord_labels(event=None):
    # Get the values from sliders and entry fields
    j1_value = float(var1.get())
    j2_value = float(var2.get())
    j3_value = float(var3.get())
    j4_value = float(var4.get())

    # Call the function to calculate SCARA coordinates using FK
    x, y, z, gripper_or = scara_FK(j1_value, j2_value, j3_value, j4_value)

    # Update the labels with the calculated coordinates
    x_label.config(text=f"X: {x:.2f}")
    y_label.config(text=f"Y: {y:.2f}")
    z_label.config(text=f"Z: {z:.2f}")
    g_label.config(text=f"A: {gripper_or:.2f}")



def add_10_to_stored_y():
    goal_y = curr_pos[1]+10
    target = [str(curr_pos[0]), str(goal_y), str(curr_pos[2])]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def add_5_to_stored_y():
    goal_y = curr_pos[1]+5
    target = [str(curr_pos[0]), str(goal_y), str(curr_pos[2])]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def add_1_to_stored_y():
    goal_y = curr_pos[1]+1
    target = [str(curr_pos[0]), str(goal_y), str(curr_pos[2])]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def sub_1_from_stored_y():
    goal_y = curr_pos[1] - 1

    target = [str(curr_pos[0]), str(goal_y), str(curr_pos[2])]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def sub_5_from_stored_y():
    goal_y = curr_pos[1] - 5
    target = [str(curr_pos[0]), str(goal_y), str(curr_pos[2])]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def sub_10_from_stored_y():
    goal_y = curr_pos[1] - 10
    target = [str(curr_pos[0]), str(goal_y), str(curr_pos[2])]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def add_10_to_stored_x():
    goal_x = curr_pos[0]+10
    target = [str(goal_x), str(curr_pos[1]), str(curr_pos[2])]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def add_5_to_stored_x():
    goal_x = curr_pos[0]+5
    target = [str(goal_x), str(curr_pos[1]), str(curr_pos[2])]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def add_1_to_stored_x():
    goal_x = curr_pos[0]+1
    target = [str(goal_x), str(curr_pos[1]), str(curr_pos[2])]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def sub_1_from_stored_x():
    goal_x = curr_pos[0]-1
    target = [str(goal_x), str(curr_pos[1]), str(curr_pos[2])]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def sub_5_from_stored_x():
    goal_x = curr_pos[0]-5
    target = [str(goal_x), str(curr_pos[1]), str(curr_pos[2])]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def sub_10_from_stored_x():
    goal_x = curr_pos[0]-10
    target = [str(goal_x), str(curr_pos[1]), str(curr_pos[2])]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def add_10_to_stored_z():
    goal_z = curr_pos[2]+10
    target = [str(curr_pos[0]), str(curr_pos[1]), str(goal_z)]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def add_5_to_stored_z():
    goal_z = curr_pos[2]+5
    target = [str(curr_pos[0]), str(curr_pos[1]), str(goal_z)]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def add_1_to_stored_z():
    goal_z = curr_pos[2]+1
    target = [str(curr_pos[0]), str(curr_pos[1]), str(goal_z)]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def sub_1_from_stored_z():
    goal_z = curr_pos[2]-1
    target = [str(curr_pos[0]), str(curr_pos[1]), str(goal_z)]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def sub_5_from_stored_z():
    goal_z = curr_pos[2]-5
    target = [str(curr_pos[0]), str(curr_pos[1]), str(goal_z)]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)

def sub_10_from_stored_z():
    goal_z = curr_pos[2]-10
    target = [str(curr_pos[0]), str(curr_pos[1]), str(goal_z)]
    print("TARGET X:", target[0])
    print("TARGET Y:", target[1])
    print("TARGET Z:", target[2])
    G_input = g_entry.get()
    Speed_input = speed_entry.get()
    Acc_input = acc_entry.get()

    # Constructing the command
    command = [
        "python", "./send_IK_UI.py",
        "--x", target[0],
        "--y", target[1],
        "--z", target[2],
        "--a", G_input,
        "--speed", Speed_input,
        "--acceleration", Acc_input
    ]

    subprocess.Popen(command)



# GO_CART button
go_cart_button = tk.Button(
    window,
    text="GO",
    font=("Inter Bold", 14),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=go_cart,
    relief="flat"
)
go_cart_button.place(x=385.0, y=410.0, width=45.0, height=45.0)

# GO_JOG button
go_jog_button = tk.Button(
    window,
    text="GO",
    font=("Inter Bold", 14),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=go_joint,
    relief="flat"
)
go_jog_button.place(x=435.0, y=710.0, width=45.0, height=45.0)


# SAVE button
save_button = tk.Button(
    window,
    text="SAVE",
    font=("Inter Bold", 12),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    # command=save_function,
    relief="flat"
)
save_button.place(x=750.0, y=589.0, width=60.0, height=40.0)

# HOMING button
homing_button = tk.Button(
    window,
    text="HOMING",
    font=("Inter Bold", 14),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=run_homing_script,
    relief="flat"
)
homing_button.place(x=900.0, y=625.0, width=124.0, height=59.0)

# NEAR MODE button
near_mode_button = tk.Button(
    window,
    text="NEAR MODE",
    font=("Inter Bold", 14),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=run_near_mode_script,
    relief="flat"
)
near_mode_button.place(x=900.0, y=690.0, width=124.0, height=59.0)


# MOTORS OFF
motors_off_button = tk.Button(
    window,
    text="MOTORS OFF",
    font=("Inter Bold", 14),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=stop_motors,
    relief="flat"
)
motors_off_button.place(x=700.0, y=690.0, width=124.0, height=59.0)

# Y+10
y10_button = tk.Button(
    window,
    text="+10",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=add_10_to_stored_y,
    relief="flat"
)
y10_button.place(x=165.0, y=80.0, width=51.0, height=30.0)

# Y+5
y5_button = tk.Button(
    window,
    text="+5",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=add_5_to_stored_y,
    relief="flat"
)
y5_button.place(x=165.0, y=115.0, width=51.0, height=30.0)

# Y+1
y1_button = tk.Button(
    window,
    text="+1",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=add_1_to_stored_y,
    relief="flat"
)
y1_button.place(x=165.0, y=150.0, width=51.0, height=30.0)

# Y-1
ym1_button = tk.Button(
    window,
    text="-1",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=sub_1_from_stored_y,
    relief="flat"
)
ym1_button.place(x=165.0, y=231.0, width=51.0, height=30.0)

# Y-5
ym5_button = tk.Button(
    window,
    text="-5",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=sub_5_from_stored_y,
    relief="flat"
)
ym5_button.place(x=165.0, y=266.0, width=51.0, height=30.0)

# Y-10
ym10_button = tk.Button(
    window,
    text="-10",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=sub_10_from_stored_y,
    relief="flat"
)
ym10_button.place(x=165.0, y=301.0, width=51.0, height=30.0)

# X+1
x1_button = tk.Button(
    window,
    text="+1",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=add_1_to_stored_x,
    relief="flat"
)
x1_button.place(x=216.0, y=180.0, width=30.0, height=51.0)

# X+5
x5_button = tk.Button(
    window,
    text="+5",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=add_5_to_stored_x,
    relief="flat"
)
x5_button.place(x=251.0, y=180.0, width=30.0, height=51.0)

# X+10
x10_button = tk.Button(
    window,
    text="+10",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=add_10_to_stored_x,
    relief="flat"
)
x10_button.place(x=286.0, y=180.0, width=30.0, height=51.0)

# X-1
xm1_button = tk.Button(
    window,
    text="-1",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=sub_1_from_stored_x,
    relief="flat"
)
xm1_button.place(x=135.0, y=180.0, width=30.0, height=51.0)

# X-5
xm5_button = tk.Button(
    window,
    text="-5",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=sub_5_from_stored_x,
    relief="flat"
)
xm5_button.place(x=100.0, y=180.0, width=30.0, height=51.0)

# X-10
xm10_button = tk.Button(
    window,
    text="-10",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=sub_10_from_stored_x,
    relief="flat"
)
xm10_button.place(x=65.0, y=180.0, width=30.0, height=51.0)


# Z+10
z10_button = tk.Button(
    window,
    text="+10",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=add_10_to_stored_z,
    relief="flat"
)
z10_button.place(x=365.0, y=80.0, width=51.0, height=30.0)

# Z+5
z5_button = tk.Button(
    window,
    text="+5",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=add_5_to_stored_z,
    relief="flat"
)
z5_button.place(x=365.0, y=115.0, width=51.0, height=30.0)

# Z+1
z1_button = tk.Button(
    window,
    text="+1",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=add_1_to_stored_z,
    relief="flat"
)
z1_button.place(x=365.0, y=150.0, width=51.0, height=30.0)

# Z-1
zm1_button = tk.Button(
    window,
    text="-1",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=sub_1_from_stored_z,
    relief="flat"
)
zm1_button.place(x=365.0, y=231.0, width=51.0, height=30.0)

# Z-5
zm5_button = tk.Button(
    window,
    text="-5",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=sub_5_from_stored_z,
    relief="flat"
)
zm5_button.place(x=365.0, y=266.0, width=51.0, height=30.0)

# Z-10
zm10_button = tk.Button(
    window,
    text="-10",
    font=("Inter Bold", 9),
    fg="#FFFFFF",
    bg="#1350EC",
    borderwidth=0,
    highlightthickness=0,
    command=sub_10_from_stored_z,
    relief="flat"
)
zm10_button.place(x=365.0, y=301.0, width=51.0, height=30.0)



# Pos Saving
pos_name_entry = tk.Entry(window, font=("Inter Bold", 14), fg="#000000", bg="#FFFEFE")
pos_name_entry.place(x=650.0, y=589.0, width=90.0, height=40.0)




label_x = tk.Label(
    window,
    text="X",
    font=("Inter Bold", 18),
    fg="#14141B",
    bg="#FFFFFF"
)
label_x.place(x=320.0, y=193.0, anchor="nw")

label_y = tk.Label(
    window,
    text="Y",
    font=("Inter Bold", 18),
    fg="#14141B",
    bg="#FFFFFF"
)
label_y.place(x=180.0, y=47.0, anchor="nw")

label_minus_x = tk.Label(
    window,
    text="-X",
    font=("Inter Bold", 18),
    fg="#14141B",
    bg="#FFFFFF"
)
label_minus_x.place(x=36.0, y=193.0, anchor="nw")

label_minus_y = tk.Label(
    window,
    text="-Y",
    font=("Inter Bold", 18),
    fg="#14141B",
    bg="#FFFFFF"
)
label_minus_y.place(x=175.0, y=332.0, anchor="nw")

label_z = tk.Label(
    window,
    text="Z",
    font=("Inter Bold", 18),
    fg="#14141B",
    bg="#FFFFFF"
)
label_z.place(x=385.0, y=47.0, anchor="nw")

label_minus_z = tk.Label(
    window,
    text="-Z",
    font=("Inter Bold", 18),
    fg="#14141B",
    bg="#FFFFFF"
)
label_minus_z.place(x=380.0, y=332.0, anchor="nw")


# Entry fields
label_x_2 = tk.Label(
    window,
    text="X",
    font=("Inter Bold", 16),
    fg="#14141B",
    bg="#FFFFFF"
)
label_x_2.place(x=40.0, y=385.0, anchor="nw")

label_y_2 = tk.Label(
    window,
    text="Y",
    font=("Inter Bold", 16),
    fg="#14141B",
    bg="#FFFFFF"
)
label_y_2.place(x=100.0, y=385.0, anchor="nw")

label_z = tk.Label(
    window,
    text="Z",
    font=("Inter Bold", 16),
    fg="#14141B",
    bg="#FFFFFF"
)
label_z.place(x=160.0, y=385.0, anchor="nw")

label_a = tk.Label(
    window,
    text="A",
    font=("Inter Bold", 16),
    fg="#14141B",
    bg="#FFFFFF"
)
label_a.place(x=220.0, y=385.0, anchor="nw")

label_speed = tk.Label(
    window,
    text="Speed",
    font=("Inter Bold", 16),
    fg="#14141B",
    bg="#FFFFFF"
)
label_speed.place(x=258.0, y=385.0, anchor="nw")

label_acc = tk.Label(
    window,
    text="Acc",
    font=("Inter Bold", 16),
    fg="#14141B",
    bg="#FFFFFF"
)
label_acc.place(x=330.0, y=385.0, anchor="nw")

# Create a ttk Style object
style = ttk.Style()

# Configure the background color of the sliders to white
style.configure("TScale", background="#FFFFFF")

# Create Entry widgets for x, y, z, g, speed, and acc
x_entry = tk.Entry(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE")
x_entry.place(x=25.0, y=419.0, width=52.0, height=26.0)
x_entry.insert(0, "{:.2f}".format(curr_pos[0]))  # Set the default value for x_entry with 2 decimal places

y_entry = tk.Entry(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE")
y_entry.place(x=85.0, y=419.0, width=52.0, height=26.0)
y_entry.insert(0, "{:.2f}".format(curr_pos[1]))  # Set the default value for y_entry with 2 decimal places

z_entry = tk.Entry(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE")
z_entry.place(x=145.0, y=419.0, width=52.0, height=26.0)
z_entry.insert(0, "{:.2f}".format(curr_pos[2]))  # Set the default value for z_entry with 2 decimal places

g_entry = tk.Entry(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE")
g_entry.place(x=205.0, y=419.0, width=52.0, height=26.0)
g_entry.insert(0, "{:.2f}".format(curr_pos[3]))  # Set the default value for g_entry with 2 decimal places

speed_entry = tk.Entry(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE")
speed_entry.place(x=265.0, y=419.0, width=52.0, height=26.0)
speed_entry.insert(0, str(default_speed))  # Set the default value for speed_entry from default_speed

acc_entry = tk.Entry(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE")
acc_entry.place(x=325.0, y=419.0, width=52.0, height=26.0)
acc_entry.insert(0, str(default_acc))  # Set the default value for acc_entry from default_acc

#Display joints from world coords via IK
j1_label = tk.Label(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE", text=f"J1: {curr_pos[0]:.2f}")
j1_label.place(x=45.0, y=450.0, width=70.0, height=26.0)

j2_label = tk.Label(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE", text=f"J2: {curr_pos[1]:.2f}")
j2_label.place(x=125.0, y=450.0, width=70.0, height=26.0)

j3_label = tk.Label(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE", text=f"J3: {curr_pos[2]:.2f}")
j3_label.place(x=205.0, y=450.0, width=70.0, height=26.0)

j4_label = tk.Label(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE", text=f"J4: {curr_pos[3]:.2f}")
j4_label.place(x=285.0, y=450.0, width=70.0, height=26.0)

# Bind the event handler function to Entry fields
x_entry.bind("<FocusOut>", calculate_and_update_labels)
y_entry.bind("<FocusOut>", calculate_and_update_labels)
z_entry.bind("<FocusOut>", calculate_and_update_labels)
g_entry.bind("<FocusOut>", calculate_and_update_labels)

# Call the event handler function initially to set the labels based on the default values
calculate_and_update_labels()

def update_slider_from_entry(entry, slider):
    try:
        # Parse the value from the entry field
        value = float(entry.get())
        
        # Round the value to 2 decimal places
        value = round(value, 2)
        
        # Update the slider
        slider.set(value)
        
        # Call the update_labels function to update the labels
        update_labels()
    except ValueError:
        pass

# Function to update entry field from slider
def update_entry_from_slider(slider, entry):
    value = slider.get()
    
    # Update the entry field with the rounded value
    entry.delete(0, tk.END)
    entry.insert(0, f'{value:.2f}')
    
    # Call the update_labels function to update the labels
    update_labels()

def update_labels():
    x, y, z, g = scara_FK(slider1.get(), slider2.get(), slider3.get(), slider4.get())
    x_label.config(text=f"X: {x:.2f}")
    y_label.config(text=f"Y: {y:.2f}")
    z_label.config(text=f"Z: {z:.2f}")
    g_label.config(text=f"A: {g:.2f}")

# Sliders
slider1 = ttk.Scale(window, from_=-180, to=180, orient="horizontal", style="TScale")
slider2 = ttk.Scale(window, from_=0, to=330, orient="horizontal", style="TScale")
slider3 = ttk.Scale(window, from_=-150, to=150, orient="horizontal", style="TScale")
slider4 = ttk.Scale(window, from_=-180, to=180, orient="horizontal", style="TScale")

# Create Entry widgets to allow manual value input
entry1 = tk.Entry(window, width=5)
entry2 = tk.Entry(window, width=5)
entry3 = tk.Entry(window, width=5)
entry4 = tk.Entry(window, width=5)

# Place the sliders and entry fields in your desired positions
slider1.place(x=50, y=550, width=350)
slider2.place(x=50, y=590, width=350)
slider3.place(x=50, y=630, width=350)
slider4.place(x=50, y=670, width=350)

entry1.place(x=410, y=550)
entry2.place(x=410, y=590)
entry3.place(x=410, y=630)
entry4.place(x=410, y=670)

# Create labels for sliders
label1 = tk.Label(window, text="J1", font=("Inter Bold", 14), fg="#000000", bg="#FFFFFF")
label2 = tk.Label(window, text="J2", font=("Inter Bold", 14), fg="#000000", bg="#FFFFFF")
label3 = tk.Label(window, text="J3", font=("Inter Bold", 14), fg="#000000", bg="#FFFFFF")
label4 = tk.Label(window, text="J4", font=("Inter Bold", 14), fg="#000000", bg="#FFFFFF")

# Place the labels and sliders in your desired positions
label1.place(x=20, y=548)
label2.place(x=20, y=588)
label3.place(x=20, y=628)
label4.place(x=20, y=668)

# Bind entry fields to sliders
entry1.bind("<FocusOut>", lambda event, s=slider1, e=entry1: update_slider_from_entry(e, s))
slider1.bind("<Motion>", lambda event, s=slider1, e=entry1: update_entry_from_slider(s, e))

entry2.bind("<FocusOut>", lambda event, s=slider2, e=entry2: update_slider_from_entry(e, s))
slider2.bind("<Motion>", lambda event, s=slider2, e=entry2: update_entry_from_slider(s, e))

entry3.bind("<FocusOut>", lambda event, s=slider3, e=entry3: update_slider_from_entry(e, s))
slider3.bind("<Motion>", lambda event, s=slider3, e=entry3: update_entry_from_slider(s, e))

entry4.bind("<FocusOut>", lambda event, s=slider4, e=entry4: update_slider_from_entry(e, s))
slider4.bind("<Motion>", lambda event, s=slider4, e=entry4: update_entry_from_slider(s, e))

# Create labels for display
x_label = tk.Label(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE", text="X: 0.00")
x_label.place(x=45.0, y=700.0, width=70.0, height=26.0)

y_label = tk.Label(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE", text="Y: 0.00")
y_label.place(x=125.0, y=700.0, width=70.0, height=26.0)

z_label = tk.Label(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE", text="Z: 0.00")
z_label.place(x=205.0, y=700.0, width=70.0, height=26.0)

g_label = tk.Label(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE", text="A: 0.00")
g_label.place(x=285.0, y=700.0, width=70.0, height=26.0)

# Update labels initially
update_labels()



curr_j1_label = tk.Label(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE")
curr_j1_label.place(x=550.0, y=480.0, width=65.0, height=26.0)

curr_j2_label = tk.Label(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE")
curr_j2_label.place(x=650.0, y=480.0, width=65.0, height=26.0)

curr_j3_label = tk.Label(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE")
curr_j3_label.place(x=750.0, y=480.0, width=65.0, height=26.0)

curr_j4_label = tk.Label(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE")
curr_j4_label.place(x=850.0, y=480.0, width=65.0, height=26.0)

curr_j1_label.config(text=f"J1: {stored_j1:.2f}")
curr_j2_label.config(text=f"J2: {stored_j2:.2f}")
curr_j3_label.config(text=f"J3: {stored_j3:.2f}")
curr_j4_label.config(text=f"J4: {stored_j4:.2f}")

curr_x_label = tk.Label(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE")
curr_x_label.place(x=550.0, y=515.0, width=65.0, height=26.0)

curr_y_label = tk.Label(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE")
curr_y_label.place(x=650.0, y=515.0, width=65.0, height=26.0)

curr_z_label = tk.Label(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE")
curr_z_label.place(x=750.0, y=515.0, width=65.0, height=26.0)

curr_a_label = tk.Label(window, font=("Inter Bold", 10), fg="#000000", bg="#FFFEFE")
curr_a_label.place(x=850.0, y=515.0, width=65.0, height=26.0)

curr_x_label.config(text=f"X: {curr_pos[0]:.2f}")
curr_y_label.config(text=f"Y: {curr_pos[1]:.2f}")
curr_z_label.config(text=f"Z: {curr_pos[2]:.2f}")
curr_a_label.config(text=f"A: {curr_pos[3]:.2f}")













####################################################################################################

# PLOTTING
fig = Figure(figsize=(5, 4), dpi=120)
plot = fig.add_subplot(111, projection='3d')
canvas = FigureCanvasTkAgg(fig, master=window)
canvas.get_tk_widget().place(x=440, y=0)

# JUST FOR TESTING FK
# curr_joint = [-90, 100, 0, 0]
old_joint = [0.0, 0.0, 0.0, 0.0]
end_joint = [90, 300, -100, -50]
diff_joint = [end - start for start, end in zip(curr_joint, end_joint)]
t = 3

# Function to update the 3D plot data
def update_plot(iteration):
    global curr_joint, old_joint
    plot.clear()
    # plot.plot([0,0], [0,0], [0,670], 'b-', linewidth=20, label='Link 1')
    plot.plot([0,0], [0,0], [0,350], '#000099', linewidth=20, label='Link 1')
    (link1_x, link1_y, link1_z, link2_x, link2_y, link2_z, link3_x, link3_y, link3_z, link4_x_v1, link4_y_v1, link4_z_v1, link4_x_v2, link4_y_v2, link4_z_v2) = FK_UI.calculate_forward_kinematics([stored_j1, stored_j2, stored_j3, stored_j4])

    # plot.plot(link1_x, link1_y, link1_z, 'c-', linewidth=20, label='Link 1')
    plot.plot(link2_x, link2_y, link2_z, '#0000CC', linewidth=10, label='Link 2')
    plot.plot(link3_x, link3_y, link3_z, '#0000FF', linewidth=5, label='Link 3')
    plot.plot(link4_x_v1, link4_y_v1, link4_z_v1, '#3333FF', linewidth=2, label='Link 4')
    plot.plot(link4_x_v2, link4_y_v2, link4_z_v2, '#3333FF', linewidth=2, label='Link 4')

    plot.set_xlim([-750, 750])
    plot.set_ylim([-750, 750])
    plot.set_zlim([0, 750])

    # Change the font size of the tick labels on X, Y, and Z axes
    plot.tick_params(axis='x', labelsize=8)  # X-axis
    plot.tick_params(axis='y', labelsize=8)  # Y-axis
    plot.tick_params(axis='z', labelsize=8)  # Z-axis

    plot.set_xlabel('X')
    plot.set_ylabel('Y')
    plot.set_zlabel('Z')
    # plot.set_title('3D Plot')

    canvas.draw()

    # if iteration <= t:
    #     inc_joint = [d / t for d in diff_joint]
    #     print("INC_JOINT:", inc_joint)

    #     # Call calculate_forward_kinematics function from FK_UI with inc_joint
    #     (link1_x, link1_y, link1_z, link2_x, link2_y, link2_z, link3_x, link3_y, link3_z,
    #         link4_x_v1, link4_y_v1, link4_z_v1, link4_x_v2, link4_y_v2, link4_z_v2) = FK_UI.calculate_forward_kinematics([stored_x, stored_y, stored_z, stored_a])

    #     # Set the alpha (transparency) based on the iteration number
    #     alpha = 0.15 if iteration != t else 1.0

    #     # Generate and plot updated data with transparency
    #     plot.plot(link1_x, link1_y, link1_z, 'b-', linewidth=20, label='Link 1', alpha=alpha)
    #     plot.plot(link2_x, link2_y, link2_z, 'b-', linewidth=10, label='Link 2', alpha=alpha)
    #     plot.plot(link3_x, link3_y, link3_z, 'b-', linewidth=5, label='Link 3', alpha=alpha)
    #     plot.plot(link4_x_v1, link4_y_v1, link4_z_v1, 'b-', linewidth=2, label='Link 4', alpha=alpha)
    #     plot.plot(link4_x_v2, link4_y_v2, link4_z_v2, 'b-', linewidth=2, label='Link 4', alpha=alpha)

    #     plot.set_xlim([-750, 750])
    #     plot.set_ylim([-750, 750])
    #     plot.set_zlim([0, 350])

    #     # Change the font size of the tick labels on X, Y, and Z axes
    #     plot.tick_params(axis='x', labelsize=8)  # X-axis
    #     plot.tick_params(axis='y', labelsize=8)  # Y-axis
    #     plot.tick_params(axis='z', labelsize=8)  # Z-axis

    #     plot.set_xlabel('X')
    #     plot.set_ylabel('Y')
    #     plot.set_zlabel('Z')
    #     # plot.set_title('3D Plot')

    #     canvas.draw()
    #     curr_joint = [start + inc for start, inc in zip(curr_joint, inc_joint)]
    #     window.after(1, update_plot, iteration + 1)
    old_joint = [stored_j1, stored_j2, stored_j3, stored_j4]
        


def check_if_plot():
    while True:
        curr_joint = [stored_j1, stored_j2, stored_j3, stored_j4]
        if old_joint != curr_joint:
            update_plot(1)
        time.sleep(0.1)

condition_thread = threading.Thread(target=check_if_plot)
condition_thread.daemon = True
condition_thread.start()

window.resizable(False, False)
window.mainloop()
