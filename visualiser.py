import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import PySimpleGUI as sg
import matplotlib
import matplotlib.pyplot as plt
from pygame import time

from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import axes3d

# Argh lots of imports

from main import *

# Helper functions for graphics

def draw_figure(canvas, figure):
    """
    Helper function for PySimpleGUI/Matplotlib
    """
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=1)
    return (figure_canvas_agg)

def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def plot_robot(robot, fig, ax, tool_active=False):
    """
    Plots robot's position on axes based on current state
    Will need changing when we switch to a matrix based system
    """
    robot.update_position()

    frame_corners = robot.frame_corners
    # Add the vertical corners of the frame
    for i in frame_corners[0:4]:
        frame_corners.append([
            i[0],
            i[1],
            i[2] + robot.dimensions[2]
        ])

    # Each combo represents a line between two points
    combos = [[0, 2], [0, 4], [2, 6], [4, 6],
    [1, 3], [1, 5], [3, 7], [5, 7],
    [4, 5], [6, 7]]

    # Plot the frame of the robot
    for i in combos:
        start = frame_corners[i[0]]
        end = frame_corners[i[1]]
        ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]], "green")
    
    # Determine points to indicate the tool within the frame
    tool_corners = [frame_corners[4]]
    tool_corners.append([
        tool_corners[0][0] + robot.tool_location[0] * np.cos(np.deg2rad(robot.rotation)),
        tool_corners[0][1] + robot.tool_location[0] * np.sin(np.deg2rad(robot.rotation)),
        tool_corners[0][2]
    ])
    tool_corners.append([
        tool_corners[1][0] - robot.tool_location[1] * np.sin(np.deg2rad(robot.rotation)),
        tool_corners[1][1] + robot.tool_location[1] * np.cos(np.deg2rad(robot.rotation)),
        tool_corners[0][2]
    ])
    tool_corners.append(robot.true_location)

    for i in range(3):
        start = tool_corners[i]
        end = tool_corners[i + 1]
        ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]], "red", linewidth=3)
    
    if tool_active:
        ax.scatter(end[0],end[1],end[2], c="b", linewidth=5)
    else:
        ax.scatter(end[0],end[1],end[2], c="#cccccc", linewidth=5)

def generate_command_list(robot):
    """
    Enunciates task stack for output purposes
    """
    out = ""
    for i in robot.task_stack:
        out += (i["operation_name"] + " Task ID: "+str(i["task_id"]) + "\n")
        out += ("     Location type: " + i["location_type"] + "\n")
        out += ("     Status: " + i["task_status"] + "\n")
    
    return(out)


# GUI stuff
left_column = [
    [sg.Text("Current vehicle status: "), 
    sg.Text(robot_status, key="-STATUS-", background_color="white", text_color="black", size=(25,1))],
    [sg.Text("Current command stack")],
    [sg.Multiline(default_text="", size=(40,30), auto_refresh=True, key="-STACK-")],
    [sg.Button("Start sim", key="-START-"), sg.Button("Pause sim", key="-PAUSE-")]
]

plot_column = [
    [sg.Canvas(size=(100,100),key="-CANVAS-")],
    [sg.Text("Commands go here")]
]

layout = [
    [sg.Column(left_column),
    sg.VSeparator(),
    sg.Column(plot_column)]
]


# Matplotlib initialisation
fig = plt.figure(figsize=(10,8))
ax = plt.axes(projection="3d")

ax.set_xlim3d(0, 2)
ax.set_ylim3d(-0.5, 1.5)
ax.set_zlim3d(0, 2)

matplotlib.use("TkAgg")

clock = time.Clock()


# Robot initialisation

farmm = RobotPlaceholder([0,0,0],[0.2,0.2,0.1],0)
farmm.process_instruction_stack(task_queue)
for task in farmm.task_stack:
    farmm.slice_motion(task)

# Robot state variables for use in GUI
# Determine which of these need to be included in actual system

robot_status = ""

previous_location = [farmm.frame_location, farmm.tool_location]
next_location = []
steps_required = 0
steps_done = 0
tool_active = False

sim_running = False

window = sg.Window("FARMM status viewer", layout, finalize=True)


plot_robot(farmm, fig, ax)
fig_agg = draw_figure(window["-CANVAS-"].TKCanvas, fig)
fig_agg.draw()

while True:
    event, values = window.read(timeout=50)
    if event == "Exit" or event == sg.WIN_CLOSED:
        # Close the window
        break
    
    elif event == "-START-":
        # Start button pressed
        sim_running = True
    
    elif event == "-PAUSE-":
        # Pause button pressed
        sim_running = False

    elif next_location != [] and not tool_active and sim_running:
        # Move the robot

        steps_done += 1
        percent = steps_done/steps_required

        # Update each coordinate based on what % of the motion has been carried out
        updated_location = list(previous_location)
        for i in range(0,2):
            for j in range(0,3):
                updated_location[i][j] += percent * (next_location[i][j] - previous_location[i][j])
        
        farmm.frame_location = updated_location[0]
        farmm.tool_location = updated_location[1]
        

        if percent >=1:
            # The motion has ended
            previous_location = list(next_location)
            next_location = []
            steps_required = 0
            steps_done = 0

            farmm.motion_stack.pop(0)

    elif tool_active and sim_running:
        # Fire the tool, increasing the number of steps done

        steps_done += 1
        if steps_done >= steps_required:
            tool_active = False
            steps_required = 0
            steps_done = 0

            farmm.motion_stack.pop(0)

    if len(farmm.motion_stack) != 0 and next_location == [] and not tool_active and sim_running:
        # Select the next point from the motion stack and implement it

        next_point = farmm.motion_stack[0]

        # If the next point is a tool command, do a tool thing
        if type(next_point[0]) == str:
            tool_active = True
            steps_required = 5
        
        else:
            # Set the appropriate location
            next_location = [[next_point[3], next_point[4], 0], next_point[0:3]]

            # Determine how long the action will take
            max_frame_distance = max([
                abs(previous_location[0][0] - next_location[0][0]),
                abs(previous_location[0][1] - next_location[0][1]),
                abs(previous_location[0][2] - next_location[0][2])
            ])
            max_tool_distance = max([
                abs(previous_location[1][0] - next_location[1][0]),
                abs(previous_location[1][1] - next_location[1][1]),
                abs(previous_location[1][2] - next_location[1][2])
            ])
            if max_tool_distance > max_frame_distance:
                # The tool travels at 0.1m/s
                steps_required = max_tool_distance/0.1
            else:
                # The frame travels at 0.2m/s
                steps_required = max_frame_distance/0.2
    
    window["-STACK-"].update(value=generate_command_list(farmm))


    # Update the graph
    ax.cla()
    fig.clf()

    ax = plt.axes(projection="3d")
    ax.set_xlim3d(0, 2)
    ax.set_ylim3d(-0.5, 1.5)
    ax.set_zlim3d(0, 2)

    plot_robot(farmm, fig, ax, tool_active)
    fig_agg.draw()

    # Set the robot status text
    if next_location == [] and tool_active == False:
        robot_status = "Inactive"
    elif next_location == [] and tool_active == True:
        robot_status = "Tool running"
    elif next_location != []:
        if next_location[0] == previous_location[0] and next_location[1] != previous_location[1]:
            robot_status = "Tool moving inside frame"
        elif next_location[0] != previous_location[0] and next_location[1] == previous_location[1]:
            robot_status = "Frame moving, tool stationary"
        elif next_location[0] != previous_location[0] and next_location[1] != previous_location[1]:
            robot_status = "Both tool and frame moving"
    
    window["-STATUS-"].update(robot_status)
    
    # Limits to 20 FPS
    clock.tick(20)
