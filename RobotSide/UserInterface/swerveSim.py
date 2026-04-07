import sys
from typing import cast

import PySimpleGUI as sg
sys.path.append(r'c:\Dev\Zeus\RobotSide')
from UserInterface import zeusInput
from Utils.vector2d import Vector2d
from swerve.swerveKinematics import SwerveKinematics



layout = [[sg.Text("Zeus Driver Station")], [sg.Button("OK")], [sg.Graph(canvas_size=(400, 400), graph_bottom_left=(-100, -100), graph_top_right=(100, 100), background_color='white', enable_events=True, key='graph')]]

# Create the window
window = sg.Window("Demo", layout, margins=(200, 200),finalize=True)
graph = window['graph']

modules = []
swerve = SwerveKinematics(20, 20)

for i in range(4):
    points = (swerve.moduleLocations[i][0], swerve.moduleLocations[i][1])
    point =  graph.draw_point((points[0], points[1]), 10, color='green') # type: ignore
    modules.append(points)


zeusInput.start()

# Create an event loop
while True:
    currentControls = zeusInput.getControls()

    yMovement = zeusInput.rescale_value(currentControls.LY, -128, 127, -1, 1)
    xMovement = zeusInput.rescale_value(currentControls.LX, -128, 127, -1, 1)
    Turn = zeusInput.rescale_value(currentControls.RX, -128, 127, -1, 1)

    input = (xMovement, yMovement, Turn)

    output = swerve.normalize(swerve.toSwerveModuleStates(input[0], input[1], input[2]),15)

    for module in range(4):
        vector = Vector2d(x=output[0][module], y=output[1][module])
        graph.draw_line((modules[module][0], modules[module][1]), vector.getPoint()) # type: ignore

    try:
        event, values = window.read() or (None, None)
    except Exception as e:
        print(f"Error occurred: {e}")
        break
    
    # End program if user closes window or presses the OK button
    if event == "OK" or event == sg.WIN_CLOSED:
        break

window.close()