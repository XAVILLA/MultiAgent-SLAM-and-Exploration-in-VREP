"""
@author: Andrius Bernatavicius, 2019
"""

from __future__ import print_function
from lib.env    import VrepEnvironment
from lib.agents import Pioneer, Display, Controller
import settings, time, argparse
import matplotlib.pyplot as plt
import numpy as np
from io import StringIO
from pynput.keyboard import Key, Listener
from lib.frontier import frontier_cluster

debug=False

def on_press(event):
    speed = 2
    if event == Key.up:
        if debug:
            print("UP")
        robot.change_velocity([speed, speed])
        print('--------------------------------- Direction Straight')
    elif event == Key.right:
        if debug:
            print("Right")
        robot.change_velocity([speed/8, -speed/8])
        #robot.change_velocity(1, target='left')
        #robot.change_velocity(-1, target='right')
        print('--------------------------------- Direction turn right')
    elif event == Key.left:
        if debug:
            print("Left")
        robot.change_velocity([-speed/8, speed/8])
        #robot.change_velocity(-1, target='left')
        #robot.change_velocity(1, target='right')
        print('--------------------------------- Direction turn left')
    elif event == Key.down:
        if debug:
            print("down")
        robot.change_velocity([-speed, -speed])
        print('--------------------------------- Direction turn Back')


def on_release(key):
    if debug:
        print("{} released").format(key)
    robot.change_velocity([0, 0])


def loop(robot, display):
    
    """
    Agent control loop
    """
    #byteArray = display.bytearray
    #print(byteArray)
    #array = np.frombuffer(display.bytearray, dtype=np.uint8)
    
    # where = 
    #array[array == 127] = -1
    # array /= (255/100)
    # array[array<0] = -1
    #buff = StringIO()
    #array = [int(x/(255/100)) for x in array if x!=127 and x!=-1 and not np.isnan(x)]
    #a = array.reshape((settings.image_size, settings.image_size))
    #output.serialize_numpy(buff, np)
    #print(type(output.data[0]))
    if robot.current_target:
        robot.drive_to_target()
    
if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--test', action='store_true', help='Test in a room environment')
    args = parser.parse_args()

    env = args.test
    
    # Initialize and start environment
    # if env:
    #     environment = VrepEnvironment(settings.SCENES + '/room_d.ttt') # Open the file containing our scene (robot and its environment)
    # else:
    #     environment = VrepEnvironment(settings.SCENES + '/room_d.ttt')  # Open the file containing our scene (robot and its environment)
    environment = VrepEnvironment(None) # Start the simulator
    print(environment.port)
    environment.connect()    # Connect our program to the simulator
    print(environment.port)
    # Create our robot in the current environment
    controller = Controller()
    robot   = Pioneer(environment, controller)
    display = Display(robot, True)  # Display the information of the robot

    print('\nDemonstration of Simultaneous Localization and Mapping using V-REP robot simulation software. \nPress "CTRL+C" to exit.\n')
    
    try:
        # LOOP
        start = time.time()
        step = 0
        #environment.start_simulation()
        #environment.step_simulation()
        camera_handle = environment.get_handle('Vision_sensor')
        
        # listener = Listener(on_press=on_press, on_release=on_release)
        # listener.start()
        
        while step < settings.simulation_steps:
            print()
            print("step:", step)
            environment.pause_simulation()
            display.update()
            environment.start_simulation()            
            loop(robot, display)
            step += 1
            #if robot.pos[0] < 330:
                #end = time.time()
                #print('Track completed!\nTime: {}'.format(end - start))
                #environment.destroy_instances()
                #break
            #if robot.distance_closest()[0] < 0.25 or robot.distance_closest()[1]<.25:
                #print('Hit an obstacle!\nTime: {}'.format(time.time() - start))
                #environment.destroy_instances()
                #break

    except KeyboardInterrupt:
        end = time.time()
        print('\n\nInterrupted! Time: {}s'.format(end-start))
        environment.destroy_instances()
    
    
