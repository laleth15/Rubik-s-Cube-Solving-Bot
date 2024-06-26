from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
from time import sleep
import sys
import argparse
from integrate_position_and_orientation import *
from execute_moves import *
from integrate_scan import scan_cube
from cube_orientation import re_orient_cube
from converter import *


def locate_and_grab(cam_name, cam_num, bot):
    '''
    A function to locate and grab Rubik's cube

        Parameters:
            cam_num (int): Camer ID/Number
            cam_name (string): Name of the camera/file name of calibration data
            bot (object): The initialized robot object
        Rrturns:
            (Boolean): 1 if grasp is success, 0 if fail, -1 if cancelled
            up_marker_id (int): Tag ID of the top facing cube side
            camera_side_id (int): Tag ID of the cube side facing the camera 
    '''

    base_frame_offset = 0.2
    
    confirm = input("Press Y to try grasp or N to end the program:\n")

    if confirm == "Y" or confirm == "y":
        # Locate cube
        try:
            # cube_x is in meters
            cube_x, up_marker_id, camera_side_id  = get_cube_x(cam_name, cam_num)

            # If cube is closer than 10 cm or far than 45 cm cannot grab it
            if cube_x/100+base_frame_offset < 0.1 or cube_x/100+base_frame_offset > 0.45 :
                raise Exception({"locate_cube": 1}) # Cube is out of reach
                
        except Exception as err:
            raise Exception(err)

        # Grab cube
        try:
            if cube_x/100+base_frame_offset > 0.30:

                success = grab_cube_at_x_large_reach(bot, (cube_x/100)+ base_frame_offset)
                bot.arm.set_ee_pose_components(x=0.30, z = 0.07)
                bot.gripper.open()
                bot.arm.set_ee_pose_components(x=0.10, z = 0.07, pitch = 1.5)
                bot.arm.go_to_home_pose()
                
            if success == True:
                success = grip_cube_at_x_small_reach(bot, (cube_x/100)+ base_frame_offset)
            
            if success == False:
                bot.gripper.open()
                bot.arm.go_to_sleep_pose()
                raise Exception({"grab_cube": 1})
            
            return 1, up_marker_id, camera_side_id 
                
        except Exception as err:
            raise Exception(err)
    
    elif confirm == "N" or confirm =="n":
        return -1
    else:
        return 0


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Get the position of the marker")
    parser.add_argument("-c", "--camera", type=int, help="Enter camera number")
    parser.add_argument("-n", "--name", type=str, help="Enter camera name")
    args = parser.parse_args()

    if args.name is None:
        print("Please enter camera name using -n or --name")
        exit()
    else:
        CAMERA_NAME = args.name
        print(f"Using camera name {CAMERA_NAME}")

    if args.camera is None:
        print("Please enter camera number using -c or --camera")
        exit()
    else:
        CAMERA_NUMBER = args.camera
        print(f"Using camera number {CAMERA_NUMBER}")

    
    BOT = InterbotixManipulatorXS("px150", "arm", "gripper")
    SUCCESS = False

    # Set Robot at an intial pose
    BOT.arm.go_to_sleep_pose()
    BOT.gripper.set_pressure(1)
    BOT.gripper.open()

    re_orient_move_set = None
    solve_move_set = None

    try:
        while SUCCESS == False:

            try:

                # Try locating and grasping of the cube
                res, up_marker_id, camera_side_id = locate_and_grab(CAMERA_NAME, CAMERA_NUMBER, BOT)

                if res == -1:
                    BOT.arm.go_to_sleep_pose()
                    BOT.gripper.open()
                    sys.exit(0)

                elif res != 1:
                    print("Failed to locate and grab. Trying again.")
                    continue

                if res == 1:
                    print("Grasping Successful. Trying to place it in home position.")

                
                    
            except Exception as err:
                # 
                # if err.keys()[0] == "locate_cube":
                #     if err.values()[0] == 1:
                #         "Error while locating cube: Cube is out of reach. Relocate the cube in the range 10 to 45 cm from base "
                # 
                #
                # if err.keys()[0] == "grab_cube":
                #     if err.values()[0] == 1:
                #         "Error while grabbing cube: No valid pose found \n Try relocating the cube"

                print("Error while grabbing cube: ", err)

            # Try placing cube at home position
            try:
                res = move_gripper_just_above_home_position(BOT)
                BOT.gripper.open()
                BOT.arm.go_to_sleep_pose()

                if res:
                    print("Cube placed at home successfully.")
                    
                else:
                    print("Failed to place cube at home. Trying again.")
                    print("May need to reposition the cube in workspace.")
                    continue

            except Exception as err:
                print(err)


            # Get the move set required to re-orient the cube so that white side faces up and green side faces the robot.
            # This step is necessary because the scan program requires the cube to be in a specific orientation.
            if re_orient_move_set == None:

                try:
                    re_orient_move_set = re_orient_cube(up_marker_id, camera_side_id )
                    print("Robot move set to reorient the cube: ",re_orient_move_set)
                    if len(re_orient_move_set) != 0:
                        res,err = perform_actions(BOT, re_orient_move_set)
                        
                        if res != True:
                            print("Failed to perform action: ", err)
                            continue

                except Exception as err:
                    print(err)

            

            # Scan cthe cube sides to analyse it's state/scramble
            # Refer coverter.py and script_test.py for the move set inference
            if solve_move_set == None:
                try:
                    solve_move_set = scan_cube(CAMERA_NUMBER=CAMERA_NUMBER, CAMERA_NAME=CAMERA_NAME, bot=BOT)
                    print("Solution to solve in terms of cube moves: ",solve_move_set)

                    robot_moves = convert_to_robot_moves(solve_move_set)
                    print("Solution to solve in terms of robot moves: ", robot_moves)

                except Exception as err:
                    print(err)

            # Set robot at a position to grab the cube and perform the solution moveset
            move_gripper_just_above_home_position(BOT)
            BOT.gripper.open()

            #sample_solution = ["z","z'", "x","x'",'y',"y'",'y2']
            try:
                res,err = perform_actions(BOT, robot_moves)
            except Exception as err:
                print(err)
           
            if res != True:
                print("Failed to perform action: ", err)
                continue
            
            # End the loop
            else:
                SUCCESS = True
            

    except KeyboardInterrupt:
        print('Interrupted')
        try:
            BOT.arm.go_to_sleep_pose()
            BOT.gripper.open()
            sys.exit(130)
        except Exception as err:
            print(err)
            

    BOT.arm.go_to_home_pose()
    # move_gripper_just_above_home_position(BOT)
    BOT.gripper.open()
    BOT.arm.go_to_sleep_pose()
    sys.exit(0)

