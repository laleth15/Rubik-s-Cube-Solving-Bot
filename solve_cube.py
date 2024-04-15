from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
from time import sleep
import sys
import argparse
from integrate_position_and_orientation import *
from script_test import *
from scan_integrate import scan_cube
from marker_flatten import re_orient_cube
from converter import *

def locate_cube(cam_number):

    return

def place_cube_at_home(bot):

    return


def locate_and_grab(cam_name, cam_num, bot):

    base_frame_offset = 0.2
    
    confirm = input("Press Y to try grasp or N to end the program:\n")

    if confirm == "Y" or confirm == "y":
        # Locate cube
        try:
            cube_x, up_marker_id, camera_side_id  = get_cube_x(cam_name, cam_num)
            print(cube_x/100+base_frame_offset)
            # If cube is closer than 30 cm cannot grab it
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
                success = grip_cube_at_x_small_reach(bot, 0.30)
                
            elif cube_x/100+base_frame_offset < 0.30:

                success = grip_cube_at_x_small_reach(bot, (cube_x/100)+ base_frame_offset)
            
            # if success:
            #         bot.arm.set_ee_cartesian_trajectory(z = 0.05)
            #         bot.arm.go_to_home_pose()

            # if no valid pose if found
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

    BOT.arm.go_to_sleep_pose()
    BOT.gripper.set_pressure(1)
    BOT.gripper.open()

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
                    print("Grasping Successful")

                
                    
            except Exception as err:
                print(err)
                if err.keys()[0] == "locate_cube":
                    if err.values()[0] == 1:
                        print("Error while locating cube: Cube is out of reach \n Relocate the cube in the range 10 to 45 cm from base ")
                    else:
                        print("Error while locating cube: ", err)
                        BOT.arm.go_to_sleep_pose()
                        BOT.gripper.open()
                        sys.exit(1)

                if err.keys()[0] == "grab_cube":
                    if err.values()[0] == 1:
                        print("Error while grabbing cube: No valid pose found \n Try relocating the cube")

                else:
                    print("Error while grabbing cube: ", err)


            # Try placing cube at home position
            try:
                res = move_gripper_just_above_home_position(BOT)
                BOT.gripper.open()
                BOT.arm.go_to_sleep_pose()

                if res:
                    print("Place cube at home.")
                    SUCCESS = True
                else:
                    print("Failed to place cube at home. Trying again.")
                    continue

            except Exception as err:
                print(err)


            # Re-orient the cube in correct position
            try:
                re_orient_move_set = re_orient_cube(up_marker_id, camera_side_id )
                print(re_orient_move_set)
                if len(re_orient_move_set) != 0:
                    perform_actions(BOT, re_orient_move_set)


            except Exception as err:
                print(err)

            

            # Scan cube
            try:
                move_set = scan_cube(CAMERA_NUMBER=CAMERA_NUMBER, CAMERA_NAME=CAMERA_NAME, bot=BOT)
                print(move_set)
                robot_moves = convertToRobotMoves(move_set)
                print(robot_moves)
                move_gripper_just_above_home_position(BOT)
                BOT.gripper.open()
                #sample = ["z","z'", "x","x'",'y',"y'",'y2']
                perform_actions(BOT, robot_moves)
            
            except Exception as err:
                print(err)
            # Run solving program and get a set of moves

            # Solve cube

    except KeyboardInterrupt:
        BOT.arm.go_to_sleep_pose()
        BOT.gripper.open()
        sys.exit(0)

    BOT.arm.go_to_home_pose()
    # move_gripper_just_above_home_position(BOT)
    BOT.gripper.open()
    BOT.arm.go_to_sleep_pose()
    sys.exit(0)

