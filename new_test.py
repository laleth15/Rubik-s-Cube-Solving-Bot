from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
from time import sleep

def move_gripper_to_cube_home_position(bot, left=False, right=False):
    roll_angle = home_pitch_adjust = 0
    home_x = 0.14
    if left or right:
        roll_angle = -np.pi/2 if left else np.pi/2
        home_pitch_adjust = -0.05
        home_x = 0.135
    bot.arm.set_ee_pose_components(x=home_x, z=0.11, pitch=np.pi/2 + home_pitch_adjust, roll=roll_angle)
    bot.arm.set_ee_pose_components(x=home_x, z=0.045, pitch=np.pi/2 + home_pitch_adjust, roll=roll_angle)

def raise_cube_from_home_position(bot, left=False, right=False):
    roll_angle = home_pitch_adjust = 0
    home_x = 0.13 if left or right else 0.14
    if left or right:
        roll_angle = -np.pi/2 if left else np.pi/2
        home_pitch_adjust = -0.05
    bot.arm.set_ee_pose_components(x=home_x, z=0.15, pitch=np.pi/2 + home_pitch_adjust, roll=roll_angle)
    bot.arm.set_ee_pose_components(x=0.2, z=0.25, pitch=0, roll=0)

def lower_cube_for_regripping(bot, bottom=False):
    roll = np.pi - 0.01 if bottom else 0
    vertical_offset = 0.005 if bottom else 0
    bot.arm.set_ee_pose_components(x=0.2, z=0.25, pitch=0, roll=roll)
    bot.arm.set_ee_pose_components(x=0.25, z=0.065 + vertical_offset, pitch=0, roll=roll)
    bot.gripper.open()
    bot.arm.set_ee_pose_components(x=0.25, z=0.15, pitch=0, roll=0)
    bot.arm.set_ee_pose_components(x=0.175, z=0.07, pitch=np.pi/2, roll=0)

def grip_cube_at_x(bot, x):
    pitch_offset = -0.135
    bot.arm.set_ee_pose_components(x=x, z=0.07, pitch=np.pi/2 + pitch_offset, roll=0)
    bot.gripper.close()

def process(bot, cube_position, regrab=False):
    move_gripper_to_cube_home_position(bot, left=cube_position == 'left', right=cube_position == 'right')
    bot.gripper.close()
    raise_cube_from_home_position(bot, left=cube_position == 'left', right=cube_position == 'right')
    lower_cube_for_regripping(bot, bottom=regrab)
    grip_cube_at_x(bot, x=0.27 if cube_position != 'right' else 0.265)
    bot.arm.set_ee_pose_components(x=0.14, z=0.07, pitch=np.pi/2, roll=0)
    bot.gripper.open()

def main_sequence(bot):
    bot.gripper.set_pressure(1)
    bot.gripper.open()
    bot.arm.go_to_sleep_pose()
    process(bot, 'left')
    process(bot, 'left')
    process(bot, 'right')
    process(bot, 'left', regrab=True)
    process(bot, 'left', regrab=True)
    process(bot, 'right', regrab=True)
    process(bot, 'left')
    process(bot, '', regrab=True)
    bot.gripper.open()
    bot.arm.go_to_sleep_pose()

if __name__ == '__main__':
    bot = InterbotixManipulatorXS("px150", "arm", "gripper")
    main_sequence(bot)
