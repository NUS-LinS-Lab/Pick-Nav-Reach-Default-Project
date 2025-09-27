import numpy as np
import pybullet as p


class KeyBoardController:
    def __init__(self, env):
        self.env = env
        self.action = np.zeros(self.env.action_size)
        self.turn = 0
        self.forward = 0
        self.yaw = 0
        self.torso_lift = 0
        self.head_pan = 0
        self.head_tilt = 0
        self.shoulder_pan = 0
        self.shoulder_lift = 0
        self.upperarm_roll = 0
        self.elbow_flex = 0
        self.forearm_roll = 0
        self.wrist_flex = 0
        self.wrist_roll = 0
        self.gripper_open = 0

    def key_callback(self):
        keys = p.getKeyboardEvents()

        for k,v in keys.items():
            # root translation
            if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                self.turn = -1
            if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
                self.turn = 0
            if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                self.turn = 1
            if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
                self.turn = 0
            if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                self.forward = 1
            if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
                self.forward = 0
            if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                self.forward = -1
            if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
                self.forward = 0

            # root rotation
            if (k == ord('r') and (v&p.KEY_WAS_TRIGGERED)):
                self.yaw = 1
            if (k == ord('r') and (v&p.KEY_WAS_RELEASED)):
                self.yaw = 0
            if (k == ord('f') and (v&p.KEY_WAS_TRIGGERED)):
                self.yaw = -1
            if (k == ord('f') and (v&p.KEY_WAS_RELEASED)):
                self.yaw = 0

            # torso_lift
            # if (k == ord('r') and (v & p.KEY_WAS_TRIGGERED)):
            #     self.torso_lift = 1
            # if (k == ord('r') and (v & p.KEY_WAS_RELEASED)):
            #     self.torso_lift = 0
            # if (k == ord('f') and (v & p.KEY_WAS_TRIGGERED)):
            #     self.torso_lift = -1
            # if (k == ord('f') and (v & p.KEY_WAS_RELEASED)):
            #     self.torso_lift = 0

            # head_pan
            # if (k == ord('a') and (v & p.KEY_WAS_TRIGGERED)):
            #     self.head_pan = -1
            # if (k == ord('a') and (v & p.KEY_WAS_RELEASED)):
            #     self.head_pan = 0
            # if (k == ord('d') and (v & p.KEY_WAS_TRIGGERED)):
            #     self.head_pan = 1
            # if (k == ord('d') and (v & p.KEY_WAS_RELEASED)):
            #     self.head_pan = 0

            # head_tilt
            # if (k == ord('r') and (v & p.KEY_WAS_TRIGGERED)):
            #     self.head_tilt = 1
            # if (k == ord('r') and (v & p.KEY_WAS_RELEASED)):
            #     self.head_tilt = 0
            # if (k == ord('f') and (v & p.KEY_WAS_TRIGGERED)):
            #     self.head_tilt = -1
            # if (k == ord('f') and (v & p.KEY_WAS_RELEASED)):
            #     self.head_tilt = 0

            # shoulder_pan
            # if (k == ord('r') and (v & p.KEY_WAS_TRIGGERED)):
            #     self.shoulder_pan = 1
            # if (k == ord('r') and (v & p.KEY_WAS_RELEASED)):
            #     self.shoulder_pan = 0
            # if (k == ord('f') and (v & p.KEY_WAS_TRIGGERED)):
            #     self.shoulder_pan = -1
            # if (k == ord('f') and (v & p.KEY_WAS_RELEASED)):
            #     self.shoulder_pan = 0

            # shoulder_lift
            if (k == ord('o') and (v & p.KEY_WAS_TRIGGERED)):
                self.shoulder_lift = 1
            if (k == ord('o') and (v & p.KEY_WAS_RELEASED)):
                self.shoulder_lift = 0
            if (k == ord('l') and (v & p.KEY_WAS_TRIGGERED)):
                self.shoulder_lift = -1
            if (k == ord('l') and (v & p.KEY_WAS_RELEASED)):
                self.shoulder_lift = 0

            # upperarm_roll
            # if (k == ord('y') and (v & p.KEY_WAS_TRIGGERED)):
            #     self.upperarm_roll = 1
            # if (k == ord('y') and (v & p.KEY_WAS_RELEASED)):
            #     self.upperarm_roll = 0
            # if (k == ord('h') and (v & p.KEY_WAS_TRIGGERED)):
            #     self.upperarm_roll = -1
            # if (k == ord('h') and (v & p.KEY_WAS_RELEASED)):
            #     self.upperarm_roll = 0

            # elbow_flex
            if (k == ord('u') and (v & p.KEY_WAS_TRIGGERED)):
                self.elbow_flex = 1
            if (k == ord('u') and (v & p.KEY_WAS_RELEASED)):
                self.elbow_flex = 0
            if (k == ord('j') and (v & p.KEY_WAS_TRIGGERED)):
                self.elbow_flex = -1
            if (k == ord('j') and (v & p.KEY_WAS_RELEASED)):
                self.elbow_flex = 0

            # forearm_roll
            # if (k == ord('u') and (v & p.KEY_WAS_TRIGGERED)):
            #     self.forearm_roll = 1
            # if (k == ord('u') and (v & p.KEY_WAS_RELEASED)):
            #     self.forearm_roll = 0
            # if (k == ord('j') and (v & p.KEY_WAS_TRIGGERED)):
            #     self.forearm_roll = -1
            # if (k == ord('j') and (v & p.KEY_WAS_RELEASED)):
            #     self.forearm_roll = 0
                
            # wrist_flex
            if (k == ord('y') and (v & p.KEY_WAS_TRIGGERED)):
                self.wrist_flex = 1
            if (k == ord('y') and (v & p.KEY_WAS_RELEASED)):
                self.wrist_flex = 0
            if (k == ord('h') and (v & p.KEY_WAS_TRIGGERED)):
                self.wrist_flex = -1
            if (k == ord('h') and (v & p.KEY_WAS_RELEASED)):
                self.wrist_flex = 0
                
            # wrist_roll
            if (k == ord('1') and (v & p.KEY_WAS_TRIGGERED)):
                self.wrist_roll = 1
            if (k == ord('1') and (v & p.KEY_WAS_RELEASED)):
                self.wrist_roll = 0
            if (k == ord("2") and (v & p.KEY_WAS_TRIGGERED)):
                self.wrist_roll = -1
            if (k == ord("2") and (v & p.KEY_WAS_RELEASED)):
                self.wrist_roll = 0

            # gripper
            if (k == ord('q') and (v & p.KEY_WAS_TRIGGERED)):
                self.gripper_open = -1
            if (k == ord('q') and (v & p.KEY_WAS_RELEASED)):
                self.gripper_open = 0
            if (k == ord('e') and (v & p.KEY_WAS_TRIGGERED)):
                self.gripper_open = 1
            if (k == ord('e') and (v & p.KEY_WAS_RELEASED)):
                self.gripper_open = 0

        self.action[:] = [self.forward, self.turn, self.yaw, self.torso_lift, self.head_pan, self.head_tilt, self.shoulder_pan, self.shoulder_lift, self.upperarm_roll, self.elbow_flex, self.forearm_roll, self.wrist_flex, self.wrist_roll, self.gripper_open, self.gripper_open]

    def get_action(self):
        self.key_callback()
        return np.clip(self.action, -1.0, 1.0)