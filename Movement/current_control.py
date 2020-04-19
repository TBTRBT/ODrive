#!/usr/bin/env python3

from __future__ import print_function

import odrive
from odrive.enums import *  # a checker
import time
from math import *

class Param:
    def __init__(self):
        print("finding an odrive...")
        self.odrv0 = odrive.find_any()
        print('Odrive found ! ')

    def config(self):
        # 40Amp max dans le moteur (gros couple et sécurité pour pas fumer le moteur)
        self.odrv0.axis0.motor.config.current_lim = 10
        self.odrv0.axis1.motor.config.current_lim = 10

        # vmax en tick/s les encodeurs font 8192 tick/tours
        # controller.*.vel_limite prend le pas sur trap_traj.*.vel_limt
        self.odrv0.axis0.controller.config.vel_limit = 5000
        self.odrv0.axis1.controller.config.vel_limit = 5000

        # trap_traj parametrage des valeurs limit du comportement dynamique
        self.odrv0.axis1.trap_traj.config.vel_limit = 3000
        self.odrv0.axis0.trap_traj.config.vel_limit = 3000

        self.odrv0.axis0.trap_traj.config.accel_limit = 1000
        self.odrv0.axis1.trap_traj.config.accel_limit = 1000

        self.odrv0.axis0.trap_traj.config.decel_limit = 1000
        self.odrv0.axis1.trap_traj.config.decel_limit = 1000

        # test avec  calib_saved.py
        # self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        # self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def calib(self):
        # Fonction de calibration sans condition

        # Lance la calibration moteur si pas déjà faite
        print("starting calibration...")
        self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

        while self.odrv0.axis0.current_state != 1 and self.odrv0.axis1.current_state != 1:
            time.sleep(0.1)

        # Met les moteurs en boucle fermée
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def unlock_wheels(self):
        # AXIS_STATE_IDLE , libère le moteur : boucle ouverte
        self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
        self.odrv0.axis1.requested_state = AXIS_STATE_IDLE

    def current_control(self):
        target = 81920
        # valeur (A) avant glissement des roues
        seuil = 1
        # self.odrv0.axis0.config.control_mode = CTRL_MODE_CURRENT_CONTROL
        self.odrv0.axis0.controller.move_to_pos(target)
        self.odrv0.axis1.controller.move_to_pos(-target)
        while self.odrv0.axis0.encoder.pos_estimate < target:
            print(self.odrv0.axis0.motor.current_control.Iq_measured)
            if abs(self.odrv0.axis0.motor.current_control.Iq_measured) >= seuil and abs(self.odrv0.axis1.motor.current_control.Iq_measured) >= seuil:
                print("Robot en butée")
                self.odrv0.axis0.controller.set_vel_setpoint(0, 0)
                self.odrv0.axis1.controller.set_vel_setpoint(0, 0)
                time.sleep(0.1)
        return


def test():
    param = Param()
    param.config()
    param.calib()
    param.current_control()


if __name__ == '__main__':
    # Pour ne pas lancer l'expérience : 'python3 main.py False ___'
    # Pour lancer l'homologation par Mat : 'python3 main.py ___ True
    Param.test()
