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
        self.odrv0.axis0.controller.config.vel_limit = 50000
        self.odrv0.axis1.controller.config.vel_limit = 50000

        # trap_traj parametrage des valeurs limit du comportement dynamique
        self.odrv0.axis1.trap_traj.config.vel_limit = 10000
        self.odrv0.axis0.trap_traj.config.vel_limit = 10000

        self.odrv0.axis0.trap_traj.config.accel_limit = 7000
        self.odrv0.axis1.trap_traj.config.accel_limit = 7000

        self.odrv0.axis0.trap_traj.config.decel_limit = 7000
        self.odrv0.axis1.trap_traj.config.decel_limit = 7000

        # test avec  calib_saved.py
        # self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        # self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def raz(self):
        # Fonction de remise à zero des moteurs pour initialisation si calib déja faite
        flag = 'N'
        flag = input("Le robot est hors sol ? (Y or N)")
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        if flag == 'Y':
            """self.odrv0.axis0.controller.config.vel_limit = 7000
            self.odrv0.axis1.controller.config.vel_limit = 7000

            # trap_traj parametrage des valeurs limit du comportement dynamique
            self.odrv0.axis1.trap_traj.config.vel_limit = 1000
            self.odrv0.axis0.trap_traj.config.vel_limit = 1000

            self.odrv0.axis0.trap_traj.config.accel_limit = 750
            self.odrv0.axis1.trap_traj.config.accel_limit = 750

            self.odrv0.axis0.trap_traj.config.decel_limit = 750
            self.odrv0.axis1.trap_traj.config.decel_limit = 750"""
            # Remise en position 0 des moteurs pour initialisation
            self.odrv0.axis0.controller.move_to_pos(0)
            self.odrv0.axis1.controller.move_to_pos(0)
            print("Poser le robot au sol")
            time.sleep(10)

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
