#!/usr/bin/env python3

from __future__ import print_function
import odrive
# à checker
from odrive.enums import *
import time

print("Recherche Odrive...")
odrv0 = odrive.find_any()
print("Lancement Calib & Sauvegarde config")
# 40Amp max dans le moteur (gros couple et sécurité pour pas fumer le moteur)
odrv0.axis0.motor.config.current_lim = 10
odrv0.axis1.motor.config.current_lim = 10

# vmax en tick/s les encodeurs font 8192 tick/tours
# controller.*.vel_limite prend le pas sur trap_traj.*.vel_limt
odrv0.axis0.controller.config.vel_limit = 50000
odrv0.axis1.controller.config.vel_limit = 50000

# trap_traj parametrage des valeurs limit du comportement dynamique
odrv0.axis1.trap_traj.config.vel_limit = 40000
odrv0.axis0.trap_traj.config.vel_limit = 40000

odrv0.axis0.trap_traj.config.accel_limit = 10000
odrv0.axis1.trap_traj.config.accel_limit = 10000

odrv0.axis0.trap_traj.config.decel_limit = 10000
odrv0.axis1.trap_traj.config.decel_limit = 10000

odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while odrv0.axis0.current_state != 1 and odrv0.axis1.current_state != 1:
    time.sleep(0.1)
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True
# Met les moteurs en boucle fermée
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.save_configuration()
