#!/usr/local/bin python3

from __future__ import print_function
import time
import param as p
import move as m

import sys
sys.path.append('../../')
# from utils.communication import Communication
# import utils.Switch as s


def demo_simple(odrv0):

    move = m.Move(odrv0)

    # time.sleep(1)
    # move.translation(500, [False, False, False, False, False])
    time.sleep(1)
    move.rotation(-90, [False, False, False, False, False])
    time.sleep(1)
    move.translation(1000, [False, False, False, False, False])


def homologation(odrv0):
    move = m.Move(odrv0)
    move.translation(500, [True, True, True, False, False])
    time.sleep(1)
    move.rotation(-90, [True, True, True, False, False])
    time.sleep(1)
    move.translation(300, [True, True, True, False, False])
    time.sleep(1)
    move.rotation(-90, [True, True, True, False, False])
    time.sleep(1)
    move.translation(300, [True, True, True, False, False])


def demo_tour(odrv0):
    move = m.Move(odrv0)
    # Fait  3 tours de carreaux (chez Martial)
    for i in range(0, 12):
        move.translation(350)
        time.sleep(2)
        move.rotation(-90)
        time.sleep(2)


""" Paramétrage et Calibration """
param = p.Param()

# param.raz()# Lance fonction remise à zero des moteurs
# time.sleep(5)
# Lance la configuration du odrive
param.config()

param.calib()
""" ------------------------------- """

""" Choix de lancement des demos : """
'''
s.tirette()

if s.cote() == True: #Jaune

    homologation(param.odrv0)
    #demo_tour(param.odrv0)
    #demo_rotation(param.odrv0)
else : run_test(param.odrv0) # Violet
'''
"""--------------------------------"""
demo_simple(param.odrv0)

print('Fin du programme')

# odrv0.reboot()
