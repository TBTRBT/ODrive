#!/usr/bin/env python3

from __future__ import print_function
import time
import param as p
import move as m

import sys
sys.path.append('../../')
from utils.communication import Communication
import utils.Switch as s


def demo_simple(odrv0):

    move = m.Move(odrv0)

    time.sleep(1)
    move.translation(500)
    time.sleep(1)
    move.rotation(-90)
    time.sleep(1)
    move.translation(1000)


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
    for i in range(0,12):
        move.translation(350)
        time.sleep(2)
        move.rotation(-90)
        time.sleep(2)


def run_test(odrv0):
    # Strategie proposé de parcour
    com = Communication('/dev/ttyACM0')
    move = m.Move(odrv0)
    print("Initialisation des Actionneurs...")
    com.waitEndMove(Communication.MSG["Initialisation"], True)
    print("Initilisation DONE")
    time.sleep(1)

    print("Palet_Floor_In...")
    com.send(Communication.MSG["Palet_Floor_In"])
    print("Waiting moving forward...")
    while not com.Avancer:
        com.read(True)
    time.sleep(.1)

    # ICI ON AVANCE
    move.translation(500)
    print("Moving forward DONE")
    com.send(Communication.MSG["Action_Finished"])
    print("Waiting moving backward...")
    while not com.Reculer:
        com.read(True)
    time.sleep(.1)

    # ICI ON TOURNE
    move.rotation(-90)
    print("Rotation DONE")
    com.send(Communication.MSG["Action_Finished"])
    print("Waiting end of movement.")
    while not com.readyNext:
        com.read(True)

    # ICI ON RECULE
    move.translation(-500)
    print("Moving backward DONE")
    com.send(Communication.MSG["Action_Finished"])
    print("Waiting end of movement.")
    while not com.readyNext:
        com.read(True)
    print("Palet_Floor_In DONE.")
    time.sleep(1)

    print("Transport...")
    com.waitEndMove(Communication.MSG["Transport"], True)
    print("Transport DONE")
    time.sleep(1)

    print("Turn off robot...")
    com.send(Communication.MSG["Arret"])
    print("Robot is turned off!")


""" Paramétrage et Calibration """
param = p.Param()
# Lance fonction remise à zero des moteurs
param.raz()
# Lance la configuration du odrive (vit max / acc max / decc max / A max ...)
# param.config()
# param.calib()
""" ------------------------------- """

""" Choix de lancement des demos : """

s.tirette()

if s.cote() is True: #Jaune

    homologation(param.odrv0)
    # demo_tour(param.odrv0)
    # demo_rotation(param.odrv0)
else:
    run_test(param.odrv0) # Violet

"""--------------------------------"""

print('Fin du programme')

# odrv0.reboot()
