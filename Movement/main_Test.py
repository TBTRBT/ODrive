#!/usr/bin/env python3

from __future__ import print_function
import time
import param as p
import move_test as m

import sys
sys.path.append('../../')
from utils.communication import Communication
import utils.Switch as s


def demo_simple(odrv0) :

    move_test = m.Move(odrv0)

    time.sleep(1)
    move_test.translation(500)
    time.sleep(1)
    move_test.rotation(-90)
    time.sleep(1)
    move_test.translation(1000)


def homologation(odrv0) :
    move_test = m.Move(odrv0)
    move_test.translation(500, [True,True,True,False,False])
    time.sleep(1)
    move_test.rotation(-90,[True,True,True,False,False])
    time.sleep(1)
    move_test.translation(300,[True,True,True,False,False])
    time.sleep(1)
    move_test.rotation(-90,[True,True,True,False,False])
    time.sleep(1)
    move_test.translation(300,[True,True,True,False,False])


def demo_tour(odrv0) :
    move_test = m.Move(odrv0)
    # Fait  3 tours de carreaux (chez Martial)
    for i in range(0,12):
        move_test.translation(350)
        time.sleep(2)
        move_test.rotation(-90)
        time.sleep(2)


def run_test(odrv0):
    # Strategie proposé de parcour
    com = Communication('/dev/ttyACM0')
    move_test = m.Move(odrv0)
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
    move_test.translation(500)
    print("Moving forward DONE")
    com.send(Communication.MSG["Action_Finished"])
    print("Waiting moving backward...")
    while not com.Reculer:
        com.read(True)
    time.sleep(.1)

    # ICI ON TOURNE
    move_test.rotation(-90)
    print("Rotation DONE")
    com.send(Communication.MSG["Action_Finished"])
    print("Waiting end of movement.")
    while not com.readyNext:
        com.read(True)

    # ICI ON RECULE
    move_test.translation(-500)
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
