# Objet du script :
# Valider la connexion d'un module LoRa-E5 à un réseau LoRaWAN privé sur TTN,
# préalablement configuré.
# Cet exemple est obtenu à partir des ressources mises à disposition par
# Vittascience :
# https://github.com/vittascience/stm32-libraries/tree/main/grove_modules

# Importation des différents pilotes
import machine
from stm32_LoRa import *
from utime import sleep_ms
import gc # Ramasse miettes, pour éviter de saturer la mémoire
from machine import I2C # Bibliothèque pour le bus I2C
import VL53L0X # Bibliothèque pour le VL53L0X


# Fréquence du timer, en hertz
MIN_FREQUENCY_HZ = 0.25
MAX_FREQUENCY_HZ = 10.

# Bornage de l'intervalle de mesures du capteur, valeurs en mm
MIN_DIST_MM = const(1)
MAX_DIST_MM = const(1700)

frequency = MIN_FREQUENCY_HZ # Fréquence initiale du timer
distance = MAX_DIST_MM # distance initiale d'un supposé obstacle

# Port série de la NUCLEO_WB55
UART_WB55 = const(2)

# Identifiants sur le réseau LoRaWAN
devAddr = ''
appEui	= '00 00 00 00 00 00 00 00'
appKey	= ''

# Temporisations diverses
DelayRebootLostConnection = 300 # Exprimé en minutes
DelayTryJoin = 10 # Exprimé en secondes
MaxTryJoin = int((DelayRebootLostConnection * 60) / DelayTryJoin)
DelaySend = 2 # Exprimé en secondes

# Initialisation du module LoRa-E5 
loRa = LoRa(9600, UART_WB55, DataReceiveCallback = None)

# Paramètres d'identification du module pour sa connexion au réseau LoRaWAN
status = loRa.setIdentify(DevAddr = devAddr,AppEui = appEui,AppKey = appKey)

# Affichage des différents paramètres du réseau LoRaWAN
def PrintLoRaParameters():
	identify = loRa.getIdentify()
	if(identify != -1):
		print("#####################################################################")
		print("########## INITIALIZE                                        ########")
		print("#####################################################################")
		print("LORA_DRIVER_VERSION: " + loRa.getDriverVersion())
		print("#### " + loRa.getMode() + " ####")
		print("#### AppKey: " + identify['AppKey'])
		print("#### DevEUI: " + identify['DevEui'])
		print("#### AppEUI: " + identify['AppEui'])
		print("#### DevAddr: " + identify['DevAddr'])
	else:
		print("#### = Read identify fail.\nReboot!")
		sleep_ms(2000)
		machine.reset()
	if status == -1:
		print("#### = Initialize fail.\nReboot!")
		sleep_ms(2000)
		machine.reset()
	else:
		print("#### = Initialize success.")

# Etablissement de la connexion ("join") LoRaWAN
def JoinNetwork():
	# Essaie de se connecter au réseau
	joinStatus = False
	tryJoin = 0
	while joinStatus == False:
		# Join LoRaWAN
		print("#### = Try join n°" + str(tryJoin + 1))
		status = loRa.join()
		if status == -1:
			print("#### = Join Fail, retry in " + str(DelayTryJoin) + " seconds.")
			tryJoin += 1
			# Si MaxTryJoin tentatives de connexion ont échoué
			if tryJoin > MaxTryJoin:
				# Reboot de la carte
				print("Reboot!")
				machine.reset()
			sleep_ms(DelayTryJoin * 1000)
		else:
				joinStatus = True
				print("#### = Join sucess.")

# Exécution des fonctions
PrintLoRaParameters() # Affichage des paramètres
JoinNetwork() # Join

#Initialisation du bus I2C numéro 1 du STM32WB55 
i2c = I2C(1)

# Pause d'une seconde pour laisser à l'I2C le temps de s'initialiser
sleep_ms(1000)

tof = VL53L0X.VL53L0X(i2c) # Instance du capteur de distance

# Fonction pour remapper un intervalle de valeurs dans un autre
@micropython.native # Produit un bytecode optimisé pour STM32
def map (value, from_min, from_max, to_min, to_max):
	return (value-from_min) * (to_max-to_min) / (from_max-from_min) + to_min
	
# Fonction principale
@micropython.native  # Produit un bytecode optimisé pour STM32
def main():

	global distance
	TEMPO = const(1000)
		
	sleep_ms(500) # Temporisation "de sécurité"
	tof.start() # démarrage du capteur
	tof.read() # Première mesure "à blanc"
	sleep_ms(500) # Temporisation "de sécurité"
	
	# Décompte des tentatives d'émission d'une trame 
	trySend = 0
	
	# Initialisation d'un tableau de 11 octets qui contiendra la trame
	# LoRaWAN au format Cayenne LPP
	loRaFrame = [0x00] * 2

	try: # Gestion d'erreurs
		
		while True:

			# Mesure de la distance et correction des valeurs aberrantes
			distance = min(max(tof.read(), MIN_DIST_MM), MAX_DIST_MM)
			print(distance)
			
			#loRaFrame[0] = 0x01 # Index de la donnée (tout simplement 1, 2, 3, 4, etc.).
			loRaFrame[0] = (distance >> 8) & 0xFF # Donnée codée sur 16 bits, extraction du bit de poids faible
			loRaFrame[1] = distance & 0xFF # Donnée codée sur 16 bits, extraction du bit de poids fort

			# Emission de la trame LoRaWAN
			
			print("#### = Send data.")
			trySend += 1
			sendStatus = loRa.sendData(loRaFrame, Port=1, NeedAck= False)
			
			# Si l'émission échoue, reessaye trySend fois puis reboote
			if sendStatus == -1:
				print("#### = Join fail.")
				if trySend > MaxTrySend:
					# Reboot board
					print("Reboot!")
					machine.reset()
			else:
				print("#### = Send success.")
				trySend = 0

			# Place le module LoRa-E5 en mode veille
			print("#### = LoRa module enter low power mode.")
			loRa.enterLowPowerMode()
			
			# Place le STM32WB55 en mode basse consommation
			pyb.wfi() 
			
			# Remappe la distance dans l'intervalle de fréquence souhaité
			#frequency = map(distance, MIN_DIST_MM, MAX_DIST_MM, MIN_FREQUENCY_HZ, MAX_FREQUENCY_HZ)
			
			# Prend le complément de la fréquence (pour qu'elle augmente lorsqu'on s'approche d'un obstacle).
			#frequency = round(max(MAX_FREQUENCY_HZ - frequency, MIN_FREQUENCY_HZ),3)

			# Ajuste la fréquence du timer
			# tim1.freq(frequency)
			
			# Appel du ramasse-miettes, indispensable pour que le programme ne se bloque pas
			# très rapidement en fragmentant complètement la RAM.
			gc.collect()
			
			sleep_ms(TEMPO) # Temporisation de 50 millisecondes

	# En cas d'interruption clavier avec *[CTRL]-[C]*
	except KeyboardInterrupt:
		# tim1.deinit() # Arrêt du timer
		# buzzer_pin.value(0) # Arrêt du buzzer
		tof.stop() # Arrêt du capteur

# Appel de la fonction principale
main()