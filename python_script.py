#!/usr/bin/python
# -*- coding: utf8 -*-
import RPi.GPIO as GPIO
import signal
import time
import datetime
import sys
import os
from pirc522 import RFID
import socket
import smbus
#sys.path.insert(0,'/home/pi/docker_test/docker-Python_Script/pirc522/')

rdr = RFID()

debug = True
Card_Insert = 0

def end_read(signal,frame):
    global run
    print("\nCtrl+C captured, ending read.")
    run = False
    rdr.cleanup()
    sys.exit()

signal.signal(signal.SIGINT, end_read)

#********************************************************
# Bus GPIO
LedPin = 32
#********************************************************
GPIO_LEDR = 32          # LED Rouge
#********************************************************
GPIO_LEDV = 36          # LED Verte
#********************************************************
GPIO_buzzer = 7         # uzzer est branche sur la pin 7 / GPIO4
#********************************************************
GPIO_relais = 40        # Relais est branche sur la pin 40 / GPIO21
#********************************************************

#********************************************************
# Afficheur LCD via le bus I2C
I2C_ADDR  = 0 # pour test 0x27 et  0x3f pour city club
I2C_LINE = "2" # NB de lignes de l'afficheur, 2 lignes ou 4 lignes
LCD_ON = 1 # permet de désactiver le LCD si souci avec
LCD_WIDTH = 20   # Maximum characters per line
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command
LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line
LCD_BACKLIGHT  = 0x08  # On
ENABLE = 0b00000100 # Enable bit
E_PULSE = 0.0005 # Timing constants
E_DELAY = 0.0005 # Timing constants


# Initialisation LCD ************************************
def lcd_init(): 
    lcd_byte(0x33,LCD_CMD) # 110011 Initialise
    lcd_byte(0x32,LCD_CMD) # 110010 Initialise
    lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
    lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off 
    lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
    lcd_byte(0x01,LCD_CMD) # 000001 Clear display
    time.sleep(E_DELAY)

# Send byte to data pins, bits = the data, mode = 1 for data, 0 for command
def lcd_byte(bits, mode):
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT
    bus.write_byte(I2C_ADDR, bits_high) # High bits
    lcd_toggle_enable(bits_high)
    bus.write_byte(I2C_ADDR, bits_low) # Low bits
    lcd_toggle_enable(bits_low)

# LCD Toggle Enable **************************************
def lcd_toggle_enable(bits):
    time.sleep(E_DELAY)
    bus.write_byte(I2C_ADDR, (bits | ENABLE))
    time.sleep(E_PULSE)
    bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
    time.sleep(E_DELAY)

# Affichage Message sur une Ligne ************************
def lcd_string(message,line):
    message = message.ljust(LCD_WIDTH," ") # Send string to display
    lcd_byte(line, LCD_CMD)
    for lcd_i in range(LCD_WIDTH):
        lcd_byte(ord(message[lcd_i]),LCD_CHR)
        
        
def lcd_clear():
    lcd_byte(0x01,LCD_CMD) # 000001 Clear display

# Initialiser le LCD
I2C_ADDR = 0X27
bus = smbus.SMBus(1)
try:
    lcd_init()
    lcd_byte(0x01,LCD_CMD)
    lcd_init()
except:
    I2C_ADDR = 0x3F
    try:
        lcd_init()
        lcd_byte(0x01,LCD_CMD)
        lcd_init()
    except:
        I2C_ADDR = 0 # pas de LCD
        LCD_ON = 0 # pas de LCD
            
key_public    = [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF]      # Clé publique
B3S12= 12
# Blink Interval 
time_sleep_led = .5 #Time interval in Seconds
debug = True
rpiname = str(socket.gethostname())     # Nom RPI
def setup():
    # Initialisation du bus GPIO
    GPIO.setmode(GPIO.BOARD)            # comme la librairie MFRC522
    GPIO.setwarnings(False)             # 
    GPIO.setup(GPIO_relais, GPIO.OUT)   # Pin Relais
    GPIO.output(GPIO_relais, True)      # éteindre le Relais
    GPIO.setup(GPIO_buzzer, GPIO.OUT)   # Pin Buzzer
    GPIO.output(GPIO_buzzer, True)      # éteindre le Buzzer
    GPIO.setup(LedPin, GPIO.OUT)   # Pin Relais
    GPIO.output(LedPin, False)       # éteindre la LED Verte
    GPIO.setup(GPIO_LEDV, GPIO.OUT)     # Pin Led Verte
    GPIO.output(GPIO_LEDV, False)       # éteindre la LED Verte
    GPIO.setup(GPIO_LEDR, GPIO.OUT)     # Pin LED Rouge
    GPIO.output(GPIO_LEDR, False)       # éteindre LED Rouge
    
  
# Pour déclencher le buzzer : bip
def declenchebuzzer():
    GPIO.output(GPIO_buzzer,GPIO.LOW)  # led on
    time.sleep(0.1)        # Attendre
    GPIO.output(GPIO_buzzer, GPIO.HIGH) # led off
    time.sleep(0.1)        # Attendre

# Pour déclencher le buzzer 3 fois : bip-bip-bip
def declenchebuzzer3():
    for x in range(1, 4):
        declenchebuzzer()
        time.sleep(0.05)
#******************************************************* 
def turnOn(pin):
    GPIO.output(pin,True)
    time.sleep(time_sleep_led)
    GPIO.output(pin,False)
#********************************************************    
def destroy():
  GPIO.output(GPIO_LEDV, GPIO.LOW)   # led off
  GPIO.output(GPIO_LEDR, GPIO.LOW)   # led Rouge off
  GPIO.output(GPIO_buzzer,True)   # Buzzer off
  GPIO.cleanup()                  # Release resource
  
def Detect_Card():
    E_Carte = False
    E_Auth = False
    my_UID = []
    B3S12_data = []
    global debug
    # first try with CC key
    if True: # (not E_Carte) and (not E_Auth):
        (error_q, data) = rdr.request()
        if error_q:
            (error_q, data) = rdr.request()
        if not error_q:
            if debug: print("\nDetected: " + format(data, "02x"))
            E_Carte = True
            (error_u, my_UID) = rdr.anticoll()
            if error_u:
                #if debug: print("Card read UID: ",my_UID)
                (error_u, my_UID) = rdr.anticoll()
                #if error_u: (error_u, my_UID) = rdr.anticoll()
            if not error_u:
                if debug: print("Selecting UID " + str(my_UID))
                error_s = rdr.select_tag(my_UID)                
                if not error_s:
                    error_a = rdr.card_auth(rdr.auth_a, B3S12, key_public, my_UID)
                    if debug: print("Authenticate A CC ...") 
                    if not error_a:
                        if debug: print("READ CC ...")
                        (error_r, B3S12_data) = rdr.read(B3S12)
                        if not error_r:
                            if debug: print("\nReading block B3S12 with CityClub key : " + str(B3S12_data))
                            E_Auth = True
                            rdr.stop_crypto()
                            if debug: print("Stopping crypto1")
    # Renvoyer Réponse
    if my_UID == []: # au cas où la carte n'a pas été correctement détectée
        E_Carte = False
        # print(".", end="")
    # else:
        # print("")
    return (E_Carte, E_Auth, my_UID, B3S12_data)
#********************************************************

#********************************************************
def Wait_for_Card_Removing(old_UID):
# Attend jusqu'à ce que la carte soit retirée
    continue_waiting = True
    # rdr.stop_crypto()
    data = []
    while continue_waiting:
        (error_q, data) = rdr.request()
        if error_q:
            (error_q, data) = rdr.request()
        # print("Request : ", error_q)
        if not error_q:
            (error_u, my_UID) = rdr.anticoll()
            if error_u:
               (error_u, my_UID) = rdr.anticoll()
               #continue_waiting = True
##            if debug: print("UID: ", error_u, old_UID, my_UID)
            if not error_u:
                continue_waiting = (old_UID == my_UID)
##            else:
##                continue_waiting = False
        else:
            continue_waiting = False
        if continue_waiting:
            # print("+", end="")
            time.sleep(0.5)
    # en sortie
    rdr.stop_crypto()
    time.sleep(1) # afin de laisser le message affiché à l'écran
    print("Carte retirée ...\n")


if __name__ == '__main__':     # Program start from here
  setup()
  declenchebuzzer3()
  try:
      print("Starting")
      while True:
        # Mémoriser la Date et l'heure en cours
        strtoday= datetime.datetime.today() # qui sera affiché sur l'écran
        if LCD_ON: lcd_string(strtoday.strftime("%Y-%m-%d %H:%M:%S"),LCD_LINE_1)
        if LCD_ON: lcd_string("CARTE ?  ",LCD_LINE_2)            
        print("Attente Carte : ...")
        (Est_Carte, Est_Auth, tag_uid, data_bloc) = Detect_Card()
        if Est_Carte and len(tag_uid) >= 4:
            declenchebuzzer() # bip d'indication de lecture de la carte
            Card_Insert = 1
            turnOn(GPIO_LEDV)
            GCP_UID = '%02X' % tag_uid[0] + '%02X' % tag_uid[1] + '%02X' % tag_uid[2] + '%02X' % tag_uid[3]
            print("GCP_UID : ",GCP_UID)
        if Card_Insert == 1:
            Card_Insert = 0 # pour ne pas recommencer à la prochaine boucle sans carte
            Wait_for_Card_Removing(tag_uid)
            if debug: print("\nDeauthorizing")
  except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
    destroy()
