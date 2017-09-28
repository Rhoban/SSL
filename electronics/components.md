Pins count
1- M1 PWM
2- M1 DIR
3- M2 PWM
4- M2 DIR
5- M3 PWM
6- M3 DIR
7- M4 PWM
8- M4 DIR
9- M5 PWM

10- SCK
11- MISO
12- MOSI

13- IR Emit
14- IR Receive

15- Buzzer
16- Solenoid
17- IMU SDA
18- IMU SCL

19- CSA1
20- CSA2
21- CSA3

22 - IRQ1
23 - IRQ2
24 - IRQ3


Pour chaque moteur:
- W1
- W2
- W3
- GND
- +
- H1
- H2
- H3
- A
- B


A ajouter:
- Les contrôleurs moteurs
- Connecteurs moteurs?
- Un peu plus de découplage
- Décodeur de quadrature ls7366r
  + quartz
- Barrière infrarouge
- Condensateur sur la board?
- Modules de COM

A vérifier:
- La taille des trous pour le gros MOS
- Mettre deux kickers?
- Taille des capas

Notes
Les trous de fixation sont sur un rectangle de 126 x 82

MOS to kick IRFPE30 http://fr.farnell.com/infineon/irfp4242pbf/transistor-mosfet-n-300v-to-247/dp/1298480
OR FDH44N50 (farnell 500V / 40A)
http://fr.farnell.com/on-semiconductor-fairchild/fdh44n50/transistor-mosfet-n-to-247/dp/1095012

NPN SOT-23 BC817 Transistors
http://fr.farnell.com/nxp/bc817-215/transistor-npn-boitier-sot-23/dp/1081223

TRACOPOWER TSR 1-2450 5V/1A Switching
http://fr.farnell.com/tracopower/tsr-1-2450/convertisseur-dc-dc5v1asip/dp/1696320

TRACOPOWER TSR 1-2433 3.3V/1A Switching
http://fr.farnell.com/tracopower/tsr-1-2433/converter-dc-to-dc-3-3v-1a-sip/dp/1696319

Barrière infrarouge, avec émetteurs et récepteurs de 5mm (100mA d'émission):
- http://fr.farnell.com/vishay/tsal6200/emetteur-ir-940nm-5mm-traversant/dp/3152856
- http://fr.farnell.com/osram/sfh213-fa/photodiode-ir-filtree/dp/1212763

Quartz 40Mhz pour LS7366R-S
- http://fr.farnell.com/abracon/abl-40-000mhz-b2/crystal-40m-18pf-cl-hc49-4h/dp/1611785

Header 2x8 0.1" pour les modules de com
- http://fr.farnell.com/samtec/ssw-104-02-g-d-ra/embase-2-54mm-coudee-double-8voies/dp/1668343

Capacitor 160V 3300uF (30x50mm)

Centrale GY-85
Maple Mini


Resistors 0805
- 
- 
- 