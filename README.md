# Arduino-Robot-RP5
A selfdriving robot, using an Arduino and RP5-CH02 as a chassis




  Additional Info:

  Values for motorspeeds may change depending on the smoothness of your floor, I'm leaving my default values in, which were on PVC floor
  Rright or left motor may swap, depending on your wiring
  Errorcodes (for debugging)
  To find out the needed value for ir_dist, build a loop which prints the received value from the sensor into serial monitor, move your robot at the desired distance from the wall, use that value (or a median of the last few printed)

  DEBUGGING:

  Put a red led between ground and pin 12, put a green led between ground and pin 11
  LED Red:

  ON (during unplanned right turn): distance on the left is not enough
  ON (during right turn): 180° turn

  LED green:

  ON (during unplanned left turn): distance on the right is not enough
  ON (during right turn): 90° turn

  Module sources:

  Infrared Sharp (GP2Y0A21YK0F, distance sensor):
  https://www.ebay.de/itm/Sharp-IR-Sensor-GP2Y0A21YK0F-Distanzsensor-Kabel-Arduino-Infrarot-Raspberry-Pi/253638031006

  Gyroscope(MPU6050, angle measuring):
  https://github.com/tockn/MPU6050_tockn
  https://www.researchgate.net/post/Who_have_used_the_Arduino_and_mpu6050_Can_you_tell_me_how_to_set_the_sample_rate_for_mpu6050 (Abtastgeschwindigkeit)
  https://www.luis.uni-hannover.de/fileadmin/kurse/material/CKurs/list_Operatoren.pdf (signed short int)

  Endstops:
  https://www.amazon.de/gp/product/B0744HCY6G

  NAND-Gate for endstops:
  https://www.ebay.de/itm/CD4093BE-CMOS-Quad-2-Input-NAND-Schmitt-Triggers-HLF-DIP-14-1-oder-2-St%C3%BCck/172410722963?ssPageName=STRK%3AMEBIDX%3AIT&var=471240518309&_trksid=p2057872.m2749.l2649
  https://www.petervis.com/GCSE_Design_and_Technology_Electronic_Products/nand-gate-timers/nand-gate-timer-delay-on/cd4093b-pinout.gif

  Bracket for Ultrasonic sensor (I modified this one):
  https://www.thingiverse.com/thing:189585

  Gearbox cover:
  https://www.thingiverse.com/thing:2985540
