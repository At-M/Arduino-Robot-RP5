# Arduino-Robot-RP5
A selfdriving robot, using an Arduino and RP5-CH02 as a chassis.
This code was written under time pressure and is anything but good. (for example: global variables)

Images may follow.


## Additional Info:

  Values for motorspeeds may change depending on the smoothness of your floor, I'm leaving my default values in (smooth but "rubbery" PVC floor)
  Right or left motor may swap, depending on your wiring.
    To find out the needed value for ir_dist, build a loop which prints the received value from the sensor into serial monitor, move your robot at the desired distance from the wall, use that value (or a median of the last few printed)

 ## DEBUGGING:

  Put a red led between ground and pin 12, put a green led between ground and pin 11
  The pins are hardcoded right now, but may change.
  
  **LED ON** | **LED red** | **LED green**
  --- | --- | ---
  during unplanned left turn | / | distance right not enough
  during unplanned right turn | distance left not enough | /
  during right turn | 180° turn initiated | 90° turn initiated
  

 ## Module sources:

  Infrared Sharp (GP2Y0A21YK0F, distance sensor):
  
  [eBay (Germany)](https://www.ebay.de/itm/Sharp-IR-Sensor-GP2Y0A21YK0F-Distanzsensor-Kabel-Arduino-Infrarot-Raspberry-Pi/253638031006)
  

  Gyroscope(MPU6050, angle measuring):
  
  [Library](https://github.com/tockn/MPU6050_tockn)
  
  [Information about sample rate](https://www.researchgate.net/post/Who_have_used_the_Arduino_and_mpu6050_Can_you_tell_me_how_to_set_the_sample_rate_for_mpu6050)
  
  [Why signed short int?](https://www.luis.uni-hannover.de/fileadmin/kurse/material/CKurs/list_Operatoren.pdf)

  
  Endstops:
  
  [Amazon (Germany)](https://www.amazon.de/gp/product/B0744HCY6G)

  
  NAND-Gate for endstops:
  
  [eBay (Germany)]https://www.ebay.de/itm/CD4093BE-CMOS-Quad-2-Input-NAND-Schmitt-Triggers-HLF-DIP-14-1-oder-2-St%C3%BCck/172410722963?ssPageName=STRK%3AMEBIDX%3AIT&var=471240518309&_trksid=p2057872.m2749.l2649
  
  [Pinout](https://www.petervis.com/GCSE_Design_and_Technology_Electronic_Products/nand-gate-timers/nand-gate-timer-delay-on/cd4093b-pinout.gif)

  
  Bracket for Ultrasonic sensor (I modified this one):
  
  [Thingiverse](https://www.thingiverse.com/thing:189585)

  
  Gearbox cover:
  
  [Thingiverse](https://www.thingiverse.com/thing:2985540)
