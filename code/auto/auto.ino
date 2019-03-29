#include <MPU6050_tockn.h> // Bibliothek für Gyroskop
#include <Wire.h> // Voraussetzung für Gyroskop 
/*
   Autonomer Roboter "Gary"
   Version 1.3
   Made by:
   David Schmidt, Max Grüning, Steven Bolln, Pascal Harders
   Feb-Mar 2019

  Fehlercodes:

  LED ROT:

  Ein (während Rechtsdrehung): Entfernung links zu gering
  Ein (während Rechtsdrehung): 180° Drehung

  LED GRÜN:

  Ein (während Linksdrehung): Entfernung rechts zu gering
  Ein (während Rechtsdrehung): 90° Drehung

  (Kauf-)Quellen:

  Infrarot (KY-032, schnelle Bewegungserkennung):
  http://sensorkit.joy-it.net/index.php?title=KY-032_Hindernis_Detektor_Modul
  https://www.amazon.de/gp/product/B07CN8F14V

  Infrarot Sharp (GP2Y0A21YK0F, Entfernungsmessung):
  https://www.ebay.de/itm/Sharp-IR-Sensor-GP2Y0A21YK0F-Distanzsensor-Kabel-Arduino-Infrarot-Raspberry-Pi/253638031006

  Gyroskop(MPU6050, Winkelmessung):
  https://github.com/tockn/MPU6050_tockn
  https://www.researchgate.net/post/Who_have_used_the_Arduino_and_mpu6050_Can_you_tell_me_how_to_set_the_sample_rate_for_mpu6050 (Abtastgeschwindigkeit)
  https://www.luis.uni-hannover.de/fileadmin/kurse/material/CKurs/list_Operatoren.pdf (signed short int)

  Ultraschall (HC-SR04, Entfernungsmessung):
  https://playground.arduino.cc/Code/NewPing
  https://www.amazon.de/gp/product/B00R2U8HK6

  Endschalter:
  https://www.amazon.de/gp/product/B0744HCY6G

  OR-Gate für Endschalter (unbenutzt!)
  https://www.ebay.de/itm/5x-CMOS-4071-OR-Gatter-4-fach-2-Eingänge-C-MOS-IC-DIP14/311014657148

  NAND-Gate für Endschalter:
  https://www.ebay.de/itm/CD4093BE-CMOS-Quad-2-Input-NAND-Schmitt-Triggers-HLF-DIP-14-1-oder-2-St%C3%BCck/172410722963?ssPageName=STRK%3AMEBIDX%3AIT&var=471240518309&_trksid=p2057872.m2749.l2649
  https://www.petervis.com/GCSE_Design_and_Technology_Electronic_Products/nand-gate-timers/nand-gate-timer-delay-on/cd4093b-pinout.gif

  Halterung Ultraschallsensor:
  https://www.thingiverse.com/thing:189585

  Abdeckung Getriebe:
  https://www.thingiverse.com/thing:2985540

*/

// Gleichstrommotor 1 (Rechts)
byte GSM1 = 10;
byte in1 = 9;
byte in2 = 8;

// Gleichstrommotor 2 (Links)
byte GSM2 = 5;
byte in3 = 7;
byte in4 = 6;

// Motorgeschwindigkeiten
int motorvarR = 160; // Rechter Motor (160)
int motorvarL = 215; // Linker Motor (210/207)

// Sharp Infrarotsensoren
int ir_entf = 209;  // 209 (Wand im Flur)
byte ir1 = 14; // Sharp IR Entfernungssensor vorne
byte ir2 = 15; // Sharp IR Entfernungssensor links
byte ir3 = 17; // Sharp IR Entfernungssensor rechts
int ir_delay = 50; // Wartezeit zwischen den Ausleseversuchen des IRSensors

// Gyroskop
MPU6050 mpu6050(Wire); // Gyro soll mithilfe der Wire Bibliothek funktionieren
int gradcounter = 0; // Drehvariable
bool gyro = 0; // Rückgabe der Funktion gyro_sensor
int gyro_delay = 50; // Wartezeit zwischen den Ausleseversuchen des Gyroskops

// Endschalter / Bumper (Failsafe, im idealen Normalfall nicht genutzt)
byte bumpPin1 = 3; // Vorne

// Zeitrelevant
const unsigned short sec = 1000;  // Dauer von 1 Sekunde

// Funktionsprototypen

// Sensorfunktionen
int ir_sensor(int); // Input: Sensornummer | Rückgabe: Integer
bool gyro_sensor(signed short int angle, int grad); // Input: Aktueller Winkel, zu Drehende Gradzahl | Rückgabe: Bool

// Tasterfunktion
bool bumpers(); // Input: Nichts | Rückgabe: Gedrückt o. Ungedrückt

// Drehfunktion
void turn(int, int); // Input: Gradzahl, Korrigieren o. Drehung | Rückgabe: Nichts

// Wartefunktion (Ersatz für delay();)
void wartezeit(unsigned short); // Input: Dauer | Rückgabe: Nichts

// ######################## SETUP ########################
// Einmalig ausgeführte Festlegungen
void setup()
{
  Serial.begin(9600); // Startet die Serielle Schnittstelle
  Serial.println("##PROGRAMMSTART##");

  // Festlegung der Pins
  pinMode(GSM1, OUTPUT); // Gleichstrommotor 1
  pinMode(GSM2, OUTPUT); // Gleichstrommotor 2

  pinMode(in1, OUTPUT); // GSM1 input1
  pinMode(in2, OUTPUT); // GSM1 input2

  pinMode(in3, OUTPUT); // GSM2 input1
  pinMode(in4, OUTPUT); // GSM2 input2

  pinMode(bumpPin1, INPUT_PULLUP); // Bumper 1, sicherheitshalber inkl. Pullup

  pinMode (ir1, INPUT); // Sharp IR Entfernungssensor vorne
  pinMode (ir2, INPUT); // Sharp IR Entfernungssensor links
  pinMode (ir3, INPUT); // Sharp IR Entfernungssensor rechts

  // GYROSKOP
  Serial.println("##GYRO...##");
  Wire.begin(); // Initialisierung der Wire Bibliothek
  mpu6050.begin(); // Initialisierung der Gyroskop Bibliothek
  mpu6050.calcGyroOffsets(true); // Kalibrierung des Gyroskops soll erfolgen
  Serial.println("\n##CALIBRATED!##");

}
// ######################## LOOP ########################
// Auszuführen in Dauerschleife:
void loop()
{
  int sensorvar = 0;
  int sensorvar2 = 0;
  int leftvar = 0;
  int rightvar = 0;

  // Motorengeschwindigkeit festlegen
  analogWrite(GSM1, motorvarR); // Rechter Motor
  analogWrite(GSM2, motorvarL); // Linker Motor

  Serial.println("##LOOPSTART##");

  // Beginnen der Vorwärtsfahrt
  // Motor 1 vorwärts
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // Motor 2 vorwärts
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Läuft solange wie, weder der vordere IR Sensor, noch die Bumper ein Hindernis erkannt haben
  while ((sensorvar == 0) && (sensorvar2 == 0)) {
    sensorvar = ir_sensor(0); // Sensor 0 = Vorne

    // Checke Entfernung links
    leftvar = ir_sensor(1);
    
    // Wenn Entfernung links zu gering
    while (leftvar == 1) {
      digitalWrite(12, HIGH); // LED Rot ein
      // Korrigieren
      turn(25, 1); // 25°
      digitalWrite(12, LOW); // LED Rot aus
      leftvar = 0;
    }
    // Checke Entfernung rechts
    rightvar = ir_sensor(2);
    // Wenn Entfernung rechts zu gering
    while (rightvar == 1) {
      digitalWrite(11, HIGH); // LED Grün ein
      // Korrigieren
      turn(-35, 2); // -35°
      digitalWrite(11, LOW); // LED Grün aus
      rightvar = 0;
    }
    // wenn entfernung groß genug
    // setze motorgeschw zurück
    analogWrite(GSM1, motorvarR); // Rechter Motor
    analogWrite(GSM2, motorvarL); // Linker Motor
    sensorvar2 = bumpers(); // Taster vorne überprüfen
  }
  // Anhalten
  // Motor 1 aus
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  // Motor 2 aus
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  wartezeit(50);
  // Unterscheiden zwischen 90° und 180° Drehung
  // Winkel sind nicht akkurat, dswg. ist 90° im "echten Leben" hier fast das doppelte

  switch (gradcounter) {
    case 0:
      // 90 grad drehen
      digitalWrite(11, HIGH); // LED Grün ein
      digitalWrite(12, LOW); // LED Rot aus

      turn(170, 0); // 90° -> 170

      gradcounter++; // Drehvariable auf 1 setzen (1 addieren)
      digitalWrite(11, LOW); //LED Grün aus
      break;
    case 1:
      // 180 Grad drehen
      digitalWrite(11, LOW); // LED Grün aus
      digitalWrite(12, HIGH); // LED Rot ein

      turn(320, 0); // 180° -> 360-320

      gradcounter--; // Drehvariable auf 0 setzen (1 subtrahieren)
      digitalWrite(11, LOW); // LED Grün ein
      digitalWrite(12, LOW); // LED Rot aus
      break;
    default:
      // keine Reaktion
      break;
  }
}
// ######################## IR_Sensor ########################
int ir_sensor(int sensornr) {

  int irtemp = 0; // Temporäre Sensornummmer
  int distancetemp = 0; // Temporärer Entfernungswert
  int sumtemp = 0; // Temporärer Entfernungs-Summenwert

  int repeat = 4; // Anzahl der Mehrfachauslesungen

  int entfrange = 70; // Entfernungsbereich für Auslesungen

  switch (sensornr)
  {
    // Vorne
    case 0:
      irtemp = ir1;
      entfrange = 50; // Vorne weniger sensibel, da sonst Wand zuerst von vorne erkannt
      break;
    // Links
    case 1:
      irtemp = ir2;
      break;
    // Rechts
    case 2:
      irtemp = ir3;
      break;
    default:
      // not happening
      break;
  }
  distancetemp = analogRead(irtemp); // Entfernungswert des Sensors übergeben

  // Fehlereinlesungen umgehen
  if ((distancetemp == 0) || (distancetemp == -1)) {
    distancetemp = 100;
  }
  // Mehrfachauslesung der Entfernung
  for (int i = 0; i < repeat; i++)
  {
    // Entfernung auslesen
    distancetemp = analogRead(irtemp); // Entfernungswert des Sensors übergeben
    sumtemp += distancetemp; // Entfernungswert zur Summe addieren
    wartezeit(ir_delay);
  }
  sumtemp = sumtemp / repeat; // Mittelwert ausrechnen
  //Entfernung < ca. 20cm +- X? (Invertierte logik!)
  if ((ir_entf - entfrange < sumtemp) && (ir_entf + entfrange > sumtemp))
  {
    sumtemp = 0; // Summe für nächste Berechnung zurücksetzen
    distancetemp = analogRead(irtemp); // Entfernungswert des Sensors übergeben
    // Mehrfachauslesung der Entfernung
    for (int i = 0; i < repeat; i++)
    {
      distancetemp = analogRead(irtemp); // Entfernungswert des Sensors übergeben
      sumtemp = sumtemp + distancetemp; // Entfernungswert zur Summe addieren
      if ((distancetemp == 0) || (distancetemp == -1)) {
        distancetemp = 100;
      }
      wartezeit(ir_delay);
    }
    sumtemp = sumtemp / repeat; // Mittelwert ausrechnen
    sumtemp = sumtemp + 50; // DEBUG TEST

    // Wenn der Mittelwert kleiner als die ir_entf ist.. (Invertierte logik!)
    if (sumtemp > ir_entf)
    {
      //Entfernung zu nah
      return 1;
    }
  }
  else {
    // Entfernung noch OK
    return 0;
  }
}
// ######################## BUMPER ########################
bool bumpers() {
  int value = digitalRead(bumpPin1);
  int i = 0;
  if (value == 0) {
    // Gegengefahren
    for (i = 0; i < 10; i++) {
      // Motor 1 aus
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 aus
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);

      return 1;
    }
  }
  else {
    // Kein Kontakt
    return 0;
  }
}
// ######################## TURN ########################
void turn(int grad, int korrig) {
  mpu6050.update(); // Werte des Sensors aktualisieren
  signed short int cur_angle = mpu6050.getAngleZ(); // Winkel beim Starten der Funktion

  // Motorengeschwindigkeit festlegen
  analogWrite(GSM1, motorvarR); // Rechter Motor
  analogWrite(GSM2, motorvarL); // Linker Motor
  switch (korrig)
  {
    case 0:
      // Motor 1 rückwärts
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      // Motor 2 rückwärts
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);

      delay(2000); // "Fahre für 2 Sekunden"

      // Wartezeitfunktion spontan seit der Abnahme defekt, dswg delay darüber.
      // Bei anderen Funktionen ist kein wirklicher Unterschied ohne wartezeit erkennbar, dswg. wurde es dort weggelassen
      //wartezeit(2000);

      // Motor 1 aus
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 aus
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);

      // Rechtsdrehung
      // Motor 1 rückwärts
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      // Motor 2 vorwärts
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);

      gyro = 0;
      while (gyro == 0) {
        gyro = gyro_sensor(cur_angle, grad);
      }

      // Anhalten
      // Motor 1 aus
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 aus
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);

      wartezeit(500);
      break;
    case 1:
      // Links SENSOR
      // Motor 1 aus
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 aus
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);

      // Motorengeschwindigkeit festlegen
      analogWrite(GSM1, motorvarR); // Rechter Motor
      analogWrite(GSM2, motorvarL); // Linker Motor

      // Rechtsdrehung
      // Motor 1 vorwärts
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      // Motor 2 rückwärts
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);

      gyro = 0;
      while (gyro == 0) {
        gyro = gyro_sensor(cur_angle, grad);
      }
      // Anhalten
      // Motor 1 aus
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 aus
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      // Motor 1 vorwärts
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      // Motor 2 vorwärts
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      break;
    case 2:
      // RECHTS SENSOR
      // Motor 1 aus
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 aus
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);

      // Motorengeschwindigkeit festlegen
      analogWrite(GSM1, motorvarR); // Rechter Motor
      analogWrite(GSM2, motorvarL); // Linker Motor

      // Linksdrehung
      // Motor 1 vorwärts
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      // Motor 2 rückwärts
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);

      gyro = 0;
      while (gyro == 0) {
        gyro = gyro_sensor(cur_angle, grad);
      }
      // Anhalten
      // Motor 1 aus
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 aus
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);

      // Motor 1 vorwärts
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      // Motor 2 vorwärts
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      break;
    default:
      // Not happening
      break;
  }
  gyro = 0;
}
// ######################## GYRO ########################
// Überprüfe ob Drehung vollendet, wenn ja gebe "1" zurück
bool gyro_sensor(signed short int angle, int grad) {
  // Angle = Winkel beim Starten der Funktion
  // Grad = Zu erreichender Winkel(Gradzahl)
  signed short int cur_angle = angle; // Aktueller Wert

  if (grad < 0) {
    // Linksdrehung
    // Warte, bis das Gyroskop eine vollständige X° Drehung erfasst hat (<-180)
    while (cur_angle <= angle + grad) {
      mpu6050.update(); // Werte des Sensors aktualisieren
      cur_angle = mpu6050.getAngleZ(); // Werte der Z-Achse (Drehwinkel) übergeben
      wartezeit(gyro_delay);
    }
  }
  else {
    // Rechtsdrehung
    // Warte, bis das Gyroskop eine vollständige X° Drehung erfasst hat (<-180)
    while (cur_angle >= angle - grad) {
      mpu6050.update(); // Werte des Sensors aktualisieren
      cur_angle = mpu6050.getAngleZ(); // Werte der Z-Achse (Drehwinkel) übergeben
      wartezeit(gyro_delay);
    }
  }
  // Drehung abgeschlossen
  return 1;
}
// ######################## WARTEZEIT ########################
// Ersatz für delay() mithilfe millis()
void wartezeit(unsigned short dauer) {
  unsigned short currentMillis = 0; // Aktueller Zeitpunkt
  unsigned short startMillis = (unsigned short)millis(); // Startpunkt der Wartefunktion in ms

  // Führe solange aus, wie die Dauer noch nicht verstrichen ist
  while (currentMillis - startMillis <= dauer) {
    currentMillis = (unsigned short)millis(); // Aktueller Zeitpunkt in ms
  }
}
