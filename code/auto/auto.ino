#include <MPU6050_tockn.h> // Bibliothek für Gyroskop
#include <Wire.h> // Voraussetzung für Gyroskop 
/*
   Autonomer Roboter "Gary"
   Version 1.3
   Made by:
   David Schmidt, Max Grüning, Steven Bolln, Pascal Harders
   Feb-Mar 2019

   TODO:

  fix wartezeit funktion

  Codecleanup
  Langsam stoppen um Getriebe/Motor zu schonen?

  Fehlercodes:

  LED ROT:
  Ein (während Rechtsdrehung): Entfernung links zu gering
  Ein (während Rechtsdrehung): 180° Drehung
  Blinkend: Bumperkontakt erkannt

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

  Interrupt Handler:
  https://forum.arduino.cc/index.php?topic=45000.msg325940#msg325940

*/

// Gleichstrommotor 1 (Rechts)
byte GSM1 = 10;
byte in1 = 9;
byte in2 = 8;

// Gleichstrommotor 2 (Links)
byte GSM2 = 5;
byte in3 = 7;
byte in4 = 6;

int motorvarR = 160; // Rechter Motor (160)
int motorvarL = 215; // Linker Motor (210/207)

// Endschalter / Bumper (Failsafe, im Normalfall nicht genutzt)
byte bumpPin1 = 3; // Vorne
//byte bumpPin2 = 2; // Hinten // UNUSED

// Sharp Infrarotsensoren
int ir_entf = 209;  // 209 (Schule)
int ir_entf2 = 150; // 170 war eben
byte ir1 = 14; // Sharp IR Entfernungssensor vorne
byte ir2 = 15; // Sharp IR Entfernungssensor links
byte ir3 = 17; // Sharp IR Entfernungssensor rechts
int ir_delay = 50; // Wartezeit zwischen den Ausleseversuchen des IRSensors

// Zeitrelevant
const unsigned short sec = 1000;  // Dauer von 1 Sekunde

// Interruptbezogen
volatile bool IR_flag1 = 0; // Interruptflag 1/0, wenn 1 dann wurde Interrupt1 (bump_ISR) ausgelöst

// Gyroskop
MPU6050 mpu6050(Wire);
int gradcounter = 0; // Drehvariable
bool gyro = 0; // Rückgabe der Funktion gyro_sensor
int gyro_delay = 50; // Wartezeit zwischen den Ausleseversuchen des Gyroskops

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

// DebugLEDS
//ROT: 12
//GRÜN: 11



// ######################## SETUP ########################
// Einmalig ausgeführte Festlegungen
void setup()
{
  Serial.begin(9600); // Startet die Serielle Schnittstelle
  Serial.println("##PROGRAMMSTART##"); // DEBUG ONLY

  // Festlegung der Pins
  pinMode(GSM1, OUTPUT); // Gleichstrommotor 1
  pinMode(GSM2, OUTPUT); // Gleichstrommotor 2

  pinMode(in1, OUTPUT); // GSM1 input1
  pinMode(in2, OUTPUT); // GSM1 input2

  pinMode(in3, OUTPUT); // GSM2 input1
  pinMode(in4, OUTPUT); // GSM2 input2

  pinMode(bumpPin1, INPUT_PULLUP); // Bumper 1

  pinMode (ir1, INPUT); // Sharp IR Entfernungssensor vorne
  pinMode (ir2, INPUT); // Sharp IR Entfernungssensor links
  pinMode (ir3, INPUT); // Sharp IR Entfernungssensor rechts

  // GYROSKOP
  Serial.println("##GYRO...##"); // DEBUG ONLY
  Wire.begin(); // Initialisierung der Wire Bibliothek
  mpu6050.begin(); // Initialisierung der Gyroskop Bibliothek
  mpu6050.calcGyroOffsets(true); // Kalibrierung des Gyroskops soll erfolgen
  Serial.println("\n##CALIBRATED!##"); // DEBUG ONLY

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

  Serial.println("MOVE - INITIAL START"); // DEBUG ONLY

  // Motor 1 vorwärts
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // Motor 2 vorwärts
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  while ((sensorvar == 0) && (sensorvar2 == 0)) {
    //while (sensorvar == 0) {
    sensorvar = ir_sensor(0); // Sensor 0 = Vorne
    Serial.print("SENSORVAR: ");
    Serial.println(sensorvar);

    //sensorvar1 = bumpers(); // Taster vorne checken
    // Checke entfernung links
    leftvar = ir_sensor(1);
    // wenn entfernung links zu gering
    while (leftvar == 1) {
      digitalWrite(12, HIGH); // LED Rot an
      // Korrigieren
      turn(35, 1); // 20°
      digitalWrite(12, LOW); // LED Rot aus
      leftvar = 0;
    }
    // Checke entfernung rechts
    rightvar = ir_sensor(2);
    // wenn entfernung rechts zu gering
    while (rightvar == 1) {
      digitalWrite(11, HIGH); // LED Grün an
      // Korrigieren
      turn(-35, 2); // -20°
      digitalWrite(11, LOW); // LED Grün aus
      rightvar = 0;
    }
    // wenn entfernung groß genug
    // setze motorgeschw zurück
    analogWrite(GSM1, motorvarR); // Rechter Motor
    analogWrite(GSM2, motorvarL); // Linker Motor
    sensorvar2 = bumpers(); // Taster vorne überprüfen

    Serial.print("SENSORVAR2: ");
    Serial.println(sensorvar2);

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

  switch (gradcounter) {
    case 0:
      // 90 grad drehen
      digitalWrite(11, HIGH); // LED Grün ein
      digitalWrite(12, LOW); // LED Rot aus

      turn(170, 0); // 90° -> 170,

      Serial.println("DREH - 90 grad gedreht"); // DEBUG ONLY
      gradcounter++; // Drehvariable auf 1 setzen (1 addieren)
      digitalWrite(11, LOW); //LED Grün aus
      Serial.println("Gradcounter nachher 90:"); // DEBUG ONLY
      Serial.println(gradcounter); // DEBUG ONLY
      break;
    case 1:
      // 180 Grad drehen
      digitalWrite(11, LOW); // LED Grün aus
      digitalWrite(12, HIGH); // LED Rot ein

      turn(320, 0); // 180° -> 360-320

      Serial.println("DREH - 180 grad gedreht"); // DEBUG ONLY
      gradcounter--; // Drehvariable auf 0 setzen (1 subtrahieren)
      digitalWrite(11, LOW); // OK LED ein
      digitalWrite(12, LOW); // LED Rot aus
      Serial.println("Gradcounter nachher 180:"); // DEBUG ONLY
      Serial.println(gradcounter); // DEBUG ONLY
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
      Serial.print("IR_SENS - Vorne:"); // DEBUG ONLY
      break;
    // Links
    case 1:
      irtemp = ir2;
      Serial.print("IR_SENS - Links:"); // DEBUG ONLY
      break;
    // Rechts
    case 2:
      irtemp = ir3;
      Serial.print("IR_SENS - Rechts:"); // DEBUG ONLY
      break;
    default:
      // not happening
      break;
  }
  distancetemp = analogRead(irtemp); // Entfernungswert des Sensors übergeben
  Serial.print("IR_SENS - INITIAL Distance:"); // DEBUG ONLY
  Serial.println(distancetemp); // DEBUG ONLY

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
  //Entfernung < 20cm +- X? (Invertierte logik!)
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
      Serial.print("IR_SENS - Entfernung - SUMTEMP: "); // DEBUG ONLY
      Serial.print(sumtemp); // DEBUG ONLY
      Serial.print(" - DISTEMP: "); // DEBUG ONLY
      Serial.print(distancetemp); // DEBUG ONLY
      Serial.print(" - i: "); // DEBUG ONLY
      Serial.print(i); // DEBUG ONLY
      Serial.print(" - SUM/I: "); // DEBUG ONLY
      Serial.println(sumtemp / i + 1); // DEBUG ONLY
      wartezeit(ir_delay);
    }
    sumtemp = sumtemp / repeat; // Mittelwert ausrechnen
    sumtemp = sumtemp + 50; // DEBUG TEST
    Serial.println("###################"); // DEBUG ONLY
    Serial.println(entfrange); // DEBUG ONLY
    Serial.println("###################"); // DEBUG ONLY

    Serial.print("IR_SENS - Entfernung ERGEBNIS: "); // DEBUG ONLY
    Serial.println(sumtemp); // DEBUG ONLY

    Serial.print("IR_SENS - Entfernung ERGEBNIS: "); // DEBUG ONLY
    Serial.println(ir_entf); // DEBUG ONLY

    // Wenn der Mittelwert kleiner als die ir_entf ist.. (Invertierte logik!)
    if (sumtemp > ir_entf)
    {
      Serial.println("IR_SENS - Ergebnis > ir_entf"); // DEBUG ONLY
      return 1;
    }
  }
  else {
    Serial.println("IR_SENS - ENTF OK"); // DEBUG ONLY
    return 0;
  }
}
// ######################## BUMPER ########################
bool bumpers() {
  int value = digitalRead(bumpPin1);
  int i = 0;
  if (value == 0) {
    Serial.println("BUMP - Gegengefahren"); // DEBUG ONLY
    for (i = 0; i < 10; i++) {
      digitalWrite(12, HIGH); // LED Rot ein
      wartezeit(150);
      digitalWrite(12, LOW); // LED Rot aus
      wartezeit(150);

      // Motor 1 aus
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 aus
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);

      return 1; // Gegengefahren
    }
  }
  else {
    Serial.println("BUMP - OK"); // DEBUG ONLY
    return 0; // Kein Kontakt
  }
}
// ######################## TURN ########################
void turn(int grad, int korrig) {
  mpu6050.update(); // Werte des Sensors aktualisieren
  signed short int cur_angle = mpu6050.getAngleZ(); // Winkel beim Starten der Funktion
  Serial.print("TURN INITIAL -");  Serial.println(cur_angle); // DEBUG ONLY
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
      delay(2000);
      wartezeit(2000); // <- DAS IST DER FEHLER
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
      Serial.println("Turn Rechtsssensor");
      // Motor 1 aus
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 aus
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);

      //    wartezeit(3000);
      // Motorengeschwindigkeit festlegen
      analogWrite(GSM1, motorvarR); // Rechter Motor
      analogWrite(GSM2, motorvarL); // Linker Motor

      // Linksdrehung
      // Motor 1 vorwärts
      Serial.println("Mot1 Rückwärts");
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      // Motor 2 rückwärts
      Serial.println("Mot2 Vorwärts");
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      //wartezeit(1000);
      gyro = 0;
      while (gyro == 0) {
        gyro = gyro_sensor(cur_angle, grad);
        Serial.println("Gyrorechtsdrehung..");
      }
      // Anhalten
      // Motor 1 aus
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 aus
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      //wartezeit(1000);
      // Motor 1 vorwärts
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      // Motor 2 vorwärts
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      break;
    case 2:
      // RECHTS SENSOR
      Serial.println("Turn Rechtsssensor");
      // Motor 1 aus
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 aus
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);

      //    wartezeit(3000);
      // Motorengeschwindigkeit festlegen
      analogWrite(GSM1, motorvarR); // Rechter Motor
      analogWrite(GSM2, motorvarL); // Linker Motor

      // Linksdrehung
      // Motor 1 vorwärts
      Serial.println("Mot1 Vorwärts");
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      // Motor 2 rückwärts
      Serial.println("Mot2 Rückwärts");
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      //wartezeit(1000);
      gyro = 0;
      while (gyro == 0) {
        gyro = gyro_sensor(cur_angle, grad);
        Serial.println("Gyrolinksdrehung..");
      }
      // Anhalten
      // Motor 1 aus
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 aus
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      //wartezeit(1000);
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
// Überprüfe ob Drehung vollendet
bool gyro_sensor(signed short int angle, int grad) {
  // Angle = Winkel beim Starten der Funktion
  // Grad = Zu erreichender Winkel(Gradzahl)
  signed short int cur_angle = angle; // Aktueller Wert
  Serial.print("GYRO INIT CURANGLE -");  Serial.println(cur_angle); // DEBUG ONLY
  Serial.print("GYRO INIT ANGLE -");  Serial.println(angle); // DEBUG ONLY

  if (grad < 0) {
    // Linksdrehung
    Serial.print("GYRO LINKSDREHUNG"); // DEBUG ONLY
    // Warte, bis das Gyroskop eine vollständige X° Drehung erfasst hat (<-180)
    while (cur_angle <= angle + grad) {
      mpu6050.update(); // Werte des Sensors aktualisieren
      cur_angle = mpu6050.getAngleZ(); // Werte der Z-Achse (Drehwinkel) übergeben
      wartezeit(gyro_delay);
      Serial.print("GYRO -");  Serial.println(cur_angle); // DEBUG ONLY
    }
  }
  else {
    // Rechtsdrehung
    Serial.print("GYRO RECHTSDREHUNG"); // DEBUG ONLY
    // Warte, bis das Gyroskop eine vollständige X° Drehung erfasst hat (<-180)
    while (cur_angle >= angle - grad) {
      mpu6050.update(); // Werte des Sensors aktualisieren
      cur_angle = mpu6050.getAngleZ(); // Werte der Z-Achse (Drehwinkel) übergeben
      wartezeit(gyro_delay);
      Serial.print("GYRO -");  Serial.println(cur_angle); // DEBUG ONLY
    }
  }
  Serial.print(grad); // DEBUG ONLY
  Serial.println("° Drehung abgeschlossen"); // DEBUG ONLY
  return 1; // Drehung abgeschlossen
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
