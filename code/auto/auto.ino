#include <MPU6050_tockn.h> // Bibliothek für Gyroskop
#include <Wire.h> // Voraussetzung für Gyroskop 
/*
   Autonomer Roboter "Gary"
   Version 0.9
   Made by:
   David Schmidt, Max Grüning, Steven Bolln, Pascal Harders
   Feb-Mar 2019

   TODO:
  Interrupt ändern / entfernen
  do drive vorwärts while wert von IR nicht doof
  Hinderniserkennung beim Rückwärtsfahren?
  Codecleanup
  Langsam stoppen um Getriebe/Motor zu schonen?

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

  OR-Gate für Endschalter:
  https://www.ebay.de/itm/5x-CMOS-4071-OR-Gatter-4-fach-2-Eingänge-C-MOS-IC-DIP14/311014657148

  Halterung Ultraschallsensor:
  https://www.thingiverse.com/thing:189585

  Abdeckung Getriebe:
  https://www.thingiverse.com/thing:2985540

  Interrupt Handler:
  https://forum.arduino.cc/index.php?topic=45000.msg325940#msg325940

*/

// Gleichstrommotor 1
byte GSM1 = 10;
byte in1 = 9;
byte in2 = 8;

// Gleichstrommotor 2
byte GSM2 = 5;
byte in3 = 7;
byte in4 = 6;

// Endschalter / Bumper (Failsafe, im Normalfall nicht genutzt)
byte bumpPin1 = 3; // Vorne
//byte bumpPin2 = 2; // Hinten // UNUSED

// Sharp Infrarotsensoren
int ir_entf = 209;  // 20cm = ca. 333 (David @Home) 209 (Schule)
byte ir1 = 14; // Sharp IR Entfernungssensor vorne
byte ir2 = 15; // Sharp IR Entfernungssensor hinten
int ir_delay = 50; // Wartezeit zwischen den Ausleseversuchen des IRSensors

// Zeitrelevant
const unsigned short halfsec = 500;  // Dauer von 0.5 Sekunden
const unsigned short ngrad = 900;  // Dauer von 0.67/0.8 Sekunden, eine 90° Drehung
const unsigned short oneeightyg = 1100;  // Dauer von 1.1 Sekunden, eine 180° Drehung
const unsigned short onesec = 1000;  // Dauer von 1 Sekunde

// Interruptbezogen
volatile bool IR_flag1 = 0; // Interruptflag 1/0, wenn 1 dann wurde Interrupt1 (bump_ISR) ausgelöst

// Gyroskop
MPU6050 mpu6050(Wire);
int gradcounter = 0; // Drehvariable

// Funktionsprototypen

// Bewegungsfunktion
void movecar(byte, const unsigned short); // Input: Richtung, Dauer | Rückgabe: Nichts

// Sensorfunktion
void irsensor(); // Input: Nichts | Rückgabe: Nichts

// Wartefunktion (Ersatz für delay();)
void wartezeit(unsigned short); // Input: Dauer | Rückgabe: Nichts


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

  // pinMode(infraR1, INPUT); // Quick-Infrarot 1

  pinMode (ir1, INPUT); // Sharp IR Entfernungssensor vorne
  pinMode (ir2, INPUT); // Sharp IR Entfernungssensor hinten

  // Interrupt UNUSED

  //attachInterrupt(digitalPinToInterrupt(bumpPin1), bump_ISR, FALLING); // Interrupt1 wenn HIGH -> LOW vom Bumper1


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
  /*
     Richtungen für die move() Funktion:

     Richtungen:   (Nr)
     Vorwärts:      1
     Rückwärts:     3
     Rechtsdreh.:   5
     Anhalten:      9
  */
  // Während kein Interrupt ausgelöst wurde..
  while (IR_flag1 == 0) {
    Serial.print("Bump info - "); Serial.println(digitalRead(bumpPin1)); // 0 = Pressed; 1 = Unpressed // DEBUG ONLY

    // Gyroskop Startposition aktualisieren (Winkel ~0°)
    mpu6050.update();
    Serial.print("Winkel: ");// DEBUG ONLY
    Serial.println(mpu6050.getAngleZ());// DEBUG ONLY

    irsensor(); // Infrarotsensoren auslesen
    movecar(1, halfsec); // Vorwärts fahren für X Sekunden

  }// .. führe oberes aus(Interruptflag = 0)

  // Wenn ein Interruptflag gesetzt wurde
  if (IR_flag1 == 1)
  {

    Serial.println("Bumper ausgelöst"); // DEBUG ONLY
    Serial.println("IRF - stop,back2s"); // DEBUG ONLY

    movecar(9, halfsec); // Anhalten
    movecar(3, onesec * 2); // Hinten, 2sek
    IR_flag1 = 0; // Interruptflag zurücksetzen
    Serial.println("##INTERRUPT DONE/RESET##"); // DEBUG ONLY
  }
  else {
    // do nothing
  }
}
// ######################## INTERRUPT VON BUMPER ########################
// Sobald "gegen" ein Hindernis gefahren wird
void bump_ISR() {
  IR_flag1 = 0;
  static unsigned short last_interrupt_time = 0; // Zeitpunkt des letzten Interrupts
  unsigned short interrupt_time = (unsigned short)millis(); // Zeitpunkt des aktuellen Interrupts


  // Wenn ein weiterer Interrupt schneller als X ms seit dem letzten kommt, oder ein Interrupt bereits läuft, wird ein neuer Interrupt ignoriert
  if (interrupt_time - last_interrupt_time > 1000)
  {
    Serial.println("##INTERRUPT - STOP##"); // DEBUG ONLY
    // Motor 1 aus
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    // Motor 2 aus
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    IR_flag1 = 1;
    Serial.println("##STOPPED##"); // DEBUG ONLY
    //Serial.print("IRS - Last_Interrupt: "); Serial.print(last_interrupt_time); Serial.print("| interrupt_time - "); Serial.print(interrupt_time); Serial.print(" Last-Current - "); Serial.println(interrupt_time - last_interrupt_time); // DEBUG ONLY
  }
  // Wenn Mehrfachauslösungen erkannt wurden..
  else if (IR_flag1 == 1) {
    // .. warte
    Serial.println("##INTERRUPT## - Debounce"); // DEBUG ONLY
    Serial.print("Bump info - "); Serial.println(digitalRead(bumpPin1)); // DEBUG ONLY
  }
  last_interrupt_time = interrupt_time; // Aktuellen Interruptzeitpunkt als letzten Zeitpunkt speichern


}
// ######################## BEWEGUNGSFUNKTION ########################
void movecar(byte richtung, const unsigned short dauer) {
  /*
    GSM1 / Motor 1 = Rechts | GSM2 / Motor 2 = Links
    Hinweis: Motor 2 dreht andersherum als Motor 1
    d.h. die Befehle bei Motor2 sind invertiert zum Motor 1

    Bei gleicher Geschwindigkeitsvariable fährt der Roboter nach links, dswg GSM2 etwas schneller
  */
  bool motaus = 0; // Variable für Motor Ausschalterkennung

  mpu6050.update(); // Werte des Sensors aktualisieren

  signed short int angle = mpu6050.getAngleZ(); // Winkel beim Starten der Funktion
  signed short int cur_angle = angle; // Aktueller Winkel

  // Motorengeschwindigkeit festlegen
  analogWrite(GSM1, 160); // Rechter Motor (160)
  analogWrite(GSM2, 200); // Linker Motor (240)
  Serial.println("MOVE - Start"); // DEBUG ONLY

  //###########Vorwärts###########
  switch (richtung) {
    case 1:
      // Motor 1 vorwärts
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      // Motor 2 vorwärts
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      break;

    //###########Rückwärts###########
    case 3:
      // Motor 1 rückwärts
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      // Motor 2 Rückwärts
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      break;

    //###########Rechtsdrehung###########
    case 5:
      // Motor 1 rückwärts
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      // Motor 2 vorwärts
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      break;

    //###########Anhalten###########
    case 9:
      // Motor 1 aus
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 aus
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      motaus = 1;
      break;

    //###########FAILSAFE###########
    default:
      // Falls die Werteübergabe nicht korrekt funktioniert, dann..
      // Motor 1 aus
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 aus
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      break;
  }
  // Wenn eine Rechtsdrehung erfolgen soll..
  if (richtung == 5) {
    // .. und die Wartedauer einer "180°-Drehungs-Dauer" entspricht..
    if (dauer == oneeightyg) {
      // Warte, bis das Gyroskop eine vollständige 180° Drehung erfasst hat (<-180)
      while (cur_angle >= angle - 180) {
        mpu6050.update(); // Werte des Sensors aktualisieren
        cur_angle = mpu6050.getAngleZ(); // Werte der Z-Achse (Drehwinkel) übergeben
        wartezeit(ir_delay);
        Serial.print("SIR(180) -");  Serial.println(cur_angle); // DEBUG ONLY
      }
    }
    // .. oder die Wartedauer eine "90°-Drehungs-Dauer" entspricht..
    else if (dauer == ngrad)
      // Warte, Bis das Gyroskop eine vollständige 90° Drehung erfasst hat (<-90)
      while (cur_angle >= angle - 93) {
        mpu6050.update(); // Werte des Sensors aktualisieren
        cur_angle = mpu6050.getAngleZ(); // Werte der Z-Achse (Drehwinkel) übergeben
        wartezeit(ir_delay);
        Serial.print("SIR(90) -");  Serial.println(cur_angle); // DEBUG ONLY
      }
    Serial.println("Drehung abgeschlossen"); // DEBUG ONLY
  }
  else {
    wartezeit(dauer); // Warte X ms
  }

  //###########Bewegung stoppen###########
  // Wenn der Motor noch nicht ausgeschaltet wurde, dann..
  if (motaus == 0) {
    // Motor 1 aus
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    // Motor 2 aus
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  // .. wenn doch
  else {
    // Variable zurücksetzen
    motaus = 0;
  }
  Serial.println("MOVE - Done"); // DEBUG ONLY
}
// ######################## LESEN DER SHARP IR SENSOREN ########################
void irsensor() {
  long distanceV = 1; // Entfernung vorne
  long distanceH = 0; // Entfernung hinten DEBUG ONLY UNUSED
  int sumV = 0; // Summe der Entfernungen vorne
  int repeat = 4; // Anzahl der Mehrfachauslesungen
  int repeat1 = 4; // 2. Anzahl der Mehrfachauslesungen

  // Entfernung auslesen
  distanceV = analogRead(ir1);
  distanceH = analogRead(ir2); // DEBUG ONLY UNUSED
  Serial.print("MAX - DV:"); // DEBUG ONLY
  Serial.println(distanceV); // DEBUG ONLY
  Serial.print("MAX - DH:"); // DEBUG ONLY
  Serial.println(distanceH); // DEBUG ONLY

  // Fehlereinlesungen umgehen
  if ((distanceV == 0) || (distanceV == -1)) {
    distanceV = 100;
  }
  // Mehrfachauslesung der Entfernung
  for (int i = 0; i < repeat1; i++)
  {
    // Entfernung auslesen
    distanceV = analogRead(ir1); // Entfernungswert des Sensors übergeben
    sumV += distanceV; // Entfernungswert zur Summe addieren
    Serial.print("Wirklicher Wert? - "); // DEBUG ONLY
    Serial.println(sumV / i); // DEBUG ONLY
    wartezeit(ir_delay);
  }
  sumV = sumV / repeat1; // Mittelwert ausrechnen

  //Entfernung < 20cm +- X? (Invertierte logik!)
  if ((ir_entf - 50 < sumV) && (ir_entf + 50 > sumV))
  {
    sumV = 0; // Summe für nächste Berechnung zurücksetzen
    distanceV = analogRead(ir1); // Entfernungswert des Sensors übergeben
    Serial.print("MAX - KLEINER 20 DV:"); // DEBUG ONLY
    Serial.println(distanceV); // DEBUG ONLY
    movecar(9, halfsec); // Anhalten
    Serial.println("MAX - ANGEHALTEN"); // DEBUG ONLY

    // Mehrfachauslesung der Entfernung
    for (int i = 0; i < repeat; i++)
    {
      distanceV = analogRead(ir1); // Entfernungswert des Sensors übergeben
      sumV += distanceV; // Entfernungswert zur Summe addieren
      Serial.print("MAX - Entfernung vorne - "); // DEBUG ONLY
      Serial.println(sumV / i); // DEBUG ONLY
      wartezeit(ir_delay);
    }
    sumV = sumV / repeat; // Mittelwert ausrechnen

    Serial.print("MAX - Entfernung ERGEBNIS: "); // DEBUG ONLY
    Serial.println(sumV); // DEBUG ONLY

    // Wenn der Mittelwert kleiner als die ir_entf ist.. (Invertierte logik!)
    if (sumV > ir_entf)
    {
      Serial.println("MAX - Ergebnis > ir_entf"); // DEBUG ONLY

      // Zurückfahren
      movecar(3, onesec);
      Serial.println("MAX - Zurückgefahren, Gradcounter:"); // DEBUG ONLY
      Serial.println(gradcounter); // DEBUG ONLY
      
      // Unterscheiden zwischen 90° und 180° Drehung
      switch (gradcounter) {
        case 0:
          // 90 grad drehen
          digitalWrite(16, HIGH); // LED Grün ein
          digitalWrite(17, LOW); // LED Rot aus
          movecar(5, ngrad);
          Serial.println("MAX - 90 grad gedreht"); // DEBUG ONLY
          gradcounter++; // Drehvariable auf 1 setzen (1 addieren)
          digitalWrite(16, LOW); //LED Grün aus
          Serial.println("Gradcounter nachher90:"); // DEBUG ONLY
          Serial.println(gradcounter); // DEBUG ONLY
          break;
        case 1:
          // 180 Grad drehen
          digitalWrite(16, LOW); // LED Grün aus
          digitalWrite(17, HIGH); // LED Rot ein
          movecar(5, oneeightyg);
          Serial.println("MAX - 180 grad gedreht"); // DEBUG ONLY
          gradcounter--; // Drehvariable auf 0 setzen (1 subtrahieren)
          digitalWrite(16, LOW); // OK LED ein
          digitalWrite(17, LOW); // Interrupt LED aus
          Serial.println("Gradcounter nachher180:"); // DEBUG ONLY
          Serial.println(gradcounter); // DEBUG ONLY
          break;
        default:
          // keine Reaktion
          break;
      }
    }
  }
}

// Ersatz für delay() mithilfe millis()
void wartezeit(unsigned short dauer) {
  unsigned short currentMillis = 0; // Aktueller Zeitpunkt
  unsigned short startMillis = (unsigned short)millis(); // Startpunkt der Wartefunktion in ms
  Serial.print("WAIT - "); // DEBUG ONLY
  Serial.print(dauer); // DEBUG ONLY
  do {
    currentMillis = (unsigned short)millis(); // Aktueller Zeitpunkt in ms
  } while (currentMillis - startMillis <= dauer); // Führe oberes aus, bis die Dauer verstrichen ist
  Serial.println(" ...DONE!"); // DEBUG ONLY
}
