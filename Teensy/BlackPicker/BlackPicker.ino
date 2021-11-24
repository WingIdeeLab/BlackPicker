/*==========================================================================
 BlackPicker Steuerungsprogramm
 Läuft auf einem Teensy 3.6 zusammen mit einem Raspberry Pi 4
 Ramon Hofer Kraner
 IDEE OST Fachhochschule St.Gallen
 9.6.21
 ===========================================================================*/

#include "TeensyStep.h" // https://github.com/luni64/TeensyStep
#include <Servo.h>
#include <LCD_I2C_Teensy36.h> // https://github.com/dejanmeznarc/teensy-i2c-lcd
 
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>


// LCD ist ein 5V LCD, aber funktioniert mit 3.3V Interface. Nur Spannung muss noch von 5V Versorgung gespiesen werden
LCD_I2C_Teensy36 lcd(0x3F, 16, 4);

// Servo und Stepper Objekte
Servo blackServoROT;  // create servo object to control a servo 
Servo blackServoINC;  // create servo object to control a servo 
Servo blackServoGRIP;  // create servo object to control a servo 
Stepper motor_X(7, 10);       // STEP pin: 22, DIR pin: 15
Stepper motor_Y(6, 9);       // STEP pin: 23, DIR pin: 16
Stepper motor_Z(8, 11);       // STEP pin: 21, DIR pin: 14
StepControl controller;    // Use default settings 

// Die maximalen Geschwindigkeiten wurden experimentell bestimmt
// Es geht nicht mehr schneller, da dann entwder die Trägheit die Schritte beeinflusst oder die Geschindigkeit der Treiber (bis 20Khz)
int maxSpeedX = 14000; // begrenzt durch Treiber und Masse
int maxSpeedY = 20000;  
int maxSpeedZ = 14000;
int maxAccX = 22000;
int maxAccY = 25000; 
int maxAccZ = 22000;

//HW Serial für die Kommunikation zum Raspberry Pi
#define HWSERIAL Serial4

// Definition der PinAnschlüsse
int EndSwitchPin_X = 16; // End SwitchPin X == B on Board
int EndSwitchPin_Y = 15; // End SwitchPin Y == A on Board
int EndSwitchPin_Z = 14; // End SwitchPin Z == C on Board
int ServoRotPin = 23;  // Rotation
int ServoIncPin = 22;  // Inklination
int ServoGripPin = 21; // Greifer
int OrientCheckPin = 37; // if low we have small hole
int PresentCheckPin = 38; // if low we have something in
int FailSafeHeightPin = 39;
int LEDonBoard = 13;
int SoundPin = 17;
int CheckRaspiPin = 28;
int Selector1Pin = 24;
int Selector2Pin = 25;
int Selector3Pin = 26;
int Selector4Pin = 27;

int StartButton = 33;// initialize the LCD

int greifKlaueZuQuer = 112;  // servo Rotation // vorher 42
int greifKlaueOffenQuer = 60;  // servo Rotation
int greifKlaueSchiebenQuer = 135;  // servo Rotation

int MaxX = 13000;  // maximale Strecke X
int MaxY = 26500; // maximale Strecke Y
int MinZ = -1450; // minimales Z
int Gripmax = 160; // minimales Z

// Warte Position
int warteX = 300;
int warteY = 300;
int warteZ = 0;

// serial Com Variables
const byte numChars = 64;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
char CommandFromPC[numChars] = {0};
int xFromPC = 0;
int yFromPC = 0;
int RotFromPC = 0;
boolean newData = false;

// calibration values
int RotationCircleOffset = 60; // Radius in x mm * 20 = steps (1 step = 0.05mm / 1mm = 20)
float yverzerrung = 0.20;

float alpharadians;

// positionholder für servos und motoren
int CurrentServoROT = 0;
int CurrentServoINC = 0;
int CurrentServoGRIP = 0;
int xcorr, ycorr;
int CurrentXGreifer = 0;
int CurrentYGreifer = 0;

//dataHolderSerial
struct DataStructure {
  char commandPC[numChars] = {0};
  int x;
  int y;
  int rot;
  bool isstanding;
};
int teileProduktionsNummer;

// Initialisieren des Dataholders
DataStructure CurrentDataHolder = { "" , 0 , 0 ,0};

// sleevcount handling vars
int sleeveLength = 400;// // in Steps (20 steps = 1mm)
int traySizeX = 7280; // in Steps
int traySizeY = 5300; // in Steps


int sleeveCNT = 0; 
int row5yyCNT = 0;
int row5yyHopCNT = 0;
int row5xxCNT = 0;
int TrayCNT = 0;
int row5yyHop_offset = 0;
int row5xx_offset = 0;
int row5yy_offset = 0;

int nof_FailsC=0;
int nof_FailsR=0;
bool sucess;
bool serieEnde = false;


// flags
bool RotCorrection = true;
bool RelMove = false;
bool SD_fail = false;

// eeprom
unsigned int eeAddress = 0;
long TotalSleeveCounter;
int EEpromMaxValue = 3;

// time
int nulltime, starttime;
float endtime, totaltime;
             




//******************************************************************//
// Alles initialisieren
void setup()
{

  pinMode(ServoRotPin, OUTPUT);
  pinMode(ServoIncPin, OUTPUT);
  pinMode(ServoGripPin, OUTPUT);

  digitalWrite(ServoRotPin,LOW);
  digitalWrite(ServoIncPin,LOW);
  digitalWrite(ServoGripPin,LOW);
  
  pinMode(EndSwitchPin_X, INPUT_PULLUP);
  pinMode(EndSwitchPin_Y, INPUT_PULLUP);
  pinMode(EndSwitchPin_Z, INPUT_PULLUP);
  pinMode(StartButton, INPUT_PULLUP);
  pinMode(CheckRaspiPin, INPUT_PULLUP);

  pinMode(OrientCheckPin, INPUT_PULLUP);
  pinMode(PresentCheckPin, INPUT_PULLUP);
  pinMode(FailSafeHeightPin, INPUT_PULLUP);
  pinMode(SoundPin, OUTPUT);

  pinMode(Selector1Pin, INPUT_PULLUP);
  pinMode(Selector2Pin, INPUT_PULLUP);
  pinMode(Selector3Pin, INPUT_PULLUP);
  pinMode(Selector4Pin, INPUT_PULLUP);

  

  pinMode(LEDonBoard, OUTPUT);
  for (int i=0; i<10; i++){
    digitalWrite(LEDonBoard, LOW);
    delay(50);
    digitalWrite(LEDonBoard, HIGH);
    delay(50); 
  }


  // initialize the LCD
  lcd.begin();
  lcd.backlight();


  Serial.begin(115200);
  HWSERIAL.begin(9600);


  Serial.println("Starting Machine!");
  Serial.println(" Alle Teile vom Greifer entfernen");
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Initialisierug Z ...");

  init_ZMotor();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("1) Greifer befreien");
  lcd.setCursor(0,1);
  lcd.print("2) Start Druecken");

  WaitForButton(); //**************
  
  blackServoROT.attach(ServoRotPin); 
  blackServoINC.attach(ServoIncPin); 
  blackServoGRIP.attach(ServoGripPin);

  SetblackServoROT(90);
  SetblackServoINC(90);
  SetblackServoGRIP(greifKlaueOffenQuer);



  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Initialisierung X");
  init_XMotor();
  lcd.setCursor(18,0);
  lcd.print("OK");
  
  lcd.setCursor(0,1);
  lcd.print("Initialisierung Y");
  init_YMotor();
  lcd.setCursor(18,1);
  lcd.print("OK");

  delay(1000);
  

}

//*****************************************************************//
// Spielt verschiedene Töne. Mit nbr kann ein Ton ausgewählt werden
// Library nutzt einen Timer, der für andere Funktionen gebraucht wird, daher so implementiert
void playTune(int nbr)
{
  switch (nbr) {

     // Eingabe
     case 1: // do di
        for (int i; i<250; i++){
            digitalWrite(SoundPin, HIGH);
            delayMicroseconds(500);
            digitalWrite(SoundPin, LOW);
            delayMicroseconds(500);
        }
        /*delay(10);
        for (int i; i<250; i++){
            digitalWrite(SoundPin, HIGH);
            delayMicroseconds(300);
            digitalWrite(SoundPin, LOW);
            delayMicroseconds(300);
        }*/
        break;

     // Fehler
     case 2: // dii
        for (int i; i<400; i++){
            digitalWrite(SoundPin, HIGH);
            delayMicroseconds(150);
            digitalWrite(SoundPin, LOW);
            delayMicroseconds(150);
        }
        break;

      // Ende Teile
      case 3: // di da do
        for (int i; i<300; i++){
            digitalWrite(SoundPin, HIGH);
            delayMicroseconds(400);
            digitalWrite(SoundPin, LOW);
            delayMicroseconds(400);
        }
        delay(10);
        for (int i; i<250; i++){
            digitalWrite(SoundPin, HIGH);
            delayMicroseconds(600);
            digitalWrite(SoundPin, LOW);
            delayMicroseconds(600);
        }
        delay(10);
        for (int i; i<250; i++){
            digitalWrite(SoundPin, HIGH);
            delayMicroseconds(800);
            digitalWrite(SoundPin, LOW);
            delayMicroseconds(800);
        }
        break;

      // PAUSE
      case 4: // di di
        for (int i; i<300; i++){
            digitalWrite(SoundPin, HIGH);
            delayMicroseconds(300);
            digitalWrite(SoundPin, LOW);
            delayMicroseconds(300);
        }
        delay(10);
        for (int i; i<300; i++){
            digitalWrite(SoundPin, HIGH);
            delayMicroseconds(300);
            digitalWrite(SoundPin, LOW);
            delayMicroseconds(300);
        }
        break;

     // Greif Fehler
     case 5: // dii
        for (int i; i<400; i++){
            digitalWrite(SoundPin, HIGH);
            delayMicroseconds(200);
            digitalWrite(SoundPin, LOW);
            delayMicroseconds(200);
        }
        
        break;
        
  }

    
}




//******************************************************************//
// Z Achse an den Anschlag fahren
void init_ZMotor(){

  
  motor_Z.setAcceleration(maxAccZ); // steps/s^2 
  
   // ZZZZZZ
  motor_Z.setMaxSpeed(1000) ;     // steps/s
  motor_Z.setTargetRel(50000);  // Set target position to 1000 steps from current position
  controller.moveAsync(motor_Z);     // This will start the movement and return immediately
  
  while(controller.isRunning()){              // wait until the movement is finished
      if (digitalRead(EndSwitchPin_Z) == HIGH){
        controller.emergencyStop();  // stops the movementSerial.print("Playing file: ");
        motor_Z.setPosition(0); 
      }      
  }

  motor_Z.setMaxSpeed(maxSpeedZ);

}



//******************************************************************//
// X Achse an den Anschlag fahren
void init_XMotor(){

  motor_X.setAcceleration(maxAccX); // steps/s^2 
  

  // XXXX
  motor_X.setMaxSpeed(1500) ;     // steps/s
  motor_X.setTargetRel(-50000);  // Set target position to 1000 steps from current position
  controller.moveAsync(motor_X);     // This will start the movement and return immediately
  
  while(controller.isRunning()){              // wait until the movement is finished
      if (digitalRead(EndSwitchPin_X) == HIGH){
        controller.emergencyStop();  // stops the movement immediately
        motor_X.setPosition(0); 
      }      
  }

  motor_X.setMaxSpeed(maxSpeedX);
}

//******************************************************************//
// Y Achse an den Anschlag fahren
void init_YMotor(){

  motor_Y.setAcceleration(maxAccY); // steps/s^2 
  
  // YYYYY
  motor_Y.setMaxSpeed(1500) ;     // steps/s
  motor_Y.setTargetRel(-50000);  // Set target position to 1000 steps from current position
  controller.moveAsync(motor_Y);     // This will start the movement and return immediately
  
  while(controller.isRunning()){              // wait until the movement is finished
      if (digitalRead(EndSwitchPin_Y) == HIGH){
        controller.emergencyStop();  // stops the movement immediately
        motor_Y.setPosition(0); 
      }      
  }

  motor_Y.setMaxSpeed(maxSpeedY);

}


//******************************************************************//
// Greifen, wenn die Huelsen quer liegen an oder aus
void GripQuer(bool onoff){
  if (onoff) {
    SetblackServoGRIP(greifKlaueZuQuer);
  }
  else {
    SetblackServoGRIP(greifKlaueOffenQuer);
  }
  delay(500);
}


//******************************************************************//
// Die Hauptfunktion zum Verfahren der Achsen. Es fahren alle Achse gleichzeitig.
void XYZAbsVerfahren(int x,int y, int z){

  if (x > MaxX || x<0 || y > MaxY || y<0 || z>0 || z<MinZ){

    Serial.print("X or Y Values too large");
    return;
  }
  
  if (RotCorrection){
    
    xcorr = rotCorrection(true,CurrentServoROT);
    ycorr = rotCorrection(false,CurrentServoROT);
    
    //Position der Achse
    int xa = motor_X.getPosition();
    int ya = motor_Y.getPosition();
    // Position des Greifers
    int xg = motor_X.getPosition()+xcorr;
    int yg = motor_Y.getPosition()+ycorr;
    
    Serial.print("Rotations-Korrektur: ");
    Serial.print(xcorr);
    Serial.print (", ");
    Serial.println(ycorr);
    Serial.print("Ursprungs Position (Greifer): ");
    Serial.print(xg);
    Serial.print (", ");
    Serial.print(yg);
    Serial.print("  -   Ursprungs Position (Achse): ");
    Serial.print(xa);
    Serial.print (", ");
    Serial.println(ya);
        
  }
  else {
    xcorr = 0;
    ycorr = 0;
  }

  motor_X.setTargetAbs(x-xcorr);  
  motor_Y.setTargetAbs(y-ycorr);  
  motor_Z.setTargetAbs(z); 
  controller.move(motor_X, motor_Y, motor_Z); 

  int xna = motor_X.getPosition();
  int yna = motor_Y.getPosition();
  CurrentXGreifer = xna + xcorr;
  CurrentYGreifer = yna + ycorr;

  Serial.print("Neue Position (Greiferspitze): ");
  Serial.print(CurrentXGreifer);
  Serial.print (", ");
  Serial.print(CurrentYGreifer);
  Serial.print("  -  Neue Position (Achse): ");
  Serial.print(xna);
  Serial.print (", ");
  Serial.println(yna);
  Serial.println();  
  Serial.print("Z Hoehe: ");
  Serial.println(motor_Z.getPosition());
  
}

//******************************************************************//
// Beweguen der Servos. Die aktuelle Position wird in einer globalen Variable gespeichert.
// Korrektur von 270° Servos zu 180°
int SetblackServoROT(int deg){
  CurrentServoROT = deg;
  // wegen 270 grad servos:
  int servo270 = map(deg, 0, 180, 23, 160);
  blackServoROT.write(servo270);
}

int SetblackServoINC(int deg){
  CurrentServoINC = deg;
  int servo270 = map(deg, 0, 180, 20, 158);
  blackServoINC.write(servo270);
}

int SetblackServoGRIP(int deg){
  CurrentServoGRIP = deg;
  if (deg > Gripmax) deg = Gripmax;
  int servo270 = map(deg, 0, 180, 22, 160);
  blackServoGRIP.write(servo270);
}



//******************************************************************//
// Korrektur des Winkelfehlers in der z Achse (wegen ungenauem 3D Druck)
float rotCorrection(bool xy, int alpha){

  int winkelverschiebung = 225;// plus heisst genuhrzeigersinn verdreht
  alpharadians = radians(winkelverschiebung-alpha);

  // skalierung y anpassen (Ellypische Verzerrung)
  float  skalary = (float)alpha * yverzerrung ;
  
  //normal
  // x zurückgeben
  if (xy == true) return -cos(alpharadians) * RotationCircleOffset;
   // y zurückgeben
  if (xy == false) return +sin(alpharadians) * RotationCircleOffset + skalary; 
  
}



//******************************************************************//
// Menu zur Hilfsfunktion, um über Eingaben auf der seriellen Schnittstelle zu verfahren.
void showSerialMenu(){
   
   Serial.println();
   Serial.println("----- Serielle Kontrolle eingeschaltet -----");
   Serial.println(" Folgende Eingaben sind moeglich:");
   Serial.println();
   Serial.println(" - xschritte:   X Achse ABSOLUT Schritte fahren (z.b x200) ");
   Serial.println(" - yschritte:   Y Achse ABSOLUT Schritte fahren (z.b y200) ");
   Serial.println(" - zschritte:   Z Achse ABSOLUT Schritte fahren (z.b z200) ");
   Serial.println(" - rgrad:   Kopfrotation 0-180");
   Serial.println(" - ngrad:   Kopfneigung 0-180");
   Serial.println(" - ggrad:   Greifer 0-180"); 
   Serial.println(" - k:       Rotationskorrektur ein/aus"); 
   Serial.println(" - t:       Rotationskorrektur Test (Start lange Drücken um zu Beenden)");   
   Serial.println(" - q:       Zurück zur Produktion"); 

   
   Serial.println();
   Serial.println();

}

//******************************************************************//
// Hilfsfunktion, um über Eingaben auf der seriellen Schnittstelle zu verfahren.
void SerialControl(){

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Serielle Kontrolle!");
  lcd.setCursor(0,1);
  lcd.print("Keine Produktion!");
  
  int pos;
  char incomingByte;
  int x,y,z;
 
  showSerialMenu();
  
  while(1){
    
    if (Serial.available() > 0) {
   
          incomingByte = Serial.read();
         
          switch (incomingByte) {
            
            case 120: //x
              pos = Serial.parseInt();
              Serial.print("Moving x to: ");
              Serial.println(pos);
              y = motor_Y.getPosition() + rotCorrection(false,CurrentServoROT);
              z = motor_Z.getPosition();
              XYZAbsVerfahren(pos,y,z);
              break;
              
            case 121:  //y
              pos = Serial.parseInt();
              Serial.print("Moving y to: ");
              Serial.println(pos);
              x = motor_X.getPosition() + rotCorrection(true,CurrentServoROT);
              z = motor_Z.getPosition();
              XYZAbsVerfahren(x,pos,z);
              break;
              
            case 122:  //z
              pos = Serial.parseInt();
              Serial.print("Moving z to: ");
              Serial.println(pos);
              x = motor_X.getPosition() + rotCorrection(true,CurrentServoROT);
              y = motor_Y.getPosition() + rotCorrection(false,CurrentServoROT);    
              XYZAbsVerfahren(x,y,pos);
              break;
              
            case 114:  //r
              pos = Serial.parseInt();
              Serial.print("Rotation Kopf: ");
              Serial.println(pos);
              SetblackServoROT(pos);
              break;
              
             case 110:  //n
              pos = Serial.parseInt();
              Serial.print("Neigung Kopf: ");
              Serial.println(pos);
              SetblackServoINC(pos);
              break;
              
            case 103:  //g
              pos = Serial.parseInt();
              Serial.print("Greifer: ");
              Serial.println(pos);
              SetblackServoGRIP(pos);
              break;
              
            case 107:  //k
              Serial.print("Rotationskorrektur ");
              if (RotCorrection == true){
                RotCorrection = false;
                Serial.println("AUS");
              }
              else if (RotCorrection == false){
                RotCorrection = true;
                Serial.println("EIN");
              }
              break;
                
            case 113:  //q
              Serial.println("Quit Serial Control");
              Serial.println("Production Control enabled");
              Serial.println();
              return;
              break;                            
            
            case 116:  //t
              Serial.println("Rotationskorrektur Test (Start lange Drücken um zu Beenden)");
              Serial.println();
              rotationsKorrekturTest();
              showSerialMenu();
              break;                            
            }  


                         
    }
  }
    
  
}

//******************************************************************//
// Hilfsfunktion zum Empfangen von Paketen
// Wartet eingehende serielle Pakete ab und detektiert Start und Endmarker und extrahiert die Daten dazwischen.  
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (HWSERIAL.available() > 0 && newData == false) {
        rc = HWSERIAL.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}


//******************************************************************//
// Fordert über die Serielle schnittstelle neue Koordinaten beim Raspi an.
// Diese werden geparst und dann in den globalen Dataholder geschrieben.
void KoordinatenAnfordernUndParsen(){


    Serial.println("Hole neue Koordinaten");
    // warten bis bild fertig
    
    //koordinaten über Serial abfragen
    HWSERIAL.print("<get,0>");


    // Daten empfangen
    while (newData == false){
        recvWithStartEndMarkers();
    }
    newData = false;


    // Format: //  <t,2000,3000,20,1>
    // Format: //  <command,x,y,rotation,is_standing>

    strcpy(tempChars, receivedChars);
        // this temporary copy is necessary to protect the original data
        // because strtok() used in parseData() replaces the commas with \0

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    strcpy(CurrentDataHolder.commandPC, strtokIndx); // copy it to messageFromPC
    
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    CurrentDataHolder.x = atol(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    CurrentDataHolder.y = atol(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    CurrentDataHolder.rot = atol(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    CurrentDataHolder.isstanding = atol(strtokIndx);     // convert this part to an integer

    // das wäre für floats
   // strtokIndx = strtok(NULL, ",");
   // floatFromPC = atof(strtokIndx);     // convert this part to a float

  
}

//******************************************************************//
// Selektiert aufgrund der Teilenummer ein Produktionsprogramm
bool CheckPickSequenceByPartNumber(){

  switch (teileProduktionsNummer)
  {
      case 1:
          // Im Moment das einzige Implementierte Programm
          return PickSequence1();
          break;
      case 2:
          //return PickSequence2();
          break;
      case 3:
          //return PickSequence3();
          break;
      case 4:
          //return PickSequence4();
          break;
  }
      
      
  
}

//******************************************************************//
// Die Logik für Fails, Ende und das Auswählen eines Pick Programms
void  ProduktionAusfuehren() {      // split the data into its parts

        
     if (CurrentDataHolder.commandPC[0] == 116 ){ //"t"
        Serial.println("Command detected:");
        Serial.println("Go and Pick some sleeves");
        Serial.print(CurrentDataHolder.x);
        Serial.print(", ");
        Serial.print(CurrentDataHolder.y);
        Serial.print(", ");
        Serial.print(CurrentDataHolder.rot);
        Serial.print(", ");
        Serial.println(CurrentDataHolder.isstanding);

        // Hauptproduktion (zuerst noch Abfrage um welches Teil es sich handelt)
        sucess = CheckPickSequenceByPartNumber();

        // falls kein Erfolg beim Picken neue Koordinaten holen
        if (sucess == false) {
          
          Serial.print("Fail Detection!!!!!!!!!!!:    ");
          KoordinatenAnfordernUndParsen(); // Weitere koordinaten holen OHNE neues Bild zu machen
          sucess = CheckPickSequenceByPartNumber();

          if (sucess == false) {
          
                Serial.print("Fail Detection!!!!!!!!!!!:    ");
                KoordinatenAnfordernUndParsen(); // Weitere koordinaten holen OHNE neues Bild zu machen
                sucess = CheckPickSequenceByPartNumber();


                if (sucess == false) {
                  Serial.print("Fail Detection!!!!!!!!!!!:    ");
      
                  // wegfahren, damit wir photo machen können
                  XYZAbsVerfahren(11000,16000,0);
                
                  // Nach 3 mal keinen Erfolg neues Bild aufnehmen
                  // raspi soll bild aufnehmen und teile tracken
                  HWSERIAL.print("<pic,0>");
                  delay(2000);
                }
          }

        }
     }

     else if (CurrentDataHolder.commandPC[0] == 110 ){// "n"

        // Warte Positionn anfahren
        XYZAbsVerfahren(warteX,warteY,warteZ);
        
        Serial.println("KEINE neuen DATEN vorhanden");
        playTune(3);

        Serial.println("Bitte Auffüllen!!");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Keine Teile mehr");
        lcd.setCursor(0,1);
        lcd.print("# Teile: ");
        lcd.setCursor(9,1);
        lcd.print(sleeveCNT);
        lcd.setCursor(0,2);
        lcd.print("1) Auffuellen");
        lcd.setCursor(0,3);
        lcd.print("2) Start Druecken");

        WaitForButton();
        
        // raspi soll bild aufnehmen und teile tracken
        HWSERIAL.print("<pic,0>");
        delay(2000);
     
        
     }


     else {
      Serial.println("-----> No valid Format or command!!!");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Fehler");
      lcd.setCursor(0,1);
      lcd.print("--> Kommunikation");
     }



}



//******************************************************************//
// Das eigentliche Pick'n'Place Hauptprogramm
// Hier wird der gesamte Ablauf von Aufgreifen, über Testen, bis Ablegen gesteuert
bool PickSequence1(){

    // alle offsets für die sleevablageorte berechnen
    
    int round5sleeve = sleeveCNT / 5;
    int offset_5er = round5sleeve*sleeveLength;  //Positionsverschiebung in eine 5er reihe
    int offset_reihe = round5sleeve * 700;     //schauen in welcher Reihe auf dem tray wir ablegen müssen 

    Serial.println(" Sleeve Overview Counting:");
    Serial.print(" SCNT: ");
    Serial.print(sleeveCNT);

    row5yyHopCNT = (sleeveCNT / 40) % 2; // alle 40 stück reihe wechseln
    row5yyHop_offset = row5yyHopCNT * 2300;
    Serial.print(" yHop: ");
    Serial.print(row5yyHopCNT);
    Serial.print(" (");
    Serial.print(row5yyHop_offset);
    Serial.print(") ");
    
    row5yyCNT = sleeveCNT % 5; // 0,1,2,3,4,5 und wieder von vorne
    row5yy_offset = row5yyCNT *sleeveLength + row5yyHop_offset;  //Positionsverschiebung in einer 5er reihe + verschiebung bei2erreihe
    Serial.print(" yCNT: ");
    Serial.print(row5yyCNT);
    Serial.print(" (");
    Serial.print(row5yy_offset);
    Serial.print(" / "); 
    Serial.print(row5yy_offset);
    Serial.print(") ");  
 
    row5xxCNT = (sleeveCNT / 5 ) % 8; // alle 8 reihen wieder von vorne
    row5xx_offset = row5xxCNT * 700;
    Serial.print(" xCNT: ");
    Serial.print(row5xxCNT);
    Serial.print(" (");
    Serial.print(row5xx_offset);
    Serial.print(") ");
  
    TrayCNT = sleeveCNT / 80; // alle 80 sleeves (1Tray) wieder von vorne (0 - 3 )
    Serial.print(" TCNT: ");
    Serial.println(TrayCNT);



    int trayx_offset;
    int trayy_offset;

    //TrayCNT=1;

    switch (TrayCNT){

        case 0:
            Serial.println(" Tray 0 ");
            trayx_offset = 0;
            trayy_offset = 0;
            break;
        case 1:
            Serial.println(" Tray 1 ");
            trayx_offset = 0;
            trayy_offset = traySizeY;
            break;
        case 2:
            Serial.println(" Tray 2 ");
            trayx_offset = traySizeX;
            trayy_offset = 0;
            break;
        case 3:
            Serial.println(" Tray 3 ");
            trayx_offset = traySizeX;
            trayy_offset = traySizeY;
            break;
    }


    

    

    
    
    int xc = motor_X.getPosition();
    int yc = motor_Y.getPosition();
    int diffrot = 0;
    int difftrans = 0;

    

    // aktuelle Daten aus dem Dataholder holen
    int x = CurrentDataHolder.x;
    int y = CurrentDataHolder.y;
    int rot = CurrentDataHolder.rot;

               
    XYZAbsVerfahren(xc,yc,0); // mal auf höhe an position fahren
    SetblackServoGRIP(110);


    // erstmal checken, ob was in der Teststation liegt, falls ja müssen wir abbrechen und User bitten es zu entfernen
    if (digitalRead(PresentCheckPin) == LOW) {
      
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Teil in Station!");
          lcd.setCursor(0,1);
          lcd.print("--> Bitte entfernen");
          lcd.setCursor(0,2);
          lcd.print("--> OK druecken");
          playTune(2);
          WaitForButton();
          playTune(1);
          
    }




    //liegende sleeves =========================================
    
    if (CurrentDataHolder.isstanding == false){

        SetblackServoROT(rot);
        XYZAbsVerfahren(x, y, 0);

        
        XYZAbsVerfahren(x, y, -1050);
        SetblackServoGRIP(95);  
        XYZAbsVerfahren(x, y, -1100);
        
        // Falls Greifer aufsteht sind wir im Offset oder es ist sonst was falsch
        if (CheckFailSafe(x,y) == false) {
          return false;
        }
     
        XYZAbsVerfahren(x, y, -1350);
        SetblackServoGRIP(greifKlaueZuQuer); // 

        // Falls Greifer aufsteht sind wir im Offset oder es ist sonst was falsch
        if (CheckFailSafe(x,y) == false) {
          return false;
        }
        
        delay(200);
        XYZAbsVerfahren(x, y, -320);


        // TESTSTATION LYING  -----------------------
    
        // go to Teststation
        SetblackServoROT(90);
        SetblackServoINC(45);
        
        XYZAbsVerfahren(7450, 14720, -320);
        XYZAbsVerfahren(7450, 14720, -970);
        SetblackServoGRIP(greifKlaueOffenQuer);

        // falls nix in der Teststation 
        if (digitalRead(PresentCheckPin) == HIGH) {
          
          XYZAbsVerfahren(7450, 14720, 0);
          //XYZAbsVerfahren(11000,16000,0);
          SetblackServoROT(90);
          SetblackServoINC(90);
          playTune(2);
          return false;
        }

        XYZAbsVerfahren(7450, 14720, -970); // wegnahme position
        //Testing delay for sensors
        delay(200);

    }



    //stehende sleeves   =========================================
    
    if (CurrentDataHolder.isstanding == true){

        SetblackServoROT(90);
        SetblackServoGRIP(160);
        
        XYZAbsVerfahren(x, y, 0);

        //SerialControl();
        
        // Auf teil fahren   
        XYZAbsVerfahren(x, y, -1100); //1050     

        // Falls Greifer aufsteht sind wir nicht im Loch und stehen am Rand auf oder wir stehen auf einer liegenden Sleeve auf
        if (CheckFailSafe(x,y) == false) return false;

        // sonst können wir weiter runter
        else {
          
          XYZAbsVerfahren(x, y, -1240); //1200
          
          // nochmals checken ob wir aufstehen. Falls ja steht die Hülse mit dem kleinen Loch nach oben
          if (digitalRead(FailSafeHeightPin) == HIGH){
            Serial.println(" ***** Kleines Loch oben ***** ");
            Serial.println();
            XYZAbsVerfahren(x, y, -1100);
            SetblackServoGRIP(130);        
          }
          
          else {
            Serial.println(" ***** Grosses Loch oben *****");
            Serial.println();
            XYZAbsVerfahren(x, y, -1100); 
            SetblackServoGRIP(110);
          } 
        }
        
        
        
        delay(500);
        
        //hochfahren
        XYZAbsVerfahren(x, y, 0);

        //neigen
        SetblackServoINC(135); 


        // TESTSTATION STANDING  -----------------------
        
        // go to Teststation 5420
        XYZAbsVerfahren(5380, 14000, 0); // wegen tiefem greifer ein wenig vorher anhalten (sonst crash)
        XYZAbsVerfahren(5380, 14680, 0);
       
        XYZAbsVerfahren(5380, 14680, -450); // runter
        
        // loslassen (zusammengreifen)
        SetblackServoGRIP(160);
        delay(400);

        
        // falls nix in der Teststation abbrechen
        if (digitalRead(PresentCheckPin) == HIGH) {

          
          XYZAbsVerfahren(5380, 14680, 0);
          //XYZAbsVerfahren(11000,16000,0);
          SetblackServoROT(90);
          SetblackServoINC(90);
          playTune(2);
          return false;
        }

        
        // schräg rausfahren  
        XYZAbsVerfahren(5200, 14680, -150);
        

        // Checken ob Teil noch drin liegt! Falls nicht, dürfen wir nicht aufmachen, denn es ist noch am greifer verkantet
        if (digitalRead(PresentCheckPin) == HIGH) {
          
          XYZAbsVerfahren(5200, 10000, 0);  

           // Falls Teil trotzdem noch drin, können wir wieder weitermachen
          if (digitalRead(PresentCheckPin) == LOW){
              XYZAbsVerfahren(5200, 14590, -200);
          }
          
          // Sonst schmeissen wir Teil wieder auf die Fläche
          else {
            SetblackServoROT(90);
            SetblackServoINC(90);
            delay(500);
            XYZAbsVerfahren(11000,16000,0);
            
            playTune(2);
            
            return false;
          }
          
        }

        
        // aufmachen
        SetblackServoGRIP(50); // aufmachen
        SetblackServoINC(45); // neigen  
        
        // hochfahren um crash zu vermeiden
        delay(300);
        XYZAbsVerfahren(7450, 14710, -150); // y erhöhen wegen ungleicher neigung...   
        XYZAbsVerfahren(7450, 14710, -960); // wegnahme position
        //Testing delay for sensors
        delay(0);
    }

    
      
      
    
      // TEILE VON STATION HOLEN UND ABLEGEN  =========================================
                    
      // falls orientierung grosses loch unten
      if (digitalRead(OrientCheckPin) == HIGH){
         diffrot = 0;
      }
      //falls kleines Loch unten
      else {
          diffrot = 180; 
      }

  
      // Wegnehmen
      SetblackServoGRIP(greifKlaueZuQuer); // grip on
      delay(500);
      XYZAbsVerfahren(7450, 14720, -200); // hochfahren

      // falls wir das 3. Tray in der rechten Reihe auffüllen, ist der Kopf zu nahe um direkt abzudrehen
      // hier muss zuerst ein wenig weg gefahren werden
      if (TrayCNT == 2 && row5yyHopCNT == 1) {
          XYZAbsVerfahren(7450, 16000, -200); // wenig in y wegfahren
      }
  
      // Ausrichten  
      SetblackServoROT(diffrot);
      SetblackServoINC(90);

      

       // Bild schon mal aufnehmen, damit wir zeit sparen!!
      // raspi soll bild aufnehmen und teile tracken
      HWSERIAL.print("<pic,0>");
      

      //letzte sleeve einfaedelen:
      if ((sleeveCNT % 5) == 4){
        
          int faedelStrecke = 700; // Wie weit wird die sleeve vor der letzt plazierten abgesetzt?
          
          XYZAbsVerfahren(12400 - row5xx_offset- trayx_offset, 20600 - row5yy_offset - faedelStrecke + trayy_offset, -200);

          // falls wir mit 0 Grad anfahren
          if (diffrot==0){

              // anneigen
              SetblackServoINC(70);
              // seneken
              XYZAbsVerfahren(12320 - row5xx_offset- trayx_offset, 20600 - row5yy_offset - faedelStrecke + trayy_offset, -850);
              // vorfahren
              XYZAbsVerfahren(12320  - row5xx_offset- trayx_offset, 20950 - row5yy_offset - faedelStrecke + trayy_offset, -850);
              // leicht senken
              XYZAbsVerfahren(12320 - row5xx_offset- trayx_offset, 20950 - row5yy_offset - faedelStrecke + trayy_offset, -950);
              // Sleeve loslassen
              SetblackServoGRIP(greifKlaueOffenQuer); // grip off
              delay(100);
              // hochfahren
              XYZAbsVerfahren(12320 - row5xx_offset- trayx_offset, 20850 - row5yy_offset - faedelStrecke + trayy_offset, 0);
          }
          
          // falls wir mit 180 Grad anfahren
          else {
            
              // anneigen
              SetblackServoINC(110);
              // seneken
              XYZAbsVerfahren(12320 - row5xx_offset- trayx_offset, 20620 - row5yy_offset - faedelStrecke + trayy_offset, -750);
              // vorfahren
              XYZAbsVerfahren(12320 - row5xx_offset- trayx_offset, 20940 - row5yy_offset - faedelStrecke + trayy_offset, -750);
              // leicht senken
              XYZAbsVerfahren(12320 - row5xx_offset- trayx_offset, 20940 - row5yy_offset - faedelStrecke + trayy_offset, -850);
              // vorfahren
              XYZAbsVerfahren(12320 - row5xx_offset- trayx_offset, 20960 - row5yy_offset - faedelStrecke + trayy_offset, -850);
              // Sleeve loslassen
              SetblackServoGRIP(greifKlaueOffenQuer); // grip off
              delay(100);
              // hochfahren
              XYZAbsVerfahren(12320 - row5xx_offset- trayx_offset, 20960 - row5yy_offset - faedelStrecke + trayy_offset, 0);
          }
      }



      // Normal Einfaedeln, falls nicht letzte sleeve
      else {

          int ablegemargin = 150;
        
          //Ablageort anfahren
          XYZAbsVerfahren(12320 - row5xx_offset - trayx_offset, 20600 - ablegemargin - row5yy_offset + trayy_offset, -200); 
          
          // runterfahren bis evtl. auf bereits vorhanenes sleeve aufsetzt
          XYZAbsVerfahren(12320 - row5xx_offset - trayx_offset, 20600 - ablegemargin - row5yy_offset+ trayy_offset, -630);
          
          // dann checken ob wir aufstehen, falls ja abbrechen
          if (CheckFailSafe(12320 - row5xx_offset - trayx_offset,20600 - ablegemargin - row5yy_offset+ trayy_offset) == false) {
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Teil blockiert!");
            lcd.setCursor(0,1);
            lcd.print("--> Bitte entfernen");
            lcd.setCursor(0,2);
            lcd.print("--> RESET druecken");
            playTune(2);
            while(1);
          }
          
          // sonst weiter runter fahren      
          XYZAbsVerfahren(12320 - row5xx_offset - trayx_offset, 20600 - ablegemargin  - row5yy_offset+ trayy_offset, -860);
          // andocken
          XYZAbsVerfahren(12320 - row5xx_offset - trayx_offset, 20750 - row5yy_offset+ trayy_offset, -860);
          SetblackServoGRIP(greifKlaueOffenQuer); // grip off
          delay(200);
          XYZAbsVerfahren(12320 - row5xx_offset - trayx_offset, 20800- row5yy_offset+ trayy_offset, 0);
         
          
      }
      
      // Anzahl hülsen hochzählen
      sleeveCNT++;
      
     
      //XYZAbsVerfahren(11000,16000,0);
      SetblackServoROT(90);
      SetblackServoINC(90);
      return true;
              
 
    
}


//******************************************************************//
// Einfaches Checken ob der Greifer aufsteht, wir im Offset sind oder was falsch ist
bool CheckFailSafe(int x,int y){
  


        if (digitalRead(FailSafeHeightPin) == HIGH){
          playTune(5);
          Serial.println(" *****  ACHTUNG: Greifer steht auf!!  ***** ");
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Greiffehler");
          lcd.setCursor(0,1);
          lcd.print("--> Fahre neu an");
          
          // hochfahren
          XYZAbsVerfahren(x, y, -200);
          //XYZAbsVerfahren(x, 100, -200);
          SetblackServoGRIP(70);
          
          return false;
        }
        
        else return true;
}



//******************************************************************//
// Eine Testfunktion für die Rotationskorrektur
void rotationsKorrekturTest(){

  XYZAbsVerfahren(2100, 1400,0);
  SetblackServoGRIP(160);
  SetblackServoROT(0);
  delay(500);
  
  // rotationskorrektur testen
  while (digitalRead(StartButton)== HIGH){

      for (int angle = 0; angle < 180; angle+=15){
        SetblackServoROT(angle);
        XYZAbsVerfahren(5000, 5000,-1000);
        delay(1000);
      }
      for (int angle = 180; angle > 0; angle-=15){
        SetblackServoROT(angle);
        XYZAbsVerfahren(5000, 5000,-1000);
        delay(1000);
      }

  }
  
  
}

//******************************************************************//
// Auf Benutzerinput warten
void WaitForButton(){

  playTune(1);
  Serial.println(" ***** Warte auf Button *****");
  Serial.println();
  while (digitalRead(StartButton)== HIGH){}
  
}


//******************************************************************//
// Eine Pausenfunktion, um die Maschine an einen sicheren Ort zu fahren und danach wieder am gleichen Ort im Ablauf weiter zu fahren
void CheckPause(){

     
      if (digitalRead(StartButton)== LOW) {
          
          Serial.println(" ***** PAUSE *****");
          Serial.println();

          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("** PAUSE **");
          lcd.setCursor(0,1);
          lcd.print("Weiter mit START");
          lcd.setCursor(0,2);
          lcd.print("Sortiere Tray: ");
          lcd.setCursor(15,2);
          lcd.print(TrayCNT);

          lcd.setCursor(0,3);
          lcd.print("Bei Teil: ");
          lcd.setCursor(15,3);
          lcd.print(sleeveCNT);

          // Warte Positionn anfahren
          XYZAbsVerfahren(warteX,warteY,warteZ);

          playTune(4);
          
          delay(800);
          if (digitalRead(StartButton) == LOW){  // wenn button immer noch gedrückt
            while (digitalRead(StartButton) == LOW) {}
            delay(800);
          }
          
          while (digitalRead(StartButton) == HIGH){} // nun warten, bis button wieder gedrückt wird
        
          // raspi soll bild aufnehmen und teile tracken
          HWSERIAL.print("<pic,0>");

          playTune(4);

          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Weiter sortieren");
          lcd.setCursor(0,1);
          lcd.print("...");

          delay(1500);


          
         
          
      }
  
  
}




//******************************************************************//
// Checken auf welche Postion der Mehrfachschalter steht und so Produktion setzen.
int TeileArtEinlesen(){

  int artNr = -1;
  
    if      (digitalRead(Selector1Pin) == LOW){artNr = 1;}
    else if (digitalRead(Selector2Pin) == LOW){artNr = 2;}
    else if (digitalRead(Selector3Pin) == LOW){artNr = 3;}
    else if (digitalRead(Selector4Pin) == LOW){artNr = 4;}
    

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Teil ");
    lcd.setCursor(6,0);
    lcd.print(artNr);
    lcd.setCursor(0,1);
    lcd.print("Ausgewaehlt");
    lcd.setCursor(0,2);
    lcd.print("--> Weiter mit Start");

  // überschreiben
    if (artNr == -1){
      playTune(2);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("1) Teil selektieren");
      lcd.setCursor(0,1);
      lcd.print("2) Reset");
      while(1){}
    }

    WaitForButton(); //**************
    

    return artNr;
    
     
}




//******************************************************************//
//******************************************************************//
//******************************************************************//
// Hauptloop
void loop() {
  

  if  (digitalRead(StartButton)== LOW){
    SerialControl();
  }
  

  // Testfunktionen: Einkommentieren zum Nutzen 
  //SerialControl();
  //rotationsKorrekturTest();
  //sleeveCountTest();


  // Schauen ob rapspi system schon bereit
  if  (digitalRead(CheckRaspiPin)== HIGH){
    
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("KEINE BILDERKENNUNG");
      lcd.setCursor(0,1);
      lcd.print("-->Warte auf System");

      while (digitalRead(CheckRaspiPin)== HIGH){}
      
  }

  // teileselektion abarbeiten
  teileProduktionsNummer = TeileArtEinlesen();
  //teileProduktionsNummer = 1;
  Serial.print(" Selektion Teil: ");
  Serial.println(teileProduktionsNummer);

  // raspi melden, welches teil sortiert wird, damit die tracking parameter angepasst werden können
  HWSERIAL.print("<teil,");
  HWSERIAL.print(teileProduktionsNummer);
  HWSERIAL.print(">");
    

 

// -------- MAIN LOOP -----------

  while(1){

    // Warte Positionn anfahren
    XYZAbsVerfahren(warteX,warteY,warteZ);
    
    // warten auf button
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Produktionsbereit");
    lcd.setCursor(0,1);
    lcd.print("1) Teile verteilen");
    lcd.setCursor(0,2);
    lcd.print("2) Trays leeren");
    lcd.setCursor(0,3);
    lcd.print("3) Start Druecken");
  
    WaitForButton(); //**************

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Starte Sortierung");
  
    sleeveCNT = 0; // mit 0 starten

    // raspi soll bild aufnehmen und teile tracken
    HWSERIAL.print("<pic,0>");

    nulltime = millis();
    totaltime=0;
    
    while (sleeveCNT < 320) {

      // Zeitmessung start
      starttime = millis();


      // Produktion STARTEN
      KoordinatenAnfordernUndParsen();
      ProduktionAusfuehren();
      // Produktion STARTEN

      // Zeitmessung stopp (Zeit pro Teil)
      endtime = millis() - starttime; // in ms
      totaltime += endtime;
      
      Serial.print(" ---->>>>>> Number of Sleeves sorted: ");
      Serial.println(sleeveCNT);
      Serial.println();
           
      EEPROM.get( eeAddress, TotalSleeveCounter );
      
      Serial.println("# Total Sleeves in EEprom: ");
      Serial.println(TotalSleeveCounter);
      TotalSleeveCounter += 1;
      Serial.println("# Writing to EEprom: ");
      Serial.println(TotalSleeveCounter);
      
      EEPROM.put( eeAddress, TotalSleeveCounter );
      //EEPROM.put( EElargeaddr, TotalSleeveCounter);
      
      Serial.print(" Teilezeit: ");
      Serial.println(endtime/1000,2);
      Serial.print(" Totalzeit: ");
      Serial.println(millis() - nulltime);
      
      Serial.print(" Durschnittszeit / Teil : ");
      Serial.print(totaltime/sleeveCNT/1000,2);
      Serial.println(" s");

     
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Sortieren:");
      lcd.setCursor(0,1);
      lcd.print("# Teile:");
      lcd.setCursor(9,1);
      lcd.print(sleeveCNT);
      lcd.setCursor(0,2);
      lcd.print("# Insgesamt:");
      lcd.setCursor(13,2);
      lcd.print(TotalSleeveCounter);   
      lcd.setCursor(0,3);
      lcd.print("# Teilezeit:");
      lcd.setCursor(13,3);
      lcd.print(endtime/1000,2);

      CheckPause();
        
  
    }

    // Abschlusssound
    playTune(3);
    
  }
}

      
      
    
