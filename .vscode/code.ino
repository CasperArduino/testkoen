#include <String.h>
#include <SoftwareSerial.h>
#include <gprs.h>
SoftwareSerial sim800l(3, 2);
//versie 9.4

//beltegoed checken
//SMS dan gratis het woord BUNDELTEGOED naar nummer 1266 om je tegoed en de houdbaarheid van je bundel(s) op te vragen.

///hij blijft aan als je de pot naar rechts draait, als je em terugdraait gaat ie wel weer uit
//hij verstuurt de blupblup sms nog niet
//https://ardupilot.org/

unsigned long previousMillis = 0;        // laatste keer dat de interval gebeurt is          'long' betekent een groter getal dan int
const long interval = 3600000;             // interval voor de levelcheck (en zo nodig SMSen)  3600000 is een uur
unsigned long currentMillis = millis();


int IsDeGebruikerAlGewaarschuwdSindsDeVorigeKeerDatDeBatterijLeegWas = 0;
float waarde_waarop_low_level_battery_waarschuwing_komt = 10.00;

int Potentiometer_waarde_waarop_de_pomp_moet_aanspringen = 800;
int Potentiometer_waarde_waarop_de_pomp_moet_uitgaan = 500;                             // waterlevel van elke uur testen
int Potentiometer_waarde_waarop_de_pomp_moet_uitgaan_van_boot_leeg_pompen = 200;
int hoeveelkeermoetiegaanpompenenlevelchecken = 5;
int hoelangmoetiepompenvoordatiegaatlevelchecken = 2000; // 1 minuut
int hoeveelkeermoetiegaanpompenenlevelcheckenBOOTlegen = 10;
int waarde_drukknop = 0;
int PompAAN = 0;
float volt = 0;

const int PIN_volt = A2;
const int PIN_pomp = 4;                  // 'const' betekent dat hij niet meer kan wijzigen
const int PIN_drukknop = 6;
const int PIN_led_pomp = 7;
const int PIN_led_verbonden = 8;
const int PIN_potentiometer_waterlevel = A1;

int IsDeGebruikerAlGewaarschuwdSindsDeVorigeKeerDatHetWaterHoogStond = 0;

char* text;                             // char* is een rijtje van karakters
char* number;
char incomingMessage;                   // char is 1 karakter
String SMS_ontvangen;                   // String is een stukje text

int tmp = 0;
int var = 0;

//pomp1minuut, pomp2minuten etc
//zijnweerklaarvoor (en dan krijg je de status van de batterij en het waterlevel terug)
//geennattevoeten (pompt totdat ie droog is hoe lang t ook duurt)

//als je langere teksten wil sturen dan moet je de SoftwareSerial buffer aanpassen, heb bestandje in de Arduino IDE daarvoor aangepast
//https://forum.arduino.cc/index.php?topic=429975.0
//toevoegen: de mogelijkheid om 1 minuut te pompen
//toevoegen: 1 keer per uur checken wat de sensor waarde is en als te hoog dan smsen
//bij LEVEL nog een SMS versturen via de functie

void setup() {

  Serial.begin(9600);

  pinMode(PIN_volt, INPUT);
  pinMode(PIN_drukknop, INPUT_PULLUP);             // als ie normale input is krijg je rare waarden zonder input  https://www.arduino.cc/en/Tutorial/Foundations/DigitalPins
  pinMode(PIN_pomp, OUTPUT);                       // Zet relay als OUTPUT
  pinMode(PIN_potentiometer_waterlevel, INPUT);
  pinMode(PIN_led_verbonden, OUTPUT);
  pinMode(PIN_led_pomp, OUTPUT);
  digitalWrite(PIN_led_verbonden, LOW);
  digitalWrite(PIN_led_pomp, LOW);
  digitalWrite(PIN_pomp, LOW);                    // keep the load OFF at the begining. If you wanted to be ON, change the HIGH to LOW

  sim800l.begin(9600);                            // Zet sim800l in communicatie mode

  while   (!sim800l.available()) {                // Zet sim800l module klaar voor gebruik
    sim800l.println("AT");                        // Laat sim800l contact maken met server, je krijgt OK terug
    delay(1000);                                  // Wacht 1000 m sec (1 sec)
    Serial.println("Connecting...");              // Print en serial monitor "connecting"
    delay(1000);                                  // Wacht 1000 m sec (1 sec)
  }

  Serial.println("Connected!");                   // Print in serial monitor "connected"
  sim800l.println("AT+CMGF=1");                   // Zet sim800l in SMS mode
  delay(1000);                                    // Wacht 1000 m sec (1 sec)
  sim800l.println("AT+CNMI=2,2,0,0,0");           // Procedure om nieuwe berichten te verwerken
  delay(1000);                                    // Wacht 1000 m sec (1 sec)
  sim800l.println("AT+CMGL=\"REC UNREAD\"");      // Read Unread Messages
  digitalWrite(PIN_led_verbonden, HIGH);
}



void StuurEenSMS(String SMS_omteversturen) {

  sim800l.print("AT+CMGF=1\r");                   // Zet sim800l in SMS mode
  delay(500);                                     // Wacht 100 m sec
  sim800l.print("AT+CMGS=\"+31645200415\"\r");    // Zet mob nummer in sim800l "\r" einde regel
  delay(500);                                     // Wacht 500 m sec
  sim800l.print(SMS_omteversturen);
  delay(500);
  sim800l.print((char)26);                        // Einde sms
  delay(500);
  sim800l.println();                              // Zend SMS
  delay(500);
  Serial.println("SMS verstuurd met daarin de tekst " + SMS_omteversturen);
  delay(500);
  SMS_ontvangen = "";
  delay(500);
  // sim800l.println("AT+CMGDA=\"DEL ALL\"");
  // delay(500);

}


void CheckOfDeKnopIsIngedrukt() {

  waarde_drukknop = digitalRead(PIN_drukknop);// read the push value - ingedrukt is LAAG!!!!!

  if (waarde_drukknop == LOW && PompAAN == LOW) {
    Serial.println("Pomp aangezet met de knop");
    digitalWrite(PIN_pomp, HIGH);
    digitalWrite(PIN_led_pomp, HIGH);
    PompAAN = HIGH;
    delay(100);
    waarde_drukknop = HIGH;
    delay(1000);
  } else if (waarde_drukknop == LOW && PompAAN == HIGH) {
    Serial.println("Pomp uitgezet met de knop");
    digitalWrite(PIN_pomp, LOW);
    delay(100);
    digitalWrite(PIN_led_pomp, LOW);
    PompAAN = LOW;
    waarde_drukknop = HIGH;
    delay(1000);
  }

}


void PompAANfunctie() {
  digitalWrite(PIN_pomp, HIGH);                  // Zet RELAY pin LOW (pomp aan)
  delay(100);
  digitalWrite(PIN_led_pomp, HIGH);              // Zet LED pin LOW (pomp uit)
  PompAAN = HIGH;
}


void PompUITfunctie() {
  digitalWrite(PIN_pomp, LOW);                    // Zet RELAY pin HIGH (pomp uit)
  delay(100);
  digitalWrite(PIN_led_pomp, LOW);                // Zet LED pin LOW (pomp uit)
  PompAAN = LOW;
}

void Alsdebootzovolisgadanmaximaal5x30secpompen() {

  previousMillis = currentMillis;
  if (interval < 60000) {
    tmp = interval / 1000;  //in de variable tmp (van temporary ook bekend als tijdelijk) slaan we even de interval (de wachttijd) op gedeeld door 1000 (want het zijn miliseconden)
    Serial.println("Er zijn " + String(tmp) + " seconden verstreken");
  } else {
    tmp = interval / 60000;
    Serial.println("Er zijn " +  String(tmp)  + " minut(en) verstreken");
  }
  Serial.println("de tijd is om we gaan de battery level check doen");


  float volt = analogRead(PIN_volt);
  volt = volt * 15 / 1000 - 0.10;
  Serial.print("Volts ");                      // Print in serial monitor "Distance"
  Serial.print(volt);                    // Zet de gemeten CM in serial monitor
  Serial.println("V");                          // Zet achter "distance" cm

  //Serial.println("Voltage SMSen");

  if (volt < waarde_waarop_low_level_battery_waarschuwing_komt) {
    //stuur de waarschuwing als we niet al gewaarschuwd hebben
    Serial.println("batterijwaarde is lager dan we willen");
    if (IsDeGebruikerAlGewaarschuwdSindsDeVorigeKeerDatDeBatterijLeegWas == 0) {
      Serial.println("Gebruiker is nog niet gewaarchuwd over low battery level dus dat gaan we nu doen");
      //stuur waarschuwing
      sim800l.print("AT+CMGF=1\r");                    //Zet sim800l in SMS mode
      delay(500);                                      //Wacht 100 m sec
      sim800l.print("AT+CMGS=\"+31645200415\"\r");     //Zet mob nummer in sim800l "\r" einde regel
      delay(500);                                      //Wacht 500 m sec
      sim800l.print("voltage ");
      delay(500);
       sim800l.print(volt);
      delay(500);
      sim800l.print((char)26);                         //Einde sms
      delay(500);
      sim800l.println();                               //Zend SMS
      delay(500);
      //Serial.println("SMS verstuurd met daarin de tekst energieop " + String(volt) + "V");
      delay(500);
      SMS_ontvangen = "";
      delay(500);
      
    }
    //zet IsDeGebruikerAlGewaarschuwdSindsDeVorigeKeerDatDeBatterijLeegWas op 1
    IsDeGebruikerAlGewaarschuwdSindsDeVorigeKeerDatDeBatterijLeegWas = 1;
  } else {
    IsDeGebruikerAlGewaarschuwdSindsDeVorigeKeerDatDeBatterijLeegWas = 0;
  }




  int waterlevel = analogRead(PIN_potentiometer_waterlevel);

  if (waterlevel > Potentiometer_waarde_waarop_de_pomp_moet_aanspringen) {
    Serial.println("Het waterlevel is BOVEN de " + String(Potentiometer_waarde_waarop_de_pomp_moet_aanspringen) + " het is namelijk: " + String(waterlevel));

    Serial.println(IsDeGebruikerAlGewaarschuwdSindsDeVorigeKeerDatHetWaterHoogStond); //TESTKOEN

    if (IsDeGebruikerAlGewaarschuwdSindsDeVorigeKeerDatHetWaterHoogStond == 0) {
      //je kan geen functie aanroepen vanuit een functie kuthoeren
      //StuurEenSMS("blupblup"); //waarchuwen hier waterlevel inzetten? en de waarde waarop hij moet aanspringen?


      //kut SMS versturen werkt niet wtf
      sim800l.print("AT+CMGF=1\r");                    //Zet sim800l in SMS mode
      delay(500);                                      //Wacht 100 m sec
      sim800l.print("AT+CMGS=\"+31645200415\"\r");     //Zet mob nummer in sim800l "\r" einde regel
      delay(500);                                      //Wacht 500 m sec
      sim800l.print("Uhhmm HALLO? Ik zink bijna. ");
      delay(500);
      sim800l.print(waterlevel);
      delay(500);
      sim800l.print(" cm water GEK!");
      delay(500);
      sim800l.print((char)26);                         //Einde sms
      delay(500);
      sim800l.println();                               //Zend SMS
      delay(500);
      Serial.println("SMS verstuurd met daarin de tekst blupblup " + String(waterlevel));
      delay(500);
      SMS_ontvangen = "";
      delay(500);

      //handig om hier een functie "level sturen" ofzo voor te maken? functionaliteit wordt ook meerdere keren gebruikt
      //je kan ook dat we een BEFORE en AFTER nemen en dan achteraf (na het pompen) sturen in 1 sms
      IsDeGebruikerAlGewaarschuwdSindsDeVorigeKeerDatHetWaterHoogStond = 1;
    }

    var = 0;
    while (var < hoeveelkeermoetiegaanpompenenlevelchecken) {             //even pompen, level checken, even pompen, level checken etc.
      digitalWrite(PIN_pomp, HIGH);                  // Zet RELAY pin LOW (pomp aan)
      delay(100);
      digitalWrite(PIN_led_pomp, HIGH);              // Zet LED pin LOW (pomp uit)
      PompAAN = HIGH;
      delay(hoelangmoetiepompenvoordatiegaatlevelchecken); //30 seconden, dan levelcheck
      int waterlevel = analogRead(PIN_potentiometer_waterlevel); //levelcheck
      //hier kan je ook nog checken of de hoogte van het water wel veranderd is
      if (waterlevel < Potentiometer_waarde_waarop_de_pomp_moet_uitgaan) {  //if levelcheck = leeg genoeg dan stoppen met de loop
        digitalWrite(PIN_pomp, LOW);                    // Zet RELAY pin HIGH (pomp uit)
        delay(100);
        digitalWrite(PIN_led_pomp, LOW);                // Zet LED pin LOW (pomp uit)
        PompAAN = LOW;
        var = 6;
      }
      var++; //tel er 1 bij op
    }
    //hier eventueel ook de pomp uitzetten?
    //stuur nieuw level (waarschijnlijk al ONDER Potentiometer_waarde_waarop_de_pomp_moet_uitgaan
    //en anders gaat ie na de interval nog een keer proberen om em onder dat niveau te krijgen

  } else {
    Serial.println("Het waterlevel is ONDER " + String(Potentiometer_waarde_waarop_de_pomp_moet_aanspringen) + " het is namelijk: " + String(waterlevel));
    //zet gewaarschuwd op 0
    IsDeGebruikerAlGewaarschuwdSindsDeVorigeKeerDatHetWaterHoogStond = 0;
  }


}


void StuurEenLevelSMS() {

  int waterlevel = analogRead(PIN_potentiometer_waterlevel);

  Serial.print("Waterlevel ");                      // Print in serial monitor "Distance"
  Serial.print(waterlevel);                    // Zet de gemeten CM in serial monitor
  Serial.println(" cm");                          // Zet achter "distance" cm

  Serial.println("Waterlevel SMSen");              // Print in serial monitor "Sending data"

  sim800l.print("AT+CMGF=1\r");                   // Zet sim800l in SMS mode
  delay(500);                                     // Wacht 100 m sec
  sim800l.print("AT+CMGS=\"+31645200415\"\r");    // Zet mob nummer in sim800l "\r" einde regel
  delay(500);
  //sim800l.print("Level ");
  //delay(500);                                     // Wacht 500 m sec
  sim800l.print (String(waterlevel));                     // Zet in sim800l de gemeten waarde "distance"
  delay(500);                                     // Wacht 500 m sec
  sim800l.print(" cm water");                           // Zet in sim800l na gemeten waarde CM
  delay(500);                                     // Wacht 500 m sec
  sim800l.print((char)26);                        // Einde sms
  delay(3000);
  sim800l.println();                              //stuur SMS
  delay(3000);                                     // Wacht 500 m sec
  SMS_ontvangen = "";
  delay(500);
  sim800l.println("AT");
  //  sim800l.println("AT+CMGDA=\"DEL ALL\"");
  //  delay(500);
}


void  SMS_DeVoltage()  {

  float volt = analogRead(PIN_volt);
  volt = volt * 15.0 / 1000 - 0.10;
  Serial.print("Volts ");                      // Print in serial monitor "Distance"
  Serial.print(volt);                    // Zet de gemeten CM in serial monitor
  Serial.println("V");                          // Zet achter "distance" cm

  Serial.println("Voltage SMSen");

  sim800l.print("AT+CMGF=1\r");                    //Zet sim800l in SMS mode
  delay(500);                                      //Wacht 100 m sec
  sim800l.print("AT+CMGS=\"+31645200415\"\r");     //Zet mob nummer in sim800l "\r" einde regel
  delay(500);                                      //Wacht 500 m sec
  sim800l.print("Voltage ");
  delay(500);
  sim800l.print(String(volt));
  delay(500);
  sim800l.print("V");
  delay(500);
  sim800l.print((char)26);                         //Einde sms
  delay(500);
  sim800l.println();                               //Zend SMS
  delay(500);
  Serial.println("SMS verstuurd met daarin de tekst Voltage " + String(volt) + "V");
  delay(500);
  SMS_ontvangen = "";
  delay(500);

}

void SMS_Status() {

  float volt = analogRead(PIN_volt);
  volt = volt * 15.0 / 1000 - 0.10;
  Serial.print("Volts ");                      // Print in serial monitor "Distance"
  Serial.print(volt);                    // Zet de gemeten CM in serial monitor
  Serial.println("V");                          // Zet achter "distance" cm

  int waterlevel = analogRead(PIN_potentiometer_waterlevel);



  if (waterlevel > Potentiometer_waarde_waarop_de_pomp_moet_uitgaan_van_boot_leeg_pompen || volt < waarde_waarop_low_level_battery_waarschuwing_komt) {


    //Serial.print("Waterlevel ");                      // Print in serial monitor "Distance"
    Serial.print(waterlevel);                    // Zet de gemeten CM in serial monitor
    //Serial.println(" cm");                          // Zet achter "distance" cm

    sim800l.print("AT+CMGF=1\r");                    //Zet sim800l in SMS mode
    delay(500);                                      //Wacht 100 m sec
    sim800l.print("AT+CMGS=\"+31645200415\"\r");     //Zet mob nummer in sim800l "\r" einde regel
    delay(500);                                      //Wacht 500 m sec
    sim800l.print("Nee want ");
    delay(500);
    sim800l.print(String(waterlevel));
    delay(500);
    sim800l.print("cm water en/of ");
    delay(500);
    sim800l.print(volt);   
    delay(500);
    sim800l.print("V Moet ik hem eventueel leegpompen?");
    delay(500);
    sim800l.print((char)26);                         //Einde sms
    delay(500);
    sim800l.println();                               //Zend SMS
    delay(500);
    // Serial.println("SMS verstuurd met daarin de tekst Voltage " + String(volt) + "V");
    // delay(500);
    SMS_ontvangen = "";
    delay(500);

  }


  else

  {


    //Serial.print("Waterlevel ");                      // Print in serial monitor "Distance"
    Serial.print(waterlevel);                    // Zet de gemeten CM in serial monitor
    //Serial.println(" cm");                          // Zet achter "distance" cm

    sim800l.print("AT+CMGF=1\r");                    //Zet sim800l in SMS mode
    delay(500);                                      //Wacht 100 m sec
    sim800l.print("AT+CMGS=\"+31645200415\"\r");     //Zet mob nummer in sim800l "\r" einde regel
    delay(1000);                                      //Wacht 500 m sec
    sim800l.print("Ja dat kan want ");
    delay(500);
    sim800l.print(String(waterlevel));
    delay(500);
    sim800l.print(" cm water en ");
    delay(500);
    sim800l.print(volt);
    delay(500);
    sim800l.print(" V");
    delay(500);
    sim800l.print((char)26);                         //Einde sms
    delay(500);
    sim800l.println();                               //Zend SMS
    delay(500);
    //Serial.println("SMS verstuurd met daarin de tekst Voltage " + String(volt) + "V");
    //delay(500);
    SMS_ontvangen = "";
    delay(500);


  }

}




void SMS_Jadoemaarbootleegpompen() {

   Serial.println("Functie ja doe maar starten");
    
    delay(100);
  
  int waterlevel = analogRead(PIN_potentiometer_waterlevel);

  if (waterlevel > Potentiometer_waarde_waarop_de_pomp_moet_uitgaan_van_boot_leeg_pompen) {


Serial.println("Beginnen met pompen");

delay(100);

    var = 0;
    while (var < hoeveelkeermoetiegaanpompenenlevelcheckenBOOTlegen) {             //even pompen, level checken, even pompen, level checken etc.
      digitalWrite(PIN_pomp, HIGH);                  // Zet RELAY pin LOW (pomp aan)
      delay(100);
      digitalWrite(PIN_led_pomp, HIGH);              // Zet LED pin LOW (pomp uit)
      delay(100);
      PompAAN = HIGH;
      delay(hoelangmoetiepompenvoordatiegaatlevelchecken); // 1 minuut, dan levelcheck
      int waterlevel = analogRead(PIN_potentiometer_waterlevel); //levelcheck
      Serial.print(waterlevel);
      //hier kan je ook nog checken of de hoogte van het water wel veranderd is
      if (waterlevel < Potentiometer_waarde_waarop_de_pomp_moet_uitgaan_van_boot_leeg_pompen) {  //if levelcheck = leeg genoeg dan stoppen met de loop

        digitalWrite(PIN_pomp, LOW);                    // Zet RELAY pin HIGH (pomp uit)
        delay(100);
        digitalWrite(PIN_led_pomp, LOW);                // Zet LED pin LOW (pomp uit)
        delay(100);
        PompAAN = LOW;
        var = 11;
      }
      var++; //tel er 1 bij op
    }

  }

  // opnieuw waterlevel meten
  waterlevel = analogRead(PIN_potentiometer_waterlevel);



  if (waterlevel < Potentiometer_waarde_waarop_de_pomp_moet_uitgaan_van_boot_leeg_pompen) {


    Serial.print("Waterlevel ");                      // Print in serial monitor "Distance"
    Serial.print(waterlevel);                    // Zet de gemeten CM in serial monitor
    Serial.println(" cm");                          // Zet achter "distance" cm

    Serial.println("Waterlevel SMSen");              // Print in serial monitor "Sending data"

    sim800l.print("AT+CMGF=1\r");                   // Zet sim800l in SMS mode
    delay(500);                                     // Wacht 100 m sec
    sim800l.print("AT+CMGS=\"+31645200415\"\r");    // Zet mob nummer in sim800l "\r" einde regel
    delay(500);
    //sim800l.print("Level ");
    //delay(500);                                     // Wacht 500 m sec
    sim800l.print("Boot is leeg namelijk ");   // Zet in sim800l de gemeten waarde "distance"
    delay(500);
    sim800l.print(String(waterlevel)); 
    delay(500);                                     // Wacht 500 m sec
    sim800l.print(" cm");                           // Zet in sim800l na gemeten waarde CM
    delay(500);                                     // Wacht 500 m sec
    sim800l.print((char)26);                        // Einde sms
    delay(3000);
    sim800l.println();                              //stuur SMS
    delay(3000);                                     // Wacht 500 m sec
    SMS_ontvangen = "";
    delay(500);
    sim800l.println("AT");


  }
  else
  {
  
    
    delay(100);
    digitalWrite(PIN_pomp, LOW);                    // Zet RELAY pin HIGH (pomp uit)
    delay(100);
    digitalWrite(PIN_led_pomp, LOW);                // Zet LED pin LOW (pomp uit)
    delay(100);
    PompAAN = LOW;
    delay(100);
    Serial.print("Waterlevel ");                      // Print in serial monitor "Distance"
    Serial.print(waterlevel);                    // Zet de gemeten CM in serial monitor
    Serial.println(" cm");                          // Zet achter "distance" cm

    Serial.println("Waterlevel SMSen");              // Print in serial monitor "Sending data"

    sim800l.print("AT+CMGF=1\r");                   // Zet sim800l in SMS mode
    delay(500);                                     // Wacht 100 m sec
    sim800l.print("AT+CMGS=\"+31645200415\"\r");    // Zet mob nummer in sim800l "\r" einde regel
    delay(500);
    sim800l.print("Boot is niet leeg namelijk ");      // Zet in sim800l de gemeten waarde "distance"
    delay(500);   
    sim800l.print(String(waterlevel));
    delay(500);                                     // Wacht 500 m sec
    sim800l.print(" cm");                           // Zet in sim800l na gemeten waarde CM
    delay(500);                                     // Wacht 500 m sec
    sim800l.print((char)26);                        // Einde sms
    delay(3000);
    sim800l.println();                              //stuur SMS
    delay(3000);                                     // Wacht 500 m sec
    SMS_ontvangen = "";
    delay(500);
    sim800l.println("AT");


  }

}

void loop() {

  CheckOfDeKnopIsIngedrukt();
  delay(100);                                        // Wacht 100 m sec

  if (sim800l.available()) {                        // Als sim800l beschikbaar is
    delay(100);                                     // Wacht 100 m sec

    while (sim800l.available()) {                    // Terwijl sim800l beschikbaar is
      incomingMessage = sim800l.read();
      SMS_ontvangen += incomingMessage;
    }

    delay(100);                                     // Wacht 100 m sec
    Serial.println(SMS_ontvangen);
    SMS_ontvangen.toUpperCase();                      // Zet ontvangen bericht om naar hoofdletters

    if (SMS_ontvangen.indexOf("POMP AAN") > -1) {    // Als de tekst "POMP AAN" binnenkomt
      PompAANfunctie();
      SMS_ontvangen = "";
      Serial.println("Pomp aangezet via SMS");
    }

    if (SMS_ontvangen.indexOf("POMP UIT") > -1) {   // Als de tekst "POMP UIT" binnenkomt
      PompUITfunctie();
      SMS_ontvangen = "";
      Serial.println("Pomp uitgezet via SMS");
    }

    if (SMS_ontvangen.indexOf("LEVEL") > -1) {         // Als de tekst "LEVEL" binnenkomt
      StuurEenLevelSMS();
      //      int waterlevel2 = analogRead(PIN_potentiometer_waterlevel);
      //      String onzin = "Level " + String(waterlevel2) + "cm";
      //      StuurEenSMS(onzin);
      //      SMS_ontvangen = "";
    }

    if (SMS_ontvangen.indexOf("HALLO DAAR") > -1) {
      StuurEenSMS("Hey hoe gaat het?");
    }

    if (SMS_ontvangen.indexOf("GOED HOOR") > -1) {
      StuurEenSMS("Gaan we vissen?");
    }

    if (SMS_ontvangen.indexOf("MISSCHIEN") > -1) {
      StuurEenSMS("Oke dan!!!");
    }

    if (SMS_ontvangen.indexOf("NEE") > -1) {
      StuurEenSMS("Fuck jou dan");
    }

    if (SMS_ontvangen.indexOf("VOLT") > -1) {
      SMS_DeVoltage();
    }

    if (SMS_ontvangen.indexOf("KAN IK VISSEN") > -1) {
      SMS_Status();
    }

    if (SMS_ontvangen.indexOf("POMP LEEG") > -1) {
      SMS_Jadoemaarbootleegpompen();
    }

    if (SMS_ontvangen.indexOf("JA DOE MAAR") > -1) {
      SMS_Jadoemaarbootleegpompen();
    }


    if (SMS_ontvangen.indexOf("POMP1") > -1) {    // Als de tekst "POMP AAN" binnenkomt
      PompAANfunctie();
      Serial.println("Pomp 1 minaangezet via SMS");
      delay(60000);
      PompUITfunctie();
      Serial.println("Pomp uigezet");
    }

    if (SMS_ontvangen.indexOf("POMP2") > -1) {    // Als de tekst "POMP AAN" binnenkomt
      PompAANfunctie();
      Serial.println("Pomp 2 min aangezet via SMS");
      delay(120000);
      PompUITfunctie();
      Serial.println("Pomp uigezet");

    }

    if (SMS_ontvangen.indexOf("POMP3") > -1) {    // Als de tekst "POMP AAN" binnenkomt
      PompAANfunctie();
      Serial.println("Pomp 3 min aangezet via SMS");
      delay(180000);
      PompUITfunctie();
      Serial.println("Pomp uigezet");

    }

    if (SMS_ontvangen.indexOf("POMP4") > -1) {    // Als de tekst "POMP AAN" binnenkomt
      PompAANfunctie();
      Serial.println("Pomp 4 min aangezet via SMS");
      delay(240000);
      PompUITfunctie();
      Serial.println("Pomp uigezet");

    }

    if (SMS_ontvangen.indexOf("POMP5") > -1) {    // Als de tekst "POMP AAN" binnenkomt
      PompAANfunctie();
      Serial.println("Pomp 5 min aangezet via SMS");
      delay(300000);
      PompUITfunctie();
      Serial.println("Pomp uigezet");

    }

    if (SMS_ontvangen.indexOf("POMP10") > -1) {    // Als de tekst "POMP AAN" binnenkomt
      PompAANfunctie();
      Serial.println("Pomp 10 min aangezet via SMS");
      delay(600000);
      PompUITfunctie();
      Serial.println("Pomp uigezet");

    }


    if (SMS_ontvangen.indexOf("OK") == -1) {          // Als na een actie "OK" staat, delete alles
      Serial.println("OK ontvangen ik gooi alles weg");
      sim800l.println("AT+CMGDA=\"DEL ALL\"");
      delay(500);
      SMS_ontvangen = "";
    }

  }


  //unsigned long currentMillis = millis();
  currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {                      // als het uur verlopen is
    Alsdebootzovolisgadanmaximaal5x30secpompen();
  }
}
