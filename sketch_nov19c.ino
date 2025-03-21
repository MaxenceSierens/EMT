/************************************/
/***    Arduino Jouletmeter v1.0  ***/
/***    Board: Arduino uno R3     ***/
/***      By: Maxence Sierens     ***/
/***           Louis Orlach       ***/
/***  VERSION MODIFIEE JOULEMETRE ***/
/***        Fevrier 2025          ***/
/************************************/


/********************************************************/
/* Code modifié pour être utilisé avec une Arduino uno */
/********************************************************/


#include <LiquidCrystal.h>
#include <Arduino_JSON.h>
#include <assert.h>
#include <avr/io.h>
#include <avr/interrupt.h>




const float adcSense = 1.074219e-3;  // ADC conversion factor volt/bit (1.1V / 1024)
float Vscale = 1.0;                  // Mise à l'echelle tension
float Cscale = 1.0;                  //Mise à l'echelle courant
byte muxCnt = 4;                     // valeur de départ du multiplexeur ADC
int nullVal = 512;                   // mesure de la tension de référence
unsigned int vCnt = 0;               // compteur valeurs


/*** Primary averaging ***/
int primCnt = 0;                       // Primary averaging counter
boolean primReady = false;             // Primary averaging complete flag
int adcVolt;                           // Copy ADC voltage
const unsigned int primAvLength = 64;  // Number of samples for primary averaging
long primMeanVolt = 0;                 // Cumulative ADC mean voltage
long primMeanVoltCopy = 0;             // Averaged primary mean voltage
int adcCurr;                           // Copy ADC current
long primMeanCurr = 0;                 // Cumulative ADC mean current
long primMeanCurrCopy = 0;             // Averaged primary current
long primMeanPow = 0;                  // Cumulative ADC mean power
long primMeanPowCopy = 0;              // Averaged primary mean power


/*** Secondary averaging ***/
unsigned int secArrCnt = 0;
const unsigned int secAvLength = 50;
long secMeanVoltArr[secAvLength];  // Mean Voltage secondary averageing array
long secMeanCurrArr[secAvLength];  // Mean Current secondary averageing array
long secMeanPowArr[secAvLength];   // Real Power secondary averageing array
long secMeanVolt = -0.398;         // Result secondary averaging mean voltage
long secMeanCurr = 0;              // Result secondary averaging mean current
long secMeanPow = 0;               // Result secondary averaging mean power


/*** Cumulative values ***/
byte tEnergy[8];              // energie totale cumulée sur 64 bits 8*8octets
unsigned int preTimeCnt = 0;  // Prediviseur du timer
unsigned long timeCnt = 0;    // variable comptage temps (secondes)
unsigned long timeCntm = 0;
float paramValues[] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };  //tableau contenant les différentes valeurs à afficher


/***  ***/
float totAverage = 10.0;                 // Total averaging length (primary * secondary)
float fSample = 10.0;                    // ADC channel sample frequency
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);  // 3,4,6,11,12,13,14
unsigned long dernierAppui;
unsigned int boucleAffichage = 0;


/*********** Calibration & Hardware Data ***********/
float Vdiv = 1001;   // Voltage conversion factor, ***66.243** (-2.723*log(paramValues[0]))+ / 78*(30.08/27.70)
float Cdiv = 15.87;  // Current conversion factor  ***4*** Cdiv = 20*(4.12/4.657)


/*** Programme ***/
void setup() {
  /*** Bouton ***/
  //MCUCR &= 0xEF;                 // Enable pull-up resistors
  //DDRB &= 0xCF;                  // Buttons on D9...12 are inputs
  //DDRB |= 0x20;                  // Overflow LED on D13 is output
  //PORTB |= 0x1E;                 // Pull-up resistor for the buttons
  //bitClear(PORTB, 5);            // Overflow LED off
  initiateADC();       // initiate AD converter
  setProperties();     // Set properties and Clear measured values
  sei();               // Activer les interruptions globales avec "sei"
  Serial.begin(9600);  // set interrupt flag
  //initLCD();

  // --------------- les boutons ----------------
  pinMode(6, INPUT_PULLUP);  // d / t / v
  pinMode(5, INPUT_PULLUP);  //reset
  pinMode(4, INPUT_PULLUP);  //Start
  pinMode(3, INPUT_PULLUP);  // "stop"
  dernierAppui = millis();
  // --------------- les boutons ----------------
}

// Initialise l'écran au démarrage du joulemètre
void initLCD() {

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Eco Motion Team");
  lcd.setCursor(0, 1);
  lcd.print("Polytech Nancy");

  //delay(1000);

  lcd.setCursor(0, 0);
  lcd.print("Temps      00:00");

  lcd.setCursor(0, 1);
  lcd.print("Energie          ");
}

/*void sendJSONData() {
    // Utiliser un buffer JSON pour stocker les données
    StaticJsonDocument<200> doc;

    // Ajouter les données mesurées au document JSON
    doc["temps_minutes"] = timeCnt / 60;
    doc["temps_secondes"] = timeCnt % 60;
    doc["energie_joules"] = paramValues[19];
    doc["tension_volts"] = paramValues[0];
    doc["courant_amperes"] = paramValues[7];
    doc["puissance_watts"] = paramValues[14];

    // Envoyer le JSON sous forme de chaîne via le port série
    serializeJson(doc, Serial);
    Serial.println();  // Nouvelle ligne pour distinguer les messages

}*/

void loop() {

  if (primReady == true) {
    /*** Secondary avearaging Mean Volt ***/
    secMeanVolt -= secMeanVoltArr[secArrCnt];  // Subtract oldest value strored in array from average
    secMeanVolt += primMeanVoltCopy;
    // Serial.println(secMeanVolt);                  // Add newest value to average
    secMeanVoltArr[secArrCnt] = primMeanVoltCopy;  // Store newest value in array

    /*** Secondary avearaging Mean Current ***/
    secMeanCurr -= secMeanCurrArr[secArrCnt];  // and mean current
    secMeanCurr += primMeanCurrCopy;
    //Serial.println(secMeanCurr/totAverage);
    secMeanCurrArr[secArrCnt] = primMeanCurrCopy;

    /*** Secondary avearaging Mean Power ***/
    secMeanPow -= secMeanPowArr[secArrCnt];  // and power
    secMeanPow += primMeanPowCopy;
    secMeanPowArr[secArrCnt] = primMeanPowCopy;

    /*** Array Pointer ***/
    secArrCnt++;  // Increase secondary averaging array pointer
    if (secArrCnt >= secAvLength) {
      secArrCnt = 0;
    }
    /*** Cumulative values ***/
    extraLongAddLong(tEnergy, primMeanPowCopy);  // Add primary averaged power to total Energy
    primReady = false;
  }

  /*** Mise à jour valeurs à afficher (stocké dans tableau paramValues***/
  if (vCnt > 1000) {  //Après 1000 conversions ADC
    int timeCntm = timeCnt / 60;
    int timeCnts = timeCnt - 60 * timeCntm;

    vCnt = 0;
    //mise à l'echelle de la tension
    /*** Calculate values Voltage ***/
    paramValues[0] = ((float(primMeanVoltCopy) * Vscale / totAverage) * 4.4) + 2;  //Calcul tension moyenne  - 354.76) / 16.949; - 11.71 ) / (-65.9))*10

    /*** Calculate values Current ***/
    paramValues[7] = ((float(primMeanCurrCopy) * Cscale / totAverage) * 37) + 0.5;  //calucl courant moyen - 341.33) / -41.667;    - 1.65 )/ 6.114

    /*** Calculate values Power ***/
    paramValues[14] = paramValues[0] * paramValues[7];  // Real Power    float(secMeanPow) * Vscale * Cscale / totAverage;
    /***Energy ***/

    paramValues[19] = paramValues[0] * paramValues[7] * timeCnts;  // Energy extraLongToFloat(tEnergy) * Vscale * Cscale / fSample

    float loup = Serial.read();
    Serial.println("*************");
    Serial.print(loup);
//reception des donné du power clutching
//*******************************
   /* String inputstring = Serial.readStringUntil('\n');
    char donne[inputstring.length() + 1];
    float L_var1 = 0.0;
    float L_var2 = 0.0;

    char *ptr = strtok(donne, ";");

    if(ptr != NULL){
      L_var1 = atof(ptr);
      ptr = strtok(NULL, ";");
      if (ptr != NULL);{
        L_var2 = atof(ptr);
      }
    }*/
// *****************************
    if (boucleAffichage == 0) {

      lcd.setCursor(0, 1);
      lcd.print("Energie ");
      lcd.setCursor(9, 1);
      lcd.print((paramValues[0] * paramValues[7] * timeCnt) / 1000);
      lcd.setCursor(13, 1);
      lcd.print(" kJ");
    }

    // Affichage de la tension
    if (boucleAffichage == 1) {

      lcd.setCursor(0, 1);
      lcd.print("Tension   ");
      lcd.setCursor(9, 1);
      //lcd.print(paramValues[0]);
      lcd.print(loup);
      lcd.setCursor(14, 1);
      lcd.print(" m/s");
      //lcd.print(" V");

      /*lcd.setCursor(0,0);        
    lcd.print("Courant   ");    
    lcd.setCursor(9,0);         
    lcd.print(paramValues[7]);  
    lcd.setCursor(14,0);        
    lcd.print(" A");    */
    }
    // Affichage de l'intensité
    if (boucleAffichage == 2) {

      lcd.setCursor(0, 1);
      lcd.print("Courant   ");
      lcd.setCursor(9, 1);
      lcd.print(paramValues[7]);
      lcd.setCursor(14, 1);
      lcd.print(" A");
    }
    lcd.print("   ");

    lcd.setCursor(0, 0);
    lcd.print("Temps");
    lcd.setCursor(11, 0);


    if (timeCntm < 10) {
      lcd.print("0");
      lcd.print(timeCntm);
    } else {
      lcd.print(timeCntm);
    }

    lcd.print(":");

    if (timeCnts < 10) {
      lcd.print("0");
      lcd.print(timeCnts);
    } else {
      lcd.print(timeCnts);
    }

    lcd.print("     ");
    float p_joules = ((paramValues[0] * paramValues[7] * timeCnt) / 1000);
    float p_volts = paramValues[0];
    float p_amperes = paramValues[7];
    float p_watt = paramValues[14];
  //  float p_vitesse = L_var1;
    //float p_distance = L_var2;

    JSONVar doc;
    doc["temps_minutes"] = timeCnt / 60;
    doc["temps_secondes"] = timeCnt % 60;
    doc["energie_joules"] = p_joules;
    doc["tension_volts"] = p_volts;
    doc["courant_amperes"] = p_amperes;
    //doc["pc_vitesse"] = p_vitesse;
    //doc["pc_distance"] = p_distance;


    Serial.println(JSON.stringify(doc));
    delay(100);

    //Envoie des données sur le téléphone *************************************************************************
    /* Serial.print("temps = ");
    Serial.print(timeCntm);
    Serial.println("min");

    Serial.print(timeCnts);
    Serial.println("s");

    Serial.print("tension = ");
    Serial.print(paramValues[0]);
    Serial.println(" V");

    Serial.print("intensite = ");
    Serial.print(paramValues[7]);
    Serial.println(" A");

    Serial.print("energie = ");
    Serial.print(paramValues[19]);
    Serial.println(" J");

    Serial.print("Puissance = ");
    Serial.print(paramValues[14]);
    Serial.println(" W");
    Serial.println(" ");

    Serial.println("Vscal");
    Serial.println(Vscale);*/

    // ******************* JSON **************//
    /* Serial.print("{ temps :  \"");
    Serial.print(doc["temps_minutes"]);
    Serial.print(",");

    Serial.print("temps_sec : \"");
    Serial.print(doc["temps_secondes"]);
    Serial.print(",");

    Serial.print("tension : \"");
    Serial.print(doc["tension_volts"]);
    Serial.print(",");

    Serial.print("intensite : \"");
    Serial.print(doc["courant_amperes"]);
    Serial.print(",");

    Serial.print("energie : \"");
    Serial.print(doc["energie_joules"]);
    Serial.print(",");

    Serial.print("Puissance : \"");
    Serial.print(doc["puissance_watts"]);
    Serial.print("}");


    Serial.print("vitesse : \"");
    Serial.print(doc["vitesse_kmh"]);
    Serial.print("}");*/

    // ******************* JSON **************// *********************************************************************************
  }


  // Bouton Start
  if (digitalRead(4) == LOW) {
    ADCSRA = 0xAE;  // Active le CAN
    bitWrite(ADCSRA, 6, 1);
  }


  // Bouton Stop
  if (digitalRead(3) == LOW) {
    ADCSRA = 0x2E;  // Désactive le CAN
    bitWrite(ADCSRA, 6, 1);
  }


  // Bouton Remise à zéro
  if (digitalRead(5) == LOW) {
    ADCSRA = 0x2E;
    bitWrite(ADCSRA, 6, 1);

    setProperties();
    initiateADC();


    lcd.setCursor(0, 0);
    lcd.write("Remise  A  Zero   ");
    lcd.setCursor(0, 1);
    lcd.write("  en cours ....  ");
    delay(1000);
    initLCD();

    boucleAffichage = 0;
  }


  // Bouton changement d'unité
  if (digitalRead(6) == LOW) {
    // Délai d'une seconde entre chaque appui
    if (millis() - dernierAppui > 1000) {
      dernierAppui = millis();
      if (boucleAffichage >= 2) {
        boucleAffichage = 0;
      } else {
        boucleAffichage++;
      }
    }
  }
}


/*** Clear extra long ***/
void extraLongClear(byte* elVal) {
  for (int ci = 0; ci < 8; ci++) {
    elVal[ci] = 0x00;  //Effacement des 8 octets pour le type extra long
  }
}


/*** Clear array ***/
void avArrClear(unsigned int arrLen, long* avArr) {
  for (int ci = 0; ci < arrLen; ci++) {
    avArr[ci] = 0;  //Mise à 0 des valeurs tableaux
  }
}


/*** Paramétrages et effacement des variables***/
void setProperties() {
  /*** Set division factors ***/
  totAverage = float(primAvLength * secAvLength);  //valeur pour le moyennage final (3200 valeurs (64*50))
  fSample = 19230.7 / 3.0;                         //fréquence d'échantillonnage pour chaque voie (Fosc=16Mhz -> ADC prediv=64 -> ADC freq=250KHz -> Freq acquisition = Freq ADC/13 = 19230,77Hz)
  Vscale = adcSense * Vdiv;                        //mise à l'echelle de la tension
  Cscale = adcSense * Cdiv;                        //mise à l'échelle du courant

  /***Initialisation variables***/
  avArrClear(secAvLength, secMeanVoltArr);  // Clear the secondary averaging arrays ...
  avArrClear(secAvLength, secMeanCurrArr);
  avArrClear(secAvLength, secMeanPowArr);
  secArrCnt = 0;    // Clear secondary array counter
  secMeanVolt = 0;  // Clear secondary averaging values ...
  secMeanCurr = 0;
  secMeanPow = 0;
  extraLongClear(tEnergy);
  preTimeCnt = 0;  // Clear time prescaler ...
  timeCnt = 0;     // ... and time counter
  timeCntm = 0;
}


/***Conversion extra long vers flottant***/
float extraLongToFloat(byte* elVal) {
  boolean sign bitRead(elVal[7], 7);  // true = negative, false = positive value
  int fi;
  for (fi = 7; fi > 3; fi--) {  // Find the first MS byte without leading zero's in the higherst 4
    if (sign == false && (elVal[fi] != 0x00 || bitRead(elVal[fi - 1], 7) == true)) {
      break;
    }  // Find the first MS byte without leading 0xFF in the higherst 4
    if (sign == true && (elVal[fi] != 0xFF || bitRead(elVal[fi - 1], 7) == false)) {
      break;
    }
  }
  long tVal;
  long loVal = 0;
  for (int shi = 24; shi >= 0; shi -= 8) {  // Shift values for each byte
    tVal = long(elVal[fi--]);               // Take first relevant byte
    loVal += tVal << shi;                   // and shift it into the long
  }
  float flVal = float(loVal);           // Make it a float
  flVal *= pow(2.0, (fi + 1.0) * 8.0);  // and multiply it by the last byte exponent
  return flVal;
}


/*** addition d'un format long vers format extra long (64 bits 8*8octets) ***/
void extraLongAddLong(byte* elVal, long addVal) {
  byte add0 = byte(addVal);  // put the long in seperate bytes.
  addVal = addVal >> 8;
  byte add1 = byte(addVal);
  addVal = addVal >> 8;
  byte add2 = byte(addVal);
  addVal = addVal >> 8;
  byte add3 = byte(addVal);
  byte ext = 0x00;  // extend the long with 0x00 ...
  if (bitRead(add3, 7) == true) {
    ext = 0xFF;  // ... or 0xFF if the long is negative.
  }
  asm volatile(
    "add %[v0], %[a0]\n"  // add the bytes of the long to the extralong
    "adc %[v1], %[a1]\n"
    "adc %[v2], %[a2]\n"
    "adc %[v3], %[a3]\n"
    "adc %[v4], %[bext]\n"
    "adc %[v5], %[bext]\n"
    "adc %[v6], %[bext]\n"
    "adc %[v7], %[bext]\n"
    : [v0] "+r"(elVal[0]),  // ouput operand list
      [v1] "+r"(elVal[1]),
      [v2] "+r"(elVal[2]),
      [v3] "+r"(elVal[3]),
      [v4] "+r"(elVal[4]),
      [v5] "+r"(elVal[5]),
      [v6] "+r"(elVal[6]),
      [v7] "+r"(elVal[7])
    : [a0] "r"(add0),  // input operand list
      [a1] "r"(add1),
      [a2] "r"(add2),
      [a3] "r"(add3),
      [bext] "r"(ext));
}


void initiateADC() {
  ADCSRB = 0x00;
  DIDR0 = 0x30;   //Désactivation des entrées numériques (nécessaire pour ADC)
  ADMUX = 0xC0;   //Paramétrage référence de tension interne 1,1V 0xC4
  ADCSRA = 0xAE;  //ADC activé avec prédiviseur *64
  ADCSRB = 0x40;  //ADC en mode libre (convertit à la volée, sans arrêt)
}


/*** Interruption ADC***/
ISR(ADC_vect) {

  /***Lecture de la valeur convertie (10 bits)***/
  int adcInt = ADCL;                     // store lower byte ADC
  adcInt += ADCH << 8;                   // store higher bytes ADC
  long adcVal = long(adcInt) - nullVal;  //valeur relative (par rapport au potentiel de référence)

  /***Compteur pour multiplexeur de selection de voie***/
  muxCnt++;               //S'incrémente de 0 à 3 (repasse à 0 lorsqu'il atteint 4)
  muxCnt &= 0x03;         // Pour sélectionner la voie sur laquelle l'ADC écoute
  ADMUX = muxCnt | 0xC0;  // Active la voie 0xC4


  /***Mesure de tension***/
  if (muxCnt == 2) {  //Lecture de l'entrée ADC tension
    adcVolt = adcVal;
    primMeanVolt += adcVal;  //ajout de cette valeur à la valeur de moyennage tension
  }

  /*** Mesure de courant ***/
  if (muxCnt == 3) {  //Lecture de l'entrée ADC courant
    adcVal = adcVal;  // Inverting preamp
    adcCurr = adcVal;
    primMeanCurr += adcVal;  // Primary add Mean Voltage
  }

  /*** Calcul de la puissance ***/
  if (muxCnt == 0) {  //Lecture de la tension de référence puis calcul de la puissance
    nullVal = long(adcInt);
    long TadcVal = long(adcVolt) * long(adcCurr);  //calcul de puissance instantanée
    primMeanPow += TadcVal;                        //puis on l'ajoute à la variable de moyennage puissance
  }

  /*** Transfer primary averaged values ***/
  if (muxCnt == 1) {                // At a quiet moment ... (Analog channel 7 isn't used)
    primCnt++;                      // increase primary averaging counter
    if (primCnt >= primAvLength) {  // If the required averaging number is reached ...
      primCnt = 0;
      primMeanVoltCopy = primMeanVolt;  // Make a copy of all the primary averaging values ...
      primMeanVolt = 0;                 // ... and clear the primary averaging value.
      primMeanCurrCopy = primMeanCurr;
      primMeanCurr = 0;
      primMeanPowCopy = primMeanPow;
      primMeanPow = 0;
      primReady = true;  // The primary averaging values are availeble for the secondary averaging.
    }
    vCnt++;  // Increase value counter

    /*** Time ***/
    preTimeCnt++;
    if (preTimeCnt >= 4731) {  //Interruption ADC à 19231Hz, on compte jusqu'à 19231 puis on incrémente timeCnt (1 fois par seconde)
      preTimeCnt = 0;
      timeCnt++;  //variable temps (fréquence d'incrément 1Hz)
    }
  }
}


