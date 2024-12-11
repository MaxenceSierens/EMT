/************************************/
/***    Arduino Wattmeter v1.0    ***/
/***    Board: Arduino Nano 3.0   ***/
/***      By: FredAlf_HiltThos    ***/
/***  VERSION MODIFIEE JOULEMETRE ***/
/***        Fevrier 2016          ***/
/************************************/

/*********** Calibration & Hardware Data ***********/
float Vdiv = 101.0;        // Voltage conversion factor
float Cdiv = 10.0;         // Current conversion factor

const float adcSense = 1.074219e-3;   // ADC conversion factor volt/bit (1.1V / 1024)
float Vscale = 1.0;                   // Mise à l'echelle tension
float Cscale = 1.0;                   // Mise à l'echelle courant
byte muxCnt = 4;                      // valeur de départ du multiplexeur ADC
int nullVal = 512;                    // mesure de la tension de référence
unsigned int vCnt = 0;                // compteur valeurs

/*** Primary averaging ***/
int primCnt = 0;                      // Primary averaging counter
boolean primReady = false;            // Primary averaging complete flag
int adcVolt;                          // Copy ADC voltage
const unsigned int primAvLength = 64; // Number of samples for primary averaging
long primMeanVolt = 0;                // Cumulative ADC mean voltage
long primMeanVoltCopy = 0;            // Averaged primary mean voltage
int  adcCurr;                          // Copy ADC current
long primMeanCurr = 0;                // Cumulative ADC mean current
long primMeanCurrCopy = 0;            // Averaged primary current
long primMeanPow = 0;                 // Cumulative ADC mean power
long primMeanPowCopy = 0;             // Averaged primary mean power

/*** Secondary averaging ***/
unsigned int secArrCnt = 0;
const unsigned int secAvLength = 50;
long secMeanVoltArr[secAvLength];    // Mean Voltage secondary averageing array
long secMeanCurrArr[secAvLength];    // Mean Current secondary averageing array
long secMeanPowArr[secAvLength];     // Real Power secondary averageing array
long secMeanVolt = 0;                // Result secondary averaging mean voltage 
long secMeanCurr = 0;                // Result secondary averaging mean current
long secMeanPow = 0;                 // Result secondary averaging mean power

/*** Cumulative values ***/
byte tEnergy[8];                     // energie totale cumulée sur 64 bits 8*8octets
unsigned int preTimeCnt = 0;         // Prediviseur du timer
unsigned long timeCnt = 0;           // variable comptage temps (secondes)
float paramValues[]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};  //tableau contenant les différentes valeurs à afficher

/***  ***/
float totAverage = 10.0;             // Total averaging length (primary * secondary)
float fSample = 10.0;                 // ADC channel sample frequency

/*** Programme ***/
void setup() {
  MCUCR &= 0xEF;                       // Enable pull-up resistors
  DDRB &= 0xE1;                        // Buttons on D9...12 are inputs
  DDRB |= 0x20;                        // Overflow LED on D13 is output
  PORTB |= 0x1E;                       // Pull-up resistor for the buttons
  bitClear(PORTB, 5);                  // Overflow LED off
  initiateADC();                       // initiate AD converter
  setProperties();                     // Set properties and Clear measured values
  sei();                               // set interrupt flag
}


void loop() {
  if (primReady == true) {
    /*** Secondary avearaging Mean Volt ***/
    secMeanVolt -= secMeanVoltArr[secArrCnt];             // Subtract oldest value strored in array from average
    secMeanVolt += primMeanVoltCopy;                      // Add newest value to average
    secMeanVoltArr[secArrCnt] = primMeanVoltCopy;         // Store newest value in array
    
    /*** Secondary avearaging Mean Current ***/
    secMeanCurr -= secMeanCurrArr[secArrCnt];             // and mean current
    secMeanCurr += primMeanCurrCopy;
    secMeanCurrArr[secArrCnt] = primMeanCurrCopy;
    
    /*** Secondary avearaging Mean Power ***/
    secMeanPow -= secMeanPowArr[secArrCnt];               // and power
    secMeanPow += primMeanPowCopy;
    secMeanPowArr[secArrCnt] = primMeanPowCopy;
    
    /*** Array Pointer ***/
    secArrCnt++;                                  // Increase secondary averaging array pointer
    if(secArrCnt >= secAvLength) {
      secArrCnt = 0;
    }
    /*** Cumulative values ***/
    extraLongAddLong(tEnergy, primMeanPowCopy);   // Add primary averaged power to total Energy
    primReady = false;
  }
  
  /*** Mise à jour valeurs à afficher (stocké dans tableau paramValues***/
  if (vCnt > 1000) {            //Après 1000 conversions ADC
    vCnt = 0;
    /*** Calcul des valeurs de la tension ***/
    paramValues[0] = float(secMeanVolt) * Vscale / totAverage;                 //Calcul tension moyenne
    
    /*** Calcul des valeurs du courant ***/
    paramValues[7] = float(secMeanCurr) * Cscale / totAverage;                 //calucl courant moyen
    
    /*** Calcul des valeurs de la puissance ***/
    paramValues[14] = float(secMeanPow) * Vscale * Cscale / totAverage;        //calcul puissance active
    
    /***Calcul des valeurs de l'énergie ***/
    paramValues[19] = extraLongToFloat(tEnergy) * Vscale * Cscale / fSample;    //calcul energie (Joules) totale (format 64 bits vers flottant)
  }
}

/*** Clear extra long ***/
void extraLongClear(byte* elVal) {
  for(int ci=0; ci<8; ci++) {
    elVal[ci] = 0x00;               //Effacement des 8 octets pour le type extra long
  }
}

/*** Clear array ***/
void avArrClear(unsigned int arrLen, long* avArr) {
  for(int ci=0; ci<arrLen; ci++) {
    avArr[ci] = 0;                  //Mise à 0 des valeurs tableaux
  }
}

/*** Paramétrages et effacement des variables***/
void setProperties() {
  /*** Set division factors ***/
  totAverage = float(primAvLength * secAvLength);  //valeur pour le moyennage final (3200 valeurs (64*50))
  fSample = 19230.7 / 4.0;                         //fréquence d'échantillonnage pour chaque voie (Fosc=16Mhz -> ADC prediv=64 -> ADC freq=250KHz -> Freq acquisition = Freq ADC/13 = 19230,77Hz)
  Vscale = adcSense * Vdiv;                        //mise à l'echelle de la tension
  Cscale = adcSense * Cdiv;                        //mise à l'échelle du courant
  
  /***Initialisation variables***/
  avArrClear(secAvLength, secMeanVoltArr);         // Clear the secondary averaging arrays ...
  avArrClear(secAvLength, secMeanCurrArr);
  avArrClear(secAvLength, secMeanPowArr); 
  secArrCnt = 0;                                   // Clear secondary array counter
  secMeanVolt = 0;                                 // Clear secondary averaging values ...
  secMeanCurr = 0;
  secMeanPow = 0;
  extraLongClear(tEnergy);
  preTimeCnt = 0;                                  // Clear time prescaler ...
  timeCnt = 0;                                     // ... and time counter
}

/***Conversion extra long vers flottant***/
float extraLongToFloat(byte* elVal) {
  boolean sign bitRead(elVal[7], 7);    // true = negative, false = positive value
  int fi;
  for(fi=7; fi>3; fi--) {               // Find the first MS byte without leading zero's in the higherst 4
    if(sign == false && (elVal[fi] != 0x00 || bitRead(elVal[fi-1], 7) == true)) {
      break;
    }                                   // Find the first MS byte without leading 0xFF in the higherst 4
    if(sign == true && (elVal[fi] != 0xFF || bitRead(elVal[fi-1], 7) == false)) {
      break;
    }
  }
  long tVal;
  long loVal = 0;
  for(int shi=24; shi>=0; shi-=8) {      // Shift values for each byte 
    tVal = long(elVal[fi--]);            // Take first relevant byte
    loVal += tVal << shi;                // and shift it into te long
  }
  float flVal = float(loVal);            // Make it a float
  flVal *= pow(2.0, (fi + 1.0) * 8.0);   // and multiply it by the last byte exponent
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
  byte ext = 0x00;           // extend the long with 0x00 ...
  if(bitRead(add3, 7) == true) {
    ext = 0xFF;              // ... or 0xFF if the long is negative.
  }
  asm volatile(
    "add %[v0], %[a0]\n"     // add the bytes of the long to the extralong
    "adc %[v1], %[a1]\n"
    "adc %[v2], %[a2]\n"
    "adc %[v3], %[a3]\n"
    "adc %[v4], %[bext]\n"
    "adc %[v5], %[bext]\n"
    "adc %[v6], %[bext]\n"
    "adc %[v7], %[bext]\n"
    : [v0] "+r" (elVal[0]),  // ouput operand list
      [v1] "+r" (elVal[1]),
      [v2] "+r" (elVal[2]),
      [v3] "+r" (elVal[3]),
      [v4] "+r" (elVal[4]),
      [v5] "+r" (elVal[5]),
      [v6] "+r" (elVal[6]),
      [v7] "+r" (elVal[7])
    : [a0] "r" (add0),       // input operand list
      [a1] "r" (add1),
      [a2] "r" (add2),
      [a3] "r" (add3),
      [bext] "r" (ext)
  );
}


void initiateADC() {
  ADCSRB = 0x00;
  DIDR0 = 0x30;            //Désactivation des entrée numériques (nécessaire pour ADC)
  ADMUX = 0xC4;            //Paramétrage référence de tension interne 1,1V
  ADCSRA = 0xAE;           //ADC activé avec prédiviseur *64
  ADCSRB = 0x40;           //ADC en mode libre (convertit à la volée, sans arrêt)
  bitWrite(ADCSRA, 6, 1);  //Démarre le convertisseur
}


/*** Interruption ADC***/
ISR(ADC_vect) {
  
  /***Lecture de la valeur convertie (10 bits)***/
  int adcInt = ADCL;                       // store lower byte ADC
  adcInt += ADCH << 8;                     // store higher bytes ADC
  long adcVal = long(adcInt) - nullVal;    //valeur relative (par rapport au potentiel de référence)
  
  /***Compteur pour multiplexeur de selection de voie***/
  muxCnt++;                                //S'incrémente de 0 à 3 (repasse à 0 lorsqu'il atteint 4)
  muxCnt &= 0x03;
  ADMUX = muxCnt | 0xC4;
  
  /***Mesure de tension***/
  if(muxCnt == 2) {                        //Lecture de l'entrée ADC tension
    adcVolt = adcVal;
    primMeanVolt += adcVal;                //ajout de cette valeur à la valeur de moyennage tension
  }
  
  /*** Current measurement ***/
  if(muxCnt == 3) {                        //Lecture de l'entrée ADC courant
    adcVal = -adcVal;                      // Inverting preamp
    adcCurr = adcVal;
    primMeanCurr += adcVal;                // Primary add Mean Voltage
  }
  
  /*** Power calculation ***/
  if(muxCnt == 0) {                       //Lecture de la tension de référence puis calcul de la puissance
    nullVal = long(adcInt);               //
    long TadcVal = long(adcVolt) * long(adcCurr);  //calcul de puissance instantanée
    primMeanPow += TadcVal;                        //puis on l'ajoute à la variable de moyennage puissance
  }
  
  /*** Transfer primary averaged values ***/
  if(muxCnt == 1) {                              // At a quiet moment ... (Analog channel 7 isn't used)
    primCnt++;                                   // increase primary averaging counter
    if(primCnt >= primAvLength) {                // If the required averaging number is reached ...
      primCnt = 0;
      primMeanVoltCopy = primMeanVolt;           // Make a copy of all the primary averaging values ...
      primMeanVolt = 0;                          // ... and clear the primary averaging value.
      primMeanCurrCopy = primMeanCurr;
      primMeanCurr = 0;
      primMeanPowCopy = primMeanPow;
      primMeanPow = 0;
      primReady = true;                          // The primary averaging values are availeble for the secondary averaging.
    }
    vCnt++;                                      // Increase value counter 
    
  /*** Time ***/
  preTimeCnt++;
  if(preTimeCnt >= 19231) {        //Interruption ADC à 19231Hz, on compte jusqu'à 19231 puis on incrémente timeCnt (1 fois par seconde)
    preTimeCnt = 0;
    timeCnt++;                     //variable temps (fréquence d'incrément 1Hz)
  }
  }
}
