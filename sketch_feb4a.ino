

/*********** Calibration & Hardware Data ***********/
//float Vdiv = 101.0;        // Voltage conversion factor
//float Vdiv = 97.4;        // new Voltage conversion factor
float Vdiv = 1001; 
//float Cdiv = 10.0;         // Current conversion factor pour R3=10mΩ

//Si R3= 4x 0.020Ω en parallèle soit donc R3=0.005Ω=5mΩ
//le facteur de conversion sera alors de 20.0 on écrira donc
float Cdiv = 20.0;         // Current conversion factor
//float Cdiv = 15.4;         // Current conversion factor
const byte LCDlines = 2;   // LCD: Number of lines
const byte LCDwidth = 16;  // LCD: Number of character per line

/***************************************************/
boolean Flag=0;
int compteur=0;
const float adcSense = 1.074219e-3;   // ADC conversion factor volt/bit (1.1V / 1024)
float Vscale = 1.0;                   // Voltage scale
float Cscale = 1.0;                   // Current scale
byte muxCnt = 4;                      // Analog multiplexer start
int nullVal = 512;                    // Measured null value
boolean overFlowVolt = false;         // Voltage overflow flag
boolean overFlowCurr = false;         // Current overflow flag
unsigned int vCnt = 0;                // Values counter

/*** Primary averaging ***/

int primCnt = 0;                      // Primary averaging counter
boolean primReady = false;            // Primary averaging complete flag
int adcVolt;                          // Copy ADC voltage
const unsigned int primAvLength = 64; // Number of samples for primary averaging
long primMeanVolt = 0;                // Cumulative ADC mean voltage
long primMeanVoltCopy = 0;            // Averaged primary mean voltage
long primMaxVolt = 0;                 // Highest measured ADC voltage
long primMinVolt = 0;                 // Lowest measured ADC voltage
int adcCurr;                          // Copy ADC current
long primMeanCurr = 0;                // Cumulative ADC mean current
long primMeanCurrCopy = 0;            // Averaged primary current
long primMaxCurr = 0;                 // Highest measured ADC current
long primMinCurr = 0;                 // Lowest measured ADC current
long primMeanPow = 0;                 // Cumulative ADC mean power
long primMeanPowCopy = 0;             // Averaged primary mean power
long primMaxPow = 0;                  // Highest measured ADC power
long primMinPow = 0;                  // Lowest measured ADC power

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

byte tEnergy[8];                     // 64 bits integrated ADC power
unsigned int preTimeCnt = 0;         // Time prescaler
unsigned long timeCnt = 0;           // Time seconds counter
unsigned long timeCntm = 0;           // Time minutes counter
unsigned long timeCnts = 0;           // Time minutes counter

/***  ***/

float totAverage = 10.0;             // Total averaging length (primary * secondary)
float fSample = 10.0;                 // ADC channel sample frequency
const byte LCDlinePos[4] = {0x80,0xC0,0x94,0xD0}; // First char position for each line (as DDRAM instruction) 
byte lineSelect = 0;                 // The selected dsplay line
byte paramPointers[2] = {3,4}; // Parameter pointer for each display line



char const *paramLabels[] = {"Vmean ","Imean ","Preal ","Energy ","Time "};
char const *paramUnits[] = 
                                        {"V",//0_Vmean : tension moyenne 
                                         
                                        "A",//7_Imean : courant moyen
                                         
                                         "W",//14_Preal : puissance réelle
                                        
                                         "J",//19_énergie en Joule
                                    
                                        ":"}; //21_temps en Secondes

//zone réservée d'affichage, si on isole un de ces paramètres il faut aussi penser à isoler la zone réservée pour l'affichage de la valeur mesurée ou calculée
float paramValues[] =
                                   {0.0, //0
                                   
                                    0.0, //7
                                  
                                    0.0, //14
                                 
                                    0.0, //19
                                   
                                    0.0}; //21
//int paramRange[] = {0,0,99,99,100};  // 99=autorange, 100=int-number
int paramRange[] = 
{                                    0, //0
                                    
                                     0,//7
                                  
                                     99,//14
                                    
                                    99, //19
                                  
                                    100  //21
};  // 99=autorange, 100=int-number
//int paramDigits[] = {4,4,4,5,7};         // number of digits

int paramDigits[] = {
                                    4, //0
                               
                                     4,//7
                                 
                                    4,//14
                                    
                                    4,//19                                   
                                    7//21
                                    };// number of digits


byte antiBounceCnt = 0;              // Buttons anti bounce counter
boolean buttonState = false;         // Button is pressed
unsigned int buttonPushed = 0;       // Pressed buttons


void setup() {
  Serial.begin(9600);
  MCUCR &= 0xEF;                       // Enable pull-up resistors
  DDRB &= 0xE1;                        // Buttons on D9...12 are inputs
  DDRB |= 0x20;                        // Overflow LED on D13 is output
  PORTB |= 0x1E;                       // Pull-up resistor for the buttons
  bitClear(PORTB, 5);                  // Overflow LED off
  initiateADC();                       // initiate AD converter
 
 
  Serial.println("ShellEcoMarathon");

  Serial.println("  POLYTECH 2022");
  for(int md=0; md<100; md++) {        // Wait 2 seconds
    muDelay(10000);
  }

  Serial.println("    EMT_2022     ");
 
  Serial.println("   JOULEMETRE  ");
  for(int md=0; md<100; md++) {        // Wait 2 seconds
    muDelay(10000);
  }

  
  Serial.println(" BENMIRA Nabil  ");
  
  Serial.println(" MATHIEU Bryce  ");
  for(int md=0; md<100; md++) {        // Wait 2 seconds
    muDelay(30000);
  }
  
  
  Serial.println("            ");
 
  
  setProperties();                     // Set properties and Clear measured values
  updateParamLabels();                 // Write Parameter labels to LCD
  updateParamValues();                 // Write parameter values to LCD
  sei();                               // set interrupt flag

}


void loop() {

 
if (buttonPushed == 0x02){
  buttonPushed = 0;
  Flag=1; }

if (Flag == 1){
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

   /*** Update display ***/
   if (vCnt > 1000) {            // After 1000 channel samples refresh values on display
     vCnt = 0;
     /*** Calculate values Voltage ***/
     paramValues[0] = float(secMeanVolt) * Vscale / totAverage;                 // calc. mean Voltage
     /*** Calculate values Current ***/
     paramValues[1] = float(secMeanCurr) * Cscale / totAverage;                 // calc. mean Current               // Min current
      /*** Calculate values Power ***/
      paramValues[2] = float(secMeanPow) * Vscale * Cscale / totAverage;        // Real Power
     /*** Flux, Charge, Energy ***/
      paramValues[3] = extraLongToFloat(tEnergy) * Vscale * Cscale / fSample;    // Energy
      /*** Time ***/
      paramValues[4] = (float) round(timeCnt);                                  // Time
    
      /*** Update LCD values ***/
      updateParamValues();    // Write values to LCD

       /*** OverFlow ***/
      if (overFlowVolt == true || overFlowCurr == true) {                        // If there was an overflow ...
        overFlowVolt = false;                                                    // ... clear overflow flags ...
        overFlowCurr = false;
        bitSet(PORTB, 5);                                                        // ... turn overflow LED on.
      }
      else {
        bitClear(PORTB, 5);                                                      // Else overflow LED off.
      }
    }
  }
}

/*** Clear extra long ***/
void extraLongClear(byte* elVal) {
  for(int ci=0; ci<8; ci++) {
    elVal[ci] = 0x00;               // Clear all 8 bytes of the extra long
  }
}

/*** Clear array ***/
void avArrClear(unsigned int arrLen, long* avArr) {
  for(int ci=0; ci<arrLen; ci++) {
    avArr[ci] = 0;                  // Clear all values in the array
  }
}

/*** Set elementary properties & clear all parameters***/
void setProperties() {
  /*** Set division factors ***/
  totAverage = float(primAvLength * secAvLength);
  fSample = 19230.7 / 4.0;                          // sample frequency for each channel
  Vscale = adcSense * Vdiv;                        // Voltage scale
  Cscale = adcSense * Cdiv;                        // Voltage scale
  /*** Set parameter ranges ***/
  int pp;
  int expo = ceil(log10(Vscale*560.0));            // Calculate exponent of highest expected voltage
  for(pp=0; pp<=0; pp++) {
    if(paramRange[pp] < 98) {                      // If not auto range or int ...
      paramRange[pp] = expo; }                     // ... set range for voltage related parameters.
  }
  expo = ceil(log10(Cscale*560.0));                // Calculate exponent of highest expected current
  for(pp=1; pp<=1; pp++) {
    if(paramRange[pp] < 98) {                      // If not auto range or int ...
      paramRange[pp] = expo; }                     // ... set range for current related parameters.
  }
  expo = ceil(log10(Vscale*Cscale*313600.0));      // Calculate exponent of highest expected power
  for(pp=2; pp<=3; pp++) {
    if(paramRange[pp] < 98) {                      // If not auto range or int ...
      paramRange[pp] = expo; }                     // ... set range for power related parameters.
  }
  /*** Clear values ***/
  avArrClear(secAvLength, secMeanVoltArr);         // Clear the secondary averaging arrays ...
  avArrClear(secAvLength, secMeanCurrArr);
  avArrClear(secAvLength, secMeanPowArr); 
  secArrCnt = 0;                                   // Clear secondary array counter
  secMeanVolt = 0;                                 // Clear secondary averaging values ...
  secMeanCurr = 0;
  secMeanPow = 0;
  primMaxVolt = -1024;                             // Set maximum vales at lowest possible value ...
  primMinVolt = 1024;                              // ... and minimum values at highest possible value.
  primMaxCurr = -1024;
  primMinCurr = 1024;
  primMaxPow = -1048576;
  primMinPow = 1048576;
  extraLongClear(tEnergy);
  preTimeCnt = 0;                                  // Clear time prescaler ...
  timeCnt = 0;                                     // ... and time counter
}

/*** Select the next or previous parameter ***/
void changeParamPointer(boolean dir) {
  int paramMax = 4;                      // Number of parameters
  if(dir == true) {
    paramPointers[lineSelect]++;          // Increase parameter pointer for the selected line
    if(paramPointers[lineSelect] > paramMax) {
      paramPointers[lineSelect] = 0; }
  }
  else {
    if(paramPointers[lineSelect] == 0) {  // Decrease parameter pointer for the selected line
      paramPointers[lineSelect] = paramMax + 1; }
   paramPointers[lineSelect]--; 
  }
}

/*** Write parameter label on each line ***/
void updateParamLabels() {
  for (int ci=0; ci<LCDlines; ci++) {

 //   Serial.println(paramLabels[paramPointers[ci]]);
  }
}

/*** Write parameter value on each line ***/
void updateParamValues() {
  for (int ci=0; ci<LCDlines; ci++) {    
    lcdParamValues(paramPointers[ci], LCDlinePos[ci] + LCDwidth);
  }
}


void lcdParamValues(byte paramPo, byte pos) {
  boolean ofm = false;
  if(paramPo < 1) {                 // Voltage related parameters
    ofm = overFlowVolt; }
  else if(paramPo < 2) {           // Current related parameters
    ofm = overFlowCurr; }
  else if(paramPo < 4) {           // Power related parameters
    ofm = overFlowVolt | overFlowCurr; }
  if(paramRange[paramPo] == 100) {  // int number

    /** Conversion secondes/minutes **/

    int timeCntm = timeCnt/60;
    int timeCnts=timeCnt-60*timeCntm;
    lcdTechNot(0, timeCnts, paramRange[paramPo], paramDigits[paramPo], paramUnits[paramPo], ofm, pos+2, true); //affichage secondes LCD
    lcdTechNot(0, timeCntm, paramRange[paramPo], paramDigits[paramPo], paramUnits[paramPo], ofm, pos-2, true); //affichage minutes LCD
  }
  else {                            // float number
    lcdTechNot(paramValues[paramPo], 0, paramRange[paramPo], paramDigits[paramPo], paramUnits[paramPo], ofm, pos, true);
  }
}

/*** Convert an extra long value to float ***/
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


/*** Add a long to an extra long ***/
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
  DIDR0 = 0x30;            // digital inputs disabled for A4 & A5
  ADMUX = 0xC4;            // measuring on ADC4, left adjust, internal 1.1V ref
  ADCSRA = 0xAE;           // AD-converter on, interrupt enabled, prescaler = 64
  ADCSRB = 0x40;           // AD channels MUX on, free running mode
  bitWrite(ADCSRA, 6, 1);  // Start the conversion by setting bit 6 (=ADSC) in ADCSRA
}


/*** Interrupt routine ADC ready ***/
ISR(ADC_vect) {
  /*** Read ADC result ***/
  int adcInt = ADCL;                       // store lower byte ADC
  adcInt += ADCH << 8;                     // store higher bytes ADC
  long adcVal = long(adcInt) - nullVal;    // relative ADC value
  /*** ADC multiplexer counter ***/
  muxCnt++;                                // Multiplexer counter +1
  muxCnt &= 0x03;
  ADMUX = muxCnt | 0xC0;
  /*** Voltage measurement ***/
  if(muxCnt == 2) {                        // Result Analog voltage input ready
    if(adcInt < 12 || adcInt > 1010) {     // Voltage overflow detection
      overFlowVolt = true;
    }
    adcVolt = adcVal;
    primMeanVolt += adcVal;                // Primary add add Mean Voltage
    if(adcVal > primMaxVolt) {             // Store new maximum voltage if greater
      primMaxVolt = adcVal; }
    if(adcVal < primMinVolt) {             // Store new minimum voltage if smaller
      primMinVolt = adcVal; }
   
  }

  /*** Current measurement ***/
  if(muxCnt == 3) {                        // Result Analog current input ready
    if(adcInt < 12 || adcInt > 1010) {     // Current overflow detection
      overFlowCurr = true;
    }
    adcVal = -adcVal;                      // Inverting preamp
    adcCurr = adcVal;
    primMeanCurr += adcVal;                // Primary add Mean Voltage
    if(adcVal > primMaxCurr) {             // Store new maximum current if greater
      primMaxCurr = adcVal; }
    if(adcVal < primMinCurr) {             // Store new minimum current if smaller
      primMinCurr = adcVal; }
  }
  /*** Power calculation ***/
  if(muxCnt == 0) {                       // Result Analog null reference input ready
    nullVal = long(adcInt);               // Store null reference
    long TadcVal = long(adcVolt) * long(adcCurr);  // Calc. the instantanious power ...
    primMeanPow += TadcVal;                        // ... and primary add it.
    if(TadcVal > primMaxPow) {                     // Store new maximum power if greater
      primMaxPow = TadcVal; }
    if(TadcVal < primMinPow) {                     // Store new minimum power if smaller
      primMinPow = TadcVal; }
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
    vCnt++;    // value count +1                 // Increase value counter 

     /*** Button debouncing ***/
    byte buttons = ~PINB & 0x1E;                 // Read the four buttons
    if(buttons == 0) {                           // If no button is pressed ...
      if(antiBounceCnt == 0) {                   // ... and anti bounce is already zero ...
        buttonState = false;
       buttonPushed = 0;                         // clear the button pressed register
      }
      else {
        antiBounceCnt--; }                       // decrease anti bounce counter if it is not zero
    }
    else {
      if(antiBounceCnt < 40) {
        antiBounceCnt++; }                       // Increase anti bounce counter if not maximum
      else if(buttonState == false) {            // If anti bounce reaches maximum the first time ...
        buttonState = true;
        buttonPushed = buttons;                  // ... copy button status for further processing
      }
    }

  }
  /*** Time ***/
  preTimeCnt++;
  if(preTimeCnt >= 19231) {        // Quand le prescaler atteint 1931, on inscrémente le temps
    preTimeCnt = 0;
    timeCnt++;       // Seconds counter  
    int timeCntm = timeCnt/60;
    int timeCnts=timeCnt-60*timeCntm;
    Serial.print(timeCntm);
    Serial.print("min");
    Serial.print(timeCnt-60*timeCntm);
    Serial.println("s");
    Serial.print("tension = ");
    Serial.print(paramValues[0]);
    Serial.println(" V");
    Serial.print("intensité = ");
    Serial.print(paramValues[1]);
    Serial.println(" A");
    Serial.print("énergie = ");
    Serial.print(paramValues[3]);
    Serial.println(" J");
    Serial.println(" ");
  }
}


void lcdCheckBusyFlag() {
  /*** check until LCD is ready ***/ 
  byte dVal;
  DDRD = DDRD & 0x0F;           // PD4...7 = input
  bitSet(PORTD, 3);             // R/W = 1
  bitClear(PORTD, 2);           // RS=0
  do {
    bitSet(PORTB, 0);           // enable = 1
    __asm__("nop\n\tnop\n\t");
    dVal = PIND & 0xF0;         // Read MS nibble
    bitClear(PORTB, 0);         // enable = 0
    __asm__("nop\n\tnop\n\tnop\n\t");
    bitSet(PORTB, 0);           // enable = 1
    __asm__("nop\n\tnop\n\t");
    dVal = dVal | (PIND >> 4);  // Read LS nibble
    bitClear(PORTB, 0); // enable = 0
    __asm__("nop\n\tnop\n\tnop\n\t");
  } while(bitRead(dVal, 7) == HIGH);
  bitClear(PORTD, 3);           // R/W = 0
}


void lcdTechNot(float fval, long lval, int rangeExp, int digits, const char* unit, boolean overFlow, byte pos, boolean alignR) {
  /*** Write a value in Technical Notation on the LCD ***/
  /*** fval = float value, ore use lval = long int value ***/
  /*** rangeExp = exponent of the limit value (e.g. rangeExp=2 => 10^2=100 as limit), 99=autorange, 100 = use the long int value ***/
  /*** digits = number of digits ***/
  /*** unit = the unit: "V", "A",... ***/
  /*** overFlow = external overflow indicator, e.g. from ADC. Puts the "^" char between number and unit ***/
  /*** pos & alignR = LCD position & right align: last position if alignR=true, first position if alignR=false) ***/
  byte totChar = 10;                        // Total number of chars to fill on the LCD (to ensure the old valu is erased)
  char valStr[totChar];                     // Text string
  int strPnt = 0;                           // Text string pointer
  digits -= 1;
  int decpoint = digits + 3;                // No decimal point if val is long
 signed char vvArr[] = {'f','p','n',(signed char)(0xE4),'m',' ','k','M','G','T','P'};  // prefix array CONVERSION
  int expo;                                 // Exponent
  if (rangeExp == 99) {                     // If auto range ...
    expo = floor(log10(fabs(fval)));        // ... use the value exponent ... 
  }
  else {
    expo = rangeExp - 1;                    // ... else use a fixed exponent as range
  }
  if(rangeExp != 100) {                     // If the float value is used ...
    lval = long(round(fval / pow(10, expo - digits)));   // ... calculate the value in N digits.
    if(lval >= round(pow(10, digits + 1))) {             // Fix floating point errors
      lval /= 10;
      expo++;
    }
    decpoint = digits - ((expo+15) % 3);    // Calc. decimal point position
  }
  else {                                    // If the long int value is used ...
    expo = 0;                               // ... prevent using a prefix. 
  }
  if(lval < 0) {                            // If the value is negative ...
    lval = abs(lval);                       // ... make the value absolute ...
    valStr[strPnt++] = 0x2D;                // ... and put a negative sign => string
  }
  long dec;
  byte num;
  boolean prtStart = false;                 // Don't push any number into the string yet (no leading zero's)
  for(int nd=digits; nd>=0; nd--) {         // For the number of digits the value is long ...
    dec = long(round(pow(10, nd)));         // calc. the 10th power number for the current digit,
    num = 0;                                // start value is 0.
    while(lval >= dec) {                    // While the 10th power number is smaller than the value ...
      lval -= dec;                          // decrease the value,
      num++;                                // increase the current digit.
      if(num > 9) {                         // If digit > 9 ...
        num = 0x3C;                         // digit is "<" if value doesn't fit
        lval += dec;                        // for all digit places
        break;
      }
    }
    if(prtStart == true || num != 0 || nd == 0) {  // If printing is active or digit is not zero or LS digit
      prtStart = true;                             // printing is active,
      valStr[strPnt++] = num | 0x30;               // digit => string.
    }
    else if(prtStart == false && decpoint == nd) { // If no number is printed yet and decimal point must be printed
      prtStart = true;                             // printing is active,
      valStr[strPnt++] = 0x30;                     // 0 => string.
    }
    if(decpoint == nd && nd != 0) {                // If decimal point must be printed and it is not the LS digit
      valStr[strPnt++] = 0x2E;                     // decimal point => string.
    }
  }
  if(overFlow == true) {
    valStr[strPnt++] = 0x5E;             // "^" => string if there the external overflow flag is set
  }
  else {
    valStr[strPnt++] = 0x20;             // or a white space => string
  }
  int ediv = floor(expo / 3.0);          // Calc. prefix pointer
  if (ediv != 0) {
    valStr[strPnt++] = vvArr[ediv+5];    // Prefix => string
  }
  byte ci;
  for(ci=0; ci<strlen(unit); ci++) {     // For every unit character ...
    valStr[strPnt++] = unit[ci];         // ... unit char = string.
  }
  if(alignR == true) {                   // If the alignment is right ...
    pos -= totChar;                      // ... calc. new start position.
  }
  //Serial.println(true, pos);                   // Position cursor LCD
  for(byte iw=0; iw<(totChar-strPnt); iw++) { // For every unoccupied preceding character ...
  //  Serial.println(false, 0x20);               // ... send white spaces to LCD to erase old text 
  }
  for(ci=0; ci<strPnt; ci++) {           // For every character ...
  //  Serial.println(false, valStr[ci]);         // ... send character to LCD
  }
}


    
void muDelay(unsigned int mutime) {
  /*** micro seconds delay ***/
  for(unsigned int ci=0; ci<mutime; ci++) {
    __asm__("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
  }
}