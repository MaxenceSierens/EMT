#include <LiquidCrystal.h>

//Déclaration variable
 int mesure;
 
void setup(){
  Serial.begin(9600);
 
}

void loop(){
  
  mesure=analogRead(0); // Lecture de la valeur numérique
  
  Serial.println (mesure);
  

  delay (500);
  
}
  
