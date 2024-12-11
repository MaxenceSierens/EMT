#include <LiquidCrystal.h>


int mesure;
float tension;
LiquidCrystal lcd ( 12,11,5,4,3,2);

void setup(){
  Serial.begin(9600);
  lcd.begin(16,2);

}

void loop(){
  
  mesure=analogRead(0); // Lecture de la valeur numérique
  tension= mesure * 4.93/1024; // Tension mesurée à l'entrée de la carte
  Serial.println (mesure);
  lcd.setCursor (0,0);
  lcd.print("Mesure Tension");
  lcd.setCursor (10,1);
  lcd.print(tension);
  lcd.print("V");
  lcd.setCursor(0,1);
  lcd.print(mesure);
  lcd.print ("bits =");
  
  
  
  delay (500);
  
}
  
