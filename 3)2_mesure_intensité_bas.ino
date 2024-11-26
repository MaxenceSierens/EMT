#include <LiquidCrystal.h>

//Déclaration des variables
int mesure;
float tension;
float intensite;

LiquidCrystal lcd(12,11,5,4,3,2);

void setup(){
  Serial.begin(9600);
  lcd.begin(16,2);
  
}

void loop (){
  lcd.setCursor(0,0);
  lcd.print("MesureIntensite");
  
  mesure=analogRead(4);
  tension= mesure*4.93/1024;
  intensite=((tension)/150)*1000; 
  Serial.println(mesure);
  lcd.setCursor(0,1);
  lcd.print(mesure);
  lcd.print(";");
  
  lcd.setCursor(4,1);//Positionnement du curseur
  lcd.print(tension); //Affichage de la tension à l'entrée de la carte
  lcd.print(";");
  
  lcd.setCursor(9,1);//Positionnement du curseur
  lcd.print(intensite);// Affichage de la tension batterie avant pont diviseur
  lcd.print("mA");
  
  delay(500);
  
}
