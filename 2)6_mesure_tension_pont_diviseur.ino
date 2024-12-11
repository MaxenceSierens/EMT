
//Mesure tension batterie avec utilisation d'un pont diviseur de tension avec un rapport de 1/6
//de façon à ne pas dépasser 5V sur la valeur de la tension d'entrée analogique carte
// Les variables sont les suivantes : mesure (bits), tension(V: entrée carte) et batterie (V:batterie)

#include <LiquidCrystal.h>


int mesure;
float tension;
float batterie; // Déclaration de la variable batterie
LiquidCrystal lcd ( 12,11,5,4,3,2);

void setup(){
  Serial.begin(9600);
  lcd.begin(16,2);

}

void loop(){
  
  lcd.setCursor (0,0);
  lcd.print("MesureTensionBat");
  
  mesure=analogRead(0); // Lecture de la valeur numérique
  tension= mesure * 4.93/1024; // Valeur numérique à l'entrée de la carte
  batterie=tension *6; 
  Serial.println (mesure); // Affichage dans la fenêtre console
  lcd.setCursor(0,1);// Positionnement du curseur 
  lcd.print(mesure);// Affichage du nbr de bits de la valeur numérique sur LCD
  lcd.print ("b;");
  
  lcd.setCursor (5,1);//Positionnement du curseur
  lcd.print(tension); //Affichage de la tension à l'entrée de la carte
  lcd.print(";");
 
  lcd.setCursor (10,1);//Positionnement du curseur
  lcd.print(batterie);// Affichage de la tension batterie avant pont diviseur
  lcd.print("V");

  
  delay (500);
  
  
}
  
