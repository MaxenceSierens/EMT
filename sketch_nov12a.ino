#include <LiquidCrystal.h>
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);  //Pins des connexions à l'écran (RS, E, D4, D5, D6, D7)

void setup() {
  lcd.begin(16, 2);   //Initialisons l'écran
}

void loop() {
  //Message 1
  lcd.setCursor(0, 0);          //Place le pointeur sur la colonne 0, ligne 0
  lcd.write("Essai 1:");      //écrit le message
  
  lcd.setCursor(0, 1);         //Place le pointeur sur la colonne 0, ligne 1
  lcd.write("Bonjour!");    //écrit le message
  lcd.scrollDisplayRight();
  delay(200);

  
}