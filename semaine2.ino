// Importation de la biblio liquidCrystad
//Nécessaire pour l'afficheur
#include <LiquidCrystal.h>

//Déclaration des variables
//Affectation des pins entre carte afficheur et la carte Arduino UNO
//RS= pin 12; Enable= pin 11;
//Data 4=  pin 5, Data 5= pin 4, Data 6= pin 3, Data 7= pin 2

LiquidCrystal lcd ( 12,11,5,4,3,2);


void setup (){
  // Initialiser le nombre des caractères et des lignes
  lcd.begin(16,2);
   //Afficher le message
   lcd.setCursor(2,0);
   lcd.print("ESSTIN-NANCY");
   
}

// Programme
void loop (){
  //Positionner le curseur sur la 1ère ligne et la 2ème colonne
  lcd.print ("ESSTIN-NANCY");
 delay (500);
}
  
  
  



