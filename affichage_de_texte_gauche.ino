// Importation de la biblio liquidCrystad
//Nécessaire pour l'afficheur
#include <LiquidCrystal.h>

//Déclaration des variables
//Affectation des pins entre carte afficheur et la carte Arduino UNO
//RS= pin 12; Enable= pin 11;
//Data 4=  pin 5, Data 5= pin 4, Data 6= pin 3, Data 7= pin 2

LiquidCrystal lcd ( 12,11,5,4,3,2);

//Programme
void setup (){
  // Initialiser le nombre des caractères et des lignes
  lcd.begin(16,2);
   //Positionner le curseur sur la 1ère ligne et la 2ème colonne
  lcd.setCursor(2,0);
  //1er message à afficher 
  lcd.print ("ESSTIN-NANCY");
  lcd.setCursor (0,1);
  //2ème message à afficher
 lcd.print ("ShellEcoMarathon");
}


// Programme pour afficher le message 
void loop (){
 
 lcd.setCursor (16,1);
 
 //Décaler le texte vers la gauche
 lcd.autoscroll();
 lcd.print(" ");
 delay (500);
 
}
  
