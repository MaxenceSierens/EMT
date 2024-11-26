// Faire clignoter une lampe LED sur la pin 13 pendant 1s

// Déclaration de la variable Led 

int led = 13;

// Définition de la pin 13 comme sortie
void setup ()
{
  pinMode ( led, OUTPUT);
 
} 
  
// Définition du programme 

void loop (){
  
  digitalWrite (led, HIGH);
  delay (500);
  digitalWrite (led, LOW);
  delay (500);
  
}
  
  
