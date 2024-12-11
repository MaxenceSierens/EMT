int mesure;
float tension;

void setup(){
  Serial.begin(9600);

}

void loop(){
  
  //Boucle d'incrémentation
  for (mesure = 1; mesure <= 50; mesure ++){
  Serial.println (mesure);
  tension= mesure * 4.93/1024; // Tension mesurée à l'entrée de la carte
  Serial.println (tension);
  delay (500); 
  }
  
  delay (500);
  
}
  
