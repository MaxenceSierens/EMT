#include <LiquidCrystal.h>


LiquidCrystal xiamen(7, 8, 9, 10,11,12);
void setup() {
 xiamen.begin(16, 2);
}


void loop() {

 xiamen.setCursor(0,0);
 xiamen.print("Hello World!");
 xiamen.scrollDisplayRight();
 delay(1000);
}


