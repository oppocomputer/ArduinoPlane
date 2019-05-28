int humidity_sensor = A0;
int humidity_value;

void setup(){
	Serial.begin(9600);
	Serial.println("|#|#|#|#|#|#|#|#|#|#|#|#|#|#|#|#|#|#|#|#|#|#");
	Serial.println("|#|#|#|#|#|#|#|#|#|#|#|#|#|#|#|#|#|#|#|#|#|#");




}
void loop(){
humidity_value = analogRead(humidity_sensor); 
humidity_value = map(humidity_value,0,1023,1,100);
Serial.print("Vochtigheid: ");
Serial.print(humidity_value);
Serial.print("Pomp: ");
switch(humidity_value){
}
	
	
	
	
}
