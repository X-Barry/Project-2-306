int sensorValue = A0;
void setup() {
// put your setup code here, to run once:
Serial.begin(9600);
}
void loop() {
// put your main code here, to run repeatedly:
float volts = analogRead(sensorValue)*5.0/1024.0; //pin value is a 8-bit byte
Serial.print("analogyADC ");
Serial.print(analogRead(sensorValue));
Serial.print(" ");
Serial.print("A0 = ");
Serial.println(volts);
delay(100);
}