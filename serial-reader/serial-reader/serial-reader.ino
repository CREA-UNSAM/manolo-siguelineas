char rcvd;


void setup() {

  Serial.begin(9600);
  Serial.println("Is this on?...");
}


void loop() {

  if(Serial.available() > 0) {
    rcvd = Serial.read();
    Serial.println(rcvd);
  }
}