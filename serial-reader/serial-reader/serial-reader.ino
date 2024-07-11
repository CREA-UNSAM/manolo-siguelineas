char rcvd;


void setup() {

  Serial.begin(9600);
  Serial.println("STARTING SERIAL READER...");
  Serial.println("------------------------");
}


void loop() {

  if(Serial.available() > 0) {
    rcvd = Serial.read();
    if (rcvd == '\n' || rcvd == '\r'){
      Serial.println();
    }
    else{
      Serial.print(rcvd);
    }
  }
}