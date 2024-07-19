char rcvd;

//TODO: Try this thing
//https://randomnerdtutorials.com/esp32-webserial-library/ 

void setup() {
  Serial.begin(9600);
  Serial.println("STARTING SERIAL READER...");
  Serial.println("------------------------");
}


void loop() {

  if(Serial.available() > 0) {
    rcvd = Serial.read();
    if (rcvd == '\n' || rcvd == '\r' || rcvd == '\0' || rcvd == '#') {
      Serial.println();
    }
    else{
      Serial.print(rcvd);
    }
  }
}