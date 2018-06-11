int ledPin = 9;
int tasterPin = 11;


void setup() {
  Serial.begin(9600);
  Serial.println("Testprogramm zum Einlesen eines Tastersignals");
  pinMode(ledPin, OUTPUT);
  pinMode(tasterPin, INPUT);

}

void loop() {
  if (digitalRead(tasterPin) == HIGH) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

int Data = digitalRead(tasterPin);
Serial.print("Port");Serial.print(tasterPin);Serial.print(": ");
Serial.println(Data);



}

