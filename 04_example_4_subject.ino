#define PIN_LED 7
unsigned int on = 0, off = 1;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, on); // turn on LED.
}

void loop() {
  delay(1000);
  for (int i = 0; i<5; i++) {
    digitalWrite(PIN_LED, off); // update LED status.
    delay(100); // wait for 0.1 seconds
    digitalWrite(PIN_LED, on);
    delay(100);
    }
  digitalWrite(PIN_LED, off);
  while (1) {} // infinite loop 
}
