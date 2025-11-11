const uint8_t SENSOR_PIN = A5;

void setup() {
  Serial.begin(115200);
  pinMode(SENSOR_PIN, INPUT);
}

void loop() {
  int value = analogRead(SENSOR_PIN); // 0–1023 for QTR-xA
  Serial.println(value);
  delay(50);
}