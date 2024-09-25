#include <Arduino.h>

#define DHT11_PIN 21  // GPIO pin connected to DHT11 sensor

// Global variables for temperature and humidity
float temperature = 0.0;
float humidity = 0.0;

// Function to read DHT11 sensor data
bool readDHT11() {
  uint8_t data[5] = {0}; // Array to hold the received data
  uint8_t bitIndex = 0;
  unsigned long startTime;

  // 1. Send start signal to the DHT11
  pinMode(DHT11_PIN, OUTPUT);
  digitalWrite(DHT11_PIN, LOW);  // Pull pin low for at least 18ms
  delay(20);
  digitalWrite(DHT11_PIN, HIGH); // Pull pin high for 20-40µs
  delayMicroseconds(40);
  pinMode(DHT11_PIN, INPUT);     // Set pin as input

  // 2. Wait for DHT11 response
  startTime = micros();
  while (digitalRead(DHT11_PIN) == HIGH) {
    if ((micros() - startTime) > 85) return false;  // Timeout, no response
  }

  // 3. Wait for the response signal to finish (80µs LOW, 80µs HIGH)
  startTime = micros();
  while (digitalRead(DHT11_PIN) == LOW) {
    if ((micros() - startTime) > 85) return false;  // Timeout
  }
  
  startTime = micros();
  while (digitalRead(DHT11_PIN) == HIGH) {
    if ((micros() - startTime) > 85) return false;  // Timeout
  }

  // 4. Read 40 bits of data
  for (uint8_t i = 0; i < 40; i++) {
    // Wait for 50µs low level
    startTime = micros();
    while (digitalRead(DHT11_PIN) == LOW) {
      if ((micros() - startTime) > 85) return false;  // Timeout
    }

    // Measure the length of the high pulse
    startTime = micros();
    while (digitalRead(DHT11_PIN) == HIGH) {
      if ((micros() - startTime) > 85) return false;  // Timeout
    }

    // If high pulse > 40µs, it's a 1, otherwise it's a 0
    if ((micros() - startTime) > 40) {
      data[bitIndex / 8] |= (1 << (7 - (bitIndex % 8)));
    }

    bitIndex++;
  }

  // 5. Validate the checksum
  if (data[4] == (data[0] + data[1] + data[2] + data[3])) {
    humidity = data[0] + data[1] * 0.1;
    temperature = data[2] + data[3] * 0.1;
    return true;
  } else {
    return false;  // Invalid checksum
  }
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (readDHT11()) {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
    
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
  } else {
    Serial.println("Failed to read from DHT11 sensor");
  }
  
  delay(2000);  // Delay between readings
}
