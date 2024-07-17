#include <LiquidCrystal.h>
#include <TinyGPS.h>
#include <Servo.h>

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
Servo waterSprinkler; // Declare Servo object
const int gas_Sensor = 8;
const int flame_Sensor = 9;
const int ultrasonic_trigPin = 10; // Trigger pin of ultrasonic sensor
const int ultrasonic_echoPin = 11; // Echo pin of ultrasonic sensor
const int motor_Enable_A = 12;      // ENA pin of the L298 motor driver
const int motor_Enable_B = 13;      // ENB pin of the L298 motor driver
const int motor_Input_1 = A0;       // IN1 pin of the L298 motor driver       // IN2 pin of the L298 motor driver
const int motor_Input_3 = A2;       // IN3 pin of the L298 motor driver      // IN4 pin of the L298 motor driver
const int buzzer_Pin = A4;          // Buzzer pin
const int pump_Pin = A5;            // Pump control pin
const int servo_Pin = A1;           // Servo control pin
TinyGPS gps;
long lat, lon;
bool fire_Status = LOW;
bool flame_Status = LOW;
bool readGPS = false; // Flag to control GPS reading

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
  // Set pin modes
  pinMode(buzzer_Pin, OUTPUT);
  pinMode(pump_Pin, OUTPUT);
  pinMode(motor_Enable_A, OUTPUT);
  pinMode(motor_Enable_B, OUTPUT);
  pinMode(motor_Input_1, OUTPUT);
  pinMode(motor_Input_3, OUTPUT);
  pinMode(ultrasonic_trigPin, OUTPUT);
  pinMode(ultrasonic_echoPin, INPUT);
  pinMode(gas_Sensor, INPUT);
  pinMode(flame_Sensor, INPUT);
  // Initialize servo
  waterSprinkler.attach(servo_Pin); // Attach the servo to the pin
  // Initialize LCD
  digitalWrite(motor_Enable_A, HIGH);
  digitalWrite(motor_Enable_B, HIGH);
  lcd.begin(16, 2);
  lcd.print("Fire DETECTION");
  lcd.setCursor(0, 2);
  lcd.print(" SYSTEM");
  delay(500);
}

void loop() {
  fire_Status = digitalRead(gas_Sensor);
  flame_Status = digitalRead(flame_Sensor);

  if (fire_Status == HIGH && flame_Status == HIGH) {
    lcd.clear();
    lcd.print("Fire Detected");
    digitalWrite(motor_Enable_A, HIGH);
    digitalWrite(motor_Enable_B, HIGH);
    lcd.print("Distance: ");
    int distance = measureDistance();
    lcd.print(distance);
    lcd.print(" cm");

    digitalWrite(buzzer_Pin, HIGH);
    startMotor(); // Start the motors

    unsigned long startTime = millis();
    while (millis() - startTime < 5000) { // Run for a maximum of 5 seconds
      distance = measureDistance();
      lcd.setCursor(0, 2);
      lcd.print("Distance: ");
      lcd.print(distance);
      lcd.print(" cm");
      if (distance <= 50) {
        break; // Exit the loop if the distance is less than or equal to 50 cm
      }
      delay(100); // Delay to avoid excessive checking
    }

    stopMotor(); // Stop the motors

    SendMessage(); // Send SMS

    readGPS = true; // Set flag to start GPS reading

    while (readGPS) {
      gps_read();
    }

    // Deactivate the motor enable pins
    digitalWrite(motor_Enable_A, LOW);
    digitalWrite(motor_Enable_B, LOW);

    // Activate the pump
    digitalWrite(pump_Pin, HIGH);

    // Activate the water sprinkler
    waterSprinkler.write(90); // Assuming 90 degrees is the desired angle for watering
    delay(1000); // Allow some time for watering
    waterSprinkler.write(0); // Return to the initial position

    lcd.clear();
    lcd.print("Water Pump Started");
    delay(2000);

    digitalWrite(buzzer_Pin, LOW);
    digitalWrite(pump_Pin, LOW);

    delay(100);

    lcd.clear();
    lcd.print("Fire extinguished");

    // Wait for some time before checking again
    delay(5000);
  }
}

void startMotor() {
  digitalWrite(motor_Input_1, HIGH);
  digitalWrite(motor_Input_3, HIGH);
}

void stopMotor() {
  digitalWrite(motor_Input_1, LOW);
  digitalWrite(motor_Input_3, LOW);
}

int measureDistance() {
  // Send ultrasonic pulse
  digitalWrite(ultrasonic_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonic_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonic_trigPin, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ultrasonic_echoPin, HIGH);

  // Calculate the distance
  int distance = duration * 0.034 / 2; // Speed of sound is 0.034 cm/µs

  return distance;
}

void SendMessage() {
  Serial.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(100);  // Delay of 100 milliseconds
  Serial.println("AT+CMGS=\"+917420892460\"\r"); // Replace x with mobile number
  delay(100);
  Serial.println("Fire Detected inside home");// The SMS text you want to send
  delay(100);
  Serial.println((char)26);// ASCII code of CTRL+Z
  delay(10);
}

void gps_read() {
  byte a;

  if (Serial.available()) {
    a = Serial.read();
    if (gps.encode(a)) {
      gps.get_position(&lat, &lon); // get latitude and longitude

      Serial.println("Position: ");
      Serial.print("lat:");
      Serial.println((lat * 0.000001), 8);
      Serial.print("log:");
      Serial.println((lon * 0.000001), 8);

      readGPS = false; // Clear flag to stop GPS reading
    }
  }
}
