#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//Servo Setup
Servo myServo;
int angle = 0;

void setup() {
  Serial.begin(9600);

  // Initializing Servo Pin
  myServo.attach(9);
  pinMode(13, OUTPUT);

  // OLED initialization
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for(;;);
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println("Ready...");
  display.display();
}

void loop() {

  if (Serial.available() > 0) {

    angle = Serial.parseInt();

    if (angle >= 0 && angle <= 180) {
      digitalWrite(13, HIGH);
      myServo.write(angle);
    }

    // ---- OLED DISPLAY UPDATE ----
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);

    display.print("Angle: ");
    display.print(angle);

    display.display();

  }
}
