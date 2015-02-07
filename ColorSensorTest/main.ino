#include "SoftI2CMaster.h"
#include "TCS34725.h"

int pinLED = A4;
int pinAssaySDA = D3;
int pinAssaySCL = D4;
int pinControlSDA = D5;
int pinControlSCL = D6;

TCS34725 tcsAssay = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, pinAssaySDA, pinAssaySCL);

void setup() {
    Serial.begin(9600);
    while(!Serial.available()) {
        Spark.process();
    }

    Serial.println("Color View Test");

    pinMode(pinLED, OUTPUT);
    pinMode(pinAssaySDA, OUTPUT);
    pinMode(pinAssaySCL, OUTPUT);
    pinMode(pinControlSDA, OUTPUT);
    pinMode(pinControlSCL, OUTPUT);

    if (tcsAssay.begin()) {
        Serial.println("Found sensor");
    }
    else {
        Serial.println("TCS34725 not found");
    }
    analogWrite(pinLED, 20);
}

void loop() {
    uint16_t clear, red, green, blue;

    tcsAssay.getRawData(&red, &green, &blue, &clear);

    Serial.print("C:\t"); Serial.print(clear);
    Serial.print("\tR:\t"); Serial.print(red);
    Serial.print("\tG:\t"); Serial.print(green);
    Serial.print("\tB:\t"); Serial.println(blue);

    delay(2000);
}
