#include "Adafruit_VC0706.h"
#include "quirc.h"
#include "flashee-eeprom.h"
    using namespace Flashee;

Adafruit_VC0706 cam = Adafruit_VC0706(&Serial1);
FlashDevice* flash;
struct quirc *qr;

void setup() {
    uint8_t *buffer;

    flash = Devices::createWearLevelErase();

    Serial.begin(9600);
    while (!Serial.available()) {
        Spark.process();
    }
    Serial.read();

    if (cam.begin()) {
        Serial.println("Camera found:");
    }
    else {
        Serial.println("Camera not found");
    }

    char *reply = cam.getVersion();
    if (reply == 0) {
      Serial.println("Failed to get version");
    } else {
      Serial.println("-----------------");
      Serial.println(reply);
      Serial.println("-----------------");
    }

    cam.setImageSize(VC0706_320x240);
    /*delay(100);
    cam.setPTZ(640, 480, 0, 0);
    delay(100);
    cam.setToBlackAndWhite();*/

    Serial.println("Snap in 3 secs...");
    delay(3000);

    if (!cam.takePicture()) {
      Serial.println("Failed to snap!");
      return;
    }

    Serial.println("Picture taken!");

    // Get the size of the image (frame) taken
    uint16_t jpglen = cam.frameLength();
    Serial.print("Storing ");
    Serial.print(jpglen, DEC);
    Serial.print(" byte image.");

    int32_t time = millis();
    pinMode(8, OUTPUT);
    // Read all the data up to # bytes!
    byte wCount = 0; // For counting # of writes
    int addr = 800000;
    flash->write(&jpglen, addr, 2);
    addr += 2;
        while (jpglen > 0) {
        uint8_t bytesToRead = min(96, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
        buffer = cam.readPicture(bytesToRead);
        Serial.print('.');

        flash->write(buffer, addr, bytesToRead);

        jpglen -= bytesToRead;
        addr += bytesToRead;
    }
    Serial.println();
    Serial.flush();

    time = millis() - time;
    Serial.println("done!");
    Serial.print(time); Serial.println(" ms elapsed");
    cam.resumeVideo();

    qr = quirc_new();
    if (!qr) {
      Serial.println("Failed to allocate memory for QR decoder");
      return;
    }
    if (quirc_resize(qr, 320, 240) < 0) {
	    perror("Failed to allocate video memory");
	    return;
    }

    delay(1000);

    quirc_destroy(qr);
}

char inBuf[20];
uint16_t scanNumber() {
    int bufLen = 0;

    do {
        while (!Serial.available()) {
            Spark.process();
        }
        inBuf[bufLen] = Serial.read();
        Serial.print(inBuf[bufLen]);
    } while (inBuf[bufLen++] != '\n');

    inBuf[bufLen] = '\0';
    return (uint16_t) atoi(inBuf);
}

void loop() {
    uint16_t pan, tilt, wz, hz;

    Serial.println("Enter pan: ");
    pan = scanNumber();
    Serial.println("Enter tilt: ");
    tilt = scanNumber();
    Serial.println("Enter zoom width: ");
    wz = scanNumber();
    Serial.println("Enter zoom height: ");
    hz = scanNumber();
    Serial.println(pan);
    Serial.println(tilt);
    Serial.println(wz);
    Serial.println(hz);
    cam.setPTZ(wz, hz, pan, tilt);
    delay(10000);
}
