#include "main.h"

/////////////////////////////////////////////////////////////
//                                                         //
//                       QR SCANNER                        //
//                                                         //
/////////////////////////////////////////////////////////////

int scan_QR_code(String param) {

    unsigned long timeout;
    int buf, i = 0;

    /*Serial1.flush();*/

    pull_QR_high("");

    timeout = millis() + QR_READ_TIMEOUT;

    while (!Serial1.available() && millis() < timeout) {
      Spark.process();
    }

    pull_QR_low("");

    Serial.print("Number of characters read: ");
    Serial.println(Serial1.available());

    do {
        buf = Serial1.read();
        if (buf != -1) {
          Serial.print(i);
          Serial.print(": ");
          Serial.print(buf);
          Serial.print(", hex: ");
          Serial.print(buf, HEX);
          Serial.print(", char: ");
          qr_uuid[i++] = (char) buf;
          Serial.println((char) buf);
        }
    } while (Serial1.available());
    Serial.println();
    Serial.println("Serial1 data collected");

    qr_uuid[UUID_LENGTH] = '\0';
    Serial.print("qr_uuid: ");
    Serial.println(qr_uuid);

    return 0;
}

int pull_QR_high(String param) {
    Serial.println("Pulling pinQRTrigger high");
    digitalWrite(pinQRTrigger, HIGH);
    return 0;
}

int pull_QR_low(String param) {
    Serial.println("Pulling pinQRTrigger low");
    digitalWrite(pinQRTrigger, LOW);
    return 0;
}

/////////////////////////////////////////////////////////////
//                                                         //
//                          SETUP                          //
//                                                         //
/////////////////////////////////////////////////////////////

void setup() {
  Spark.function("scanqrcode", scan_QR_code);
  Spark.function("pullqrhigh", pull_QR_high);
  Spark.function("pullqrlow", pull_QR_low);

  pinMode(pinQRTrigger, OUTPUT);
  /*pinMode(RX, INPUT);
  pinMode(TX, OUTPUT);*/

  Serial.begin(9600);     // standard serial port
  Serial1.begin(115200);  // QR scanner interface through RX/TX pins

  pull_QR_low("");
}

/////////////////////////////////////////////////////////////
//                                                         //
//                           LOOP                          //
//                                                         //
/////////////////////////////////////////////////////////////


void loop(){
  while (Serial.available()) {
    Serial1.write(Serial.read());
  }

  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}
