#include "main.h"

/////////////////////////////////////////////////////////////
//                                                         //
//                       QR SCANNER                        //
//                                                         //
/////////////////////////////////////////////////////////////

int scan_QR_code(String param) {

    unsigned long timeout;
    int buf, i = 0;

    Serial1.begin(9600);  // QR scanner interface through RX/TX pins

    Serial.println("Pulling pinQRTrigger low");
    digitalWrite(pinQRTrigger, LOW);

    timeout = millis() + QR_READ_TIMEOUT;

    do {
        buf = Serial1.read();
        if (buf == -1) {
          Serial.print(".");
        }
        else {
          Serial.println(i);
          qr_uuid[i++] = (char) buf;
        }
    } while (i < UUID_LENGTH && millis() < timeout);
    Serial.println();
    Serial.println("Serial1 data collected");

    qr_uuid[UUID_LENGTH] = '\0';
    Serial.print("qr_uuid: ");
    Serial.println(qr_uuid);

    Serial.println("Pulling pinQRTrigger high");
    digitalWrite(pinQRTrigger, HIGH);

    Serial1.end();  // QR scanner interface through RX/TX pins

    return 0;
}


/////////////////////////////////////////////////////////////
//                                                         //
//                          SETUP                          //
//                                                         //
/////////////////////////////////////////////////////////////

void setup() {
  Spark.function("scanqrcode", scan_QR_code);

  pinMode(pinQRTrigger, OUTPUT);

  digitalWrite(pinQRTrigger, HIGH);

  Serial.begin(9600);     // standard serial port
}

/////////////////////////////////////////////////////////////
//                                                         //
//                           LOOP                          //
//                                                         //
/////////////////////////////////////////////////////////////


void loop(){

}
