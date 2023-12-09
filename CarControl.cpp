#include "CarControl.h"
#include <Arduino.h>

CarControl::CarControl(int modePin, int btTx, int btRx, int motor1A, int motor1B, int motor2A, int motor2B, int rightSensor, int leftSensor)
    : modo(modePin), bluetoothTx(btTx), bluetoothRx(btRx), bluetooth(SoftwareSerial(btTx, btRx)), Motor1A(motor1A), Motor1B(motor1B), Motor2A(motor2A), Motor2B(motor2B), rs(rightSensor), ls(leftSensor) {}

void CarControl::setup() {
    Serial.begin(9600);
    bluetooth.begin(115200);
    bluetooth.print("$$$");
    delay(100);
    bluetooth.println("U,9600,N");
    bluetooth.begin(9600);

    pinMode(Motor1A, OUTPUT);
    pinMode(Motor2A, OUTPUT);
    pinMode(Motor1B, OUTPUT);
    pinMode(Motor2B, OUTPUT);

    digitalWrite(Motor1A, LOW);
    digitalWrite(Motor2A, LOW);
    digitalWrite(Motor1B, LOW);
    digitalWrite(Motor2B, LOW);

    pinMode(rs, INPUT);
    pinMode(ls, INPUT);
}

void CarControl::loop() {
    if (modo == HIGH) {
        ControladoPorBluetooth();
    } else {
        IF();
    }
}

void CarControl::ControladoPorBluetooth() {
    if (bluetooth.available()) {
        char toSend = (char)bluetooth.read();
        if (toSend == 'S') {
            robotParar();
        }

        if (toSend == 'F' || toSend == 'G' || toSend == 'I') {
            if (flag1 != 1) {
                robotAvance();
            }
        }

        if (toSend == 'B' || toSend == 'H' || toSend == 'J') {
            if (flag1 != 2) {
                flag1 = 2;
                Serial.print("reversa");
                digitalWrite(Motor1B, HIGH);
                analogWrite(Motor1A, 0);
                digitalWrite(Motor2B, HIGH);
                analogWrite(Motor2A, 0);
            }
        }

        if (toSend == 'L' || toSend == 'G' || toSend == 'H') {
            if (flag2 != 1) {
                robotIzquierda();
            }
        } else if (toSend == 'R' || toSend == 'I' || toSend == 'J') {
            if (flag2 != 2) {
                robotDerecha();
            }
        } else {
            if (flag2 != 3) {
                flag2 = 3;
                digitalWrite(Motor2A, LOW);
                analogWrite(Motor2B, LOW);
                digitalWrite(Motor2B, LOW);
                analogWrite(Motor2A, LOW);
            }
        }
    }
}

void CarControl::IF() {
    sensorReading();

    if (leftValue == 1 && rightValue == 0) {
        robotIzquierda();
    } else if (leftValue == 0 && rightValue == 1) {
        robotDerecha();
    } else if (leftValue == 0 && rightValue == 0) {
        robotAvance();
    } else if (leftValue == 1 && rightValue == 1) {
        robotParar();
    }
}

void CarControl::sensorReading() {
    leftValue = digitalRead(ls);
    rightValue = digitalRead(rs);

    Serial.println("El valor del sensor izquierdo es ");
    Serial.println(leftValue);

    Serial.println("El valor del sensor derecho es ");
    Serial.println(rightValue);
}

void CarControl::robotParar() {
    flag1 = 0;
    flag2 = 0;

    digitalWrite(Motor1A, LOW);
    analogWrite(Motor1B, LOW);

    digitalWrite(Motor2A, LOW);
    analogWrite(Motor2B, LOW);
}

void CarControl::robotAvance() {
    Serial.print("adelante");
    flag1 = 1;

    digitalWrite(Motor1A, HIGH);
    analogWrite(Motor1B, 0);
    digitalWrite(Motor2A, HIGH);
    analogWrite(Motor2B, 0);
}

void CarControl::robotIzquierda() {
    flag2 = 1;
    Serial.print("izquierda");

    digitalWrite(Motor2B, HIGH);
    analogWrite(Motor2A, 0);
    digitalWrite(Motor1A, HIGH);
    analogWrite(Motor1B, 0);
}

void CarControl::robotDerecha() {
    flag2 = 2;
    Serial.print("derecha");

    digitalWrite(Motor1B, HIGH);
    analogWrite(Motor1A, 0);
    digitalWrite(Motor2A, HIGH);
    analogWrite(Motor2B, 0);
}




