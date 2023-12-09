#include "CarControl.h"

// Definir las conexiones de pines aquí
const int modo = A5;
int bluetoothTx = 2;
int bluetoothRx = 3;
int Motor1A = 5;
int Motor1B = 6;
int Motor2A = 9;
int Motor2B = 10;
int rs = A4;
int ls = A3;

CarControl carControl(modo, bluetoothTx, bluetoothRx, Motor1A, Motor1B, Motor2A, Motor2B, rs, ls);

void setup() {
    carControl.setup();
}

void loop() {
    carControl.loop();
}
