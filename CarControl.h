#ifndef CarControl_h
#ifndef CarControl_h
#define CarControl_h

#include <SoftwareSerial.h>
#include <Servo.h>

class CarControl {
public:
    CarControl(int modePin, int btTx, int btRx, int motor1A, int motor1B, int motor2A, int motor2B, int rightSensor, int leftSensor);
    void setup();
    void loop();
    void ControladoPorBluetooth();
    void IF();

private:
    int modo;
    int bluetoothTx;
    int bluetoothRx;
    SoftwareSerial bluetooth;
    char NOMBRE[21];
    char PASS[5];
    int Motor1A;
    int Motor1B;
    int Motor2A;
    int Motor2B;
    int rs;
    int ls;
    int lecturaSensorIzq;
    int lecturaSensorDer;
    int leftValue;
    int rightValue;
    int flag1;
    int flag2;

    void sensorReading();
    void robotParar();
    void robotAvance();
    void robotIzquierda();
    void robotDerecha();
};

#endif




