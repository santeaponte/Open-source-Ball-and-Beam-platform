#include <Wire.h>
#include "MPU6050.h"
#define TRIG 33
#define ECHO 32

MPU6050 mpu;  

// Variables para control de tiempo
unsigned long lastTime = 0;  
const unsigned long sampleInterval = 10; // tiempo de muestreo
float dt = sampleInterval / 1000.0;      

// Referencia
unsigned long lastStepChange = 0;   
const unsigned long stepInterval = 60000; // Cada cuanto cambia la referencia en este caso (60s) 

// Variables para MPU
int16_t ax, ay, az;  
int16_t gx, gy, gz;  

float angleX, angleY;   
float offsetX = 0, offsetY = 0; 

float angleY_filtered = 0.0;
const float alpha = 0.90; //Filtro MPU 

// Sensor de distancia ultrasonido 
const int N = 10;
float buffer[N];
int idx = 0;
float suma = 0;
float offset = 14.63 - 7.25; // Acondicionamiento

// Variables PID motor
float error, prevError = 0;
float integral = 0;
float derivative = 0;
float Kp = 2.5, Ki = 0.18, Kd = 0.075;  // Ganancias iniciales
float controlSignal;

// Variables PI(D) bola
float Kp_bola = 2*0.8;    
float Ki_bola = 0.03;  
//float Kd_bola = 0.12/2;     
float integral_bola = 0;
float prevErrorBola = 0;  
float derivative_bola = 0; 

// Setpoints
float setpoint = 2.0;      // Se usa para ajustar solamente el motor
float setpointBola = 40.0; // Se usa para dejar una referencia fija y no variable
float errorBola = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 conectado correctamente.");
  } else {
    Serial.println("Error: No se detecta el MPU6050.");
    while (1);
  }

  // Calibración inicial de la MPU
  delay(1000);
  long sumX = 0, sumY = 0;
  for (int i = 0; i < 100; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float tempAngleX = atan2(ay, az) * 180.0 / PI;
    float tempAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    sumX += tempAngleX;
    sumY += tempAngleY;
    delay(10);
  }
  offsetX = sumX / 100.0;
  offsetY = sumY / 100.0;

  Serial.println("Calibración completada.");
  Serial.print("OffsetX: "); Serial.println(offsetX);
  Serial.print("OffsetY: "); Serial.println(offsetY);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  for (int i = 0; i < N; i++) buffer[i] = 0;

  // Semilla para los números aleatorios
  randomSeed(analogRead(0));
  lastStepChange = millis(); 
}

void loop() {
  unsigned long currentTime = millis();

  // Cambio de referencia cada 60s
  if (currentTime - lastStepChange >= stepInterval) {
    lastStepChange = currentTime;
    setpointBola = random(10, 51); // valor entero aleatorio entre 10 y 50 cm
  }

  if (currentTime - lastTime >= sampleInterval) {
    lastTime = currentTime;

    // Sensor de distancia
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    long duracion = pulseIn(ECHO, HIGH, 30000);
    float distancia = (duracion > 0) ? duracion * 0.0343 / 2.0 : 0;

    suma -= buffer[idx];
    buffer[idx] = distancia;
    suma += buffer[idx];
    idx = (idx + 1) % N;
    float distanciaFiltrada = suma / N;

    float distanciaCalibrada = distanciaFiltrada - offset;

    // Sensor MPU
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    angleX = atan2(ay, az) * 180.0 / PI - offsetX;
    angleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI - offsetY;
    angleY_filtered = alpha * angleY_filtered + (1 - alpha) * angleY;

    // PI(D) de la bola
    errorBola = setpointBola - distanciaCalibrada;
    integral_bola += errorBola * dt;
    if (integral_bola > 50) integral_bola = 50;
    if (integral_bola < -50) integral_bola = -50;
    //derivative_bola = (errorBola - prevErrorBola) / dt; //Si se quiere usar una acción derivativa se debe activar
    derivative_bola = 0; // Se deja en cero para no usar la derivada del error
    prevErrorBola = errorBola;

    setpoint = Kp_bola * errorBola + Ki_bola * integral_bola + Kd_bola * derivative_bola;

    // Saturador para el angulo maximo y minimo del sistema
    if (setpoint < -1.2) setpoint = -1.2;
    if (setpoint > 4.3) setpoint = 4.3;

    // PID motor
    error = setpoint - angleY_filtered;
    integral += error * dt;
    if (integral > 50) integral = 50;
    if (integral < -50) integral = -50;
    derivative = (error - prevError) / dt;
    prevError = error;

    controlSignal = Kp * error + Ki * integral + Kd * derivative;

    // Saturador para la señal de control simetrica.
    if (controlSignal > 1.65) controlSignal = 1.65;
    if (controlSignal < -1.65) controlSignal = -1.65;

    // Salida para el DAC
    float dacValue = 1.65 + controlSignal; 
    int dacOut = map((int)(dacValue * 1000), 0, 3300, 0, 255);
    dacWrite(25, dacOut);

    Serial.print(currentTime); // Tiempo
    Serial.print(",");
    Serial.print(dacValue); // Salida del DAC
    Serial.print(",");
    Serial.print(angleY_filtered); //Angulo de la viga (posicion del motor)
    Serial.print(",");
    Serial.print(setpointBola); // Referencia de la bola
    Serial.print(",");
    Serial.print(errorBola); // Error de la posicion de la bola
    Serial.print(",");
    Serial.print(error); // Error de la posicion del motor
    Serial.print(",");
    Serial.println(distanciaCalibrada); // Disntacia real de la bola
  }
}
