#include <ZumoMotors.h>
#include <QTRSensors.h>

// ========== CONFIGURACIÓN DE PINES ==========
#define FRONT_SENSOR A0    // Sensor Sharp frontal
#define LEFT_SENSOR A1     // Sensor Sharp izquierdo
#define RIGHT_SENSOR A2    // Sensor Sharp derecho
#define REAR_SENSOR A3     // Sensor Sharp trasero
#define SWITCH_FRONT 2     // Switch frontal (no usado actualmente)
#define SWITCH_REAR 3      // Switch izquierdo - control sensor trasero
#define SWITCH_RIGHT 4     // Switch derecho (no usado actualmente)

// Configuración de 4 sensores de línea (minimizados)
#define NUM_LINE_SENSORS 4
unsigned char lineSensorPins[NUM_LINE_SENSORS] = {5, 6, 7, 8}; // Pines digitales

// ========== PARÁMETROS AJUSTABLES ==========
#define OBSTACLE_DISTANCE 30      // Distancia para detección frontal/lateral (cm)
#define REAR_OBSTACLE_DISTANCE 20 // Distancia para detección trasera (cm)
#define LINE_THRESHOLD 500        // Valor umbral para detección de línea

// Velocidades de los motores
#define MAX_SPEED 400             // Velocidad máxima de avance
#define REVERSE_SPEED -400        // Velocidad máxima de retroceso
#define TURN_SPEED 350            // Velocidad de giro
#define ALIGN_SPEED 200           // Velocidad para alineación fina
#define SEARCH_SPEED 200          // Velocidad de búsqueda

// ========== OBJETOS Y VARIABLES GLOBALES ==========
ZumoMotors motors;                // Controlador de motores
QTRSensorsRC qtr(lineSensorPins, NUM_LINE_SENSORS); // Sensores de línea

bool rearDetectionEnabled = false; // Control de sensor trasero
unsigned long lastSearchChange = 0;
int searchMode = 0;

// ========== FUNCIÓN SETUP ==========
void setup() {
  Serial.begin(9600);
  
  // Configuración de pines de interruptores
  pinMode(SWITCH_FRONT, INPUT_PULLUP);
  pinMode(SWITCH_REAR, INPUT_PULLUP);
  pinMode(SWITCH_RIGHT, INPUT_PULLUP);
  
  // Calibración de sensores de línea
  calibrateLineSensor();
}

// ========== FUNCIÓN PRINCIPAL LOOP ==========
void loop() {
  // 1. Leer estado de los interruptores
  rearDetectionEnabled = !digitalRead(SWITCH_REAR); // Control sensor trasero
  
  // 2. Leer todos los sensores
  int frontDist = readSharpSensor(FRONT_SENSOR);
  int leftDist = readSharpSensor(LEFT_SENSOR);
  int rightDist = readSharpSensor(RIGHT_SENSOR);
  int rearDist = rearDetectionEnabled ? readSharpSensor(REAR_SENSOR) : 999;
  
  unsigned int lineValues[NUM_LINE_SENSORS];
  qtr.readLine(lineValues);
  bool lineDetected = checkLineDetection(lineValues);

  // 3. Comprobar si está en el borde del dohyo
  if (lineDetected) {
    evadeEdge();
    return;
  }

  // 4. Lógica principal de ataque
  if (frontDist < OBSTACLE_DISTANCE) {
    attackFront(frontDist, leftDist, rightDist);
  } 
  else if (leftDist < OBSTACLE_DISTANCE) {
    attackLeft();
  } 
  else if (rightDist < OBSTACLE_DISTANCE) {
    attackRight();
  } 
  else if (rearDetectionEnabled && rearDist < REAR_OBSTACLE_DISTANCE) {
    turnAround();
  }
  else {
    searchOpponent();
  }
}

// ========== FUNCIONES DE COMPORTAMIENTO ==========

void evadeEdge() {
  // Retrocede y gira aleatoriamente al detectar línea
  motors.setSpeeds(REVERSE_SPEED, REVERSE_SPEED);
  delay(300);
  motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
  delay(random(200, 400));
}

void attackFront(int front, int left, int right) {
  // Ajuste fino de alineación
  if (abs(left - right) > 5) {
    if (left < right) {
      motors.setSpeeds(MAX_SPEED, ALIGN_SPEED);
    } else {
      motors.setSpeeds(ALIGN_SPEED, MAX_SPEED);
    }
    delay(30);
  }
  // Ataque frontal máximo
  motors.setSpeeds(MAX_SPEED, MAX_SPEED);
}

void attackLeft() {
  // Giro de 90° izquierda y ataque
  motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
  delay(250);
  motors.setSpeeds(MAX_SPEED, MAX_SPEED);
  delay(100);
}

void attackRight() {
  // Giro de 90° derecha y ataque
  motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
  delay(250);
  motors.setSpeeds(MAX_SPEED, MAX_SPEED);
  delay(100);
}

void turnAround() {
  // Giro de 180° cuando detecta oponente trasero
  motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
  delay(500);
}

void searchOpponent() {
  // Patrón de búsqueda con cambio periódico
  if (millis() - lastSearchChange > 1500) {
    searchMode = random(0, 4);
    lastSearchChange = millis();
  }

  switch (searchMode) {
    case 0: // Avance lento
      motors.setSpeeds(SEARCH_SPEED, SEARCH_SPEED);
      break;
    case 1: // Giro derecha
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
      break;
    case 2: // Giro izquierda
      motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
      break;
    case 3: // Avance en curva
      motors.setSpeeds(SEARCH_SPEED, SEARCH_SPEED/2);
      break;
  }
}

// ========== FUNCIONES AUXILIARES ==========

bool checkLineDetection(unsigned int* values) {
  for (int i = 0; i < NUM_LINE_SENSORS; i++) {
    if (values[i] > LINE_THRESHOLD) return true;
  }
  return false;
}

int readSharpSensor(int pin) {
  int raw = analogRead(pin);
  // Fórmula aproximada para GP2Y0A21YK0F (10-80cm)
  return 2076 / (raw - 11); // Ajustar según sensor específico
}

void calibrateLineSensor() {
  // Calibración automática durante 2 segundos
  for (int i = 0; i < 100; i++) {
    qtr.calibrate();
    delay(20);
  }
}