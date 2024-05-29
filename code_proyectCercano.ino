#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PulseSensorPlayground.h>

// Variables
const int PulseWire = A0;        // PulseSensor PURPLE WIRE connected to ANALOG PIN A0
const int LED = LED_BUILTIN;     // The on-board Arduino LED, close to PIN 13
const int motorPin = 9;          // Pin PWM al que está conectado el motor de vibración
const int buttonPin = 2;         // Pin al que está conectado el botón
const int bpmLowerLimit = 50;    // Límite inferior del rango normal de BPM
const int bpmUpperLimit = 100;   // Límite superior del rango normal de BPM
int Threshold = 550;             // Umbral para detectar latidos

const int vibrationDuration = 2000; // Duración de la vibración en milisegundos (2 segundos)
const unsigned long messageDuration = 5000; // Duración del mensaje en milisegundos (5 segundos)

PulseSensorPlayground pulseSensor;  // Crea una instancia del objeto PulseSensorPlayground
LiquidCrystal_I2C lcd(0x27, 16, 2); // Dirección I2C y tamaño de la pantalla LCD

bool isVibrating = false;           // Variable para controlar el estado del motor de vibración
unsigned long vibrationStartTime;   // Tiempo de inicio de la vibración
unsigned long messageStartTime;

// Variables para el filtro de paso bajo
const float alpha = 0.9; // Factor de suavizado ajustado
int filteredBPM = 0;

// Variables para el filtro de media móvil
const int windowSize = 5; // Tamaño de la ventana de media móvil
int bpmWindow[windowSize]; // Buffer circular para almacenar las últimas lecturas de BPM
int windowIndex = 0; // Índice del buffer circular


bool dispositivoEncendido = false; // Estado del dispositivo


// Función para calcular la media móvil
int movingAverage(int *buffer, int size) {
  long sum = 0;
  for (int i = 0; i < size; i++) {
    sum += buffer[i];
  }
  return sum / size;
}

void setup() {
  Serial.begin(115200);            // Para el monitor serie

  // Configurar el objeto PulseSensor
  pulseSensor.analogInput(PulseWire);
  pulseSensor.blinkOnPulse(LED);   // Hacer que el LED parpadee automáticamente con el latido del corazón
  pulseSensor.setThreshold(Threshold);

  // Inicializar la pantalla LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("BPM:");

  // Configurar el motor de vibración
  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, LOW); // Apagar el motor de vibración al principio

  // Configurar el botón
  pinMode(buttonPin, INPUT_PULLUP); // Configurar el pin del botón como entrada con resistencia pull-up interna

  // Verificar que el objeto PulseSensor se haya creado y comience a ver una señal
  if (pulseSensor.begin()) {
    Serial.println("We created a pulseSensor Object!"); // Esto se imprime una vez al encender el Arduino o al reiniciarlo.
  } else {
    Serial.println("Error al inicializar el sensor de pulso");
  }
}

void loop() {

  // Leer el estado del botón
  if (digitalRead(buttonPin) == LOW) {
    // Esperar a que el botón se suelte para evitar rebotes
    delay(50);
    while (digitalRead(buttonPin) == LOW);
    delay(50);

    // Cambiar el estado del dispositivo
    dispositivoEncendido = !dispositivoEncendido;

    // Apagar el motor de vibración y el LED cuando se apague el dispositivo
    if (!dispositivoEncendido) {
      isVibrating = false;
      digitalWrite(motorPin, LOW);
      digitalWrite(LED, LOW); // Apagar el LED
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Apagado");
      delay(1000); // Mostrar el mensaje de apagado por un segundo
      lcd.noBacklight(); // Apagar la iluminación de la pantalla LCD
    } else {
      lcd.backlight(); // Encender la iluminación de la pantalla LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("BPM:");
    }
  }

  // Si el dispositivo está apagado, no hacer nada más
  if (!dispositivoEncendido) {
    return;
  }


  int rawBPM = analogRead(PulseWire); // Leer la señal cruda del sensor

  // Aplicar filtro de paso bajo
  filteredBPM = (int)(alpha * filteredBPM + (1 - alpha) * rawBPM);

  Serial.print("Raw BPM: ");
  Serial.print(rawBPM);
  Serial.print(", Filtered BPM: ");
  Serial.println(filteredBPM);

  if (pulseSensor.sawStartOfBeat()) { // Verificar constantemente si "ocurrió un latido".
    int myBPM = pulseSensor.getBeatsPerMinute(); // Llamar a la función del objeto PulseSensor que devuelve el BPM como un "int".

    // Descartar lecturas fuera de rango
    if (myBPM < 30 || myBPM > 200) {
      return;
    }

    // Agregar la nueva lectura al buffer de media móvil
    bpmWindow[windowIndex] = myBPM;
    windowIndex = (windowIndex + 1) % windowSize;

    // Calcular la media móvil de las lecturas de BPM
    int avgBPM = movingAverage(bpmWindow, windowSize);

    // Mostrar el BPM suavizado en la pantalla LCD
    lcd.setCursor(5, 0);
    lcd.print("    ");
    lcd.setCursor(5, 0);
    lcd.print(avgBPM);

    Serial.println("♥ A HeartBeat Happened!"); // Si la prueba es "true", imprimir un mensaje "a heartbeat happened".
    Serial.print("BPM: ");                     // Imprimir la frase "BPM: "
    Serial.println(avgBPM);                    // Imprimir el valor dentro de avgBPM.

    // Verificar si el BPM está fuera del rango normal
    if (avgBPM < bpmLowerLimit || avgBPM > bpmUpperLimit) {
      // Activar el motor de vibración si el BPM está fuera del rango normal
      if (!isVibrating) {
        isVibrating = true;
        vibrationStartTime = millis(); // Guardar el tiempo de inicio de la vibración
        digitalWrite(motorPin, HIGH);
      }
      if (millis() - messageStartTime >= messageDuration) {
        lcd.setCursor(0, 1);
        lcd.print("Tomate 5 seg.");
        messageStartTime = millis();
      }
    } else {
      // Desactivar el motor de vibración si el BPM está dentro del rango normal
      if (isVibrating) {
        isVibrating = false;
        digitalWrite(motorPin, LOW);
      }
      // Limpiar el mensaje después de cierto tiempo
      if (millis() - messageStartTime >= messageDuration) {
        lcd.setCursor(0, 1);
        lcd.print("               ");
      }
    }
  } else {
    // Mostrar el BPM actual en la pantalla LCD si no se detectó un pico
    lcd.setCursor(5, 0);
    lcd.print("    ");
    lcd.setCursor(5, 0);
    lcd.print(pulseSensor.getBeatsPerMinute());
  }

  // Verificar si ha pasado el tiempo de vibración y apagar el motor
  if (isVibrating && (millis() - vibrationStartTime >= vibrationDuration)) {
    isVibrating = false;
    digitalWrite(motorPin, LOW);
  }

  delay(20); // Considerado como una buena práctica en un sketch simple.
}