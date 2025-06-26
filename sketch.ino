#define BLYNK_TEMPLATE_ID "TMPL2WcNf4F2N"
#define BLYNK_TEMPLATE_NAME "smart home"
#define BLYNK_AUTH_TOKEN "NMPzF6z75zj39vkCq1vXDifXLh_ZBTrh"

const char* ssid = "Wokwi-GUEST";
const char* password = "";

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <ESP32Servo.h>

// Configuration du clavier matriciel
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {32, 33, 34, 35};
byte colPins[COLS] = {27, 14, 12, 13};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Configuration du capteur DHT
#define DHT_PIN 16
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// Configuration du capteur PIR
#define PIR_PIN 17

// Configuration de l'écran LCD
#define SDA_PIN 21
#define SCL_PIN 22
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialisation de la connexion MQTT
const char* mqttServer = "broker.mqttdashboard.com";
const int mqttPort = 1883;
const char* mqttUser = "";  // Nom d'utilisateur MQTT si nécessaire
const char* mqttPassword = "";
WiFiClient espClient;
PubSubClient client(espClient);

// Configuration du servo
#define SERVO_PIN 15
Servo myServo;

// LED pour indication
#define LED_GREEN_PIN 4
#define LED_RED_PIN 5

// Variables de temps
unsigned long previousMillisSensors = 0;
const long intervalSensors = 1000;
bool doorOpen = false;
unsigned long doorOpenTime = 0;
unsigned long ledChangeTime = 0;
bool ledState = false;

void setup() {
  // Initialisation
  Serial.begin(115200);
  
  // Connexion Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connexion au Wi-Fi...");
  }
  Serial.println("Connecté au Wi-Fi!");

  // Connexion Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

  // Initialisation capteur DHT
  dht.begin();

  // Initialisation capteur PIR
  pinMode(PIR_PIN, INPUT);

  // Initialisation LCD
  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Villa 12:");

  // Initialisation servo
  myServo.attach(SERVO_PIN); 
  myServo.write(0); // Position initiale

  // Initialisation LED
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
}

void loop() {
  Blynk.run(); // Nécessaire pour la connexion Blynk

  // Connexion au serveur MQTT si nécessaire
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();  // Maintien de la connexion MQTT active

  unsigned long currentMillis = millis();

  // Mise à jour des capteurs
  if (currentMillis - previousMillisSensors >= intervalSensors) {
    previousMillisSensors = currentMillis;
    updateSensors();
  }

  // Lecture des touches du clavier
  char key = keypad.getKey();
  if (key) {
    handleKeypadInput(key);
    delay(200); // Délai pour éviter les répétitions rapides
  }

  // Gestion de l'ouverture et fermeture de la porte avec changement de LED
  if (doorOpen) {
    if (currentMillis - doorOpenTime >= 7000) {
      myServo.write(0); // Ferme la porte
      doorOpen = false;
      doorOpenTime = currentMillis; // Met à jour le temps de fermeture
      Serial.println("Porte fermée automatiquement");
      digitalWrite(LED_GREEN_PIN, LOW);  // Éteindre LED verte
      digitalWrite(LED_RED_PIN, HIGH);   // Allumer LED rouge
    }
  } else {
    if (currentMillis - doorOpenTime >= 7000) {
      myServo.write(90); // Ouvre la porte
      doorOpen = true;
      doorOpenTime = currentMillis; // Met à jour le temps d'ouverture
      Serial.println("Porte ouverte automatiquement");
      digitalWrite(LED_RED_PIN, LOW); // Éteindre LED rouge
      digitalWrite(LED_GREEN_PIN, HIGH); // Allumer LED verte
    }
  }
}

void reconnectMQTT() {
  // Tentative de connexion au serveur MQTT si déconnecté
  client.setServer(mqttServer, mqttPort);
  while (!client.connected()) {
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("Connecté au serveur MQTT");
    } else {
      Serial.print("Échec de la connexion MQTT, code erreur : ");
      Serial.println(client.state());
      delay(8000);
    }
  }
}

void updateSensors() {
  // Lecture du capteur DHT
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  if (!isnan(temperature) && !isnan(humidity)) {
    Blynk.virtualWrite(V0, temperature);
    Blynk.virtualWrite(V1, humidity);
    client.publish("home/temperature", String(temperature).c_str());
    client.publish("home/humidity", String(humidity).c_str());
  }

  // Lecture du capteur PIR
  int motionDetected = digitalRead(PIR_PIN);
  String motionState = motionDetected ? "Mouvement détecté!" : "Aucun mouvement.";
  static String lastMotionState = "";  // Variable pour vérifier les changements d'état
  if (motionState != lastMotionState) {
    Serial.print("État du mouvement: ");
    Serial.println(motionState);
    Blynk.virtualWrite(V2, motionState.c_str());
    client.publish("home/motion", motionState.c_str());
    lastMotionState = motionState;  // Mise à jour de l'état précédent
  }

  String doorState = doorOpen ? "Porte ouverte" : "Porte fermée";
  static String lastDoorState = "";  // Variable pour vérifier les changements d'état
  if (doorState != lastDoorState) {
    Blynk.virtualWrite(V3, doorState);
    lastDoorState = doorState;  // Mise à jour de l'état précédent
  }
}


void handleKeypadInput(char key) {
  // Affiche la touche pressée dans le moniteur série
  Serial.print("Touche pressée: ");
  Serial.println(key);

  if (key == 'A') {
    // Ouvrir la porte, allumer LED verte et afficher LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Porte ouverte");
    Serial.println("Porte ouverte");
    doorOpen = true;
    doorOpenTime = millis(); // Démarre le timer pour fermeture
    digitalWrite(LED_GREEN_PIN, HIGH);  // Allumer LED verte
    digitalWrite(LED_RED_PIN, LOW);     // Éteindre LED rouge
  } else if (key == 'B') {
    // Fermer la porte, allumer LED rouge et afficher LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Porte fermée");
    Serial.println("Porte fermée");
    doorOpen = false;
    doorOpenTime = millis(); // Démarre le timer pour ouverture
    digitalWrite(LED_GREEN_PIN, LOW);  // Éteindre LED verte
    digitalWrite(LED_RED_PIN, HIGH);   // Allumer LED rouge
  }
}
