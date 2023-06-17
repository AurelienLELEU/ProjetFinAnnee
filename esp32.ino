#include <Wire.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <KalmanFilter.h>
#include <AccelStepper.h>
#include <PID_v1.h>

const char* ssid = "VotreSSIDWiFi";
const char* password = "VotreMotDePasseWiFi";

const int pinsTrig[] = {2, 3, 4, 5, 6};
const int pinsEcho[] = {7, 8, 9, 10, 11};
const int nbCapteurs = 5;

const int adresseMPU6050 = 0x68; // Adresse I2C du MPU-6050

KalmanFilter kf;

AsyncWebServer serveur(80);

struct Localisation {
  float x;
  float y;
};

const int tailleHistoriqueMax = 100;
Localisation historique[tailleHistoriqueMax];
int indexHistorique = 0;

AccelStepper moteurPasX(AccelStepper::DRIVER, 12, 13);
AccelStepper moteurPasY(AccelStepper::DRIVER, 14, 15);

double consigneX, mesureX, sortieX;
double consigneY, mesureY, sortieY;
double Kp = 1, Ki = 0, Kd = 0;
PID pidX(&mesureX, &sortieX, &consigneX, Kp, Ki, Kd, DIRECT);
PID pidY(&mesureY, &sortieY, &consigneY, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < nbCapteurs; i++) {
    pinMode(pinsTrig[i], OUTPUT);
    pinMode(pinsEcho[i], INPUT);
  }

  Wire.begin();
  Wire.beginTransmission(adresseMPU6050);
  Wire.write(0x6B); // Registre PWR_MGMT_1
  Wire.write(0);    // Réveiller le MPU-6050
  Wire.endTransmission(true);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connexion au WiFi...");
  }

  moteurPasX.setMaxSpeed(1000);
  moteurPasX.setAcceleration(500);
  moteurPasY.setMaxSpeed(1000);
  moteurPasY.setAcceleration(500);

  serveur.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<html><body>";
    // Afficher la carte et l'historique ici
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  serveur.begin();
}

void loop() {
  float donneesCapteurs[nbCapteurs] = {0};
  for (int i = 0; i < nbCapteurs; i++) {
    digitalWrite(pinsTrig[i], LOW);
    delayMicroseconds(2);
    digitalWrite(pinsTrig[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(pinsTrig[i], LOW);

    float duree = pulseIn(pinsEcho[i], HIGH);
    donneesCapteurs[i] = duree * 0.034 / 2.0; // Convertir en distance en cm
  }

  // Obtenir les données du MPU-6050
  Wire.beginTransmission(adresseMPU6050);
  Wire.write(0x3B); // Commencer à l'adresse du registre 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(adresseMPU6050, 14, true); // Demander 14 octets au MPU-6050

  int16_t donneesAccelerometre[3];
  int16_t donneesGyro[3];

  for (int i = 0; i < 3; i++) {
    donneesAccelerometre[i] = Wire.read() << 8 | Wire.read();
  }

  // Traiter les données des capteurs en utilisant le filtre de Kalman et les algorithmes SLAM
  // Mettre à jour la carte et l'historique avec les données filtrées

  // Sauvegarder la position actuelle dans l'historique
  historique[indexHistorique].x = positionFiltreeX;
  historique[indexHistorique].y = positionFiltreeY;
  indexHistorique = (indexHistorique + 1) % tailleHistoriqueMax;

  // Vérifier la détection de la sortie
  bool sortieDetectee = false;
  for (int i = 0; i < nbCapteurs; i++) {
    if (donneesCapteurs[i] == 0) {
      sortieDetectee = true;
      break;
    }
  }

  // Mettre à jour les consignes du PID
  consigneX = historique[indexHistorique].x; // Définir la consigne sur la coordonnée x actuelle
  consigneY = historique[indexHistorique].y; // Définir la consigne sur la coordonnée y actuelle

  // Mettre à jour les mesures du PID
  mesureX = /* Votre mesure actuelle de la position sur l'axe X */;
  mesureY = /* Votre mesure actuelle de la position sur l'axe Y */;

  // Calculer les sorties du PID
  pidX.Compute();
  pidY.Compute();

  // Déplacer les moteurs pas à pas en fonction des sorties du PID
  moteurPasX.runSpeed();
  moteurPasY.runSpeed();

  // Afficher la carte et l'historique sur l'interface web
  String donneesCarte = ""; // Stocker les données de la carte ici
  String donneesHistorique = ""; // Stocker les données de l'historique ici

  // Envoyer les données de la carte au client
  serveur.on("/carte", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", donneesCarte);
  });

  // Envoyer les données de l'historique au client
  serveur.on("/historique", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", donneesHistorique);
  });

  delay(1000);
}
