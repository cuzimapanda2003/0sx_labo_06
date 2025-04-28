#include <HCSR04.h>
#include <LCD_I2C.h>
#include <U8g2lib.h>
#include <AccelStepper.h>

#define MOTOR_INTERFACE_TYPE 4

#define IN_1 31
#define IN_2 33
#define IN_3 35
#define IN_4 37

AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);



#define CLK_PIN 30
#define DIN_PIN 34
#define CS_PIN 32  // Chip Select

U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(
  U8G2_R0,             // rotation
  /* clock=*/CLK_PIN,  // pin Arduino reliée à CLK (horloge)
  /* data=*/DIN_PIN,   // pin Arduino reliée à DIN (données)
  /* cs=*/CS_PIN,      // pin Arduino reliée à CS (chip select)
  /* dc=*/U8X8_PIN_NONE,
  /* reset=*/U8X8_PIN_NONE);

#define TRIGGER_PIN 3
#define ECHO_PIN 2

int buzzerPin = 4;
int frequence = 1;

int redPin = 6;
int bluePin = 7;

LCD_I2C lcd(0x27, 16, 2);

HCSR04 hc(TRIGGER_PIN, ECHO_PIN);

const long interval = 100;

enum etat_distance { ALERTE,
                     TROP_PROCHE,
                     MOTEUR,
                     TROP_LOIN };
etat_distance etatDistance = TROP_LOIN;
int distance;
int previousDistance = -1;

unsigned long tempsDepuisLoin = 0;
bool objetEstLoin = false;
const unsigned long delaiExtinction = 3000;
bool firstTime = true;
int alerte = 15;


long targetPosition = 0;
long previousTarget = -1;
float degree;

int maxSpeed = 500;
int maxAccel = 100;
int inf = 30;
int sup = 60;

bool alarmeActive = false;
unsigned long alarmeStartTime = 0;

unsigned long tempsAffichageSymbole = 0;
bool symboleActif = false;






void lcdstart() {
  lcd.print("2168637");
  lcd.setCursor(0, 1);
  lcd.print("labo5");
  delay(2000);
}

void setup() {
  pinMode(buzzerPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  ecranSetup();


  Serial.begin(115200);
  lcd.begin();
  lcd.backlight();
  lcdstart();

  myStepper.setMaxSpeed(maxSpeed);
  myStepper.setAcceleration(maxAccel);
}

void loop() {
  chercherDistance();
  affichage();
  etatSystem();
  commande();
  verifierSymbole();
  myStepper.run();
}




void chercherDistance() {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    distance = hc.dist();
  }
}


void etatSystem() {
  unsigned long currentTimes = millis();


  if (distance <= alerte) {
    etatDistance = ALERTE;

    if (!alarmeActive) {
      alarmeActive = true;
      alarmeStartTime = currentTimes;
    }

    objetEstLoin = false;
  } else if (distance > alerte && distance < inf) {
    etatDistance = TROP_PROCHE;
    objetEstLoin = false;
  } else if (distance >= inf && distance <= sup) {
    etatDistance = MOTEUR;
    objetEstLoin = false;
  } else {
    etatDistance = TROP_LOIN;
    objetEstLoin = true;
  }

  switch (etatDistance) {
    case ALERTE:
      tropPres();
      break;
    case TROP_PROCHE:
      tropPres();
      break;
    case MOTEUR:
      targetPosition = map(distance, 30, 60, 0, 1024);
      if (targetPosition != previousTarget) {
        myStepper.moveTo(targetPosition);
        previousTarget = targetPosition;
      }
      affichageMilieu(targetPosition);
      break;
    case TROP_LOIN:
      tropLoin();
      break;
  }


  if (alarmeActive) {
    if (currentTimes - alarmeStartTime <= delaiExtinction) {
      alarme();
      girophare();
    } else {
      alarmeOff();
      girophareEteint();
      alarmeActive = false;
    }
  } else {
    alarmeOff();
    girophareEteint();
  }
}




void affichage() {

  if (distance != previousDistance) {
    lcd.setCursor(6, 0);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print("dist = ");
    lcd.setCursor(8, 0);
    lcd.print(distance);

    lcd.setCursor(6, 1);

    lcd.setCursor(0, 1);
  }
}

void affichageMilieu(long steps) {
  degree = (steps / 2048.0) * 360.0;
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("Deg  : ");
  lcd.print(degree, 1);
}


void tropLoin() {
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("obj  : trop loin");
}


void tropPres() {
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("obj  : trop pret");
}



void alarme() {
  tone(buzzerPin, frequence);
}
void alarmeOff() {
  noTone(buzzerPin);
}
void girophare() {
  static unsigned long lastSwitchTime = 0;
  static bool isRedOn = true;
  unsigned long currentMillis = millis();

  if (currentMillis - lastSwitchTime >= interval) {
    lastSwitchTime = currentMillis;
    isRedOn = !isRedOn;
  }

  if (isRedOn) {
    digitalWrite(redPin, HIGH);
    digitalWrite(bluePin, LOW);
  } else {
    digitalWrite(redPin, LOW);
    digitalWrite(bluePin, HIGH);
  }
}


void girophareEteint() {
  digitalWrite(redPin, LOW);
  digitalWrite(bluePin, LOW);
}


void ecranSetup() {
  u8g2.begin();
  u8g2.setContrast(5);
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

void verifierSymbole() {
  if (symboleActif && millis() - tempsAffichageSymbole >= 3000) {
    u8g2.clearBuffer();
    u8g2.sendBuffer();
    symboleActif = false;
  }
}



void dessinInterdit() {

  u8g2.clearBuffer();
  u8g2.drawCircle(3, 4, 3);
  u8g2.drawLine(0, 0, 8, 8);
  u8g2.sendBuffer();
  symboleActif = true;
  tempsAffichageSymbole = millis();
}

void dessinX() {
  u8g2.clearBuffer();
  u8g2.drawLine(7, 0, 0, 7);
  u8g2.drawLine(0, 0, 8, 8);
  u8g2.sendBuffer();
  symboleActif = true;
  tempsAffichageSymbole = millis();
}

void dessinWeGood() {
  u8g2.clearBuffer();
  u8g2.drawLine(4, 4, 2, 6);
  u8g2.drawLine(1, 1, 4, 4);
  u8g2.sendBuffer();
  symboleActif = true;
  tempsAffichageSymbole = millis();
}


void analyserCommande(const String& tampon, String& commande, String& arg1, String& arg2) {
  commande = "";
  arg1 = "";
  arg2 = "";

  int firstSep = tampon.indexOf(';');
  int secondSep = tampon.indexOf(';', firstSep + 1);

  if (firstSep == -1) {
    // Pas de point-virgule, c'est peut-être "stop" ou autre commande sans paramètre
    commande = tampon;
    return;
  }

  // Extraire la commande
  commande = tampon.substring(0, firstSep);

  // Extraire arg1
  if (secondSep != -1) {
    arg1 = tampon.substring(firstSep + 1, secondSep);

    arg2 = tampon.substring(secondSep + 1);
  } else {
    // Il y a une seule valeur après la commande
    arg1 = tampon.substring(firstSep + 1);
  }
}



void commande() {
  while (Serial.available()) {
    String tampon = Serial.readStringUntil('\n');

    String commande;
    String arg1, arg2;
    analyserCommande(tampon, commande, arg1, arg2);

    bool commandeValide = false;

    if (commande == "g_dist") {
      Serial.println(distance);
      dessinWeGood();
      commandeValide = true;
    } else if (commande == "cfg" && arg1 == "alm") {
      alerte = arg2.toInt();
      dessinWeGood();
      commandeValide = true;
    } else if (commande == "cfg" && arg1 == "lim_inf") {
      if (arg2.toInt() > sup) {
        Serial.println("erreur 🚫");
        dessinInterdit();
      } else {
        inf = arg2.toInt();
        dessinWeGood();
      }
      commandeValide = true;
    } else if (commande == "cfg" && arg1 == "lim_sup") {
      if (arg2.toInt() < inf) {
        Serial.println("erreur 🚫");
        dessinInterdit();
      } else {
        sup = arg2.toInt();
        dessinWeGood();
      }
      commandeValide = true;
    }

    if (!commandeValide && tampon != "") {
      dessinX();
    }
  }
}
