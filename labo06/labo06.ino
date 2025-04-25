#include <HCSR04.h>
#include <LCD_I2C.h>
#include <U8g2lib.h>



#define CLK_PIN 30
#define DIN_PIN 34
#define CS_PIN 32  // Chip Select

U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(
  U8G2_R0,       // rotation
  /* clock=*/ CLK_PIN, // pin Arduino reliée à CLK (horloge)
  /* data=*/ DIN_PIN,  // pin Arduino reliée à DIN (données)
  /* cs=*/ CS_PIN,    // pin Arduino reliée à CS (chip select)
  /* dc=*/ U8X8_PIN_NONE,
  /* reset=*/ U8X8_PIN_NONE
);

#define TRIGGER_PIN 3
#define ECHO_PIN 2
#define buzzer 4
int buzzerPin = 4;
int frequence = 1;

int redPin = 6;
int bluePin = 7;

LCD_I2C lcd(0x27, 16, 2);

HCSR04 hc(TRIGGER_PIN, ECHO_PIN);

const long interval = 100;

enum etat_distance { TROP_PRES,
                     TROP_LOIN };
etat_distance etatDistance = TROP_LOIN;
int distance;
int previousDistance = -1;

unsigned long tempsDepuisLoin = 0;
bool objetEstLoin = false;
const unsigned long delaiExtinction = 3000;
bool firstTime = true;


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


  Serial.begin(9600);
  lcd.begin();
  lcd.backlight();
  lcdstart();
}

void loop() {
  chercherDistance();
  affichage();
  etatSystem();
}




void chercherDistance() {

  distance = hc.dist();
}


void etatSystem() {
  unsigned long currentTimes = millis();
  if (distance <= 15) {

    etatDistance = TROP_PRES;
    objetEstLoin = false;
  } else {
    etatDistance = TROP_LOIN;
    if (!objetEstLoin) {
      objetEstLoin = true;
      tempsDepuisLoin = millis();
    }
  }

  switch (etatDistance) {

    case TROP_PRES:
      tropPres();
      girophare();
      alarme();

      break;

    case TROP_LOIN:
      tropLoin();


      if (currentTimes - tempsDepuisLoin >= delaiExtinction) {
        firstTime = false;
        girophareEteint();
        alarmeOff();
        girophareEteint();
      } else if (firstTime == false) {

        girophare();
        alarme();
      }

      break;
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

void tropLoin() {
  lcd.print("obj  : ");
  lcd.setCursor(7, 1);
  lcd.print("trop loin");
}

void tropPres() {
  lcd.print("obj  : ");
  lcd.setCursor(7, 1);
  lcd.print("trop pret");
}


void alarme() {
  tone(buzzer, frequence);
}
void alarmeOff() {
  noTone(buzzer);
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


void ecranSetup(){
  u8g2.begin();
  u8g2.setContrast(5);
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.clearBuffer(); 
  u8g2.sendBuffer(); 
}

void dessinInterdit(){

  u8g2.clearBuffer();
  u8g2.drawCircle(3, 4, 3);
  u8g2.drawLine(0, 0, 8, 8);
  u8g2.sendBuffer();
}

void dessinX(){
  u8g2.clearBuffer();
  u8g2.drawLine(7, 0, 0, 7);
  u8g2.drawLine(0, 0, 8, 8);
  u8g2.sendBuffer();
}

void dessinWeGood(){
  u8g2.clearBuffer();
  u8g2.drawLine(4, 4, 2, 6);
  u8g2.drawLine(1, 1, 4, 4);
  u8g2.sendBuffer();
  
}
























