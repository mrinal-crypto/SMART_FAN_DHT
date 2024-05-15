#include <DHT.h>
#include <EEPROM.h>

#define DHTPIN 12
#define PWM 6
#define POT A1
#define POWER 2
#define MODE_SELECT 3
#define RELAY 4
#define BUZ 9

#define DHTTYPE DHT11

int speedValue;
int fanLastState;
int lastPotValue;
float factor;

unsigned int switchingInterval = 1000;
bool autoModeSelect;

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();

  pinMode(PWM, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(BUZ, OUTPUT);
  pinMode(POT, INPUT);
  pinMode(POWER, INPUT);
  pinMode(MODE_SELECT, INPUT);
  delay(100);
  fanLastState = EEPROM.read(0);
  autoModeSelect = EEPROM.read(1);
  //  Serial.println(autoModeSelect);
  delay(100);
  lastPotValue = analogRead(POT);
  delay(100);
  int speedMappingValue = map(lastPotValue, 0, 1023, 0, 255);
  float temperature = dht.readTemperature();
  speedFactor(temperature, speedMappingValue);
  delay(100);
}

void loop()
{
  modeSelect();
  powerSwitch();
  fanPower(fanLastState, speedValue);


  if (autoModeSelect == false) {
    manualSpeedControll();
  }
  if (autoModeSelect == true) {
    autoSpeedControll();
  }

//  delay(1000);
}

////////////////////////////////////////////////////////////////////////
void speedFactor(float tempValue, int pwmValue) {
  factor = pwmValue / tempValue;
  //  Serial.println(factor);
}

/////////////////////////////////////////////////////////////////////////
void autoSpeedControll() {
  Serial.println("auto mode selected!");

  int currentPotValue = analogRead(POT);
  int speedMappingValue = map(currentPotValue, 0, 1023, 0, 255);
  float temperature = dht.readTemperature();

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" speedValue: ");
  Serial.print(speedValue);
  Serial.print(" factor: ");
  Serial.println(factor);

  if (abs(currentPotValue - lastPotValue) > 4) {
    Serial.println("speed factor changing!");

    speedFactor(temperature, speedMappingValue);
    int pwmValue = factor * temperature;

    if (pwmValue > 255) {
      Serial.println("speed limit over! set to max");
      buzAlert();
      speedValue = 255;
    } else {
      speedValue = pwmValue;
    }

    lastPotValue = currentPotValue;

  } else {
    int pwmValue = factor * temperature;
    if (pwmValue > 255) {
      Serial.print("speed limit over! set to max");
      buzAlert();
      speedValue = 255;
    } else {
      speedValue = pwmValue;
    }
  }

}
/////////////////////////////////////////////////////////////////////////
void manualSpeedControll() {
  Serial.println("manual mode selected!");

  int readValue = analogRead(POT);
  speedValue = map(readValue, 0, 1023, 0, 255);

  Serial.print("speed: ");
  Serial.println(speedValue);

}
/////////////////////////////////////////////////////////////////////////
void powerSwitch() {
  if (digitalRead(POWER) == HIGH) {
    buzAlert();
    if (digitalRead(RELAY) == HIGH) {
      Serial.println("POWERING OFF");
      delay(switchingInterval);
      EEPROM.update(0, LOW);
      fanLastState = LOW;
    }
    if (digitalRead(RELAY) == LOW) {
      Serial.println("POWERING ON");
      delay(switchingInterval);
      EEPROM.update(0, HIGH);
      fanLastState = HIGH;
    }
  }
}
/////////////////////////////////////////////////////////////////////////
void fanPower(int lastState, int speed) {
  digitalWrite(RELAY, lastState);
  analogWrite(PWM, speed);
  //  Serial.println(fanLastState);
}
////////////////////////////////////////////////////////////////////////
void buzAlert() {
  digitalWrite(BUZ, HIGH);
  delay(100);
  digitalWrite(BUZ, LOW);
  delay(50);
  digitalWrite(BUZ, HIGH);
  delay(100);
  digitalWrite(BUZ, LOW);
}
////////////////////////////////////////////////////////////////////////
void modeSelect() {

  if (digitalRead(MODE_SELECT) == HIGH) {
    buzAlert();
    autoModeSelect = true;

    delay(1000);
    if (digitalRead(MODE_SELECT) == HIGH) {
      digitalWrite(BUZ, HIGH);
      delay(2000);
      digitalWrite(BUZ, LOW);
      autoModeSelect = false;
    }
    EEPROM.update(1, autoModeSelect);
    Serial.println(autoModeSelect);

  }
}
/////////////////////////////////////////////////////////////////////////
