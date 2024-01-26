#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define BUZZER_PIN 4
#define IR_SENSOR_PIN 25
#define MQ2_SENSOR_PIN 27
#define MQ3_SENSOR_PIN 26
#define RELAY_PIN 18

#define THRESHOLD_MQ2 3800
#define THRESHOLD_MQ3 4000
#define TEMPERATURE_THRESHOLD 40

OneWire oneWire(33);
DallasTemperature sensors(&oneWire);

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Task handles
TaskHandle_t sensorTaskHandle;
TaskHandle_t lcdTaskHandle;

// Shared variables
volatile bool mq2Alert = false;
volatile bool mq3Alert = false;
volatile bool tempAlert = false;
volatile bool irAlert = false;
float temperatureCelsius = 0;

unsigned long irSensorStartTime = 0;
unsigned long buzzerStartTime = 0;

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(MQ2_SENSOR_PIN, INPUT);
  pinMode(MQ3_SENSOR_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  Serial.begin(9600);
  sensors.begin();
  lcd.begin();
  lcd.setCursor(0,0);
  lcd.print("Smart Drivers");
  lcd.setCursor(0,1);
  lcd.print("  Safety Device");
  delay(2000);
  lcd.clear();

  // Create the sensor task
  xTaskCreate(sensorTask, "SensorTask", 2048, NULL, 1, &sensorTaskHandle);

  // Create the LCD task
  xTaskCreate(lcdTask, "LCDTask", 2048, NULL, 1, &lcdTaskHandle);
}

void loop() {
  // Empty loop. All tasks are running independently.
}

void sensorTask(void *pvParameters) {
  for (;;) {
    // Check IR sensor
    if (digitalRead(IR_SENSOR_PIN) == LOW) {
      if (millis() - irSensorStartTime >= 5000) {
        buzzerBeep();
        irAlert = true;
        Serial.println("EYE Alert");
      }
    } else {
      irSensorStartTime = millis();
      digitalWrite(BUZZER_PIN, LOW);
      irAlert = false;
    }

    // Check MQ2 smoke sensor
    int mq2Value = analogRead(MQ2_SENSOR_PIN);
    Serial.print("mq2 v:"); 
    Serial.println(mq2Value);
    mq2Alert = mq2Value > THRESHOLD_MQ2;

    // Check MQ3 smoke sensor
    int mq3Value = analogRead(MQ3_SENSOR_PIN);
    Serial.print("mq3 v:"); 
    Serial.println(mq3Value);
    mq3Alert = mq3Value > THRESHOLD_MQ3;

    // Check temperature sensor
    sensors.requestTemperatures();
    temperatureCelsius = sensors.getTempCByIndex(0);
    tempAlert = temperatureCelsius > TEMPERATURE_THRESHOLD;

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay 1 second
  }
}

void lcdTask(void *pvParameters) {
  for (;;) {
    // Display sensor readings on the LCD
    lcd.setCursor(0, 0);
    lcd.print("Eng Temp:");
    lcd.print(temperatureCelsius);
    lcd.print("C");

    lcd.setCursor(0, 1);
    if (mq2Alert) {
         buzzerBeep();
      lcd.print("Smoke Alert!   ");
      Serial.println("Smoke Alert");
    } else
    {
       lcd.print("Smoke NotFound");
    }
    delay(1000);
    lcd.setCursor(0, 1);
    if (mq3Alert) {
         buzzerBeep();
      lcd.print("Alcohol Alert!  ");
      Serial.println("Alcohol Alert");
       
    } else
    {
       lcd.print("Alcohol NotFound");
    }
    
    delay(1000);
    lcd.setCursor(0, 1);
    
    if (tempAlert) {
         buzzerBeep();
      lcd.print("Temperature Alert");
      Serial.println("Temperature Alert");
    } else
  {
     lcd.print("Temp Normal      ");
  }
  
    delay(1000);
    lcd.setCursor(0, 1);
    
    if (irAlert) {
      lcd.print("Sleep Alert   ");
    } else {
      lcd.print("Sleep NotFound");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay 1 seconds
    lcd.clear();

  if (mq3Alert || tempAlert)
  {
      digitalWrite(RELAY_PIN, HIGH);
  }
  else {
      digitalWrite(RELAY_PIN, LOW);
  }
  }
}

void buzzerBeep() {
  if (millis() - buzzerStartTime >= 1000) {
    int buzzerState = digitalRead(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, !buzzerState);
    buzzerStartTime = millis();
  }
}
