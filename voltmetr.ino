#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <U8g2lib.h>

// Налаштування Wi-Fi
const char* ssid = "prol";
const char* password = "korotkov";

// Ініціалізація INA219
Adafruit_INA219 ina219;

// Ініціалізація OLED-дисплея
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 22, /* data=*/ 21, U8X8_PIN_NONE);

// HW-872A (аналоговий датчик струму)
#define HW872A_PIN 34           
#define ADC_RESOLUTION 4095.0   
#define MAX_VOLTAGE 3.3         
#define SHUNT_RESISTOR 0.01     

// Налаштування реле
#define RELAY_PIN 5
bool relayState = false;        
bool manualControl = false;     
bool relayLocked = false;       

// Параметри акумулятора
float initialCapacity = 65.0;   // Початкова ємність (Ah)
float PeukertExponent = 1.3;    // Експонента Пекерта
float In = 3.25;                // Номінальний струм (зазвичай 0.05C від ємності)

// Поточні вимірювання
float busVoltage = 0.0;        
float measuredCurrent = 0.0;   
float batteryCapacity = 0.0;   
float remainingCapacity = 0.0;  

// Поріг заряду для вимкнення реле (20% від початкової ємності)
float capacityThreshold = 0.2 * initialCapacity;

// Час оновлення вимірювань
unsigned long lastMeasurementTime = 0;
const unsigned long measurementInterval = 1000;

// Ініціалізація вебсервера
AsyncWebServer server(80);

// Функція розрахунку ступеня заряду з урахуванням Пекерта
float calculateStateOfCharge(float I) {
    float adjustedCapacity = initialCapacity * pow((In / I), PeukertExponent - 1);
    float stateOfCharge = ((adjustedCapacity - batteryCapacity) / adjustedCapacity) * 100;
    return max(stateOfCharge, 0.0); // Запобігання від'ємним значенням
}

void setup() {
    Serial.begin(115200);

    // Підключення до Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Підключення до Wi-Fi...");
    }
    Serial.println("Wi-Fi підключено!");
    Serial.print("IP-адреса: ");
    Serial.println(WiFi.localIP());

    // Ініціалізація I²C
    Wire.begin(25, 27);

    // Ініціалізація INA219
    if (!ina219.begin()) {
        Serial.println("Помилка: датчик INA219 не знайдено!");
        while (1);
    }
    Serial.println("INA219 успішно підключено!");

    // Ініціалізація OLED-дисплея
    u8g2.begin();

    // Налаштування реле
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);

    // Запуск вебсервера
    server.begin();
}

void loop() {
    unsigned long currentTime = millis();
    if (currentTime - lastMeasurementTime >= measurementInterval) {
        lastMeasurementTime = currentTime;

        // Зчитування параметрів
        busVoltage = ina219.getBusVoltage_V();
        int rawADC = analogRead(HW872A_PIN);
        float sensorVoltage = (rawADC / ADC_RESOLUTION) * MAX_VOLTAGE;
        measuredCurrent = sensorVoltage / SHUNT_RESISTOR;

        // Розрахунок залишкової ємності
        float currentDurationHours = measurementInterval / 3600000.0;
        batteryCapacity += measuredCurrent * currentDurationHours;
        remainingCapacity = initialCapacity - batteryCapacity;

        // Розрахунок ступеня заряду
        float stateOfCharge = calculateStateOfCharge(measuredCurrent);

        // Логіка вимкнення реле
        if (!manualControl) {
            if (stateOfCharge <= 20.0) {
                relayState = false;
                relayLocked = true;
                digitalWrite(RELAY_PIN, HIGH);
            }
        }

        // Виведення даних у серійний монітор
        Serial.print("Voltage: ");
        Serial.print(busVoltage, 2);
        Serial.print(" V, Current: ");
        Serial.print(measuredCurrent, 2);
        Serial.print(" A, SOC: ");
        Serial.print(stateOfCharge, 1);
        Serial.println(" %");

        // Оновлення OLED-дисплея
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_mf);
        u8g2.setCursor(0, 10);
        u8g2.print("Voltage: ");
        u8g2.print(busVoltage, 2);
        u8g2.print(" V");
        u8g2.setCursor(0, 20);
        u8g2.print("Current: ");
        u8g2.print(measuredCurrent, 2);
        u8g2.print(" A");
        u8g2.setCursor(0, 30);
        u8g2.print("SOC: ");
        u8g2.print(stateOfCharge, 1);
        u8g2.print(" %");
        u8g2.sendBuffer();
    }
}
