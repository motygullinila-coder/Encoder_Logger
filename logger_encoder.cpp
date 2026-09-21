#include <LittleFS.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ModbusRTU.h>

// -> test
  // Пины энкодера и референтного сигнала
  #define ENCODER_PIN_A 32
  #define ENCODER_PIN_B 33
  #define ENCODER_PIN_R 25

  // Пины RX/TX для Modbus
  #define MODBUS_PIN_RX 16
  #define MODBUS_PIN_TX 17
  #define MODBUS_PIN_OUT_IN 4

  #define SLAVED_CONTROLLER 1

  ModbusRTU modbus;

  const float encoderStep = 0.005; // 5 мкм = 0.005 мм

  volatile long encoderPosition = 0;
  volatile int lastEncode = 0;
  volatile bool referenceTriggered = false;

  long lastPositionMMx10 = -99999;
  long lastEncoderPosition = -99999;

  void IRAM_ATTR handleEncoder() {
    bool A_signal = digitalRead(ENCODER_PIN_A);
    bool B_signal = digitalRead(ENCODER_PIN_B);

    int encode = (A_signal << 1) | B_signal;
    int combined = (lastEncode << 2) | encode;

    switch (combined) {
      case 0b0001: case 0b0111: case 0b1110: case 0b1000:
        encoderPosition++;
        break;
      
      case 0b0010: case 0b0100: case 0b1101: case 0b1011:
        encoderPosition--;
        break;
      
      default:
        break;
    }

    lastEncode = encode;
  }

  void IRAM_ATTR handleReference() {
    referenceTriggered = true;
  }

  // -> server
    IPAddress ip(192, 169, 2, 1);
    IPAddress geteway(192, 169, 2, 1);
    IPAddress subnet(255, 255, 255, 0);

    AsyncWebServer server(80);

    const char* ssid = "Torex_Logger_Encoder";
    const char* password = "123098456765";

    void initial_wifi() {
      WiFi.softAP(ssid, password);
      WiFi.softAPConfig(ip, geteway, subnet);
    }

    void create_route_server() {
      server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request -> send(LittleFS, "/index.html", "text/html");
      });

      server.on("/encoder", HTTP_GET, [](AsyncWebServerRequest* request) {
        long ticks = encoderPosition;
        float position = ticks * encoderStep;
        bool references = referenceTriggered;

        String json = "{";
        json += "\"position\":";
        json += String(position, 3);
        json += ",\"ticks\":";
        json += String(ticks);
        json += ",\"references\":";
        json += references ? "true" : "false";
        json += "}";

        request -> send(200, "application/json", json);
      });
    }

    void start_server() {
      create_route_server();
      server.begin();
    }
  // -> server

  void setup() {
    Serial.begin(115200);

    if (!LittleFS.begin(true)) {
      return;
    }

    initial_wifi();
    start_server();

    pinMode(ENCODER_PIN_A, INPUT);
    pinMode(ENCODER_PIN_B, INPUT);
    pinMode(ENCODER_PIN_R, INPUT);

    lastEncode = (digitalRead(ENCODER_PIN_A) << 1 | digitalRead(ENCODER_PIN_B));

    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handleEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), handleEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_R), handleReference, RISING);

    Serial1.begin(9600, SERIAL_8N1, MODBUS_PIN_RX, MODBUS_PIN_TX);
    
    modbus.begin(&Serial1, MODBUS_PIN_OUT_IN);
    modbus.slave(SLAVED_CONTROLLER);

    modbus.addIreg(0, 0); // PositionMM
    modbus.addIreg(1, 0); // PositionTicks LOW
    modbus.addIreg(2, 0); // PositionTicks HIGH
    modbus.addHreg(3, 0); // Резерв
    modbus.addIreg(4, 0); // Резерв
    modbus.addIreg(5, 0); // Резерв
    modbus.addIreg(6, 0); // Резерв
    modbus.addHreg(10, 0); // PLC DATA
  }

  void loop() {
    modbus.task();

    long currentTicks = encoderPosition;

    static uint16_t lastAcceptedPlcValue = 65535;  // Последнее принятое значение от ПЛК
    static bool newPlcCommandHandled = true;       // Было ли оно уже обработано

    uint16_t plcData = modbus.Hreg(10);

    if (plcData != 65535 && (plcData != lastAcceptedPlcValue || newPlcCommandHandled)) {
      encoderPosition = plcData * 20;
      Serial.print("Получена команда от ПЛК: ");
      Serial.println(plcData);

      lastAcceptedPlcValue = plcData;
      newPlcCommandHandled = false;
    }

    if (plcData == 65535 && !newPlcCommandHandled) {
      newPlcCommandHandled = true;
    }

    long positionEncoderNew = (long)(encoderPosition * encoderStep * 10.0f);

    uint32_t positionTicksU = (uint32_t)(encoderPosition);
    modbus.Ireg(1, (uint16_t)(positionTicksU & 0xFFFF));         // low word
    modbus.Ireg(2, (uint16_t)((positionTicksU >> 16) & 0xFFFF)); // high word
    modbus.Ireg(0, (int16_t)positionEncoderNew);                 // мм * 10

    if (positionEncoderNew != lastPositionMMx10 || currentTicks != lastEncoderPosition) {
      Serial.print("Position: ");
      Serial.print(positionEncoderNew, 1);
      Serial.print(" мм | Ticks: ");
      Serial.println(currentTicks);
      lastPositionMMx10 = positionEncoderNew;
      lastEncoderPosition = currentTicks;
    }

    if (referenceTriggered) {
      Serial.print("Референс активирован. Текущая позиция: ");
      Serial.print(positionEncoderNew / 10.0, 1);
      Serial.println(" мм");
      referenceTriggered = false;
    }

    delay(1000);
  }
// -> test
