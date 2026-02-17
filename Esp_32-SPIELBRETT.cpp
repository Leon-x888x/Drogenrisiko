#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// === KONFIGURATION ===
#define DEVICE_NAME "ESP32_Spielbrett"
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_RESPONSE_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// === BT STATUS LED KONFIGURATION ===
#define BT_STATUS_LED_PIN 22  // ← Hier kannst du den Pin ändern
#define LED_BLINK_INTERVAL 500  // Blinkgeschwindigkeit in ms (Pairing-Modus)

// Sicherheitsliste
const int ALLOWED_PINS[] = {2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 23, 25, 26, 27, 32, 33};
const int NUM_ALLOWED_PINS = sizeof(ALLOWED_PINS) / sizeof(ALLOWED_PINS[0]);

// Globale Variablen
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLECharacteristic *pResponseCharacteristic = NULL;
BLEAdvertising *pAdvertising = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
unsigned long connectionStartTime = 0;
String lastClientAddress = "";
bool allowReconnect = false;

// LED Status Variablen
unsigned long lastBlinkTime = 0;
bool ledBlinkState = false;

// LED Status Enum
enum LEDStatus {
  LED_OFF,          // Aus / System aus
  LED_PAIRING,      // Blinken / Wartet auf Verbindung
  LED_CONNECTED     // Dauerhaft an / Verbunden
};

LEDStatus currentLEDStatus = LED_OFF;

// === LED FUNKTIONEN ===
void setupStatusLED() {
  pinMode(BT_STATUS_LED_PIN, OUTPUT);
  digitalWrite(BT_STATUS_LED_PIN, LOW);
  Serial.print("→ Status LED initialisiert auf Pin ");
  Serial.println(BT_STATUS_LED_PIN);
}

void setLEDStatus(LEDStatus status) {
  currentLEDStatus = status;
  
  switch(status) {
    case LED_OFF:
      digitalWrite(BT_STATUS_LED_PIN, LOW);
      Serial.println("💡 Status LED: AUS");
      break;
      
    case LED_PAIRING:
      // Blinken wird in updateStatusLED() gehandhabt
      Serial.println("💡 Status LED: BLINKT (Pairing-Modus)");
      break;
      
    case LED_CONNECTED:
      digitalWrite(BT_STATUS_LED_PIN, HIGH);
      Serial.println("💡 Status LED: AN (Verbunden)");
      break;
  }
}

void updateStatusLED() {
  if (currentLEDStatus == LED_PAIRING) {
    unsigned long currentMillis = millis();
    
    if (currentMillis - lastBlinkTime >= LED_BLINK_INTERVAL) {
      lastBlinkTime = currentMillis;
      ledBlinkState = !ledBlinkState;
      digitalWrite(BT_STATUS_LED_PIN, ledBlinkState ? HIGH : LOW);
    }
  }
}

// === MUX SELBSTTEST FUNKTION ===
void performMuxSelfTest() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║     MUX SELBSTTEST STARTET             ║");
  Serial.println("╚════════════════════════════════════════╝");
  
  // Test alle 16 MUX-Positionen (0-15)
  for (int i = 0; i < 16; i++) {
    // Bits extrahieren
    bool p16 = (i & 0x01); // Bit 0 (LSB)
    bool p17 = (i & 0x02); // Bit 1
    bool p18 = (i & 0x04); // Bit 2
    bool p19 = (i & 0x08); // Bit 3 (MSB)
    
    // MUX-Pins setzen
    digitalWrite(16, p16 ? HIGH : LOW);
    digitalWrite(17, p17 ? HIGH : LOW);
    digitalWrite(18, p18 ? HIGH : LOW);
    digitalWrite(19, p19 ? HIGH : LOW);
    
    // Log ausgeben
    Serial.printf("→ MUX Position %d (Binär: %d%d%d%d) | GPIO16=%s, GPIO17=%s, GPIO18=%s, GPIO19=%s\n",
                  i, p19, p18, p17, p16,
                  p16 ? "HIGH" : "LOW",
                  p17 ? "HIGH" : "LOW",
                  p18 ? "HIGH" : "LOW",
                  p19 ? "HIGH" : "LOW");
    
    // 500ms warten (0,5 Sekunden)
    delay(500);
  }
  
  // Am Ende: Alles ausschalten (Position 15 = 1111)
  digitalWrite(16, HIGH);
  digitalWrite(17, HIGH);
  digitalWrite(18, HIGH);
  digitalWrite(19, HIGH);
  
  Serial.println("\n✓ Selbsttest abgeschlossen - Alle LEDs ausgeschaltet");
  Serial.println("─────────────────────────────────────────\n");
}

// === HILFSFUNKTIONEN ===
void printTimestamp() {
  unsigned long seconds = millis() / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  Serial.printf("[%02lu:%02lu:%02lu] ", hours % 24, minutes % 60, seconds % 60);
}

void printConnectionDuration() {
  if (connectionStartTime > 0) {
    unsigned long duration = (millis() - connectionStartTime) / 1000;
    Serial.print("Verbindungsdauer: ");
    Serial.print(duration);
    Serial.println(" Sekunden");
  }
}

void sendResponse(const char* message) {
  if (deviceConnected && pResponseCharacteristic != NULL) {
    pResponseCharacteristic->setValue(message);
    pResponseCharacteristic->notify();
    printTimestamp();
    Serial.print("📤 Bestätigung gesendet: ");
    Serial.println(message);
  }
}

bool isPinAllowed(uint8_t pin) {
  // Status LED Pin darf nicht gesteuert werden
  if (pin == BT_STATUS_LED_PIN) {
    return false;
  }
  
  for (int i = 0; i < NUM_ALLOWED_PINS; i++) {
    if (ALLOWED_PINS[i] == pin) {
      return true;
    }
  }
  return false;
}

// === SERVER CALLBACKS ===
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      connectionStartTime = millis();
      
      // LED auf VERBUNDEN setzen
      setLEDStatus(LED_CONNECTED);
      
      // Client Adresse speichern
      BLEDevice::getAddress();
      
      Serial.println("\n╔════════════════════════════════════════╗");
      Serial.println("║     VERBINDUNG HERGESTELLT ✓          ║");
      Serial.println("╚════════════════════════════════════════╝");
      printTimestamp();
      Serial.println("Client hat sich verbunden");
      
      // Erlaube Reconnect mit diesem Client
      allowReconnect = true;
      
      Serial.println("Status: BEREIT für Befehle");
      Serial.println("Auto-Reconnect: AKTIVIERT");
      Serial.println("─────────────────────────────────────────\n");
      
      // Sende Willkommensnachricht
      delay(100);
      sendResponse("ESP32 bereit!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      
      // LED auf PAIRING setzen (blinken)
      setLEDStatus(LED_PAIRING);
      
      // === ALLE LEDS AUSSCHALTEN (MUX auf 1111 = AUS) ===
      digitalWrite(16, HIGH);
      digitalWrite(17, HIGH);
      digitalWrite(18, HIGH);
      digitalWrite(19, HIGH);
      
      Serial.println("\n╔════════════════════════════════════════╗");
      Serial.println("║     VERBINDUNG GETRENNT ✗             ║");
      Serial.println("╚════════════════════════════════════════╝");
      printTimestamp();
      Serial.println("Client hat Verbindung getrennt");
      printConnectionDuration();
      
      printTimestamp();
      Serial.println("→ Alle LEDs ausgeschaltet (MUX = 1111)");
      
      if (allowReconnect) {
        Serial.println("Auto-Reconnect: WARTET auf bekannten Client");
      }
      
      Serial.println("─────────────────────────────────────────\n");
    }
};

// === CHARACTERISTIC CALLBACKS ===
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      
      printTimestamp();
      
      // Datenpaket validieren (jetzt 3 Bytes: Pin, State, OnOff)
      if (value.length() != 3) {
        Serial.print("📥 Befehl empfangen (");
        Serial.print(value.length());
        Serial.println(" Bytes)");
        Serial.println("⚠ FEHLER: Ungültige Datenlänge. Erwartet 3 Bytes (Pin, State, OnOff).");
        sendResponse("ERROR: Ungueltige Datenlaenge");
        return;
      }

      uint8_t pin = (uint8_t)value[0];
      uint8_t state = (uint8_t)value[1];
      uint8_t onOff = (uint8_t)value[2];

      Serial.print("📥 Befehl empfangen: Pin=");
      Serial.print(pin);
      Serial.print(", State=");
      Serial.print(state);
      Serial.print(", OnOff=");
      Serial.println(onOff);

      // === BATCH BEFEHL FÜR PIN 255 (MUX STEUERUNG) ===
      if (pin == 255) {
         Serial.println("📥 BATCH-BEFEHL (MUX Steuerung)");
         
         // OnOff prüfen
         if (onOff == 0) {
            // Alles ausschalten (15 = 1111 = AUS)
            Serial.println("→ Schalte alle LEDs AUS (Wert 15)");
            state = 15;
         }
         // Wenn onOff == 1, verwende den gegebenen State-Wert
         
         // Sicherstellen, dass die Pins Ausgänge sind
         pinMode(16, OUTPUT);
         pinMode(17, OUTPUT);
         pinMode(18, OUTPUT);
         pinMode(19, OUTPUT);
         
         // Bits extrahieren (Standard Binär Mapping)
         bool p16 = (state & 0x01); // Bit 0 (LSB)
         bool p17 = (state & 0x02); // Bit 1
         bool p18 = (state & 0x04); // Bit 2
         bool p19 = (state & 0x08); // Bit 3 (MSB)

         digitalWrite(16, p16 ? HIGH : LOW);
         digitalWrite(17, p17 ? HIGH : LOW);
         digitalWrite(18, p18 ? HIGH : LOW);
         digitalWrite(19, p19 ? HIGH : LOW);
         
         // === DETAILLIERTER LOG ===
         Serial.println("--------------------------------");
         Serial.printf("OnOff: %s, Wert: %d (Binär: %d%d%d%d)\n", 
                      onOff ? "EIN" : "AUS", state, p19, p18, p17, p16);
         Serial.printf("GPIO 16 (A): %s\n", p16 ? "HIGH" : "LOW");
         Serial.printf("GPIO 17 (B): %s\n", p17 ? "HIGH" : "LOW");
         Serial.printf("GPIO 18 (C): %s\n", p18 ? "HIGH" : "LOW");
         Serial.printf("GPIO 19 (D): %s\n", p19 ? "HIGH" : "LOW");
         Serial.println("--------------------------------");
         
         sendResponse("OK: Mux Update");
         return; 
      }
      // === ENDE BATCH ===

      // Pin validieren
      if (!isPinAllowed(pin)) {
        Serial.print("⚠ SICHERHEIT: Pin ");
        Serial.print(pin);
        Serial.println(" ist NICHT erlaubt!");
        sendResponse("ERROR: Pin nicht erlaubt");
        return;
      }

      // Status validieren
      if (state != 0 && state != 1) {
        Serial.print("⚠ FEHLER: Ungültiger Status (");
        Serial.print(state);
        Serial.println(", erwartet 0 oder 1)");
        sendResponse("ERROR: Ungueltiger Status");
        return;
      }

      // OnOff validieren
      if (onOff != 0 && onOff != 1) {
        Serial.print("⚠ FEHLER: Ungültiger OnOff Wert (");
        Serial.print(onOff);
        Serial.println(", erwartet 0 oder 1)");
        sendResponse("ERROR: Ungueltiger OnOff");
        return;
      }

      // === EINZEL-BEFEHL AUSFÜHREN ===
      pinMode(pin, OUTPUT);
      
      // OnOff prüfen: 0 = Ausschalten, 1 = State anwenden
      if (onOff == 0) {
        digitalWrite(pin, LOW);
        Serial.print("✓ BEFEHL AUSGEFÜHRT → GPIO");
        Serial.print(pin);
        Serial.println(" = LOW (AUS)");
        
        char response[50];
        sprintf(response, "OK: Pin %d = AUS", pin);
        sendResponse(response);
      } else {
        digitalWrite(pin, state == 1 ? HIGH : LOW);
        Serial.print("✓ BEFEHL AUSGEFÜHRT → GPIO");
        Serial.print(pin);
        Serial.print(" = ");
        Serial.print(state == 1 ? "HIGH" : "LOW");
        Serial.print(" (");
        Serial.print(state == 1 ? "EIN" : "AUS");
        Serial.println(")");
        
        char response[50];
        sprintf(response, "OK: Pin %d = %s", pin, state == 1 ? "HIGH" : "LOW");
        sendResponse(response);
      }
    }
};

// === ADVERTISING ===
void startAdvertising() {
  if (pAdvertising != NULL) {
    printTimestamp();
    Serial.println("📡 Starte Advertising...");
    
    // LED auf PAIRING setzen (blinken)
    setLEDStatus(LED_PAIRING);
    
    if (allowReconnect) {
      Serial.println("→ Wartet auf bekannten Client");
    } else {
      Serial.println("→ Bereit für neue Verbindungen");
    }
    
    pAdvertising->start();
    Serial.println("─────────────────────────────────────────\n");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║                                        ║");
  Serial.println("║      ESP32 BLE PIN CONTROLLER          ║");
  Serial.println("║       Version 3.6 (Mit Selbsttest)     ║");
  Serial.println("║  Mit Status-LED & Auto-Reconnect       ║");
  Serial.println("║                                        ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  printTimestamp();
  Serial.println("System initialisiert");
  Serial.println("─────────────────────────────────────────");
  
  // Status LED initialisieren
  setupStatusLED();

  // MUX Pins initialisieren
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  
  // STARTZUSTAND: Alles HIGH (1111) -> Damit ist Adresse 15 gewählt (AUS)
  digitalWrite(16, HIGH); 
  digitalWrite(17, HIGH);
  digitalWrite(18, HIGH);
  digitalWrite(19, HIGH);
  
  // === MUX SELBSTTEST DURCHFÜHREN ===
  performMuxSelfTest();
  
  // BLE initialisieren
  BLEDevice::init(DEVICE_NAME);
  printTimestamp();
  Serial.print("→ Gerätename: ");
  Serial.println(DEVICE_NAME);

  // Server erstellen
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  printTimestamp();
  Serial.println("→ BLE Server erstellt");

  // Service erstellen
  BLEService *pService = pServer->createService(SERVICE_UUID);
  printTimestamp();
  Serial.print("→ Service UUID: ");
  Serial.println(SERVICE_UUID);

  // Command Characteristic (WRITE)
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pCharacteristic->setCallbacks(new MyCallbacks());
  
  printTimestamp();
  Serial.print("→ Command Characteristic: ");
  Serial.println(CHARACTERISTIC_UUID);

  // Response Characteristic (NOTIFY)
  pResponseCharacteristic = pService->createCharacteristic(
                              CHARACTERISTIC_RESPONSE_UUID,
                              BLECharacteristic::PROPERTY_READ |
                              BLECharacteristic::PROPERTY_NOTIFY
                            );
  pResponseCharacteristic->addDescriptor(new BLE2902());
  
  printTimestamp();
  Serial.print("→ Response Characteristic: ");
  Serial.println(CHARACTERISTIC_RESPONSE_UUID);

  // Service starten
  pService->start();
  printTimestamp();
  Serial.println("→ Service gestartet");

  // Advertising konfigurieren
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  
  printTimestamp();
  Serial.println("→ Advertising konfiguriert");
  
  startAdvertising();
  
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║   SYSTEM BEREIT - WARTE AUF CLIENT   ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  // Erlaubte Pins
  Serial.println("📌 Erlaubte GPIO Pins:");
  Serial.print("   ");
  for (int i = 0; i < NUM_ALLOWED_PINS; i++) {
    Serial.print(ALLOWED_PINS[i]);
    if (i < NUM_ALLOWED_PINS - 1) Serial.print(", ");
    if ((i + 1) % 10 == 0 && i < NUM_ALLOWED_PINS - 1) {
      Serial.print("\n   ");
    }
  }
  Serial.println();
  Serial.print("\n⚠ Pin ");
  Serial.print(BT_STATUS_LED_PIN);
  Serial.println(" ist für Status-LED reserviert");
  Serial.println("\n📋 PROTOKOLL: 3 Bytes [Pin, State, OnOff]");
  Serial.println("   Pin 255 = Batch-Befehl für MUX");
  Serial.println("   OnOff: 0=AUS, 1=State anwenden");
  Serial.println("─────────────────────────────────────────\n");
}

void loop() {
  // Status LED aktualisieren (für Blinken)
  updateStatusLED();
  
  // Verbindung getrennt
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    
    printTimestamp();
    Serial.println("🔄 AUTO-RECONNECT aktiviert");
    
    startAdvertising();
    
    oldDeviceConnected = deviceConnected;
    connectionStartTime = 0;
  }
  
  // Neu verbunden
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  delay(100);
}
