
/*
 * ESP32 FTMS Indoor Bike Sensor
 * Copyright (c) 2026 Rafaday: https://github.com/Rafaday
 * Licensed under the MIT License.
 *
 * See LICENSE file in the project root for full license information.
 */


#include <math.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// ===== PARÁMETROS DEL FILTRO POR DELTA =====
const float DELTA1 = 3.0f;
const float DELTA2 = 10.0f;
const float DELTA3 = 20.0f; 

const float ALPHA_MIN  = 0.12f;
const float ALPHA_MID  = 0.35f;
const float ALPHA_HIGH = 0.80f;
const float ALPHA_MAX  = 0.95f; 

const float ALPHA_SMOOTHING = 0.70f;

// ===== PARÁMETROS GENERALES =====
const unsigned long SEND_INTERVAL_MS    = 300;
const unsigned long MIN_INTERVAL_US     = 200000UL; // 200ms de bloqueo anti-rebote (máx 300 RPM) Se puede adaptar a límites más creibles para un ciclista. 
const unsigned long RPM_DECAY_START_MS  = 1500;
const unsigned long RPM_ZERO_TIMEOUT_MS = 3500;
const float         DECAY_FACTOR        = 0.92f;

// ===== HARDWARE =====
const int PIN_SENSOR = 5;
const int PIN_POT    = A0; // Pin analógco de tu placa ESP32 (Puede variar. Consulta documentación.)

// =======================================================
// VARIABLES DE FILTRADO Y HARDWARE
// =======================================================
volatile unsigned long lastPulseTime  = 0;
volatile unsigned long pulseInterval  = 0;
volatile bool          pulseDetected  = false;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

float    rpmFiltered         = 0.0f;
float    rpmSmoothed         = 0.0f;
uint16_t cadenceRPM          = 0;
unsigned long lastValidPulseTime = 0;  

float lastInstantRPM = 0.0f;
float lastDelta      = 0.0f; 
float lastAlphaUsed  = ALPHA_MID;
float smoothedAlpha  = ALPHA_MID;

// =======================================================
// TABLA DE POTENCIA (Estimación para resistencia de 8 niveles. Calibra para mejores resultados. K = Según aumento exponencial de las corrientes de Foucoult. C= Resistencias lineales como los rodamientos.)
// =======================================================
float K_values[8] = { 0.006f, 0.009f, 0.012f, 0.017f,
                      0.022f, 0.030f, 0.039f, 0.050f };
float C_values[8] = { 0.05f,  0.07f,  0.10f,  0.14f,
                      0.18f,  0.24f,  0.30f,  0.38f };

// =======================================================
// BLE UUIDs
// =======================================================
#define FTMS_SERVICE_UUID            "00001826-0000-1000-8000-00805f9b34fb"
#define INDOOR_BIKE_DATA_UUID        "00002ad2-0000-1000-8000-00805f9b34fb"
#define FITNESS_MACHINE_FEATURE_UUID "00002acc-0000-1000-8000-00805f9b34fb"

BLECharacteristic* indoorBikeDataChar = nullptr;
bool deviceConnected = false;

// =======================================================
// INTERRUPCIÓN (Anti-rebote Sólido Clásico)
// =======================================================
void IRAM_ATTR sensorISR() {
    unsigned long now = micros();
    // Bloqueo duro de 200ms para evitar falsos rebotes del sensor magnético
    if (now - lastPulseTime > MIN_INTERVAL_US) {
        portENTER_CRITICAL_ISR(&mux);
        pulseInterval = now - lastPulseTime;
        lastPulseTime = now;
        pulseDetected = true;
        portEXIT_CRITICAL_ISR(&mux);
    }
}

// =======================================================
// FILTROS MATEMÁTICOS
// =======================================================
float calculateAlphaByDelta(float currentInstantRPM, float previousInstantRPM) {
    float delta = fabs(currentInstantRPM - previousInstantRPM);
    float alpha;
    if (delta < DELTA1) {
        alpha = ALPHA_MIN;
    } else if (delta < DELTA2) {
        float t = (delta - DELTA1) / (DELTA2 - DELTA1);
        alpha = ALPHA_MIN + t * (ALPHA_MID - ALPHA_MIN);
    } else if (delta < DELTA3) {
        float t = (delta - DELTA2) / (DELTA3 - DELTA2);
        alpha = ALPHA_MID + t * (ALPHA_HIGH - ALPHA_MID);
    } else {
        alpha = ALPHA_MAX;
    }
    return constrain(alpha, 0.05f, 0.95f);
}

float getSmoothedAlpha(float desiredAlpha) {
    if (desiredAlpha >= ALPHA_MAX) {
        smoothedAlpha = desiredAlpha;
    } else {
        smoothedAlpha = (ALPHA_SMOOTHING * desiredAlpha) + ((1.0f - ALPHA_SMOOTHING) * smoothedAlpha);
    }
    return smoothedAlpha;
}

// =======================================================
// ACTUALIZACIÓN DE RPM
// =======================================================
void updateRPM(bool detected, unsigned long interval, unsigned long nowMillis) {
    static float lastRPMForDelta = 0.0f;
    static unsigned long lastDecayApplied = 0;

    if (detected && interval > 0) {
        float instantRpm = 60000000.0f / (float)interval;

        if (instantRpm > 5.0f && instantRpm < 180.0f) {
            float desiredAlpha = calculateAlphaByDelta(instantRpm, lastRPMForDelta);
            lastDelta = fabs(instantRpm - lastRPMForDelta); 
            float finalAlpha = getSmoothedAlpha(desiredAlpha);

            lastAlphaUsed = finalAlpha;
            lastRPMForDelta = instantRpm;
            lastInstantRPM = instantRpm;

            if (rpmSmoothed == 0.0f) {
                rpmSmoothed = instantRpm;
                smoothedAlpha = ALPHA_MID;
            } else {
                rpmSmoothed = (finalAlpha * instantRpm) + ((1.0f - finalAlpha) * rpmSmoothed);
            }

            rpmFiltered = rpmSmoothed;
            lastValidPulseTime = nowMillis;  
            lastDecayApplied = nowMillis;
        }
    }

    // DECAY (Frenado progresivo o detención)
    unsigned long timeSinceLastPulse = nowMillis - lastValidPulseTime;

    if (lastValidPulseTime > 0 && timeSinceLastPulse > RPM_ZERO_TIMEOUT_MS) {
        rpmFiltered = 0.0f;
        rpmSmoothed = 0.0f;
    }
    else if (lastValidPulseTime > 0 && timeSinceLastPulse > RPM_DECAY_START_MS &&
             rpmFiltered > 0.0f && (nowMillis - lastDecayApplied) >= SEND_INTERVAL_MS) {
        rpmFiltered *= DECAY_FACTOR;
        rpmSmoothed  = rpmFiltered;
        lastDecayApplied = nowMillis;
        if (rpmFiltered < 1.0f) {
            rpmFiltered = 0.0f;
            rpmSmoothed = 0.0f;
        }
    }

    cadenceRPM = (uint16_t)rpmFiltered;
}

// =======================================================
// CALLBACKS BLE
// =======================================================
class MyCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer*) override {
        deviceConnected = true;
        Serial.println("✅ BLE: Conectado a la app");
    }
    void onDisconnect(BLEServer*) override {
        deviceConnected = false;
        Serial.println("❌ BLE: Desconectado");
        delay(100);
        BLEDevice::startAdvertising();
    }
};

// =======================================================
// POTENCIÓMETRO Y POTENCIA
// =======================================================
int getLevelFromPot() {
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += analogRead(PIN_POT);
        delayMicroseconds(200);
    }
    int raw = (int)(sum >> 3);
    float normalized = (float)(raw - 120) / (3980.0f - 120.0f);
    int level = (int)(normalized * 8.0f) + 1;
    return constrain(level, 1, 8);
}

int16_t estimatePower(float rpm, int level) {
    if (rpm < 10.0f) return 0;
    level = constrain(level, 1, 8);
    float p = K_values[level - 1] * rpm * rpm + C_values[level - 1] * rpm;
    return (int16_t)constrain(p, 0.0f, 3000.0f);
}

// =======================================================
// ENVÍO BLE FTMS 
// =======================================================
void sendIndoorBikeData(uint16_t cadence, int16_t power) {
    // 0x0044: Indica a la app que enviamos Velocidad, Cadencia y Potencia
    const uint16_t FLAGS = 0x0044; 

    uint16_t speedValue = cadence * 30; // Simulamos la velocidad para que la app no tire error
    uint16_t cadenceValue = cadence * 2;  

    uint8_t payload[8];
    payload[0] = FLAGS & 0xFF;
    payload[1] = (FLAGS >> 8) & 0xFF;
    payload[2] = speedValue & 0xFF;
    payload[3] = (speedValue >> 8) & 0xFF;
    payload[4] = cadenceValue & 0xFF;
    payload[5] = (cadenceValue >> 8) & 0xFF;
    payload[6] = power & 0xFF;
    payload[7] = (power >> 8) & 0xFF;

    indoorBikeDataChar->setValue(payload, sizeof(payload));
    indoorBikeDataChar->notify();
}

// =======================================================
// SETUP
// =======================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    // Configuración del ADC para leer hasta 3.3V
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db); 

    Serial.println("\n=== ESP32 FTMS Bike v5.9 - READY ===");

    pinMode(PIN_SENSOR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_SENSOR), sensorISR, FALLING);

    // --- INICIALIZACIÓN BLE ---
    BLEDevice::init("ESP32 FTMS Bike");
    BLEServer* server = BLEDevice::createServer();
    server->setCallbacks(new MyCallbacks());

    BLEService* ftmsService = server->createService(FTMS_SERVICE_UUID);

    indoorBikeDataChar = ftmsService->createCharacteristic(
        INDOOR_BIKE_DATA_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    indoorBikeDataChar->addDescriptor(new BLE2902());

    BLECharacteristic* featureChar = ftmsService->createCharacteristic(
        FITNESS_MACHINE_FEATURE_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    
    // Feature flags: Cadencia, Potencia
    uint8_t featureData[8] = {0x02, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    featureChar->setValue(featureData, sizeof(featureData));

    ftmsService->start();

    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(FTMS_SERVICE_UUID);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();

    Serial.println("🚀 BLE inicializado y publicando...");
}

// =======================================================
// LOOP PRINCIPAL
// =======================================================
void loop() {
    static unsigned long lastSend    = 0;
    static unsigned long lastPotRead = 0;
    static int           cachedLevel = 1;

    portENTER_CRITICAL(&mux);
    unsigned long interval = pulseInterval;
    bool detected = pulseDetected;
    pulseDetected = false;
    portEXIT_CRITICAL(&mux);

    unsigned long nowMillis = millis();

    updateRPM(detected, interval, nowMillis);

    // Leer el potenciómetro cada 1 segundo
    if (nowMillis - lastPotRead > 1000) {
        cachedLevel = getLevelFromPot();
        lastPotRead = nowMillis;
    }

    // Envío periódico cada 300ms (Serial + BLE)
    if (nowMillis - lastSend > SEND_INTERVAL_MS) {
        lastSend = nowMillis;

        int16_t power = estimatePower(rpmFiltered, cachedLevel);

        // Print simplificado para que siempre lo veas en pantalla
        Serial.printf("RPM: %3d | Nivel: %d | Potencia: %4dW | BLE: %s\n",
                      cadenceRPM, cachedLevel, power,
                      deviceConnected ? "ON" : "OFF");

        if (deviceConnected) {
            sendIndoorBikeData(cadenceRPM, power);
        }
    }
}
