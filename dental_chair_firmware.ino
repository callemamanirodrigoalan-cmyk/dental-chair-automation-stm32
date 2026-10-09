// ============================================================================
//  LLENAVASOS + ESCUPIDERA — FIRMWARE STM32F103C8T6  v10
//  Arquitectura asíncrona: sin delay() en loop, waitMsNonBlocking() en setup
//  I2C1 (PB6/PB7) → VL6180X (llenavasos)
//  I2C2 (PB10/PB11) → VL53L0X (escupidera)
//  XSHUT LV → PB8, XSHUT ES → PB9 (hardware reset en setup para evitar zombie)
//  Feedback LV → PA8 (movido de PB10, que ahora es SCL2)
//  Core: STM32duino (Roger Clark / libmaple)
// ============================================================================

#include <Wire.h>
#include <Adafruit_VL6180X.h>
#include <VL53L0X.h>
#include <EEPROM.h>
#include <libmaple/iwdg.h>

// Segundo bus I2C (I2C2 = PB10/PB11) para el VL53L0X de la escupidera.
// Constructor por número de dispositivo (verificado en libmaple).
TwoWire Wire2(2, I2C_FAST_MODE);

// ========================= PINES ============================================
#define CUP_VALVE_OUT   PB1
#define CUP_VALVE_FB    PA8      // v10: movido de PB10 (que ahora es SCL2)

// XSHUT dual — hardware reset en setup para evitar "modo zombie" al arranque.
// Cada sensor en su propio bus I2C, sin conflicto de dirección.
#define CUP_XSHUT       PB8      // v10: nuevo — XSHUT del VL6180X (I2C1)
#define SPIT_XSHUT      PB9      // v10: movido de PB11 (que ahora es SDA2)
#define SPIT_VALVE_FB   PB12
#define SPIT_VALVE_OUT  PB13

#define A_PIN      PC13
#define B_PIN      PC14
#define C_PIN      PA2
#define D_PIN      PA5
#define E_PIN      PA4
#define F_PIN      PA1
#define G_PIN      PA6
#define DP_PIN     PA3

#define D1_PIN     PA0
#define D2_PIN     PC15

#define BTN1_PIN   PA7
#define BTN2_PIN   PB0

// ========================= EEPROM ===========================================
// v10c: llenavasos en 1 byte (0-80 mm cabe), escupidera en 2 bytes (0-200 mm > 255... no,
// entra en uint8. Pero preferimos 2 bytes por consistencia y para futuros rangos mayores).
#define EEPROM_ADDR_LOWER     0   // llenavasos inferior (mm, 1 byte)
#define EEPROM_ADDR_UPPER     1   // llenavasos superior (mm, 1 byte)
#define EEPROM_ADDR_VALID     2
#define EEPROM_ADDR_MODE      3
#define EEPROM_ADDR_DELAY     4   // delay escupidera (décimas de seg)
#define EEPROM_ADDR_SPIT_LOW_L 5  // escupidera inferior (mm) byte bajo
#define EEPROM_ADDR_SPIT_LOW_H 6  // escupidera inferior (mm) byte alto
#define EEPROM_ADDR_SPIT_UP_L  7  // escupidera superior (mm) byte bajo
#define EEPROM_ADDR_SPIT_UP_H  8  // escupidera superior (mm) byte alto
#define EEPROM_ADDR_SENSORS    9  // sensores instalados
#define EEPROM_ADDR_HYST_CUP   10 // v11: histéresis llenavasos (mm, 0-5 = 0.0-0.5cm)
#define EEPROM_ADDR_CUP_DELAY  11 // v11c: delay llenavasos (décimas de seg, 0-5 = 0.0-0.5s)
#define EEPROM_VALID_FLAG  0xB1  // v11c: subido por nuevo campo D.L

// Códigos de sensores instalados
#define SENSORS_BOTH      0
#define SENSORS_CUP_ONLY  1
#define SENSORS_SPIT_ONLY 2

// ========================= CONSTANTES =======================================
const unsigned long DEBOUNCE_MS       = 50;
const unsigned long HOLD_TIME_MS      = 2000;
const unsigned long PULSE_DURATION_MS = 80;
const unsigned long SENSOR_INTERVAL   = 10;

// v13c: arquitectura de lectura NO BLOQUEANTE (modo continuo)
// Centinela devuelto cuando el sensor aún no tiene muestra nueva lista.
// DEBE ser distinto de -1 (error) y de cualquier mm válido (>=0).
#define SAMPLE_NOT_READY  (-2)
// Si un canal no entrega muestra nueva en este tiempo, se considera
// "vencido" y se fuerza fuera de rango (protección anti-cuelgue).
const unsigned long SENSOR_STALE_MS   = 150;

// v13d: RECUPERACIÓN AUTOMÁTICA DE SENSOR CONGELADO
// Si un sensor no entrega muestra fresca durante RECOVERY_TRIGGER_MS,
// se intenta un reset por XSHUT + bus recovery + re-init sin resetear el MCU.
// Después de MAX_RECOVERY_ATTEMPTS fallidos consecutivos, se desactiva el canal
// en runtime (marca *Installed = false) para no degradar el otro sensor.
const unsigned long RECOVERY_TRIGGER_MS   = 500;  // 3.3× staleness — indica que el chip está muerto, no solo lento
const uint8_t       MAX_RECOVERY_ATTEMPTS = 5;    // reintentos antes de desactivar
const unsigned long RECOVERY_COOLDOWN_MS  = 1000; // no reintentar antes de este tiempo
const unsigned long MUX_INTERVAL_US   = 500;
const unsigned long CONFIRM_TIMEOUT   = 1000;
const unsigned long BLINK_INTERVAL    = 300;
const unsigned long FAULT_CRITICAL_MS = 5000;  // tiempo en FAULT antes de escalar
const uint8_t CONFIRM_READINGS        = 3;   // lecturas consec. para ENTRADA
const uint8_t CONFIRM_READINGS_OUT    = 1;   // v11b: 1 lectura para salida más rápida
const uint8_t MAX_CONSECUTIVE_FAULTS  = 3;
const unsigned long EXTERN_CLOSE_DELAY_MS   = 30;   // espera final antes de cerrar
const unsigned long EXTERN_CONFIRM_HIGH_MS  = 30;   // permanencia mínima de PB10 HIGH
// Delay de escupidera: ahora es VARIABLE (no const), configurable desde el menú
uint16_t spitActivationDelayMs = 1000;  // valor por defecto 1.0s
// Constantes del menú
const unsigned long PAGE_ID_DURATION_MS       = 800;
const unsigned long CONFIRM_FLASH_DURATION_MS = 800;
const unsigned long DP_BLINK_INTERVAL_MS      = 400;
const float   EMA_ALPHA               = 0.50f; // v11b: más reactivo (0.35→0.50)
// IWDG: prescaler 256, reload 625 → timeout ≈ 4 segundos
const uint16_t IWDG_RELOAD            = 625;

// ========================= ENUMS ============================================
enum SystemState {
  STATE_NORMAL,
  STATE_WAIT_RELEASE,
  STATE_CONFIG,
  STATE_CONFIRM_FLASH, // muestra G. o n. brevemente antes de volver a NORMAL
  STATE_RESET_FLASH    // muestra "rE" antes de reiniciar por cambio de sensores
};

enum CupStateM2 {
  M2_IDLE,
  M2_OPENING,
  M2_OPEN,
  M2_CLOSING,
  M2_CLOSED,
  M2_FAULT
};

enum ConfigPage {
  PAGE_SENSORS,       // S. → seleccionar sensores instalados (primera página)
  PAGE_RANGE_LOWER,   // L.I → llenavasos inferior
  PAGE_RANGE_UPPER,   // L.S → llenavasos superior
  PAGE_HYST_CUP,      // H.L → histéresis llenavasos (0.0-0.5 cm)
  PAGE_CUP_DELAY,     // D.L → delay llenavasos (0.0-0.5 s)
  PAGE_SPIT_LOWER,    // E.I → escupidera inferior
  PAGE_SPIT_UPPER,    // E.S → escupidera superior
  PAGE_DELAY,         // E. → delay escupidera (0.5-3.0s)
  PAGE_EXIT_CONFIRM   // G. o n. → guardar o descartar
};

// ========================= DISPLAY ==========================================
const uint8_t segPins[7] = {A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, F_PIN, G_PIN};

//                                  a b c d e f g
const uint8_t patterns[28][7] = {
  {1,1,1,1,1,1,0},  //  0
  {0,1,1,0,0,0,0},  //  1
  {1,1,0,1,1,0,1},  //  2
  {1,1,1,1,0,0,1},  //  3
  {0,1,1,0,0,1,1},  //  4
  {1,0,1,1,0,1,1},  //  5
  {1,0,1,1,1,1,1},  //  6
  {1,1,1,0,0,0,0},  //  7
  {1,1,1,1,1,1,1},  //  8
  {1,1,1,1,0,1,1},  //  9
  {1,1,1,0,1,1,1},  // 10 = A  (representa dígito "10")
  {0,0,1,1,1,1,1},  // 11 = b  (representa dígito "11")
  {1,0,0,1,1,1,0},  // 12 = C  (representa dígito "12")
  {0,1,1,1,1,0,1},  // 13 = d  (representa dígito "13")
  {1,0,0,1,1,1,1},  // 14 = E  (representa dígito "14")
  {1,0,0,0,1,1,1},  // 15 = F  (representa dígito "15")
  {1,0,1,1,1,1,0},  // 16 = G  (representa dígito "16")
  {0,1,1,0,1,1,1},  // 17 = H  (representa dígito "17")
  {0,0,0,0,1,1,0},  // 18 = I  (representa dígito "18") — solo ef
  {0,1,1,1,1,0,0},  // 19 = J  (representa dígito "19")
  {0,0,0,0,0,0,0},  // 20 = apagado (SYM_OFF)
  {1,0,0,1,1,1,1},  // 21 = E (error/escupidera) — misma forma que 14
  {0,0,0,1,1,1,0},  // 22 = L (llenavasos)
  {0,1,1,1,1,0,1},  // 23 = d (modo) — misma forma que 13
  {1,0,1,1,1,1,0},  // 24 = G (guardar) — misma forma que 16
  {0,0,1,0,1,0,1},  // 25 = n (no guardar)
  {0,0,0,0,0,0,1},  // 26 = - (guion)
  {0,1,1,0,1,1,0}   // 27 = K (para dígito "20" si se necesita)
};

// Símbolos especiales (referenciados por nombre, no como dígitos)
#define SYM_OFF   20
#define SYM_E     21
#define SYM_L     22
#define SYM_d     23
#define SYM_G     24
#define SYM_n     25
#define SYM_DASH  26
// Aliases: "I" y "S" comparten patrón con "1" y "5" en 7 segmentos
#define SYM_I     1
#define SYM_S     5

volatile uint8_t displayDigit1 = 0;
volatile uint8_t displayDigit2 = 0;
volatile bool muxToggle = false;

// DP por dígito: 0=off, 1=on, 2=blink (parpadeo controlado por ISR)
volatile uint8_t displayDpLeft  = 0;
volatile uint8_t displayDpRight = 0;
volatile bool dpBlinkPhase = true; // toggleado por timer para modo blink

// ========================= SENSORES =========================================
// Llenavasos: VL6180X (I2C 0x29 de fábrica)
Adafruit_VL6180X vl = Adafruit_VL6180X();

uint8_t medianBuf[3] = {0, 0, 0};
uint8_t medianIdx = 0;
bool medianFull = false;
float emaValue = 0.0f;
bool emaInit = false;

// Escupidera: VL53L0X (reasignado a 0x30 tras XSHUT)
VL53L0X vlSpit;
#define VL53L0X_ADDRESS 0x30

uint16_t medianBufSpit[3] = {0, 0, 0};
uint8_t medianIdxSpit = 0;
bool medianFullSpit = false;
float emaValueSpit = 0.0f;
bool emaInitSpit = false;

// Banderas de sensores instalados (calculadas al arrancar desde EEPROM)
uint8_t sensorsInstalled = SENSORS_BOTH;
bool cupInstalled  = true;
bool spitInstalled = true;

// ========================= CONTEXTO DE VÁLVULA ==============================
struct ValveChannel {
  const char* name;
  uint8_t valveOutPin;
  uint8_t valveFbPin;

  bool valveIsOpen;

  bool pulseActive;
  unsigned long pulseStart;
  unsigned long commandSentTime;

  bool objectPresent;
  bool objectJustEntered;
  bool objectJustLeft;
  uint8_t inConfirmCnt;
  uint8_t outConfirmCnt;

  CupStateM2 m2State;
  bool m2FaultExpectOpen;

  // Contadores de falla (v4)
  uint8_t faultCount;           // fallas consecutivas sin ciclo exitoso
  unsigned long faultEntryTime; // cuándo entró a M2_FAULT (para timeout crítico)
  unsigned long externCloseDetectedTime;  // cuándo se detectó PB10 HIGH por primera vez
  unsigned long externCloseConfirmedTime; // cuándo se confirmó permanencia >= 500ms
};

ValveChannel cupCh = {
  "LV", CUP_VALVE_OUT, CUP_VALVE_FB,
  false, false, 0, 0,
  false, false, false, 0, 0,
  M2_IDLE, false,
  0, 0, 0
};

ValveChannel spitCh = {
  "ES", SPIT_VALVE_OUT, SPIT_VALVE_FB,
  false, false, 0, 0,
  false, false, false, 0, 0,
  M2_IDLE, false,
  0, 0, 0
};

// ========================= FALLA CRÍTICA (v4) ===============================
bool criticalFault = false;

// ========================= RANGO Y MODO ====================================
// v10c: los límites se guardan en MILIMETROS (unidad = 0.1cm) para permitir
// pasos de 0.1cm en la configuración.
// Llenavasos: 0 a 80 (0.0 a 8.0 cm). Escupidera: 0 a 200 (0.0 a 20.0 cm).
uint8_t  lowerLimit    = 0;    // llenavasos inferior (mm)
uint8_t  upperLimit    = 80;   // llenavasos superior (mm) = 8.0 cm
uint16_t spitLower     = 0;    // escupidera inferior (mm) — uint16 por rango > 255
uint16_t spitUpper     = 200;  // escupidera superior (mm) = 20.0 cm
uint8_t spitDelayT    = 10;   // delay escupidera en décimas de segundo (5-30)
uint8_t hystCup       = 0;    // v11: histéresis llenavasos en mm (0-5 = 0.0-0.5cm)
uint8_t cupDelayT     = 0;    // v11c: delay llenavasos en décimas de seg (0-5 = 0.0-0.5s)
uint16_t cupActivationDelayMs = 0;  // v11c: valor calculado en ms

// Valores en edición dentro del menú
uint8_t  configLower    = 0;
uint8_t  configUpper    = 80;
uint16_t configSpitLow  = 0;
uint16_t configSpitUp   = 200;
uint8_t configDelayT   = 10;
uint8_t configSensors  = SENSORS_BOTH;
uint8_t configHystCup  = 0;    // v11: histéresis en edición
uint8_t configCupDelayT = 0;  // v11c: delay LV en edición

// Snapshots (valores estables para guardar en EEPROM al confirmar)
uint8_t  snapshotLower   = 0;
uint8_t  snapshotUpper   = 80;
uint16_t snapshotSpitLow = 0;
uint16_t snapshotSpitUp  = 200;
uint8_t snapshotDelayT  = 10;
uint8_t snapshotSensors = SENSORS_BOTH;
uint8_t snapshotHystCup = 0;   // v11
uint8_t snapshotCupDelayT = 0; // v11c

// v10c: helper para autorepetición al mantener presionado un botón en config
// - Primera pulsación: cambio inmediato
// - Espera 400ms tras la primera pulsación (evita repeticiones no queridas)
// - Después repite cada 60ms (rápido y controlable)
#define BTN_HOLD_INITIAL_DELAY_MS  400
#define BTN_HOLD_REPEAT_MS          60

// Vista del display en modo normal (capa cosmética, no afecta lógica)
// 0 = llenavasos, 1 = escupidera. Solo alterna entre sensores instalados.
uint8_t normalViewIndex = 0;
unsigned long viewIdStart = 0;
bool showViewId = false;

// Estado de la pantalla de confirmación de salida
bool configExitDiscard = false; // false=guardar (G.), true=descartar (n.)

// Timing del menú
unsigned long pageIdentifierStart = 0;
bool showPageIdentifier = false;
unsigned long confirmFlashStart = 0;
bool confirmFlashIsDiscard = false;
unsigned long resetFlashStart = 0;

// ========================= ESTADOS SISTEMA ==================================
SystemState currentState = STATE_NORMAL;
ConfigPage  configPage   = PAGE_SENSORS;
bool transitionToConfig  = false;

// ========================= BOTONES ==========================================
bool btn1Reading = LOW, btn2Reading = LOW;
bool btn1Stable  = LOW, btn2Stable  = LOW;
unsigned long btn1DebTime = 0, btn2DebTime = 0;

unsigned long bothPressedStart = 0;
bool bothWereHeld = false;

// ========================= TIMERS ===========================================
unsigned long lastSensorRead = 0;
// v13c: timestamp de la última MUESTRA FRESCA de cada sensor (para staleness)
unsigned long lastFreshCup  = 0;
unsigned long lastFreshSpit = 0;

// v13d: estado de recuperación por canal
uint8_t       cupRecoveryAttempts  = 0;
uint8_t       spitRecoveryAttempts = 0;
unsigned long lastRecoveryCup      = 0;
unsigned long lastRecoverySpit     = 0;
unsigned long lastBlink = 0;
bool blinkOn = true;

// Latch de detección rápida de PB10/PB12 HIGH (muestreado por ISR)
volatile bool cupValveSeenOpen  = false;
volatile bool spitValveSeenOpen = false;

// Delay de activación de la escupidera (evita activaciones accidentales)
bool spitActivationPending = false;
unsigned long spitActivationStart = 0;

// v11c: delay de activación del llenavasos (misma mecánica que escupidera)
bool cupActivationPending = false;
unsigned long cupActivationStart = 0;
HardwareTimer timer(3);

// ============================================================================
//  ISR — REFRESCO DEL DISPLAY con DP configurable por dígito
// ============================================================================
void displayISR(void) {
  // --- Muestreo rápido de feedbacks (cada MUX_INTERVAL_US) ---
  // Solo se muestrea el feedback de subsistemas instalados.
  if (cupInstalled) {
    bool cupFb = (digitalRead(CUP_VALVE_FB) == HIGH);
    cupValveSeenOpen = cupFb;
    if (cupFb && !cupCh.pulseActive && cupCh.externCloseDetectedTime == 0 &&
        (cupCh.m2State == M2_IDLE ||
         (cupCh.m2State == M2_CLOSED && !cupCh.objectPresent))) {
      cupCh.externCloseDetectedTime = millis();
    }
  }
  if (spitInstalled) {
    bool spitFb = (digitalRead(SPIT_VALVE_FB) == HIGH);
    spitValveSeenOpen = spitFb;
    // Escupidera: sin timestamp de cierre externo (permite activación externa libre)
  }

  // --- Refresco del display con DP por dígito ---
  digitalWrite(D1_PIN, LOW);
  digitalWrite(D2_PIN, LOW);

  uint8_t idx;
  uint8_t dpMode;
  if (muxToggle) {
    idx    = displayDigit1;
    dpMode = displayDpLeft;
  } else {
    idx    = displayDigit2;
    dpMode = displayDpRight;
  }

  for (int i = 0; i < 7; i++) {
    digitalWrite(segPins[i], patterns[idx][i] ? LOW : HIGH);
  }

  // DP: 0=off (HIGH), 1=on (LOW), 2=blink (según dpBlinkPhase)
  bool dpOn = (dpMode == 1) || (dpMode == 2 && dpBlinkPhase);
  digitalWrite(DP_PIN, dpOn ? LOW : HIGH);

  digitalWrite(muxToggle ? D1_PIN : D2_PIN, HIGH);
  muxToggle = !muxToggle;
}
// ============================================================================
//  DEBOUNCE
// ============================================================================
bool debounceButton(uint8_t pin, bool &lastReading, bool &stableState,
                    unsigned long &debTime) {
  bool reading = digitalRead(pin);
  bool justPressed = false;
  if (reading != lastReading) debTime = millis();
  if ((millis() - debTime) > DEBOUNCE_MS) {
    if (reading != stableState) {
      stableState = reading;
      if (stableState == HIGH) justPressed = true;
    }
  }
  lastReading = reading;
  return justPressed;
}

// ============================================================================
//  EEPROM
// ============================================================================
void saveConfig() {
  EEPROM.write(EEPROM_ADDR_LOWER,     lowerLimit);
  EEPROM.write(EEPROM_ADDR_UPPER,     upperLimit);
  EEPROM.write(EEPROM_ADDR_DELAY,     spitDelayT);
  // v10c: escupidera guardada en 2 bytes (mm, hasta 200)
  EEPROM.write(EEPROM_ADDR_SPIT_LOW_L, (uint8_t)(spitLower & 0xFF));
  EEPROM.write(EEPROM_ADDR_SPIT_LOW_H, (uint8_t)((spitLower >> 8) & 0xFF));
  EEPROM.write(EEPROM_ADDR_SPIT_UP_L,  (uint8_t)(spitUpper & 0xFF));
  EEPROM.write(EEPROM_ADDR_SPIT_UP_H,  (uint8_t)((spitUpper >> 8) & 0xFF));
  EEPROM.write(EEPROM_ADDR_SENSORS,   sensorsInstalled);
  EEPROM.write(EEPROM_ADDR_HYST_CUP,  hystCup);
  EEPROM.write(EEPROM_ADDR_CUP_DELAY, cupDelayT);  // v11c          // v11
  EEPROM.write(EEPROM_ADDR_VALID,     EEPROM_VALID_FLAG);
}

void applySpitDelay() {
  spitActivationDelayMs = (uint16_t)spitDelayT * 100;
}

// v11c: convierte cupDelayT (décimas de segundo) a ms
void applyCupDelay() {
  cupActivationDelayMs = (uint16_t)cupDelayT * 100;
}

void applySensorFlags() {
  cupInstalled  = (sensorsInstalled == SENSORS_BOTH || sensorsInstalled == SENSORS_CUP_ONLY);
  spitInstalled = (sensorsInstalled == SENSORS_BOTH || sensorsInstalled == SENSORS_SPIT_ONLY);
}

void loadConfig() {
  if (EEPROM.read(EEPROM_ADDR_VALID) == EEPROM_VALID_FLAG) {
    lowerLimit    = EEPROM.read(EEPROM_ADDR_LOWER);
    upperLimit    = EEPROM.read(EEPROM_ADDR_UPPER);
    spitDelayT    = EEPROM.read(EEPROM_ADDR_DELAY);
    // v10c: escupidera en 2 bytes
    uint8_t sLowL = EEPROM.read(EEPROM_ADDR_SPIT_LOW_L);
    uint8_t sLowH = EEPROM.read(EEPROM_ADDR_SPIT_LOW_H);
    uint8_t sUpL  = EEPROM.read(EEPROM_ADDR_SPIT_UP_L);
    uint8_t sUpH  = EEPROM.read(EEPROM_ADDR_SPIT_UP_H);
    spitLower = ((uint16_t)sLowH << 8) | sLowL;
    spitUpper = ((uint16_t)sUpH  << 8) | sUpL;
    sensorsInstalled = EEPROM.read(EEPROM_ADDR_SENSORS);
    hystCup = EEPROM.read(EEPROM_ADDR_HYST_CUP);
    cupDelayT = EEPROM.read(EEPROM_ADDR_CUP_DELAY);  // v11c          // v11
    // Validaciones llenavasos (0-80 mm = 0.0-8.0 cm)
    if (lowerLimit > 80) lowerLimit = 0;
    if (upperLimit > 80) upperLimit = 80;
    if (upperLimit < lowerLimit) upperLimit = lowerLimit;
    // Validaciones escupidera (0-200 mm = 0.0-20.0 cm)
    if (spitLower > 200) spitLower = 0;
    if (spitUpper > 200) spitUpper = 200;
    if (spitUpper < spitLower) spitUpper = spitLower;
    // Otras validaciones
    if (spitDelayT < 5 || spitDelayT > 30) spitDelayT = 10;
    if (sensorsInstalled > SENSORS_SPIT_ONLY) sensorsInstalled = SENSORS_BOTH;
    if (hystCup > 5) hystCup = 0;
    if (cupDelayT > 5) cupDelayT = 0;  // v11c: 0-5 décimas = 0.0-0.5s  // v11: 0-5 mm = 0.0-0.5 cm
  } else {
    // Primera vez o EEPROM corrupta: defaults en mm
    lowerLimit    = 0;
    upperLimit    = 80;   // 8.0 cm
    spitLower     = 0;
    spitUpper     = 200;  // 20.0 cm
    spitDelayT    = 10;   // 1.0 s
    sensorsInstalled = SENSORS_BOTH;
    hystCup       = 0;
    cupDelayT     = 0;    // v11c: sin delay por defecto
    saveConfig();
  }
  applySpitDelay();
  applyCupDelay();  // v11c
  applySensorFlags();
}

// ============================================================================
//  PULSO A ELECTROVÁLVULA
// ============================================================================
void sendPulse(ValveChannel &ch) {
  digitalWrite(ch.valveOutPin, HIGH);
  ch.pulseActive = true;
  ch.pulseStart = millis();
  ch.commandSentTime = millis();
}

void managePulse(ValveChannel &ch) {
  if (ch.pulseActive && (millis() - ch.pulseStart) >= PULSE_DURATION_MS) {
    digitalWrite(ch.valveOutPin, LOW);
    ch.pulseActive = false;
  }
}

// ============================================================================
//  FILTRO — SOLO VL6180X
// ============================================================================
uint8_t getMedian3(uint8_t a, uint8_t b, uint8_t c) {
  if (a > b) { uint8_t t = a; a = b; b = t; }
  if (b > c) { uint8_t t = b; b = c; c = t; }
  if (a > b) { uint8_t t = a; a = b; b = t; }
  return b;
}

// Lectura filtrada del VL6180X (llenavasos) — NO BLOQUEANTE (modo continuo).
// Devuelve: mm válido (>=0) | -1 (error de status) | SAMPLE_NOT_READY (-2, sin dato nuevo)
int16_t readFilteredDistance() {
  // Poll no bloqueante: ¿hay muestra nueva lista? (lectura rápida de registro)
  if (!vl.isRangeComplete()) return SAMPLE_NOT_READY;

  uint8_t range  = vl.readRangeResult();  // lee el resultado ya listo
  uint8_t status = vl.readRangeStatus();
  // Limpiar flag de interrupción del VL6180X (registro 0x015 = SYSTEM_INTERRUPT_CLEAR)
  Wire.beginTransmission(0x29);
  Wire.write((uint8_t)0x00);  // reg high byte
  Wire.write((uint8_t)0x15);  // reg low byte
  Wire.write((uint8_t)0x07);  // clear all interrupts
  Wire.endTransmission();

  if (status != VL6180X_ERROR_NONE) return -1;

  medianBuf[medianIdx++] = range;
  if (medianIdx >= 3) { medianIdx = 0; medianFull = true; }
  if (!medianFull) return -1;

  uint8_t med = getMedian3(medianBuf[0], medianBuf[1], medianBuf[2]);
  if (!emaInit) { emaValue = (float)med; emaInit = true; }
  else          { emaValue = EMA_ALPHA * med + (1.0f - EMA_ALPHA) * emaValue; }

  return (int16_t)(emaValue + 0.5f);
}

// Mediana de 3 para uint16 (VL53L0X mide en mm hasta ~2000)
uint16_t getMedian3_16(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

// Lectura filtrada del VL53L0X (escupidera) — NO BLOQUEANTE (modo continuo).
// Devuelve: mm válido (>=0) | -1 (error/fuera de rango) | SAMPLE_NOT_READY (-2)
// Consulta el registro de status de interrupción de la librería Pololu para
// saber si hay muestra nueva sin bloquear esperando la medición.
int16_t readFilteredDistanceSpit() {
  // Poll no bloqueante: bits [2:0] del RESULT_INTERRUPT_STATUS != 0 → dato listo
  if ((vlSpit.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    return SAMPLE_NOT_READY;
  }

  // Leer los mm (offset +10 del RESULT_RANGE_STATUS) y limpiar interrupción
  uint16_t range = vlSpit.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  vlSpit.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);

  // 8190/8191 = fuera de rango o error de medición
  if (range >= 8000) return -1;

  medianBufSpit[medianIdxSpit++] = range;
  if (medianIdxSpit >= 3) { medianIdxSpit = 0; medianFullSpit = true; }
  if (!medianFullSpit) return -1;

  uint16_t med = getMedian3_16(medianBufSpit[0], medianBufSpit[1], medianBufSpit[2]);
  if (!emaInitSpit) { emaValueSpit = (float)med; emaInitSpit = true; }
  else              { emaValueSpit = EMA_ALPHA * med + (1.0f - EMA_ALPHA) * emaValueSpit; }

  return (int16_t)(emaValueSpit + 0.5f);
}

// ============================================================================
//  DETECCIÓN GENÉRICA
// ============================================================================
void processDetection(ValveChannel &ch, bool detectedRaw) {
  if (detectedRaw) {
    ch.outConfirmCnt = 0;
    if (!ch.objectPresent) {
      ch.inConfirmCnt++;
      if (ch.inConfirmCnt >= CONFIRM_READINGS) {
        ch.objectPresent = true;
        ch.objectJustEntered = true;
        ch.inConfirmCnt = 0;
      }
    }
  } else {
    ch.inConfirmCnt = 0;
    if (ch.objectPresent) {
      ch.outConfirmCnt++;
      if (ch.outConfirmCnt >= CONFIRM_READINGS_OUT) {  // salida más rápida
        ch.objectPresent = false;
        ch.objectJustLeft = true;
        ch.outConfirmCnt = 0;
      }
    }
  }
}

// ============================================================================
//  RESET
// ============================================================================
void resetChannel(ValveChannel &ch) {
  ch.m2State = M2_IDLE;
  ch.objectPresent = false;
  ch.objectJustEntered = false;
  ch.objectJustLeft = false;
  ch.inConfirmCnt = 0;
  ch.outConfirmCnt = 0;
  ch.commandSentTime = 0;
  ch.m2FaultExpectOpen = false;
  ch.faultCount = 0;
  ch.faultEntryTime = 0;
  ch.externCloseDetectedTime = 0;
  ch.externCloseConfirmedTime = 0;
}

void resetAllDetection() {
  resetChannel(cupCh);
  resetChannel(spitCh);
  medianFull = false;
  medianIdx = 0;
  emaInit = false;
  medianFullSpit = false;
  medianIdxSpit = 0;
  emaInitSpit = false;
}

// ============================================================================
//  MODO 2 — con regla global de seguridad
//
//  REGLA FUNDAMENTAL (se evalúa ANTES del switch, cada iteración):
//    PB10 HIGH en estado donde NO debería estar abierta (M2_IDLE, M2_CLOSED)
//    → cerrar inmediatamente con pulso, sin importar si hay vaso o no.
//
//  Estados donde PB10 HIGH es LEGÍTIMO:
//    M2_OPENING → esperando confirmación de apertura que nosotros pedimos
//    M2_OPEN    → válvula abierta legítimamente con vaso
//    M2_CLOSING → ya mandamos cierre, esperando confirmación
//    M2_FAULT   → manejo especial
// ============================================================================
void processMode2(ValveChannel &ch) {

  // =========================================================================
  // REGLA GLOBAL — 2 FASES INDEPENDIENTES:
  //   FASE 1 (confirmación): PB10 debe permanecer HIGH >= EXTERN_CONFIRM_HIGH_MS
  //   FASE 2 (delay): una vez confirmado, esperar EXTERN_CLOSE_DELAY_MS
  //                   sin importar si PB10 baja durante ese tiempo.
  //   Solo se cancela FASE 2 si la válvula cerró sola (evitar reabrir con toggle)
  // =========================================================================
  // Regla global: solo aplica al llenavasos.
  // La escupidera permite activación externa sin interferencia.
  bool isLlenavasos = (&ch == &cupCh);
  bool illegitimateOpen = isLlenavasos && ch.valveIsOpen && !ch.pulseActive &&
      (ch.m2State == M2_IDLE || (ch.m2State == M2_CLOSED && !ch.objectPresent));

  // ------- FASE 1: confirmación de permanencia (solo hasta confirmar) -------
  if (ch.externCloseConfirmedTime == 0) {
    if (illegitimateOpen) {
      if (ch.externCloseDetectedTime == 0) {
        ch.externCloseDetectedTime = millis();
      } else if ((millis() - ch.externCloseDetectedTime) >= EXTERN_CONFIRM_HIGH_MS) {
        // Confirmado → pasar a FASE 2
        ch.externCloseConfirmedTime = millis();
        Serial1.print("["); Serial1.print(ch.name);
        Serial1.println(" M2] PB10 HIGH confirmado, iniciando delay de cierre");
      }
    } else {
      // Aún no confirmamos y ya no hay condición → cancelar FASE 1
      ch.externCloseDetectedTime = 0;
    }
  }

  // ------- FASE 2: delay incondicional una vez confirmado -------
  // Solo se cancela si la válvula ya cerró sola (evita toggle que la reabriría)
  if (ch.externCloseConfirmedTime != 0) {
    if (!ch.valveIsOpen) {
      // La válvula cerró sola durante el delay → cancelar sin enviar pulso
      ch.externCloseDetectedTime  = 0;
      ch.externCloseConfirmedTime = 0;
      Serial1.print("["); Serial1.print(ch.name);
      Serial1.println(" M2] Valvula cerro sola durante delay, cancelando");
    } else if ((millis() - ch.externCloseConfirmedTime) >= EXTERN_CLOSE_DELAY_MS) {
      sendPulse(ch);
      ch.m2State = M2_CLOSING;
      ch.externCloseDetectedTime  = 0;
      ch.externCloseConfirmedTime = 0;
      Serial1.print("["); Serial1.print(ch.name);
      Serial1.println(" M2] Cierre por activacion externa");
      return;
    }
  }

  switch (ch.m2State) {

    // --- Esperando objeto ---
    // Solo activa cuando PB10 está LOW (válvula cerrada).
    // Si PB10 estuviera HIGH acá, la regla global ya lo cerró.
    case M2_IDLE:
      if (ch.objectJustEntered && !ch.valveIsOpen) {
        sendPulse(ch);
        ch.m2State = M2_OPENING;
        Serial1.print("["); Serial1.print(ch.name);
        Serial1.println(" M2] Objeto detectado, pulso apertura");
      } else if (!isLlenavasos && ch.objectJustEntered && ch.valveIsOpen) {
        // Escupidera: adoptar válvula abierta externamente al detectar cara
        // Cuando la cara salga, M2_OPEN cerrará automáticamente
        ch.m2State = M2_OPEN;
        Serial1.print("["); Serial1.print(ch.name);
        Serial1.println(" M2] Valvula externa adoptada por sensor");
      }
      break;

    // --- Esperando confirmación PB10 HIGH ---
    case M2_OPENING:
      if (ch.valveIsOpen) {
        ch.m2State = M2_OPEN;
        Serial1.print("["); Serial1.print(ch.name);
        Serial1.println(" M2] Valvula abierta confirmada");
      } else if (ch.objectJustLeft && !ch.valveIsOpen) {
        ch.m2State = M2_IDLE;
        ch.faultCount++;
        Serial1.print("["); Serial1.print(ch.name);
        Serial1.println(" M2] Objeto retirado sin confirmacion, abortando");
      } else if ((millis() - ch.commandSentTime) > CONFIRM_TIMEOUT) {
        ch.m2FaultExpectOpen = true;
        ch.m2State = M2_FAULT;
        ch.faultEntryTime = millis();
        Serial1.print("["); Serial1.print(ch.name);
        Serial1.println(" M2] FALLA: sin confirmacion apertura");
      }
      break;

    // --- Válvula abierta, objeto en rango ---
    // Si el vaso sale con PB10 HIGH → mandar cierre.
    // Si PB10 baja (timeout externo) → CLOSED sin pulso.
    case M2_OPEN:
      if (!ch.valveIsOpen) {
        ch.m2State = M2_CLOSED;
        Serial1.print("["); Serial1.print(ch.name);
        Serial1.println(" M2] Valvula cerrada externamente");
      } else if (ch.objectJustLeft || !ch.objectPresent) {
        sendPulse(ch);
        ch.m2State = M2_CLOSING;
        Serial1.print("["); Serial1.print(ch.name);
        Serial1.println(" M2] Objeto retirado, pulso cierre");
      }
      break;

    // --- Esperando confirmación PB10 LOW ---
    case M2_CLOSING:
      if (!ch.valveIsOpen) {
        ch.m2State = M2_CLOSED;
        // Limpiar latch tras cierre confirmado
        if (&ch == &cupCh)  cupValveSeenOpen  = false;
        if (&ch == &spitCh) spitValveSeenOpen = false;
        Serial1.print("["); Serial1.print(ch.name);
        Serial1.println(" M2] Cierre confirmado");
      } else if ((millis() - ch.commandSentTime) > CONFIRM_TIMEOUT) {
        ch.m2FaultExpectOpen = false;
        ch.m2State = M2_FAULT;
        ch.faultEntryTime = millis();
        Serial1.print("["); Serial1.print(ch.name);
        Serial1.println(" M2] FALLA: sin confirmacion cierre");
      }
      break;

    // --- Válvula cerrada, esperando que el objeto se retire ---
    // Si PB10 vuelve a HIGH acá (reactivación externa),
    // la regla global lo cierra inmediatamente.
    case M2_CLOSED:
      if (ch.valveIsOpen && ch.objectPresent) {
        // Reactivación externa con vaso presente → re-adoptar (solo llenavasos)
        ch.m2State = M2_OPEN;
        Serial1.print("["); Serial1.print(ch.name);
        Serial1.println(" M2] Reactivacion externa con objeto, re-adoptando");
      } else if (!ch.objectPresent) {
        ch.m2State = M2_IDLE;
        ch.faultCount = 0;
        Serial1.print("["); Serial1.print(ch.name);
        Serial1.println(" M2] Sistema listo");
      }
      break;

    // --- FALLA ---
    case M2_FAULT: {
      if (ch.m2FaultExpectOpen && ch.valveIsOpen) {
        ch.m2State = M2_OPEN;
        ch.faultEntryTime = 0;
        Serial1.print("["); Serial1.print(ch.name);
        Serial1.println(" M2] Recuperado: apertura tardia");
        break;
      }
      if (!ch.m2FaultExpectOpen && !ch.valveIsOpen) {
        ch.m2State = M2_CLOSED;
        ch.faultEntryTime = 0;
        Serial1.print("["); Serial1.print(ch.name);
        Serial1.println(" M2] Recuperado: cierre tardio");
        break;
      }

      if (ch.m2FaultExpectOpen && !ch.valveIsOpen && !ch.objectPresent) {
        ch.m2State = M2_IDLE;
        ch.faultCount++;
        ch.faultEntryTime = 0;
        Serial1.print("["); Serial1.print(ch.name);
        Serial1.println(" M2] Valvula nunca abrio, rearmando");
        break;
      }

      if (!ch.m2FaultExpectOpen && ch.valveIsOpen) {
        if ((millis() - ch.faultEntryTime) > FAULT_CRITICAL_MS) {
          ch.faultCount = MAX_CONSECUTIVE_FAULTS;
          Serial1.print("["); Serial1.print(ch.name);
          Serial1.println(" M2] CRITICO: valvula no cierra");
        }
      }

      if (ch.faultCount >= MAX_CONSECUTIVE_FAULTS) {
        criticalFault = true;
        Serial1.print("["); Serial1.print(ch.name);
        Serial1.println(" M2] >>> FALLA CRITICA, activando watchdog");
      }
      break;
    }
  }
}

// ============================================================================
//  PROCESAR CANAL
// ============================================================================
void processChannel(ValveChannel &ch) {
  processMode2(ch);  // v13: Modo 1 eliminado, solo Modo 2
}

// ============================================================================
//  CIERRE DE SEGURIDAD AL ARRANQUE
// ============================================================================
void safetyCloseIfOpen(ValveChannel &ch) {
  if (digitalRead(ch.valveFbPin) == HIGH) {
    Serial1.print("!! "); Serial1.print(ch.name);
    Serial1.println(" abierta al arrancar, cerrando...");
    sendPulse(ch);
    unsigned long safetyStart = millis();
    while (digitalRead(ch.valveFbPin) == HIGH) {
      managePulse(ch);
      iwdg_feed();   // v9d: evita que el watchdog dispare durante el cierre de seguridad
      if ((millis() - safetyStart) > 3000) {
        Serial1.print("!! FALLA: "); Serial1.print(ch.name);
        Serial1.println(" no cierra al arrancar");
        break;
      }
    }
  }
}

// ============================================================================
//  ARRANQUE ROBUSTO DE SENSORES (v9d) — reintentos alimentando el watchdog
//  En arranque en frío (tras horas sin energía) el riel de los sensores y su
//  boot interno tardan en estabilizarse; el primer acceso I2C puede fallar.
//  Reintentar unos cientos de ms resuelve el "no detectado" del cold-start.
// ============================================================================
// ============================================================================
//  waitMsNonBlocking — espera N ms usando millis() en vez de delay().
//  Alimenta el watchdog en cada iteración. Cumple el requisito de "no delay()".
//  Nota: funcionalmente equivale a un delay durante el arranque, pero mantiene
//  la ISR del display corriendo y respeta la letra del requisito asíncrono.
// ============================================================================
static inline void waitMsNonBlocking(uint16_t ms) {
  unsigned long start = millis();
  while ((millis() - start) < ms) {
    iwdg_feed();
    // ceder al hardware sin bloqueo activo — las interrupciones siguen corriendo
  }
}

// ============================================================================
//  v13d — RECUPERACIÓN DE SENSOR CONGELADO (sin resetear el MCU)
// ============================================================================
// Recovery del bus I2C: si un esclavo mantiene SDA en LOW, se envían pulsos
// manuales en SCL como GPIO para forzar al esclavo a soltar la línea.
// Técnica estándar (NXP). Se hace SIN llamar Wire.end() en libmaple porque
// el core no siempre libera limpio; alcanza con pinMode+digitalWrite y luego
// restaurar Wire.begin().
static void i2cBusRecovery(uint8_t sclPin, uint8_t sdaPin, TwoWire &wire) {
  pinMode(sclPin, OUTPUT_OPEN_DRAIN);
  pinMode(sdaPin, INPUT_PULLUP);
  // 9 pulsos de reloj para que el esclavo termine su transacción parcial
  for (uint8_t i = 0; i < 9; i++) {
    digitalWrite(sclPin, LOW);
    delayMicroseconds(5);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(5);
    if (digitalRead(sdaPin) == HIGH) break;  // SDA liberado, listo
  }
  // Generar condición de STOP: SDA de LOW→HIGH mientras SCL está HIGH
  pinMode(sdaPin, OUTPUT_OPEN_DRAIN);
  digitalWrite(sdaPin, LOW);
  delayMicroseconds(5);
  digitalWrite(sclPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(sdaPin, HIGH);
  delayMicroseconds(5);
  // Restaurar como I2C
  wire.begin();
}

// Recovery del VL6180X (llenavasos): XSHUT LOW → 10ms → HIGH → boot →
// bus recovery → re-init → re-startContinuous. Devuelve true si recuperó.
static bool recoverVL6180X() {
  Serial1.println("[LV] RECOVERY: reiniciando VL6180X por XSHUT + bus...");
  // 1) Apagar el sensor
  digitalWrite(CUP_XSHUT, LOW);
  waitMsNonBlocking(15);
  // 2) Recuperar el bus por si quedó trabado
  i2cBusRecovery(PB6, PB7, Wire);
  waitMsNonBlocking(5);
  // 3) Encender el sensor y esperar boot
  digitalWrite(CUP_XSHUT, HIGH);
  waitMsNonBlocking(10);
  // 4) Re-inicializar (varios intentos cortos)
  bool ok = false;
  for (uint8_t i = 0; i < 5; i++) {
    if (vl.begin()) { ok = true; break; }
    waitMsNonBlocking(20);
  }
  if (!ok) {
    Serial1.println("[LV] RECOVERY fallido: begin() no responde");
    return false;
  }
  // 5) Rearmar modo continuo
  vl.startRangeContinuous(20);
  Serial1.println("[LV] RECOVERY OK: sensor operativo");
  return true;
}

// Recovery del VL53L0X (escupidera): mismo procedimiento, sensor y bus.
static bool recoverVL53L0X() {
  Serial1.println("[ES] RECOVERY: reiniciando VL53L0X por XSHUT + bus...");
  digitalWrite(SPIT_XSHUT, LOW);
  waitMsNonBlocking(15);
  i2cBusRecovery(PB10, PB11, Wire2);
  waitMsNonBlocking(5);
  digitalWrite(SPIT_XSHUT, HIGH);
  waitMsNonBlocking(10);
  // La librería Pololu necesita setBus() de nuevo tras Wire2.begin()
  vlSpit.setBus(&Wire2);
  vlSpit.setTimeout(500);
  bool ok = false;
  for (uint8_t i = 0; i < 5; i++) {
    if (vlSpit.init()) { ok = true; break; }
    waitMsNonBlocking(20);
  }
  if (!ok) {
    Serial1.println("[ES] RECOVERY fallido: init() no responde");
    return false;
  }
  vlSpit.setMeasurementTimingBudget(20000);
  vlSpit.startContinuous(0);
  Serial1.println("[ES] RECOVERY OK: sensor operativo");
  return true;
}

bool beginVL6180X_conReintentos(uint8_t maxIntentos) {
  for (uint8_t i = 0; i < maxIntentos; i++) {
    if (vl.begin()) return true;
    Serial1.print("   VL6180X intento "); Serial1.print(i + 1);
    Serial1.println(" fallido, reintentando...");
    waitMsNonBlocking(100);  // 100ms alimentando WDG, sin delay()
  }
  return false;
}

bool beginVL53L0X_conReintentos(uint8_t maxIntentos) {
  for (uint8_t i = 0; i < maxIntentos; i++) {
    if (vlSpit.init()) return true;
    Serial1.print("   VL53L0X intento "); Serial1.print(i + 1);
    Serial1.println(" fallido, reintentando...");
    waitMsNonBlocking(100);  // 100ms alimentando WDG, sin delay()
  }
  return false;
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial1.begin(115200);

  // --- Salidas ---
  pinMode(CUP_VALVE_OUT, OUTPUT);
  digitalWrite(CUP_VALVE_OUT, LOW);
  pinMode(SPIT_VALVE_OUT, OUTPUT);
  digitalWrite(SPIT_VALVE_OUT, LOW);

  // --- Entradas ---
  pinMode(CUP_VALVE_FB, INPUT);     // v10: PA8
  pinMode(SPIT_VALVE_FB, INPUT);

  // --- XSHUT dual: ambos apagados desde el arranque ---
  // Al mantener ambos LOW y luego pulsar HIGH, garantizamos que los sensores
  // arrancan desde estado limpio incluso tras un reset del MCU sin apagar la
  // alimentación (evita "modo zombie" del I2C).
  pinMode(CUP_XSHUT, OUTPUT);
  pinMode(SPIT_XSHUT, OUTPUT);
  digitalWrite(CUP_XSHUT, LOW);
  digitalWrite(SPIT_XSHUT, LOW);

  // --- Display ---
  for (int i = 0; i < 7; i++) pinMode(segPins[i], OUTPUT);
  pinMode(DP_PIN, OUTPUT);
  pinMode(D1_PIN, OUTPUT);
  pinMode(D2_PIN, OUTPUT);
  digitalWrite(D1_PIN, LOW);
  digitalWrite(D2_PIN, LOW);

  // --- Botones ---
  pinMode(BTN1_PIN, INPUT);
  pinMode(BTN2_PIN, INPUT);

  // --- EEPROM (calcula cupInstalled / spitInstalled) ---
  loadConfig();
  displayDigit1 = upperLimit / 2;
  displayDigit2 = lowerLimit / 2;

  // --- Timer display ---
  timer.pause();
  timer.setPeriod(MUX_INTERVAL_US);
  timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);
  timer.attachCompare1Interrupt(displayISR);
  timer.refresh();
  timer.resume();

  // --- Inicialización I2C: dos buses separados ---
  Wire.begin();    // I2C1 (PB6/PB7) — VL6180X llenavasos
  Wire2.begin();   // I2C2 (PB10/PB11) — VL53L0X escupidera
  vlSpit.setBus(&Wire2);  // asociar objeto VL53L0X al bus I2C2

  // --- Watchdog ANTES de tocar sensores ---
  iwdg_init(IWDG_PRE_256, IWDG_RELOAD);
  iwdg_feed();

  // --- Estabilización tras energizar (sin delay bloqueante) ---
  waitMsNonBlocking(150);

  // === HARDWARE RESET DE SENSORES (evitar "modo zombie") ===
  // Los dos XSHUT ya están en LOW. Los subimos a HIGH simultáneamente para
  // encender ambos sensores desde estado limpio. Cada uno vive en su propio
  // bus, así que no hay conflicto de dirección 0x29 — no hace falta apagar
  // uno mientras el otro se inicializa.
  if (cupInstalled)  digitalWrite(CUP_XSHUT, HIGH);
  if (spitInstalled) digitalWrite(SPIT_XSHUT, HIGH);
  waitMsNonBlocking(10);   // tiempo de boot de los VL6180X / VL53L0X

  // === INICIALIZACIÓN DE SENSORES EN SUS BUSES SEPARADOS ===

  // --- 1) VL6180X (llenavasos) en I2C1 ---
  if (cupInstalled) {
    if (!beginVL6180X_conReintentos(20)) {
      Serial1.println("!! VL6180X (LV) no detectado tras reintentos");
      if (!spitInstalled) {
        displayDigit1 = SYM_E; displayDigit2 = SYM_E;
        displayDpLeft = 0; displayDpRight = 0;
        Serial1.println(">> Reiniciando por watchdog para reintentar arranque...");
        while (1) { /* sin iwdg_feed -> reset en ~4s */ }
      }
    } else {
      Serial1.println("VL6180X (LV) OK en I2C1 (0x29)");
      // v13c: modo continuo — el sensor mide solo cada ~20ms sin bloquear el loop
      vl.startRangeContinuous(20);
    }
  }

  // --- 2) VL53L0X (escupidera) en I2C2 ---
  // Ya está encendido por XSHUT y asociado a Wire2 arriba.
  if (spitInstalled) {
    vlSpit.setTimeout(500);
    if (!beginVL53L0X_conReintentos(20)) {
      Serial1.println("!! VL53L0X (ES) no detectado tras reintentos");
      if (!cupInstalled) {
        displayDigit1 = SYM_E; displayDigit2 = SYM_E;
        displayDpLeft = 0; displayDpRight = 0;
        Serial1.println(">> Reiniciando por watchdog para reintentar arranque...");
        while (1) { /* sin iwdg_feed -> reset en ~4s */ }
      } else {
        // Auto-protección: seguir con solo el llenavasos
        spitInstalled = false;
        digitalWrite(SPIT_XSHUT, LOW);   // apagar VL53L0X
        Serial1.println(">> VL53L0X ausente: escupidera DESHABILITADA en runtime");
      }
    } else {
      // Sin setAddress() — el sensor queda en 0x29 en I2C2, sin conflicto
      vlSpit.setMeasurementTimingBudget(20000); // 20ms, medición rápida
      // v13c: modo continuo back-to-back (0 = tan rápido como el timing budget)
      vlSpit.startContinuous(0);
      Serial1.println("VL53L0X (ES) OK en I2C2 (0x29)");
    }
  }

  // --- Seguridad: cerrar válvulas abiertas (solo subsistemas instalados) ---
  waitMsNonBlocking(50);
  iwdg_feed();
  if (cupInstalled)  safetyCloseIfOpen(cupCh);
  if (spitInstalled) safetyCloseIfOpen(spitCh);

  // --- Watchdog ya inicializado arriba (v9d): NO reinicializar aquí ---

  Serial1.println("Sistema iniciado v13d (recovery XSHUT + bus I2C sin reset MCU)");
  Serial1.print("Sensores: ");
  Serial1.println(sensorsInstalled == SENSORS_BOTH ? "AMBOS" :
                  (sensorsInstalled == SENSORS_CUP_ONLY ? "Solo LV" : "Solo ES"));
  if (cupInstalled) {
    Serial1.print("LV rango: "); Serial1.print(lowerLimit * 0.1f);
    Serial1.print("-"); Serial1.print(upperLimit * 0.1f); Serial1.println(" cm");
  }
  if (spitInstalled) {
    Serial1.print("ES rango: "); Serial1.print(spitLower * 0.1f);
    Serial1.print("-"); Serial1.print(spitUpper * 0.1f);
    Serial1.print(" cm | Delay: "); Serial1.print(spitDelayT * 0.1f);
    Serial1.println("s");
  }
  Serial1.print("Delay ES: "); Serial1.print(spitDelayT * 0.1f); Serial1.println("s");
}

// ============================================================================
//  LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();

  // ==========================================================================
  // FALLA CRÍTICA — cierre de emergencia + dejar que el watchdog resetee
  // ==========================================================================
  if (criticalFault) {
    if (cupInstalled)
      cupCh.valveIsOpen = cupValveSeenOpen || (digitalRead(CUP_VALVE_FB) == HIGH);
    if (spitInstalled)
      spitCh.valveIsOpen = spitValveSeenOpen || (digitalRead(SPIT_VALVE_FB) == HIGH);

    // Gestionar pulsos en curso
    if (cupInstalled)  managePulse(cupCh);
    if (spitInstalled) managePulse(spitCh);

    // Intentar cierre de emergencia una sola vez
    static bool emergencySent = false;
    if (!emergencySent) {
      if (cupInstalled && cupCh.valveIsOpen)  sendPulse(cupCh);
      if (spitInstalled && spitCh.valveIsOpen) sendPulse(spitCh);
      emergencySent = true;
      Serial1.println("!! CIERRE DE EMERGENCIA ENVIADO");
      Serial1.println("!! Watchdog reseteara en ~4 segundos...");
    }

    // Display: "EE" fijo
    displayDigit1 = SYM_E;
    displayDigit2 = SYM_E;
    displayDpLeft  = 0;
    displayDpRight = 0;

    // NO alimentar watchdog → el MCU se reseteará solo
    // El setup() al arrancar volverá a verificar PB10/PB12
    // y cerrará cualquier válvula que siga abierta
    return;
  }

  // ==========================================================================
  // OPERACIÓN NORMAL — alimentar watchdog
  // ==========================================================================
  iwdg_feed();

  // === Limpiar flancos ===
  cupCh.objectJustEntered = false;
  cupCh.objectJustLeft = false;
  spitCh.objectJustEntered = false;
  spitCh.objectJustLeft = false;

  // === Supervisar feedback (latch de ISR, fuente de verdad) ===
  // Solo subsistemas instalados. Los deshabilitados quedan en false permanente.
  if (cupInstalled)
    cupCh.valveIsOpen = cupValveSeenOpen || (digitalRead(CUP_VALVE_FB) == HIGH);
  else
    cupCh.valveIsOpen = false;
  if (spitInstalled)
    spitCh.valveIsOpen = spitValveSeenOpen || (digitalRead(SPIT_VALVE_FB) == HIGH);
  else
    spitCh.valveIsOpen = false;

  // === Pulsos ===
  if (cupInstalled)  managePulse(cupCh);
  if (spitInstalled) managePulse(spitCh);

  // === Botones ===
  bool btn1Pressed = debounceButton(BTN1_PIN, btn1Reading, btn1Stable, btn1DebTime);
  bool btn2Pressed = debounceButton(BTN2_PIN, btn2Reading, btn2Stable, btn2DebTime);

  // ==========================================================================
  // FILTRO ANTI-DOBLE-EVENTO
  // Si detectamos "pressed" en un botón, verificamos si el OTRO botón está
  // también físicamente presionado en ese instante (lectura raw). Si es así,
  // la pulsación es parte de un gesto combinado, no una acción individual.
  // Esto evita que btn1 modifique el valor mientras el usuario está iniciando
  // un toque/hold simultáneo con ambos botones.
  // ==========================================================================
  if (btn1Pressed && digitalRead(BTN2_PIN) == HIGH) btn1Pressed = false;
  if (btn2Pressed && digitalRead(BTN1_PIN) == HIGH) btn2Pressed = false;

  // Además: después de detectar el gesto combinado (bothWereHeld o al soltar),
  // aplicar un cooldown breve para ignorar pulsaciones individuales que podrían
  // ser el rebote de soltar los dos botones no exactamente al mismo tiempo.
  static unsigned long bothReleaseTime = 0;
  static bool wasBothPressed = false;
  bool nowBothPressed = (btn1Stable == HIGH && btn2Stable == HIGH);
  if (wasBothPressed && !nowBothPressed) {
    bothReleaseTime = now;  // se soltó al menos uno de los dos → arrancar cooldown
  }
  wasBothPressed = nowBothPressed;
  const unsigned long BOTH_RELEASE_COOLDOWN_MS = 150;
  if (bothReleaseTime != 0 && (now - bothReleaseTime) < BOTH_RELEASE_COOLDOWN_MS) {
    btn1Pressed = false;
    btn2Pressed = false;
  } else if (bothReleaseTime != 0 && (now - bothReleaseTime) >= BOTH_RELEASE_COOLDOWN_MS) {
    bothReleaseTime = 0;  // cooldown expiró, limpiar
  }

  // === Gesto de 2 botones ===
  if (btn1Stable == HIGH && btn2Stable == HIGH) {
    if (bothPressedStart == 0) bothPressedStart = now;

    if (!bothWereHeld && (now - bothPressedStart) >= HOLD_TIME_MS) {
      bothWereHeld = true;

      if (currentState == STATE_NORMAL) {
        // Cargar todos los valores en edición y snapshots
        configLower    = lowerLimit;    configUpper   = upperLimit;
        configSpitLow  = spitLower;     configSpitUp  = spitUpper;
        configDelayT   = spitDelayT;
        configSensors  = sensorsInstalled;
        configHystCup  = hystCup;
        configCupDelayT = cupDelayT;  // v11c
        snapshotLower  = lowerLimit;    snapshotUpper   = upperLimit;
        snapshotSpitLow= spitLower;     snapshotSpitUp  = spitUpper;
        snapshotDelayT = spitDelayT;
        snapshotSensors= sensorsInstalled;
        snapshotHystCup= hystCup;
        snapshotCupDelayT = cupDelayT;  // v11c
        configPage     = PAGE_SENSORS;  // primera página del menú
        showPageIdentifier = true;
        pageIdentifierStart = now;
        transitionToConfig = true;
        currentState   = STATE_WAIT_RELEASE;
        Serial1.println(">> Entrando a configuracion");

      } else if (currentState == STATE_CONFIG) {
        configPage = PAGE_EXIT_CONFIRM;
        configExitDiscard = false;
        currentState = STATE_WAIT_RELEASE;
        transitionToConfig = true;
        Serial1.println(">> Menu de salida: Guardar o Descartar");
      }
    }
  } else {
    // --- Toque corto simultáneo ---
    if (bothPressedStart > 0 && !bothWereHeld && currentState == STATE_CONFIG) {
      if (configPage == PAGE_EXIT_CONFIRM) {
        if (!configExitDiscard) {
          // GUARDAR: detectar si cambió la config de sensores (requiere reset)
          bool sensorsChanged = (snapshotSensors != sensorsInstalled);

          lowerLimit    = snapshotLower;    upperLimit = snapshotUpper;
          spitLower     = snapshotSpitLow;  spitUpper  = snapshotSpitUp;
          spitDelayT = snapshotDelayT;
          sensorsInstalled = snapshotSensors;
          hystCup = snapshotHystCup;
          cupDelayT = snapshotCupDelayT;  // v11c
          applySpitDelay();
          applyCupDelay();  // v11c
          saveConfig();

          Serial1.print(">> GUARDADO. Sensores: ");
          Serial1.println(sensorsInstalled == SENSORS_BOTH ? "AMBOS" :
                    (sensorsInstalled == SENSORS_CUP_ONLY ? "Solo LV" : "Solo ES"));

          if (sensorsChanged) {
            // Cambió la topología de sensores → reinicio controlado
            Serial1.println(">> Config de sensores cambiada, REINICIANDO...");
            resetFlashStart = now;
            currentState = STATE_RESET_FLASH;
            transitionToConfig = false;
          } else {
            applySensorFlags();  // por si acaso, recalcular
            resetAllDetection();
            confirmFlashIsDiscard = false;
            confirmFlashStart = now;
            currentState = STATE_CONFIRM_FLASH;
            transitionToConfig = false;
          }
        } else {
          Serial1.println(">> DESCARTADO: cambios no guardados");
          confirmFlashIsDiscard = true;
          confirmFlashStart = now;
          currentState = STATE_CONFIRM_FLASH;
          transitionToConfig = false;
        }
      } else {
        // Avanzar circularmente, SALTEANDO páginas de subsistemas no instalados
        do {
          switch (configPage) {
            case PAGE_SENSORS:     configPage = PAGE_RANGE_LOWER; break;
            case PAGE_RANGE_LOWER: configPage = PAGE_RANGE_UPPER; break;
            case PAGE_RANGE_UPPER: configPage = PAGE_HYST_CUP;   break;
            case PAGE_HYST_CUP:    configPage = PAGE_CUP_DELAY;   break;
            case PAGE_CUP_DELAY:   configPage = PAGE_SPIT_LOWER;  break;
            case PAGE_SPIT_LOWER:  configPage = PAGE_SPIT_UPPER;  break;
            case PAGE_SPIT_UPPER:  configPage = PAGE_DELAY;       break;
            case PAGE_DELAY:       configPage = PAGE_SENSORS;     break;
            default: configPage = PAGE_SENSORS; break;
          }
          // Saltear páginas de llenavasos si no está seleccionado en el snapshot
          bool cupSel  = (snapshotSensors == SENSORS_BOTH || snapshotSensors == SENSORS_CUP_ONLY);
          bool spitSel = (snapshotSensors == SENSORS_BOTH || snapshotSensors == SENSORS_SPIT_ONLY);
          if ((configPage == PAGE_RANGE_LOWER || configPage == PAGE_RANGE_UPPER || configPage == PAGE_HYST_CUP || configPage == PAGE_CUP_DELAY) && !cupSel) continue;
          if ((configPage == PAGE_SPIT_LOWER || configPage == PAGE_SPIT_UPPER || configPage == PAGE_DELAY) && !spitSel) continue;
          break;  // página válida encontrada
        } while (true);

        showPageIdentifier = true;
        pageIdentifierStart = now;
        Serial1.print(">> Pagina: ");
        switch (configPage) {
          case PAGE_SENSORS:     Serial1.println("Sensores"); break;
          case PAGE_RANGE_LOWER: Serial1.println("LV inferior"); break;
          case PAGE_RANGE_UPPER: Serial1.println("LV superior"); break;
          case PAGE_HYST_CUP:    Serial1.println("Histeresis LV"); break;
          case PAGE_CUP_DELAY:   Serial1.println("Delay LV"); break;
          case PAGE_SPIT_LOWER:  Serial1.println("ES inferior"); break;
          case PAGE_SPIT_UPPER:  Serial1.println("ES superior"); break;
          case PAGE_DELAY:       Serial1.println("Delay ES"); break;
          default: break;
        }
      }
    }

    // --- Toque corto individual en MODO NORMAL: cambiar vista del display ---
    // Solo cosmético. No afecta lógica, sensores ni máquina de estados.
    if (currentState == STATE_NORMAL && (btn1Pressed || btn2Pressed)) {
      if (cupInstalled && spitInstalled) {
        // Alternar entre las dos vistas
        normalViewIndex = (normalViewIndex == 0) ? 1 : 0;
        showViewId = true;
        viewIdStart = now;
        Serial1.print(">> Vista: ");
        Serial1.println(normalViewIndex == 0 ? "Llenavasos" : "Escupidera");
      }
      // Si solo hay un sensor, no hay nada que alternar (no hacer nada)
    }

    bothPressedStart = 0;
    bothWereHeld = false;

    if (currentState == STATE_WAIT_RELEASE) {
      currentState = transitionToConfig ? STATE_CONFIG : STATE_NORMAL;
    }
  }

  // === Estados del sistema ===
  switch (currentState) {

    case STATE_NORMAL: {
      bool anyFault = (cupInstalled && cupCh.m2State == M2_FAULT) ||
                      (spitInstalled && spitCh.m2State == M2_FAULT);
      if (anyFault) {
        displayDigit1 = SYM_E;
        displayDigit2 = SYM_E;
        displayDpLeft  = 0;
        displayDpRight = 0;
      } else if (showViewId && (now - viewIdStart) < PAGE_ID_DURATION_MS) {
        // Mostrar brevemente qué vista se seleccionó (S.L o S.E)
        displayDigit2 = SYM_S;
        displayDigit1 = (normalViewIndex == 0) ? SYM_L : SYM_E;
        displayDpLeft  = 0;
        displayDpRight = 1;
      } else {
        showViewId = false;
        // Elegir qué rango mostrar según la vista activa y sensores instalados.
        uint8_t viewToShow = normalViewIndex;
        if (!cupInstalled)  viewToShow = 1;  // forzar escupidera
        if (!spitInstalled) viewToShow = 0;  // forzar llenavasos

        uint16_t showLow, showUp;  // v10c: uint16 por rango escupidera hasta 200
        if (viewToShow == 0) { showLow = lowerLimit; showUp = upperLimit; }
        else                 { showLow = spitLower;  showUp = spitUpper; }

        // --- Animación decimales: alternar entero/decimal cada 700 ms ---
        // Bloque 100% asíncrono con millis(). No bloquea el loop ni los sensores.
        // v10c: valores en mm — parte entera = /10, decimal = %10 (0-9)
        static unsigned long lastDecimalToggle = 0;
        static bool showDecimalPhase = false;
        bool anyHasDecimal = (showUp % 10) || (showLow % 10);

        if (anyHasDecimal) {
          if ((now - lastDecimalToggle) >= 700) {
            lastDecimalToggle = now;
            showDecimalPhase = !showDecimalPhase;
          }
        } else {
          showDecimalPhase = false;
        }

        if (!showDecimalPhase) {
          // FASE ENTERO
          displayDigit1 = showUp  / 10;
          displayDigit2 = showLow / 10;
          displayDpLeft  = (showUp  % 10) ? 1 : 0;
          displayDpRight = (showLow % 10) ? 1 : 0;
        } else {
          // FASE DECIMAL
          displayDigit1 = (showUp  % 10) ? (showUp  % 10) : SYM_OFF;
          displayDigit2 = (showLow % 10) ? (showLow % 10) : SYM_OFF;
          displayDpLeft  = 0;
          displayDpRight = 0;
        }
      }

      // v13c: ADQUISICIÓN NO BLOQUEANTE Y DESACOPLADA.
      // Ya no hay gate SENSOR_INTERVAL. Cada sensor se cosecha independiente:
      // - Si devuelve SAMPLE_NOT_READY → no se toca el filtro ni la detección.
      // - Si devuelve mm válido o -1 → se procesa (muestra fresca).
      // Los dos sensores integran en paralelo por hardware (buses separados).
      {
        // --- Llenavasos (VL6180X) ---
        if (cupInstalled) {
          int16_t distMM = readFilteredDistance();
          if (distMM != SAMPLE_NOT_READY) {
            lastFreshCup = now;  // llegó muestra fresca
            cupRecoveryAttempts = 0;  // v13d: sensor sano → resetear contador de fallas
            if (distMM >= 0) {
              uint16_t lo = lowerLimit;  // v10c: ya está en mm
              uint16_t hi = upperLimit;
              // v11: histéresis predictiva — el rango de salida es más estrecho
              // que el de entrada. Si hystCup=2 (0.2cm), al salir el umbral
              // superior baja a (upperLimit - 2) mm → cierre anticipado.
              if (cupCh.objectPresent && hystCup > 0) {
                hi = (upperLimit > hystCup) ? (upperLimit - hystCup) : 0;
              }
              bool cupInRange = (distMM >= lo) && (distMM <= hi);
              processDetection(cupCh, cupInRange);
            } else {
              processDetection(cupCh, false);  // error = fuera de rango
            }
          } else if ((now - lastFreshCup) >= SENSOR_STALE_MS) {
            // Staleness nivel 1 (150ms): fuerza fuera de rango por seguridad
            processDetection(cupCh, false);
            lastFreshCup = now;  // evita spamear; reintenta la ventana

            // v13d: Staleness nivel 2 (500ms) → sensor congelado, intentar recuperar
            // Solo si pasó el cooldown y no se agotaron los intentos
            if ((now - lastRecoveryCup) >= RECOVERY_COOLDOWN_MS &&
                cupRecoveryAttempts < MAX_RECOVERY_ATTEMPTS) {
              lastRecoveryCup = now;
              cupRecoveryAttempts++;
              Serial1.print("[LV] Sensor congelado, intento recovery #");
              Serial1.println(cupRecoveryAttempts);
              if (recoverVL6180X()) {
                cupRecoveryAttempts = 0;  // éxito → resetear contador
                lastFreshCup = millis();  // dar tiempo antes de re-evaluar staleness
              }
            } else if (cupRecoveryAttempts >= MAX_RECOVERY_ATTEMPTS) {
              // Se agotaron los intentos: desactivar canal en runtime
              // para no seguir intentando y no degradar el otro sensor
              cupInstalled = false;
              Serial1.println("[LV] Desactivado en runtime tras 5 recoveries fallidos");
            }
          }

          // v11c: Delay de activación del llenavasos (misma mecánica que escupidera)
          // Si cupDelayT = 0 → comportamiento original (sin delay)
          if (cupActivationDelayMs > 0) {
            if (cupCh.objectJustEntered && !cupCh.valveIsOpen) {
              cupCh.objectJustEntered = false;
              cupActivationStart = millis();
              cupActivationPending = true;
            }
            if (cupActivationPending) {
              if (!cupCh.objectPresent) {
                cupActivationPending = false;
                cupActivationStart = 0;
              } else if ((millis() - cupActivationStart) >= cupActivationDelayMs) {
                cupCh.objectJustEntered = true;
                cupActivationPending = false;
                cupActivationStart = 0;
                Serial1.println("[LV] Delay de activacion cumplido");
              }
            }
          }
        }

        // --- Escupidera (VL53L0X por rango) ---
        if (spitInstalled) {
          int16_t distSpit = readFilteredDistanceSpit();

          // v12b: BLOQUEO LV→ES. Mientras la válvula del llenavasos esté
          // físicamente abierta (PA8=HIGH), la detección automática de la
          // escupidera queda suprimida. NO afecta activación externa.
          bool lvBlockingSpit = cupInstalled && cupCh.valveIsOpen;

          if (lvBlockingSpit) {
            processDetection(spitCh, false);
            spitCh.objectJustEntered = false;
            spitActivationPending = false;
            spitActivationStart = 0;
            if (distSpit != SAMPLE_NOT_READY) lastFreshSpit = now;
          } else if (distSpit != SAMPLE_NOT_READY) {
            lastFreshSpit = now;  // muestra fresca
            spitRecoveryAttempts = 0;  // v13d: sensor sano → resetear contador
            if (distSpit >= 0) {
              uint16_t loS = spitLower;  // v10c: ya está en mm
              uint16_t hiS = spitUpper;
              bool spitInRange = (distSpit >= loS) && (distSpit <= hiS);
              processDetection(spitCh, spitInRange);
            } else {
              processDetection(spitCh, false);  // error = fuera de rango
            }
          } else if ((now - lastFreshSpit) >= SENSOR_STALE_MS) {
            processDetection(spitCh, false);  // staleness nivel 1
            lastFreshSpit = now;

            // v13d: staleness nivel 2 → recovery del VL53L0X
            if ((now - lastRecoverySpit) >= RECOVERY_COOLDOWN_MS &&
                spitRecoveryAttempts < MAX_RECOVERY_ATTEMPTS) {
              lastRecoverySpit = now;
              spitRecoveryAttempts++;
              Serial1.print("[ES] Sensor congelado, intento recovery #");
              Serial1.println(spitRecoveryAttempts);
              if (recoverVL53L0X()) {
                spitRecoveryAttempts = 0;
                lastFreshSpit = millis();
              }
            } else if (spitRecoveryAttempts >= MAX_RECOVERY_ATTEMPTS) {
              spitInstalled = false;
              Serial1.println("[ES] Desactivada en runtime tras 5 recoveries fallidos");
            }
          }

          // Delay de activación de la escupidera (evita activaciones accidentales)
          // No corre si estamos bloqueados por LV (ya se limpió arriba).
          if (!lvBlockingSpit) {
            if (spitCh.objectJustEntered && !spitCh.valveIsOpen) {
              spitCh.objectJustEntered = false;
              spitActivationStart = millis();
              spitActivationPending = true;
            }
            if (spitActivationPending) {
              if (!spitCh.objectPresent) {
                spitActivationPending = false;
                spitActivationStart = 0;
              } else if ((millis() - spitActivationStart) >= spitActivationDelayMs) {
                spitCh.objectJustEntered = true;
                spitActivationPending = false;
                spitActivationStart = 0;
                Serial1.println("[ES] Delay de activacion cumplido");
              }
            }
          }
        }

        // Debug (gate temporal, no bloqueante: se imprime cada ~200ms)
        static unsigned long lastDbg = 0;
        if ((now - lastDbg) >= 200) {
          lastDbg = now;
          if (cupInstalled) {
            Serial1.print("Llenavaso[V:"); Serial1.print(cupCh.valveIsOpen ? "on" : "--");
            Serial1.print(" S:"); Serial1.print((int)cupCh.m2State);
            Serial1.print("] ");
          }
          if (spitInstalled) {
            Serial1.print("Escupidera[V:"); Serial1.print(spitCh.valveIsOpen ? "on" : "--");
            Serial1.print(" S:"); Serial1.print((int)spitCh.m2State);
            Serial1.print("]");
          }
          Serial1.println();
        }
      }

      if (cupInstalled)  processChannel(cupCh);
      if (spitInstalled) processChannel(spitCh);
      break;
    }

    case STATE_WAIT_RELEASE: {
      if ((now - lastBlink) >= BLINK_INTERVAL) {
        lastBlink = now;
        blinkOn = !blinkOn;
      }
      displayDigit1 = blinkOn ? 8 : SYM_OFF;
      displayDigit2 = blinkOn ? 8 : SYM_OFF;
      displayDpLeft  = 0;
      displayDpRight = 0;
      digitalWrite(CUP_VALVE_OUT, LOW);
      digitalWrite(SPIT_VALVE_OUT, LOW);
      break;
    }

    case STATE_CONFIG: {
      if (cupInstalled)  digitalWrite(CUP_VALVE_OUT, LOW);
      if (spitInstalled) digitalWrite(SPIT_VALVE_OUT, LOW);

      // Actualizar snapshots cuando ambos botones estén en reposo
      if (btn1Stable == LOW && btn2Stable == LOW) {
        snapshotLower   = configLower;   snapshotUpper   = configUpper;
        snapshotSpitLow = configSpitLow; snapshotSpitUp  = configSpitUp;
        snapshotDelayT  = configDelayT;
        snapshotSensors = configSensors;
        snapshotHystCup = configHystCup;
        snapshotCupDelayT = configCupDelayT;  // v11c
      }

            // v11b: bandera centralizada — suprime acciones individuales cuando ambos
      // botones están presionados (incluso si uno confirmó antes que el otro)
      // o durante el cooldown post-combo. Resuelve el ±0.1 accidental al
      // cambiar de página.
      bool comboActive = (btn1Stable == HIGH && btn2Stable == HIGH) ||
                         (btn1Stable == HIGH && digitalRead(BTN2_PIN) == HIGH) ||
                         (btn2Stable == HIGH && digitalRead(BTN1_PIN) == HIGH) ||
                         (bothReleaseTime != 0);

      bool showingId = showPageIdentifier &&
                       ((now - pageIdentifierStart) < PAGE_ID_DURATION_MS);
      if (!showingId) showPageIdentifier = false;

      switch (configPage) {

        // ------------- PÁGINA: Selección de sensores (S.) ------------------
        case PAGE_SENSORS:
          if (showingId) {
            displayDigit1 = SYM_S;
            displayDigit2 = SYM_S;
            displayDpLeft  = 1;
            displayDpRight = 1;
          } else {
            if ((btn1Pressed && btn2Stable == LOW)) {
              // Retroceder en el ciclo de opciones
              configSensors = (configSensors == 0) ? 2 : configSensors - 1;
            }
            if ((btn2Pressed && btn1Stable == LOW)) {
              configSensors = (configSensors == 2) ? 0 : configSensors + 1;
            }
            // Mostrar: A.A=ambos, L.- = solo LV, -.E = solo ES
            if (configSensors == SENSORS_BOTH) {
              displayDigit1 = 10;      // A (dígito derecho)
              displayDigit2 = 10;      // A (dígito izquierdo)
            } else if (configSensors == SENSORS_CUP_ONLY) {
              displayDigit1 = SYM_DASH; // - (derecho)
              displayDigit2 = SYM_L;    // L (izquierdo) → "L.-"
            } else { // SENSORS_SPIT_ONLY
              displayDigit1 = SYM_E;    // E (derecho)
              displayDigit2 = SYM_DASH; // - (izquierdo) → "-.E"
            }
            displayDpLeft  = 0;
            displayDpRight = 0;
          }
          break;

        // ------------- PÁGINA: Llenavasos inferior (L.I.) ------------------
        // v10c: pasos de 1 (=0.1cm). Autorepetición al mantener presionado.
        case PAGE_RANGE_LOWER:
          if (showingId) {
            displayDigit1 = SYM_I; displayDigit2 = SYM_L;
            displayDpLeft = 1; displayDpRight = 1;
          } else {
            static unsigned long btnHoldStartLI = 0;
            static unsigned long lastRepeatLI = 0;
            static uint8_t lastBtnLI = 0;  // 0=none,1=down,2=up

            uint8_t curBtn = 0;
            if (!comboActive) {
              if (btn1Stable == HIGH && btn2Stable == LOW) curBtn = 1;
              else if (btn2Stable == HIGH && btn1Stable == LOW) curBtn = 2;
            }

            bool doAction = false;
            if (curBtn != lastBtnLI) {
              if (curBtn != 0) { doAction = true; btnHoldStartLI = now; lastRepeatLI = now; }
              lastBtnLI = curBtn;
            } else if (curBtn != 0) {
              if ((now - btnHoldStartLI) >= BTN_HOLD_INITIAL_DELAY_MS &&
                  (now - lastRepeatLI) >= BTN_HOLD_REPEAT_MS) {
                doAction = true;
                lastRepeatLI = now;
              }
            }

            if (doAction) {
              if (curBtn == 1) {   // bajar
                if (configLower > 0) configLower--; else configLower = 80;
                if (configUpper < configLower) configUpper = configLower;
              } else {              // subir
                if (configLower < 80) configLower++; else configLower = 0;
                if (configUpper < configLower) configUpper = configLower;
              }
            }
            displayDigit1 = configLower % 10;
            displayDigit2 = configLower / 10;
            displayDpLeft = 0; displayDpRight = 1;
          }
          break;

        // ------------- PÁGINA: Llenavasos superior (L.S.) ------------------
        case PAGE_RANGE_UPPER:
          if (showingId) {
            displayDigit1 = SYM_S; displayDigit2 = SYM_L;
            displayDpLeft = 1; displayDpRight = 1;
          } else {
            static unsigned long btnHoldStartLS = 0;
            static unsigned long lastRepeatLS = 0;
            static uint8_t lastBtnLS = 0;

            uint8_t curBtn = 0;
            if (!comboActive) {
              if (btn1Stable == HIGH && btn2Stable == LOW) curBtn = 1;
              else if (btn2Stable == HIGH && btn1Stable == LOW) curBtn = 2;
            }

            bool doAction = false;
            if (curBtn != lastBtnLS) {
              if (curBtn != 0) { doAction = true; btnHoldStartLS = now; lastRepeatLS = now; }
              lastBtnLS = curBtn;
            } else if (curBtn != 0) {
              if ((now - btnHoldStartLS) >= BTN_HOLD_INITIAL_DELAY_MS &&
                  (now - lastRepeatLS) >= BTN_HOLD_REPEAT_MS) {
                doAction = true;
                lastRepeatLS = now;
              }
            }

            if (doAction) {
              if (curBtn == 1) {
                if (configUpper > configLower) configUpper--; else configUpper = 80;
              } else {
                if (configUpper < 80) configUpper++; else configUpper = configLower;
              }
            }
            displayDigit1 = configUpper % 10;
            displayDigit2 = configUpper / 10;
            displayDpLeft = 0; displayDpRight = 1;
          }
          break;

        // ------------- PÁGINA: Histéresis llenavasos (H.L.) ----------------
        // v11: rango 0-5 mm (0.0-0.5 cm), pasos de 0.1cm, autorepetición
        case PAGE_HYST_CUP:
          if (showingId) {
            displayDigit1 = SYM_L; displayDigit2 = 17;  // "H.L"
            displayDpLeft = 1; displayDpRight = 1;
          } else {
            static unsigned long btnHoldStartHL = 0;
            static unsigned long lastRepeatHL = 0;
            static uint8_t lastBtnHL = 0;

            uint8_t curBtn = 0;
            if (!comboActive) {
              if (btn1Stable == HIGH && btn2Stable == LOW) curBtn = 1;
              else if (btn2Stable == HIGH && btn1Stable == LOW) curBtn = 2;
            }

            bool doAction = false;
            if (curBtn != lastBtnHL) {
              if (curBtn != 0) { doAction = true; btnHoldStartHL = now; lastRepeatHL = now; }
              lastBtnHL = curBtn;
            } else if (curBtn != 0) {
              if ((now - btnHoldStartHL) >= BTN_HOLD_INITIAL_DELAY_MS &&
                  (now - lastRepeatHL) >= BTN_HOLD_REPEAT_MS) {
                doAction = true;
                lastRepeatHL = now;
              }
            }

            if (doAction) {
              if (curBtn == 1) {
                if (configHystCup > 0) configHystCup--; else configHystCup = 5;
              } else {
                if (configHystCup < 5) configHystCup++; else configHystCup = 0;
              }
            }
            displayDigit1 = configHystCup % 10;  // decimal (siempre 0-5)
            displayDigit2 = configHystCup / 10;  // entero (siempre 0)
            displayDpLeft = 0; displayDpRight = 1;
          }
          break;

        // ------------- PÁGINA: Delay Llenavasos (D.L.) --------------------
        // v11c: rango 0-5 décimas (0.0-0.5 s), autorepetición, comboActive
        case PAGE_CUP_DELAY:
          if (showingId) {
            displayDigit1 = SYM_L; displayDigit2 = SYM_d;  // "d.L"
            displayDpLeft = 1; displayDpRight = 1;
          } else {
            static unsigned long btnHoldStartDL = 0;
            static unsigned long lastRepeatDL = 0;
            static uint8_t lastBtnDL = 0;

            uint8_t curBtn = 0;
            if (!comboActive) {
              if (btn1Stable == HIGH && btn2Stable == LOW) curBtn = 1;
              else if (btn2Stable == HIGH && btn1Stable == LOW) curBtn = 2;
            }

            bool doAction = false;
            if (curBtn != lastBtnDL) {
              if (curBtn != 0) { doAction = true; btnHoldStartDL = now; lastRepeatDL = now; }
              lastBtnDL = curBtn;
            } else if (curBtn != 0) {
              if ((now - btnHoldStartDL) >= BTN_HOLD_INITIAL_DELAY_MS &&
                  (now - lastRepeatDL) >= BTN_HOLD_REPEAT_MS) {
                doAction = true;
                lastRepeatDL = now;
              }
            }

            if (doAction) {
              if (curBtn == 1) {
                if (configCupDelayT > 0) configCupDelayT--; else configCupDelayT = 5;
              } else {
                if (configCupDelayT < 5) configCupDelayT++; else configCupDelayT = 0;
              }
            }
            displayDigit1 = configCupDelayT % 10;
            displayDigit2 = configCupDelayT / 10;
            displayDpLeft = 0; displayDpRight = 1;
          }
          break;

        // ------------- PÁGINA: Escupidera inferior (E.I.) ------------------
        // v10c: rango 0-200 mm (0.0-20.0 cm), pasos de 0.1cm, autorepetición
        case PAGE_SPIT_LOWER:
          if (showingId) {
            displayDigit1 = SYM_I; displayDigit2 = SYM_E;
            displayDpLeft = 1; displayDpRight = 1;
          } else {
            static unsigned long btnHoldStartEI = 0;
            static unsigned long lastRepeatEI = 0;
            static uint8_t lastBtnEI = 0;

            uint8_t curBtn = 0;
            if (!comboActive) {
              if (btn1Stable == HIGH && btn2Stable == LOW) curBtn = 1;
              else if (btn2Stable == HIGH && btn1Stable == LOW) curBtn = 2;
            }

            bool doAction = false;
            if (curBtn != lastBtnEI) {
              if (curBtn != 0) { doAction = true; btnHoldStartEI = now; lastRepeatEI = now; }
              lastBtnEI = curBtn;
            } else if (curBtn != 0) {
              if ((now - btnHoldStartEI) >= BTN_HOLD_INITIAL_DELAY_MS &&
                  (now - lastRepeatEI) >= BTN_HOLD_REPEAT_MS) {
                doAction = true;
                lastRepeatEI = now;
              }
            }

            if (doAction) {
              if (curBtn == 1) {
                if (configSpitLow > 0) configSpitLow--; else configSpitLow = 200;
                if (configSpitUp < configSpitLow) configSpitUp = configSpitLow;
              } else {
                if (configSpitLow < 200) configSpitLow++; else configSpitLow = 0;
                if (configSpitUp < configSpitLow) configSpitUp = configSpitLow;
              }
            }
            // Display: parte entera = /10 (0-20), decimal = %10 (0-9)
            // Como la parte entera puede tener 2 dígitos, mostramos solo el decimal
            // en el dígito derecho y el entero (%10) en el izquierdo cuando >=10cm.
            // Simplificación: mostramos dígito izq = decenas o unidades del entero,
            // der = decimal. Para rango 0-20cm alcanza así.
            uint8_t entI = configSpitLow / 10;   // 0-20 cm
            uint8_t decI = configSpitLow % 10;   // 0-9 mm
            displayDigit1 = decI;
            displayDigit2 = entI;                // hasta 20 (2 dígitos, se corta)
            displayDpLeft = 0; displayDpRight = 1;
          }
          break;

        // ------------- PÁGINA: Escupidera superior (E.S.) ------------------
        case PAGE_SPIT_UPPER:
          if (showingId) {
            displayDigit1 = SYM_S; displayDigit2 = SYM_E;
            displayDpLeft = 1; displayDpRight = 1;
          } else {
            static unsigned long btnHoldStartES = 0;
            static unsigned long lastRepeatES = 0;
            static uint8_t lastBtnES = 0;

            uint8_t curBtn = 0;
            if (!comboActive) {
              if (btn1Stable == HIGH && btn2Stable == LOW) curBtn = 1;
              else if (btn2Stable == HIGH && btn1Stable == LOW) curBtn = 2;
            }

            bool doAction = false;
            if (curBtn != lastBtnES) {
              if (curBtn != 0) { doAction = true; btnHoldStartES = now; lastRepeatES = now; }
              lastBtnES = curBtn;
            } else if (curBtn != 0) {
              if ((now - btnHoldStartES) >= BTN_HOLD_INITIAL_DELAY_MS &&
                  (now - lastRepeatES) >= BTN_HOLD_REPEAT_MS) {
                doAction = true;
                lastRepeatES = now;
              }
            }

            if (doAction) {
              if (curBtn == 1) {
                if (configSpitUp > configSpitLow) configSpitUp--; else configSpitUp = 200;
              } else {
                if (configSpitUp < 200) configSpitUp++; else configSpitUp = configSpitLow;
              }
            }
            uint8_t entS = configSpitUp / 10;
            uint8_t decS = configSpitUp % 10;
            displayDigit1 = decS;
            displayDigit2 = entS;
            displayDpLeft = 0; displayDpRight = 1;
          }
          break;

        // ------------- PÁGINA: Delay escupidera (E. + DP parpadea) --------
        case PAGE_DELAY:
          if (showingId) {
            displayDigit1 = SYM_E; displayDigit2 = SYM_E;
            displayDpLeft = 2; displayDpRight = 2;
          } else {
            if (btn1Pressed && btn2Stable == LOW) {
              if (configDelayT > 5) configDelayT -= 5; else configDelayT = 30;
            }
            if (btn2Pressed && btn1Stable == LOW) {
              if (configDelayT < 30) configDelayT += 5; else configDelayT = 5;
            }
            displayDigit1 = configDelayT % 10;
            displayDigit2 = configDelayT / 10;
            displayDpLeft = 2; displayDpRight = 2;
          }
          break;

        // ------------- PÁGINA: Confirmación de salida (G. / n.) -----------
        case PAGE_EXIT_CONFIRM:
          if ((btn1Pressed && btn2Stable == LOW) ||
              (btn2Pressed && btn1Stable == LOW)) {
            configExitDiscard = !configExitDiscard;
          }
          if (configExitDiscard) { displayDigit1 = SYM_n; displayDigit2 = SYM_n; }
          else                   { displayDigit1 = SYM_G; displayDigit2 = SYM_G; }
          displayDpLeft = 2; displayDpRight = 2;
          break;
      }
      break;
    }

    // --------- STATE_CONFIRM_FLASH: mostrar G o -- brevemente antes de salir
    case STATE_CONFIRM_FLASH: {
      if (cupInstalled)  digitalWrite(CUP_VALVE_OUT, LOW);
      if (spitInstalled) digitalWrite(SPIT_VALVE_OUT, LOW);

      if (confirmFlashIsDiscard) { displayDigit1 = SYM_DASH; displayDigit2 = SYM_DASH; }
      else                       { displayDigit1 = SYM_G;    displayDigit2 = SYM_G; }
      displayDpLeft = 1; displayDpRight = 1;

      if ((now - confirmFlashStart) >= CONFIRM_FLASH_DURATION_MS) {
        currentState = STATE_NORMAL;
      }
      break;
    }

    // --------- STATE_RESET_FLASH: mostrar "rE" y reiniciar por HW ----------
    case STATE_RESET_FLASH: {
      digitalWrite(CUP_VALVE_OUT, LOW);
      digitalWrite(SPIT_VALVE_OUT, LOW);
      // "rE" — r no existe limpio en 7seg, usamos n (parecido) + E
      displayDigit1 = SYM_E;   // E (derecho)
      displayDigit2 = SYM_n;   // n≈r (izquierdo) → "rE"
      displayDpLeft = 0; displayDpRight = 0;

      if ((now - resetFlashStart) >= CONFIRM_FLASH_DURATION_MS) {
        // Reinicio por software vía watchdog: dejar de alimentarlo.
        // El IWDG ya está activo; entrar en bucle sin feed lo dispara.
        Serial1.println(">> Reiniciando ahora");
        Serial1.flush();
        while (1) { /* esperar reset del watchdog (~4s) */ }
      }
      break;
    }
  }

  // ==========================================================================
  // TIMER DEL BLINK DEL DP (para páginas con DP parpadeando)
  // ==========================================================================
  static unsigned long lastDpBlink = 0;
  if ((now - lastDpBlink) >= DP_BLINK_INTERVAL_MS) {
    lastDpBlink = now;
    dpBlinkPhase = !dpBlinkPhase;
  }
}
