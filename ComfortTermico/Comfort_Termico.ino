/**
 * @file ControlPMV_RFID.ino
 * @brief Sistema con máquina de estados para control PMV/alarma/usuario con RFID, teclado y LCD.
 *
 * @details
 * - Plataforma: Arduino (C++).
 * - Dispositivos: DHT11, NTC (termistor), LCD 16x2 (LiquidCrystal), teclado 4x4 (Keypad),
 *   RFID MFRC522 (SPI), Servo, buzzer, relé, LEDs (RGB), sensor IR, EEPROM.
 * - Arquitectura: Máquina de estados (StateMachineLib) + tareas periódicas (AsyncTaskLib).
 */

#include "StateMachineLib.h"
#include <AsyncTaskLib.h>
#include <Keypad.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <Servo.h>
#include <MFRC522.h>
#include <SPI.h>
#include <math.h>
#include <EEPROM.h>
#include <string.h>

/* ==========================
 *  Definiciones de hardware
 * ========================== */

/**
 * @name Pines de E/S
 * @{
 */
/// Pines de salida/entrada
#define PIN_LED_VERDE  28  /**< LED indicador verde (salida). */
#define PIN_LED_ROJO   27  /**< LED indicador rojo (salida).  */
#define PIN_LED_AZUL   29  /**< LED indicador azul (salida).  */
#define PIN_BOTON      49  /**< Botón de usuario. */
#define PIN_DHT        22  /**< Pin del sensor DHT11. */
#define PIN_RELE       25  /**< Pin de control del relé. */
#define PIN_SERVO      26  /**< Pin de control del servo. */
#define PIN_RFID_RST    9  /**< Pin RST del MFRC522. */
#define PIN_RFID_SS    53  /**< Pin SS (SDA) del MFRC522 (SPI). */
#define PIN_BUZZER      7  /**< Pin del buzzer. */
#define PIN_IR         23  /**< Pin del sensor infrarrojo. */
/**@}*/

/**
 * @brief Tipo de sensor DHT en uso.
 */
#define DHT_TIPO DHT11

/**
 * @name Parámetros del NTC y ADC
 * @{
 */
#define PIN_ANALOGICO A0      /**< Pin analógico para el NTC. */
#define NTC_BETA      3950.0  /**< Coeficiente beta del NTC. */
#define NTC_R_SERIE      10.0 /**< Resistencia serie (kΩ). */
#define NTC_R0           10.0 /**< Resistencia nominal del NTC a T0. */
#define NTC_T0        298.15  /**< Temperatura nominal (K) correspondiente a R0. */
/**@}*/

/* ==========================
 *  Objetos globales HW
 * ========================== */

DHT   sensorDHT(PIN_DHT, DHT_TIPO);              /**< Objeto del sensor DHT. */
Servo servoPuerta;                               /**< Objeto del servo. */
MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);         /**< Lector RFID MFRC522. */

/// LCD
const int LCD_RS = 12, LCD_EN = 11, LCD_D4 = 5, LCD_D5 = 4, LCD_D6 = 3, LCD_D7 = 2;
LiquidCrystal pantalla(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7); /**< Pantalla LCD 16x2. */

/// Teclado 4x4
const byte TECLADO_FILAS = 4;  /**< Número de filas del teclado. */
const byte TECLADO_COLS  = 4;  /**< Número de columnas del teclado. */

char mapaTeclas[TECLADO_FILAS][TECLADO_COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

byte pinesFilas[TECLADO_FILAS] = { 30, 31, 32, 33 };
byte pinesCols [TECLADO_COLS ] = { 34, 35, 36, 37 };

Keypad teclado = Keypad(makeKeymap(mapaTeclas), pinesFilas, pinesCols, TECLADO_FILAS, TECLADO_COLS); /**< Teclado matricial. */

/// Clave MIFARE default
MFRC522::MIFARE_Key mifareKey; /**< Clave MIFARE de 6 bytes para autenticación por defecto. */

/* ==========================
 *  Estructuras y constantes
 * ========================== */

/**
 * @brief Resultado PMV (Predicted Mean Vote).
 */
struct PMVResult { float pmv; };  /**< Valor PMV en el rango [-3, 3]. */

/**
 * @brief Registro de perfil RFID en EEPROM.
 * @details
 * - `valid` = 0xA5 indica ocupado.
 * - `uid` contiene el UID de 4 bytes.
 * - `nombre` (bloque 4) y `rango` (bloque 5) contienen cadenas ASCII.
 */
struct RfidSlot {
  byte  valid;       ///< 0xA5 = ocupado
  byte  uid[4];      ///< UID tarjeta
  char  nombre[16];  ///< nombre (bloque 4)
  char  rango[8];    ///< rango temp (bloque 5, ej "13-25")
};

static const int RFID_MAX_SLOTS   = 16;               /**< Máximo de slots almacenables. */
static const int RFID_EE_BASE     = 0;                /**< Dirección base en EEPROM. */
static const int RFID_SLOT_BYTES  = sizeof(RfidSlot); /**< Tamaño de un slot en EEPROM. */

/**
 * @brief Devuelve la dirección base en EEPROM para un slot.
 * @param idx Índice del slot (0..RFID_MAX_SLOTS-1).
 * @return Dirección absoluta (entero) para operaciones EEPROM.
 */
static inline int rfid_dir_slot(int idx){ return RFID_EE_BASE + idx * RFID_SLOT_BYTES; }

/// UIDs de ejemplo (se conservan)
byte uidTarjeta[4] = {0x43, 0x89, 0x4F, 0x2E};  /**< UID de tarjeta de prueba. */
byte uidLlavero[4] = {0x56, 0x34, 0xDA, 0x73}; /**< UID de llavero de prueba. */

/* ==========================
 *  Máquina de estados
 * ========================== */

/**
 * @enum State
 * @brief Estados de la FSM (no cambiar nombres).
 */
enum State { inicio, Config, Bloqueado, Alarma, Monitor, pmv_alto, pmv_bajo };

/**
 * @enum Input
 * @brief Eventos/entradas que disparan transiciones.
 */
enum Input {
  tiempo, boton, Unknown, pmv, temperatura, keypadInput, keypadBlock, alarmaTemp, sensorIR
};

/// FSM y entrada actual
StateMachine fsm(7, 13);  /**< Máquina de estados finita. */
Input evento;             /**< Último evento/entrada para la FSM. */

/* ==========================
 *  Variables de control
 * ========================== */

String claveSistema = "1234";  /**< Clave requerida para desbloqueo. */
String entradaClave = "";      /**< Buffer de clave digitada. */

float pmvActual = 0.0f;        /**< PMV calculado más reciente. */
int   reintentosTempAlta = 0;  /**< Contador de intentos con PMV alto. */
float tempActual = 0.0f;       /**< Temperatura de aire actual. */
bool  flagSalirPMVAlto = false;/**< Bandera para salida de estado PMV alto. */

/// IR debounce
unsigned long irUltimo = 0;                 /**< Marca de tiempo para anti-rebote del IR. */
const unsigned long IR_DEBOUNCE_MS = 500;   /**< Tiempo de debounce en ms. */
bool irArmado = true;                        /**< Arma/desarma la detección IR. */

/* ==========================
 *  Tareas asíncronas
 * ========================== */

AsyncTask tareaCfg(5000, true, [](){ evento = tiempo; });    /**< Tarea periódica para estado Config. */
AsyncTask tareaMon(7000, true, [](){ evento = tiempo; });    /**< Tarea periódica para estado Monitor. */
AsyncTask tareaPMVAlto(5000, true, [](){ evento = tiempo; });/**< Tarea periódica para PMV alto. */
AsyncTask tareaPMVBajo(3000, true, [](){ evento = tiempo; });/**< Tarea periódica para PMV bajo. */

AsyncTask tareaAzulOn (300, false, [](){ digitalWrite(PIN_LED_AZUL, HIGH); }); /**< Enciende LED azul tras 300 ms. */
AsyncTask tareaAzulOff(400, false, [](){ digitalWrite(PIN_LED_AZUL, LOW ); }); /**< Apaga LED azul tras 400 ms.   */

AsyncTask tareaVerdeOn (200, false, [](){ digitalWrite(PIN_LED_VERDE, HIGH); }); /**< Enciende LED verde tras 200 ms. */
AsyncTask tareaVerdeOff(300, false, [](){ digitalWrite(PIN_LED_VERDE, LOW ); }); /**< Apaga LED verde tras 300 ms.    */

AsyncTask tareaRojoOn (100, false, [](){ digitalWrite(PIN_LED_ROJO, HIGH); }); /**< Enciende LED rojo tras 100 ms. */
AsyncTask tareaRojoOff(300, false, [](){ digitalWrite(PIN_LED_ROJO, LOW ); });  /**< Apaga LED rojo tras 300 ms.    */

AsyncTask tareaRojoCortoOn (100, false, [](){ digitalWrite(PIN_LED_ROJO, HIGH); }); /**< Pulso corto de LED rojo (on).  */
AsyncTask tareaRojoCortoOff(500, false, [](){ digitalWrite(PIN_LED_ROJO, LOW ); });  /**< Pulso corto de LED rojo (off). */

AsyncTask tareaBuzzer(500, true, [](){
  static bool bz = false; bz = !bz;
  digitalWrite(PIN_BUZZER, bz ? HIGH : LOW);
}); /**< Parpadeo del buzzer cada 500 ms. */

/* ==========================
 *  Prototipos
 * ========================== */

/**
 * @brief Lee y resuelve el próximo evento de entrada para la FSM.
 * @return Un valor de la enumeración ::Input (o ::Unknown si no hay evento).
 */
int  leerEvento();

/**
 * @brief Configura todas las transiciones de la FSM y callbacks de entrada/salida.
 */
void configurarFSM();

/* Callbacks de estados (nombres preservados) */
void enteringInicio();   void leavingInicio();
void enteringConfig();   void leavingConfig();
void enteringBloqueado();void leavingBloqueado();
void enteringAlarma();   void leavingAlarma();
void enteringMonitor();  void leavingMonitor();
void enteringPMVALTO();  void leavingPmvAlto();
void enteringPMVBAJO();  void leavingPmvBajo();

/* RFID/EEPROM helpers */
static bool  rfid_uid_igual(const byte a[4], const byte b[4]);
/**
 * @brief Copia una cadena Arduino a un buffer C fijo.
 * @param s     Cadena origen.
 * @param dst   Buffer destino (se garantiza terminación en @c '\\0').
 * @param maxLen Capacidad total del buffer @p dst (incluye el terminador).
 */
static void  rfid_copiar_str_fija(const String& s, char* dst, int maxLen);
/**
 * @brief Lee un registro de EEPROM.
 * @param slot Índice de slot [0..RFID_MAX_SLOTS-1].
 * @param out  Estructura destino.
 */
static void  rfid_eeprom_leer(int slot, RfidSlot &out);
/**
 * @brief Escribe un registro en EEPROM.
 * @param slot Índice de slot [0..RFID_MAX_SLOTS-1].
 * @param in   Estructura origen.
 */
static void  rfid_eeprom_grabar(int slot, const RfidSlot &in);
/**
 * @brief Busca un slot por UID.
 * @param uid UID de 4 bytes.
 * @return Índice del slot o -1 si no existe.
 */
static int   rfid_buscar_por_uid(const byte uid[4]);
/**
 * @brief Busca el primer slot libre.
 * @return Índice libre o -1 si no hay espacio.
 */
static int   rfid_buscar_libre();
/**
 * @brief Inicializa la tabla de perfiles si está vacía.
 */
static void  rfid_eeprom_init_si_vacia();
/**
 * @brief Guarda/actualiza un perfil RFID.
 * @param uid    UID de 4 bytes.
 * @param nombre Nombre del usuario.
 * @param rango  Rango preferido (ej. "13-25").
 * @return Slot utilizado (>=0). Si no hay libres, sobrescribe 0.
 */
static int   rfid_guardar_perfil(const byte uid[4], const String& nombre, const String& rango);
/**
 * @brief Imprime por Serial el estado de todos los slots.
 */
static void  rfid_print_estado();

void        rfid_leerPerfil();
/**
 * @brief Autentica un bloque MIFARE con clave A por defecto.
 * @param bloque Número de bloque a autenticar (0–63 según tarjeta).
 */
void        rfid_autenticarBloque(byte bloque);
/**
 * @brief Lee un bloque MIFARE como cadena ASCII.
 * @param bloque Número de bloque a leer.
 * @return Cadena (máx. 16 chars, detiene en @c 0x00).
 */
String      rfid_leerBloque(byte bloque);
/**
 * @brief Compara dos UIDs de 4 bytes.
 * @param uid Puntero a UID leído.
 * @param ref Puntero a UID de referencia.
 * @return @c true si coinciden; @c false en caso contrario.
 */
bool        rfid_uidCoincide(byte *uid, byte *ref);

/* UI / Utilidades */
/**
 * @brief Solicita y lee una clave de 4 dígitos (timeout 15 s).
 * @return Cadena de 4 dígitos o cadena vacía si expira.
 */
String ui_recibirClave();
/**
 * @brief Lee una cadena terminada en @c '\\0' desde EEPROM.
 * @param base Dirección base.
 * @return Cadena leída (vacía si hay @c 0xFF o @c '\\0' en la primera celda).
 */
String eeprom_leerCadena(int base);
/**
 * @brief Indica si una dirección de EEPROM está “vacía”.
 * @param base Dirección a comprobar.
 * @return @c true si el byte es 0xFF o @c '\\0'; @c false en otro caso.
 */
bool   eeprom_estaVacio(int base);
/**
 * @brief Refresca la segunda línea del LCD con T y PMV actuales.
 */
void   ui_actualizarDisplay();

/* PMV / Sensores */
/**
 * @brief Presión de vapor de saturación (kPa).
 * @param T Temperatura del aire en °C.
 * @return Presión de vapor de saturación en kPa.
 */
static inline float svp_kPa(float T);
/**
 * @brief Calcula el índice PMV.
 * @param Ta  Temperatura del aire [°C].
 * @param Tr  Temperatura radiante media [°C].
 * @param RH  Humedad relativa [% 0–100].
 * @param met Metabolismo [met].
 * @param clo Aislamiento de ropa [clo].
 * @param va  Velocidad del aire [m/s].
 * @return ::PMVResult con el valor PMV [-3, 3].
 */
PMVResult pmv_calcular(float Ta, float Tr, float RH, float met, float clo, float va);
/**
 * @brief Lee temperatura desde el NTC (°C).
 * @return Temperatura en grados Celsius.
 */
float ntc_leerC();

/* ==========================
 *  Implementación - Utilidades PMV/NTC
 * ========================== */

static inline float svp_kPa(float T) {
  return 0.6105f * expf((17.27f * T) / (T + 237.3f));
}

PMVResult pmv_calcular(float Ta, float Tr, float RH, float met, float clo, float va) {
  if (isnan(Ta) || isnan(Tr) || isnan(RH)) return { 0.0f };

  if (Ta < -10.0f || Ta > 50.0f) Ta = 25.0f;
  if (Tr < -10.0f || Tr > 50.0f) Tr = Ta;
  if (RH < 0.0f) RH = 0.0f;
  if (RH > 100.0f) RH = 100.0f;
  if (va < 0.0f) va = 0.1f;

  const float M = met * 58.15f;
  const float W = 0.0f;

  float p_sat = svp_kPa(Ta);
  float p_a   = (RH / 100.0f) * p_sat * 1000.0f;

  float f_cl = (clo <= 0.078f) ? (1.05f + 0.1f * clo) : (1.0f + 0.2f * clo);
  float I_cl = clo * 0.155f;
  float T_cl = Ta + 0.1f;

  for (int i = 0; i < 200; i++) {
    float h_c  = 12.1f * sqrtf(max(0.0001f, va));
    float d    = fabsf(T_cl - Ta);
    float h_c2 = 2.38f * powf(d, 0.25f);
    if (h_c2 > h_c) h_c = h_c2;

    float tclK = T_cl + 273.15f;
    float trK  = Tr   + 273.15f;
    float rad  = 3.96e-8f * f_cl * (powf(tclK, 4.0f) - powf(trK, 4.0f));

    float T_new = 35.7f - 0.028f * (M - W) - I_cl * (rad + f_cl * h_c * (T_cl - Ta));

    if (fabsf(T_new - T_cl) < 1e-4f) { T_cl = T_new; break; }
    T_cl = T_new;
  }

  float h_c  = 12.1f * sqrtf(max(0.0001f, va));
  float d    = fabsf(T_cl - Ta);
  float h_c2 = 2.38f * powf(d, 0.25f);
  if (h_c2 > h_c) h_c = h_c2;

  float tclK = T_cl + 273.15f;
  float trK  = Tr   + 273.15f;
  float rad  = 3.96e-8f * f_cl * (powf(tclK, 4.0f) - powf(trK, 4.0f));

  float F = 0.303f * expf(-0.036f * M) + 0.028f;
  float B = (M - W)
          - 3.05e-3f * (5733.0f - 6.99f * (M - W) - p_a)
          - 0.42f    * ((M - W) - 58.15f)
          - 1.7e-5f  * M * (5867.0f - p_a)
          - 0.0014f  * M * (34.0f - Ta)
          - rad
          - f_cl * h_c * (T_cl - Ta);

  float pmv = F * B;
  if (pmv >  3.0f) pmv =  3.0f;
  if (pmv < -3.0f) pmv = -3.0f;
  return { pmv };
}

float ntc_leerC() {
  int   adc  = analogRead(PIN_ANALOGICO);
  float Vout = adc * (5.0f / 1023.0f);
  float Rntc = (NTC_R_SERIE * Vout) / (5.0f - Vout);
  float Tkel = 1.0f / ((1.0f / NTC_T0) + (1.0f / NTC_BETA) * log(Rntc / NTC_R0));
  return Tkel - 273.15f;
}

/* ==========================
 *  Implementación - RFID/EEPROM
 * ========================== */

static bool rfid_uid_igual(const byte a[4], const byte b[4]) {
  for (byte i=0;i<4;i++) if (a[i]!=b[i]) return false;
  return true;
}

static void rfid_copiar_str_fija(const String& s, char* dst, int maxLen) {
  int n = s.length(); if (n > maxLen-1) n = maxLen-1;
  for (int i=0;i<n;i++) dst[i] = s[i];
  dst[n] = '\0';
  for (int i=n+1;i<maxLen;i++) dst[i] = '\0';
}

static void rfid_eeprom_leer(int slot, RfidSlot &out) {
  EEPROM.get(rfid_dir_slot(slot), out);
}

static void rfid_eeprom_grabar(int slot, const RfidSlot &in) {
  EEPROM.put(rfid_dir_slot(slot), in);
}

static int rfid_buscar_por_uid(const byte uid[4]) {
  RfidSlot s;
  for (int i=0;i<RFID_MAX_SLOTS;i++) {
    rfid_eeprom_leer(i, s);
    if (s.valid==0xA5 && rfid_uid_igual(s.uid, uid)) return i;
  }
  return -1;
}

static int rfid_buscar_libre() {
  RfidSlot s;
  for (int i=0;i<RFID_MAX_SLOTS;i++) {
    rfid_eeprom_leer(i, s);
    if (s.valid != 0xA5) return i;
  }
  return -1;
}

static void rfid_eeprom_init_si_vacia() {
  RfidSlot s0; rfid_eeprom_leer(0, s0);
  bool vacia = (s0.valid != 0xA5);
  if (vacia) {
    RfidSlot blank; memset(&blank, 0xFF, sizeof(blank));
    for (int i=0;i<RFID_MAX_SLOTS;i++) {
      blank.valid = 0x00;
      for (int k=0;k<4;k++) blank.uid[k]=0x00;
      blank.nombre[0]='\0';
      blank.rango[0]='\0';
      rfid_eeprom_grabar(i, blank);
    }
  }
}

static int rfid_guardar_perfil(const byte uid[4], const String& nombre, const String& rango) {
  int slot = rfid_buscar_por_uid(uid);
  if (slot < 0) {
    slot = rfid_buscar_libre();
    if (slot < 0) slot = 0;
  }

  RfidSlot s; memset(&s, 0, sizeof(s));
  s.valid = 0xA5;
  for (byte i=0;i<4;i++) s.uid[i] = uid[i];
  rfid_copiar_str_fija(nombre, s.nombre, sizeof(s.nombre));
  rfid_copiar_str_fija(rango,  s.rango,  sizeof(s.rango));
  rfid_eeprom_grabar(slot, s);
  return slot;
}

static void rfid_print_estado() {
  Serial.println(F("== Estado EEPROM RFID =="));
  RfidSlot s;
  for (int i=0;i<RFID_MAX_SLOTS;i++) {
    rfid_eeprom_leer(i, s);
    if (s.valid==0xA5) {
      Serial.print(F("Slot ")); Serial.print(i); Serial.print(F(": UID "));
      for (byte k=0;k<4;k++){ Serial.print(s.uid[k]<0x10?"0":""); Serial.print(s.uid[k], HEX); if(k<3) Serial.print(' '); }
      Serial.print(F(" | Nombre: ")); Serial.print(s.nombre);
      Serial.print(F(" | Rango: "));  Serial.println(s.rango);
    } else {
      Serial.print(F("Slot ")); Serial.print(i); Serial.println(F(": LIBRE"));
    }
  }
}

/**
 * @brief Lee nombre y rango desde una tarjeta y actualiza EEPROM y UI.
 * @details
 * - Requiere tarjeta presente y lectura de UID.
 * - Si el UID coincide con @ref uidTarjeta o @ref uidLlavero, lee bloques 4 y 5.
 * - Muestra mensajes en LCD y guarda el perfil en EEPROM.
 */
void rfid_leerPerfil() {
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  byte bloqueNombre = 4;
  byte bloqueTemp   = 5;

  Serial.print(F("\nUID detectado: "));
  for (byte i = 0; i < rfid.uid.size; i++) {
    Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(rfid.uid.uidByte[i], HEX);
  }
  Serial.println();

  if (rfid_uid_igual(rfid.uid.uidByte, uidTarjeta)) {
    String nombre = rfid_leerBloque(bloqueNombre);
    String temp   = rfid_leerBloque(bloqueTemp);

    pantalla.clear();
    pantalla.setCursor(0, 0);
    pantalla.print("Bienvenido:");
    pantalla.setCursor(0, 1);
    pantalla.print(nombre.substring(0, 16));
    delay(2000);

    pantalla.clear();
    pantalla.setCursor(0, 0);
    pantalla.print("Confort (usr):");
    pantalla.setCursor(0, 1);
    pantalla.print(temp.substring(0, 16));
    delay(2000);

    Serial.print(F("Bienvenido ")); Serial.println(nombre);
    Serial.print(F("Temperatura preferida: ")); Serial.println(temp);

    int slot = rfid_guardar_perfil(rfid.uid.uidByte, nombre, temp);
    if (slot >= 0) { Serial.print(F("Perfil guardado en slot ")); Serial.println(slot); }
    else           { Serial.println(F("ERROR: no se pudo guardar en EEPROM")); }

    tareaCfg.Start();
    pantalla.clear();
    pantalla.setCursor(0, 0); pantalla.print("Configurado");
    pantalla.setCursor(0, 1); pantalla.print("Correctamente");
  }
  else if (rfid_uid_igual(rfid.uid.uidByte, uidLlavero)) {
    String nombre = rfid_leerBloque(bloqueNombre);
    String temp   = rfid_leerBloque(bloqueTemp);
    Serial.print(F("Bienvenido ")); Serial.println(nombre);
    Serial.print(F("Temperatura preferida: ")); Serial.println(temp);

    int slot = rfid_guardar_perfil(rfid.uid.uidByte, nombre, temp);
    if (slot >= 0) { Serial.print(F("Perfil guardado en slot ")); Serial.println(slot); }
    else           { Serial.println(F("ERROR: no se pudo guardar en EEPROM")); }

    tareaCfg.Start();
    pantalla.clear();
    pantalla.setCursor(0, 0); pantalla.print("Configurado");
    pantalla.setCursor(0, 1); pantalla.print("Correctamente");
  }
  else {
    Serial.println(F("UID desconocido → no registrado"));
    pantalla.clear();
    pantalla.setCursor(0, 0); pantalla.print("Tarjeta no");
    pantalla.setCursor(0, 1); pantalla.print("reconocida");
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  delay(2000);
}

void rfid_autenticarBloque(byte bloque) {
  byte trailer = bloque - (bloque % 4) + 3;
  MFRC522::StatusCode st = (MFRC522::StatusCode) rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A,
                                                                       trailer, &mifareKey, &(rfid.uid));
  if (st != MFRC522::STATUS_OK) {
    Serial.print(F("Error de autenticación en bloque "));
    Serial.print(bloque);
    Serial.print(F(": "));
    Serial.println(rfid.GetStatusCodeName(st));
  }
}

String rfid_leerBloque(byte bloque) {
  rfid_autenticarBloque(bloque);
  byte buffer[18];
  byte size = sizeof(buffer);
  MFRC522::StatusCode st = (MFRC522::StatusCode)rfid.MIFARE_Read(bloque, buffer, &size);
  if (st != MFRC522::STATUS_OK) {
    Serial.print(F("Error leyendo bloque ")); Serial.println(bloque);
    return "";
  }

  String out = "";
  for (int i = 0; i < 16; i++) {
    if (buffer[i] == 0) break;
    out += (char)buffer[i];
  }
  return out;
}

/* ==========================
 *  Implementación - UI/EEPROM utils
 * ========================== */

String ui_recibirClave() {
  pantalla.setCursor(0, 1);
  pantalla.print("Clave: []     ");
  pantalla.setCursor(7, 1);

  String res = "";
  unsigned long t0 = millis();

  while (res.length() < 4 && (millis() - t0) < 15000) {
    char k = teclado.getKey();
    if (k) {
      if (k >= '0' && k <= '9') {
        pantalla.print('*');
        res += k;
      } else if (k == '*' && res.length() > 0) {
        res.remove(res.length()-1);
        pantalla.setCursor(7, 1);
        for (int i = 0; i < res.length(); i++) pantalla.print('*');
        for (int i = res.length(); i < 4; i++)  pantalla.print(' ');
        pantalla.setCursor(7 + res.length(), 1);
      }
    }
    delay(50);
  }
  if (res.length() < 4) return "";
  return res;
}

String eeprom_leerCadena(int base) {
  char c; String s=""; int i=0;
  while (true) {
    c = EEPROM.read(base + i);
    if (c == '\0' || c == 0xFF) break;
    s += c; i++;
  }
  return s;
}

bool eeprom_estaVacio(int base) {
  byte v = EEPROM.read(base);
  return (v == 0xFF || v == '\0');
}

void ui_actualizarDisplay() {
  pantalla.setCursor(0, 1);
  pantalla.print("T:");
  pantalla.print(tempActual, 1);
  pantalla.print("C PMV:");
  pantalla.print(pmvActual, 2);
  pantalla.print("  ");
}

/* ==========================
 *  Implementación - FSM y entradas
 * ========================== */

/**
 * @brief Lee eventos de entrada según estado actual.
 * @return Un valor de ::Input para la máquina de estados.
 */
int leerEvento() {
  State st = fsm.GetState();
  char k = teclado.getKey();

  // ALARMA
  if (st == Alarma) {
    int pres = digitalRead(PIN_IR);
    unsigned long now = millis();

    if (pres == LOW && irArmado && (now - irUltimo > IR_DEBOUNCE_MS)) {
      Serial.println("=== PRESENCIA DETECTADA - TRANSICION A INICIO ===");
      irUltimo = now; irArmado = false;
      digitalWrite(PIN_LED_ROJO, LOW);
      digitalWrite(PIN_BUZZER, LOW);
      tareaBuzzer.Stop();
      return Input::sensorIR;
    }

    if (pres == HIGH && !irArmado) {
      irArmado = true;
      Serial.println("=== SENSOR IR LIBERADO - REARMADO ===");
    }

    if (k == '#') {
      Serial.println("=== TECLA # PRESIONADA - TRANSICION A INICIO ===");
      digitalWrite(PIN_LED_ROJO, LOW);
      digitalWrite(PIN_BUZZER, LOW);
      tareaBuzzer.Stop();
      tareaRojoCortoOn.Stop();
      tareaRojoCortoOff.Stop();
      return Input::keypadInput;
    }
    return Input::Unknown;
  }

  // INICIO
  if (st == inicio) {
    String cod = ui_recibirClave();
    if (cod.length() == 4) {
      if (cod == claveSistema) return Input::keypadInput;
      else                     return Input::keypadBlock;
    }
  }

  // BLOQUEADO
  if (st == Bloqueado) {
    if (k == '*') {
      Serial.println("Tecla '*' presionada - Volviendo a INICIO");
      return Input::keypadInput;
    }
  }

  // CONFIG
  if (st == Config) {
    rfid_leerPerfil();
    if (evento == tiempo) return Input::tiempo;
  }

  // PMV_ALTO
  if (st == pmv_alto) {
    if (evento == tiempo) {
      Serial.println(">>> Timer pmv_alto disparado - Leyendo sensores");
      float Ta = sensorDHT.readTemperature();
      float RH = sensorDHT.readHumidity();
      float Tr = ntc_leerC();

      if (isnan(Ta) || isnan(RH)) {
        Serial.println("ERROR: Lecturas NaN - Reintentando");
        evento = Unknown;
        tareaPMVAlto.Start();
        return Input::Unknown;
      }

      tempActual = Ta;
      PMVResult r = pmv_calcular(Ta, Tr, RH, 1.0f, 0.61f, 0.1f);
      pmvActual = r.pmv;

      Serial.print("PMV_ALTO -> T:"); Serial.print(Ta);
      Serial.print("C | RH:");        Serial.print(RH);
      Serial.print("% | PMV:");       Serial.println(pmvActual);

      evento = Unknown;

      if (pmvActual <= 0.22f) {
        Serial.println("PMV NORMALIZADO - PREPARANDO SALIDA A MONITOR");
        tareaPMVAlto.Stop();
        reintentosTempAlta = 0;
        flagSalirPMVAlto = true;
        return Input::Unknown;
      }

      if (tempActual >= 21.0f) {
        reintentosTempAlta++;
        Serial.print("Intento "); Serial.print(reintentosTempAlta); Serial.println("/3 - PMV continua alto");
        if (reintentosTempAlta >= 3) {
          Serial.println("ALARMA: 3 intentos agotados - TRANSICION A ALARMA");
          tareaPMVAlto.Stop();
          return Input::alarmaTemp;
        }
      } else {
        Serial.println("Temperatura <=22 -> reseteo contador");
        reintentosTempAlta = 0;
      }

      tareaPMVAlto.Start();
      return Input::Unknown;
    }
    return Input::Unknown;
  }

  // PMV_BAJO
  if (st == pmv_bajo) {
    if (evento == tiempo) {
      tareaPMVBajo.Start();
      return Input::tiempo;
    }

    float Ta = sensorDHT.readTemperature();
    float RH = sensorDHT.readHumidity();
    float Tr = ntc_leerC();
    tempActual = Ta;

    if (!isnan(Ta) && !isnan(RH)) {
      PMVResult r = pmv_calcular(Ta, Tr, RH, 1.0f, 0.61f, 0.1f);
      pmvActual = r.pmv;

      if (pmvActual >= -0.1f) {
        Serial.print("PMV normalizado en estado BAJO: ");
        Serial.println(pmvActual);
        return Input::tiempo;
      }
    }
  }

  // MONITOR
  if (st == Monitor) {
    if (evento == tiempo) return Input::tiempo;

    float Ta = sensorDHT.readTemperature();
    float RH = sensorDHT.readHumidity();
    float Tr = ntc_leerC();
    tempActual = Ta;

    if (!isnan(Ta) && !isnan(RH)) {
      PMVResult r = pmv_calcular(Ta, Tr, RH, 1.0f, 0.61f, 0.1f);
      pmvActual = r.pmv;

      Serial.print("Monitor - Temp: "); Serial.print(Ta);
      Serial.print("C, PMV: ");         Serial.println(pmvActual);

      if (pmvActual > 0.2f)  return Input::pmv;
      if (pmvActual < -0.1f) return Input::pmv;
    }
  }

  int btn = digitalRead(PIN_BOTON);
  if (btn == LOW) {
    delay(50);
    if (digitalRead(PIN_BOTON) == LOW) return Input::boton;
  }

  return Input::Unknown;
}

/**
 * @brief Configura todas las transiciones y callbacks de la FSM (idénticas al original).
 */
void configurarFSM() {
  // INICIO
  fsm.AddTransition(inicio, Config, [](){ return evento == keypadInput; });
  fsm.AddTransition(inicio, Bloqueado, [](){ return evento == keypadBlock; });

  // BLOQUEADO
  fsm.AddTransition(Bloqueado, inicio, [](){ return evento == boton; });
  fsm.AddTransition(Bloqueado, inicio, [](){ return evento == keypadInput; });

  // CONFIG <-> MONITOR
  fsm.AddTransition(Config,  Monitor, [](){ return evento == tiempo; });
  fsm.AddTransition(Monitor, Config,  [](){ return evento == tiempo; });

  // MONITOR -> PMV
  fsm.AddTransition(Monitor, pmv_alto, [](){ return evento == pmv && pmvActual > 0.2f; });
  fsm.AddTransition(Monitor, pmv_bajo, [](){ return evento == pmv && pmvActual < -0.1f; });

  // PMV_ALTO -> ALARMA
  fsm.AddTransition(pmv_alto, Alarma, [](){ return evento == alarmaTemp && reintentosTempAlta >= 3; });

  // PMV_ALTO -> MONITOR (condición directa)
  fsm.AddTransition(pmv_alto, Monitor, [](){ return pmvActual <= 0.22f; });

  // PMV_BAJO -> MONITOR
  fsm.AddTransition(pmv_bajo, Monitor, [](){ return evento == tiempo || pmvActual >= -0.1f; });

  // ALARMA -> INICIO
  fsm.AddTransition(Alarma, inicio, [](){ return evento == sensorIR; });
  fsm.AddTransition(Alarma, inicio, [](){ return evento == keypadInput; });

  // Callbacks de entrada
  fsm.SetOnEntering(inicio,       enteringInicio);
  fsm.SetOnEntering(Config,       enteringConfig);
  fsm.SetOnEntering(Bloqueado,    enteringBloqueado);
  fsm.SetOnEntering(Alarma,       enteringAlarma);
  fsm.SetOnEntering(Monitor,      enteringMonitor);
  fsm.SetOnEntering(pmv_alto,     enteringPMVALTO);
  fsm.SetOnEntering(pmv_bajo,     enteringPMVBAJO);

  // Callbacks de salida
  fsm.SetOnLeaving(inicio,        leavingInicio);
  fsm.SetOnLeaving(Config,        leavingConfig);
  fsm.SetOnLeaving(Bloqueado,     leavingBloqueado);
  fsm.SetOnLeaving(Alarma,        leavingAlarma);
  fsm.SetOnLeaving(Monitor,       leavingMonitor);
  fsm.SetOnLeaving(pmv_alto,      leavingPmvAlto);
  fsm.SetOnLeaving(pmv_bajo,      leavingPmvBajo);
}

/* ==========================
 *  Callbacks de estados (nombres preservados)
 * ========================== */

void leavingInicio() {
  Serial.println("Leaving INICIO");
  entradaClave = "";
  reintentosTempAlta = 0;
}

void leavingConfig() {
  Serial.println("Leaving CONFIG");
  tareaCfg.Stop();
  evento = Unknown;
}

void leavingBloqueado() {
  Serial.println("Leaving BLOQUEADO");
  tareaRojoOn.Stop();
  tareaRojoOff.Stop();
  digitalWrite(PIN_LED_ROJO, LOW);
}

void leavingAlarma() {
  Serial.println("Leaving ALARMA");
  reintentosTempAlta = 0;
  tareaBuzzer.Stop();
  digitalWrite(PIN_BUZZER, LOW);
  irArmado = true;
  irUltimo = 0;
}

void leavingMonitor() {
  Serial.println("Leaving MONITOR");
  tareaMon.Stop();
  evento = Unknown;
}

void leavingPmvAlto() {
  Serial.println("Leaving PMV_ALTO");
  digitalWrite(PIN_RELE, LOW);
  digitalWrite(PIN_LED_AZUL, LOW);
  tareaPMVAlto.Stop();

  tareaAzulOn.Stop();
  tareaAzulOff.Stop();
  digitalWrite(PIN_LED_AZUL, LOW);

  if (evento != alarmaTemp) {
    reintentosTempAlta = 0;
    Serial.println("Contador de temperatura alta reseteado");
  }
  flagSalirPMVAlto = false;
  evento = Unknown;
}

void leavingPmvBajo() {
  Serial.println("Leaving PMV_BAJO");
  servoPuerta.write(0);
  digitalWrite(PIN_LED_VERDE, LOW);
  tareaPMVBajo.Stop();
  tareaVerdeOn.Stop();
  tareaVerdeOff.Stop();
  evento = Unknown;
}

void enteringInicio() {
  Serial.println("-> Estado: INICIO");
  pantalla.clear();
  pantalla.setCursor(0, 0); pantalla.print("Sistema Listo");
  pantalla.setCursor(0, 1); pantalla.print("Ingrese clave:");
}

void enteringConfig() {
  Serial.println("-> Estado: CONFIG");
  pantalla.clear();
  pantalla.setCursor(0, 0); pantalla.print("Modo CONFIG");
  pantalla.setCursor(0, 1); pantalla.print("Escanee tarjeta");
}

void enteringBloqueado() {
  Serial.println("-> Estado: BLOQUEADO");
  pantalla.clear();
  pantalla.setCursor(0, 0); pantalla.print("BLOQUEADO");
  pantalla.setCursor(0, 1); pantalla.print("Clave incorrecta , presione *");
  digitalWrite(PIN_LED_ROJO, LOW);
  tareaRojoOn.Start();
}

void enteringAlarma() {
  tareaBuzzer.Start();
  Serial.println("-> Estado: ALARMA");
  Serial.println("*** ALARMA ACTIVADA ***");
  pantalla.clear();
  pantalla.setCursor(0, 0); pantalla.print("*** ALARMA ***");
  pantalla.setCursor(0, 1); pantalla.print("Presione OFF");
  irArmado = true;
  irUltimo = 0;
}

void enteringMonitor() {
  tareaMon.Start();
  Serial.println("-> Estado: MONITOR");
  flagSalirPMVAlto = false;

  float Ta = sensorDHT.readTemperature();
  float RH = sensorDHT.readHumidity();
  float Tr = ntc_leerC();
  if (!isnan(Ta) && !isnan(RH)) {
    tempActual = Ta;
    PMVResult r = pmv_calcular(Ta, Tr, RH, 1.0f, 0.61f, 0.1f);
    pmvActual = r.pmv;
  }

  pantalla.setCursor(0, 0);
  pantalla.print("T:"); pantalla.print(tempActual, 1);
  pantalla.print("C Tr:"); pantalla.print(Tr, 1); pantalla.print("C");

  pantalla.setCursor(0, 1);
  pantalla.print("H:"); pantalla.print(RH, 0);
  pantalla.print("% PMV:"); pantalla.print(pmvActual, 2);

  Serial.print("Temperatura: "); Serial.println(tempActual);
  Serial.print("PMV: ");        Serial.println(pmvActual);
}

void enteringPMVALTO() {
  tareaPMVAlto.Start();
  digitalWrite(PIN_RELE, HIGH);
  digitalWrite(PIN_LED_AZUL, LOW);
  tareaAzulOn.Start();
  Serial.println("-> Estado: PMV_ALTO");
  pantalla.clear();
  pantalla.setCursor(0, 0);
  pantalla.print("PMV ALTO "); pantalla.print(pmvActual, 1);
  pantalla.setCursor(0, 1);
}

void enteringPMVBAJO() {
  tareaPMVBajo.Start();
  digitalWrite(PIN_LED_VERDE, LOW);
  tareaVerdeOn.Start();
  servoPuerta.write(90);
  Serial.println("-> Estado: PMV_BAJO");
  pantalla.clear();
  pantalla.setCursor(0, 0); pantalla.print("PMV BAJO");
  pantalla.setCursor(0, 1); pantalla.print("Calentando...");
}

/* ==========================
 *  setup() y loop()
 * ========================== */

/**
 * @brief Inicialización de hardware y FSM.
 * @post El sistema arranca en el estado ::inicio con periféricos inicializados.
 */
void setup() {
  Serial.begin(9600);
  rfid_eeprom_init_si_vacia();

  pinMode(PIN_BOTON, INPUT_PULLUP);
  pinMode(PIN_LED_AZUL, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);
  pinMode(PIN_RELE, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_IR, INPUT);

  digitalWrite(PIN_RELE, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  servoPuerta.attach(PIN_SERVO);
  servoPuerta.write(0);

  SPI.begin();
  rfid.PCD_Init();

  // Clave MIFARE por defecto (0xFF * 6)
  for (byte i = 0; i < 6; i++) mifareKey.keyByte[i] = 0xFF;

  pantalla.begin(16, 2);
  sensorDHT.begin();

  Serial.println("Starting State Machine...");
  configurarFSM();
  Serial.println("State Machine Started");

  fsm.SetState(inicio, false, true);
}

/**
 * @brief Bucle principal: gestiona eventos, actualiza FSM y tareas asíncronas.
 * @details
 * - Llama a ::leerEvento para obtener eventos.
 * - Ejecuta @c fsm.Update() y actualiza todas las tareas.
 * - Mantiene una pequeña pausa de 10 ms.
 */
void loop() {
  Input nuevo = static_cast<Input>(leerEvento());
  if (nuevo != Unknown) evento = nuevo;

  fsm.Update();

  static State prev = inicio;
  State cur = fsm.GetState();
  if (cur != prev) {
    Serial.print("CAMBIO DE ESTADO: "); Serial.print(prev);
    Serial.print(" -> ");               Serial.println(cur);
    prev = cur; evento = Unknown;
  }

  // Tareas
  tareaCfg.Update();
  tareaMon.Update();
  tareaPMVAlto.Update();
  tareaPMVBajo.Update();

  tareaAzulOn.Update(tareaAzulOff);
  tareaAzulOff.Update(tareaAzulOn);

  tareaVerdeOn.Update(tareaVerdeOff);
  tareaVerdeOff.Update(tareaVerdeOn);

  tareaRojoOn.Update(tareaRojoOff);
  tareaRojoOff.Update(tareaRojoOn);

  tareaRojoCortoOn.Update(tareaRojoCortoOff);
  tareaRojoCortoOff.Update(tareaRojoCortoOn);

  tareaBuzzer.Update();

  delay(10);
}
