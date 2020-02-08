/*
 * Project: Capteurs-EB
 * Description: Code pour tous les Capteurs-EB de l'Érablière Brunelle
 *              Il suffit de choisir la configuraion correspondante à la fonctionnalité voulu
 *              et modifier la valeur #define DEVICE_CONF X (ligne 45)
 *              sélectionner un "device" à programmer.
 * Author: Pierre Leboeuf
 * Date: Feb. 2016
 */

// This #include statement was automatically added by the Particle IDE.
#include "Particle.h"
#include "spark-dallas-temperature.h"
#include "math.h"

STARTUP(WiFi.selectAntenna(ANT_AUTO));
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));
SYSTEM_THREAD(ENABLED);

// Firmware version et date
#define FirmwareVersion "1.7.1"   // Version du firmware du capteur.
String F_Date  = __DATE__;
String F_Time = __TIME__;
String FirmwareDate = F_Date + " " + F_Time; // Date et heure de compilation UTC

#define NONE 0
#define US100 1
#define MB7389 2

/*
Paramètres de compilation conditionnelle. Le programme qui suit est le même pour tous les devices.
Les paramètres définissent la configuration des devices et le code est compilé conditionnellement
dépendamment de cette configuration.
*/

/********* Choisir la configuration de device à compiler *********/
#define DEVICE_CONF 0
// Config pour:
// P1, P2, P3 -> DEVICE_CONF == 0
// V1, V2, V3 -> DEVICE_CONF == 1
// PT1, PT2 -> DEVICE_CONF == 2
// RS1, RS2, RS3, RS4, RF2 -> DEVICE_CONF == 3
// RF1, RC1, RC2 -> DEVICE_CONF == 4
// RS5, RS6 -> DEVICE_CONF == 5
// RHC -> DEVICE_CONF == 6
// VEcTk -> DEVICE_CONF == 7
/*****************************************************************/

#if (DEVICE_CONF == 0)
  String config = "0 -> P1, P2, P3";  // String info de configuration
  #define DISTANCESENSOR NONE         // Pour compilation conditionnelle du serial handler: US100. MB7389, None
  #define PUMPMOTORDETECT true        // Pour compilation conditionnelle de la routin e d'interruption
  #define HASDS18B20SENSOR false      // Pour le code spécifique au captgeur de température DS18B20
  #define HASHEATING false            // Pour le chauffage du boitier
  #define HASVACUUMSENSOR true        // Un capteur de vide est installé
  #define HASVALVES false             // Des valves sont relié à ce capteur
  #define HASRELAYOUTPUT false        // Un relais SSR peut être relié à ce capteur
  #define HASUS100THERMISTOR false    // Un thermistor est présent pour mesurer la température du boitier US100 robuste
  #define baseSampling  1             // basic sampling interval for main loop
  #define pumpMinRunTime 17 * second  // Ignore cycles where pump is running less than 17 seconds
  #define pumpRunTimeLimit 2 * minute // Maximum pump run time before a warning is issued
  /*Config for P1, P2, P3 -> DEVICE_CONF == 0
  DEVICE   DISTANCESENSOR  PUMPMOTORDETECT HASDS18B20SENSOR HASHEATING HASVACUUMSENSOR HASVALVES HASRELAYOUTPUT HASUS100THERMISTOR
  P1        None            true            false            false      true            false     false          false
  P2        None            true            false            false      true            false     false          false
  P3        None            true            false            false      true            false     false          false
  */

#elif (DEVICE_CONF == 1)
  String config = "1 -> V1, V2, V3";  // String info de configuration
  #define DISTANCESENSOR NONE         // Pour compilation conditionnelle du serial handler: US100. MB7389, None
  #define PUMPMOTORDETECT true        // Pour compilation conditionnelle de la routin e d'interruption
  #define HASDS18B20SENSOR true       // Pour le code spécifique au captgeur de température DS18B20
  #define HASHEATING true             // Pour le chauffage du boitier
  #define HASVACUUMSENSOR true        // Un capteur de vide est installé
  #define HASVALVES false             // Des valves sont relié à ce capteur
  #define HASRELAYOUTPUT true         // Un relais SSR peut être relié à ce capteur
  #define HASUS100THERMISTOR false    // Un thermistor est présent pour mesurer la température du boitier US100 robuste
  #define baseSampling  1             // basic sampling interval for main loop
  #define pumpMinRunTime 0 * second   // Ignore cycles where pump is running less than 17 seconds
  #define pumpRunTimeLimit 30 * 24 * heure // Maximum pump run time before a warning is issued
  /*Config for V1, V2, V3 -> DEVICE_CONF == 1
  DEVICE   DISTANCESENSOR  PUMPMOTORDETECT HASDS18B20SENSOR HASHEATING HASVACUUMSENSOR HASVALVES HASRELAYOUTPUT HASUS100THERMISTOR
  V1        None            true            true(1)          true       true            false     true           false
  V2        None            true            true(1)          true       true            false     true           false
  V3        None            true            true(1)          true       true            false     true           false
  */

#elif (DEVICE_CONF == 2)
  String config = "2 -> PT1, PT2";    // String info de configuration
  #define DISTANCESENSOR NONE         // Pour compilation conditionnelle du serial handler: US100. MB7389, None
  #define PUMPMOTORDETECT true        // Pour compilation conditionnelle de la routin e d'interruption
  #define HASDS18B20SENSOR true       // Pour le code spécifique au captgeur de température DS18B20
  #define HASHEATING false            // Pour le chauffage du boitier
  #define HASVACUUMSENSOR false       // Un capteur de vide est installé
  #define HASVALVES false             // Des valves sont relié à ce capteur
  #define HASRELAYOUTPUT true         // Un relais SSR peut être relié à ce capteur
  #define HASUS100THERMISTOR false    // Un thermistor est présent pour mesurer la température du boitier US100 robuste
  #define baseSampling  1             // basic sampling interval for main loop
  #define pumpMinRunTime 0 * second   // Ignore cycles where pump is running less than 17 seconds
  #define pumpRunTimeLimit 3 * 24 * heure // Maximum pump run time before a warning is issued
 /*Config for PT1, PT2 -> DEVICE_CONF == 2
  DEVICE   DISTANCESENSOR  PUMPMOTORDETECT HASDS18B20SENSOR HASHEATING HASVACUUMSENSOR HASVALVES HASRELAYOUTPUT HASUS100THERMISTOR
  PT1       None            true            true(1)          false      false           false     true           false
  PT2       None            true            true(1)          false      false           false     true           false
  */

#elif (DEVICE_CONF == 3)
  String config = "3 -> RS1, RS2, RS3, RS4, RF2"; // String info de configuration
  #define DISTANCESENSOR US100        // Pour compilation conditionnelle du serial handler: US100. MB7389, None
  #define PUMPMOTORDETECT false       // Pour compilation conditionnelle de la routin e d'interruption
  #define HASDS18B20SENSOR true       // Pour le code spécifique au captgeur de température DS18B20
  #define HASHEATING true             // Pour le chauffage du boitier
  #define HASVACUUMSENSOR false       // Un capteur de vide est installé
  #define HASVALVES true              // Des valves sont relié à ce capteur
  #define HASRELAYOUTPUT false        // Un relais SSR peut être relié à ce capteur
  #define HASUS100THERMISTOR true     // Un thermistor est présent pour mesurer la température du boitier US100 robuste
  #define baseSampling  2             // basic sampling interval for main loop. Very slow to avoid echo
  /*Config for RS1, RS2, RS3, RS4, RF2 -> DEVICE_CONF == 3
  DEVICE   DISTANCESENSOR  PUMPMOTORDETECT HASDS18B20SENSOR HASHEATING HASVACUUMSENSOR HASVALVES HASRELAYOUTPUT HASUS100THERMISTOR
  RS1       US100           false           true(2)          true       false           true      false          true
  RS2       US100           false           true(1)          true       false           true      false          true
  RS3       US100           false           true(1)          true       false           true      false          true
  RS4       US100           false           true(1)          true       false           true      false          true
  RF2       US100           false           true(1)          true       false           true      false          true
  */

#elif (DEVICE_CONF == 4)
  String config = "4 -> RF1, RC1, RC2"; // String info de configuration
  #define DISTANCESENSOR US100        // Pour compilation conditionnelle du serial handler: US100. MB7389, None
  #define PUMPMOTORDETECT false       // Pour compilation conditionnelle de la routin e d'interruption
  #define HASDS18B20SENSOR true       // Pour le code spécifique au captgeur de température DS18B20
  #define HASHEATING true             // Pour le chauffage du boitier
  #define HASVACUUMSENSOR false       // Un capteur de vide est installé
  #define HASVALVES false             // Des valves sont relié à ce capteur
  #define HASRELAYOUTPUT false        // Un relais SSR peut être relié à ce capteur
  #define HASUS100THERMISTOR false    // Un thermistor est présent pour mesurer la température du boitier US100 robuste
  #define baseSampling  1             // basic sampling interval for main loop
  /*Config for RF1, RC1, RC2 -> DEVICE_CONF == 4
  DEVICE   DISTANCESENSOR  PUMPMOTORDETECT HASDS18B20SENSOR HASHEATING HASVACUUMSENSOR HASVALVES HASRELAYOUTPUT HASUS100THERMISTOR
  RF1       US100           false           true(1)          true       false           false     false          false
  RC1       US100           false           true(1)          true       false           false     false          false
  RC2       US100           false           true(1)          true       false           false     false          false
  */

#elif (DEVICE_CONF == 5)
  String config = "5 -> RS5, RS6";    // String info de configuration
  #define DISTANCESENSOR US100        // Pour compilation conditionnelle du serial handler: US100. MB7389, None
  #define PUMPMOTORDETECT false       // Pour compilation conditionnelle de la routin e d'interruption
  #define HASDS18B20SENSOR false      // Pour le code spécifique au captgeur de température DS18B20
  #define HASHEATING false            // Pour le chauffage du boitier
  #define HASVACUUMSENSOR false       // Un capteur de vide est installé
  #define HASVALVES true              // Des valves sont relié à ce capteur
  #define HASRELAYOUTPUT false        // Un relais SSR peut être relié à ce capteur
  #define HASUS100THERMISTOR false    // Un thermistor est présent pour mesurer la température du boitier US100 robuste
  #define baseSampling  1             // basic sampling interval for main loop
  /*Config for RS5, RS6 -> DEVICE_CONF == 5
  DEVICE   DISTANCESENSOR  PUMPMOTORDETECT HASDS18B20SENSOR HASHEATING HASVACUUMSENSOR HASVALVES HASRELAYOUTPUT HASUS100THERMISTOR
  RS5       US100           false           false            false      false           true      false          false
  RS6       US100           false           false            false      false           true      false          false
  */

#elif (DEVICE_CONF == 6)
  String config = "6 -> RHC";         // String info de configuration
  #define DISTANCESENSOR MB7389       // Pour compilation conditionnelle du serial handler: US100. MB7389, None
  #define PUMPMOTORDETECT false       // Pour compilation conditionnelle de la routin e d'interruption
  #define HASDS18B20SENSOR true       // Pour le code spécifique au captgeur de température DS18B20
  #define HASHEATING true             // Pour le chauffage du boitier
  #define HASVACUUMSENSOR false       // Un capteur de vide est installé
  #define HASVALVES false             // Des valves sont relié à ce capteur
  #define HASRELAYOUTPUT false        // Un relais SSR peut être relié à ce capteur
  #define HASUS100THERMISTOR false    // Un thermistor est présent pour mesurer la température du boitier US100 robuste
  #define baseSampling  2             // basic sampling interval for main loop. Very slow to avoid echo
  /*Config for RHC -> DEVICE_CONF == 6
  DEVICE   DISTANCESENSOR  PUMPMOTORDETECT HASDS18B20SENSOR HASHEATING HASVACUUMSENSOR HASVALVES HASRELAYOUTPUT HASUS100THERMISTOR
  RHC       MB7389          false           true(1)          true       false           false     false          false
  */

#elif (DEVICE_CONF == 7)
  String config = "7 -> VEcTk";       // String info de configuration
  #define DISTANCESENSOR NONE         // Pour compilation conditionnelle du serial handler: US100. MB7389, None
  #define PUMPMOTORDETECT false       // Pour compilation conditionnelle de la routin e d'interruption
  #define HASDS18B20SENSOR true       // Pour le code spécifique au captgeur de température DS18B20
  #define HASHEATING true             // Pour le chauffage du boitier
  #define HASVACUUMSENSOR false       // Un capteur de vide est installé
  #define HASVALVES true              // Des valves sont relié à ce capteur
  #define HASRELAYOUTPUT false        // Un relais SSR peut être relié à ce capteur
  #define HASUS100THERMISTOR false    // Un thermistor est présent pour mesurer la température du boitier US100 robuste
  #define baseSampling  1             // basic sampling interval for main loop
  /*Config for VEcTk -> DEVICE_CONF == 7
  DEVICE   DISTANCESENSOR  PUMPMOTORDETECT HASDS18B20SENSOR HASHEATING HASVACUUMSENSOR HASVALVES HASRELAYOUTPUT HASUS100THERMISTOR
  VEcTk     NONE            false           true(1)          true       false           true      false          false
  */

#elif (DEVICE_CONF == 8)
  String config = "8 -> Dummy";       // String info de configuration
  #define DISTANCESENSOR NONE         // Pour compilation conditionnelle du serial handler: US100. MB7389, None
  #define PUMPMOTORDETECT false       // Pour compilation conditionnelle de la routin e d'interruption
  #define HASDS18B20SENSOR false      // Pour le code spécifique au captgeur de température DS18B20
  #define HASHEATING false            // Pour le chauffage du boitier
  #define HASVACUUMSENSOR false       // Un capteur de vide est installé
  #define HASVALVES true              // Des valves sont relié à ce capteur
  #define HASRELAYOUTPUT false        // Un relais SSR peut être relié à ce capteur
  #define HASUS100THERMISTOR false    // Un thermistor est présent pour mesurer la température du boitier US100 robuste
  #define baseSampling  1             // basic sampling interval for main loop
  /*Config for DummyDevice -> DEVICE_CONF == 8
  DEVICE   DISTANCESENSOR  PUMPMOTORDETECT HASDS18B20SENSOR HASHEATING HASVACUUMSENSOR HASVALVES HASRELAYOUTPUT HASUS100THERMISTOR
  Dummy     NONE            false           false            false      false           true      false          false
  */

#else
  #error Invalid device configuration!
#endif

/********* Fin de la configuration pour compilation *********/

/* Assignation des variables exposées dans le nuage */
String distSensorName = "NONE";
bool motorInput = PUMPMOTORDETECT;
bool ds18Sensor = HASDS18B20SENSOR;
bool hasHeatingRes = HASHEATING;
bool hasVacSensor = HASVACUUMSENSOR;
bool hasValveInput = HASVALVES;
bool hasRelayOutput = HASRELAYOUTPUT;
bool hasUs100Thermistor = HASUS100THERMISTOR;

// General definitions
#define second 1000UL             // 1000 millisecond per sesond
#define minute 60000UL            // 60000 millisecond per minute
#define heure 3600000UL           // 3600000 millisecond par heure
#define unJourEnMillis (24 * 60 * 60 * second)
#define valveDebounceDelay 50     // Debounce time in milliseconds for valve position readswitch
#define pumpDebounceDelay 50      // Debounce time in milliseconds for pump mechanical start/stop switch
#define fastSampling  6000UL      // in milliseconds
#define slowSampling  10000UL     // in milliseconds
#define numReadings 10            // Number of readings to average for filtering
#define minDistChange 2.0 * numReadings      // Minimum change in distance to publish an event (1/16")
#define minTempChange 0.5 * numReadings      // Minimum temperature change to publish an event
#define minVacuumChange 0.1       // Changement de 0.1 Po Hg avant publication du niveau de vide
#define minVacuumForCoulee -5.0   // Minimum value a vacuum required to consider a pump operation as beginning a coulee
#define maxRangeUS100 3000        // Distance maximale valide pour le captgeur
#define maxRangeMB7389 1900       // Distance maximale valide pour le captgeur
#define ONE_WIRE_BUS D4           // senseur sur D4
#define DallasSensorResolution 10 // Résolution de lecture de température
#define MaxHeatingPowerPercent 80 // Puissance maximale appliqué sur la résistance de chauffage
#define HeatingSetPoint 25        // Température cible à l'intérieur du boitier
#define DefaultPubDelay 5         // Interval de publication en minutes par défaut
#define TimeoutDelay 6 * slowSampling // Device watch dog timer time limit
// #define pumpRunTimeLimit 2 * minute // Maximum pump run time before a warning is issued
// #define pumpMinRunTime 17 * second // Ignore cycles where pump is running less than 17 seconds
#define delaisFinDeCoulee 3 * heure // Temps sans activité de la pompe pour décréter la fin de la couléé
#define pumpONstate 0             // Pump signal is active low.
#define pumpOFFstate 1            // Pump signal is active low.
#define StartDSTtime 1552197600   // dim 10 mar 2019, 02 h 00 = 1552197600 local time
#define EndDSTtime 1572850800     // dim 4 nov 2019, 02 h 00 = 1572850800 local time
#define MB7389_pin2 RX            // MB7389 pulse pin (pin 2) connected to serial1 Rx

// Definition for vacuum transducer
#define R1 18000                    // pressure xducer scaling resistor 1
#define R2 36000                    // Pressure xducer scaling resistor 2
#define Vref 3.3                    // Analog input reference voltage
#define K_fact 0.007652             // Vacuum xducer K factor
#define Vs 5.0                      // Vacuum xducer supply voltage

// Nom des indices du tableau eventName
#define evPompe_T1 0
#define evPompe_T2 1
#define evDistance 2
#define evUS100Temperature 3
#define evOutOfRange 4
#define evPumpEndCycle 5            // Événement fin d'un cycle de pompe
#define evRunTooLong 6              // Alerte Pompe en marche depuis plus de 5 min. Inutilisé.
#define evDebutDeCoulee 7           // Début de coulée. Inutilisé.
#define evFinDeCoulee 8             // Fin de coulée. 
#define evRelais 9
#define evVacuum 10
#define evFlowmeterFlow 11
#define evFlowmeterVolume 12
#define evAtmosPressure 13
#define evEnclosureTemp 14
#define evAmbientTemp 15
#define evHeatingPowerLevel 16
#define evNewGenSN 17
#define evBootTimestamp 18
#define evValve1_Position 19
#define evValve2_Position 20
#define evPumpCurrentState 21
#define evCurrentDutyCycle 22
#define evPompe_T1_OFFtime 23
#define evPompe_T2_ONtime 24

// Table des nom d'événements
String eventName[] = {
  "pump/T1",                        // Pump state. Pump start
  "pump/T2",                        // Pump state. Pump stop
  "sensor/level",                   // Tank level. Post processing required for display
  "sensor/US100sensorTemp",         // Temperature read on the US100 senson
  "sensor/outOfRange",              // Level sensor is out of max range
  "pump/endCycle",                  // Indicate a pump start/stop cycle is completed
  "pump/warningRunTooLong",         // Dummy event Place holder
  "pump/debutDeCoulee",             // Dummy event Place holder
  "pump/finDeCoulee",               // Dummy event Place holder
  "output/ssrRelayState",           // Output ssrRelay pin state. Active LOW
  "sensor/vacuum",                  // Vacuum rsensor eading
  "sensor/flowmeterValue",          // Flowmeter reading. Not used
  "computed/flowmeterVolume",       // Volume computed from flowmert readings. Not used
  "sensor/atmPressure",             // Atmospheric pressure
  "sensor/enclosureTemp",           // Temperature inside device enclosure.
  "sensor/ambientTemp",             // Ambient temperature read by remote probe.
  "output/enclosureHeating",        // Value of PWM output to heating resistor.
  "device/NewGenSN",                // New generation of serial numbers for this device
  "device/boot",                    // Device boot or reboot timestamp
  "sensor/Valve1Pos",               // Valve 1 position string
  "sensor/Valve2Pos",               // Valve 2 position string
  "pump/state",                     // Étant actuel de la pompe
  "pump/CurrentDutyCycle",          // Étant actuel du dutyCycle
  "pump/T1_OFFtime",                // Durée de marche de la pompe en millisecondes
  "pump/T2_ONtime"                  // Durée de marche de la pompe en millisecondes
  };

// Structure définissant un événement
struct Event{
  uint32_t noSerie; // Le numéro de série est généré automatiquement
  time_t timeStamp; // Timestamp du début d'une génération de noSerie.
  uint32_t timer;   // Temps depuis la mise en marche du capteur. Overflow après 49 jours.
  uint16_t namePtr; // Pointeur dans l'array des nom d'événement. (Pour sauver de l'espace NVRAM)
  unsigned int eData; // Données pour cet événement. Entier 32 bits. Pour sauvegarder des données en point flottant
                    // multiplié d'abord la donnée par un facteur (1000 par ex.) et convertir en entier.
                    // Il suffira de divisé la données au moment de la réception de l'événement.
};

// Variable relié à l'opération du buffer circulaire
const int buffSize = 151; // Nombre max d'événements que l'on peut sauvegarder
retained struct Event eventBuffer[buffSize];
retained unsigned int buffLen = 0;
retained unsigned int writePtr = 0;
retained unsigned int readPtr = 0;
retained unsigned int replayPtr = 0;
unsigned int replayBuffLen = 0;
retained unsigned int savedEventCount = 0;

// Name space utilisé pour les événements
// DomainName/DeptName/FunctionName/SubFunctionName/ValueName
String DomainName = "";
String DeptName = "";

// Pin pour l' I/O
int RGBled_Red = D0;
int RGBled_Green = D1;
int RGBLed_Blue = D2;
int led = D7;                     // Feedback led
int ssrRelay = D6;                // Solid state relay
int RelayState = false;
int motorState = A1;              // input pour Pompe marche/arrêt
int heater = D3;                  // Contrôle le transistor du chauffage
#if HASVACUUMSENSOR
  int VacuumSensor = A0;          // Analogue input pour la mesure du vide
  float VacCalibration = 0;       // Variable contenant la valeur de calibration du capteur.
  double VacAnalogvalue = 0;      // Mesure du vide
  double prev_VacAnalogvalue = 0; // Mesure précédente du vide
#endif

// Mesure du signal WiFi
int rssi = 0;

// Variables liés à la pompe
bool PumpOldState = pumpOFFstate;          // Pour déterminer le chanement d'état
bool PumpCurrentState = pumpOFFstate;      // Initialize pump in the OFF state
bool PumpWarning = false;                  // Alerte pour indiquer que la pompe fonctionne trop longtemps
bool couleeEnCour = false;                 // État de la coulée
volatile unsigned long changeTime = 0;     // Moment du dernier changement d'état de la pompe
volatile unsigned long T0 = millis();      // Début d'un cycle. (pompe arrête)
volatile unsigned long T1 = T0 + 200000000; // Milieu d'un cycle. (pompe démarre)
volatile unsigned long T2 = T1 + 20000;    // Fin d'un cycle. (pompe arrête), égale T0 du cycle suivant.
volatile unsigned long T_ON = 0;           // Pump ON time
volatile unsigned long T_OFF = 0;          // Pump OFF time
unsigned long T_Cycle = 100000UL;          // Pump cycle time
double dutyCycle = 0;                      // Duty cycle du dernier cycle de la pompe.

// Variables liés aux valves
int ValvePos_pin[] = {A5, A4, A3, A2};
bool ValvePos_state[] = {true, true, true, true};
// int ValvePos_Name[] = {evValve1_OpenSensorState, evValve1_CloseSensorState, evValve2_OpenSensorState, evValve2_CloseSensorState};

// Variables liés à la mesure de Température
unsigned int US100HighByte = 0;
unsigned int US100LowByte  = 0;
int TempUS100 = 0;
int TempThermUS100 = 0;
int prev_TempUS100 = 0;
int varTempUS100 = 0;
int allTempReadings[numReadings];

#if HASDS18B20SENSOR
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature ds18b20Sensors(&oneWire);
  DeviceAddress enclosureThermometer, outsideThermometer;
  int ds18b20Count = 0;
  bool validTempExterne = false;
  bool validEnclosureTemp = false;
  double prev_EnclosureTemp = 99;
  double prev_TempExterne = 99;
#endif

int HeatingPower = 0;
int prev_HeatingPower = 64;

// Variables liés à la mesure de distance
int dist_mm  = 0;
int prev_dist_mm = 0;
int rawDistmm = 0;
int allDistReadings[numReadings];

// Variables liés au temps
unsigned long lastPublish = millis();
unsigned long lastAllPublish = 0;
unsigned long lastRTCSync = millis();
unsigned int samplingInterval = fastSampling;
unsigned int samplingIntervalCnt = 4;
unsigned long timeLastUnit = 0;
unsigned long lastRunWarning = millis();

// Variables liés aux publications
char publishString[buffSize];
retained time_t newGenTimestamp = 0;
retained uint32_t noSerie = 0; // Mettre en Backup RAM
int maxPublishInterval = DefaultPubDelay;
volatile unsigned long maxPubDelay_ms = maxPublishInterval * minute;
int pumpEvent = 0;
bool connWasLost = false;


// Variable associé au capteur MB7389 seulement
#if DISTANCESENSOR == MB7389
  bool MB7389Valid = false;
  String Dist_MB7389Str;
  unsigned int MB7389latestReading = 0;
  const int R = 82;
  const int CR = 13;
  long MB7389_pulse;
#endif

/*
   handler to receive the module name
*/
String myDeviceName = "";
IPAddress deviceIP;
char myDeviceIP[20] = "";
String myIpString = "";

void nameHandler(const char *topic, const char *data) {
  myDeviceName =  String(data);
  /*Log.info("received " + String(topic) + ": " + String(data));*/
}

#if PUMPMOTORDETECT
/*
  Attach interrupt handler to pin A1 to monitor pump Start/Stop
*/
  class PumpState_A1 {
    public:
      PumpState_A1() {
        pinMode(A1, INPUT);
        attachInterrupt(A1, &PumpState_A1::A1Handler, this, CHANGE);
      }
      void A1Handler() {
        // IMPORTANT: Pump is active LOW. Pump is ON when PumpCurrentState == false
        delayMicroseconds(pumpDebounceDelay * 1000);
        PumpCurrentState = digitalRead(A1);
        changeTime = millis();
      }
  };

  PumpState_A1 ThePumpState; // Instantiate the class A1State
#endif

#if HASDS18B20SENSOR
// function to print a device address
  void printAddress(DeviceAddress deviceAddress){
    for (uint8_t i = 0; i < 8; i++)
    {
      if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
#endif

// Declare the watchdog timer
ApplicationWatchdog wd(TimeoutDelay, System.reset);

/* Define a log handler on Serial1 for log messages */
SerialLogHandler logHandler(115200, LOG_LEVEL_TRACE, {   // Logging level for non-application messages
    { "app", LOG_LEVEL_INFO }                      // Logging level for application messages
});

/*******************************************************************************
    Setup Routine
*******************************************************************************/
void setup() {
// Initialisation des pin I/O
  RGB.mirrorTo(RGBled_Red, RGBled_Green, RGBLed_Blue, true);
  delay(3000UL); // Pour partir le moniteur série pour début
  WiFi.disconnect();

  pinMode(led, OUTPUT);
  pinMode(MB7389_pin2, INPUT);
  digitalWrite(led, HIGH); // Mettre le led de status à OFF
  pinMode(ssrRelay, OUTPUT);
  #if HASHEATING
  pinMode(heater, OUTPUT);
  HeatingPower =  0; // Valeur de PWM de chauffage
  analogWrite(heater, HeatingPower, 500); // Désactiver le chauffage
  #endif
  digitalWrite(ssrRelay, LOW);

  PumpCurrentState = digitalRead(A1);
  for (int i=0; i <= 3; i++) {
      pinMode(ValvePos_pin[i], INPUT_PULLUP);
  }

  for (int i = 0; i < numReadings; i++){ // Init readings array
      allDistReadings[i] = 0;
      allTempReadings[i] = 0;
  }

#if HASVACUUMSENSOR
  VacCalibration = VacCalibre();
#endif

// connect RX to Echo/Rx (US-100), TX to Trig/Tx (US-100)
  Serial.begin(115200); // Pour débug
  // #if DISTANCESENSOR == MB7389
  //     Serial1.begin(9600);  // Le capteur US-100 fonctionne à 9600 baud
  // #endif
// Enregistrement des fonctions et variables disponible par le nuage
  Log.info("(setup) Enregistrement des variables et fonctions\n");
  Particle.variable("Version", FirmwareVersion);
  Particle.variable("Date", FirmwareDate);
  Particle.variable("config", config);
  Particle.variable("hasDistance", distSensorName);
#if (DISTANCESENSOR == US100 || DISTANCESENSOR == MB7389)
  Particle.variable("rawDistance", rawDistmm);
#endif
#if (DISTANCESENSOR == US100)
  Particle.variable("TempUS100", varTempUS100);
#endif
  Particle.variable("hasMotorIn", motorInput);
  Particle.variable("hasDs18b20", ds18Sensor);
#if HASDS18B20SENSOR
  Particle.variable("DS18B20Cnt", ds18b20Count);
  Particle.variable("caseTemp", prev_EnclosureTemp);
  Particle.variable("outdoorTemp",prev_TempExterne);
#endif
  Particle.variable("hasHeating", hasHeatingRes);
  Particle.variable("hasVacuum", hasVacSensor);
  Particle.variable("hasValves", hasValveInput);
  Particle.variable("hasRelayOut", hasRelayOutput);
  Particle.variable("hasThermist", hasUs100Thermistor);
#if HASUS100THERMISTOR
  Particle.variable("thermistor", TempThermUS100);
#endif

  Particle.variable("relayState", RelayState);
  Particle.variable("rssi", rssi);
  Particle.variable("myIPaddress", myDeviceIP);
// Fonctions disponible dans le nuage
  Particle.function("relay", toggleRelay);
  Particle.function("set", remoteSet);
  Particle.function("reset", remoteReset);
  Particle.function("replay", replayEvent);

#if DISTANCESENSOR == MB7389
  //   Serial1.halfduplex(true); // Ce capteur envoie seulement des données sésie dans une seule direction
                              // On peut utiliser la pin RX pour contrôler son fonctionnement
                              // RX = LOW pour arrêter le capteur, RX = HIGH pour le démarrer
  distSensorName = "MB7389";
  Log.info(" $$$$$$$ Set distSensorName to MB7389");
#endif

  Log.info("Configuration du wifi... ");
  WiFi.setCredentials("BoilerHouse", "Station Shefford");
  WiFi.setCredentials("PumpHouse", "Station Laporte");
  Log.info("Connexion au wifi... ");
  WiFi.connect();

// Attendre la connection au nuage
  Log.info("En attente...");
  Particle.connect();
  if (waitFor(Particle.connected, 10000)) {
    delay(1000);
    Serial.print(".");
  }
  if(Particle.connected()){
    Log.info("Connecté au nuage. :)");
    Particle.syncTime();
    if (not(Time.isValid()) || Time.year() < 2019) {
      Log.info("(setup) Syncing time ");
      Particle.syncTime();
      waitUntil(Particle.syncTimeDone);
      Log.info("(setup) syncTimeDone " + Time.timeStr());
      newGenTimestamp = Time.now();
    }
    deviceIP = WiFi.localIP();
    sprintf(myDeviceIP, "%d.%d.%d.%d", deviceIP[0], deviceIP[1], deviceIP[2], deviceIP[3]);
  } else {
    Log.info("Pas de connexion au nuage. :( ");
  }

  delay(1000UL);
  if (savedEventCount == 0){ // If there's no accumulated event
    remoteReset("serialNo"); // the non
  }

// check that all buffer pointers are ok.
  checkPtrState();
  replayBuffLen = 0;
  replayPtr = readPtr;

// initialisation des capteurs de températures
#if HASDS18B20SENSOR
  initDS18B20Sensors();
#endif
// show position of valves initially
#if HASVALVES
  CheckValvePos(true);
#endif

  Time.zone(-5);
  Time.setFormat(TIME_FORMAT_ISO8601_FULL);
  Particle.syncTime();
  pushToPublishQueue(evBootTimestamp, 0,  millis());
  #if (DEVICE_CONF == 0) // Événement pour les pompes 1 2 et 3 seulement
  pushToPublishQueue(evPumpEndCycle, 1, millis());
  pushToPublishQueue(evFinDeCoulee, false, millis());
  #endif

  // PhotonWdgs::begin(true, true, TimeoutDelay, TIMER7);
  lastPublish = millis(); // Initialise le temps initial de publication
  changeTime = lastPublish; // Initialise le temps initial de changement de la pompe
}

/*******************************************************************************
    Boucle principale
*******************************************************************************/
void loop(){
  unsigned long loopTime = millis();
// Execute every nextSampleTime(6 seconds)) read all sensors and reset watchdog
  if (loopTime - timeLastUnit >= (baseSampling * second)){
    digitalWrite(led, LOW); // Pour indiqué le début de la prise de mesure
    // PhotonWdgs::tickle(); // Reset watchdog
    readSelectedSensors(samplingIntervalCnt); // 
    PublishAll(); // Check if theres something on the publish queue
    /*delay(20UL);  // Just to have a visible flash on the LED*/
    digitalWrite(led, HIGH); // Pour indiqué la fin de la prise de mesure
    timeLastUnit = loopTime;
    /*Log.info("*** Loop No. %d end. Loop time is:<< %d >>", samplingIntervalCnt, millis() - loopTime);*/
    if (samplingIntervalCnt == 5){
      samplingIntervalCnt = 0;
    } else {
      samplingIntervalCnt++;
    }
  }
  delay(200UL);
}

/*******************************************************************************
    PublishAll routine
*******************************************************************************/
void PublishAll(){
  // Publie au moins une fois à tous les "maxPubDelay_ms" millisecond
  unsigned long now = millis();
   #if (DEVICE_CONF == 0) // Événement pour les pompes P1,P2 et P3 seulement
    static unsigned long Old_T1 = 0;
    static unsigned long Old_T2 = 0;
  #endif

  #if HASVALVES
    CheckValvePos(false); // publish if valve state changed
  #endif

  #if HASHEATING
    simpleThermostat(HeatingSetPoint); // Control the enclosure heation
  #endif

  #if PUMPMOTORDETECT 
  // À faire lors d'un changement dans l'état de la pompe
  if (PumpCurrentState != PumpOldState)
  {
    // 
    // À faire quand la pompe se met en marche (Pump ON)
    // 
    if (PumpCurrentState == pumpONstate){
      pumpEvent = evPompe_T1;
      #if (DEVICE_CONF == 0) // Événement pour les pompes P1,P2 et P3 seulement
        // S'assurer qu'il n'y a pas eu d'overflow des compteurs
        // Si il n'y a pas de coulée en cour et que la pompe à vide est en fonction -> débuté la coulée
        if (!couleeEnCour && prev_VacAnalogvalue < minVacuumForCoulee)
        {
          couleeEnCour = true;
            pushToPublishQueue(evDebutDeCoulee, couleeEnCour, changeTime);
        }
        Old_T1 = T1;
      #endif
      T1 = changeTime;
    } else {
      // 
      // À faire quand la pompe s'arrête (Pump OFF)
      // Un cycle se termine (T2) et un autre commence (T0)
      // 
      pumpEvent = evPompe_T2;
      #if (DEVICE_CONF == 0) // Événement pour les pompes P1,P2 et P3 seulement
      Old_T2 = T2;
      #endif
      T2 = changeTime;
      // S'assurer qu'il n'y a pas eu d'overflow des compteurs avant de calculer le dutyCycle
      // et que le temps de marche de la pompe est acceptable
      if ((T2 > T1 && T1 > T0) && (T2 - T1 > pumpMinRunTime) && (T2 - T1 < pumpRunTimeLimit)) {
        T_ON = (T2 - T1);     // Temps de marche de la pompe
        T_OFF = (T1 - T0);
        T_Cycle = (T2 - T0);  // Temps de cycle total.
        // Calculer le dutyCycle si la pompe n'est pas resté en marche trop longtemps.
        dutyCycle = (float)T_ON / (float)T_Cycle; // In case of overflow of T1 or T2, assume the dutycycle did not changed.
        pushToPublishQueue(evPompe_T1_OFFtime, T_OFF, now);
        pushToPublishQueue(evPompe_T2_ONtime, T_ON, now);
        Log.info("(PublishAll) - T_OFF: %lu ms", T_OFF);
        Log.info("(PublishAll) - T_ON: %lu ms", T_ON);
        Log.info("(PublishAll) - dutyCycle: %f", dutyCycle);
        Log.info("T0= %lu, T1= %lu, T2= %lu, dutyCycle : %.3f", T0, T1, T2, dutyCycle);
        T0 = T2;
        #if (DEVICE_CONF == 0) // Événement pour les pompes P1,P2 et P3 seulement
          pushToPublishQueue(evPumpEndCycle, (int)(dutyCycle * 1000), changeTime);
        #endif
      } else 
      {
        #if (DEVICE_CONF == 0) // Événement pour les pompes P1,P2 et P3 seulement
          T1 = Old_T1;
          T2 = Old_T2;
        #endif
      }
    }

    pushToPublishQueue(pumpEvent, PumpCurrentState, changeTime);
    // pushToPublishQueue(evPumpCurrentState, PumpCurrentState, changeTime);
    PumpOldState = PumpCurrentState;
  }  

  // Si la coulée est en cour ET la pompe est arrêté depuis plus longtemps que le délais établit (3h)
  // alors marque la coulée comme arrêté et émettre un événement de fin de coulée.
  #if (DEVICE_CONF == 0) // Événement pour les pompes P1,P2 et P3 seulement
    // Occasionnellement T2 est plus grand que now parse que la valeur de T2 est changé par une interruption 
    if (now  > T2) {
      bool delaisExpire = ((now - T2) > delaisFinDeCoulee);
      if (couleeEnCour && (PumpCurrentState == pumpOFFstate) && delaisExpire){
        Log.printf("Condition en fin de coulée - couleeEnCour: %s, pumpState: %s, now: %lu, T2: %lu, delaisExpire: %s\n", 
                  couleeEnCour ? "true" : "false", PumpCurrentState ? "true" : "false", now, T2, delaisExpire ? "true" : "false");
        couleeEnCour = false;
        pushToPublishQueue(evFinDeCoulee, couleeEnCour, now);
      }
    }

  // Publier un avertissement si la pompe fonctionne depuis trop longtemps
    if (PumpCurrentState == pumpONstate && ((now - lastRunWarning) > 1 * minute)){
      if (now > T1){
        unsigned long pumpRunTime = now - T1;
        if (pumpRunTime > pumpRunTimeLimit){
          // Publish an event with the number of seconds the pump is running
          pushToPublishQueue(evRunTooLong, (int)(pumpRunTime / 1000UL), now);
          lastRunWarning = now;
        }
      }
    }
  #endif

  #endif
  // Publish all every maxPubDelay_ms
  if (now - lastAllPublish > maxPubDelay_ms){

    Log.info("(PublishAll) - Time to publish: (now - lastAllPublish)= %lu maxPubDelay_ms= %lu", now - lastAllPublish, maxPubDelay_ms);
    lastAllPublish = now;

    #if DISTANCESENSOR == US100
      pushToPublishQueue(evDistance, (int)(dist_mm / numReadings), now);
      pushToPublishQueue(evUS100Temperature, (int)(TempUS100/ numReadings), now);
    #endif

    #if DISTANCESENSOR == MB7389
      pushToPublishQueue(evDistance, (int)(dist_mm / numReadings), now);
    #endif

    #if HASDS18B20SENSOR
      if (ds18b20Count == 1){
          pushToPublishQueue(evEnclosureTemp, (int)prev_EnclosureTemp, now);
      } else if (ds18b20Count == 2){
          pushToPublishQueue(evEnclosureTemp, (int)prev_EnclosureTemp, now);
          pushToPublishQueue(evAmbientTemp, (int)prev_TempExterne, now);
      }
    #endif

    #if HASVACUUMSENSOR
        pushToPublishQueue(evVacuum, (int)(prev_VacAnalogvalue * 100), now);
    #endif

    #if HASVALVES
      CheckValvePos(true);
    #endif

    #if PUMPMOTORDETECT
      // Pour s'assurer que le dutycycle ne reste pas indéfiniment plus grand que zero à la fin d'une coulée
      // Si la coulée est toujours en cour et le compteur de temps ne fait pas d'overflow et que la pompe est arrêté
      // depuis plus longtemps que le cycle précédent alors on recalcule le dutycycle et on publie un nouvel événement.
      // On assume que le temps ON de la pompe est le même qu'au cycle précédent.
      Log.info("(PublishAll) - Coulée en cour: %d, PumpState: %d, T0: %lu, T1: %lu, T_ON: %lu, T_cycle: %lu", couleeEnCour, PumpCurrentState, T0, T1, T_ON, T_Cycle);
      if (couleeEnCour && (PumpCurrentState == pumpOFFstate) && (now > T0) && ((now - T0 + T_ON) > T_Cycle))
      {
        float T_Cycle_est = (now - T0 + T_ON);    // Temps de cycle estimé
        dutyCycle = (float)T_ON / T_Cycle_est;
        if (dutyCycle < 0.005) {dutyCycle = 0;}  // Mettre à zéro si inférieur à 0.5%
        #if (DEVICE_CONF == 0) // Événement pour les pompes P1 ,P2 et P3 seulement
          pushToPublishQueue(evCurrentDutyCycle, (int)(dutyCycle * 1000), now);
        #endif
      }
      // Publication de l'état de la pompe à interval
      pushToPublishQueue(evPumpCurrentState, PumpCurrentState, changeTime);
    #endif
  }

// Synchronisation du temps avec Particle Cloud une fois par jour
  if (millis() - lastRTCSync > unJourEnMillis) {
      Particle.syncTime();
      Log.info("Synchronisation du temps avec le nuage de Particle");
      lastRTCSync = millis();
      if (!Time.isDST() && Time.now() >= StartDSTtime){
        Time.beginDST();
        Log.info("Début de l'heure avancé");
      } else if (Time.isDST() && Time.now() >= EndDSTtime){
        Time.endDST();
        Log.info("Fin de l'heure avancé");
      }
  }
// Publier les événements se trouvant dans le buffer
  if(buffLen > 0){
    // Log.info("BufferLen = %u, Cloud = %s", buffLen, (Particle.connected() ? "true" : "false")); // Pour debug
    publishQueuedEvents();
    // bool success = publishQueuedEvents();
    // Log.info("Publishing = %u, Status: %s", readPtr - 1, (success ? "Fait" : "Pas Fait")); // Pour debug
  } else if (replayBuffLen > 0){
    replayQueuedEvents();
    // bool success = replayQueuedEvents();
    // Log.info("replayBuffLen = %u, Replay = %u, Status: %s", replayBuffLen, replayPtr, (success ? "Fait" : "Pas Fait"));
  }
}

// *******************************************************************************
//  readSelectedSensors routine
//  Read the sensors attached to the device
//  If no sensor is present, the compiler substitute a delay of 20ms to get a visible flash on the activity led
// *******************************************************************************
void readSelectedSensors(int sensorNo) {
  // Log.info("readSelectedSensors) - Now reading sensorNo: %d", sensorNo);

  switch (sensorNo)
  {
    case 0:
    // Measure the distance
      #if DISTANCESENSOR == US100
        ReadDistance_US100();
        Particle.process();
      #else
        delay(20UL);  // Just to have a visible flash on the LED
      #endif

      #if DISTANCESENSOR == MB7389
        ReadDistance_MB7389();
        Particle.process();
      #else
        delay(20UL);  // Just to have a visible flash on the LED
      #endif
      break;

    case 1:
    // Measure temperature with DS10b20 sensors
      #if HASDS18B20SENSOR
        readDS18b20temp();
        Particle.process();
      #else
        delay(20UL);  // Just to have a visible flash on the LED
      #endif
      break;

    case 2:
    // Read US100 sensor temperature
      #if DISTANCESENSOR == US100
        Readtemp_US100(); // 
        Particle.process();
      #else
        delay(20UL);  // Just to have a visible flash on the LED
      #endif
      break;

    case 3:
      #if HASVACUUMSENSOR
        VacReadVacuumSensor(); // Read vacuum
        Particle.process();
      #endif
      delay(20UL);  // Just to have a visible flash on the LED
      break;

    case 4:
    // And maybe the US100 enclosure thermistor
      #if HASUS100THERMISTOR
        ReadTherm_US100();
        Particle.process();
      #else
        delay(20UL);  // Just to have a visible flash on the LED
      #endif
      break;

    case 5:
      rssi = WiFi.RSSI();
      delay(20UL);  // Just to have a visible flash on the LED
      break;

    // default:
    // Pour permettre la modification de maxPubDelay_ms par le nuage
      maxPubDelay_ms = maxPublishInterval * minute;
  }
}

#if HASDS18B20SENSOR
  void initDS18B20Sensors(){
    // Configuration des capteurs de température DS18B20
    float insideTempC, outsideTempC;
    int D3res = analogWriteResolution(heater);
    int D3maxFreq = analogWriteMaxFrequency(heater);
    unsigned long now = millis();
    Log.info("(initDS18B20Sensors) - Resolution and Frequency for pin 'heater': %d bits, maxFreq: %d Hz", D3res, D3maxFreq);

    int j = 0;
    do
    {
      ds18b20Sensors.begin();
      delay(500);
      ds18b20Count = ds18b20Sensors.getDeviceCount();
      j++;
    } while (ds18b20Count == 0 && j < 5);

    ds18b20Sensors.setWaitForConversion(true);
    Log.info("(initDS18B20Sensors) - DS18B20 found: %d after %d try.", ds18b20Count, j);

    if (ds18b20Count == 1){
      Log.info("(initDS18B20Sensors) - Configuration de 1 ds18b20");

      ds18b20Sensors.getAddress(enclosureThermometer, 0);
      printAddress(enclosureThermometer);
      ds18b20Sensors.setResolution(enclosureThermometer, DallasSensorResolution);
      Log.info("(initDS18B20Sensors) - Device 0 Resolution: %d", ds18b20Sensors.getResolution(enclosureThermometer));
      ds18b20Sensors.requestTemperaturesByAddress(enclosureThermometer); // requête de lecture
      insideTempC = ds18b20Sensors.getTempC(enclosureThermometer);
      Log.info("(initDS18B20Sensors) - Test device 0 enclosureThermometer = %f", insideTempC);

    } else if (ds18b20Count == 2){
      Log.info("(initDS18B20Sensors) - Configuration de 2 ds18b20");

      ds18b20Sensors.getAddress(enclosureThermometer, 0); // capteur Index 0
      printAddress(enclosureThermometer);
      ds18b20Sensors.setResolution(enclosureThermometer, DallasSensorResolution);
      Log.info("(initDS18B20Sensors) - Device 0 Resolution: %d", ds18b20Sensors.getResolution(enclosureThermometer));
      ds18b20Sensors.requestTemperaturesByAddress(enclosureThermometer); // requête de lecture
      insideTempC = ds18b20Sensors.getTempC(enclosureThermometer);
      Log.info("(initDS18B20Sensors) - Test device 0 enclosureThermometer = %f", insideTempC);

      ds18b20Sensors.getAddress(outsideThermometer, 1); // capteur Index 1
      printAddress(outsideThermometer);
      ds18b20Sensors.setResolution(outsideThermometer, DallasSensorResolution);
      Log.info("(initDS18B20Sensors) - Device 1 Resolution: %d", ds18b20Sensors.getResolution(outsideThermometer));
      ds18b20Sensors.requestTemperaturesByAddress(outsideThermometer); // requête de lecture
      outsideTempC = ds18b20Sensors.getTempC(outsideThermometer);
      Log.info("(initDS18B20Sensors) - Test device 1 outsideThermometer = %f", outsideTempC);

      pushToPublishQueue(evEnclosureTemp, (int) insideTempC, now);
      pushToPublishQueue(evAmbientTemp, (int) outsideTempC, now);
    }
    Log.info("(initDS18B20Sensors) - ");
    delay(2000UL);
  }
#endif

#if DISTANCESENSOR == US100
// Cette routine mesure la distance entre la surface de l'eau et le capteur ultason
  void ReadDistance_US100(){
    unsigned long now = millis();
      distSensorName = "US100";
      unsigned int currentReading;
      Log.info("(ReadDistance_US100) - Distance meas routine: ReadDistance_US100");
      Serial1.begin(9600);  // Le capteur US-100 fonctionne à 9600 baud
      Serial1.write(0X55);                            // trig US-100 begin to measure the distance
      delay(60UL);                                     // delay 20ms to wait result
      if(Serial1.available() >= 2)                    // when receive 2 bytes
      {
          US100HighByte = Serial1.read();                   // High byte of distance
          US100LowByte  = Serial1.read();                   // Low byte of distance
          currentReading = US100HighByte*256 + US100LowByte;          // Combine the two bytes
  //        Len_mm  = (US100HighByte*256 + US100LowByte)/25.4;    // Calculate the distance in inch
          rawDistmm = currentReading;
          if((currentReading > 1) && (currentReading < maxRangeUS100)){       // normal distance should between 1mm and 2500 mm (1mm, 2,5m)
              dist_mm = AvgDistReading(currentReading); // Average the distance readings
              Log.info("(ReadDistance_US100) - Dist.: %dmm, now= %lu, lastPublish= %lu, RSSI= %d", (int)(dist_mm / numReadings), now, lastPublish, rssi);
              if (abs(dist_mm - prev_dist_mm) > minDistChange){         // Publish event in case of a change in temperature
                  lastPublish = now;                               // reset the max publish delay counter.
                  pushToPublishQueue(evDistance, (int)(dist_mm / numReadings), now);
                  prev_dist_mm = dist_mm;
                  samplingInterval = fastSampling;   // Measurements NOT stable, increase the sampling frequency
              }

          } else {
              /*Particle.publish("Hors portée: ","9999",60,PRIVATE);*/
              pushToPublishQueue(evOutOfRange, 9999, now);
              Log.info("(ReadDistance_US100) - Hors portée: ");             // output distance to serial monitor
              Log.info("(ReadDistance_US100) - currentReading %d mm", currentReading);
              delay(2000);
          }
      } else {
          Log.info("(ReadDistance_US100) - Données non disponible");
      }
      Serial1.end();  // Le capteur US-100 fonctionne à 9600 baud
  }

  // Cette routine lit la température sur le capteur US-100.
  // Note: La valeur 45 DOIT être soustraite pour obtenir la température réelle.
  void Readtemp_US100(){
      unsigned long now = millis();
      int Temp45 = 0;
      Serial1.begin(9600);  // Le capteur US-100 fonctionne à 9600 baud
      Serial1.write(0X50);            // Demander la lecture de température sur le US-100 en envoyant 50 (Hex)
      delay(30UL);                   // Attente du résult (20ms)
      if(Serial1.available() >= 1)    // lors de la réception d'au moins 1 bytes
      {
          Temp45 = Serial1.read();     // Lire la température brut
          if((Temp45 > 1) && (Temp45 < 130))   // Vérifier si la valeur est acceptable
          {
              TempUS100 = AvgTempReading(Temp45 - 45); // Conversion en température réelle et filtrage
              Log.info("(Readtemp_US100) - Temp. US100: %dC now= %lu, lastPublish= %lu",  (int)(TempUS100/ numReadings), now, lastPublish); // Pour debug
              if (abs(TempUS100 - prev_TempUS100) > minTempChange){    // Vérifier s'il y a eu changement depuis la dernière publication
                      lastPublish = now;                     // Remise à zéro du compteur de délais de publication
                      pushToPublishQueue(evUS100Temperature, (int)(TempUS100 / numReadings), now); // Publication
                      prev_TempUS100 = TempUS100;
                      varTempUS100 = (int)(TempUS100 / numReadings);  // 
                      samplingInterval = fastSampling;   // Augmenter la vitesse d'échantillonnage puisqu'il y a eu changement
                  }
          }
      }
      Serial1.end();  // Le capteur US-100 fonctionne à 9600 baud
  }
#endif

#if HASUS100THERMISTOR
// Cette routine lit la température sur le capteur thermistor du
// ##### boitier US-100 robuste #####.
// Note: La valeur 45 DOIT être soustraite pour obtenir la température réelle.
void ReadTherm_US100(){
    int Therm45 = 0;
    Serial1.begin(9600);  // Le capteur US-100 fonctionne à 9600 baud
    Serial1.write(0X5A);            // Demander la lecture du thermistor sur le boitier US-100 en envoyant 5A (Hex)
    delay(30UL);                   // Attente du résult
    if(Serial1.available() >= 1)    // lors de la réception d'au moins 1 bytes
    {
        Therm45 = Serial1.read();     // Lire la température brut
        if((Therm45 > 0) && (Therm45 < 255))   // Vérifier si la valeur est acceptable
        {
            TempThermUS100 = (Therm45 - 45); // Conversion en température réelle
            Log.info("(ReadTherm_US100) -Temp. thermistor: %dC now= %lu",  (byte)(Therm45 - 45), millis()); // Pour debug
        }
    }
    Serial1.end();  // Le capteur US-100 fonctionne à 9600 baud
}
#endif

#if DISTANCESENSOR == MB7389
// Cette routine mesure la distance entre la surface de l'eau et le capteur ultason
  void ReadDistance_MB7389(){
      unsigned long now = millis();
      MB7389_pulse = pulseIn(MB7389_pin2, HIGH);
      long currentReading = MB7389_pulse;
      Log.info("(ReadDistance_MB7389) - Distance meas routine: ReadDistance_MB7389");
      Log.info("(ReadDistance_MB7389) - MB7389latestReading: %lu", currentReading);
      rawDistmm = currentReading;
      if((currentReading > 1) && (currentReading < maxRangeMB7389)){       // normal distance should between 1mm and 2500 mm (1mm, 2,5m)
          dist_mm = AvgDistReading(currentReading); // Average the distance readings
          Log.info("(ReadDistance_MB7389) - Dist.: %dmm, now= %lu, lastPublish= %lu, RSSI= %d", (int)(dist_mm / numReadings), now, lastPublish, rssi);
          if (abs(dist_mm - prev_dist_mm) > minDistChange){         // Publish event in case of a change in temperature
              lastPublish = now;                               // reset the max publish delay counter.
              pushToPublishQueue(evDistance, (int)(dist_mm / numReadings), now);
              prev_dist_mm = dist_mm;
              samplingInterval = fastSampling;   // Measurements NOT stable, increase the sampling frequency
          }

      } else {
          dist_mm = 9999; // Code d'erreur
          if (abs(dist_mm - prev_dist_mm) > minDistChange){         // Publish event in case of a change in temperature
            lastPublish = now;                               // reset the max publish delay counter.
            pushToPublishQueue(evOutOfRange, dist_mm, now);
            prev_dist_mm = dist_mm;
            Log.info("(ReadDistance_MB7389) - Hors portée: ");             // output distance to serial monitor
            Log.info("(ReadDistance_MB7389) - currentReading: %lu mm", currentReading);
          }
      }
   }
#endif

// Filtre par moyenne mobile pour les distances
// Note: Il s'agit en fait de la somme des x dernière lecture.
//       La division se fera au moment de la publication
int AvgDistReading(int thisReading){
    long Avg = 0;
    for (int i = 1; i < numReadings; i++){
        allDistReadings[i-1] = allDistReadings[i]; // Shift all readings
       Avg += allDistReadings[i-1]; // Total of readings except the last one
    }
    allDistReadings[numReadings-1] = thisReading; // Current reading in the last position
    Avg += thisReading; // including the last one
    return (Avg); // Avg sera divisé par numReadings au moment de la publication
}

// Filtre par moyenne mobile pour les température
// Note: Il s'agit en fait de la somme des x dernière lecture.
//       La division se fera au moment de la publication
int AvgTempReading(int thisReading){
    long Avg = 0;
    for (int i = 1; i < numReadings; i++){
        allTempReadings[i-1] = allTempReadings[i]; // Shift all readings
        Avg += allTempReadings[i-1]; // Total of readings except the last one
    }
    allTempReadings[numReadings - 1] = thisReading; // Current reading in the last position
    Avg += thisReading; // total including the last one
    return (Avg); // Avg sera divisé par numReadings au moment de la publication
}

#if HASDS18B20SENSOR
  double readDS18b20temp(){
      float insideTempC;
      float outsideTempC;
      int i;
      int maxTry = 3;
      unsigned long now = millis();
      Particle.process();
      if (ds18b20Count > 0){
          if (ds18b20Count > 1){
              // Un capteur à l'intérieur du boitier et un à l'extérieur
              Log.info("(readDS18b20temp) - Lecture de 2 capteurs");

              ds18b20Sensors.requestTemperaturesByIndex(0); // requête de lecture
              for (i = 0; i < maxTry; i++){
                  insideTempC = ds18b20Sensors.getTempC(enclosureThermometer);
                  if (isValidDs18b20Reading(insideTempC)) break;
              }
              if (insideTempC > 30.0){
                analogWrite(heater, HeatingPower, 500); // for enclosure safety
              }
              validEnclosureTemp = isValidDs18b20Reading(insideTempC);
              if (validEnclosureTemp){
                  prev_EnclosureTemp = insideTempC;
                  Log.info("(readDS18b20temp) - DS18b20 interne: %f, try= %d", insideTempC, i + 1);
              } else {
                  Log.info("(readDS18b20temp) - DS18B20 interne: Erreur de lecture");
                  insideTempC = 99;
              }

              ds18b20Sensors.requestTemperaturesByIndex(1); // requête de lecture
              for (i = 0; i < maxTry; i++){
                  outsideTempC = ds18b20Sensors.getTempC(outsideThermometer); // 5 tentatives de lecture au maximum
                  if (isValidDs18b20Reading (outsideTempC)) break;
              }
              validTempExterne = isValidDs18b20Reading (outsideTempC);
              if (validTempExterne){
                  // Si la mesure est valide
                  if (abs(outsideTempC - prev_TempExterne) >= 1){
                      // Publier s'il y a eu du changement
                      pushToPublishQueue(evAmbientTemp, (int) outsideTempC, now);
                      prev_TempExterne = outsideTempC;
                  }
                  Log.info("(readDS18b20temp) - DS18b20 externe: %f, try= %d", outsideTempC, i + 1);
              } else {
                  // Si la measure est invalide
                  Log.info("(readDS18b20temp) - DS18B20 externe: Erreur de lecture");
                  outsideTempC = 99;
              }

              delay(100UL);
              return (outsideTempC);

           } else {
              // Un capteur à l'intérieur du boitier seulement
              Log.info("(readDS18b20temp) - lecture de 1 capteur");
              /*ds18b20Sensors.requestTemperaturesByAddress(enclosureThermometer); // requête de lecture*/
              ds18b20Sensors.requestTemperaturesByIndex(0); // requête de lecture
              for (i = 0; i < 5; i++){
                  insideTempC = ds18b20Sensors.getTempC(enclosureThermometer);
                  if (isValidDs18b20Reading(insideTempC)){
                      break;
                  }
              }
              validEnclosureTemp = isValidDs18b20Reading(insideTempC);
              if (validEnclosureTemp){
                  prev_EnclosureTemp = insideTempC;
                  Log.info("(readDS18b20temp) - DS18b20 interne: %f", insideTempC);
              } else {
                  Log.info("(readDS18b20temp) - DS18B20 interne: Erreur de lecture");
                  insideTempC = 99;
              }
              return (insideTempC);
           }

      }
      return(-99);
  }

  bool isValidDs18b20Reading(float reading){
      if (reading > -127 && reading < 85){
          return (true);
      } else {
          return (false);
      }
  }
#endif

#if HASHEATING
// Imprémentation d'un thermostat simple ON/OFF
// La routine ne fonctionne que si un capteur de température interne est trouvé
int simpleThermostat(double setPoint){
    // executer la fonction de thermostat si on a un capteur de température
    if (ds18b20Count > 0){
        // executer la fonction de thermostat si la température interne est valide
        if (prev_EnclosureTemp != 99){
            if (prev_EnclosureTemp < (setPoint - 0.5)){
                HeatingPower =  32767 *  MaxHeatingPowerPercent / 100UL;
            } else if (prev_EnclosureTemp > (setPoint + 0.5)){
                HeatingPower =  0;
            }
        } else if(prev_EnclosureTemp == 99){
        // Si non mettre le chauffage à 1/4 de puissance pour éviter le gel.
            HeatingPower =  0.25 * (32767 *  MaxHeatingPowerPercent / 100UL); // Chauffage fixe au 1/4 de la puissance
        }
        analogWrite(heater, HeatingPower, 500);
        Log.info("(simpleThermostat) - HeatingPower= %d, enclosureTemp= %0.1f, now= %lu", HeatingPower, prev_EnclosureTemp, millis());
        // if (HeatingPower != prev_HeatingPower){
        //    pushToPublishQueue(evHeatingPowerLevel, HeatingPower, now);
        //    prev_HeatingPower = HeatingPower;
        // }
    }
    return HeatingPower;
}
#endif

/*
Section réservé pour le code de mesure du vide (vacuum)
*/
#if HASVACUUMSENSOR
  void VacReadVacuumSensor(){
    int val = analogRead(VacuumSensor);
    double VacAnalogvalue = VacRaw2kPa(val, VacCalibration);
    Log.info("(VacReadVacuumSensor) - Vac raw= %d, VacAnalogvalue= %f, DeltaVac= %f", val, VacAnalogvalue, fabs(VacAnalogvalue - prev_VacAnalogvalue) );
    if (fabs(VacAnalogvalue - prev_VacAnalogvalue) > minVacuumChange){  // Publish event in case of a change in vacuum
        lastPublish = millis();                                             // reset the max publish delay counter.
        pushToPublishQueue(evVacuum, (int)(VacAnalogvalue * 100), lastPublish); // The measurements value is converted to integers for storage in the event buffer
        prev_VacAnalogvalue = VacAnalogvalue;                          // The value will be divided by 10 for display to recover the decimal
    }
  }

  // Routine de calibration du capteur de vide
  double VacCalibre() {
    double calibrer = 33340 - (double) analogRead(VacuumSensor);

    // 3850.24 correspond à la valeur idéal pour une pression de 0 Kpa
    // Il s'obtient ainsi : x = 0.94 * Vmax  (avec Vmax la valeur pour VCC soit 4096)

    // Nous faisons cinq lectures de suite afin de réchaufer le capteur
    for (int i=0; i <= 5; i++){
      delay(50);
      /*calibrer = 3850.24 - analogRead(VacuumSensor);*/
      calibrer = -490; // Mesure en absolue
    }
    return calibrer;
  }

  /* Fonction de conversion valeur numérique <> pression en Kpa */
  /*
   * D'aprés le datasheet du MPXV6115V6U
   * Vout = Vs * (Vac * 0.007652 + 0.92)
   * soit : Vac_kpa = (Vout / Vs * k) - 0,92 / 0.007652
   * avec Vout = Vref*Vraw*r1xr2/(4096*r2)
   * pour convertir de kpa to Hg (inch of mercure) il faut multiplier par 0.295301
   */
  double VacRaw2kPa(int raw, double calibration) {
    double Vout = Vref * (raw + calibration) * (R1 + R2) / (4096UL * R2);      // Vout = Vref*Vraw*(r1_+r2_)/(4096*r2_)
    double Vac_kpa = (Vout-(Vs-0.92))/(Vs*K_fact);  // Vac_kpa = (Vout-(Vs-0,92))/(Vs*k)
    double Vac_inHg = Vac_kpa * 0.2953001;
    /*Log.info("(VacRaw2kPa) - Vout= %f, Vac_kpa= %f, Vac_inHg= %f", Vout, Vac_kpa, Vac_inHg);*/
    return Vac_inHg;                              // multiplie par 0.295301 pour avoir la valeur en Hg
  }
#endif

#if HASVALVES
// Check the state of the valves position reedswitch
// when statusAll is true, publish even if there no change in the state
void CheckValvePos(bool statusAll){
    bool valveCurrentState = false;
    unsigned long now = 0;
    String stateStr = "";
    String positionCode[] = {"Erreur", "Ouvert", "Fermé", "Partiel"};
    for (int i=0; i <= 3; i++) {
        valveCurrentState = digitalRead(ValvePos_pin[i]);
        if ((ValvePos_state[i] != valveCurrentState) || statusAll == true){
            delay(valveDebounceDelay);  // Debounce time
            // time_t time = Time.now();
            valveCurrentState = digitalRead(ValvePos_pin[i]);
            ValvePos_state[i] = valveCurrentState;
            now = millis();
            if (valveCurrentState == true){
                stateStr = "/Open";
            } else {
                stateStr = "/Undefined";
            }
            // Now publish the position of the valves
            if (i <= 1){
              int Valve1Pos = getValvePosition(ValvePos_state[0], ValvePos_state[1]);
              pushToPublishQueue(evValve1_Position, Valve1Pos, now);
              Log.info("(CheckValvePos) - " + DomainName + DeptName + "Valve 1 position: " + Valve1Pos + ", " + positionCode[Valve1Pos]);
            } else {
              int Valve2Pos = getValvePosition(ValvePos_state[2], ValvePos_state[3]);
              pushToPublishQueue(evValve2_Position, Valve2Pos, now);
              Log.info("(CheckValvePos) - " + DomainName + DeptName + "Valve 2 position: " + Valve2Pos + ", " +  positionCode[Valve2Pos]);
            }
        }
    }
}

int getValvePosition(bool OpenSensor, bool CloseSensor){
    int ValvePos;
    if (OpenSensor == 0 && CloseSensor == 0){
      ValvePos = 0; // "Erreur". Impossible en fonctionnement normal
    }
    if (OpenSensor == 0 && CloseSensor == 1){
      ValvePos = 1; // "Ouvert".
    }
    if (OpenSensor == 1 && CloseSensor == 0){
      ValvePos = 2; // "Fermé"
    }
    if (OpenSensor == 1 && CloseSensor == 1){
      ValvePos = 3; // "Partiel"
    }
    return ValvePos;
}
#endif

// Active ou désactive le relais SSR
int toggleRelay(String command) {
    if (command=="on" || command=="1") {
        RelayState = HIGH;
        digitalWrite(ssrRelay, RelayState);
        /*Particle.publish("Relais", "on", 60, PRIVATE);*/
        pushToPublishQueue(evRelais, RelayState, millis());
        return 1;
    }
    else if (command=="off" || command=="0") {
        RelayState = LOW;
        digitalWrite(ssrRelay, RelayState);
        /*Particle.publish("Relais", "off", 60, PRIVATE);*/
        pushToPublishQueue(evRelais, RelayState, millis());
        return 0;
    }
    else {
        return -1;
    }
}

// Pour modifier l'interval de publication par défault
int remoteSet(String command){
  String token;
  String data;
  int sep = command.indexOf(",");
  if (sep > 0){
    token = command.substring(0, sep);
    data = command.substring(sep + 1);
  } else {
    return -1; // Fail
  }

  if (token == "MaxPubDelay"){
    unsigned long newInterval;
    newInterval = data.toInt();
    if (newInterval > 0){
        maxPublishInterval = data.toInt();
        Log.info("(remoteSet) - Now publishing at %d minutes interval", maxPublishInterval);
        return 0;
    } else {
        return -1;
    }
  } else if (token == "MaxHeatingPower"){
    return -1;
  }
  return -1;
}

// Pour resetter le capteur à distance au besoin
int remoteReset(String command) {
// Reset standard
  if (command == "device"){
    System.reset();
// ou juste les numéros de série.
  } else if (command == "serialNo") {
    Particle.syncTime();
    for (int i = 0; i < 30; i++){
        delay(100UL);
    }
    if (Time.isValid()){
        newGenTimestamp = Time.now();
        noSerie = 0;
        savedEventCount = 0;
        Log.info("(remoteReset) - Nouvelle génération de no de série maintenant: %lu", newGenTimestamp);
        pushToPublishQueue(evNewGenSN, -1, millis());
        return 0;
    } else {
        Log.info("(remoteReset) - Time is still invalid!: %lu", Time.now());
        return -1;
    }
// ou redémarre en safe mode (pour forcer une mise à jour)
  } else if (command == "safeMode") {
    System.enterSafeMode();
  } else {
    return -1;
  }
  return -1;
}


// Sauvegarde d'un événement en mémoire
bool writeEvent(struct Event thisEvent){
  if (readPtr == (writePtr + 1) % buffSize){
    return false; // Le buffer est plein, ne rien sauver.
  }
  eventBuffer[writePtr] = thisEvent;
  writePtr = (writePtr + 1) % buffSize; // avancer writePtr
  if ((writePtr - readPtr) < 0){
      buffLen = writePtr - readPtr + buffSize;
  } else {
      buffLen = writePtr - readPtr;
  }
  if (savedEventCount < buffSize) { // increment the number of saved events up until buffer length
    savedEventCount++;

  } // Number of stored events in the buffer.
   // pour debug
  // Log.info("(writeEvent) - W------> " + DomainName + DeptName + eventName[thisEvent.namePtr]);
  // Log.info("(writeEvent) - : writeEvent:: writePtr= %u, readPtr= %u, buffLen= %u, noSerie: %u, eData: %u, timer: %u",
  //                                    writePtr, readPtr, buffLen, thisEvent.noSerie, thisEvent.eData, thisEvent.timer);
  return true;
}

// Lecture d'un événement en mémoire
struct Event readEvent(){
  struct Event thisEvent = {};
  if (readPtr == writePtr){
    // Log.info("(readEvent) - <------- readEvent:: writePtr= %u, readPtr= %u, buffLen= %u, *** buffer vide ***",
    //                                   writePtr, readPtr, buffLen);
    return thisEvent; // événement vide
  }
  thisEvent = eventBuffer[readPtr];
  readPtr = (readPtr + 1) % buffSize;
  if ((writePtr - readPtr) < 0){
      buffLen = writePtr - readPtr + buffSize;
  } else {
      buffLen = writePtr - readPtr;
  }
  // pour debug
  // Log.info("(readEvent) - <R------ " + DomainName + DeptName + eventName[thisEvent.namePtr]);
  // Log.info("(readEvent) - : readEvent:: writePtr= %u, readPtr= %u, buffLen= %u, noSerie: %u, eData: %u, timer: %u",
  //                                   writePtr, readPtr, buffLen, thisEvent.noSerie, thisEvent.eData, thisEvent.timer);
  return thisEvent;
}

// Lecture d'un événement en mémoire sans avancé le pointeur
struct Event peekEvent(uint16_t peekReadPtr){
  struct Event thisEvent = {};
  if (peekReadPtr == writePtr){
    Log.info("(peekEvent) -  ------- peekEvent:: writePtr= %u, peekReadPtr= %u, buffLen= %u, *** buffer vide ***",
                                      writePtr, peekReadPtr, buffLen);
    return thisEvent; // événement vide
  }
  thisEvent = eventBuffer[peekReadPtr];
   // pour debug
  // Log.info((peekEvent) -  " ------- " + eventName[thisEvent.namePtr]);
  // Log.info("(peekEvent) -  : peekEvent:: writePtr= %u, readPtr= %u, buffLen= %u, noSerie: %u, eData: %u, timer: %u",
  //                                   writePtr, peekReadPtr, buffLen, thisEvent.noSerie, thisEvent.eData, thisEvent.timer);
  return thisEvent;
}

// Permet de demander un replay des événements manquants
// Initialise les paramètres pour la routine replayQueuedEvents()
// Format de la string de command: "Target SN, Target generation Id"
int replayEvent(String command){
  time_t targetGen;
  uint32_t targetSerNo;
  int sep = command.indexOf(",");
  if (sep > 0){
    targetSerNo = command.substring(0, sep).toInt();
    targetGen = command.substring(sep + 1).toInt();
  } else {
    return -1; // Fail
  }
  Log.info("(replayEvent) - ??? Demande de replay Event SN: %lu, génération: %lu,  writePtr= %u, readPtr= %u, replayBuffLen= %u",
                  targetSerNo, targetGen, writePtr, readPtr, replayBuffLen);
  if (replayBuffLen > 0){
      return -2; // Replay en cour - Attendre
  }
  // Validation de la demande
  if (targetSerNo >= 0 && targetGen > 0){ // Le numéro recherché doit être plus grand que 0
    // Validation
    if (targetGen != newGenTimestamp){
      Log.info("(replayEvent) - ??? Error -99: targetGen= %lu, targetSerNo= %lu", targetGen, targetSerNo);
      return -99; // "invalid generation id"
    }
    if (targetSerNo >= noSerie){ // et plus petit que le numéro de série courant
        return -1;
    }
    if ((noSerie - targetSerNo) > savedEventCount){ // Il y a 250 événement au maximum dans le buffer
        /*targetSerNo = noSerie - buffSize;*/
        return savedEventCount; // 
    }
    // Calcul de la position de l'événement dans le buffer
    int tmpPtr = readPtr - (noSerie - targetSerNo);  // Position dans le buffer du premier événement à faire un playback
    if (tmpPtr < 0){
        replayPtr =  tmpPtr + buffSize;
    } else {
        replayPtr =  tmpPtr;
    }
    // calcul de la longueur du replayBuffLen
    if ((readPtr - replayPtr) < 0){
        replayBuffLen = readPtr - replayPtr + buffSize;
    } else {
        replayBuffLen = readPtr - replayPtr;
    }
    Log.info("(replayEvent) - ??? Accepté pour replay Event no: %lu, ReplayPtr= %u, writePtr= %u, readPtr= %u, replayBuffLen= %u, No de série courant= %lu",
                    targetSerNo, replayPtr, writePtr, readPtr, replayBuffLen, noSerie);
    return 0; // success
  } else {
    return -1;
  }
}

// Re-lecture d'un événement en mémoire
struct Event replayReadEvent(){
  struct Event thisEvent = {};
  if (replayPtr == writePtr){
    Log.info("\n(replayReadEvent) - <--&&--- replayReadEvent:: writePtr= %u, replayPtr= %u, replayBuffLen= %u, *** replay buffer vide ***",
                                      writePtr, replayPtr, replayBuffLen);
    return thisEvent; // événement vide
  }
  thisEvent = eventBuffer[replayPtr];
  replayPtr = (replayPtr + 1) % buffSize; // increment replay pointer
  if ((writePtr - replayPtr) < 0){
      replayBuffLen = writePtr - replayPtr + buffSize;
  } else {
      replayBuffLen = writePtr - replayPtr;
  }

  // pour debug
  // Log.info("(replayReadEvent) - <------- " + DomainName + DeptName + eventName[thisEvent.namePtr]);
  // Log.info("(replayReadEvent) - : readEvent:: writePtr= %u, replayPtr= %u, replayBuffLen= %u, noSerie: %u, eData: %u, timer: %u",
  //                                   writePtr, replayPtr, replayBuffLen, thisEvent.noSerie, thisEvent.eData, thisEvent.timer);
  return thisEvent;
}

/*
typedef struct Event{
  uint32_t noSerie; // Le numéro de série est généré automatiquement
  uint32_t eSnGen; // Timestamp du début d'une série de noSerie.
  uint32_t timer; // Temps depuis la mise en marche du capteur. Overflow après 49 jours.
  uint8_t namePtr; // Pointeur dans l'array des nom d'événement. (Pour sauver de l'espace NVRAM)
  int32_t eData;   // Données pour cet événement. Entier 16 bits. Pour sauvegarder des données en point flottant
*/
// Formattage standard pour les données sous forme JSON
String makeJSON(uint32_t numSerie, uint32_t timeStamp, uint32_t timer, int eData, String eName, bool replayFlag){
    sprintf(publishString,"{\"noSerie\": %lu,\"generation\": %lu,\"timestamp\": %lu,\"timer\": %lu,\"eData\":%d,\"eName\": \"%s\",\"replay\":%d}",
                              numSerie, newGenTimestamp, timeStamp, timer, eData, eName.c_str(), replayFlag);
    // Log.info ("(makeJSON) - makeJSON: %s",publishString);
    return publishString;
}

// Publie les événement et gère les no. de série et le stockage des événements
bool pushToPublishQueue(uint8_t eventNamePtr, uint32_t eData, uint32_t timer){
  struct Event thisEvent;
  noSerie++;
  thisEvent = {noSerie, Time.now(), timer, eventNamePtr, eData};
  Log.info("(pushToPublishQueue) - >>>> pushToPublishQueue::: " + eventName[eventNamePtr]);
  writeEvent(thisEvent); // Pousse les données dans le buffer circulaire
  return true;
}

// Publie un événement stocké en mémoire
bool publishQueuedEvents(){
    bool publishSuccess = false;
    struct Event thisEvent = {};
    bool replayFlag = false; // not a replay
    // Log.info("(publishQueuedEvents) - < publishQueuedEvents:::");
    thisEvent = peekEvent(readPtr);
    if (sizeof(thisEvent) == 0){
        return publishSuccess; // Rien à publié
    }
    if(Particle.connected()){
        if (connWasLost){
          connWasLost =  false;
          delay(2000); // Gives some time to avoid loosing events
        }
        publishSuccess = Particle.publish(DomainName + DeptName + eventName[thisEvent.namePtr],
                                            makeJSON(thisEvent.noSerie, thisEvent.timeStamp, thisEvent.timer, thisEvent.eData, DomainName + DeptName + eventName[thisEvent.namePtr], replayFlag), 60, PRIVATE);
        if (publishSuccess){
        readEvent(); // Avance le pointeur de lecture
        }
    } else {
     connWasLost = true;
    }
  return publishSuccess;
}

// Publie un événement stocké en mémoire
bool replayQueuedEvents(){
    bool publishSuccess = false;
    struct Event thisEvent = {};
    bool replayFlag = true; // This is a replay
    Log.info("(replayQueuedEvents) - &&&& replayQueuedEvents:::");
    thisEvent = peekEvent(replayPtr);
    if (sizeof(thisEvent) == 0){
        return publishSuccess; // Rien à publié
    }
    if(Particle.connected()){
        if (connWasLost){
          connWasLost =  false;
          delay(2000); // Gives some time to avoid loosing events
        }
        publishSuccess = Particle.publish(DomainName + DeptName + eventName[thisEvent.namePtr],
                                            makeJSON(thisEvent.noSerie, thisEvent.timeStamp, thisEvent.timer, thisEvent.eData, DomainName + DeptName + eventName[thisEvent.namePtr], replayFlag), 60, PRIVATE);
                                            // makeJSON(uint16_t numSerie, uint32_t timeStamp, uint32_t timer, int eData, String eName){
        if (publishSuccess){
        replayReadEvent(); // Avance le pointeur de lecture
        }
    } else {
     connWasLost = true;
    }
  return publishSuccess;
}

// check buffer pointers and length consistency
void checkPtrState(){
    uint16_t tmp;
// check readPtr
    tmp = readPtr;
    if (readPtr > buffSize) {
        if (writePtr < buffSize){
            readPtr = writePtr;
        }
    }
    Log.info("(checkPtrState) - Checked readPtr --> was:%u, now:%u", tmp, readPtr);
// check writePtr
    tmp = writePtr;
    if (writePtr > buffSize) {
        if (readPtr < buffSize){
            writePtr = readPtr;
        }
    }
    Log.info("(checkPtrState) - Checked writePtr --> was:%u, now:%u", tmp, writePtr);
// check buffLen
    tmp = buffLen;
    if (buffLen > buffSize){
        // inconsistance: reset pointers to 0
        readPtr  = 0;
        writePtr = 0;
    }
// calcul le buffLen
    if ((writePtr - readPtr) < 0){
        buffLen = writePtr - readPtr + buffSize;
    } else {
        buffLen = writePtr - readPtr;
    }
    Log.info("(checkPtrState) - Checked buffLen --> was:%u, now:%u", tmp, buffLen);
}
