#include <Stepper.h>
#include <LiquidCrystal.h>
#include <EEPROM.h> // Necesario para la memoria EEPROM

// --- Direcciones de la EEPROM ---
// Es crucial asignar direcciones únicas para cada variable para evitar sobrescritura.
// Asegúrate de que no se superpongan y que haya suficiente espacio para cada tipo de dato.
#define EEPROM_TOTAL_STEPS_ADDR 0     // long (4 bytes)
#define EEPROM_COMPLETED_REVS_ADDR 4  // int (2 bytes)
#define EEPROM_CURRENT_DIR_ADDR 6     // int (2 bytes)
#define EEPROM_MOTOR_RPM_ADDR 8       // int (2 bytes)
#define EEPROM_LUZ_ADDR 10            // bool (1 byte)
#define EEPROM_PAUSED_ELAPSED_ADDR 11 // unsigned long (4 bytes)
#define EEPROM_WAS_RUNNING_ADDR 15    // bool (1 byte)


// Configuración LCD Shield
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // Pines RS, EN, D4, D5, D6, D7

// Motor 28BYJ-48 (2048 pasos/rev en modo onda completa, 4096 en medio paso)
#define STEPS_PER_REV 2048 // Asegúrate de que esto coincide con tu modo de cableado/pasos
Stepper myStepper(STEPS_PER_REV, 13, 11, 12, A5); // Pines IN1, IN2, IN3, IN4

// --- Configuración de Botones ---
const byte KEYPAD_ANALOG_PIN = A0; // Pin analógico para los botones del keypad
const int RELE_PIN = A4;

// Enumeración para hacer el código más legible
enum Button {
  NONE = 0,
  RIGHT,
  UP,
  DOWN,
  LEFT,
  SELECT,
  ENC // Asumiendo que ENC es un valor analógico de A0. ¡Revisar esto!
};

// Rangos analógicos para cada botón del keypad (¡AJUSTA ESTOS VALORES PARA TU SHIELD!)
// Usa el sketch de calibración (Serial.println(analogRead(A0));)
// Los valores son rangos: (valor_min, valor_max)
const int RANGE_LEFT_MIN = 0;      const int RANGE_LEFT_MAX = 50;
const int RANGE_UP_MIN = 75;       const int RANGE_UP_MAX = 150;
const int RANGE_RIGHT_MIN = 175;   const int RANGE_RIGHT_MAX = 250;
const int RANGE_SELECT_MIN = 400;  const int RANGE_SELECT_MAX = 450;
const int RANGE_DOWN_MIN = 500;    const int RANGE_DOWN_MAX = 700;
const int RANGE_ENC_MIN = 750;     const int RANGE_ENC_MAX = 850; // Rango para el botón "ENC"
const int RANGE_NONE_MIN = 900;    // Valor cuando no se pulsa ningún botón (aprox. 900-1023)

// Variables de anti-rebote
unsigned long lastDebounceTime = 0;
const long DEBOUNCE_DELAY = 500; // 200 ms es un buen valor para anti-rebote

// Control del sistema
bool isRunning = false;
bool wasRunningBeforePowerOff = false; // Flag para EEPROM
int completedRevolutions = 0;     // Vueltas totales acumuladas
const int TARGET_REVS = 800;      // Total de vueltas (revoluciones) deseadas para el ciclo completo
int currentDirection = -1;        // -1=horario, 1=antihorario
bool luz = true;                  // Estado de la retroiluminación del LCD y el relé


// Temporizadores del ciclo de funcionamiento
unsigned long windStartTime;      // Marca de tiempo cuando la *fase* actual comenzó/reanudó
unsigned long pausedElapsedTime = 0; // Tiempo transcurrido acumulado antes de la pausa (cuando está en pausa manual)
long totalStepsTaken = 0;         // Contará los pasos totales del motor

// Configuración de RPM y modos
int currentMotorRPM = 14;        // Velocidad RPM actual del motor
const int MIN_RPM = 1;
const int MAX_RPM = 25; // Define un máximo sensato para el motor 28BYJ-48
const int RPM_STEP_INCREMENT = 1; // Cuánto se incrementa/decrementa el RPM por pulsación

const long WIND_DURATION = 300000;    // 5 minutos (300,000 ms)
const long PAUSE_DURATION = 1800000;  // 30 minutos (1800,000 ms)
unsigned long lastStepTime = 0;
// STEP_DELAY se calculará dinámicamente o la librería Stepper lo maneja

// Variable para controlar el modo de visualización del LCD
// 0: Inicio, 1: Girando, 2: Pausa (Automática), 3: Completado, 4: Pausado Manualmente (por SELECT)
// 5: Modo Configuración RPM (NUEVO)
int currentLcdMode = 0;
int lastLcdDisplayedMode = -1; // Para la actualización inteligente del LCD

// --- Funciones de EEPROM ---
void saveStateToEEPROM() {
  EEPROM.put(EEPROM_TOTAL_STEPS_ADDR, totalStepsTaken);
  EEPROM.put(EEPROM_COMPLETED_REVS_ADDR, completedRevolutions);
  EEPROM.put(EEPROM_CURRENT_DIR_ADDR, currentDirection);
  EEPROM.put(EEPROM_MOTOR_RPM_ADDR, currentMotorRPM);
  EEPROM.put(EEPROM_LUZ_ADDR, luz);
  EEPROM.put(EEPROM_PAUSED_ELAPSED_ADDR, pausedElapsedTime);
  EEPROM.put(EEPROM_WAS_RUNNING_ADDR, wasRunningBeforePowerOff);
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("CONFIG GUARDADA!");
  lcd.setCursor(0,1);
  lcd.print("Listo!"); 
  delay(1000); // Muestra el mensaje por 1 segundo
  lastLcdDisplayedMode = -1; // Para forzar el redibujo cuando se salga de este mensaje
}

void loadStateFromEEPROM() {
  long loadedTotalSteps;
  EEPROM.get(EEPROM_TOTAL_STEPS_ADDR, loadedTotalSteps);
  // Validación básica para evitar datos corruptos al inicio
  if (loadedTotalSteps < 0 || loadedTotalSteps > 2000000000L) { // Valor de seguridad
    totalStepsTaken = 0;
  } else {
    totalStepsTaken = loadedTotalSteps;
  }

  int loadedRevs;
  EEPROM.get(EEPROM_COMPLETED_REVS_ADDR, loadedRevs);
  if (loadedRevs < 0 || loadedRevs > TARGET_REVS) { // Valor de seguridad
    completedRevolutions = 0;
  } else {
    completedRevolutions = loadedRevs;
  }

  int loadedDir;
  EEPROM.get(EEPROM_CURRENT_DIR_ADDR, loadedDir);
  if (loadedDir != 1 && loadedDir != -1) { // Valor de seguridad
    currentDirection = -1;
  } else {
    currentDirection = loadedDir;
  }
  
  int loadedRPM;
  EEPROM.get(EEPROM_MOTOR_RPM_ADDR, loadedRPM);
  if (loadedRPM < MIN_RPM || loadedRPM > MAX_RPM) { // Valor de seguridad
    currentMotorRPM = 14; // Valor por defecto si es inválido
  } else {
    currentMotorRPM = loadedRPM;
  }

  //bool loadedLuz;
  //EEPROM.get(EEPROM_LUZ_ADDR, loadedLuz);
  //luz = loadedLuz;

  unsigned long loadedPausedElapsed;
  EEPROM.get(EEPROM_PAUSED_ELAPSED_ADDR, loadedPausedElapsed);
  pausedElapsedTime = loadedPausedElapsed;

  bool loadedWasRunning;
  EEPROM.get(EEPROM_WAS_RUNNING_ADDR, loadedWasRunning);
  wasRunningBeforePowerOff = loadedWasRunning;

  // Si estaba corriendo/pausado manualmente antes del apagado, establecer el estado
  // No iniciar el sistema automáticamente, sino mostrar que puede continuar
  if (wasRunningBeforePowerOff) {
    isRunning = false; // Siempre inicia pausado, el usuario debe pulsar SELECT para continuar
    currentLcdMode = 4; // Mostrará "Sistema PAUSADO"
  } else {
    // Si no estaba corriendo, reiniciar todo a 0 (aunque ya se cargaron los 0 si no había nada)
    totalStepsTaken = 0;
    completedRevolutions = 0;
    pausedElapsedTime = 0;
    currentLcdMode = 0;
  }
}

void setup() {
  pinMode(10, OUTPUT);
  pinMode(RELE_PIN, OUTPUT);
  
  lcd.begin(16, 2); // Inicializa el LCD

  loadStateFromEEPROM(); // Carga el estado al iniciar

  digitalWrite(10, luz ? HIGH : LOW); // Configura la retroiluminación según el estado cargado
  digitalWrite(RELE_PIN, luz ? LOW : HIGH); // Configura el relé según el estado cargado (asumiendo LOW = apagado para luz)

  myStepper.setSpeed(currentMotorRPM); // Establece la velocidad del motor
  
  updateDisplay();     // Muestra la pantalla inicial (ya sea Inicio o Pausado)
}

void loop() {
  handleButtons(); // Nueva función que maneja la lógica de todos los botones
  runWinder();     // Lógica de giro/pausa del motor
  updateDisplay(); // Actualización inteligente de la pantalla LCD
  
  // ¡Se elimina el guardado periódico aquí!
}

// Función para leer y procesar todos los botones
void handleButtons() {
  unsigned long currentTime = millis();

  int keypadAnalogValue = analogRead(KEYPAD_ANALOG_PIN);

  Button pressedButton = NONE;
  
  // Lógica para detectar qué botón se ha pulsado basándose en los rangos analógicos
  if (keypadAnalogValue >= RANGE_LEFT_MIN && keypadAnalogValue <= RANGE_LEFT_MAX) {
    pressedButton = LEFT;
  } else if (keypadAnalogValue >= RANGE_RIGHT_MIN && keypadAnalogValue <= RANGE_RIGHT_MAX) {
    pressedButton = RIGHT;
  } else if (keypadAnalogValue >= RANGE_UP_MIN && keypadAnalogValue <= RANGE_UP_MAX) {
    pressedButton = UP;
  } else if (keypadAnalogValue >= RANGE_SELECT_MIN && keypadAnalogValue <= RANGE_SELECT_MAX) {
    pressedButton = SELECT;
  } else if (keypadAnalogValue >= RANGE_DOWN_MIN && keypadAnalogValue <= RANGE_DOWN_MAX) {
    pressedButton = DOWN;
  } else if (keypadAnalogValue >= RANGE_ENC_MIN && keypadAnalogValue <= RANGE_ENC_MAX) {
    pressedButton = ENC;
  }
  
  if (pressedButton != NONE && (currentTime - lastDebounceTime > DEBOUNCE_DELAY)) {
    lastDebounceTime = currentTime; // Reinicia el temporador de anti-rebote

    switch (pressedButton) {
      case SELECT:        
        if (currentLcdMode == 5) { // Si estamos en modo configuración RPM
          // Salir del modo configuración. No se guarda automáticamente aquí.
          // Si estaba en modo pausa antes de entrar a config, volver a pausa. Sino, a inicio.
          currentLcdMode = wasRunningBeforePowerOff ? 4 : 0; 
        } else {
          isRunning = !isRunning; // Alterna el estado de running
          wasRunningBeforePowerOff = isRunning || (currentLcdMode == 4); // Si está corriendo o se ha pausado manualmente

          if (isRunning) {
            windStartTime = currentTime - pausedElapsedTime; // Retoma el ciclo donde se quedó
            pausedElapsedTime = 0; // Resetea tiempo de pausa
            currentLcdMode = 1;    // Modo girando
          } else {
            // Se pausó manualmente
            pausedElapsedTime = currentTime - windStartTime; // Almacena el tiempo transcurrido en la fase actual
            myStepper.step(0); 
            currentLcdMode = 4;    // Modo pausado manualmente
          }
          //saveStateToEEPROM(); // Guardar estado al cambiar entre running/pausado
        }
        break;

      case LEFT:        
        if (currentLcdMode == 5) { // Si estamos en modo configuración RPM
          currentMotorRPM = max(MIN_RPM, currentMotorRPM - RPM_STEP_INCREMENT);
          myStepper.setSpeed(currentMotorRPM);
        } else if (isRunning) { // Solo cambia dirección si el sistema está funcionando
          currentDirection = -1; // Horario
          windStartTime = currentTime; // Reinicia el temporizador para la nueva fase
          currentLcdMode = 1; // Forzar actualización de pantalla
        }      
        break;      
      case RIGHT:        
        if (currentLcdMode == 5) { // Si estamos en modo configuración RPM
          currentMotorRPM = min(MAX_RPM, currentMotorRPM + RPM_STEP_INCREMENT);
          myStepper.setSpeed(currentMotorRPM);
        } else if (isRunning) { // Solo cambia dirección si el sistema está funcionando
          currentDirection = +1; // Anti-horario
          windStartTime = currentTime; // Reinicia el temporizador para la nueva fase
          currentLcdMode = 1; // Forzar actualización de pantalla
        }      
        break;
      
      case UP:
        // Reinicia completamente el sistema
        isRunning = false;            
        windStartTime = currentTime;      // Reset del temporizador de ciclo
        completedRevolutions = 0;     // Reinicia el contador de "vueltas totales"
        currentDirection = -1;        // Asegura que siempre empiece en dirección Horaria
        pausedElapsedTime = 0;        // Resetear el tiempo de pausa
        totalStepsTaken = 0;          // Resetear pasos al reiniciar el sistema
        currentMotorRPM = 14;         // Volver a RPM por defecto
        myStepper.setSpeed(currentMotorRPM); // Aplicar velocidad por defecto
        wasRunningBeforePowerOff = false; // No estaba corriendo antes del reset
        currentLcdMode = 0;           // Forzar actualización de pantalla a modo "Inicio"
        //saveStateToEEPROM(); // Guardar el estado reiniciado
        break;

      case DOWN:
        if (currentLcdMode == 5) { // Si estamos en modo configuración RPM
            saveStateToEEPROM(); // GUARDAR EXPLICITAMENTE CON BOTON DOWN
            // Después de guardar, salir del modo de configuración.
            // Si el sistema estaba en pausa, volver a ese estado, sino, a inicio.
            currentLcdMode = wasRunningBeforePowerOff ? 4 : 0; 
        } else {
            // Comportamiento original: alternar luz y relé
            luz = !luz; 
            if (luz){
              digitalWrite(10, HIGH); // Enciende la retroiluminación
              digitalWrite(RELE_PIN, LOW); // Relé apagado (asumiendo LOW = apagado)
            } else {
              digitalWrite(10, LOW);  // Apaga la retroiluminación
              digitalWrite(RELE_PIN, HIGH); // Relé encendido
            }
            //saveStateToEEPROM(); // Guardar el estado de la luz/relé
        }
        break;

      case ENC: // Botón del potenciómetro
        if (!isRunning && currentLcdMode != 5) { // Si el sistema NO está corriendo (inicio, pausa manual, o completado) Y NO estamos ya en modo config
          currentLcdMode = 5; // Entrar en modo configuración RPM
          lastLcdDisplayedMode = -1; // Forzar redibujo
        } else if (isRunning) { // Si el sistema ESTÁ corriendo, vuelve a la funcionalidad original de Turbo
           // Lógica original de ENC cuando está corriendo (modo Turbo)
           // Aquí se asume que 'Turbo' es solo un concepto para el usuario,
           // y que se cambia entre 14 y 6 RPM.
           if (currentMotorRPM == 14) { // Si la velocidad actual es 14, cambiar a 6
             currentMotorRPM = 6;
           } else { // Si no es 14 (ej. 6 o la configurada), cambiar a 14
             currentMotorRPM = 14;
           }
           myStepper.setSpeed(currentMotorRPM); // ¡ACTUALIZA LA VELOCIDAD DEL MOTOR!
           // Puedes añadir un breve mensaje en pantalla para confirmar el cambio de RPM si lo deseas
           // pero esto puede interrumpir la fluidez del modo de giro
        } else if(currentLcdMode == 5) {
          currentLcdMode = 1;
          lastLcdDisplayedMode = -1;
          }
        break;        
    }    
  }    
}

void runWinder() {
  if (!isRunning) {
    myStepper.step(0); // Detener el motor si no está corriendo
    return;
  }
  // Si estamos en modo configuración RPM, tampoco mover el motor
  if (currentLcdMode == 5) {
      myStepper.step(0);
      return;
  }

  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - windStartTime;  

  if (elapsed < WIND_DURATION) {
    // Fase de giro
    myStepper.step(currentDirection); // Realiza un paso
    totalStepsTaken += 1; // Incrementa los pasos reales dados
    
    if (currentLcdMode != 1) {  
      currentLcdMode = 1;
    }
  } else if (elapsed < WIND_DURATION + PAUSE_DURATION) {
    // Fase de pausa (automática)
    myStepper.step(0); // Asegurarse de que el motor está parado durante la pausa
    if (currentLcdMode != 2) {
      currentLcdMode = 2;  
    }
  } else {
    // Fin de un ciclo completo (giro + pausa)
    currentDirection *= -1;       // Invierte la dirección
    windStartTime = currentTime;  // Reinicia el temporizador para el nuevo ciclo
    
    completedRevolutions = (totalStepsTaken / STEPS_PER_REV); // Vueltas completas acumuladas

    // Comprueba si se ha alcanzado el número total de vueltas deseadas
    if (completedRevolutions >= TARGET_REVS) { 
      isRunning = false;      
      wasRunningBeforePowerOff = false; // El ciclo ha terminado
      currentLcdMode = 3;      
      pausedElapsedTime = 0;  // Resetear el tiempo de pausa acumulado
      totalStepsTaken = 0;    // Resetear pasos al completar el ciclo total
      completedRevolutions = 0; // Resetear vueltas completas para un nuevo inicio
      myStepper.step(0);  // dejar el motor "parado"
      //saveStateToEEPROM(); // Guardar el estado de finalización del ciclo
      showCompletion();        
    } else {
      currentLcdMode = 1;      
    }
  }
}

void updateDisplay() {
  static unsigned long lastUpdate = 0;
  unsigned long currentTime = millis();

  // Decidir el modo LCD actual basado en el estado del sistema
  int actualLcdMode;
  if (currentLcdMode == 5) { // Modo configuración RPM tiene prioridad
    actualLcdMode = 5;
  } else if (!isRunning) {
    if (completedRevolutions >= TARGET_REVS && totalStepsTaken > 0) { // totalStepsTaken > 0 para evitar que aparezca completado si acaba de reiniciar a 0
      actualLcdMode = 3;  // Completado
    } else if (currentLcdMode == 4) {  // Pausado manualmente
      actualLcdMode = 4;
    } else {
      actualLcdMode = 0;  // Inicio
    }
  } else {  // isRunning es true
    unsigned long elapsed = currentTime - windStartTime;  
    if (elapsed < WIND_DURATION) {
      actualLcdMode = 1;  // Girando
    } else {  
      actualLcdMode = 2;  // Pausa Automática
    }
  }

  // Lógica de redibujado inteligente para evitar parpadeo y ahorrar CPU
  bool forceRedraw = false;
  if (actualLcdMode != lastLcdDisplayedMode) {  
    forceRedraw = true;
  } else if (actualLcdMode == 1 || actualLcdMode == 2) {  // Modos dinámicos
    if (currentTime - lastUpdate < 500) return; // Actualizar cada 500ms
  } else if (actualLcdMode == 5) { // Modo de configuración RPM
    if (currentTime - lastUpdate < 200) return; // Actualizar más rápido para ver los cambios de RPM
  } else {  // Modos estáticos (0, 3, 4)
    if (currentTime - lastUpdate < 2000) return; // Actualizar cada 2 segundos para asegurar visibilidad
  }

  if (!forceRedraw && (actualLcdMode != 1 && actualLcdMode != 2 && actualLcdMode != 5)) {
      return; // No redibujar si no es necesario y no es un modo dinámico
  }
  
  lastUpdate = currentTime;  
  lastLcdDisplayedMode = actualLcdMode;  

  lcd.clear();  

  switch (actualLcdMode) {
    case 0: // Pantalla de Inicio
      lcd.print("AutoWinder 1.5");
      lcd.setCursor(0, 1);
      lcd.print("SELECT: ON/PAUSE");  
      break;
      
    case 1: { // Girando
      lcd.print(currentDirection < 0 ? "HORARIO >>>" : "ANTI-HORARIO <<<");
      lcd.setCursor(0, 1);
      float currentRevolutions = (float)totalStepsTaken / (STEPS_PER_REV);
      lcd.print(currentRevolutions, 1); // Muestra con 1 decimal
      lcd.print("/");
      lcd.print(TARGET_REVS);            // TARGET_REVS es el total de vueltas
      lcd.print(" RPM:");          
      lcd.print(currentMotorRPM);            // RPM actual
      break;
    }
    case 2: { // Pausa Automática
      lcd.print(currentDirection < 0 ? "Pausa HORARIO" : "Pausa ANTI-HOR");  
      lcd.setCursor(0, 1);
      lcd.print("Espera: ");
      unsigned long elapsedInCurrentPhase = currentTime - windStartTime;
      lcd.print((WIND_DURATION + PAUSE_DURATION - elapsedInCurrentPhase) / 1000);  
      lcd.print("s");
      break;
    }
    case 3: // Completado
      lcd.print("COMPLETADO 100%");
      lcd.setCursor(0, 1);
      lcd.print("Ciclo Terminado"); // Más genérico, ya que TARGET_REVS puede cambiar
      break;
    case 4: // Pausado manualmente por SELECT
      lcd.print("Sistema PAUSADO");
      lcd.setCursor(0, 1);
      lcd.print("Continue: SELECT");
      break;
    case 5: // NUEVO: Modo Configuración RPM
      lcd.print("Config RPM:");
      lcd.setCursor(0, 1);
      lcd.print("RPM:");
      lcd.print(currentMotorRPM);
      lcd.print(" DOWN:Guardar"); // Indicamos que DOWN guarda
      break;
  }
}

void showStartScreen() {
  currentLcdMode = 0;
  lastLcdDisplayedMode = -1;  // Forzar redibujado
}

void showCompletion() {
  digitalWrite(10, HIGH); // Enciende la retroiluminación
  updateDisplay(); // Asegurarse de que el mensaje de COMPLETADO se muestre
  delay(5000); // Muestra el mensaje durante 5 segundos
  // Los resets de totalStepsTaken y completedRevolutions se hacen en runWinder
  // El reset de currentLcdMode se hace después del delay para que muestre el mensaje completo
  currentLcdMode = 0;   // Volver al modo de inicio
  lastLcdDisplayedMode = -1; // Forzar redibujado
  digitalWrite(10, LOW); // Apaga la retroiluminación
  myStepper.step(0); 
}
