#include <Arduino.h>
#include <pins.h>

// Constants and globals
#define COSTAS_SEQ_LEN 7
#define AD9850_WORD_LEN 40
#define DDS_DATA_PIN D9
#define DDS_CLK_PIN D8
#define COSTAS_TXRQ_PIN A5
#define PSK_TXRQ_PIN A4
#define PSK_CLK_PIN D2
#define PSK_TRIG_PIN A2
#define BEACON_ID_MSG "W2HAT COSTAS ARRAY BEACON" // Beacon ID message
#define COSTAS_TRIG_PIN A3
#define COSTAS_CLK_PIN D4
#define FQ_UD_PIN D6
#define DDS_RST D11
// Constants for frequency calculation
#define BASEBAND_FREQ 6000000ULL  // 6 MHz
#define FREQ_OFFSET 1000ULL       // 30 kHz offset
#define FREQ_STEP 100ULL          // 30 kHz step

// Globals for sequence control
volatile uint8_t currentIndex = 0;
volatile bool PSKPreambleActive = false;
volatile uint8_t preambleCounter = 0;
volatile bool CostasSequenceActive = false;
volatile bool PSKSequenceActive = false;
char PSKData[sizeof(BEACON_ID_MSG) + 1];
volatile uint64_t bitWord[2];
volatile bool wordLoaded = false;
volatile bool waitingForFqud = false;
volatile uint64_t currentWord = 0;

// Prototypes
void loadAD9850Word(uint64_t word);
uint64_t calculateAD9850WordCostas(uint8_t index);
void IRAM_ATTR triggerISR();
void IRAM_ATTR clockISRCostas();
void IRAM_ATTR fqudISR();



// Task handles
static const UBaseType_t adLoaderPriority = 3;  // High priority for timing-critical AD9850 loading
static const UBaseType_t sequencePriority = 2;  // Medium priority for sequence handling
TaskHandle_t adLoaderTaskCostasHandle = NULL;
TaskHandle_t sequenceTask = NULL;
TaskHandle_t adLoaderTaskPSKHandle = NULL;


// Calculate AD9850 frequency word based on sequence index for Costas array
uint64_t calculateAD9850WordCostas(uint8_t index) {
  // Costas sequence: [3,1,4,0,6,5,2]
  // The first element (255) is a placeholder to wake up AD9850
  const uint8_t costasSequence[COSTAS_SEQ_LEN+1] = {255,3,1,4,0,6,5,2};
  // Calculate frequency for this index
  if(index == 0) {
    return 0ULL; // 0 Hz for index 0; first word is empty to wake up AD9850
  }
  uint64_t freq = BASEBAND_FREQ + FREQ_OFFSET + (costasSequence[index] * FREQ_STEP);
  // Convert to AD9850 word (freq * 2^32 / clock_freq)
  return (freq * 4294967296ULL) / 125000000ULL;
}


// Calculate AD9850 frequency word based on frequency and phase
uint64_t calculateFreqPhase(uint32_t freq, uint16_t phaseDegree) {
    return ((freq * 4294967296ULL) / 125000000ULL) + (((uint64_t)(phaseDegree * 32) / 360) << 32);
}



// AD9850 bit-by-bit loader
void loadAD9850Word(uint64_t word) {
  for(int i = 0; i < 40; i++) {
    digitalWrite(DDS_DATA_PIN, (word >> i) & 1);
    digitalWrite(DDS_CLK_PIN, HIGH);
    digitalWrite(DDS_CLK_PIN, LOW);
  }
}

// Sequence handler task
void sequenceHandlerTask(void * parameter) {
  for(;;) {
    if(waitingForFqud) {
      // Wait for final FQ_UD rising edge
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      waitingForFqud = false;
    } else {
    if(CostasSequenceActive && !wordLoaded) {
      currentWord = calculateAD9850WordCostas(currentIndex);
      wordLoaded = true;
      // Wait for next clock trigger
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    else if(PSKSequenceActive && !wordLoaded) {
      // Load the word
      currentWord = bitWord[((PSKData[currentIndex/8] >> (currentIndex % 8)) & 1)];
      wordLoaded = true;
      // Wait for next clock trigger
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    } else if(PSKPreambleActive && !wordLoaded) {
      preambleCounter++;
      currentWord = bitWord[0];
      loadAD9850Word(currentWord);
      digitalWrite(DDS_CLK_PIN, HIGH);
      digitalWrite(DDS_CLK_PIN, LOW);
      if(preambleCounter > 7) {
        PSKPreambleActive = false;
        preambleCounter = 0;
        PSKSequenceActive = true;
      }
      // Wait for next fq_ud rising edge
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    } 
    }
    vTaskDelay(1);
  }
}

// AD9850 loader task for Costas sequence
void adLoaderTaskCostas(void * parameter) {
      bool waitingForFinalFqud = false;
      for(;;) {
        if(wordLoaded) {
          loadAD9850Word(currentWord);
          // Wait for FQ_UD rising edge
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          wordLoaded = false;
          if(++currentIndex >= COSTAS_SEQ_LEN+1) { // +1 for initialization
            if(!waitingForFinalFqud) {
              waitingForFqud = true;
              waitingForFinalFqud = true;
              currentIndex--;                     // Keep current index valid for one more cycle (to let the last frequency play)
            } else {
              currentIndex = 0;
              CostasSequenceActive = false;
              digitalWrite(DDS_RST, HIGH);        // Reset AD9850
              vTaskDelay(2000);                    // Give it some time to get cozy and fall asleep
              digitalWrite(DDS_RST, LOW);         // Deassert reset
              vTaskDelay(1);                    // Let a few more cycles pass
              digitalWrite(COSTAS_TXRQ_PIN, LOW);
              wordLoaded = false;
              waitingForFinalFqud = false;
            }
          }
        }
        vTaskDelay(1);
      }
    }

void adLoaderTaskPSK(void * parameter) {
  bool waitingForFinalFqud = false;
  for(;;) {
    if(wordLoaded) {
      loadAD9850Word(currentWord);
      // Wait for FQ_UD rising edge
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      wordLoaded = false;
      if(PSKSequenceActive)
      if(++currentIndex >= 8 * (uint32_t)strlen(BEACON_ID_MSG) + 1) { // +1 for initialization
        if(!waitingForFinalFqud) {
          waitingForFqud = true;
          waitingForFinalFqud = true;
          currentIndex--;                     // Keep current index valid for one more cycle (to let the last frequency play)
        } else {
          PSKSequenceActive = false;
          currentIndex = 0;
          digitalWrite(DDS_RST, HIGH);        // Reset AD9850
          vTaskDelay(2000);                    // Give it some time to get cozy and fall asleep
          digitalWrite(DDS_RST, LOW);         // Deassert reset
          vTaskDelay(1);                    // Let a few more cycles pass
          digitalWrite(PSK_TXRQ_PIN, LOW);
          waitingForFinalFqud = false;
          wordLoaded = false;
        }
      }
    }
    vTaskDelay(1);
  }
}

// ISR handlers
void IRAM_ATTR triggerISR() {
  CostasSequenceActive = true;
  digitalWrite(COSTAS_TXRQ_PIN, HIGH);
  currentIndex = 0;
}

void IRAM_ATTR triggerPSKISR() {
  PSKPreambleActive = true;
  digitalWrite(PSK_TXRQ_PIN, HIGH);
  currentIndex = 0;
}

void IRAM_ATTR clockISRCostas() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(sequenceTask, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR clockISRPSK() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(sequenceTask, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR fqudISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if(CostasSequenceActive){
    vTaskNotifyGiveFromISR(adLoaderTaskCostasHandle, &xHigherPriorityTaskWoken);
  }
  if(PSKSequenceActive || PSKPreambleActive){
    vTaskNotifyGiveFromISR(adLoaderTaskPSKHandle, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void setup() {
  // Calculate frequency and phase
  bitWord[0] = calculateFreqPhase(BASEBAND_FREQ, 0);
  bitWord[1] = calculateFreqPhase(BASEBAND_FREQ, 180);
  // Copy BEACON_ID_MSG to PSKData
  strcpy(&PSKData[1], BEACON_ID_MSG);
  PSKData[0] = 0;
  // Pin setup
  pinMode(COSTAS_TRIG_PIN, INPUT);
  pinMode(COSTAS_CLK_PIN, INPUT);
  pinMode(FQ_UD_PIN, INPUT);
  pinMode(COSTAS_TXRQ_PIN, OUTPUT);
  pinMode(DDS_DATA_PIN, OUTPUT);
  pinMode(DDS_CLK_PIN, OUTPUT);
  pinMode(DDS_RST, OUTPUT);
  pinMode(PSK_TXRQ_PIN, OUTPUT);
  pinMode(PSK_CLK_PIN, INPUT);
  pinMode(PSK_TRIG_PIN, INPUT);

  // Create tasks
  xTaskCreatePinnedToCore(
    sequenceHandlerTask,
    "sequenceHandler",
    10000,
    NULL,
    1,
    &sequenceTask,
    0
  );

  xTaskCreatePinnedToCore(
    adLoaderTaskCostas,
    "adLoaderCostas",
    10000,
    NULL,
    2,
    &adLoaderTaskCostasHandle,
    0
  );

  xTaskCreatePinnedToCore(
    adLoaderTaskPSK,
    "adLoaderPSK",
    10000,
    NULL,
    2,
    &adLoaderTaskPSKHandle,
    0
  );

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(COSTAS_TRIG_PIN), triggerISR, RISING);
  attachInterrupt(digitalPinToInterrupt(PSK_TRIG_PIN), triggerPSKISR, RISING);
  attachInterrupt(digitalPinToInterrupt(COSTAS_CLK_PIN), clockISRCostas, RISING);
  attachInterrupt(digitalPinToInterrupt(FQ_UD_PIN), fqudISR, RISING);
  attachInterrupt(digitalPinToInterrupt(PSK_CLK_PIN), clockISRPSK, RISING);
}

void loop() {
  // Main loop can be empty as everything is handled by RTOS tasks
  vTaskDelay(1);
}
