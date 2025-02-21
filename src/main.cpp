#include <Arduino.h>
#include <pins.h>

// Constants and globals
#define COSTAS_SEQ_LEN 7
#define AD9850_WORD_LEN 40
#define DDS_DATA_PIN D9
#define DDS_CLK_PIN D8
#define COSTAS_TXRQ_PIN A5
#define COSTAS_TRIG_PIN A3
#define COSTAS_CLK_PIN D4
#define FQ_UD_PIN D6
#define DDS_RST D11

// Globals for sequence control
volatile uint8_t currentIndex = 0;
volatile bool sequenceActive = false;
volatile bool wordLoaded = false;
volatile uint64_t currentWord = 0;

// Prototypes
void loadAD9850Word(uint64_t word);
uint64_t calculateAD9850Word(uint8_t index);
void IRAM_ATTR triggerISR();
void IRAM_ATTR clockISR();
void IRAM_ATTR fqudISR();

// Task handles
TaskHandle_t adLoaderTaskHandle = NULL;
TaskHandle_t sequenceTask = NULL;
// Constants for frequency calculation
#define BASEBAND_FREQ 6000000ULL  // 6 MHz
#define FREQ_OFFSET 300000ULL       // 30 kHz offset
#define FREQ_STEP 30000ULL          // 30 kHz step

// Calculate AD9850 frequency word based on sequence index
uint64_t calculateAD9850Word(uint8_t index) {
  // Costas sequence: [3,1,4,0,6,5,2]
  const uint8_t costasSequence[COSTAS_SEQ_LEN+1] = {255,3,1,4,0,6,5,2};
  // Calculate frequency for this index
  if(index == 0) {
    return 0ULL; // 0 Hz for index 0; first word is empty to wake up AD9850
  }
  uint64_t freq = BASEBAND_FREQ + FREQ_OFFSET + (costasSequence[index] * FREQ_STEP);
  // Convert to AD9850 word (freq * 2^32 / clock_freq)
  return (freq * 4294967296ULL) / 125000000ULL;
}
// AD9850 bit-by-bit loader
void loadAD9850Word(uint64_t word) {
  for(int i = 0; i < 40; i++) {
    digitalWrite(DDS_DATA_PIN, (word >> i) & 1);
    digitalWrite(DDS_CLK_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(DDS_CLK_PIN, LOW);
    delayMicroseconds(1);
  }
}

// Sequence handler task
void sequenceHandlerTask(void * parameter) {
  for(;;) {
    if(sequenceActive && !wordLoaded) {
      currentWord = calculateAD9850Word(currentIndex);
      wordLoaded = true;
      // Wait for next clock trigger
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    vTaskDelay(1); // Yield to other tasks
  }
}

// AD9850 loader task
void adLoaderTask(void * parameter) {
      bool waitingForFinalFqud = false;
      for(;;) {
        if(wordLoaded) {
          loadAD9850Word(currentWord);
          // Wait for FQ_UD rising edge
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          
          wordLoaded = false;
          if(++currentIndex >= COSTAS_SEQ_LEN+1) { // +1 for initialization
            if(!waitingForFinalFqud) {
              waitingForFinalFqud = true;
              currentIndex--;                     // Keep current index valid for one more cycle (to let the last frequency play)
            } else {
              currentIndex = 0;
              sequenceActive = false;
              digitalWrite(COSTAS_TXRQ_PIN, LOW);
              digitalWrite(DDS_RST, HIGH);        // Reset AD9850
              vTaskDelay(500);                    // Give it some time to get cozy and fall asleep
              digitalWrite(DDS_RST, LOW);         // Deassert reset
              waitingForFinalFqud = false;
            }
          }
        }
        vTaskDelay(1);
      }
    }

// ISR handlers
void IRAM_ATTR triggerISR() {
  sequenceActive = true;
  digitalWrite(COSTAS_TXRQ_PIN, HIGH);
  currentIndex = 0;
}

void IRAM_ATTR clockISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(sequenceTask, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR fqudISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(adLoaderTaskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void setup() {
  // Pin setup
  pinMode(COSTAS_TRIG_PIN, INPUT);
  pinMode(COSTAS_CLK_PIN, INPUT);
  pinMode(FQ_UD_PIN, INPUT);
  pinMode(COSTAS_TXRQ_PIN, OUTPUT);
  pinMode(DDS_DATA_PIN, OUTPUT);
  pinMode(DDS_CLK_PIN, OUTPUT);
  pinMode(DDS_RST, OUTPUT);

  // Create tasks
  xTaskCreatePinnedToCore(
    sequenceHandlerTask,
    "sequenceHandler",
    10000,
    NULL,
    2,
    &sequenceTask,
    0
  );

  xTaskCreatePinnedToCore(
    adLoaderTask,
    "adLoader",
    10000,
    NULL,
    1,
    &adLoaderTaskHandle,
    0
  );

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(COSTAS_TRIG_PIN), triggerISR, RISING);
  attachInterrupt(digitalPinToInterrupt(COSTAS_CLK_PIN), clockISR, RISING);
  attachInterrupt(digitalPinToInterrupt(FQ_UD_PIN), fqudISR, RISING);
}

void loop() {
  // Main loop can be empty as everything is handled by RTOS tasks
  vTaskDelay(1);
}
