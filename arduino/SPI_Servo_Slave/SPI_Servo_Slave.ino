#include <Arduino.h>
#include <Servo.h>
#include <util/atomic.h>
#include <avr/interrupt.h>

#if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega168__)
#error "Este sketch esta preparado para Arduino Uno/Nano AVR (ATmega328P/168)."
#endif

namespace {
constexpr uint8_t FRAME_HEADER = 0xAA;
constexpr uint8_t FRAME_FOOTER = 0x55;
constexpr uint8_t FRAME_SIZE = 6;
constexpr uint8_t FRAME_QUEUE_SIZE = 8;
constexpr uint8_t SERVO_COUNT = 5;
constexpr uint8_t SERVO_PINS[SERVO_COUNT] = {9, 6, 5, 3, 2};
constexpr uint16_t SERVO_UPDATE_INTERVAL_MS = 15;
constexpr float SERVO_SMOOTHING_FACTOR = 0.06f;

volatile uint8_t receiveBuffer[FRAME_SIZE];
volatile uint8_t receiveLength = 0;
volatile uint8_t frameQueue[FRAME_QUEUE_SIZE][FRAME_SIZE];
volatile uint8_t queueHead = 0;
volatile uint8_t queueTail = 0;
volatile bool transactionEnded = false;
volatile uint16_t receivedByteCount = 0;
volatile uint16_t invalidFrameCount = 0;
volatile uint16_t incompleteFrameCount = 0;
volatile uint16_t queueOverflowCount = 0;
volatile uint16_t acceptedFrameCount = 0;

Servo servos[SERVO_COUNT];
const int initialPositions[SERVO_COUNT] = {90, 90, 0, 90, 0};
int targetPositions[SERVO_COUNT] = {90, 90, 0, 90, 0};
float currentPositions[SERVO_COUNT] = {90.0f, 90.0f, 0.0f, 90.0f, 0.0f};
uint32_t lastServoUpdate = 0;

bool hasValidFields(const volatile uint8_t *frame) {
  const uint32_t payload =
      (static_cast<uint32_t>(frame[1]) << 16)
      | (static_cast<uint32_t>(frame[2]) << 8)
      | static_cast<uint32_t>(frame[3]);

  const uint8_t q1 = (payload >> 16) & 0x1F;
  const uint8_t q2 = (payload >> 11) & 0x1F;
  const uint8_t q3 = (payload >> 7) & 0x0F;
  const uint8_t q4 = (payload >> 2) & 0x1F;
  const uint8_t q5 = payload & 0x03;

  return (frame[1] & 0xE0) == 0
      && q1 <= 29
      && q2 <= 29
      && q3 <= 14
      && q4 <= 29
      && q5 <= 2;
}

bool hasValidChecksum(const volatile uint8_t *frame) {
  const uint8_t expected = static_cast<uint8_t>(
      FRAME_HEADER + frame[1] + frame[2] + frame[3]);
  return frame[4] == expected;
}

void enqueueReceivedFrame() {
  const uint8_t nextHead = (queueHead + 1) % FRAME_QUEUE_SIZE;
  if (nextHead == queueTail) {
    ++queueOverflowCount;
    return;
  }

  for (uint8_t index = 0; index < FRAME_SIZE; ++index) {
    frameQueue[queueHead][index] = receiveBuffer[index];
  }
  queueHead = nextHead;
  ++acceptedFrameCount;
}

void resetIncompleteTransaction() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (transactionEnded) {
      transactionEnded = false;
      if (receiveLength != 0) {
        receiveLength = 0;
        ++incompleteFrameCount;
      }
    }
  }
}

bool dequeueFrame(uint8_t *frame) {
  bool available = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (queueTail != queueHead) {
      for (uint8_t index = 0; index < FRAME_SIZE; ++index) {
        frame[index] = frameQueue[queueTail][index];
      }
      queueTail = (queueTail + 1) % FRAME_QUEUE_SIZE;
      available = true;
    }
  }
  return available;
}

void applyFrame(const uint8_t *frame) {
  const uint32_t payload =
      (static_cast<uint32_t>(frame[1]) << 16)
      | (static_cast<uint32_t>(frame[2]) << 8)
      | static_cast<uint32_t>(frame[3]);

  const uint8_t q1Level = (payload >> 16) & 0x1F;
  const uint8_t q2Level = (payload >> 11) & 0x1F;
  const uint8_t q3Level = (payload >> 7) & 0x0F;
  const uint8_t q4Level = (payload >> 2) & 0x1F;
  const uint8_t q5State = payload & 0x03;

  if (frame[0] != FRAME_HEADER
      || frame[FRAME_SIZE - 1] != FRAME_FOOTER
      || !hasValidChecksum(frame)
      || !hasValidFields(frame)) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ++invalidFrameCount;
    }
    return;
  }

  // Signed levels map to servo positions 0, 6, ..., 174 degrees.
  targetPositions[0] = q1Level * 6;
  targetPositions[1] = q2Level * 6;
  targetPositions[2] = q3Level * 6;
  targetPositions[3] = q4Level * 6;
  targetPositions[4] = q5State * 45;
}

void updateServos() {
  const uint32_t now = millis();
  if (now - lastServoUpdate < SERVO_UPDATE_INTERVAL_MS) {
    return;
  }
  lastServoUpdate = now;

  for (uint8_t index = 0; index < SERVO_COUNT; ++index) {
    currentPositions[index] +=
        (targetPositions[index] - currentPositions[index])
        * SERVO_SMOOTHING_FACTOR;
    servos[index].write(static_cast<int>(currentPositions[index] + 0.5f));
  }
}

void reportDiagnostics() {
  static uint32_t lastReport = 0;
  if (millis() - lastReport < 1000) {
    return;
  }
  lastReport = millis();

  uint16_t invalid;
  uint16_t incomplete;
  uint16_t overflow;
  uint16_t accepted;
  uint16_t receivedBytes;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    receivedBytes = receivedByteCount;
    invalid = invalidFrameCount;
    incomplete = incompleteFrameCount;
    overflow = queueOverflowCount;
    accepted = acceptedFrameCount;
  }

  Serial.print(F("SPI bytes="));
  Serial.print(receivedBytes);
  Serial.print(F(" validas="));
  Serial.print(accepted);
  Serial.print(F(" invalidas="));
  Serial.print(invalid);
  Serial.print(F(" incompletas="));
  Serial.print(incomplete);
  Serial.print(F(" cola_llena="));
  Serial.print(overflow);
  Serial.print(F(" objetivos="));
  for (uint8_t index = 0; index < SERVO_COUNT; ++index) {
    if (index != 0) {
      Serial.write(',');
    }
    Serial.print(targetPositions[index]);
  }
  Serial.println();
}
}  // namespace

ISR(SPI_STC_vect) {
  const uint8_t received = SPDR;
  ++receivedByteCount;
  if (receiveLength < FRAME_SIZE) {
    receiveBuffer[receiveLength++] = received;
  }

  if (receiveLength == FRAME_SIZE) {
    const bool valid = receiveBuffer[0] == FRAME_HEADER
        && receiveBuffer[FRAME_SIZE - 1] == FRAME_FOOTER
        && hasValidChecksum(receiveBuffer)
        && hasValidFields(receiveBuffer);
    if (valid) {
      enqueueReceivedFrame();
    } else {
      ++invalidFrameCount;
    }
    receiveLength = 0;
  }
}

ISR(PCINT0_vect) {
  if (PINB & _BV(PB2)) {
    transactionEnded = true;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Arduino SPI servo iniciado; esperando tramas de 6 bytes"));
  for (uint8_t index = 0; index < SERVO_COUNT; ++index) {
    servos[index].attach(SERVO_PINS[index]);
    servos[index].write(initialPositions[index]);
  }

  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT_PULLUP);
  SPCR = _BV(SPE) | _BV(SPIE);  // Slave, modo 0, MSB primero.

  PCIFR |= _BV(PCIF0);
  PCMSK0 |= _BV(PCINT2);  // Detecta el fin de la seleccion SS (D10).
  PCICR |= _BV(PCIE0);
}

void loop() {
  resetIncompleteTransaction();

  uint8_t frame[FRAME_SIZE];
  if (dequeueFrame(frame)) {
    applyFrame(frame);
  }

  updateServos();
  reportDiagnostics();
}