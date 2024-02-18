#include <stdint.h>
#include <stddef.h>

#ifndef ENCODER_H
#define ENCODER_H

#define ENC_DEFAULT_MAX 100
#define ENC_DEFAULT_MIN 0


typedef struct Encoder_Data {
  int32_t current;
  int32_t previous;
  uint8_t direction;
  int16_t min;
  int16_t max;
  int16_t value;
} Encoder;

void ENC_Init(Encoder * enc);
void ENC_Update(Encoder *enc);

#endif