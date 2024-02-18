#include "Encoder.h"
#include "Project.h"
#include "stm32f303x8.h"

void ENC_Init(Encoder * enc){
  enc->current = 0;
  enc->previous = 0;
  enc->direction = STOPPED;
  if(enc->min == NULL) enc->min = ENC_DEFAULT_MIN;
  if(enc->max == NULL) enc->max = ENC_DEFAULT_MAX;
}

void ENC_Update(Encoder *enc){
  enc->current = TIM3->CNT;
  
  int32_t tmp;
  
  if(enc->current > enc->previous){
    tmp = enc->previous - enc->current;
    if(tmp > 0) 
      enc->direction = FORWARD;
    else 
      enc->direction = BACKWARD;
  } else if (enc->current < enc->previous){
    tmp = enc->current - enc->previous; 
    if(tmp > 0) 
      enc->direction = BACKWARD;
    else 
      enc->direction = FORWARD;
    
  } else {
    enc->direction = STOPPED;
  }
  
  if(enc->direction == FORWARD) {
    if(++enc->value > enc->max)
      enc->value = enc->max;
  } else if(enc->direction == BACKWARD) {
    if(--enc->value < enc->min)
      enc->value = enc->min;
  }
  
  enc->previous = enc->current;

}