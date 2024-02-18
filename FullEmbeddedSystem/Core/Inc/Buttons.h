#include "Project.h"
#include "stm32f3xx_hal.h"
#include "main.h"
#include <stdint.h>
#include <stddef.h>

#ifndef BUTTONS_H
#define BUTTONS_H


typedef enum {
  BUT_NULL,
  BUT_L,
  BUT_R,
  BUT_ENC,
  BUT_MOV,
  BUT_WAIT
} keyCode;

keyCode getKeyCode(void);

#endif