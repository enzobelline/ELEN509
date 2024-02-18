#ifndef PROJECT_H
#define PROJECT_H

#define true 1
#define false 0

typedef enum {
  STOPPED,
  FORWARD,
  BACKWARD
} direction;

typedef enum {
  IDLE,
  DONE
} CaptureState;


#endif