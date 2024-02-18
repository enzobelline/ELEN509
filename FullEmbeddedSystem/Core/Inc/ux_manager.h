// ux_manager.h



#ifndef UX_MGR
#define UX_MGR

// includes
#include <stdint.h>
#include "ssd1306.h"
#include "Buttons.h"
#include "Encoder.h"

#include "main.h"
#include "project.h"
#include <stdio.h>
#include "Scheduler.h"

// typedefs
typedef enum  Screens_{
  MAIN,
  EDIT_PLOT,
  SET_POINT,
  DEBUG
//  NUMBER_OF_SCREENS
} ui_screen;


typedef struct DWuint8_t_
{
  char format[10];
  char invalidMsg[4];
  uint16_t xPos;
  uint16_t yPos;
  uint8_t valid;
  uint8_t data;
} DWuint8_t;


typedef struct DWint8_t_
{
  char format[10];
  char invalidMsg[4];
  uint16_t xPos;
  uint16_t yPos;
  uint8_t valid;
  int8_t data;
} DWint8_t;


typedef struct DWuint16_t_
{
  char format[10];
  char invalidMsg[4];
  uint16_t xPos;
  uint16_t yPos;
  uint8_t valid;
  uint16_t data;
} DWuint16_t;


typedef struct DWint16_t_
{
  char format[10];
  char invalidMsg[4];
  uint16_t xPos;
  uint16_t yPos;
  uint8_t valid;
  int16_t data;
} DWint16_t;


typedef struct DWstring_
{
  char format[10];
  char invalidMsg[4];
  uint16_t xPos;
  uint16_t yPos;
  uint8_t valid;
  char data[26];
} DWstring;


typedef struct DWfloat_
{
  char format[10];
  char invalidMsg[4];
  uint16_t xPos;
  uint16_t yPos;
  uint8_t valid;
  float data;
} DWfloat;

typedef struct bargraph8x32_t_
{
  uint8_t data[8];
  char xLabel[8];
  char yTicks[4];
  char title[16];
  uint16_t xPos;
  uint16_t yPos;
  uint8_t valid;
} bargraph8x32_t;

typedef struct linegraph_t_
{
  uint8_t data[24];
  char yTicks[4];
  char title[16];
  uint16_t xPos;
  uint16_t yPos;
  uint8_t cursor;
  uint8_t valid;
} linegraph_t;

// Global variables
// live screen data variables
extern ui_screen screenNumber;
//extern DWuint8_t counter;
//extern DWint8_t counter;
// display data
extern DWfloat counter;
extern DWfloat tempInF;
extern DWfloat humidity;
extern DWint16_t tempCJ_F;

extern bargraph8x32_t myGraph;
extern uint8_t graphUpdated;
// Global Constants


// exposed function prototypes
void SwitchScreens(ui_screen screen_no);
//void PrintTest(char * formater, uint16_t variable, uint8_t position);
//uint8_t ProcessKeyCode (uint16_t key_code);
void ProcessKeyCode (keyCode key_code);
uint8_t ProcessKeyCodeInContext (keyCode key_code, Encoder* enc);
void UpdateScreenValues(uint8_t moistureIn, uint16_t frequencyIn);
uint8_t GetKeycode(void);

void UpdateGraph(ui_screen _screen_no, linegraph_t* graph);

#endif