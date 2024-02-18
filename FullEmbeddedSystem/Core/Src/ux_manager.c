// ux_manager.c

#include "ux_manager.h"
#include "EEPROM_manager.h"
// private defines


// Global Constants

//  switch (currentScreen) {
//  case MAIN:
//    break;
//  case EDIT_PLOT:
//    break;
//  case SET_POINT:
//    break;
//  case BUT_MOV:
//    break;
//
//  }

// Modular Constants


// global variables
// Screens
ui_screen currentScreen;
ui_screen lastScreen;

// Display-wrapped values
// format seq (numeric): {<format string>, <error message>, <Xpos>, <Ypos>, <valid?>, <init value>}
DWfloat counter = {"%5.2f", "----", 0, 0, true, 0};
DWuint8_t moisture = {"%02u", "----", 0, 0, true, 0};
DWuint8_t setMoisture = {"%02u", "----", 0,0, true, 0};
DWuint16_t frequency = {"%02u", "----", 0, 0, true, 0};
//DWfloat tempInF = {"%4.1f", "----", 0, 0, true, 72.2};
//DWfloat humidity = {"%4.1f", "----", 0, 0, true, 40.1};
//DWint16_t tempCJ_F = {"%5d", "!!!!", 0, 0, true, 0};
// format seq (string): {<format string>,  <error message>, <Xpos>, <Ypos>, <valid?>, "<init value>"

// Graph variables
//bargraph8x32_t myGraph = {{4,8,6,9,20,12,15,10}, {'a','b','c','d','e','f','g','h'}, {'4','3','2','1'}, "Set Moisture", 15, 16, true};
extern linegraph_t moisturePlan;

uint8_t graphUpdated = true;
// modular variables


// module prototypes
void ShowGraph(ui_screen _screen_no, linegraph_t* graph);


// ***************
// Start Of Code
// ***************
// Screen switching utility that manages pre-, post-, and screen switch conditions
void SwitchScreens(ui_screen screen_no)
{
  lastScreen = currentScreen;
  
  
#pragma diag_suppress= Pa149
  // what must be done before current screen is switched out
  switch (lastScreen) {
  }
  
  
  // what must be done before screen is switched in
  switch (screen_no) {
  }
#pragma diag_warning= Pa149
  
  // Switch the screens
  switch (screen_no) {
  case MAIN:
    // clear the screen from the previos dispayed data
    SSD1306_Clear();
    // Put up the "persistant" info (like data labels)
    SSD1306_GotoXY (0,0);
    SSD1306_Puts ("Soil Moisture", &Font_7x10, SSD1306_COLOR_WHITE);
    // Set u X/Y coordinates for "live" data to be displayed on this screen
    moisture.xPos = 30;
    moisture.yPos = 30;
    // Send a screen update (note this does not update the live data)
    SSD1306_UpdateScreen(); //display
    break;
  case EDIT_PLOT:
    SSD1306_Clear();
    ShowGraph(screen_no, &moisturePlan);
    break;
  case SET_POINT:
    SSD1306_Clear();
    SSD1306_GotoXY (0,0);
    SSD1306_Puts ("Set Moisture", &Font_7x10, SSD1306_COLOR_WHITE);
    
    setMoisture.xPos = 5;
    setMoisture.yPos = 32;

    SSD1306_UpdateScreen();
    break;
  case DEBUG:
    // clear the screen from the previos dispayed data
    SSD1306_Clear();
    // Put up the "persistant" info (like data labels)
    SSD1306_GotoXY (0,0);
    SSD1306_Puts ("DEBUG", &Font_7x10, SSD1306_COLOR_WHITE);
    // Set u X/Y coordinates for "live" data to be displayed on this screen
    frequency.xPos = 30;
    frequency.yPos = 30;
    // Send a screen update (note this does not update the live data)
    SSD1306_UpdateScreen(); //display
    break;
  }
  
  currentScreen = screen_no;
  
#pragma diag_suppress= Pa149
  // what must be done after screen is switched in
  switch (currentScreen) {
  }
#pragma diag_warning= Pa149
  
}



void ProcessKeyCode(keyCode key_code)
{
  // process the key press combination
  switch (key_code) {
  case 0x00: // all keys pressed
    break;
  case 0x01: // S2&1 pressed
    break;
  case 0x02: // S2&4 pressed
    break;
  case 0x03: // S2 pressed
    break;
  case 0x04: // S1&4 pressed
    break;
  case 0x05: // S1 pressed
    break;
  case 0x06: // S4 pressed
    break;
  }
}


// context sensitive keyboard processor
uint8_t ProcessKeyCodeInContext (keyCode key_code, Encoder* enc)
{
  switch (currentScreen) {
  case  MAIN:
    switch (key_code) {
    case BUT_NULL:
      break;
    case BUT_L:
      break;
    case BUT_R:
      break;
    case BUT_ENC:
      EEPROM_Read(moisturePlan.data);
      SwitchScreens(EDIT_PLOT);
      break;
    case BUT_MOV:
      break;
      case BUT_WAIT:
      break;
    }
    break;
  case  EDIT_PLOT:
    switch (key_code) {
      case BUT_WAIT:
      break;
    case BUT_NULL:
      break;
    case BUT_L:
      SSD1306_Clear();
      // Highlight Correct Point
      if(--moisturePlan.cursor > 23)
        moisturePlan.cursor = 23;
      enc->value = moisturePlan.data[moisturePlan.cursor];
      ShowGraph(currentScreen, &moisturePlan);
      break;
    case BUT_R:
      SSD1306_Clear();
      // Highlight Correct Point
      if (++moisturePlan.cursor > 23)
        moisturePlan.cursor = 0;
      enc->value = moisturePlan.data[moisturePlan.cursor];
      ShowGraph(currentScreen, &moisturePlan);
      break;
    case BUT_MOV:
//      romReadData(moisturePlan.data);
      EEPROM_Read(moisturePlan.data);
      SwitchScreens(SET_POINT);
      break;
    case BUT_ENC:
      SwitchScreens(DEBUG);
      break;
    }
    break;
  case  SET_POINT:
    switch (key_code) {
    case BUT_WAIT:
      moisturePlan.data[moisturePlan.cursor] = setMoisture.data;
//      romWriteData(moisturePlan.data);
      EEPROM_Write(moisturePlan.data);
      SwitchScreens(EDIT_PLOT);
      break;
    case BUT_NULL:
      break;
    case BUT_L:
      break;
    case BUT_R:
      break;
    case BUT_ENC:
      moisturePlan.data[moisturePlan.cursor] = setMoisture.data;
//      romWriteData(moisturePlan.data);
      EEPROM_Write(moisturePlan.data);
      SwitchScreens(EDIT_PLOT);
      break;
    case BUT_MOV:
      setMoisture.data = enc->value;
      SSD1306_DrawFilledRectangle(setMoisture.xPos+31,setMoisture.yPos+1, 94, 16, SSD1306_COLOR_BLACK);
      UpdateScreenValues((uint32_t)moisture.data, (uint32_t)frequency.data);
      break;
    }
    break;
  case DEBUG:
    switch (key_code) {
    case BUT_WAIT:
      break;
    case BUT_NULL:
      break;
    case BUT_L:
      break;
    case BUT_R:
      break;
    case BUT_ENC:
      SwitchScreens(MAIN);
      break;
    case BUT_MOV:
      break;
    }
    break;
  }
  
  SSD1306_UpdateScreen();
  
  return true;
}




void UpdateScreenValues(uint8_t moistureIn, uint16_t frequencyIn)
{
  char displayString[25];
  moisture.data = moistureIn;
  frequency.data = frequencyIn;
  switch (currentScreen) {
  case MAIN:
    SSD1306_GotoXY (moisture.xPos, moisture.yPos);
    if (moisture.valid) {
      sprintf(displayString, moisture.format, moisture.data);
      SSD1306_Puts(displayString, &Font_16x26, SSD1306_COLOR_WHITE);
    }
    else 
      SSD1306_Puts(moisture.invalidMsg, &Font_16x26, SSD1306_COLOR_WHITE);
    break;
  case EDIT_PLOT:
    break;
  case SET_POINT:
    SSD1306_GotoXY (setMoisture.xPos, setMoisture.yPos);
    if (setMoisture.valid) {
      sprintf(displayString, setMoisture.format, setMoisture.data);
      SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
    }
    else 
      SSD1306_Puts(setMoisture.invalidMsg, &Font_11x18, SSD1306_COLOR_WHITE);
    
    SSD1306_DrawRectangle(setMoisture.xPos+30,setMoisture.yPos, 96, 18, SSD1306_COLOR_WHITE);
    SSD1306_DrawFilledRectangle(setMoisture.xPos+30,setMoisture.yPos, setMoisture.data*3, 18, SSD1306_COLOR_WHITE);
    
    
    break;
  case DEBUG:
    SSD1306_GotoXY (frequency.xPos, frequency.yPos);
    if (frequency.valid) {
      sprintf(displayString, frequency.format, frequency.data);
      SSD1306_Puts(displayString, &Font_16x26, SSD1306_COLOR_WHITE);
    }
    else 
      SSD1306_Puts(frequency.invalidMsg, &Font_16x26, SSD1306_COLOR_WHITE);
    break;
    
  }
  SSD1306_UpdateScreen(); //display
}

void UpdateGraph(ui_screen _screen_no, linegraph_t* graph){
  uint16_t baseX;
  uint16_t baseY;
  uint8_t i;
  
  
  baseX = graph->xPos;
  baseY = graph->yPos;
  switch (currentScreen) {
  case MAIN:
    break;
  case EDIT_PLOT:
    for(i = 0; i < 24; ++i){
      SSD1306_DrawFilledCircle(baseX+i*4, baseY+32-graph->data[i], 2, SSD1306_COLOR_BLACK);
    }
    SSD1306_UpdateScreen();
    for(i = 0; i < 24; ++i){
      SSD1306_DrawCircle(baseX+i*4, baseY+32-graph->data[i], 1, SSD1306_COLOR_WHITE);
    }
    
    SSD1306_DrawFilledCircle(baseX+graph->cursor*4, baseY+32-graph->data[graph->cursor],2, SSD1306_COLOR_YELLOW);
    break;
  case SET_POINT:
    break;
    
  }
  SSD1306_UpdateScreen();
}


void ShowGraph(ui_screen _screen_no, linegraph_t* graph)
{
  uint16_t baseX;
  uint16_t baseY;
  uint8_t i;
  char showString[2];
  
  baseX = graph->xPos;
  baseY = graph->yPos;
  
  switch (_screen_no) {
  case MAIN:
    break;
  case EDIT_PLOT:
    SSD1306_GotoXY (0,0); //Title on top
    SSD1306_Puts (&graph->title[0], &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_DrawLine(baseX, baseY, baseX, baseY+32, SSD1306_COLOR_WHITE); //vert
    SSD1306_DrawLine(baseX, baseY+32, baseX+96, baseY+32, SSD1306_COLOR_WHITE); // horiz
    
    
    for(i = 0; i < 4; ++i){
      SSD1306_DrawLine(baseX-2, baseY+(i*8), baseX+2, baseY+(i*8), SSD1306_COLOR_WHITE);
      SSD1306_GotoXY((baseX-9), baseY-3+(i*8));
      SSD1306_Putc(graph->yTicks[i], &Font_7x10, SSD1306_COLOR_WHITE);
    }
    
    for(i = 0; i < 24; ++i){
      SSD1306_DrawCircle(baseX+i*4, baseY+32-graph->data[i], 1, SSD1306_COLOR_WHITE);
    }
    
    SSD1306_DrawFilledCircle(baseX+graph->cursor*4, baseY+32-graph->data[graph->cursor],2, SSD1306_COLOR_YELLOW);
    SSD1306_GotoXY(baseX+graph->cursor*4-3, baseY+34);
    sprintf(showString, "%u", graph->cursor);
    SSD1306_Puts(showString, &Font_7x10, SSD1306_COLOR_WHITE);
    
    break;
  case SET_POINT:
    break;
    
  }
}



uint8_t GetKeycode(void)
{
  return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
}