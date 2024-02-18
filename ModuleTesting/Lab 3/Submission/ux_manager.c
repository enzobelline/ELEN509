// ux_manager.c


#include "main.h"
#include "ux_manager.h"
#include "project.h"
#include <stdio.h>

// private defines


// Global Constants


// Modular Constants


// global variables
// Screens
ui_screen currentScreen;
ui_screen lastScreen;

// Display-wrapped values
// format seq (numeric): {<format string>, <error message>, <Xpos>, <Ypos>, <valid?>, <init value>}
DWfloat counter = {"%5.2f", "----", 0, 0, true, 0};
DWfloat tempInF = {"%4.1f", "----", 0, 0, true, 72.2};
DWfloat humidity = {"%4.1f", "----", 0, 0, true, 40.1};
DWint16_t tempCJ_F = {"%5d", "!!!!", 0, 0, true, 0};
// format seq (string): {<format string>,  <error message>, <Xpos>, <Ypos>, <valid?>, "<init value>"

// Graph variables
bargraph8x32_t myGraph = {{4,8,6,9,20,12,15,10}, {'a','b','c','d','e','f','g','h'}, {'4','3','2','1'}, "myGraph", 15, 16, true};
uint8_t graphUpdated = true;
// modular variables


// module prototypes
void ShowGraph(ui_screen _screen_no, bargraph8x32_t* graph);


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
    SSD1306_Puts ("Main Screen", &Font_11x18, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY (0, 30);
    SSD1306_Puts ("Temp: ", &Font_11x18, SSD1306_COLOR_WHITE);
    // Set u X/Y coordinates for "live" data to be displayed on this screen
    tempInF.xPos = 55;
    tempInF.yPos = 30;
    // Send a screen update (note this does not update the live data)
    SSD1306_UpdateScreen(); //display
    break;
  case SHOW_TEMP:
    SSD1306_Clear();
    SSD1306_GotoXY (0,0);
    SSD1306_Puts ("Ambient Tmp", &Font_11x18, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY (0, 20);
    SSD1306_Puts ("DegF: ", &Font_11x18, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY (0, 40);
    SSD1306_Puts ("Count: ", &Font_11x18, SSD1306_COLOR_WHITE);
    tempInF.xPos = 55;
    tempInF.yPos = 20;
    counter.xPos = 60;
    counter.yPos = 40;
    break;
  case SHOW_HUM:
    SSD1306_Clear();
    ShowGraph(screen_no, &myGraph);
    break;
  }

  currentScreen = screen_no;
  
#pragma diag_suppress= Pa149
  // what must be done after screen is switched in
  switch (currentScreen) {
  }
#pragma diag_warning= Pa149
  
}


// Keyboard Processor

//uint8_t ProcessKeyCode (uint16_t key_code)
//{
//  switch (key_code) {
//  case 0:
//    break;
//  case 1:
//    break;
//  case 2:
//    break;
//  case 3:
//    break;
//  }
//  
//  return true;
//}

void ProcessKeyCode(uint16_t key_code)
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
uint8_t ProcessKeyCodeInContext (uint16_t key_code)
{
  switch (currentScreen) {
  case  MAIN:
    switch (key_code) {
    case 0:
      break;
    case 1:
      SwitchScreens(SHOW_TEMP);
      break;
    case 2:
      SwitchScreens(SHOW_HUM);
      break;
    case 3:
      break;
    }
    break;
  case  SHOW_TEMP:
    switch (key_code) {
    case 0:
      break;
    case 1:
      SwitchScreens(MAIN);
      break;
    case 2:
      SwitchScreens(SHOW_HUM);
      break;
    case 3:
      break;
    }
    break;
  case  SHOW_HUM:
    switch (key_code) {
    case 0:
      break;
    case 1:
      SwitchScreens(MAIN);
      break;
    case 2:
      SwitchScreens(SHOW_HUM);
      break;
    case 3:
      break;
    }
    break;
  case  SET_TEMP:
    switch (key_code) {
    case 0:
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    }
    break;
  case  SET_HUM:
    switch (key_code) {
    case 0:
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    }
    break;
  case  SET_TIME:   
    switch (key_code) {
    case 0:
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    }
    break;
  }
  
  SSD1306_UpdateScreen();
  
  return true;
}




void UpdateScreenValues(void)
{
  char displayString[25];
  
  switch (currentScreen) {
  case MAIN:
    SSD1306_GotoXY (tempInF.xPos, tempInF.yPos);
    if (tempInF.valid) {
      sprintf(displayString, tempInF.format, tempInF.data);
      SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
    }
    else 
      SSD1306_Puts(tempInF.invalidMsg, &Font_11x18, SSD1306_COLOR_WHITE);
    break;
  case SHOW_TEMP:
    SSD1306_GotoXY (tempInF.xPos, tempInF.yPos);
    if (tempInF.valid) {
      sprintf(displayString, tempInF.format, tempInF.data);
      SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
    }
    else 
      SSD1306_Puts(tempInF.invalidMsg, &Font_11x18, SSD1306_COLOR_WHITE);
    
    SSD1306_GotoXY (counter.xPos, counter.yPos);
    if (counter.valid) {
      sprintf(displayString, counter.format, counter.data);
      SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
    }
    else
      SSD1306_Puts(counter.invalidMsg, &Font_11x18, SSD1306_COLOR_WHITE);
    break;
  case SHOW_HUM:
    break;
  case SET_TEMP:
    break;
  case SET_HUM:
    break;
  case SET_TIME:
    break;
  }
  SSD1306_UpdateScreen(); //display
}

void UpdateGraph(void){
  switch (currentScreen) {
    case MAIN:
      break;
    case SHOW_HUM:
      if (graphUpdated){ //set graphUpdated to true everytime it changes
        graphUpdated = false;
        SSD1306_Clear();
        ShowGraph(SHOW_HUM, &myGraph);
      }
      break;
    case SET_TEMP:
      break;
    case SET_HUM:
      break;
    case SET_TIME:
      break;
  }
}


void ShowGraph(ui_screen _screen_no, bargraph8x32_t* graph)
{
  uint16_t baseX;
  uint16_t baseY;
  uint8_t i;
  
  
  baseX = graph->xPos;
  baseY = graph->yPos;
  
  switch (_screen_no) {
  case MAIN:
    break;
  case SHOW_TEMP:
    break;
  case SHOW_HUM:
    
    SSD1306_GotoXY (0,0); //Title on top
    SSD1306_Puts (&graph->title[0], &Font_7x10, SSD1306_COLOR_WHITE);
    
    SSD1306_DrawLine(baseX, baseY, baseX, baseY+32, SSD1306_COLOR_WHITE); //vert axis
    SSD1306_DrawFilledTriangle(baseX-2,baseY,baseX+2, baseY, baseX, baseY-3, SSD1306_COLOR_WHITE);
    
    SSD1306_DrawLine(baseX, baseY+32, baseX+83, baseY+32, SSD1306_COLOR_WHITE); // horiz axis
    SSD1306_DrawFilledTriangle(baseX+83,baseY+30,baseX+83, baseY+34, baseX+86, baseY+32, SSD1306_COLOR_WHITE);    
    
    for (i = 0; i < 8; i++) {
      SSD1306_GotoXY((baseX+2)+(i*10), baseY+35);
      SSD1306_Putc(graph->xLabel[i], &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_DrawFilledRectangle((baseX+2)+(i*10), baseY+32-graph->data[i], 8, graph->data[i], SSD1306_COLOR_WHITE);
      
    }
    
    for(i = 0; i < 4; i++){
      SSD1306_DrawLine(baseX-2, baseY+(i*8), baseX+2, baseY+(i*8), SSD1306_COLOR_WHITE);
      SSD1306_GotoXY((baseX-9), baseY-3+(i*8));
      SSD1306_Putc(graph->yTicks[i], &Font_7x10, SSD1306_COLOR_WHITE);
    }
    
    break;
  case SET_TEMP:
    break;
  case SET_HUM:
    break;
  case SET_TIME:
    break;
  }
}


uint8_t GetKeycode(void)
{
  return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
}