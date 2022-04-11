/*
  Example: DiscreteOutput
=
  Copyright (c) 2019 FACTS Engineering, LLC
  Licensed under the MIT license.9
*/

// Include the libraries we need

#include <Arduino.h>        // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#include <Nextion.h>

#include <OneWire.h>
//#include <DallasTemperature.h>
#include <P1AM.h>
//#include <Wire.h>
#include "i2c_Anything.h"

#include <PubSubClient.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>
#include <NTP.h>
#include <arduino-timer.h>


/*
  Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
  void SERCOM1_Handler()
*/

// Data wire is plugged into port 2 on the Arduino
//#define ONE_WIRE_BUS A1
#define TEMPERATURE_PRECISION 12
#define relayPump1 1
#define relayHeater 2
#define relayPump2 3
#define relayPumpCirc 4
#define relayBlower 5
#define relayOzone 6

//RTC on NEextion
//Variable for the temperature being read. "float" variables can store decimal values like 1.23
//
// +++ NTPtime NTPpool("us.pool.ntp.org");   // Choose server pool as required

/*
  The structure contains following fields:
  struct strDateTime
  {
  byte hour;
  byte minute;
  byte second;
  int year;
  byte month;
  byte day;
  byte dayofWeek;
  boolean valid;
  };
*/
// +++ strDateTime dateTime;
// NexRtc  rtc;
// uint32_t  rtcTime[7] = {2017, 9, 25, 12, 34, 00};

// Vars
String hltStr;
char hltChar[12];
char buffer[10];
String keepAliveCallback = "0";


// TOUmoneytSaver
bool TOUsaverEnable = true;

uint8_t TouOffpeakPeriods = 1;
uint8_t TouSuperOffpeakperiods= 1;
uint8_t TouPeakeriods = 1;

//NEYwork
byte mac[] = {0x60, 0x52, 0xD0, 0x06, 0xAE, 0xEE};
IPAddress ip (192, 168, 1, 40);
IPAddress ethServer(192, 168, 1, 1);
IPAddress myMQTTserver(192, 241, 210, 37);
EthernetClient ethClient;
PubSubClient mqtt(ethClient);
String localIP;


float TmaxSet = 106;
float TminSet = 80;

// create a timer that holds 16 tasks, with millisecond resolution,
// and a custom handler type of 'const char *
// Timer<12, millis> t_timer;

auto timer1 = timer_create_default(); // create a timer with default settings
auto timer2 = timer_create_default(); ////Timer <> timer5; // save as a
auto timer3 = timer_create_default(); ////Timer <> timer5; // save as
auto timerP1 =  timer_create_default(); //
auto timerP2 =  timer_create_default(); //
auto timerP0 =  timer_create_default(); //
auto timerBlower =  timer_create_default(); //
auto timerOZwait =  timer_create_default(); //
auto timerOZon =  timer_create_default(); //
uintptr_t tsk_blower, tsk_animate, tsk_edit;
 

bool ButtonPushed = false;
// create a9 timer that hold  s 16 tasks, with millisecond resolution,
// and a custom handler type of 'const char *

bool dummy(void *)
{

  return true;
}

//State = 0; // to hold the current state of the switc
//uint32_t DallasTemperature sensors(&oneWire);
//our Switch (Pin 31) to be an SWinput
bool switchState,
    showSetpoint = false, flagFlow = true, tempPlausible= true;

uint16_t tempRXcount=0;    
uint32_t SampleTime_1 = 0, SampleTime_2 = 0, StatusUpdtTime = 0, tempRXCheck = 0;
uint32_t StatusInterval = 1500, SampleInterval = 5000, BootTIme, timeNow;
uint32_t tempRXInterval = 29000, tempRXTimeLeft = 0;
int variable1 = 0; // This is a simple variable to  increasing a number to have something dynamic to show on the displa

int CurrentPage = 0; // Create a variable to store which page is currently loaded

float tempSetpoint = 100.0;


volatile struct STRUCT
{
  char ver;
  float tubTemp;
  float heaterTemp;
  float addTemp;
} iic_dataStruct;


volatile float tubTemp = 0.0, heaterTemp = 0.0, addTemp = 0.0;
volatile boolean haveData = false;

// Declare objects that we are going to read from the display. This includes buttons, sliders, text boxes, etc:
// Format: <type of object> <object name> = <type of object>(<page id>, <object id>, "<object name>");
/* ***** Types of objects:
   NexButton - Button
   NexDSButton - Dual-state Button
   NexHotspot - Hotspot, that is like an invisible button
   NexCheckbox - Checkbox
   NexRadio - "Radio" checkbox, that it's exactly like the checkbox but with a rounded shape
   NexSlider - Slider
   NexGauge - Gauge
   NexProgressBar - Progress Bar
   NexScrolltext - Scroll text box
   NexNumber - Number box
   NexVariable - Variable inside the nextion display
   NexPage - Page touch event
   NexGpio - To use the Expansion Board add-on for Enhanced Nextion displays
   NexRtc - To use the real time clock for Enhanced Nextion displays
 * *****
*/
NexDSButton bt1 = NexDSButton(0, 21, "bt1"); // Button added
NexDSButton bt2 = NexDSButton(0, 22, "bt2"); // Button added
NexDSButton bt3 = NexDSButton(0, 23, "bt3"); // Button added
NexDSButton bt4 = NexDSButton(0, 24, "bt4"); // Button added
NexDSButton bt5 = NexDSButton(0, 25, "bt5"); // Button added
/*
  NexButton b2 = NexButton(0, 14,"b2");  // Button added
  NexButton b3 = NexButton(0, 3, "b3");  // Button added
  NexButton b5 = NexButton(0, 6, "b5");  // Butt Butt
  NexButton b4 = NexButton(0, 4, "b4");  // Button added
*/
NexButton b5 = NexButton(1, 6, "b5"); // pAGE RERTURN TO page0
NexButton b6 = NexButton(0, 1, "b6"); // UP button  added
NexButton b7 = NexButton(0, 8, "b7"); // DOWN button

// Declare pages:
// Sending data to the display to nonexistent objects on the current page creates an error code sent by the display.
// Any error sent by the display creates lag on the arduino loop because arduino tries to read it, thinking it's a touch event.
// So to avoid this, I am only going to send data depending on the page the display is on.
// That's the reason I want the arduino to know which page is loaded on the display.
// To let arduino know what page is currently loaded, we are creating a touch event for each page.
// On the nextion project, each page most send a simulated "Touch Press Event" in the "Preinitialize Event" section so
// we can register that a new page was loaded.
NexPage page0 = NexPage(0, 0, "page0"); // Page added as a touch event
NexPage page1 = NexPage(1, 0, "page1"); // Page added as a touch event
//NexPage page2 = NexPage(2, 0, "page2");  // Page added as a touch event

// End of declaring objects=

// Declare touch event objects to the touch event list:
// You just need to add the names of the 0objects that send a touch event.
// Format: &<object name>,

NexTouch *nex_listen_list[] =
    {
        &bt1, // Button added
        &bt2,
        &bt3,
        &bt4,
        &bt5,
        &b6,
        &b7,
        &b5,
        &page0, // Page added as a touch event
        &page1, // Page added as a touch event
        NULL    // String terminated
};              // End of touch event list

uint32_t bt1state = 0; // Create variable to store value we are going to get
uint32_t bt2state = 0; // Create variable to store value we are going to get
uint32_t bt3state = 0; // Create variable to store value we are going to get
uint32_t bt4state = 0; // Create variable to store value we are going to get
uint32_t bt5state = 0; // Create variable to store value we are going to get

uint8_t bt1latch = 0, bt2latch = 0, bt3latch = 0, bt4latch = 0, bt5latch = 0;
uint8_t PumpCircLatch = 0;

// auto beat = t_timer;
// auto flashLCD = t_timer;

/********************************/
//  protoypes

void publishState(char *message, bool state);
void  resetSensorCheck();
void updatePump1(byte *message, u_int8_t len);
void updatePump2(byte *message, u_int8_t len);
void updateBlower(byte *message, u_int8_t len);
void updatePumpCirc(byte *message, u_int8_t len);
void updateOzome(byte *message, u_int8_t len);
void start_flash_temp();
void writeSetpoint(float value);
void CircIndicator(bool vis);
/********************************/

inline String BoolToStringOnOff(bool b)
{
  return b ? "ON" : "OFF";
}

inline String BoolToString(bool b)
{
  return b ? "true" : "false";
}

void SnowNan(bool vis)
{
  Serial1.write("vis t10,"); //  nextion SNOW MAN
  Serial1.write(0x30 + (uint8_t)vis);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
}

void HeaterSym(bool vis)
{
  Serial1.write("vis t4,"); //  nextion SNOW MAN
  Serial1.write(0x30 + (uint8_t)vis);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
}

void activeHeat(uint32_t pco)
{
  Serial1.print("t1.pco=");
  Serial1.print(String(pco));
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
}

void OzoneSym(uint8_t vis)
{
  Serial1.write("vis t5,"); //  nextion SUN ON
  Serial1.write(0x30 + vis);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
}

/*
  Serial1.write("vis t6,0");    //  nextio MUSIC ON
  Serial1.write(0xff);  Serial1.write(0xff);  Serial1.write(0xff);




  Serial1.write("vis t9,0");    //  nextio NO FLOW 
  Serial1.write(0xff);  Serial1.write(0xff);  Serial1.write(0xff);


  Serial1.write("vis t10,0");    //  nextion SNOW MAN
  Serial1.write(0xff);  Serial1.write(0xff);  Serial1.write(0xff);

*/

void  FlowIndicator(bool vis)
{
  Serial1.write("vis t8,");    //  nextioN no flow
  Serial1.write(0x30 + (uint8_t)vis);
  Serial1.write(0xff);  Serial1.write(0xff);  Serial1.write(0xff);
  Serial1.write("vis t9,");    // 
    Serial1.write(0x30 + (uint8_t)vis); 
  Serial1.write(0xff);  Serial1.write(0xff);  Serial1.write(0xff);
}

void SleepIndicator (bool vis)
{{
  Serial1.write("vis t7,");    //  nextion ZZZ

  Serial1.write(0x30 + (uint8_t)vis);
  Serial1.write(0xff);  Serial1.write(0xff);  Serial1.write(0xff);
}}

void CircIndicator(bool vis)
{
  Serial1.write("vis t11,");    //  nextion RECIRCULATE
  Serial1.write(0x30 + (uint8_t)vis);
  Serial1.write(0xff);  Serial1.write(0xff);  Serial1.write(0xff);
}

void ChemIndicator(bool vis)
{
  Serial1.write("vis t12,");    //  nextion CHEMICALS
  Serial1.write(0x30 + (uint8_t)vis);
  Serial1.write(0xff);  Serial1.write(0xff);  Serial1.write(0xff);
}

void setButtonStates()
{
  bt1.setValue(bt1state);
  bt2.setValue(bt2state);
  bt3.setValue(bt3state);
  bt4.setValue(bt4state);
  bt5.setValue(bt5state);
}

bool getState(byte *message, uint8_t len)
{
  bool state;
  char *msg = (char *)malloc(sizeof(byte) * len);
  memcpy(msg, message, len);
  String str(msg);
  Serial.println(str);
  if (str.startsWith("ON"))
    state = true;
  if (str.startsWith("OFF"))
    state = false;

  Serial.print(" | ");
  Serial.println(BoolToString(state));
  free(msg);
  return state;
}

bool blowerTO(void *)
{
  timerBlower.cancel();
  bool state = false;
  P1.writeDiscrete(state, 1, relayBlower);
  bt3state = (uint8_t)state;
  bt3latch = (uint8_t)state;
  bt3.setValue(bt3state);
  publishState("P1AM/sta/Blower", state);
  return state;
}

void updateBlower(byte *message, u_int8_t len)
{
  bool state = getState(message, len);
  P1.writeDiscrete(state, 1, relayBlower);

  if (state) timerBlower.in(3000, blowerTO);
  Serial.print("task handle: ");
  Serial.println(String(tsk_blower));
  Serial.println();
  bt3state = (uint8_t)state;
  bt3latch = (uint8_t)state;
  bt3.setValue(bt3state);
  publishState("P1AM/sta/Blower", state);
}

void updatePump1(byte *message, u_int8_t len)
{
  bool state = getState(message, len);
  P1.writeDiscrete(state, 1, relayPump1);
  bt4state = (uint8_t)state;
  bt4latch = (uint8_t)state;
  bt4.setValue(bt4state);
  publishState("P1AM/sta/Pump1", state);
}

bool Pump1Timer(void *)
{
  bool state = false;

  P1.writeDiscrete(state, 1, relayPump2);
  bt5state = (uint8_t)state;
  bt5latch = (uint8_t)state;
  bt5.setValue(bt5state);
  publishState("P1AM/sta/Pump2", state);
}

void updatePump2(byte *message, u_int8_t len)
{
  bool state = getState(message, len);
  P1.writeDiscrete(state, 1, relayPump2);
  bt5state = (uint8_t)state;
  bt5latch = (uint8_t)state;
  bt5.setValue(bt5state);
  publishState("P1AM/sta/Pump2", state);
}

void updatePumpCirc(byte *message, u_int8_t len)
{
  bool state = getState(message, len);
  P1.writeDiscrete(state, 1, relayPumpCirc);
  publishState("P1AM/sta/PumpCirc", state);
  PumpCircLatch = state;
  CircIndicator(state);
  resetSensorCheck();
}


void updateOzone(byte *message, u_int8_t len)
{
  bool state = getState(message, len);
  P1.writeDiscrete(state, 1, relayOzone);
  publishState("P1AM/sta/Ozone", state);
  OzoneSym(state);
}

void updateTempSet(byte *message, u_int8_t len)
{
  
  float newset;


  char *p = (char *)malloc(sizeof(byte) * len);
  memcpy(p, message, len);
  p[len] = 0x00;

  String value(p);
  newset = value.toFloat();
  
  if ((newset > TminSet) && (newset < TmaxSet))
  {
    Serial.println(value);
    tempSetpoint = newset;
    mqtt.publish("P1AM/sta/setT", String(tempSetpoint, 1).c_str());
    start_flash_temp();
  }


  free(p);
}

////////////////////////////////////////////f/////////////////////// Callbacks

//--------------------------------------------------------------------END  CALLBACK

void receiveEvent(int howMany)
{
  if (howMany >= (sizeof iic_dataStruct))
  {
    I2C_readAnything(iic_dataStruct);
    haveData = true;
  } // end if have enough da
} // end of receiveEvent

///////////////////////////////////////////////////////////////////
// supplied as a reference - persistent allocation required
//const functionPtr callbackArr[] = { hi };

/////////////////////////////////////////////////////////////////

void onButtonEvent()
{
  /*
  SampleTime_1 = millis() + SampleInterval/2;
  SampleTime_2 = millis() + SampleInterval;
*/
}

void GetNTPsetRTC()
{
  /*
  while (!dateTime.valid) {
    // getNTPtime: First parameter: Time zone in floating point (for India); second parameter: 1 for European summer time; 2 for US daylight saving time (contributed by viewwer, not tested by me)
    dateTime = NTPool.getNTPtime(1.0, 1);

    // check dateTime.valid before using the returned time
    // Use "setSendInterval" or "setRecvTimeout" if required
    Serial.p`rint(".");
    
  }
  NTPpool.printDateTime(dateTime);
  rtcTime[0] = dateTime.year;
  rtcTime[1] = dateTime.month;
  rtcTime[FD=2] = dateTime.day;
  rtcTime[3] = dateTime.hour;
  rtcTime[4] = dateTime.minute;
  rtcTime[5] = daeTime.second;

  rtc.write_rtc_time(rtcTime);
  */
}

void updateFromPage1()
{

  bt1.getValue(&bt1state);
  bt2.getValue(&bt2state);
  bt3.getValue(&bt3state);
  bt4.getValue(&bt4state);
  bt5.getValue(&bt5state);

  bt1latch = (uint8_t)bt1state;
  bt2latch = (uint8_t)bt2state;
  bt3latch = (uint8_t)bt3state;
  bt4latch = (uint8_t)bt4state;
  bt5latch = (uint8_t)bt5state;

  digitalWrite(LED_BUILTIN, bt1latch);
  HeaterSym(bt2latch);
  /*
  P1.writeDiscrete(bt3latch, 1, relayBlower);
  P1.writeDiscrete(bt4latch, 1, relayPump1);
  P1.writeDiscrete(bt5latch, 1, relayPump2);
  */
}
void publishState(char *topic, bool state)
{

  // hltStr.toCharArray(hltChar, hltStr.length()+1);
  mqtt.publish(topic, BoolToStringOnOff(state).c_str());
}

void callback(char *topic, byte *payload, unsigned int length)
{

  if (!(strcmp(topic, "P1AM/Pump1")))
  {
    updatePump1(payload, length);
  }
  else if (!(strcmp(topic, "P1AM/Pump2")))
  {
    updatePump2(payload, length);
  }
  else if (!(strcmp(topic, "P1AM/PumpCirc")))
  {
    updatePumpCirc(payload, length);
  }
  else if(!(strcmp(topic, "P1AM/Blower")))
  {
    updateBlower(payload, length);
  }
  else if (!(strcmp(topic, "P1AM/Ozone")))
  {
    updateOzone(payload, length);
  }
  else if (!(strcmp(topic, "P1AM/setT")))
  {
    updateTempSet(payload, length);
  }

  Serial.println(String(topic));

  //Serial.println(payload
}

////////////////////////// Touch events:
// Each of the following sections are going to run everytime the touch event happens:
// Is going to run the code inside each section only ones for each touch event.

void bt1PushCallback(void *ptr) // Press event for button b1
{

  bt1latch = !bt1latch;
  /*Wire.requestFrom(8, 4);    // request 4 bytes from slave device #8req 
  
  while (Wire.available()) { // slave may send less than requested
      char c = Wire.read(); // receive a byte as character
    Serial.print(c);         // print the character
  }
  */
  bt1state = (uint8_t)bt1latch;

} // End of press event

void bt1PopCallback(void *ptr) // Release event for button b1
{
  // bt1.getValue(&bt1state);
  /*Wire.requestFrom(8, 4);    // request 4 bytes from slave device #8 
   
  while (Wire.available()) { // slave may send less than requested
      char c = Wire.read(); // receive a byte as character
    Serial.print(c);         // print the character
  }
*/
  digitalWrite(LED_BUILTIN, bt1state); // Turn ON internal LED
  bt1.setValue(bt1state);
} // End of release event

void bt2PopCallback(void *ptr) // Release event for button b1
{
   bt2.setValue(bt2state);
  digitalWrite(LED_BUILTIN, LOW); // Turn OFF internal LED

  // bt2.getValue(&bt2state);  // Read value of dual state button to know the state (0 or 1)
  // HeaterSybt2state);

} // End of release event

void bt2PushCallback(void *ptr) // Press event for button b0
{

  bt2latch = !bt2latch;
  //HeaterSym(bt2latch);
  bt2state = (uint8_t)bt2latch;
  onButtonEvent();

} // End of press event

void bt3PushCallback(void *ptr) // Press event for button b1
{ timerBlower.cancel();
  bt3latch = !bt3latch;
  P1.writeDiscrete(bt3latch, 1, relayBlower);
  bt3state = (uint8_t)bt3latch;
  publishState("P1AM/sta/Blower", bt3state);
} // End of press event

void bt3PopCallback(void *ptr) // Release event for button b1
{
    timerBlower.in(3000, blowerTO);
} // End of release event

void bt4PushCallback(void *ptr) // Press event for button b1
{

  bt4latch = !bt4latch;
  P1.writeDiscrete(bt4latch, 1, relayPump1);

  bt4state = (uint8_t)bt4latch;
  publishState("P1AM/sta/Pump1", bt4state);
} // End of press event

void bt4PopCallback(void *ptr) // Release event for button b1
{

} // End of release event

void bt5PushCallback(void *ptr) // Press event for button b1
{
  bt5latch = !bt5latch;
  P1.writeDiscrete(bt5latch, 1, relayPump2);
  bt5state = (uint8_t)bt5latch;
  publishState("P1AM/sta/Pump2", bt5state);

} // End of press event

void bt5PopCallback(void *ptr) // Release event for button b1
{

} // End of release event


bool stop_flash_temp(void *)
{
  timer3.cancel();
  timer1.cancel(); 
  Serial1.print("vis t1,1"); //This is to hide or show an object. The name of the object comes before the comma, and after the comma goes a 0 to hide or 1 to show.
  Serial1.write(0xff);       // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);
  showSetpoint = false;
  return true;
}

void OnPop (){
  
  timer2.cancel();
  ButtonPushed=false;
}

void OnPush (){
  
  ButtonPushed = true;
}




bool settingscancel(void *)
{
  stop_flash_temp(nullptr);
  timer1.cancel();
  timer2.cancel();
  timer3.cancel();

  
  showSetpoint = false;
  return false;
}

void b5PopCallback(void *ptr) // Release event for button b1
{

} // End of elease event

void b5PushCallback(void *ptr) // Press event for button b1
{

} // End of press event

bool settingbegin(void *)
{
  timer1.cancel();
  showSetpoint = true;
  writeSetpoint(tempSetpoint);
  timer1.in(4000, settingscancel);
  start_flash_temp();
  return true;
}

bool addup(void *)
{
  if (tempSetpoint <= (TmaxSet - .5))
  {
    tempSetpoint = tempSetpoint + 0.5;  
    writeSetpoint(tempSetpoint);
  //  settingbegin(nullptr);
    return true;
  }
  else
  {
    return false;
  }
}

bool subdwn(void *)
{
  if (tempSetpoint >= (TminSet + .5))
  {
    tempSetpoint = tempSetpoint - 0.5;
   // settingbegin(nullptr);
     writeSetpoint(tempSetpoint);
    return true;
  }
  else
  {
    return false;
  }
}

void b6_UP_PopCallback(void *ptr) // Release event for button b1
{
 OnPop();
} // End of elease event

void b6_UP_PushCallback(void *ptr) // Press event for button b1
{
  OnPush();
  settingbegin(nullptr);
  addup(nullptr);
  timer2.every(300, addup);


} // End of press event

void b7_DOWN_PushCallback(void *ptr) // Release event for button b1
{
  OnPush();
  settingbegin(nullptr);
  subdwn(nullptr);
  timer2.every(300,subdwn);
} //\ End of release eventvoid bt5PushCallback(void *ptr)  // Press event for button b1

void b7_DOWN_PopCallback(void *ptr) // Release event for button b1
{
 OnPop();

} // End of release event

// Page change event:
void page0PushCallback(void *ptr) // If page 0 is loaded on the display, the following is going to execute:
{
  CurrentPage = 0;

  // Set variable as 0 so from now on arduino knows page 0 is loaded on the display
} // End of press event

// Page change event:
void page1PushCallback(void *ptr) // If page 1 is loaded on the display, the following is going to execute:
{
  CurrentPage = 1; // Set variable as 1 so from now on arduino knows page 1 is loaded on the display
} // End of press event

bool toggle_led(void *)
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // toggle the LED
  return true;                                          // repeat? true
}

bool print_message(const char *m)
{
  Serial.print("print_message: ");
  Serial.println(m);
  return true; // repeat? true
}

size_t repeat_count = 5;

bool repeat_x_times(void *opaque)
{
  size_t limit = (size_t)opaque;
  Serial.print("repeat_x_times: ");
  Serial.print(repeat_count);
  Serial.print("/");
  Serial.println(limit);

  return ++repeat_count <= limit; // remove this task after limit reached
}

bool flash_temp(void *)
{
  static bool flag;
  if (flag && !ButtonPushed)
  {
    Serial1.print("vis t1,0"); //This is to hide or show an object. The name of the object comes before the comma, and after the comma goes a 0 to hide or 1 to show.
    Serial1.write(0xff);       // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  if(!flag)
  {
    Serial1.print("vis t1,1"); //This is to hide or show an object. The name of the object comes before the comma, and after the comma goes a 0 to hide or 1 to show.
    Serial1.write(0xff);       // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  flag = !flag;
  return true;
}


void start_flash_temp()
{
  timer3.cancel();
  timer1.cancel();
  timer3.every(300, flash_temp);
  timer1.in(5009, stop_flash_temp);
}


void initPage0()
{

  Serial1.print("vis t0,0"); //This is to hide or show an object. The name of the object comes before the comma, and after the comma goes a 0 to hide or 1 to show.
  Serial1.write(0xff);       // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.write("vis g0,0"); // sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.  // This is the value you want to send to that object and atribute mention before.
  Serial1.write(0xff);       // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.write("vis x0,0"); // sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.  // This is the value you want to send to that object and atribute mention before.
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.write("vis t4,0"); //  nextion HEATER ON
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.write("vis t5,0"); //  nextion SUN ON
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.write("vis t6,0"); //  nextio MUSIC ON
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.write("vis t7,0"); //  nextioN SLEEP ON
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.write("vis t8,0"); //  nextio NO FLOW
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write("vis t9,0"); //  nextion SUN ON
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.write("vis t10,0"); //  nextion SNOW MAN
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.write("vis t11,0"); //  nextion RECIRCULATE
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
  
  Serial1.write("vis t12,0"); //  nextion CHEMICALS
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  HeaterSym((uint8_t)bt1latch);
}

void switchToggles()
{ // the loop routine runs over and over again forever:

  switchState = digitalRead(SWITCH_BUILTIN); //Read the state of the switch
  digitalWrite(LED_BUILTIN, switchState);    //Wait for Modules to Sign on
}

void reconnect()
{

  if (!mqtt.connected())
  {
    Serial.print("Attempting  connection ... ");
    // Attempt to connect
    if (mqtt.connect("P1AM-100"))
    {
      Serial.println("connected");
      // (re)subscribe
      // Client.subscribe(Topic);
      // mqtt.subscribe("P1AM/#");
      mqtt.subscribe("P1AM/keepAlive");
      mqtt.subscribe("P1AM/setT");
      mqtt.subscribe("P1AM/PumpCirc");
      mqtt.subscribe("P1AM/Pump1");
      mqtt.subscribe("P1AM/Pump2");
      mqtt.subscribe("P1AM/Blower");
      mqtt.subscribe("P1AM/Ozone");
    }
    else
    {
      Serial.print("Connection failed, state: ");
      Serial.print(mqtt.connected());

      Serial.println(" retrying in 5 seconds");
    }
  }
}

 void resetSensorCheck()
 {
  tempPlausible= true; 
  tempRXcount=0;
  tempRXCheck = timeNow + 2000 + tempRXInterval;
 }





void setup()
{ // Put your setup code here, to run once:
  BootTIme = millis();
  tempPlausible= true; tempRXcount=4;
  Serial.begin(115200); //initialize serial communication at 115200 bits per second
  while (!P1.init())
  {
    //Wait for Modules to Sign on
  }
  P1.writeDiscrete(HIGH, 1, relayPumpCirc);//Turn slot 1 channel 2
  PumpCircLatch = true;
  CircIndicator(PumpCircLatch);
  //P1.writeDiscrete(HIGH, 1, 6); //Turn sl;ot 1 channel 2

  Serial1.begin(115200); //; Start serial comunication at baud=9600. For Arduino mega you would have to add

  //ArduinoOTA.begin(Ethernet.localIP(), "Arduino", "adv57130", InternalStorage);
  initPage0();

  mqtt.setServer(myMQTTserver, 1883);
  mqtt.setCallback(callback);

  Ethernet.init(5);        //CS pin for P1AM-ETH
  Ethernet.begin(mac, ip); // Get IP from DHCP
  //MQTT

  pinMode(LED_BUILTIN, OUTPUT);   //Set our LED (Pin 32) to be an output
  pinMode(SWITCH_BUILTIN, INPUT); //Set
  // Start up the library

  // Register the event callback functions of each touch event:
  // You need to register press events and release events seperatly.
  // Format for press events: <object name>.attachPush(<object name>PushCallback);
  // Format for release events: <object name>.attachPop(<object name>PopCallback);
  bt1.attachPush(bt1PushCallback); // Button press
  bt1.attachPop(bt1PopCallback);   // Button release
  bt2.attachPush(bt2PushCallback); // Button press
  bt2.attachPop(bt2PopCallback);   // Button press
  bt3.attachPush(bt3PushCallback); // Button press
  bt3.attachPop(bt3PopCallback);   // Button release
  bt4.attachPush(bt4PushCallback); // Button press
  bt4.attachPop(bt4PopCallback);   // Button release
  bt5.attachPush(bt5PushCallback); // Button press
  bt5.attachPop(bt5PopCallback);   // Button release
  b5.attachPush(b5PushCallback);   // Button press
  b5.attachPush(b5PopCallback);    // Button press

  b6.attachPush(b6_UP_PushCallback);   // Button press
  b6.attachPop(b6_UP_PopCallback);     // Button pre
  b7.attachPush(b7_DOWN_PushCallback); // Button press
  b7.attachPop(b7_DOWN_PopCallback);   // Button pre
  page0.attachPush(page0PushCallback); // Page press event
  page1.attachPush(page1PushCallback); // Page press event

  bt1.getValue(&bt1state);
  bt2.getValue(&bt2state);
  bt3.getValue(&bt3state);
  bt4.getValue(&bt4state);
  bt5.getValue(&bt5state);

  bt1latch = (uint8_t)bt1state;
  bt2latch = (uint8_t)bt2state;
  bt3latch = (uint8_t)bt3state;
  bt4latch = (uint8_t)bt4state;
  bt5latch = (uint8_t)bt5state;
  

  digitalWrite(LED_BUILTIN, bt1latch);
  HeaterSym(bt2latch);


  Wire.begin(8);               // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event

  updateFromPage1();
  SampleTime_1 = millis() + 100;
  SampleTime_2 = millis() + 100 + SampleInterval / 2;
  tempRXCheck = 0;

  // call the toggle_led function every 500 millis (half second)
  //timer.every(500, toggle_led);

  // call the print_message function every 1000 millis (1 second),
  // passing it an argument string
  //t_timer.every(1000, print_message, "called every second");

  // call the print_message function in five seconds
  //t_timer.in(5000, print_message, "delayed five seconds");

  // call the print_message function at time + 10 seconds
  //t_timer.at(millis() + 10000, print_message, "call at millis() + 10 seconds");

  //imer.every(300, flash_temp);
} //end of Setup

// funct to print a device address
/*
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
*/

void writetubTemp(float value)
{
  if (!showSetpoint)
  {
    // After the name of the oect you need to put the dot val because val is the atribute we want to change on that object.
    Serial1.print("t1.txt="); // This is) sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
    Serial1.print("\"" + String(value, 2) + "\"");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
}


void writeHeaterTemp(float value)
{
 

    // After the name of the oect you need to put the dot val because val is the atribute we want to change on that object.
    Serial1.print("t2.txt="); // This is) sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
    Serial1.print("\"" + String(value, 2) + "\"");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  
}


void writeSetpoint(float value)
{
  // After the name of the oect you need to put the dot val because val is the atribute we want to change on that object.
  Serial1.print("t1.txt="); // This is) sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial1.print("\"" + String(value, 2) + "\"");
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
}

void loop()
{ // the loop routine runs over and over again forever:
  timeNow = millis();
  char c[10];

  //Nextion display
  nexLoop(nex_listen_list); // Check for any touch event

  //handling messages
  mqtt.loop();
  // ArduinoOTA.poll();

  // t_timer.tick();
  timer2.tick();
  timer1.tick();
  timer3.tick();
  timerP0.tick();
  timerP1.tick();
  timerP2.tick();
  timerBlower.tick(); 
  timerOZwait.tick();
  timerOZon.tick();

  // END  LOOP

   switchToggles();

  //P1.writeDiscrete(HIGH, 1, 1); //Turn slot 1 channel 2 on
  //wait 1 second

  //  Serial2.print("Requesting temperatures...");
  if (haveData)
  {
    Serial.print(iic_dataStruct.ver);
    Serial.print(" - tubTemp: ");
    Serial.print(iic_dataStruct.tubTemp);
    Serial.print(" | ");
    Serial.print("heaterTemp: ");
    Serial.print(iic_dataStruct.heaterTemp);      
    Serial.print(" | ");
    Serial.print("addTemp: ");
    Serial.println(iic_dataStruct.addTemp);
    
    tubTemp = iic_dataStruct.tubTemp;
    writetubTemp(tubTemp);
    heaterTemp = iic_dataStruct.heaterTemp;
    writeHeaterTemp(addTemp);   
    addTemp  = iic_dataStruct.addTemp;
    haveData = false;

    tempRXcount++;
      if (!tempRXCheck) tempRXCheck = timeNow + 7000 + tempRXInterval;
      if (tempRXcount == 4) tempRXTimeLeft = tempRXCheck - millis();
  }

  if (SampleTime_1 < timeNow)
  {
    SampleTime_1 = timeNow + SampleInterval;

    // LOOP
    if (!mqtt.connected())
    {
      Serial.print(" DisFREVed");
      Serial.println("Tring to connect to MNQTT broker");
      reconnect();
    }

    // Read Temperatures
    // hltTemp = P1.readTemperature(2, 1);  //Return temperature read from channel 1 of the input module in slot 2
    hltStr = String(tubTemp);
    hltStr.toCharArray(hltChar, hltStr.length());

    Serial.print("Tub Temperature: ");
    Serial.println(tubTemp, 2); //print the value in degrees Fahrenheit up to 2 decimal places

    writetubTemp(tubTemp);
  }

  if (SampleTime_2 < timeNow)
  {
    //requeststatus = rs_cmomplete;

    SampleTime_2 = timeNow + SampleInterval;
    // call sensors.requestTemperatures() to issue a global temperature
    // request to all devices on the bus
    /*
    Serial.print("Requesting temperatures...");
    sensors.requestTemperaturesByAddress(heaterSensorAddress);``
    Serial.println("DONE");
  
    // print the device information
    printData(heaterSensorAddress);
    heaterTemp = getTempF(heaterSensorAddress);
*/
  
    mqtt.publish("P1AM/sta/heaterTemp", String(addTemp).c_str());
    mqtt.publish("P1AM/sta/tubTemp", String(tubTemp).c_str());


    // We are going to send the varable value to the object called n0:
    // After the name o                               f the oect you need to put the dot val because val is the atribute we want to change on that object.
  }

  if (tempRXCheck < timeNow)
  { 
    tempRXCheck = timeNow + tempRXInterval;

    if (tempRXcount <3) {
        tempPlausible = false;
    }
     
    tempRXcount  = 0;
  }
  if (StatusUpdtTime < timeNow)
  {
    StatusUpdtTime = timeNow + StatusInterval;

    Serial1.print("t3.txt="); // This is) sent0 to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
    Serial1.print("\"Everything is great!\"");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);

    if (tubTemp <= 84.0)
      SnowMan(1);
    else
      SnowMan(0);

    writetubTemp(tubTemp);
  
      
  flagFlow = switchState;
  FlowIndicator(!flagFlow);
  Serial.println("pumpcirclatch: flagfloe, tempePlausible  tempRXcount  tempRXTimeleft");


  Serial.println(String(PumpCircLatch)+" | "+String(flagFlow)
          +" | "+String(tempPlausible)+" | "+String(tempRXcount)+" | "+String(tempRXTimeLeft));


   if ((((tubTemp -0.3 < tempSetpoint) || ((tubTemp + 0.1 < tempSetpoint)))
        && PumpCircLatch &&  flagFlow  && tempPlausible))
    {
      activeHeat(53620);
      P1.writeDiscrete(1,1,relayHeater);
      HeaterSym(true);
    }
    else
    {
      HeaterSym(false);
      P1.writeDiscrete(0,1,relayHeater);
      activeHeat(568);
    }
  }
  /*   Serial1.print("n0.val=");  // This is se   the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
     Serial1.print(String(tubTemp,0));  // Ts is the value  want to send to that object and atribute mention before.
     Serial1.write(0xff);
     Serial1.write(0xff);
     Serial1.write(0xff);
  */
  // We are going to change the color of the progress bar to red if the variable is greater than 49, and change to green if is below 50:
}  // the end