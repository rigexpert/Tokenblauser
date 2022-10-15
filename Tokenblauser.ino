//
// TOKENBLAUSER GPSDO
//
// Go to: Tools - Board - Boards manager, install Arduino SAMD boards
// Select: Tools - Board - Arduino SAMD - Arduino Zero (Native USB Port)
//
// Go to: Tools - Manage libraries, install Adafroit GFX Library, Adafruit SSD1306, SAMD_TimerInterrupt, FlashStorage_SAMD
// 
// Put TDC7200.h and TDC7200.cpp into the Tokenblauser folder
//
// Arduino pin names and numbers - see: 
// C:\Users\<UserName>\AppData\Local\Arduino15\packages\arduino\hardware\samd\1.8.13\variants\arduino_zero\
//


#include "wiring_private.h" // pinPeripheral() function

// https://learn.adafruit.com/adafruit-gfx-graphics-library
#include <Adafruit_GFX.h>

#include <Fonts/FreeMonoBold12pt7b.h>
#define FONT_ARRAY FreeMonoBold12pt7b

// https://www.arduino.cc/reference/en/libraries/adafruit-ssd1306/
#include <Adafruit_SSD1306.h>

// https://www.arduino.cc/reference/en/libraries/samd_timerinterrupt/
// Need version 1.9.9 (version 1.10.1 does not work)
#include "SAMDTimerInterrupt.h"

// https://www.arduino.cc/reference/en/libraries/flashstorage_samd/
#include "FlashStorage_SAMD.h"

// https://github.com/Yveaux/TDC7200
#include "TDC7200.h"

// SI5338 support: AN428
// https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN428.pdf

// PCB version

//#define DEVICE_REVISION 9 // Initial breakout 0.9
#define DEVICE_REVISION 10 // First prototype 1.0

// Firmware version

#define TODAYS_REVISION 0
static const uint32_t FIRMWARE_VERSION =
// Convert from "dd mmm yyyy" to numerical yyyymmdd
(__DATE__[7]-'0') * 100000000 +
(__DATE__[8]-'0') * 10000000 +
(__DATE__[9]-'0') * 1000000 +
(__DATE__[10]-'0') * 100000 +
(   (__DATE__[0]=='J') ? ((__DATE__[1]=='a') ? 1 : ((__DATE__[2]=='n') ? 6 : 7))
  : (__DATE__[0]=='F') ? 2
  : (__DATE__[0]=='M') ? ((__DATE__[2]=='r') ? 3 : 5)
  : (__DATE__[0]=='A') ? ((__DATE__[1]=='p') ? 4 : 8)
  : (__DATE__[0]=='S') ? 9
  : (__DATE__[0]=='O') ? 10
  : (__DATE__[0]=='N') ? 11
  : (__DATE__[0]=='D') ? 12
  : 0 
) * 1000 +
((__DATE__[4]==' ') ? 0 : (__DATE__[4]-'0')) * 100 +
(__DATE__[5]-'0') * 10 +
TODAYS_REVISION * 1;


// Misc

#define OCXO_FREQ_HZ 10000000UL // Using fixed-point math, floating points do not work for the suggested algorighm for div=a+(b/c)

uint8_t gps_comm_mode = 0;
int transparent_count = 0;

int verbose_level = 1; // 0=min, 1=max


// TIC

#define CNT_PERIOD_PS 2000000 // Period of TIC STOP signal (500 kHz)

int apply_qerr_correction = 1;
int qerr_ps = 0;
int coarse_time_ps = 0; // Coarse timer
int cont_time_ps = 0; // Continuous time
int cont_time_old_ps = 0; // Old valie
int old_offset_time_ps = CNT_PERIOD_PS / 2; //0; // Old offset value for running the coarse timer
int fq_offset_ppt = 0; // Required frequency offset


#define MAX_CONT_TIME_PS   (CNT_PERIOD_PS*10) // Maximum phase steering time, reset coarse timer to 0 after reaching this limit

boolean next_interval_ready = false;

double last_ns_value = 0;

// DAC

boolean hold_mode = false;
#define DAC_MIN 0 
#define DAC_MAX 65535 
#define DAC_MID ( (DAC_MAX-DAC_MIN)/2 + 1 ) 
uint16_t dac_value = DAC_MID;
bool dac_off = false;

// Prefilter

double tic_prefiltered_ps = 0;

int prefilter_time_const = 30; //16; // Time constant in seconds
const int prefilter_time_constants[] = {0, 5, 10, 20, 30, 50, 60, 100 };
boolean need_reset_prefilter = true;


// PI-loop

double loop_i = 0; //690;//0;

int loop_time_const = 100; // Time constant in seconds
const int loop_time_constants[] = {5, 10, 25, 50, 100, 250, 500, 1000, 2000};

int loop_damping_thousands = 3000;
const int loop_damping_thousands_constants[] = {500, 750, 1000, 2000, 3000, 5000, 7500, 10000 };

int tuning_range_ppt = // OCXO tuning range in ppt
#if DEVICE_REVISION>=10
  435000 
#else
  847000
#endif
;

// Lock mode and indication

#define LOCK_LIMIT_PS 200000
#define LOCK_TIME 30
#define UNLOCK_LIMIT_PS 250000
#define UNLOCK_TIME 10

volatile boolean locked_state = false;
volatile unsigned int locked_time = 0;
volatile int lock_unlock_counter = 0;

/*
// This is to avoid a bug when 0 is not displayed with standard print or println
inline void print_int64_t(int64_t n)
{
  if (n == 0)
    SerialUSB.print("0");
  else
    SerialUSB.print(n);
}
*/

// LEDs

#if DEVICE_REVISION>=10
  #define LED_GREEN A5 // MCU PB02
  #define LED_RED 6 // MCU PA20
#else
  #define LED_GREEN A5 // MCU PB02
  #define LED_RED A4 // MCU PA5
#endif

void set_led(int which, boolean state)
{
#if DEVICE_REVISION>=10
  digitalWrite(which, state ? LOW : HIGH);
#else
  digitalWrite(which, state ? HIGH : LOW);
#endif  
}

void toggle_led(int which)
{
  digitalWrite(which, !digitalRead(which));
}

void init_leds()
{
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  set_led(LED_GREEN, false);
  set_led(LED_RED, false);
}


// Serial USB

void init_serial_usb()
{
  SerialUSB.begin(115200); // The speed does not really matter

  // Blocking issue, TODO: check
  // https://github.com/arduino/ArduinoCore-sam/issues/71

  SerialUSB.println("Start");
}

// Serial

#define GPS_BAUDRATE_INITIAL 9600
#define GPS_BAUDRATE_OPERATING 115200

// GPS and UBX

// UBX initializing arrays

// Set 115200 at UART1
uint8_t ubx_cfg_prt1[] = 
//  Header      Class ID    len         port  res1  txReady     mode                    baudRate                inProto     outProto    flags       res2  res2  CK_A  CK_B
//                                      0     1     2     3    4                        8                       12          14          16          18    19
//  ----------  ----------  ----------  ----  ----  ----------  ----------------------  ----------------------  ----------  ----------  ----------  ----  ----  ----  ----
  { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, 0x7E };


// Enable 1PPS only when locked
uint8_t ubx_cfg_tp5[] = { 0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x40, 0x42,
                          0x0F, 0x00, 0x40, 0x42, 0x0F, 0x00, 0x40, 0x42, 0x0F, 0x00, 0x20, 0xA1, 0x07, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0xF7, 0x00, 0x00, 0x00, 0xFC, 0xCB
                        };

// Enable UBX-TIM-TP message at UART1
uint8_t ubx_cfg_msg_tim_tp[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x0D, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x17 };

// Enable UBX-NAV-SAT message at UART1
uint8_t ubx_cfg_msg_nav_sat[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x35, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x46, 0x23 };

// Set Survey-in as Time mode (for u-blox "T" series) - TMODE2
uint8_t ubx_cfg_tmode2[] = { 0xB5, 0x62, 0x06, 0x3D, 0x1C, 0x00, 0x01, 0x00, 0x00, 0x00, 0x9D, 0x31, 0x88, 0x19, 0x3A, 0xC0,
                             0x5B, 0x06, 0x29, 0xE3, 0x5A, 0x1B, 0x01, 0x83, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x00, 0xA0, 0x86,
                             0x01, 0x00, 0x61, 0x3A
                           };

uint8_t gps_enable_init = 1;


uint8_t* ubx_preinit_array[] =
{
  ubx_cfg_prt1
};

uint8_t* ubx_init_array[] =
{
  ubx_cfg_tp5,
  ubx_cfg_msg_tim_tp,
  ubx_cfg_msg_nav_sat,
  ubx_cfg_tmode2
};

int avg_cno = 0;

void init_gps()
{

  // Open the port at initial rate

  Serial1.end();
  delay(100);
  Serial1.begin(GPS_BAUDRATE_INITIAL);

  // Using the initial rate, set the operating rate

  for (int i = 0; i < sizeof(ubx_preinit_array) / sizeof(uint8_t*); i++)
  {
    delay(100);

    uint8_t* pkt = ubx_preinit_array[i];
    uint16_t pkt_len = pkt[4] + (((uint16_t)pkt[5]) << 8) + 8;

    Serial1.write(pkt, pkt_len);
  }

  // Switch the serial port to the operating rate

  Serial1.flush();
  delay(100);
  Serial1.end();
  delay(100);
  Serial1.begin(GPS_BAUDRATE_OPERATING);
  
  for (int i = 0; i < sizeof(ubx_init_array) / sizeof(uint8_t*); i++)
  {
    delay(100);

    uint8_t* pkt = ubx_init_array[i];
    uint16_t pkt_len = pkt[4] + (((uint16_t)pkt[5]) << 8) + 8;

    Serial1.write(pkt, pkt_len);
  }

  avg_cno = 0;
}

// https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf

// Check UBX checksum

bool check_ubx_checksum(const byte* buf, int len)
{
  if (len<4)
    return false;
  
  byte ck_a=0, ck_b=0;
  
  for (int i=2; i<len-2; i++) // RFC1145 algorithm, payload only
  {
    ck_a += buf[i];
    ck_b += ck_a;  
  }

  return (ck_a==buf[len-2]) && (ck_b==buf[len-1]);
}

// Decode UBX

void decode_ubx_message(const byte* buf, int len)
{
  // TODO: check UBX version
  // TODO: check length of payload
  
  byte ubx_class = buf[2];
  byte ubx_id = buf[3];

  // UBX_TIM_TP
  
  if ( (ubx_class==0x0D) && (ubx_id==0x01) && (len==24) ) 
  {
          qerr_ps = ((uint32_t)buf[14]) + (((uint32_t)buf[15])<<8) + (((uint32_t)buf[16])<<16) + (((uint32_t)buf[17])<<24);
//        qerr_ps = *( (int32_t*)(buf + 14) ); // 4 bytes of qErr - this does not work: bug in the compile or the alignment error?
   
        if (abs(qerr_ps) > 50000) // Bug
          qerr_ps = 0;
  }

  // UBX_NAV_SAT
  
  if ( (ubx_class==0x01) && (ubx_id==0x35) ) 
  {
    int num_svs = buf[6+5];

    int used_counter = 0;
    int cno_sum = 0;
    
    for (int i=0; i<num_svs; i++)
    {
      uint32_t flags = (uint32_t)buf[6+16+12*i+0] 
                    + ((uint32_t)(buf[6+16+12*i+1])<<8)
                    + ((uint32_t)(buf[6+16+12*i+2])<<16)
                    + ((uint32_t)(buf[6+16+12*i+3])<<24);

      if (flags & 0b1000)
      {
        used_counter++;
        byte cno = buf[6+10+12*i];
        cno_sum += cno;
      }
    }

    avg_cno = cno_sum/used_counter;
  }
}

// Receive GPS messages in UBX format

#define UBX_BUF_SIZE 500
byte ubx_buf[UBX_BUF_SIZE];
int ubx_index = 0;
int ubx_payload_len = 0;

void receive_gps_byte(byte b)
{
  if (ubx_index>=UBX_BUF_SIZE-1)
  {
    ubx_index = 0;
    ubx_payload_len = 0;
    return; // Bug, overflow
  }

  ubx_buf[ubx_index] = b;
  ubx_index++;

  if ( (ubx_index==1) && (b!=0xB5) )
  {
    ubx_index = 0;
    return; // No header yet
  }
  
  if ( (ubx_index==2) && (b!=0x62) )
  {
    ubx_index = 0;
    return; // No header yet
  }

  if ( ubx_index==5 )
  {
    ubx_payload_len = b;
  }  

  if ( ubx_index==6 )
  {
    ubx_payload_len += ((uint16_t)b)<<8;
  }  
  
  if ( (ubx_index>8) && (ubx_index==ubx_payload_len+8) )
  {
    if (check_ubx_checksum(ubx_buf, ubx_index))
      decode_ubx_message(ubx_buf, ubx_index);
      
    ubx_index = 0;
    ubx_payload_len = 0;
  }
}


// Display

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint8_t display_brightness = 1;

void init_display()
{

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    SerialUSB.println("SSD1306 allocation failed");
    for (;;); // Don't proceed, loop forever
  }

  // Set font
  display.setFont(&FONT_ARRAY);

  display.setTextSize(1, 1);
  display.setTextColor(SSD1306_WHITE);

  // Clear the buffer
  display.clearDisplay();

//  display.display();
}

void test_screen()
{
  // Draw the screen adjustment border
  display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.display();
  while(1); // Forever
}

// External interrupts

volatile int timeout_1pps = 0;
volatile bool present_1pps = false;
volatile bool skip_next1pps = true;

void PpsHandler() // GPS PPS interrupt
{
  timeout_1pps = 0;
  present_1pps = true;
}

#if DEVICE_REVISION>=10
//  #define PIN_PPS 31 // PB23 - cannot attach interrupt  
//  #define PIN_PPS 30 // PB22 - cannot attach interrupt  
  #define PIN_PPS 38 // PA13  
#else
  #define PIN_PPS 2 // PIN_PA14  
#endif



void  init_ext_interrupts()
{
  pinMode(PIN_PPS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_PPS), PpsHandler, RISING);
}


// Timer interrupt

SAMDTimer ITimer0(TIMER_TC3);
//volatile int dec_1s_counter = 0;
volatile int counter_2s = 0;
volatile boolean flag_2s = false;
//volatile boolean flag_1s = false;
volatile boolean flag_05s = false;
volatile boolean flag_blinker = false;
void button_timer_10ms();

void TimerHandler10ms() // 10ms timer
{

  button_timer_10ms();

  counter_2s++; // 2s counter
  
  if (counter_2s%50 == 0)
  {
    flag_05s = true;
  }

  if (counter_2s%100 == 0)
  {
//    flag_1s = true;

    if (locked_state)
      locked_time++;
  }

  if (counter_2s >= 200)
  {
    flag_2s = true;
    counter_2s = 0;
  }
  
  if (timeout_1pps >= 200) // 2 sec timeout
  {
    present_1pps = false;
    old_offset_time_ps = CNT_PERIOD_PS / 2;
    skip_next1pps = true;
    need_reset_prefilter = true;
  }
  else
    timeout_1pps ++;

  if (present_1pps)
  {
    set_led(LED_RED, false);

    if (locked_state)
    {
      set_led(LED_GREEN, true); // ON

    }
    else 
//    if (counter_2s == 0)
    if ( counter_2s%50 == 0 )
      toggle_led(LED_GREEN);

  }
  else // Lost 1PPS
  {
    if (counter_2s%100 == 0)
    {
      toggle_led(LED_RED);

      avg_cno = 0;      
    }
    set_led(LED_GREEN, false);

    //    locked_state_changed = true;
    locked_state = false;
    lock_unlock_counter = 0;
  }


}

void init_timers()
{
//  ITimer0.attachInterruptInterval(10000UL, TimerHandler10ms); // 10ms timer
  ITimer0.attachInterruptInterval_MS(10, TimerHandler10ms); // 10ms timer
}

// DAC DAC8501

#define DAC_DATA 9 // MCU PA07
#define DAC_SCLK 8 // MCU PA06
#define DAC_SYNC 7 // MCU PA21

void init_dac()
{
  pinMode(DAC_DATA, OUTPUT);
  pinMode(DAC_SCLK, OUTPUT);
  pinMode(DAC_SYNC, OUTPUT);

  digitalWrite(DAC_SCLK, LOW);
  digitalWrite(DAC_SYNC, HIGH);
}

void set_dac(uint16_t value, bool dac_off)
{
  uint32_t w = value;
  if (dac_off)
    w |= 0b000000110000000000000000UL; // PD1=PD0=1

  digitalWrite(DAC_SYNC, LOW);


  for (int i = 0; i < 24; i++)
  {
    digitalWrite(DAC_DATA, (w & (1 << (23 - i))) ? HIGH : LOW);
    digitalWrite(DAC_SCLK, HIGH);
    digitalWrite(DAC_SCLK, LOW);
  }

  digitalWrite(DAC_SYNC, HIGH);
}

// TDC7200

#define PIN_TDC7200_ENABLE 3 // MCU PA09
#define PIN_TDC7200_SPI_CS 10 // MCU PA18
#define TDC7200_CLOCK_FREQ_HZ OCXO_FREQ_HZ

TDC7200 tof(PIN_TDC7200_ENABLE, PIN_TDC7200_SPI_CS, TDC7200_CLOCK_FREQ_HZ);

#define PIN_TDC7200_INT 4 // MCU PA08
#define NUM_STOPS 1

void init_tdc7200()
{

  for (int i=0; i<10; i++)
  {
    if (tof.begin())
      break;

    SerialUSB.println("Failed to init TDC7200");
    delay(1000);
  }

  pinMode(PIN_TDC7200_INT, INPUT_PULLUP);     // active low (open drain)

  if (not tof.setupMeasurement( 10,         // cal2Periods
                                1,          // avgCycles
                                NUM_STOPS,  // numStops
                                2 ))        // mode
  {
    SerialUSB.println("Failed to setup measurement");
    //    while (1);
  }
}

// Frequency synthesizer

#define NPROFILES 4
uint8_t current_profile_nr = 1;
#define NOUTPUTS 4

#define SI_P1 1   // Assuming P1 divider always be =1, and Input Mux = RefClk, we do not read these values from the registry store 

struct output_fqs_type // Trick to be able to save/load to/from flash all frequencies at once
{
  uint32_t fq[NPROFILES][NOUTPUTS];
};
output_fqs_type output_fqs;


// Include register data for profiles
// 4 profiles, 1..4
// Out A: CLK3, Out B: CLK2, Out C: CLK1, Out D: CLK0

#define const
#define code
#define Reg_Store Reg_Store1
#define Reg_Data Reg_Data1
#include "Si5338-RevB-Registers-1.h"
#undef Reg_Store
#undef Reg_Data
#define Reg_Store Reg_Store2
#define Reg_Data Reg_Data2
#include "Si5338-RevB-Registers-2.h"
#undef Reg_Store
#undef Reg_Data
#define Reg_Store Reg_Store3
#define Reg_Data Reg_Data3
#include "Si5338-RevB-Registers-3.h"
#undef Reg_Store
#undef Reg_Data
#define Reg_Store Reg_Store4
#define Reg_Data Reg_Data4
#include "Si5338-RevB-Registers-4.h"
#undef Reg_Store
//#undef Reg_Data
#undef code
#undef const
#define const const

#define SI_VCO_MIN 2200000000 // Hz
#define SI_VCO_MAX 2840000000 // Hz
#define SI_MS_A_MAX 567 // Max value of a in multisynth
#define SI_RDIV_OUT_MAX 32 // Max value of a Rx output divider

uint8_t si_grade = 0; // A=1, ...Z
int si_max_fq = 200; // Maximum limit in MHz

/*const*/ Reg_Data* reg_stores[NPROFILES] = {(Reg_Data*)Reg_Store1, (Reg_Data*)Reg_Store2, (Reg_Data*)Reg_Store3, (Reg_Data*)Reg_Store4};

// Temporary vars for reading/writing registers
uint32_t msn_p1, msn_p2, msn_p3;
uint32_t ms_p1[4], ms_p2[4], ms_p3[4];
uint8_t r_div[4];
uint8_t r_div_in[4];

void read_reg_from_store(uint16_t reg, uint8_t value)
{
  switch ( reg )
  {
    // R0
    case 31:
      r_div[0] = 1 << ((value >> 2) & 0b111);
      r_div_in[0] = (value >> 5) & 0b111;
      break;
    // R1
    case 32:
      r_div[1] = 1 << ((value >> 2) & 0b111);
      r_div_in[1] = (value >> 5) & 0b111;
      break;
    // R2
    case 33:
      r_div[2] = 1 << ((value >> 2) & 0b111);
      r_div_in[2] = (value >> 5) & 0b111;
      break;
    // R3
    case 34:
      r_div[3] = 1 << ((value >> 2) & 0b111);
      r_div_in[3] = (value >> 5) & 0b111;
      break;

    // MS0

    case 53:
      ms_p1[0] = value;
      break;
    case 54:
      ms_p1[0] |= ((((uint32_t)value) & 0b11111111) << 8);
      break;
    case 55:
      ms_p1[0] |= ((((uint32_t)value) & 0b11) << 16);
      ms_p2[0] = ((((uint32_t)value) & 0b11111100) >> 2);
      break;
    case 56:
      ms_p2[0] |= ((((uint32_t)value) & 0b11111111) << 6);
      break;
    case 57:
      ms_p2[0] |= ((((uint32_t)value) & 0b11111111) << 14);
      break;
    case 58:
      ms_p2[0] |= ((((uint32_t)value) & 0b11111111) << 22); // bug: was 14
      break;
    case 59:
      ms_p3[0] = value;
      break;
    case 60:
      ms_p3[0] |= ((((uint32_t)value) & 0b11111111) << 8);
      break;
    case 61:
      ms_p3[0] |= ((((uint32_t)value) & 0b11111111) << 16);
      break;
    case 62:
      ms_p3[0] |= ((((uint32_t)value) & 0b00111111) << 24);
      break;

    // MS1

    case 64:
      ms_p1[1] = value;
      break;
    case 65:
      ms_p1[1] |= ((((uint32_t)value) & 0b11111111) << 8);
      break;
    case 66:
      ms_p1[1] |= ((((uint32_t)value) & 0b11) << 16);
      ms_p2[1] = ((((uint32_t)value) & 0b11111100) >> 2);
      break;
    case 67:
      ms_p2[1] |= ((((uint32_t)value) & 0b11111111) << 6);
      break;
    case 68:
      ms_p2[1] |= ((((uint32_t)value) & 0b11111111) << 14);
      break;
    case 69:
      ms_p2[1] |= ((((uint32_t)value) & 0b11111111) << 22); // bug: was 14 
      break;
    case 70:
      ms_p3[1] = value;
      break;
    case 71:
      ms_p3[1] |= ((((uint32_t)value) & 0b11111111) << 8);
      break;
    case 72:
      ms_p3[1] |= ((((uint32_t)value) & 0b11111111) << 16);
      break;
    case 73:
      ms_p3[1] |= ((((uint32_t)value) & 0b00111111) << 24);
      break;

    // MS2

    case 75:
      ms_p1[2] = value;
      break;
    case 76:
      ms_p1[2] |= ((((uint32_t)value) & 0b11111111) << 8);
      break;
    case 77:
      ms_p1[2] |= ((((uint32_t)value) & 0b11) << 16);
      ms_p2[2] = ((((uint32_t)value) & 0b11111100) >> 2);
      break;
    case 78:
      ms_p2[2] |= ((((uint32_t)value) & 0b11111111) << 6);
      break;
    case 79:
      ms_p2[2] |= ((((uint32_t)value) & 0b11111111) << 14);
      break;
    case 80:
      ms_p2[2] |= ((((uint32_t)value) & 0b11111111) << 22); // bug: was 14
      break;
    case 81:
      ms_p3[2] = value;
      break;
    case 82:
      ms_p3[2] |= ((((uint32_t)value) & 0b11111111) << 8);
      break;
    case 83:
      ms_p3[2] |= ((((uint32_t)value) & 0b11111111) << 16);
      break;
    case 84:
      ms_p3[2] |= ((((uint32_t)value) & 0b00111111) << 24);
      break;

    // MS3

    case 86:
      ms_p1[3] = value;
      break;
    case 87:
      ms_p1[3] |= ((((uint32_t)value) & 0b11111111) << 8);
      break;
    case 88:
      ms_p1[3] |= ((((uint32_t)value) & 0b11) << 16);
      ms_p2[3] = ((((uint32_t)value) & 0b11111100) >> 2);
      break;
    case 89:
      ms_p2[3] |= ((((uint32_t)value) & 0b11111111) << 6);
      break;
    case 90:
      ms_p2[3] |= ((((uint32_t)value) & 0b11111111) << 14);
      break;
    case 91:
      ms_p2[3] |= ((((uint32_t)value) & 0b11111111) << 22); // bug: was 14
      break;
    case 92:
      ms_p3[3] = value;
      break;
    case 93:
      ms_p3[3] |= ((((uint32_t)value) & 0b11111111) << 8);
      break;
    case 94:
      ms_p3[3] |= ((((uint32_t)value) & 0b11111111) << 16);
      break;
    case 95:
      ms_p3[3] |= ((((uint32_t)value) & 0b00111111) << 24);
      break;



    // MSN
    case 97:
      msn_p1 = value;
      break;
    case 98:
      msn_p1 |= ((((uint32_t)value) & 0b11111111) << 8);
      break;
    case 99:
      msn_p1 |= ((((uint32_t)value) & 0b11) << 16);
      msn_p2 = ((((uint32_t)value) & 0b11111100) >> 2);
      break;
    case 100:
      msn_p2 |= ((((uint32_t)value) & 0b11111111) << 6);
      break;
    case 101:
      msn_p2 |= ((((uint32_t)value) & 0b11111111) << 14);
      break;
    case 102:
      msn_p2 |= ((((uint32_t)value) & 0b11111111) << 22);
      break;
    case 103:
      msn_p3 = value;
      break;
    case 104:
      msn_p3 |= ((((uint32_t)value) & 0b11111111) << 8);
      break;
    case 105:
      msn_p3 |= ((((uint32_t)value) & 0b11111111) << 16);
      break;
    case 106:
      msn_p3 |= ((((uint32_t)value) & 0b00111111) << 24);
      break;

  }
}


void read_all_regs_from_store(struct Reg_Data* reg_store)
{
  // Clear registers

  msn_p1 = 0, msn_p2 = 0, msn_p3 = 0;
  ms_p1[0] = 0; ms_p1[1] = 0; ms_p1[2] = 0; ms_p1[3] = 0;
  ms_p2[0] = 0; ms_p2[1] = 0; ms_p2[2] = 0; ms_p2[3] = 0;
  ms_p3[0] = 0; ms_p3[1] = 0; ms_p3[2] = 0; ms_p3[3] = 0;
  r_div[0] = 0; r_div[1] = 0; r_div[2] = 0; r_div[3] = 0;
  r_div_in[0] = 0b110; r_div_in[1] = 0b110; r_div_in[2] = 0b110; r_div_in[3] = 0b110; 


  uint16_t counter;

  uint16_t page = 0;

  for (counter = 0; counter < NUM_REGS_MAX; counter++)
  {
    struct Reg_Data curr = reg_store[counter];

    if (curr.Reg_Mask == 0xFF)
    {
      if (curr.Reg_Addr == 255) // Switch page
        page = curr.Reg_Val;

      // Collect data
      read_reg_from_store(curr.Reg_Addr | (page << 8), curr.Reg_Val);
    }
  }
}


void update_all_fqs_from_all_stores()
{
  for (int i = 0; i < NPROFILES; i++)
  {
    read_all_regs_from_store(reg_stores[i]);

    // Ideas:
    // MSn or MSx = a+b/c = (MSx_P1 + 512 + MSx_P2 / MSx_P3) / 128
    // SI5338-RM.pdf, page 9
    // To check, see https://github.com/Elphel/linux-elphel/blob/master/src/drivers/clk/clk-si5338.c 
    // static int p123_to_ms(u64* ms,u32 * p123)
    // static int get_pll_freq(struct i2c_client *client,u64 * pll_freq)
    // static int set_pll_freq(struct i2c_client *client, u64 *vco_freq, int int_div)
    // static int set_out_div_by_frequency(struct i2c_client *client, u64* out_freq, int chn) /*chn =0..3 */

    uint64_t an,bn,cn;

    cn = msn_p3;
    bn = (cn*(msn_p1&0x7F) + msn_p2) >> 7;
    an = (msn_p1>>7)+4;

//    double msN = a+(double)b/c;
//    double msN = (msn_p1 + 512 + ((double)msn_p2) / msn_p3) / 128.0; 
    
//    double fVco = OCXO_FREQ_HZ * msN * SI_P1; 

    uint64_t fVco_mul_cn = OCXO_FREQ_HZ*SI_P1*(an*cn+bn); // fVco*cn 

//    SerialUSB.print("Profile ");
//    SerialUSB.println(i+1);
//    SerialUSB.print("msn_p1="); SerialUSB.print(msn_p1); SerialUSB.println(); 
//    SerialUSB.print("msn_p2="); SerialUSB.print(msn_p2); SerialUSB.println(); 
//    SerialUSB.print("msn_p3="); SerialUSB.print(msn_p3); SerialUSB.println(); 
////    SerialUSB.print("msN="); SerialUSB.print(msN); SerialUSB.print(", fVco="); SerialUSB.println(fVco);
//    SerialUSB.print("fVco_cn="); SerialUSB.println(fVco_cn);

    for (int j=0; j<NOUTPUTS; j++)
    {
      uint64_t ax,bx,cx;

      cx = ms_p3[j];
      bx = (cx*(ms_p1[j]&0x7F) + ms_p2[j]) >> 7;
      ax = (ms_p1[j]>>7)+4;
      
//      double msX = a+(double)b/c;
//      double msX = (ms_p1[j] + 512 + ((double)ms_p2[j]) / ms_p3[j]) / 128.0; 


      // Calculate the output frequency

      if (r_div_in[j] == 0b110) // Using MSj
        output_fqs.fq[i][j] =  fVco_mul_cn * cx /(cx*ax+bx)/ cn / r_div[j];
      else
      if (r_div_in[j] == 0b001) // Using refck
        output_fqs.fq[i][j] =  OCXO_FREQ_HZ / r_div[j];
      else
        output_fqs.fq[i][j] = 0; // We do not handle other configurations

      
////      output_fqs.fq[i][j] = fVco / msX / r_div[j];
//      SerialUSB.print(j); 
//      SerialUSB.print(", r_div_in="); SerialUSB.println(r_div_in[j]); 
//      SerialUSB.print(": MSx_P1="); SerialUSB.print(ms_p1[j]); SerialUSB.print(", MSx_P2="); SerialUSB.print(ms_p2[j]); SerialUSB.print(", MSx_P3="); SerialUSB.print(ms_p3[j]);
////      SerialUSB.print(", MSx="); SerialUSB.print(msX); 
//      SerialUSB.print(", rdivX="); SerialUSB.print(r_div[j]); SerialUSB.print(", outX="); SerialUSB.println(output_fqs.fq[i][j]);   
    }
  }
}

#define LOS_MASK 0x04
#define LOCK_MASK 0x15

#if DEVICE_REVISION>=10
  #define  SI_ADDR      0x70
#else  
  #define  SI_ADDR      0x71
#endif
  
#define SYNTH_SDA 11 // PA16
#define SYNTH_SCL 13 // PA17


TwoWire synthWire(&sercom1, SYNTH_SDA, SYNTH_SCL); // PA16, PA17

void synth_byte_write(uint8_t addr, uint8_t data)
{
  synthWire.beginTransmission(SI_ADDR);
  synthWire.write(addr);
  synthWire.write(data);
  synthWire.endTransmission();
}

uint8_t synth_byte_read(uint8_t addr)
{
  synthWire.beginTransmission(SI_ADDR);
  synthWire.write(addr);
  synthWire.endTransmission();
  synthWire.requestFrom(SI_ADDR, 1);

  uint8_t data = synthWire.read();
  return data;
}

void init_synth()
{
#if DEVICE_REVISION<10
  return;
#endif

  // Init I2C for the synth
  // https://learn.sparkfun.com/tutorials/adding-more-sercom-ports-for-samd-boards/adding-an-i2c

  synthWire.begin();
  synthWire.setClock(400000); // Fast mode
  pinPeripheral (SYNTH_SDA, PIO_SERCOM); // Peripheral C 
  pinPeripheral (SYNTH_SCL, PIO_SERCOM); // Peripheral C


  // Get the SI5338 grade
  si_grade = synth_byte_read(3) >> 3;

  // Try to guess the max frequency, see SI5338.pdf, page 37

  char grade_letter = si_grade-1+'A';

  switch (grade_letter)
  {
    case 0: 
      si_max_fq = 0; // Bug
      break;

    case 'A':
    case 'D':
    case 'G':
    case 'K':
    case 'N':
      si_max_fq = 710; 
      break;
      
    case 'B':
    case 'E':
    case 'H':
    case 'L':
    case 'P':
      si_max_fq = 350; 
      break;
      
    case 'C':
    case 'F':
    case 'J':
    case 'M':
    case 'Q':
      si_max_fq = 350; 
      break;

    default:
      si_max_fq = 0; // bug 
      break;
  }
  

/*
  // Init output frequencies

  for (int i = 1; i <= NPROFILES; i++)
  {
    output_fqs.fq[i - 1][1 - 1] = 24000000;
    output_fqs.fq[i - 1][2 - 1] = 25000000;
    output_fqs.fq[i - 1][3 - 1] = 28800000;
    output_fqs.fq[i - 1][4 - 1] = 40000000;

    //    for (int j=1; j<=NOUTPUTS; j++)
    //      output_fqs.fq[i-1][j-1] = 10000000;
  }
*/  
}


void program_synth_from_store(struct Reg_Data* reg_store)
{

//  SerialUSB.println("program_synth_from_store -0");


  synth_byte_write(230, 0x10);                   //OEB_ALL = 1
  synth_byte_write(241, 0xE5);                   //DIS_LOL = 1

  uint16_t counter;
  uint8_t curr_chip_val, clear_curr_val, clear_new_val, combined, reg;

//  SerialUSB.println("1");

  uint16_t page = 0;

  for (counter = 0; counter < NUM_REGS_MAX; counter++)
  {
    struct Reg_Data curr = reg_store[counter];

    /*
           Serial.print(curr.Reg_Addr);
           Serial.print(' ');
           Serial.print(curr.Reg_Val);
           Serial.print(' ');
           Serial.println(curr.Reg_Mask);
    */
    if (curr.Reg_Mask != 0x00)
    {
      if (curr.Reg_Mask == 0xFF)
      {
        // do a write transaction only
        // since the mask is all ones
        synth_byte_write(curr.Reg_Addr, curr.Reg_Val);

        if (curr.Reg_Addr == 255) // Switch page
          page = curr.Reg_Val;
      }
      else
      {
        curr_chip_val = synth_byte_read(curr.Reg_Addr);
        clear_curr_val = curr_chip_val & ~curr.Reg_Mask;
        clear_new_val = curr.Reg_Val & curr.Reg_Mask;
        combined = clear_new_val | clear_curr_val;
        synth_byte_write(curr.Reg_Addr, combined);
      }
    }
  }

  //  Serial.println(counter);
//  SerialUSB.println("2");

  // check LOS alarm for the xtal input
  // on IN1 and IN2 (and IN3 if necessary) -
  // change this mask if using inputs on IN4, IN5, IN6
  delay(1);
  reg = synth_byte_read(218) & LOS_MASK;
  int los_timeout = 100; // ms
  while ( (reg != 0) && (los_timeout)) {
    delay(1);
    reg = synth_byte_read(218) & LOS_MASK;
    los_timeout--;
  }
  
  synth_byte_write(49, synth_byte_read(49) & 0x7F); //FCAL_OVRD_EN = 0
  synth_byte_write(246, 2);                      //soft reset
  synth_byte_write(241, 0x65);                   //DIS_LOL = 0

//  SerialUSB.println("3");

  // wait for Si5338 to be ready after calibration (ie, soft reset)

  delay(10);

  //make sure the device locked by checking PLL_LOL and SYS_CAL
  reg = synth_byte_read(218) & LOCK_MASK;
  while (reg != 0) {
    reg = synth_byte_read(218) & LOCK_MASK;
  }
//  SerialUSB.println("4");

  //copy FCAL values
  synth_byte_write(45, synth_byte_read(235));
  synth_byte_write(46, synth_byte_read(236));
  // clear bits 0 and 1 from 47 and
  // combine with bits 0 and 1 from 237
  reg = (synth_byte_read(47) & 0xFC) | (synth_byte_read(237) & 3);
  synth_byte_write(47, reg);
  synth_byte_write(49, synth_byte_read(49) | 0x80); // FCAL_OVRD_EN = 1
  synth_byte_write(230, 0x00);                   // OEB_ALL = 0
  //------------------------------------------------------------

//  SerialUSB.println("5");
}

bool update_regs_to_store(uint16_t reg, uint8_t& value)
{
  switch ( reg )
  {
    // R0
    case 31:
//      value &= ~0b00011100;
      value &= ~0b11111110;

      value |= (r_div_in[0]<<5); 
      if (r_div_in[0] != 0b110) // Not using MSn
        value |= 0b10; // Disable MSn
    
      switch(r_div[0])
      {
        case 2:
          value |= 0b00000100;
          break;
        case 4:
          value |= 0b00001000;
          break;
        case 8:
          value |= 0b00001100;
          break;
        case 16:
          value |= 0b00010000;
          break;
        case 32:
          value |= 0b00010100;
          break;
      }      
      break;
    // R1
    case 32:
//      value &= ~0b00011100;
      value &= ~0b11111110;

      value |= (r_div_in[1]<<5); 
      if (r_div_in[1] != 0b110) // Not using MSn
        value |= 0b10; // Disable MSn
    
      switch(r_div[1])
      {
        case 2:
          value |= 0b00000100;
          break;
        case 4:
          value |= 0b00001000;
          break;
        case 8:
          value |= 0b00001100;
          break;
        case 16:
          value |= 0b00010000;
          break;
        case 32:
          value |= 0b00010100;
          break;
      }
      break;
    // R2
    case 33:
//      value &= ~0b00011100;
      value &= ~0b11111110;

      value |= (r_div_in[2]<<5); 
      if (r_div_in[2] != 0b110) // Not using MSn
        value |= 0b10; // Disable MSn
    
      switch(r_div[2])
      {
        case 2:
          value |= 0b00000100;
          break;
        case 4:
          value |= 0b00001000;
          break;
        case 8:
          value |= 0b00001100;
          break;
        case 16:
          value |= 0b00010000;
          break;
        case 32:
          value |= 0b00010100;
          break;
      }
      break;
    // R3
    case 34:
//      value &= ~0b00011100;
      value &= ~0b11111110;

      value |= (r_div_in[3]<<5); 
      if (r_div_in[3] != 0b110) // Not using MSn
        value |= 0b10; // Disable MSn
    
      switch(r_div[3])
      {
        case 2:
          value |= 0b00000100;
          break;
        case 4:
          value |= 0b00001000;
          break;
        case 8:
          value |= 0b00001100;
          break;
        case 16:
          value |= 0b00010000;
          break;
        case 32:
          value |= 0b00010100;
          break;
      }
      break;

    // MS0

    case 53:
      value = ms_p1[0];
      break;
    case 54:
      value = ms_p1[0]>>8;
      break;
    case 55:
      value = ((ms_p1[0]>>16) & 0b00000011) | ((ms_p2[0]<<2) & 0b11111100);
      break;
    case 56:
      value = ms_p2[0]>>6;
      break;
    case 57:
      value = ms_p2[0]>>14;
      break;
    case 58:
      value = ms_p2[0]>>22;
      break;
    case 59:
      value = ms_p3[0];
      break;
    case 60:
      value = ms_p3[0]>>8;
      break;
    case 61:
      value = ms_p3[0]>>16;
      break;
    case 62:
      value = (ms_p3[0]>>24) & 0b00111111;
      break;

    // MS1

    case 64:
      value = ms_p1[1];
      break;
    case 65:
      value = ms_p1[1]>>8;
      break;
    case 66:
      value = ((ms_p1[1]>>16) & 0b00000011) | ((ms_p2[1]<<2) & 0b11111100);
      break;
    case 67:
      value = ms_p2[1]>>6;
      break;
    case 68:
      value = ms_p2[1]>>14;
      break;
    case 69:
      value = ms_p2[1]>>22;
      break;
    case 70:
      value = ms_p3[1];
      break;
    case 71:
      value = ms_p3[1]>>8;
      break;
    case 72:
      value = ms_p3[1]>>16;
      break;
    case 73:
      value = (ms_p3[1]>>24) & 0b00111111;
      break;

    // MS2

    case 75:
      value = ms_p1[2];
      break;
    case 76:
      value = ms_p1[2]>>8;
      break;
    case 77:
      value = ((ms_p1[2]>>16) & 0b00000011) | ((ms_p2[2]<<2) & 0b11111100);
      break;
    case 78:
      value = ms_p2[2]>>6;
      break;
    case 79:
      value = ms_p2[2]>>14;
      break;
    case 80:
      value = ms_p2[2]>>22;
      break;
    case 81:
      value = ms_p3[2];
      break;
    case 82:
      value = ms_p3[2]>>8;
      break;
    case 83:
      value = ms_p3[2]>>16;
      break;
    case 84:
      value = (ms_p3[2]>>24) & 0b00111111;
      break;

    // MS3

    case 86:
      value = ms_p1[3];
      break;
    case 87:
      value = ms_p1[3]>>8;
      break;
    case 88:
      value = ((ms_p1[3]>>16) & 0b00000011) | ((ms_p2[3]<<2) & 0b11111100);
      break;
    case 89:
      value = ms_p2[3]>>6;
      break;
    case 90:
      value = ms_p2[3]>>14;
      break;
    case 91:
      value = ms_p2[3]>>22;
      break;
    case 92:
      value = ms_p3[3];
      break;
    case 93:
      value = ms_p3[3]>>8;
      break;
    case 94:
      value = ms_p3[3]>>16;
      break;
    case 95:
      value = (ms_p3[3]>>24) & 0b00111111;
      break;



    // MSN
    case 97:
      value = msn_p1;
      break;
    case 98:
      value = msn_p1>>8;
      break;
    case 99:
      value = ((msn_p1>>16) & 0b00000011) | ((msn_p2<<2) & 0b11111100);
      break;
    case 100:
      value = msn_p2>>6;
      break;
    case 101:
      value = msn_p2>>14;    
      break;
    case 102:
      value = msn_p2>>22;    
      break;
    case 103:
      value = msn_p3;    
      break;
    case 104:
      value = msn_p3>>8;    
      break;
    case 105:
      value = msn_p3>>16;    
      break;
    case 106:
      value = (msn_p3>>24) & 0b00111111;
      break;
    default: 
      return false;
  }

  return true;
}



void update_all_regs_to_store(struct Reg_Data* reg_store)
{

  uint16_t counter;

  uint16_t page = 0;

  for (counter = 0; counter < NUM_REGS_MAX; counter++)
  {

    if (reg_store[counter].Reg_Mask == 0xFF)
    {
      if (reg_store[counter].Reg_Addr == 255) // Switch page
        page = reg_store[counter].Reg_Val;

      // Collect data

      uint16_t addr = reg_store[counter].Reg_Addr | (page << 8);
      uint8_t value = reg_store[counter].Reg_Val;

//      uint8_t old_value = value;

      if( update_regs_to_store(addr, value) )
      {
        reg_store[counter].Reg_Val = value;
//        if (old_value != value)        
//        {
//          SerialUSB.print("addr:"); SerialUSB.print(addr);  SerialUSB.print(" old_value:"); SerialUSB.print(old_value);
//
//          SerialUSB.print(" new_value:"); SerialUSB.println(value);
//        }
      }
    }
  }
}

// Greatest common divisor

static uint64_t gcd_uint64(uint64_t a, uint64_t b)
{
//  if (a==0)
//    a = 1;
//  
//  if (b==0)
//    b = 1;

  while (b!=0)
  {
    uint64_t temp = a % b;
    a = b;
    b = temp;
  }

  return a;
}


void update_all_stores_from_all_fqs()
{
  
  for (int i = 0; i < NPROFILES; i++)
  {
    // First, read all registers for the current profile
    
    read_all_regs_from_store(reg_stores[i]);

    // Ideas:
    // MSn or MSx = a+b/c = (MSx_P1 + 512 + MSx_P2 / MSx_P3) / 128
    // SI5338-RM.pdf, page 9
    // To check, see https://github.com/Elphel/linux-elphel/blob/master/src/drivers/clk/clk-si5338.c 
    // static int p123_to_ms(u64* ms,u32 * p123)
    // static int get_pll_freq(struct i2c_client *client,u64 * pll_freq)
    // static int set_pll_freq(struct i2c_client *client, u64 *vco_freq, int int_div)
    // static int set_out_div_by_frequency(struct i2c_client *client, u64* out_freq, int chn) /*chn =0..3 */

    uint64_t an,bn,cn;

    cn = msn_p3;
    bn = (cn*(msn_p1&0x7F) + msn_p2) >> 7;
    an = (msn_p1>>7)+4;


    // We do not change the fVCO frequency, just read it
    uint64_t fVco_mul_cn = OCXO_FREQ_HZ*SI_P1*(an*cn+bn); // fVco*cn 

//    SerialUSB.print("-------------Profile "); SerialUSB.println(i+1);

//    SerialUSB.print("fVco_mul_cn="); SerialUSB.println(fVco_mul_cn);


    for (int j=0; j<NOUTPUTS; j++)
    {

      uint64_t ax,bx,cx;

      int rdiv_out = 1;

      uint64_t fOut_mul_cn_mul_rdiv_out; 


      // Calculate the rdiv
      
      while(1) 
      {
        fOut_mul_cn_mul_rdiv_out = output_fqs.fq[i][j] * cn * rdiv_out; // fOut*cn*rdiv_out 

        ax = fVco_mul_cn / fOut_mul_cn_mul_rdiv_out;

        if (ax <= SI_MS_A_MAX)
          break; // Finish

        if (rdiv_out >= SI_RDIV_OUT_MAX ) 
          break; // Give up

        rdiv_out <<= 1;
      }

      r_div[j] = rdiv_out;
      
      uint64_t remainder_mul_cn = fVco_mul_cn % fOut_mul_cn_mul_rdiv_out;

//      SerialUSB.print(j); SerialUSB.print(": "); 
//      SerialUSB.print("fOut_mul_cn_mul_rdiv_out:"); SerialUSB.println(fOut_mul_cn_mul_rdiv_out); 
//      SerialUSB.print("remainder_mul_cn:"); SerialUSB.println(remainder_mul_cn); 

      if (remainder_mul_cn==0) // Integer MSx, set b=0 and c=1
      {
        bx = 0;
        cx = 1;
      }
      else
      {
        uint64_t gcd = gcd_uint64(remainder_mul_cn, fOut_mul_cn_mul_rdiv_out);

        bx = remainder_mul_cn / gcd;
        cx = fOut_mul_cn_mul_rdiv_out / gcd;
//      SerialUSB.print("gcd:"); SerialUSB.println(gcd); 
      }
      
      // We hope that cn equals 1 (integer MSn) or at least a small number
      // If cn is too large, we might need to use rational_best_approximation() so bx and cx will be less than 2^30-1
      
//      ax = 2047;
//      bx = 0;
//      cx = 1;
//      r_div[j] = 32; 

//      SerialUSB.print("ax:"); SerialUSB.println(ax); 
//      SerialUSB.print("bx:"); SerialUSB.println(bx); 
//      SerialUSB.print("cx:"); SerialUSB.println(cx); 
//      SerialUSB.print("rdiv_out:"); SerialUSB.println(rdiv_out); 

      ms_p1[j] = ( (ax*cx+bx)*128 ) / cx - 512;
      ms_p2[j] = (bx*128) % cx;;
      ms_p3[j] = cx;      

//ms_p1[j] = 0x3FD80UL;
//ms_p1[j] = 0b00111111111011111111UL;
//ms_p1[j] = 0x3FFFFUL;

//      SerialUSB.print("ms_p1:"); SerialUSB.println(ms_p1[j]); 
//      SerialUSB.print("ms_p2:"); SerialUSB.println(ms_p2[j]); 
//      SerialUSB.print("ms_p3:"); SerialUSB.println(ms_p3[j]); 


      // Now check if the desired output frequency is 1/rdiv_out the OCXO frequency (where rdiv_out=1,2,4,8,16,32)
      // In this case, disable MSn and feed the OCXO directly to the input of Rn

      rdiv_out = 1;

      while(1) 
      {
        
        if (rdiv_out > SI_RDIV_OUT_MAX ) 
          break; // Give up

        if ( output_fqs.fq[i][j] == OCXO_FREQ_HZ / rdiv_out )
          break; // Found!
        
        rdiv_out <<= 1;
      }

      if ( rdiv_out <= SI_RDIV_OUT_MAX)
      {
        r_div_in[j] = 0b001; // Using refclk, disable MSn
        r_div[j] = rdiv_out;
      }
      else
        r_div_in[j] = 0b110; // Using MSn, enable MSn



    }

    // Update the registers for the current profile
   
    update_all_regs_to_store(reg_stores[i]);
  }
}


void program_synth()
{
#if DEVICE_REVISION<10
  return;
#endif

  
  // Now update stores from frequemcy vars
  update_all_stores_from_all_fqs();

  // Program the synth from a register store
  struct Reg_Data* reg_store = reg_stores[current_profile_nr - 1];
  program_synth_from_store(reg_store);
}


// Flash memore read/write

void print_saved_vars()
{

  SerialUSB.print("Firmware version: "); SerialUSB.println(FIRMWARE_VERSION);
  if (si_grade > 0 )
  {
    char grade_letter = si_grade-1+'A';
    if (si_grade==0)
      grade_letter = '?';
    SerialUSB.print("SI5338 grade: "); SerialUSB.print( grade_letter );
    SerialUSB.print(", 0.16 to "); SerialUSB.print(si_max_fq); SerialUSB.println(" MHz"); 
  }

  if (dac_off)
    SerialUSB.println("DAC is turned off");
  else
  {
    SerialUSB.print("Current DAC value: "); SerialUSB.println(dac_value);
  }

  if (avg_cno>0)
    SerialUSB.print("Average CNO, dBHz: "); SerialUSB.println(avg_cno);
  
  SerialUSB.print("OCXO tuning range, ppt: "); SerialUSB.println(tuning_range_ppt);
  SerialUSB.print("Prefilter time constant, s: "); SerialUSB.println(prefilter_time_const);
  SerialUSB.print("Loop time constant, s: "); SerialUSB.println(loop_time_const);
  SerialUSB.print("Damping: "); SerialUSB.println(loop_damping_thousands / 1000.0);
  SerialUSB.print("Sawtooth correction: "); SerialUSB.println(apply_qerr_correction);
  SerialUSB.print("Init GPS at startup: "); SerialUSB.println(gps_enable_init);
  SerialUSB.print("Frequency offset, ppt: "); SerialUSB.println(fq_offset_ppt);
  SerialUSB.print("Current profile number: "); SerialUSB.println(current_profile_nr);

  for (int i = 1; i <= NPROFILES; i++)
  {
    SerialUSB.print("Profile "); SerialUSB.print(i); SerialUSB.print(": ");

    for (int j = 0; j < NOUTPUTS; j++)
    {
      SerialUSB.print(char(j+'A'));
      SerialUSB.print(") ");
      SerialUSB.print(output_fqs.fq[i - 1][NOUTPUTS-1-j]);
      SerialUSB.print(" Hz ");
    }
    SerialUSB.println();
  }

  SerialUSB.println("? for help");
}

void print_legend()
{
  SerialUSB.println(
    "raw-ps"
    "\tqErr-ps"
    "\tcorr-ps"
    "\tpre-ps"
    "\t1pps-ps"
    "\tdac_val"
  );
}

FlashStorage(flash_signature, int);
FlashStorage(flash_dac_value, uint16_t);
FlashStorage(flash_loop_time_const, int);
FlashStorage(flash_tuning_range_ppt, int);
FlashStorage(flash_prefilter_time_const, int);
FlashStorage(flash_fq_offset_ppt, int);
FlashStorage(flash_loop_damping_thousands, int);

FlashStorage(flash_current_profile_nr, uint8_t);
FlashStorage(flash_output_fqs, output_fqs_type);

FlashStorage(flash_gps_enable_init, uint8_t);
FlashStorage(flash_gps_comm_mode, uint8_t);
FlashStorage(flash_display_brightness, uint8_t);



void init_flash(boolean need_reset = false)
{
  const int WRITTEN_SIGNATURE = 0xBEEFDEED;
  // Check signature at address 0
  int signature ;

  //  int signature1 = WRITTEN_SIGNATURE;
  //  flash_signature1.write(signature1);
  //  flash_signature1.read(signature1);
  //  SerialUSB.println(signature1);

  //  age_storage.write(15);  // <-- save the age
  //  int user_age;
  //  age_storage.read(user_age);

  flash_signature.read(signature);

  if ( (signature != WRITTEN_SIGNATURE) || need_reset)
  {
    signature = WRITTEN_SIGNATURE;
    if (need_reset)
      signature++; // Spoil the signature
    flash_signature.write(signature);

    flash_dac_value.write(dac_value);
    flash_loop_time_const.write(loop_time_const);
    flash_tuning_range_ppt.write(tuning_range_ppt);
    flash_prefilter_time_const.write(prefilter_time_const);
    flash_fq_offset_ppt.write(fq_offset_ppt);
    flash_loop_damping_thousands.write(loop_damping_thousands);
    flash_gps_enable_init.write(gps_enable_init);
    flash_gps_comm_mode.write(gps_comm_mode);
    flash_display_brightness.write(display_brightness);


    flash_current_profile_nr.write(current_profile_nr);

    
    update_all_fqs_from_all_stores(); // Init frequencies from the register stores
    flash_output_fqs.write(output_fqs);

  }
  else
  {
    flash_dac_value.read(dac_value);
    flash_loop_time_const.read(loop_time_const);
    flash_tuning_range_ppt.read(tuning_range_ppt);
    flash_prefilter_time_const.read(prefilter_time_const);
    flash_fq_offset_ppt.read(fq_offset_ppt);
    flash_loop_damping_thousands.read(loop_damping_thousands);

    flash_current_profile_nr.read(current_profile_nr);
    flash_output_fqs.read(output_fqs);

    flash_gps_enable_init.read(gps_enable_init);
    flash_gps_comm_mode.read(gps_comm_mode);
    flash_display_brightness.read(display_brightness);

    print_saved_vars();
    print_legend();
  }
}

// Buttons

#if DEVICE_REVISION>=10
  #define BUTTON_MENU A4 // MCU PA05
  #define BUTTON_RIGHT A3 // MCU PA04
  #define BUTTON_LEFT A2 // MCU PB09
#else
  #define BUTTON_MENU A1 // MCU PB08
  #define BUTTON_RIGHT A2 // MCU PB09
  #define BUTTON_LEFT A3 // MCU PA04
#endif

void init_buttons()
{
  pinMode(BUTTON_MENU, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);
  pinMode(BUTTON_LEFT, INPUT_PULLUP);
}

enum {BT_CODE_NONE = 0, BT_CODE_MENU, BT_CODE_LEFT, BT_CODE_RIGHT, BT_CODE_MENU_LONG, BT_CODE_LEFT_LONG, BT_CODE_RIGHT_LONG};

volatile uint8_t current_button_code = BT_CODE_NONE;

uint8_t read_button_code()
{
  uint8_t code = current_button_code;
  current_button_code = BT_CODE_NONE; // In fact, there should be a "test-and-set" atomic operation
  return code;
}

volatile uint8_t cur_button_pressed = BT_CODE_NONE;
volatile uint8_t button_timer_pressed = 0;
volatile uint8_t button_timer_released = 0;
#define BT_TIMER_SHORT 3
#define BT_TIMER_LONG 150
#define BT_TIMER_RELEASED 20

void button_timer_10ms() // 10ms timer
{

  // Optimize later (never?)

  if (button_timer_released)
  {
    button_timer_released--;
    return;
  }

  if (digitalRead(BUTTON_MENU) == LOW) // Pressed MENU
  {
    if (cur_button_pressed == BT_CODE_NONE)
    {
      if (button_timer_pressed == 0)
        button_timer_pressed = BT_TIMER_SHORT;
      else
      {
        button_timer_pressed--;

        if (button_timer_pressed == 0)
          cur_button_pressed = BT_CODE_MENU;
      }
    }
    else if (cur_button_pressed == BT_CODE_MENU)
    {
      if (button_timer_pressed == 0)
        button_timer_pressed = BT_TIMER_LONG;
      else
      {
        button_timer_pressed--;

        if (button_timer_pressed == 0)
        {
          cur_button_pressed = BT_CODE_MENU_LONG;
          current_button_code = BT_CODE_MENU_LONG;
        }
      }
    }
  }
  else if (digitalRead(BUTTON_LEFT) == LOW) // Pressed LEFT
  {
    if (cur_button_pressed == BT_CODE_NONE)
    {
      if (button_timer_pressed == 0)
        button_timer_pressed = BT_TIMER_SHORT;
      else
      {
        button_timer_pressed--;

        if (button_timer_pressed == 0)
          cur_button_pressed = BT_CODE_LEFT;
      }
    }
    else if (cur_button_pressed == BT_CODE_LEFT)
    {
      if (button_timer_pressed == 0)
        button_timer_pressed = BT_TIMER_LONG;
      else
      {
        button_timer_pressed--;

        if (button_timer_pressed == 0)
        {
          cur_button_pressed = BT_CODE_LEFT_LONG;
          current_button_code = BT_CODE_LEFT_LONG;
        }
      }
    }
  }
  else if (digitalRead(BUTTON_RIGHT) == LOW) // Pressed RIGHT
  {
    if (cur_button_pressed == BT_CODE_NONE)
    {
      if (button_timer_pressed == 0)
        button_timer_pressed = BT_TIMER_SHORT;
      else
      {
        button_timer_pressed--;

        if (button_timer_pressed == 0)
          cur_button_pressed = BT_CODE_RIGHT;
      }
    }
    else if (cur_button_pressed == BT_CODE_RIGHT)
    {
      if (button_timer_pressed == 0)
        button_timer_pressed = BT_TIMER_LONG;
      else
      {
        button_timer_pressed--;

        if (button_timer_pressed == 0)
        {
          cur_button_pressed = BT_CODE_RIGHT_LONG;
          current_button_code = BT_CODE_RIGHT_LONG;
        }
      }
    }
  }
  else // Released
  {
    button_timer_pressed = 0;
    if ( cur_button_pressed != BT_CODE_NONE )
    {
      if ( (cur_button_pressed == BT_CODE_MENU)
           || (cur_button_pressed == BT_CODE_LEFT)
           || (cur_button_pressed == BT_CODE_RIGHT)
         )
        //if(current_button_code==BT_CODE_NONE)
        current_button_code = cur_button_pressed; // Once the button if released, do not output long codes
      cur_button_pressed = BT_CODE_NONE;
      button_timer_released = BT_TIMER_RELEASED;
    }
  }

}

// Setup

void setup()
{

//  Serial.end(); // Release the EDBG pins

  init_leds();
  init_buttons();

  init_serial_usb();

  init_display();

  // Enter factory test mode, if needed
  if ( (digitalRead(BUTTON_MENU) == LOW)
      &&  (digitalRead(BUTTON_LEFT) == LOW)
      &&  (digitalRead(BUTTON_RIGHT) == LOW)      
      )
    test_screen();

  init_ext_interrupts();

  init_timers();

  init_dac();

  init_tdc7200();

  init_flash();

  // Adjust display brightness
  display.dim(display_brightness ? false : true);

  if (gps_enable_init)
    init_gps();
  else
    Serial1.begin(GPS_BAUDRATE_OPERATING);

  init_synth();

  // Do additional init

  set_dac(dac_value, dac_off);

  program_synth();

}

// Helper functions

void invoke_hold()
{
  hold_mode = true;
  //      locked_state_changed = true;
  locked_state = false;
}


void apply_new_dac_value(uint16_t dac)
{
  hold_mode = true;
  //      locked_state_changed = true;

  locked_state = false;
  set_dac(dac, dac_off);
  dac_value = dac;
}

void invoke_run()
{
  hold_mode = false;
  //      tic_prefiltered_ps = 0; // Reset filter
  need_reset_prefilter = true;
  coarse_time_ps = 0;
  //      locked_state_changed = true;
  locked_state = false;
}

void invoke_restart()
{
  NVIC_SystemReset();
}

void invoke_bootloader()
{
  display.clearDisplay();
  display.setTextSize(1, 2);
  display.setCursor(0, SCREEN_HEIGHT - 1 - 1);
  display.print("Boot...");
  display.display();

  // https://github.com/microsoft/uf2-samdx1/issues/41
  // https://forums.adafruit.com/viewtopic.php?f=57&t=135094
  *((volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4)) = 0x07738135; // Magic Word, was: 0xf01669ef;
  //*(uint32_t *)(0x20000000 + 32768 -4) = 0xf01669ef;
  NVIC_SystemReset();
}

void invoke_factory_reset()
{
  display.clearDisplay();
  display.setTextSize(1, 2);
  display.setCursor(0, SCREEN_HEIGHT - 1 - 1);
  display.print("Resetting");
  display.display();

  init_flash(true); // Rewrite flash
  NVIC_SystemReset();
}

// Command interpreter

void cmd_interpret(const char* cmd, int cmd_len)
{
  //  SerialUSB.println(cmd);

  // HELP
  if ( ((cmd_len == 1) && (!strcmp(cmd, "?"))) ||
       ((cmd_len == 4) && (!strcmp(cmd, "help"))) )
  {
    SerialUSB.println("-----------------------------");

    SerialUSB.println("hold - stop PLL");
    SerialUSB.println("run - run PLL");
    SerialUSB.println("dac<N> - set DAC output to value N=0...65535 in hold mode");
    SerialUSB.println("dacoff - turn the DAC off; dac<N>, hold or run to resume");
    SerialUSB.println("savedac - save current DAC value");
    SerialUSB.println("readdac - read saved DAC");
    SerialUSB.println("trans - enter GPS transparent mode, +++ to exit");
    SerialUSB.println("pre<N> - set prefilter time constant to N seconds, or 0 to disable prefilter");
    SerialUSB.println("loop<N> - set loop time constant to N seconds");
    SerialUSB.println("damping<N> - set loop damping to N (0.5 to 10)");
    SerialUSB.println("range<N> - set OCXO tuning range (in ppt) to N");
    SerialUSB.println("saw<N> - set sawtooth correction to 1 (on) or 0 (off)");
    SerialUSB.println("initgps<N> - init GPS at startup: 1 (on) or 0 (off), initgps to re-init");
    SerialUSB.println("offset<N> - set frequency offset (in ppt) - handle with care!");
    SerialUSB.println("ss - take a screenshot, ss<char> to use <char> instead of bricks");
    SerialUSB.println("verb<N> - set verbose level to 0 (min) or 1 (max)");
    SerialUSB.println("legend or <CR> - print legend");
    SerialUSB.println("restart - restart the device");
    SerialUSB.println("boot - jump to bootloader");
    SerialUSB.println("-----------------------------");

    print_saved_vars();
    SerialUSB.println("-----------------------------");
    print_legend();
    SerialUSB.println("-----------------------------");

//SerialUSB.println("grade:");
//SerialUSB.println(si_grade); 

//    // !!!
//
//    {
//      uint64_t fin = 10000000;
//      uint32_t an = 240, bn=0, cn=1;
//
//      // fvco = fin * (an+bn/cn)
//
//      uint64_t fvco = fin * (an+bn/cn);
//      SerialUSB.print("fvco="); SerialUSB.print(fvco); SerialUSB.println();
//      
//      uint32_t fout = 28800000;
//
//      uint32_t ax = fvco/fout;
//      SerialUSB.print("ax="); SerialUSB.print(ax); SerialUSB.println();
//      
//      uint32_t rem = fvco - ((uint64_t)ax)*fout;
//      SerialUSB.print("rem="); SerialUSB.print(rem); SerialUSB.println();
//
//
//      uint32_t gd = gcd_uint32(rem, fout);
//      SerialUSB.print("gcd="); SerialUSB.print(gd); SerialUSB.println();
//     
//      SerialUSB.print("b="); SerialUSB.print(rem/gd); SerialUSB.println();
//      SerialUSB.print("c="); SerialUSB.print(fout/gd); SerialUSB.println();
//    }
//    
    

//    update_all_fqs_from_all_stores();    
//      update_all_stores_from_all_fqs();
  }

  else

  // SS - screenshot
  if ( (cmd_len >= 2) && (!strncmp(cmd, "ss", 2)))
  {

    char cs = 0;

    if (cmd_len > 2)
    {
      cs = cmd[2];
    }

#define SS_BORDERX 4
#define SS_SPACESX 4
#define SS_BORDERY 4
#define SS_CHAR_FRAME "\u2588"
#define SS_CHAR1 "\u2588"
#define SS_CHAR0 ' '

    SerialUSB.println("Screenshot start\n");

    for (int y = 0 - SS_BORDERY; y < SCREEN_HEIGHT + SS_BORDERY; y++)
    {
      for (int z = 0; z < SS_SPACESX; z++)
        SerialUSB.print(SS_CHAR0);

      for (int x = 0 - SS_BORDERX; x < SCREEN_WIDTH + SS_BORDERX; x++)
      {
        if ( (y == 0 - SS_BORDERY) || (y == SCREEN_HEIGHT + SS_BORDERY - 1)
             || (x == 0 - SS_BORDERX) || (x == SCREEN_WIDTH + SS_BORDERX - 1) )
        {
          if (cs)
            SerialUSB.print(cs);
          else
            SerialUSB.print(SS_CHAR_FRAME);
        }
        else if ( (y >= 0) && (y < SCREEN_HEIGHT) && (x >= 0) && (x < SCREEN_WIDTH)  )
        {
          if (display.getPixel(x, y))
          {
            if (cs)
              SerialUSB.print(cs);
            else
              SerialUSB.print(SS_CHAR1);
          }
          else
            SerialUSB.print(SS_CHAR0);
        }
        else
        {
          SerialUSB.print(SS_CHAR0);
        }
      }
      SerialUSB.println();
    }

    SerialUSB.println("\nScreenshot end");
  }

  else

  // RESTART
  if ( (cmd_len == 7 ) && (!strncmp(cmd, "restart", 7)))
  {
    invoke_restart();
  }

  else

  // BOOT
  if ( (cmd_len == 4 ) && (!strncmp(cmd, "boot", 4)))
  {
    invoke_bootloader();
  }

  else

  // LEGEND
  if ( (cmd_len >= 3 ) && (!strncmp(cmd, "leg", 3)))
  {
    print_legend();
  }

  else
  // <CR>
  if ( cmd_len == 0 ) 
  {
    print_legend();
  }

  else

  // HOLD
  if ( (cmd_len == 4) && (!strcmp(cmd, "hold")))
  {
    //    hold_mode = (atoi(cmd+5)==1);
    //    if (hold_mode)

    dac_off = false;
    invoke_hold();

    if (verbose_level)
      SerialUSB.println("Hold");
    //    else
    //      SerialUSB.println("Unhold");
  }

  else

  // DACOFF
  if ( (cmd_len == 6) && (!strcmp(cmd, "dacoff")))
  {
    //    hold_mode = (atoi(cmd+5)==1);
    //    if (hold_mode)

    dac_off = true;
    invoke_hold();
    set_dac(dac_value, dac_off);

    if (verbose_level)
      SerialUSB.println("DAC off");
  }

  else

  // RUN
  if ( (cmd_len == 3) && (!strcmp(cmd, "run")))
  {
    dac_off = false;
    invoke_run();
    if (verbose_level)
      SerialUSB.println("Run");
  }

  else

  // VERB
  if ( (cmd_len >= 5) && (!strncmp(cmd, "verb", 4)))
  {
    verbose_level = atoi(cmd + 4);

    if (verbose_level)
    {
      SerialUSB.println("Set verbose level to 1");
    }
  }

  else

  // BRIGHT
  if ( (cmd_len >= 7) && (!strncmp(cmd, "bright", 6)))
  {
    display_brightness = atoi(cmd + 6) ? 1 : 0;
    flash_display_brightness.write(display_brightness);
    display.dim(display_brightness ? false : true);
    SerialUSB.print("Set brightness to ");
    SerialUSB.println(display_brightness);
  }

  else

  // DAC
  if ( (cmd_len >= 4) && (!strncmp(cmd, "dac", 3)))
  {
    uint16_t dac = atoi(cmd + 3);
    //      if (hold_mode)

    dac_off = false;
    apply_new_dac_value(dac);

    if (verbose_level)
    {
      SerialUSB.print("Set DAC to ");
      SerialUSB.println(dac);
    }

    //      else
    //        SerialUSB.println("Set hold mode first!");
  }

  else

  // SAVEDAC
  if ( (cmd_len >= 7) && (!strcmp(cmd, "savedac")))
  {
    if (verbose_level)
    {
      SerialUSB.print("Save DAC value ");
      SerialUSB.println(dac_value);
    }
    flash_dac_value.write(dac_value);

  }

  else

  // READDAC
  if ( (cmd_len >= 7) && (!strcmp(cmd, "readdac")))
  {
    hold_mode = true;
    //      locked_state_changed = true;
    locked_state = false;
    flash_dac_value.read(dac_value);

    set_dac(dac_value, dac_off);
    if (verbose_level)
    {
      SerialUSB.print("Read DAC value: ");
      SerialUSB.println(dac_value);
    }
  }

  else

  // TRANS
  if ( (cmd_len >= 5) && (!strcmp(cmd, "trans")))
  {
    gps_comm_mode = true;
    flash_gps_comm_mode.write(gps_comm_mode);
  }

  else

  // LOOP
  if ( (cmd_len >= 5) && (!strncmp(cmd, "loop", 4)))
  {
    //      tic_prefiltered_ps /= time_const; // Divide by old
//                              loop_i /= loop_time_const;   
    loop_time_const = atoi(cmd + 4);

    //      tic_prefiltered_ps *= time_const; // Multiply by new
    
//                            loop_i *= loop_time_const;

    if (verbose_level)
    {
      SerialUSB.print("Set loop time_const to ");
      SerialUSB.print(loop_time_const);
      SerialUSB.println(" s");
    }

    flash_loop_time_const.write(loop_time_const);

  }
  else

  // DAMPING
  if ( (cmd_len >= 8) && (!strncmp(cmd, "damping", 7)))
  {
    loop_damping_thousands = atof(cmd + 7) * 1000.0;

    if (verbose_level)
    {
      SerialUSB.print("Set loop damping to ");
      SerialUSB.println(loop_damping_thousands / 1000.0);
    }

    flash_loop_damping_thousands.write(loop_damping_thousands);

  }
  else

  // PRE
  if ( (cmd_len >= 4) && (!strncmp(cmd, "pre", 3)))
  {
    if (prefilter_time_const == 0)
      need_reset_prefilter = true;

    prefilter_time_const = atoi(cmd + 3);


    if (verbose_level)
    {
      SerialUSB.print("Set prefilter time_const to ");
      SerialUSB.print(prefilter_time_const);
      SerialUSB.println(" s");
    }

    flash_prefilter_time_const.write(prefilter_time_const);

  }

  else

  // OFFSET
  if ( (cmd_len >= 7) && (!strncmp(cmd, "offset", 6)))
  {

    fq_offset_ppt = atoi(cmd + 6);

    if (verbose_level)
    {
      SerialUSB.print("Set frequency offset to ");
      SerialUSB.print(fq_offset_ppt);
      SerialUSB.println(" ppt");
    }

    flash_fq_offset_ppt.write(fq_offset_ppt);

  }

  else

  // RANGE

  if ( (cmd_len >= 6) && (!strncmp(cmd, "range", 5)))
  {
    tuning_range_ppt = atoi(cmd + 5);
    if (verbose_level)
    {
      SerialUSB.print("Set OCXO tuning range to ");
      SerialUSB.print(tuning_range_ppt);
      SerialUSB.println(" ppt");
    }

    flash_tuning_range_ppt.write(tuning_range_ppt);

  }


  else

  // SAW

  if ( (cmd_len >= 4) && (!strncmp(cmd, "saw", 3)))
  {
    int param = atoi(cmd + 3);
    apply_qerr_correction = param % 3; // 0,1 or 2
    qerr_ps = 0;
    if (verbose_level)
    {
      SerialUSB.print("Set sawtooth correction to ");
      SerialUSB.println(apply_qerr_correction);
    }
  }

  else

// INITGPS

  if ( (cmd_len >= 7) && (!strncmp(cmd, "initgps", 7)))
  {
    if (cmd_len == 7)
      init_gps();
    else
    {
      int param = atoi(cmd + 7);

      gps_enable_init = param ? 1 : 0;
      flash_gps_enable_init.write(gps_enable_init);

      if (verbose_level)
      {
        SerialUSB.print("Set GPS init at startup to ");
        SerialUSB.println(gps_enable_init);
      }
    }
  }

}

#define MAX_COMMAND_LEN 100
char char_cmd[MAX_COMMAND_LEN];
int char_cmd_len = 0;

void char_interpret()
{
// While???

  if (SerialUSB.available())
  {
    char c = SerialUSB.read();
    if ( (c == '\n') || (c == '\r') )
    {
      char_cmd[char_cmd_len] = 0;

      // Interpret

      cmd_interpret(char_cmd, char_cmd_len);

      char_cmd_len = 0;
    }
    else
    {
      if (char_cmd_len < MAX_COMMAND_LEN)
      {
        char_cmd[char_cmd_len] = c;
        char_cmd_len++;
      }
    }
  }
}

void gps_transparent_exchange()
{
#define LOOPBACK_BUFSIZE 100

  char loopback_buf[LOOPBACK_BUFSIZE];
  size_t len = 0;

  if (len = SerialUSB.available())
    len = SerialUSB.readBytes(loopback_buf, min(len, LOOPBACK_BUFSIZE));
  if (len)
    Serial1.write(loopback_buf, len);
  for (int i = 0; i < len; i++)
  {
    if (loopback_buf[i] == '+')
      transparent_count++;
    else
      transparent_count = 0;

    if (transparent_count >= 3)
    {
      transparent_count = 0;
      gps_comm_mode = false;
      flash_gps_comm_mode.write(gps_comm_mode);
    }
  }

  while (len = Serial1.available())
  {
    len = Serial1.readBytes(loopback_buf, min(len, LOOPBACK_BUFSIZE));
    SerialUSB.write(loopback_buf, len);
    
    for (size_t i=0; i<len; i++)
      receive_gps_byte(loopback_buf[i]);    
  }
}

// Output data to the PC

void output_data_to_pc()
{
  int cont_time_raw_ps = cont_time_ps + qerr_ps;

  SerialUSB.print(cont_time_raw_ps); SerialUSB.print("\t");
  SerialUSB.print(qerr_ps); SerialUSB.print("\t");
  SerialUSB.print(cont_time_ps); SerialUSB.print("\t");
  SerialUSB.print(int(tic_prefiltered_ps)); SerialUSB.print("\t");
  SerialUSB.print(cont_time_ps - cont_time_old_ps); SerialUSB.print("\t");
//  double filter_gain = 65536.0 / tuning_range_ppt;
//  SerialUSB.print(loop_i*filter_gain); SerialUSB.print("\t");
  SerialUSB.print(dac_value);
  SerialUSB.println();
}

// Menu and display


enum {DM_STATUS = 0, DM_PROFILE,
      DM_OUT_A, DM_OUT_B, DM_OUT_C, DM_OUT_D,
      DM_PREFS, DM_PREFS_RANGE, 
      DM_PREFS_DAC_MIN, DM_PREFS_DAC_CEN, DM_PREFS_DAC_MAX, DM_PREFS_DAC_OFF,
      DM_PREFS_PREFILTER, DM_PREFS_LOOP, DM_PREFS_DAMPING,
      DM_PREFS_HOLD, DM_PREFS_RUN,
      DM_PREFS_INITGPS, DM_PREFS_COMM_MODE,
      DM_PREFS_BRIGHT,
      DM_PREFS_BOOT, DM_PREFS_RESALL, DM_PREFS_GRADE, DM_PREFS_VERSION
     };

enum {DM_STATUS_LOCKSTATUS = 0, DM_STATUS_LOCKTIME, DM_STATUS_DT, DM_STATUS_AVG_CNO, DM_STATUS_DAC};
enum {DM_SUBMODE_DISPLAY = 0, DM_SUBMODE_EDIT};


int display_mode = DM_STATUS;
int display_submode = 0;

#define DIGITS_NR 9
int8_t display_digits[DIGITS_NR];
int edit_digit_pos = 0;

// Process buttons

void process_buttons(uint8_t bc)
{

// Menu long --------------------------------------------------------------------

  if ( bc == BT_CODE_MENU_LONG )
  {
    bc = BT_CODE_MENU; // Replace with short press, if not used
  }

  else

// Left or Right long --------------------------------------------------------------------

  if ( (bc == BT_CODE_LEFT_LONG) || (bc == BT_CODE_RIGHT_LONG) )
  {
    switch (display_mode)
    {
      case DM_PROFILE:
      {
        if (bc == BT_CODE_LEFT_LONG) // Left
        {
          current_profile_nr--;
          if (current_profile_nr <= 0)
            current_profile_nr = NPROFILES;
        }
        else // Right
        {
          current_profile_nr++;
          if (current_profile_nr > NPROFILES)
            current_profile_nr = 1;
        }
        program_synth();
        flash_current_profile_nr.write(current_profile_nr);

        break;
      }

    case DM_OUT_A:
    case DM_OUT_B:
    case DM_OUT_C:
    case DM_OUT_D:
    {
      if (display_submode == DM_SUBMODE_EDIT)
      {
        if (bc == BT_CODE_LEFT_LONG) // Left
        {
          display_digits[edit_digit_pos]--;
          if (display_digits[edit_digit_pos] < 0)
            display_digits[edit_digit_pos] = 9;
        }
        else // Right
        {
          display_digits[edit_digit_pos]++;
          if (display_digits[edit_digit_pos] > 9)
            display_digits[edit_digit_pos] = 0;
        }
        
        int output_letter = display_mode - DM_OUT_A; // 0...3
        output_fqs.fq[current_profile_nr - 1][3-output_letter] = 0;

        for (int i = 0; i < DIGITS_NR; i++)
        {
          output_fqs.fq[current_profile_nr - 1][3-output_letter] *= 10;
          output_fqs.fq[current_profile_nr - 1][3-output_letter] += display_digits[i];
        }

        flash_output_fqs.write(output_fqs);
        program_synth();
      }
      else // display
      {
        display_submode = DM_SUBMODE_EDIT;
        if (bc == BT_CODE_LEFT_LONG) // Left
          edit_digit_pos = 0;
        else // Right
          edit_digit_pos = DIGITS_NR - 1;
      } 
     break;
    }

    case DM_PREFS_RANGE:
    {
      if (display_submode == DM_SUBMODE_EDIT)
      {
        if (edit_digit_pos == 0)
          display_digits[0] = -display_digits[0]; // Invert sign
        else
        {
          if (bc == BT_CODE_LEFT_LONG) // Left
          {
            display_digits[edit_digit_pos]--;
            if (display_digits[edit_digit_pos] < 0)
              display_digits[edit_digit_pos] = 9;
        }
          else // Right
          {
            display_digits[edit_digit_pos]++;
            if (display_digits[edit_digit_pos] > 9)
              display_digits[edit_digit_pos] = 0;
          }
        }

        tuning_range_ppt = 0;
        for (int i = 1; i < DIGITS_NR; i++)
        {
          tuning_range_ppt *= 10;
          tuning_range_ppt += display_digits[i];
        }

        if (display_digits[0] < 0)
          tuning_range_ppt = -tuning_range_ppt;


        flash_tuning_range_ppt.write(tuning_range_ppt);
      }
      else // display
      {
        display_submode = DM_SUBMODE_EDIT;

        if (bc == BT_CODE_LEFT_LONG) // Left
          edit_digit_pos = 0;
        else // Right
          edit_digit_pos = DIGITS_NR - 1;
      }

      break;
    }

    case DM_PREFS_LOOP:
    {

      if (display_submode == DM_SUBMODE_EDIT)
      {
        int new_loop_time_const;
        
        if (bc == BT_CODE_LEFT_LONG) // Left
        {
          new_loop_time_const = loop_time_constants[0];
          for (int i = 0; i < sizeof(loop_time_constants) / sizeof(int); i++)
          {
            if (loop_time_constants[i] < loop_time_const)
              new_loop_time_const = loop_time_constants[i];
          }
        }
        else // Right
        {  
          new_loop_time_const = loop_time_constants[sizeof(loop_time_constants) / sizeof(int) - 1];
          for (int i = sizeof(loop_time_constants) / sizeof(int) - 1; i >= 0; i--)
          {
            if (loop_time_constants[i] > loop_time_const)
              new_loop_time_const = loop_time_constants[i];
          }
        }
        
        loop_time_const = new_loop_time_const;

        flash_loop_time_const.write(loop_time_const);
      }
      else // display
      {
        display_submode = DM_SUBMODE_EDIT;
      }

      break;
    }

    case DM_PREFS_PREFILTER:
    {
      if (display_submode == DM_SUBMODE_EDIT)
      {
        int new_prefilter_time_const;
        if (bc == BT_CODE_LEFT_LONG) // Left
        {
          new_prefilter_time_const = prefilter_time_constants[0];
          for (int i = 0; i < sizeof(prefilter_time_constants) / sizeof(int); i++)
          {
            if (prefilter_time_constants[i] < prefilter_time_const)
              new_prefilter_time_const = prefilter_time_constants[i];
          }
        }
        else // Right
        {
          new_prefilter_time_const = prefilter_time_constants[sizeof(prefilter_time_constants) / sizeof(int) - 1];
          for (int i = sizeof(prefilter_time_constants) / sizeof(int) - 1; i >= 0; i--)
          {
            if (prefilter_time_constants[i] > prefilter_time_const)
              new_prefilter_time_const = prefilter_time_constants[i];
          }
        }
        prefilter_time_const = new_prefilter_time_const;

        flash_prefilter_time_const.write(prefilter_time_const);
      }
      else // display
      {
        display_submode = DM_SUBMODE_EDIT;
      }

      break;
    }

    case DM_PREFS_DAMPING:
    {

      if (display_submode == DM_SUBMODE_EDIT)
      {
        int new_loop_damping_thousands;
        if (bc == BT_CODE_LEFT_LONG) // Left
        {    
          new_loop_damping_thousands = loop_damping_thousands_constants[0];
          for (int i = 0; i < sizeof(loop_damping_thousands_constants) / sizeof(int); i++)
          {
            if (loop_damping_thousands_constants[i] < loop_damping_thousands)
              new_loop_damping_thousands = loop_damping_thousands_constants[i];
          }
        }
        else // Right
        {
          new_loop_damping_thousands = loop_damping_thousands_constants[sizeof(loop_damping_thousands_constants) / sizeof(int) - 1];
          for (int i = sizeof(loop_damping_thousands_constants) / sizeof(int) - 1; i >= 0; i--)
          {
            if (loop_damping_thousands_constants[i] > loop_damping_thousands)
              new_loop_damping_thousands = loop_damping_thousands_constants[i];
          }
        }
        loop_damping_thousands = new_loop_damping_thousands;

        flash_loop_damping_thousands.write(loop_damping_thousands);
      }
      else // display
      {
        display_submode = DM_SUBMODE_EDIT;
      }

      break;
    }

    case DM_PREFS:
    {

      if (bc == BT_CODE_LEFT_LONG) // Left
        display_mode = DM_PREFS_VERSION;
      else
        display_mode = DM_PREFS_RANGE;
        
      display_submode = DM_SUBMODE_DISPLAY;
      
      break;
    }
    
    case DM_PREFS_INITGPS:
    {
      gps_enable_init = !gps_enable_init;
      flash_gps_enable_init.write(gps_enable_init);
      
      break;
    }

    case DM_PREFS_COMM_MODE:
    {
      gps_comm_mode = !gps_comm_mode;
      flash_gps_comm_mode.write(gps_comm_mode);
      
      break;
    }

    case DM_PREFS_BRIGHT:
    {

      display_brightness = !display_brightness;
      flash_display_brightness.write(display_brightness);
      display.dim(display_brightness ? false : true);
      
      break;
    }

    case DM_PREFS_BOOT:
    {
      if (display_submode == DM_SUBMODE_EDIT) 
      {
        invoke_bootloader();
      }
      else
        display_submode = DM_SUBMODE_EDIT;

      break;
    }

    case DM_PREFS_RESALL:
    {
      if (display_submode == DM_SUBMODE_EDIT) // Now reset to defaults
      {
        invoke_factory_reset();
      }
      else
        display_submode = DM_SUBMODE_EDIT;

      break;
    }

    case DM_PREFS_HOLD:
    {

      display_mode = DM_STATUS;
      display_submode = DM_STATUS_LOCKSTATUS;

      dac_off = false;
      invoke_hold();

      break;
    }

    case DM_PREFS_RUN:
    {
      display_mode = DM_STATUS;
      display_submode = DM_STATUS_LOCKSTATUS;

      dac_off = false;
      invoke_run();

      break;
    }

    case DM_PREFS_DAC_MIN:
    {
      display_mode = DM_STATUS;
      display_submode = DM_STATUS_LOCKSTATUS;

      dac_off = false;
      apply_new_dac_value(DAC_MIN);

      break;
    }

    case DM_PREFS_DAC_CEN:
    {
      display_mode = DM_STATUS;
      display_submode = DM_STATUS_LOCKSTATUS;

      dac_off = false;
      apply_new_dac_value(DAC_MID);

      break;
    }

    case DM_PREFS_DAC_MAX:
    {
      display_mode = DM_STATUS;
      display_submode = DM_STATUS_LOCKSTATUS;

      dac_off = false;
      apply_new_dac_value(DAC_MAX);
      break;
    }

    case DM_PREFS_DAC_OFF:
    {
      display_mode = DM_STATUS;
      display_submode = DM_STATUS_LOCKSTATUS;

      dac_off = true;
      invoke_hold();
      break;
    }

    default:
      bc = BT_CODE_LEFT; // Replace with short press, if not used
   }   
  }

  else
  
// Menu --------------------------------------------------------------------

  if ( bc == BT_CODE_MENU )
  {

    if ( (display_mode >= DM_OUT_A) && (display_mode <= DM_OUT_D) && (display_submode == DM_SUBMODE_EDIT) )
      display_submode = DM_SUBMODE_DISPLAY;
    else
    {
      switch (display_mode)
      {
        case DM_PREFS_BOOT:
        case DM_PREFS_RESALL:
        case DM_PREFS_RANGE:
        case DM_PREFS_DAC_MIN:
        case DM_PREFS_DAC_CEN:
        case DM_PREFS_DAC_MAX:
        case DM_PREFS_DAC_OFF:
        case DM_PREFS_HOLD:
        case DM_PREFS_RUN:
        case DM_PREFS_GRADE:
        case DM_PREFS_VERSION:
        case DM_PREFS_LOOP:
        case DM_PREFS_PREFILTER:
        case DM_PREFS_DAMPING:
        case DM_PREFS_INITGPS:
        case DM_PREFS_COMM_MODE:
        case DM_PREFS_BRIGHT:
        {
          if (display_submode == DM_SUBMODE_EDIT)
            display_submode = DM_SUBMODE_DISPLAY;
          else
            display_mode = DM_PREFS;

          break;
        }

        default:
              display_mode = DM_STATUS;
      }
    }
  }

  else

// Left --------------------------------------------------------------------

  if ( bc == BT_CODE_LEFT )
  {
    if ( ( ((display_mode >= DM_OUT_A) && (display_mode <= DM_OUT_D)) || ( display_mode == DM_PREFS_RANGE ) )
         && (display_submode == DM_SUBMODE_EDIT) )
    {
      edit_digit_pos--;
      if (edit_digit_pos < 0)
        edit_digit_pos = DIGITS_NR - 1;
    }
    else if ( ( (display_mode == DM_PREFS_LOOP) || (display_mode == DM_PREFS_PREFILTER) || (display_mode == DM_PREFS_DAMPING)
                || (display_mode == DM_PREFS_BOOT) || (display_mode == DM_PREFS_RESALL) )
              && (display_submode == DM_SUBMODE_EDIT) )
    {
      display_submode = DM_SUBMODE_DISPLAY;
    }
    else
      switch (display_mode)
      {
        case DM_STATUS:
        case DM_PROFILE:
          display_mode = DM_PREFS;
          break;
        case DM_PREFS:
          display_mode = DM_OUT_D;
          display_submode = DM_SUBMODE_DISPLAY;
          break;
        case DM_OUT_D:
          display_mode = DM_OUT_C;
          display_submode = DM_SUBMODE_DISPLAY;
          break;
        case DM_OUT_C:
          display_mode = DM_OUT_B;
          display_submode = DM_SUBMODE_DISPLAY;
          break;
        case DM_OUT_B:
          display_mode = DM_OUT_A;
          display_submode = DM_SUBMODE_DISPLAY;
          break;
        case DM_OUT_A:
          display_mode = DM_PROFILE;
          break;
          
        case DM_PREFS_VERSION:
          display_mode = DM_PREFS_GRADE;
          break;
        case DM_PREFS_GRADE:
          display_mode = DM_PREFS_RESALL;
          break;
        case DM_PREFS_RESALL:
          display_mode = DM_PREFS_BOOT;
          break;
        case DM_PREFS_BOOT:
          display_mode = DM_PREFS_BRIGHT;
          break;
        case DM_PREFS_BRIGHT:
          display_mode = DM_PREFS_COMM_MODE;
          break;
        case DM_PREFS_COMM_MODE:
          display_mode = DM_PREFS_INITGPS;
          break;
        case DM_PREFS_INITGPS:
          display_mode = DM_PREFS_RUN;
          break;
        case DM_PREFS_RUN:
          display_mode = DM_PREFS_HOLD;
          break;
        case DM_PREFS_HOLD:
          display_mode = DM_PREFS_DAMPING;
          break;
        case DM_PREFS_DAMPING:
          display_mode = DM_PREFS_LOOP;
          break;
        case DM_PREFS_LOOP:
          display_mode = DM_PREFS_PREFILTER;
          break;
        case DM_PREFS_PREFILTER:
          display_mode = DM_PREFS_DAC_OFF;
          break;
        case DM_PREFS_DAC_OFF:
          display_mode = DM_PREFS_DAC_MAX;
          break;
        case DM_PREFS_DAC_MAX:
          display_mode = DM_PREFS_DAC_CEN;
          break;
        case DM_PREFS_DAC_CEN:
          display_mode = DM_PREFS_DAC_MIN;
          break;
        case DM_PREFS_DAC_MIN:
          display_mode = DM_PREFS_RANGE;
          break;
        case DM_PREFS_RANGE:
          display_mode = DM_PREFS_VERSION;
          break;
      };
  }

  else
  
// Right --------------------------------------------------------------------

  if ( bc == BT_CODE_RIGHT )
  {
    if ( ( ((display_mode >= DM_OUT_A) && (display_mode <= DM_OUT_D)) || ( display_mode == DM_PREFS_RANGE ) )
         && (display_submode == DM_SUBMODE_EDIT) )
    {
      edit_digit_pos++;
      if (edit_digit_pos >= DIGITS_NR)
        edit_digit_pos = 0;
    }
    else if ( ( (display_mode == DM_PREFS_LOOP) || (display_mode == DM_PREFS_PREFILTER) || (display_mode == DM_PREFS_DAMPING)
                || (display_mode == DM_PREFS_BOOT) || (display_mode == DM_PREFS_RESALL)  )
              && (display_submode == DM_SUBMODE_EDIT) )
    {
      display_submode = DM_SUBMODE_DISPLAY;
    }
    else
      switch (display_mode)
      {
        case DM_STATUS:
        case DM_PREFS:
          display_mode = DM_PROFILE;
          break;
        case DM_PROFILE:
          display_mode = DM_OUT_A;
          display_submode = DM_SUBMODE_DISPLAY;
          break;
        case DM_OUT_A:
          display_mode = DM_OUT_B;
          display_submode = DM_SUBMODE_DISPLAY;
          break;
        case DM_OUT_B:
          display_mode = DM_OUT_C;
          display_submode = DM_SUBMODE_DISPLAY;
          break;
        case DM_OUT_C:
          display_mode = DM_OUT_D;
          display_submode = DM_SUBMODE_DISPLAY;
          break;
        case DM_OUT_D:
          display_mode = DM_PREFS;
          break;
          
        case DM_PREFS_VERSION:
          display_mode = DM_PREFS_RANGE;
          break;
        case DM_PREFS_GRADE:
          display_mode = DM_PREFS_VERSION;
          break;
        case DM_PREFS_RESALL:
          display_mode = DM_PREFS_GRADE;
          break;
        case DM_PREFS_BOOT:
          display_mode = DM_PREFS_RESALL;
          break;
        case DM_PREFS_BRIGHT:
          display_mode = DM_PREFS_BOOT;
          break;
        case DM_PREFS_COMM_MODE:
          display_mode = DM_PREFS_BRIGHT;
          break;
        case DM_PREFS_INITGPS:
          display_mode = DM_PREFS_COMM_MODE;
          break;
        case DM_PREFS_RUN:
          display_mode = DM_PREFS_INITGPS;
          break;
        case DM_PREFS_HOLD:
          display_mode = DM_PREFS_RUN;
          break;
        case DM_PREFS_DAMPING:
          display_mode = DM_PREFS_HOLD;
          break;
        case DM_PREFS_LOOP:
          display_mode = DM_PREFS_DAMPING;
          break;
        case DM_PREFS_PREFILTER:
          display_mode = DM_PREFS_LOOP;
          break;
        case DM_PREFS_DAC_OFF:
          display_mode = DM_PREFS_PREFILTER;
          break;
        case DM_PREFS_DAC_MAX:
          display_mode = DM_PREFS_DAC_OFF;
          break;
        case DM_PREFS_DAC_CEN:
          display_mode = DM_PREFS_DAC_MAX;
          break;
        case DM_PREFS_DAC_MIN:
          display_mode = DM_PREFS_DAC_CEN;
          break;
        case DM_PREFS_RANGE:
          display_mode = DM_PREFS_DAC_MIN;
          break;
      };
  }
}

// Display the data

void display_data()
{

  display.clearDisplay();

//  display.setFont(&FreeMonoBold12pt7b);
  display.setTextSize(1, 2);

  display.clearDisplay();
  display.setCursor(0, SCREEN_HEIGHT - 1 - 4);

  switch (display_mode)
  {

    case DM_STATUS:
      {
//        int new_display_mode = display_mode;
//        int new_display_submode = display_submode;
  
        switch ( display_submode)
        {
          case DM_STATUS_LOCKSTATUS:

            if (hold_mode)
            {

               display.print("- Hold -");

              if(present_1pps)
                display_submode = DM_STATUS_DT;
              else
                display_submode = DM_STATUS_DAC;
            }
            else if (locked_state)
            {
              display.print("GPS Lock");
              display_submode = DM_STATUS_LOCKTIME;
            }
            else
            {
              if (present_1pps)
              {
                display.print("-LOCKING-");
                display_submode = DM_STATUS_DT;
              }
              else
              {
                display.setTextSize(1, 1);
                display.setCursor(0, SCREEN_HEIGHT / 2 - 1 - 2);
                display.print("Searching");
                display.setCursor(0, SCREEN_HEIGHT - 1);
                display.print("for sats");

              }
            }
            break;

          case DM_STATUS_LOCKTIME:
            {
              unsigned int t = locked_time;
              unsigned int d = t / 24 / 60 / 60;
              if (d)
              {
                display.print(d);
                display.print("d");
                t -= d * 24 * 60 * 60;
              }
              unsigned int h = t / 60 / 60;
              if (h < 10)
                display.print("0");
              display.print(h);
              display.print("h");
              t -= h * 60 * 60;
              if (d < 100)
              {
                unsigned int m = t / 60;
                if (m < 10)
                  display.print("0");
                display.print(m);
                display.print("m");

                if (d == 0)
                {
                  t -= m * 60;
                  if (t < 10)
                    display.print("0");
                  display.print(t);
                  display.print("s");
                }
              }
//              display_submode = DM_STATUS_LOCKSTATUS; //DM_STATUS_DT;
              if (avg_cno>0)
                display_submode = DM_STATUS_AVG_CNO; 
              else
                display_submode = DM_STATUS_LOCKSTATUS;

              break;
            }

          case DM_STATUS_DT:

            if (last_ns_value < -9999)
              display.print("dt<-10us");
            else if (last_ns_value > 9999)
              display.print("dt>10us");
            else
            {
              display.print("dt=");
              if (fabs(last_ns_value) > 999)
              {
                display.print(last_ns_value / 1000, 1);
                display.print("us");
              }
              else if (fabs(last_ns_value) > 5)
              {
                display.print(int(last_ns_value));
                display.print("ns");
              }
              else
              {
                display.print(last_ns_value, 1);
                display.print("ns");
              }
            }

            if (hold_mode)
              display_submode = DM_STATUS_DAC;
            else
            {
              if (avg_cno>0)
                display_submode = DM_STATUS_AVG_CNO; 
              else
                display_submode = DM_STATUS_LOCKSTATUS;
            }
            
            break;

          case DM_STATUS_AVG_CNO:

            display.print("CN=");
            display.print(avg_cno); 
            display.print("dBHz");
            display_submode = DM_STATUS_LOCKSTATUS;

            break;

          case DM_STATUS_DAC:

              if (dac_off)
                display.print("-DAC OFF-");
              else
              {
                display.print("DAC=");
                display.print(dac_value);
              }
              
              display_submode = DM_STATUS_LOCKSTATUS;
            break;
        };

//        if (flag_2s)
//        {
//          display_mode = new_display_mode;
//          display_submode = new_display_submode;
//          flag_2s = false;
//        }

        break;
      }

    case DM_PROFILE:
      {
        display.print("Profile:");
        display.print(current_profile_nr);
        break;
      }

    case DM_OUT_A:
    case DM_OUT_B:
    case DM_OUT_C:
    case DM_OUT_D:
      {
        int output_letter = display_mode - DM_OUT_A; // 0...3
        display.setTextSize(1, 1);

        if (display_submode == DM_SUBMODE_DISPLAY)
        {
          uint32_t remainder = output_fqs.fq[current_profile_nr - 1][3-output_letter];
          for (int i = 0; i < DIGITS_NR; i++)
          {
            uint32_t current_digit = remainder / 100000000;
            remainder -= current_digit * 100000000;
            remainder *= 10;

            display_digits[i] = current_digit;
          }

          edit_digit_pos = 0;

          //          display.clearDisplay();
          display.setCursor(0, SCREEN_HEIGHT / 2 - 1 - 2);
          display.print("Out ");
          display.print(char('A'+output_letter));
          display.print(": Hz");
        }
        else // Edit
        {
          display.setCursor(0, SCREEN_HEIGHT / 2 - 1 - 2);
          display.print("Edit ");
          display.print(char('A'+output_letter));
          display.print(":Hz");
        }

        int xpos = 0;

        boolean flag0 = true;
        for (int i = 0; i < DIGITS_NR; i++)
        {
          int8_t current_digit = display_digits[i];

          char c;

          if ( (display_submode == DM_SUBMODE_EDIT) && (i == edit_digit_pos) && flag_blinker )
          {
            c = ' ';
          }
          else if (flag0 && (current_digit == 0) && (display_submode == DM_SUBMODE_DISPLAY) && (i != (DIGITS_NR - 1)) )
            c = ' ';
          else
          {
            flag0 = false;
            c = current_digit + '0';
          }

          display.setCursor(xpos, SCREEN_HEIGHT - 1);
          display.print(c);
          xpos += 13;
          if ( (i == 2) || (i == 5) )
            xpos += 4;
        }

        if ( (output_fqs.fq[current_profile_nr - 1][3-output_letter] > 999999) || (display_submode == DM_SUBMODE_EDIT) )
        {
          display.drawPixel(41, 31 - 1, SSD1306_WHITE);
          display.drawPixel(41, 31, SSD1306_WHITE);
          display.drawPixel(41 - 1, 31, SSD1306_WHITE);
          display.drawPixel(41 + 1, 31, SSD1306_WHITE);
        }
        if ( (output_fqs.fq[current_profile_nr - 1][3-output_letter] > 999) || (display_submode == DM_SUBMODE_EDIT) )
        {
          display.drawPixel(84, 31 - 1, SSD1306_WHITE);
          display.drawPixel(84, 31, SSD1306_WHITE);
          display.drawPixel(84 - 1, 31, SSD1306_WHITE);
          display.drawPixel(84 + 1, 31, SSD1306_WHITE);
        }

        break;
      }


    case DM_PREFS:
      {
        display.print("Prefs...");
        break;
      }

    case DM_PREFS_BOOT:
      {
        if (display_submode == DM_SUBMODE_EDIT)
        {
          if (flag_blinker)
            display.print("Sure?");
        }
        else
        {
          display.setTextSize(1, 1);
          display.setCursor(0, SCREEN_HEIGHT / 2 - 1 - 2);
          display.print("Run boot");
          display.setCursor(0, SCREEN_HEIGHT - 1);
          display.print("loader");
        }
        break;
      }

    case DM_PREFS_RESALL:
      {
        if (display_submode == DM_SUBMODE_EDIT)
        {
          if (flag_blinker)
            display.print("Sure?");
        }
        else
        {
          display.setTextSize(1, 1);
          display.setCursor(0, SCREEN_HEIGHT / 2 - 1 - 2);
          display.print("Factory");
          display.setCursor(0, SCREEN_HEIGHT - 1);
          display.print("reset");
        }
        break;
      }

    case DM_PREFS_INITGPS:
      {
        display.setTextSize(1, 1);
        display.setCursor(0, SCREEN_HEIGHT / 2 - 1 - 2);
        display.print("Init GPS");

        display.setCursor(0, SCREEN_HEIGHT - 1);
        if (gps_enable_init)
          display.print("> YES");
        else
          display.print("> NO");

        break;
      }

    case DM_PREFS_COMM_MODE:
      {
        display.setTextSize(1, 1);
        display.setCursor(0, SCREEN_HEIGHT / 2 - 1 - 2);
        display.print("Comm mode");

        display.setCursor(0, SCREEN_HEIGHT - 1);
        if (gps_comm_mode)
          display.print("> GPS");
        else
          display.print("> Cmdline");

        break;
      }
    case DM_PREFS_BRIGHT:
      {
        display.setTextSize(1, 1);
        display.setCursor(0, SCREEN_HEIGHT / 2 - 1 - 2);
        display.print("Display");

          display.setCursor(0, SCREEN_HEIGHT - 1);
          if (display_brightness)
            display.print("> BRIGHT");
          else
            display.print("> DARK");

        break;
      }

    case DM_PREFS_GRADE:
      {
        display.setTextSize(1, 1);
        display.setCursor(0, SCREEN_HEIGHT / 2 - 1 - 2);
        display.print("SI5338:");
        char grade_letter = si_grade-1+'A';
        if (si_grade==0)
          grade_letter = '?';
        display.print(grade_letter);
        display.setCursor(0, SCREEN_HEIGHT - 1);
        display.print("(");
        display.print(si_max_fq);
        display.print(" MHz)");
        break;
      }

    case DM_PREFS_VERSION:
      {
        display.setTextSize(1, 1);
        display.setCursor(0, SCREEN_HEIGHT / 2 - 1 - 2);
        display.print("FW ver.:");
        display.setCursor(0, SCREEN_HEIGHT - 1);
        display.print(FIRMWARE_VERSION);
        break;
      }

    case DM_PREFS_LOOP:
      {
        display.setTextSize(1, 1);
        display.setCursor(0, SCREEN_HEIGHT / 2 - 1 - 2);
        display.print("Loop fil.");
        if ( (display_submode == DM_SUBMODE_DISPLAY) || flag_blinker )
        {
          display.setCursor(0, SCREEN_HEIGHT - 1);
          display.print(loop_time_const);
          display.print(" s");
        }

        break;
      }

    case DM_PREFS_PREFILTER:
      {
        display.setTextSize(1, 1);
        display.setCursor(0, SCREEN_HEIGHT / 2 - 1 - 2);
        display.print("Prefilter");
        if ( (display_submode == DM_SUBMODE_DISPLAY) || flag_blinker )
        {
          display.setCursor(0, SCREEN_HEIGHT - 1);
          if (prefilter_time_const)
          {
            display.print(prefilter_time_const);
            display.print(" s");
          }
          else
            display.print("off");
        }

        break;
      }

    case DM_PREFS_DAMPING:
      {
        display.setTextSize(1, 1);
        display.setCursor(0, SCREEN_HEIGHT / 2 - 1 - 2);
        display.print("Damping");
        if ( (display_submode == DM_SUBMODE_DISPLAY) || flag_blinker )
        {
          display.setCursor(0, SCREEN_HEIGHT - 1);
          display.print(loop_damping_thousands / 1000.0);
        }

        break;
      }

    case DM_PREFS_RANGE:
      {

        display.setTextSize(1, 1);

        if (display_submode == DM_SUBMODE_DISPLAY)
        {
          int remainder = tuning_range_ppt;

          if (remainder < 0)
          {
            display_digits[0] = -1; // Minus sign
            remainder = -remainder;
          }
          else
            display_digits[0] = 1; // Plus sign

          if (remainder >= 99999999)
            remainder = 99999999;

          for (int i = 1; i < DIGITS_NR; i++)
          {
            int current_digit = remainder / 10000000;
            remainder -= current_digit * 10000000;
            remainder *= 10;

            display_digits[i] = current_digit;
          }

          edit_digit_pos = 0;
        }

        display.setCursor(0, SCREEN_HEIGHT / 2 - 1 - 2);
        if (display_submode == DM_SUBMODE_DISPLAY)
          display.print("Range:ppt");
        else
          display.print("Edit: ppt");

        int xpos = 0;
        boolean flag0 = true;
        for (int i = 0; i < DIGITS_NR; i++)
        {
          int8_t current_digit = display_digits[i];

          char c;

          if ( (display_submode == DM_SUBMODE_EDIT) && (i == edit_digit_pos) && flag_blinker )
          {
            c = ' ';
          }
          else if (flag0 && (current_digit == 0) && (display_submode == DM_SUBMODE_DISPLAY) && (i > 0) && (i != (DIGITS_NR - 1)) )
            c = ' ';
          else
          {
            if (i > 0)
            {
              flag0 = false;
              c = current_digit + '0';
            }
            else
            {
              if ((display_submode == DM_SUBMODE_EDIT))
                c = (current_digit >= 0) ? '+' : '-';
              else
                c = (current_digit >= 0) ? ' ' : '-';
            }
          }

          display.setCursor(xpos, SCREEN_HEIGHT - 1);
          display.print(c);
          xpos += 13;
          if ( (i == 2) || (i == 5) )
            xpos += 4;
        }

        if ( (abs(tuning_range_ppt) > 999999) || (display_submode == DM_SUBMODE_EDIT) )
        {
          display.drawPixel(41, 31 - 1, SSD1306_WHITE);
          display.drawPixel(41, 31, SSD1306_WHITE);
          display.drawPixel(41 - 1, 31, SSD1306_WHITE);
          display.drawPixel(41 + 1, 31, SSD1306_WHITE);
        }
        if ( (abs(tuning_range_ppt) > 999) || (display_submode == DM_SUBMODE_EDIT)  )
        {
          display.drawPixel(84, 31 - 1, SSD1306_WHITE);
          display.drawPixel(84, 31, SSD1306_WHITE);
          display.drawPixel(84 - 1, 31, SSD1306_WHITE);
          display.drawPixel(84 + 1, 31, SSD1306_WHITE);
        }
        break;
      }

    case DM_PREFS_DAC_MIN:
      {
        display.print("SetDACmin");
        break;
      }

    case DM_PREFS_DAC_CEN:
      {
        display.print("SetDACcen");
        break;
      }

    case DM_PREFS_DAC_MAX:
      {
        display.print("SetDACmax");
        break;
      }

    case DM_PREFS_DAC_OFF:
      {
        display.print("DAC off");
        break;
      }

    case DM_PREFS_HOLD:
      {
        display.print("Hold");
        break;
      }

    case DM_PREFS_RUN:
      {
        display.print("Run");
        break;
      }

  };

  display.display();
}

// TIC state machine

uint8_t tic_state = 0;

void tic_state_machine()
{
  switch (tic_state)
  {
    case 0:
      tof.startMeasurement();
      tic_state = 1;
      break;

    case 1:
      if (digitalRead(PIN_TDC7200_INT) == LOW)
      {
        uint64_t time_ps;

        if (tof.readMeasurement(1, time_ps))
        {
          int offset_time_ps = int(time_ps) - (CNT_PERIOD_PS / 2); // Zero line is actually in the middle of period

          if (!skip_next1pps)
          {
            if ( (old_offset_time_ps > CNT_PERIOD_PS / 4) && (offset_time_ps < -CNT_PERIOD_PS / 4) )
              coarse_time_ps += CNT_PERIOD_PS;
            if ( (old_offset_time_ps < -CNT_PERIOD_PS / 4) && (offset_time_ps > CNT_PERIOD_PS / 4) )
              coarse_time_ps -= CNT_PERIOD_PS;
          }

          old_offset_time_ps = offset_time_ps;


#ifdef MAX_CONT_TIME_PS
          if ( (coarse_time_ps >= MAX_CONT_TIME_PS) || (coarse_time_ps < -MAX_CONT_TIME_PS))
          {
            // The continuous time is too far from, now reset the coarse timer
            coarse_time_ps = 0;
          }
#endif
          cont_time_old_ps = cont_time_ps;
          cont_time_ps = coarse_time_ps // Coarse timer
                         + offset_time_ps // Measured by TIC
                         - qerr_ps // qErr from GPS module
                         + fq_offset_ppt; // Add required frequency offset

          if (apply_qerr_correction == 2) // Special case, feed qErr to the input of the prefilter
          {
            cont_time_ps = -qerr_ps;
          }

          if (!skip_next1pps)
          {
            next_interval_ready = true;
          }
          skip_next1pps = false;

        }
        else
        {} // bug

        tic_state = 0;
        break;
      }
      break;

  };
}


// Arduino main loop

void loop()
{

  // TIC state machine

  tic_state_machine();

  // Read useful info from GPS

  if (!gps_comm_mode)
  {
    while (Serial1.available())
    {
      byte b = Serial1.read();

      receive_gps_byte(b);
    }
  }
  
  if (!apply_qerr_correction)
  {
    qerr_ps = 0;
  }

  // Run the PLL

  if (next_interval_ready)
  {

    // Pre-filter

    if (need_reset_prefilter)
    {
      tic_prefiltered_ps = cont_time_ps;
      need_reset_prefilter = false;
    }

    if (prefilter_time_const) // Use prefilter
      tic_prefiltered_ps += (cont_time_ps - tic_prefiltered_ps) / prefilter_time_const;
    else // Do not use prefilter
      tic_prefiltered_ps = cont_time_ps; // Skip prefilter


    if (!hold_mode)
    {

      // Lock/unlock

      if (!locked_state)
      {
        if (abs(tic_prefiltered_ps) < LOCK_LIMIT_PS)
        {
          lock_unlock_counter ++;
          if (lock_unlock_counter >= LOCK_TIME)
          {
            //              locked_state_changed = true;
            locked_state = true;
            locked_time = 0;
            lock_unlock_counter = 0;
          }
        }
        else
          lock_unlock_counter = 0;
      }
      else // Locked
      {
        if (abs(tic_prefiltered_ps) > UNLOCK_LIMIT_PS)
        {
          lock_unlock_counter ++;
          if (lock_unlock_counter >= UNLOCK_TIME)
          {
            //              locked_state_changed = true;
            locked_state = false;
            lock_unlock_counter = 0;
          }
        }
        else
          lock_unlock_counter = 0;
      }

      // PI-loop

      double loop_damping = loop_damping_thousands / 1000.0;

      double filter_gain = 65536.0 / tuning_range_ppt;

      double loop_p = tic_prefiltered_ps / loop_time_const; // P term, divided by time constant
      loop_i += loop_p / loop_damping / loop_time_const; // I term

      int int_dac = DAC_MID + (loop_p + loop_i)*filter_gain; // Output of the PI filter

      if (int_dac < DAC_MIN)
        int_dac = DAC_MIN;
      else if (int_dac > DAC_MAX)
        int_dac = DAC_MAX;

      dac_value = int_dac;

      set_dac(dac_value, dac_off); // Output to DAC
    }
  }

  // Output data to PC

  if (!gps_comm_mode && next_interval_ready)
  {
    output_data_to_pc();
  }


  // Serial data exchange

  if (gps_comm_mode)
  {
    gps_transparent_exchange(); // GPS transparent mode
  }
  else
  {
    char_interpret(); // Interpret command
  }

  // Prepare data for display

  if (next_interval_ready)
  {
    if (hold_mode)
      last_ns_value = (cont_time_ps - cont_time_old_ps) / 1000.0; // Difference between OCXO times in 1s interval (1s interval is generated by GPS)
    else
      last_ns_value = cont_time_ps / 1000.0;  // Difference between GPS time and OCXO time
  }

  next_interval_ready = false;

  // End of main algorithm

  // Now process buttons and display data

  boolean refresh_display = flag_2s; // Auto refresh display every 2 seconds
  flag_2s = false;

  // Refresh on blinker

  if ( (display_submode == DM_SUBMODE_EDIT) &&
       ( ( (display_mode >= DM_OUT_A) && (display_mode <= DM_OUT_D) )
         || (display_mode == DM_PREFS_RANGE) || (display_mode == DM_PREFS_LOOP)
         || (display_mode == DM_PREFS_PREFILTER) || (display_mode == DM_PREFS_DAMPING)  
         || (display_mode == DM_PREFS_BOOT) || (display_mode == DM_PREFS_RESALL)
       )
     )
  {
    if (flag_05s)
    {
      refresh_display = true;
      flag_05s = false;
      flag_blinker = !flag_blinker;
    }
  }

  // Process the buttons

  uint8_t bc = read_button_code();

  if (bc != BT_CODE_NONE)
  {
    process_buttons(bc);
    refresh_display = true; // Refresh display after the button is processed
    flag_2s = false;
  }

  // Display the data

  if (refresh_display)
    display_data();


  // End of main loop
}
