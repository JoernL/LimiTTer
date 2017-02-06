/*   
   It scans the Freestyle Libre Sensor every 5 minutes
   and sends the data to the xDrip Android app. You can
   see the data in the serial monitor of Arduino IDE, too.
   If you want another scan interval, simply change the
   sleepTime value. To work with Android 4 you have to disable
   all lines containing a "for Android 4" comment and set the
   sleep time to 36.
     
   This sketch is based on a sample sketch for the BM019 module
   from Solutions Cubed.

   Wiring for UNO / Pro-Mini:

   Arduino          BM019           BLE-HM11
   IRQ: Pin 9       DIN: pin 2
   SS: pin 10       SS: pin 3
   MOSI: pin 11     MOSI: pin 5 
   MISO: pin 12     MISO: pin4
   SCK: pin 13      SCK: pin 6
   I/O: pin 3 (VCC with Android 4)  VCC: pin 9 
   I/O: pin 5                       TX:  pin 2
   I/O: pin 6                       RX:  pin 4
*/

/*
 * VWI, V 0.1, 02/2017
 * LBridge, a LimiTTer with xBridge Protocol extension
 * 
 * Connects an Abbott Freestyle Libre sensor to the xDrip+ App, sending NFC BG readings to the phone every 5 min. 
 * 
 * Hardwaresource in xDrip has to be set to "xBridge Wixel", Dexcom Transmitter ID can but must not to be changed. 
 * 
 * xDrip+ will use here the xBridge protocol, which enables LimiTTer to resend packages in case of failure and queue up not 
 * sent packages with the correct timestamp in the next trasnmission.
 *  
 * This sketch is based on LimiTTer code for NFC reading and sleep mode handling and a port of the xBridge2 protocol(savek-cc fork).
 * 
 * Problems solved: Code extended and rearranged to use most of original xDrip code and to fit into the Arduino platform. 
 * There were severe differences between Wixel and Arduino in regard of millis() and data formats (short float / float). The main 
 * loop logic and timers were tweaked to replace the G4 tranismitter algo with the Freestyle libre NFC reading algo.
 * Remark: The raw field from Dexcom original data field was extended to unsigned long as the short float from the Wixel processor with 
 * 
 * 16 bit dont exist in Arduino. Due to main memory problems all constant strings are moved to Flash mem with F(...) statement. 
 * xBridge console commands are not supported yet. no settings are stored in EEPROM yet, TXID ID will be set at program start 
 * with the beacon algo.
 */

#include <SPI.h>
#include <SoftwareSerial.h>
#include <avr/sleep.h> 
#include <avr/power.h>
#include <avr/wdt.h>

/* ***********************************************
 *  config #DEFINES
 ************************************************ */

#define SHOW_LIMITTER      // show Limitter NFC readings
#define SEND_DEBUG           // console messages
#define SHOW_BLE        // show BLE data stream
#define N_SEND_TEST_DATA  // simulate BG readings for test

/* ********* LimiTTer stuff ******************** */

#define MIN_V 3450 // battery empty level
#define MAX_V 4050 // battery full level

const int SSPin = 10;  // Slave Select pin
const int IRQPin = 9;  // Sends wake-up pulse for BM019
//const int NFCPin1 = 7; // Power pin BM019
const int NFCPin2 = 8; // Power pin BM019
//const int NFCPin3 = 4; // Power pin BM019
const int BLEPin = 3; // BLE power pin.
const int MOSIPin = 11;
const int SCKPin = 13;
byte RXBuffer[24];
byte NFCReady = 0;  // used to track NFC state
byte FirstRun = 1;
byte batteryLow;
int batteryPcnt;
long batteryMv;

// sleeptime in multipliers of 8 s
//int sleepTime = 32; // SleepTime. Set this to 36 for Android 4
int sleepTime = 32;

int noDiffCount = 0;
int sensorMinutesElapse;
float lastGlucose;
float trend[16];

SoftwareSerial ble_Serial(5, 6); // RX | TX

/* ************************************************* */
/*  VWI control stuff */
/* ************************************************ */

// global error counters
long loop_count = 0;          // of main loop
long ble_connect_errors = 0;  // no BLE connect after 40 s wait time
long nfc_read_errors = 0;     // e. g. no sensor in range 
long nfc_scan_count = 0;      // how many scans?

// we need millis() which counts up during dosleep()
// how much seconds is the program running
unsigned long prg_run_time = 0;   // in sec
unsigned long loop_start_time;    // in ms
unsigned long loop_time;          // in ms

/* ************************************************************* */
/* code ported form xBridge2.c */
/* ************************************************************* */

//define the maximum command string length for USB commands.
#define COMMAND_MAXLEN 40

// defines the xBridge protocol functional level.  Sent in each packet as the last byte.
#define DEXBRIDGE_PROTO_LEVEL (0x01)

static volatile boolean do_sleep = 0; // indicates we should go to sleep between packets
static volatile boolean got_ack = 0;  // indicates if we got an ack during the last do_services.
static volatile boolean is_sleeping = 0;  // flag indicating we are sleeping.
static volatile boolean usb_connected;    // indicates DTR set on USB.  Meaning a terminal program is connected via USB for debug.
static volatile boolean writing_flash;    // indicates if we are writing to flash.
static volatile boolean ble_sleeping;   // indicates if we have recieved a message from the BLE module that it is sleeping, or if false, it is awake.
static volatile boolean dex_tx_id_set;    // indicates if the Dexcom Transmitter id (settings.dex_tx_id) has been set.  Set in doServices.
static volatile boolean ble_connected;    // bit indicating the BLE module is connected to the phone.  Prevents us from sending data without this.
//static volatile boolean save_settings;    // flag to indicate that settings should be saved to flash.
static volatile boolean got_packet;     // flag to indicate we have captured a packet.
static volatile boolean got_ok;       // flag indicating we got OK from the HM-1x
//static volatile boolean do_leds;      // flag indicating to NOT show LEDs
#ifdef SEND_DEBUG
static volatile boolean send_debug = 1; // flag indicating to send debug output
#else
static volatile boolean send_debug = 0; // flag indicating to send debug output
#endif /* SEND_DEBUG */
//static volatile boolean sleep_ble;      // flag indicating to sleep the BLE module (power down in sleep)
static volatile unsigned long dly_ms = 0;
static volatile unsigned long pkt_time = 0;
static volatile unsigned long abs_pkt_time = 0;
static volatile unsigned long last_abs_pkt_time = 0;
//static volatile boolean initialised;          // flag to indicate we have passed the initialisation.

unsigned char battery_capacity=0;
//unsigned long last_beacon = 0;

//structure of a USB command
typedef struct _command_buff
{
  unsigned char commandBuffer[COMMAND_MAXLEN];
  unsigned char nCurReadPos;
} t_command_buff;

static t_command_buff command_buff;

// _Dexcom_packet - Type Definition of a Dexcom Radio packet
// actual packet length is 17 bytes, excluding len and checksum, rssi &LQI.
// modified raw and filtered to unsigned long as Arduino float is 32 bit
// packet used for simplicity of proting the code, only used as temp storage
// reduce to what is needed for LimiTTer NFC
typedef struct _Dexcom_packet
{
//  unsigned char len;
//  unsigned long  dest_addr;
//  unsigned long  src_addr;
//  unsigned char port;
//  unsigned char device_info;
//  unsigned char txId;
//  unsigned int  raw;
//  unsigned int  filtered;
  unsigned long raw;
//  unsigned long  filtered;
//  unsigned char battery;
//  unsigned char unknown;
//  unsigned char checksum;
//  signed char  RSSI;
//  unsigned char LQI;
  unsigned long ms;
} Dexcom_packet;

#define DXQUEUESIZE 64 //only use 2^x values here!
typedef struct {
  volatile unsigned char read;
  volatile unsigned char write;
  Dexcom_packet buffer[DXQUEUESIZE];
} Dexcom_fifo;

Dexcom_fifo Pkts;

Dexcom_packet * DexPkt;

// structure of a raw record we will send.
typedef struct _RawRecord
{
  unsigned char size; //size of the packet.
  unsigned char cmd_code; // code for this data packet.  Always 00 for a Dexcom data packet.
  unsigned long raw;  //"raw" BGL value. ??? use unfiltered NFC readings here?
  unsigned long filtered; //"filtered" BGL value 
  unsigned char dex_battery;  //battery value
  unsigned char my_battery; //xBridge battery value
  unsigned long dex_src_id;   //raw TXID of the Dexcom Transmitter
  //int8  RSSI; //RSSI level of the transmitter, used to determine if it is in range.
  //uint8 txid; //ID of this transmission.  Essentially a sequence from 0-63
  unsigned long delay;
  unsigned char function; // Byte representing the xBridge code funcitonality.  01 = this level.
} RawRecord;

// _xBridge_settings - Type definition for storage of xBridge_settings
// used for compatibility
typedef struct _xBridge_settings
{
  unsigned long dex_tx_id;     //4 bytes
//  unsigned int battery_maximum; //2 bytes
//  unsigned int battery_minimum; //2 bytes
//  unsigned int flags;     //2 bytes
  unsigned long uart_baudrate; //4 bytes
} xBridge_settings;     //14 bytes total

xBridge_settings settings;

// array of HM-1x baudrates for rate detection.
unsigned long uart_baudrate[9] = {9600L,19200L,38400L,57600L,115200L,4800,2400,1200,230400L};

/* ********************************************* */
/*  help functions */
/* *********************************************** */

// get free mem available
extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;
int freeMemory() {
  int free_memory;
  
  if((int)__brkval == 0)
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
  else
    free_memory = ((int)&free_memory) - ((int)__brkval);
  
  return free_memory;
}

void memfree(String func)
{
  Serial.print(func);
  Serial.print(F(" mem: "));
  Serial.println(freeMemory());
}

void freeRam(void)
{
//  Serial.print(F("ram "));
//  Serial.println(free_Ram());
}

int free_Ram() 
{
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void checkRam(void)
{
  if ( free_Ram() < 100 ) {
    Serial.print(F("*** warning, RAM available "));
    Serial.print(free_Ram());  
  }
}

// millis() since program start, Arduino millis not counted when in sellp mode

unsigned long abs_millis(void)
{
  return(prg_run_time*1000 + (millis() - loop_start_time));
}

/* **************************************************************** */
/* modified LimiTTer code */
/* **************************************************************** */

// moved some stuff to begin of loop()
void setup() {
    pinMode(IRQPin, OUTPUT);
    digitalWrite(IRQPin, HIGH); 
    pinMode(SSPin, OUTPUT);
    digitalWrite(SSPin, HIGH);
//    pinMode(NFCPin1, OUTPUT);
//    digitalWrite(NFCPin1, HIGH);
    pinMode(NFCPin2, OUTPUT);
    digitalWrite(NFCPin2, HIGH);
//    pinMode(NFCPin3, OUTPUT);
//    digitalWrite(NFCPin3, HIGH);
    pinMode(BLEPin, OUTPUT); // Disable this for Android 4
    digitalWrite(BLEPin, HIGH); // Disable this for Android 4
    pinMode(MOSIPin, OUTPUT);
    pinMode(SCKPin, OUTPUT);

    Serial.begin(9600);
}

// new function to fit Limitter in xBridge concept, called from loop(), formerly in setup()
void configNFC(void)
{
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32);

  delay(10);                      // send a wake up
  digitalWrite(IRQPin, LOW);      // pulse to put the 
  delayMicroseconds(100);         // BM019 into SPI
  digitalWrite(IRQPin, HIGH);     // mode 
  delay(10);
  digitalWrite(IRQPin, LOW);
}

/* *********************************************************** */
/* LimiTTer code, only small modifications */
/* *********************************************************** */

void SetProtocol_Command() {

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x02);  // Set protocol command
  SPI.transfer(0x02);  // length of data to follow
  SPI.transfer(0x01);  // code for ISO/IEC 15693
  SPI.transfer(0x0D);  // Wait for SOF, 10% modulation, append CRC
  digitalWrite(SSPin, HIGH);
  delay(1);
 
  digitalWrite(SSPin, LOW);
  while(RXBuffer[0] != 8)
    {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    }
  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read         
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  digitalWrite(SSPin, HIGH);

  if ((RXBuffer[0] == 0) & (RXBuffer[1] == 0))  // is response code good?
    {
    print_state(F(" - Protocol Set Command OK"));
    NFCReady = 1; // NFC is ready
    }
  else
    {
    Serial.println(F("L - Protocol Set Command FAIL"));
    NFCReady = 0; // NFC not ready
    }
}

void Inventory_Command() {
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows is 0
  SPI.transfer(0x26);  // request Flags byte
  SPI.transfer(0x01);  // Inventory Command for ISO/IEC 15693
  SPI.transfer(0x00);  // mask length for inventory command
  digitalWrite(SSPin, HIGH);
  delay(1);
 
  digitalWrite(SSPin, LOW);
  while(RXBuffer[0] != 8)
    {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    }
  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read         
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  for (byte i=0;i<RXBuffer[1];i++)      
      RXBuffer[i+2]=SPI.transfer(0);  // data
  digitalWrite(SSPin, HIGH);
  delay(1);

  if (RXBuffer[0] == 128)  // is response code good?
    {
    print_state(F(" - Sensor in range ... OK"));
    NFCReady = 2;
    }
  else
    {
    print_state(F(" - Sensor out of range"));
    NFCReady = 1;
    }
 }
 
float Read_Memory() {
 byte oneBlock[8];
 String hexPointer = "";
 String trendValues = "";
 String hexMinutes = "";
 String elapsedMinutes = "";
 float trendOneGlucose; 
 float trendTwoGlucose;
 float currentGlucose = 0;  // initialise to avoid compiler warning
 float shownGlucose;
 float averageGlucose = 0;
 int glucosePointer;
 int validTrendCounter = 0;
 float validTrend[16];
  
 for ( int b = 3; b < 16; b++) {
 
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows
  SPI.transfer(0x02);  // request Flags byte
  SPI.transfer(0x20);  // Read Single Block command for ISO/IEC 15693
  SPI.transfer(b);  // memory block address
  digitalWrite(SSPin, HIGH);
  delay(1);
 
  digitalWrite(SSPin, LOW);
  while(RXBuffer[0] != 8)
    {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    }
  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read         
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
 for (byte i=0;i<RXBuffer[1];i++)
   RXBuffer[i+2]=SPI.transfer(0);  // data
   
  digitalWrite(SSPin, HIGH);
  delay(1);
  
 for (int i = 0; i < 8; i++)
   oneBlock[i] = RXBuffer[i+3];

  char str[24];
  unsigned char * pin = oneBlock;
  const char * hex = "0123456789ABCDEF";
  char * pout = str;
  for(; pin < oneBlock+8; pout+=2, pin++) {
      pout[0] = hex[(*pin>>4) & 0xF];
      pout[1] = hex[ *pin     & 0xF];
  }
  pout[0] = 0;
#ifdef SHOW_LIMITTER
  Serial.println(str);
#endif
  trendValues += str;
 }

 digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows
  SPI.transfer(0x02);  // request Flags byte
  SPI.transfer(0x20);  // Read Single Block command for ISO/IEC 15693
  SPI.transfer(39);  // memory block address
  digitalWrite(SSPin, HIGH);
  delay(1);
 
  digitalWrite(SSPin, LOW);
  while(RXBuffer[0] != 8)
    {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    }
  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read         
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
 for (byte i=0;i<RXBuffer[1];i++)
   RXBuffer[i+2]=SPI.transfer(0);  // data
   
  digitalWrite(SSPin, HIGH);
  delay(1);
  
 for (int i = 0; i < 8; i++)
   oneBlock[i] = RXBuffer[i+3];
    
  char str[24];
  unsigned char * pin = oneBlock;
  const char * hex = "0123456789ABCDEF";
  char * pout = str;
  for(; pin < oneBlock+8; pout+=2, pin++) {
      pout[0] = hex[(*pin>>4) & 0xF];
      pout[1] = hex[ *pin     & 0xF];
  }
  pout[0] = 0;

#ifdef SHOW_LIMITTER
  Serial.println(str);
#endif

  elapsedMinutes += str;
    
  if (RXBuffer[0] == 128) // is response code good?
    {
      hexMinutes = elapsedMinutes.substring(10,12) + elapsedMinutes.substring(8,10);
      hexPointer = trendValues.substring(4,6);
      sensorMinutesElapse = strtoul(hexMinutes.c_str(), NULL, 16);
      glucosePointer = strtoul(hexPointer.c_str(), NULL, 16);

#ifdef SHOW_LIMITTER             
      Serial.println(F(""));
      Serial.print(F("Glucose pointer: "));
      Serial.print(glucosePointer);
      Serial.println(F(""));
#endif      
      int ii = 0;
      for (int i=8; i<=200; i+=12) {
        if (glucosePointer == ii)
        {
          if (glucosePointer == 0)
          {
            String trendNow = trendValues.substring(190,192) + trendValues.substring(188,190);
            String trendOne = trendValues.substring(178,180) + trendValues.substring(176,178);
            String trendTwo = trendValues.substring(166,168) + trendValues.substring(164,166);
            currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));
            trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL ,16));
            trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL ,16));

            if (FirstRun == 1)
               lastGlucose = currentGlucose;
       
            if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
            {
               if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
                  currentGlucose = trendTwoGlucose;
               else
                  currentGlucose = trendOneGlucose;
            }
          }
          else if (glucosePointer == 1)
          {
            String trendNow = trendValues.substring(i-10,i-8) + trendValues.substring(i-12,i-10);
            String trendOne = trendValues.substring(190,192) + trendValues.substring(188,190);
            String trendTwo = trendValues.substring(178,180) + trendValues.substring(176,178);
            currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));
            trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL ,16));
            trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL ,16));

            if (FirstRun == 1)
               lastGlucose = currentGlucose;
               
            if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
            {
               if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
                  currentGlucose = trendTwoGlucose;
               else
                  currentGlucose = trendOneGlucose;
            }
          }
          else
          {
            String trendNow = trendValues.substring(i-10,i-8) + trendValues.substring(i-12,i-10);
            String trendOne = trendValues.substring(i-22,i-20) + trendValues.substring(i-24,i-22);
            String trendTwo = trendValues.substring(i-34,i-32) + trendValues.substring(i-36,i-34);
            currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));
            trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL ,16));
            trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL ,16));
            

            if (FirstRun == 1)
               lastGlucose = currentGlucose;
               
            if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
            {
               if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
                  currentGlucose = trendTwoGlucose;
               else
                  currentGlucose = trendOneGlucose;
            }
          }
        }  

        ii++;
      }
     
     for (int i=8, j=0; i<200; i+=12,j++) {
          String t = trendValues.substring(i+2,i+4) + trendValues.substring(i,i+2);
          trend[j] = Glucose_Reading(strtoul(t.c_str(), NULL ,16));
       }

    for (int i=0; i<16; i++)
    {
      if (((lastGlucose - trend[i]) > 50) || ((trend[i] - lastGlucose) > 50)) // invalid trend check
         continue;
      else
      {
         validTrend[validTrendCounter] = trend[i];
         validTrendCounter++;
      }
    }

    if (validTrendCounter > 0)
    { 
      for (int i=0; i < validTrendCounter; i++)
         averageGlucose += validTrend[i];
         
      averageGlucose = averageGlucose / validTrendCounter;
      
      if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
         shownGlucose = averageGlucose; // If currentGlucose is still invalid take the average value
      else
         shownGlucose = currentGlucose; // All went well. Take and show the current value
    }
    else
      shownGlucose = currentGlucose; // If all is going wrong, nevertheless take and show a value 

    if ((lastGlucose == currentGlucose) && (sensorMinutesElapse > 21000)) // Expired sensor check
      noDiffCount++;

    if (lastGlucose != currentGlucose) // Reset the counter
      noDiffCount = 0;

    if (currentGlucose != 0)
      lastGlucose = currentGlucose; 

    NFCReady = 2;
    FirstRun = 0;

#ifdef SEND_TEST_DATA
    static int index = 0, up = 1;
    float sim_bg;
    sim_bg = 103 + (index*7);
    Serial.print(millis());
    Serial.print(F(" - simulate bg reading "));
    Serial.print(sim_bg);
    Serial.print(F(", BatLev: "));
    Serial.print(batteryPcnt);
    Serial.print(F("%, "));
    Serial.print(F("BatMv: "));
    Serial.print(batteryMv);
    Serial.print(F("mV, "));
    Serial.print(F("SensLife: "));
    Serial.print(sensorMinutesElapse);
    Serial.println(F(" min elapsed"));
    if ( up && (index++ < 10) )
    {
      if ( index == 10 ) 
        up = 0;
    }
    else if ( !up && (index-- > 0) ) 
    {
      if ( index == 0 )
        up = 1;
    }
    return(sim_bg);
#else

    // we do support tests with expired sensors!
    noDiffCount = 0;

    Serial.print(millis());
    Serial.print(F(" - bg reading "));
    Serial.print(shownGlucose);
    Serial.print(F(", BatLev: "));
    Serial.print(batteryPcnt);
    Serial.print(F("%, "));
    Serial.print(F("BatMv: "));
    Serial.print(batteryMv);
    Serial.print(F("mV, "));
    Serial.print(F("SensLife: "));
    Serial.print(sensorMinutesElapse);
    Serial.println(F(" min elapsed"));


    if (noDiffCount > 5)
      return 0;
    else  
      return shownGlucose;
#endif    
    }
  else
    {
    Serial.print(F("L - Read Memory Block Command FAIL"));
    NFCReady = 0;
    }
    return(0);  // return added to avoid compiler warning
 }

float Glucose_Reading(unsigned int val) {
        int bitmask = 0x0FFF;
        return ((val & bitmask) / 8.5);
}

int readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  batteryMv = (high<<8) | low;
 
  batteryMv = 1125300L / batteryMv; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  int batteryLevel = min(map(batteryMv, MIN_V, MAX_V, 0, 100), 100); // Convert voltage to percentage
  return batteryLevel;
}

void goToSleep(const byte interval, int time) {
  // say how long we want to sleep in absolute
 Serial.print(millis());
 Serial.print(F(" - go to sleep for "));
 Serial.print((time)*8);
 Serial.println(F("s"));
 delay(100);

 SPI.end();
 digitalWrite(MOSIPin, LOW);
 digitalWrite(SCKPin, LOW);
// digitalWrite(NFCPin1, LOW); // Turn off all power sources completely
 digitalWrite(NFCPin2, LOW); // for maximum power save on BM019.
// digitalWrite(NFCPin3, LOW);
 digitalWrite(IRQPin, LOW);
 digitalWrite(5, LOW); // Disable this for Android 4
 digitalWrite(6, LOW); // Disable this for Android 4
 digitalWrite(BLEPin, LOW); // Disable this for Android 4
 
 for (int i=0; i<time; i++) {
 MCUSR = 0;                         
 WDTCSR |= 0b00011000;           
 WDTCSR =  0b01000000 | interval; 
 set_sleep_mode (SLEEP_MODE_PWR_DOWN);
 sleep_enable();
 sleep_cpu();           
 } 
}
ISR(WDT_vect) 
 {
 wdt_disable(); 
 }
  
void wakeUp(void) {
  sleep_disable();
  power_all_enable();
  wdt_reset();

  digitalWrite(BLEPin, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);
  delay(500);

  // skip data scrup which is still in the line
  waitDoingServices(100, 1);
  // AT+NOTI, HM-1X sends BLE connection status like OK-CONN or OK+LOST, not NULL terminated!
  send_string("AT+NOTI1");
  waitDoingServices(100, 1);
  send_string("AT+RESET");
  waitDoingServices(500, 1);

  Serial.println(F(""));
  print_state(F(" - wake up - wait 40 s for BLE"));
  int i;
  for ( i = 0 ; i < 40 ; i++) {
    waitDoingServices(1000, 1);
  }
#ifdef SHOW_BLE
  Serial.println(F(""));
#endif

  // must be connected here to start communication
  if ( !ble_connected )
    ble_connect_errors++;

  // send error counters to xDrip+ for future use
  ble_Serial.print("ER ");
  ble_Serial.print(nfc_scan_count);
  ble_Serial.print(" ");
  ble_Serial.print(nfc_read_errors);
  ble_Serial.print(" ");
  ble_Serial.print(ble_connect_errors);

//  digitalWrite(NFCPin1, HIGH);
  digitalWrite(NFCPin2, HIGH);
//  digitalWrite(NFCPin3, HIGH);
  digitalWrite(IRQPin, HIGH);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  delay(10);                      
  digitalWrite(IRQPin, LOW);       
  delayMicroseconds(100);         
  digitalWrite(IRQPin, HIGH);      
  delay(10);
  digitalWrite(IRQPin, LOW);
  
  NFCReady = 0;
}

void lowBatterySleep() {
 SPI.end();
 digitalWrite(MOSIPin, LOW);
 digitalWrite(SCKPin, LOW);
// digitalWrite(NFCPin1, LOW); // Turn off all power sources completely
 digitalWrite(NFCPin2, LOW); // for maximum power save on BM019.
// digitalWrite(NFCPin3, LOW);
 digitalWrite(IRQPin, LOW);
 digitalWrite(5, LOW); // Disable this for Android 4
 digitalWrite(6, LOW); // Disable this for Android 4
 digitalWrite(BLEPin, LOW); // Disable this for Android 4

 Serial.print(F("Battery low! LEVEL: "));
 Serial.print(batteryPcnt);
 Serial.println(F("%"));
 delay(100);

 // Switch LED on and then off shortly
    for (int i=0; i<10; i++) {
      digitalWrite(SCKPin, HIGH);
      delay(50);
      digitalWrite(SCKPin, LOW);
      delay(100);
    }
    
 MCUSR = 0;                         
 WDTCSR |= 0b00011000;               
 WDTCSR =  0b01000000 | 0b100001;
 set_sleep_mode (SLEEP_MODE_PWR_DOWN);
 sleep_enable();
 sleep_cpu();           
 sleep_disable();
 power_all_enable();
 wdt_reset(); 
}

/* ****************************************************************** */
/* port of xBridge2 code */
/* ***************************************************************** */

// help function to print a buffer
void print_byte_buf(unsigned char *p, unsigned char len, char s_bracket, char e_bracket, boolean show_hex)
{
  int i, flag = false;

//  checkRam();
  // wait up to 20 chars (@9600)
  delay(20);

  if ( Serial.availableForWrite() < len )
  { 
      flag = true;
      while( Serial.availableForWrite() < len);
  }
  if ( flag )
  {
    delay(500);
    Serial.println(F("Serial_out(): full"));
    delay(500);
  }
  
  Serial.print(s_bracket);
  for ( i = 0 ; i < len ; i++ )
  {
    if ( show_hex )
    {
      Serial.print(p[i], HEX);
      Serial.print(F(" "));
    }
    else
      Serial.write(p[i]);
  }
  Serial.println(e_bracket);
}

// send data to BLE
void send_data(unsigned char *msg, unsigned char len)
{
  unsigned char i = 0;

  for( i = 0; i < len; i++ ) {
    ble_Serial.write(msg[i]);
  }
#ifdef SHOW_BLE
  if(send_debug){
    Serial.print(F("Sending: "));
    print_byte_buf((unsigned char *)msg, len, '<', '>', 1);
    Serial.print(F("Response: "));
  }
#endif
}

// due to Arduino cast problems between String and char * use extra function
void send_string(String cmd)
{
  ble_Serial.print(cmd);
#ifdef SHOW_BLE
  if ( send_debug ) {
    Serial.println(F(""));
    Serial.print(millis());
    Serial.print(F(" ->("));
    Serial.print(cmd);
    Serial.println(F(")"));
  }
#endif
}

// function to send a beacon with the TXID
void sendBeacon(void)
{
  //char array to store the response in.
  unsigned char cmd_response[7];
  //return if we don't have a connection or if we have already sent a beacon
//  if(send_debug)
//    print_state(" - sending beacon Now"));
  //prepare the response
  //responding with number of bytes,
  cmd_response[0] = sizeof(cmd_response);
  //responding to command 01,
  cmd_response[1] = 0xF1;
  //return the encoded TXID
  memcpy(&cmd_response[2], &settings.dex_tx_id, sizeof(settings.dex_tx_id));
  cmd_response[6] = DEXBRIDGE_PROTO_LEVEL;
  send_data(cmd_response, sizeof(cmd_response));
}

int init_command_buff(t_command_buff* pCmd)
{
  if(!pCmd)
    return 0;
  memset(pCmd->commandBuffer, 0, COMMAND_MAXLEN);
  pCmd->nCurReadPos = 0;
  return 0;
}

//decode a command received ??
int commandBuffIs(char* command)
{
  unsigned char len = strlen(command);
  if(len != command_buff.nCurReadPos)
    return(0);
  return( memcmp(command, command_buff.commandBuffer, len)==0 );
}

// return code dictates whether to cancel out of current packet wait operation.
// primarily this is so if you change the start channel, it will actually start using that new start channel
// otherwise there's an infinite wait on when it may fail the initial start channel wait, which if it's interfered with may be forever.
//decode and perform the command recieved on a port
int doCommand()
{
  /* format of commands -  
        char 0 = number of bytes in command, including this one.
        char 1 = the command code.
        char [2 to char n]  = the command data.
    Note:  The convention for command responses is to send the command code back ored with 0xF0.  That way the app knows
    which command the response is for.
  */

//  checkRam();
  
  /* command 0x01 preceeds a TXID being sent by controlling device on UART1.  Packet length is 6 bytes.
    0x01, lsw2, lsw1, msw2, msw1
  */
  if(command_buff.commandBuffer[1] == 0x01 && command_buff.commandBuffer[0] == 0x06)
  {
    print_state(F(" - processing TXID packet"));
    memcpy(&settings.dex_tx_id, &command_buff.commandBuffer[2],sizeof(settings.dex_tx_id));
    // send back the TXID we think we got in response
    return(0);
  }
  /* command 0xF0 is an acknowledgement sent by the controlling device of a data packet.
    This acknowledgement lets us go to sleep immediately.  Packet length is 2 bytes.
    0x02, 0xF0
  */
  if(command_buff.commandBuffer[0] == 0x02 && command_buff.commandBuffer[1] == 0xF0 && !got_ack) {
    got_ack = 1;
    init_command_buff(&command_buff);
    return(0);
  }
  if( commandBuffIs("OK") ) 
  {
    got_ok = 1;;
    return(0);
  }
  // we don't respond to unrecognised commands.
  return(1);
}

// straight algo to detect OK+CONN or OK+LOST in succeeding bytes and set ble_connected accordingly
static int bindex = 0;

int monitor_ble(unsigned char b)
{
//  checkRam();
 
  switch ( bindex )
  {
    case 0:
      if ( b == 'O' )
        bindex++;
      break;
    case 1:
      if ( b == 'K' )
        bindex++;
      break;
    case 2:
      if ( b == '+' )
        bindex++;
      break;
    case 3:
      if ( !ble_connected && ( b == 'C' ) )
        bindex++;
      if ( ble_connected && ( b == 'L' ) )
        bindex++;
      break;
    case 4:
      if ( !ble_connected && ( b == 'O' ) )
        bindex++;
      if ( ble_connected && ( b == 'O' ) )
        bindex++;
      break;
    case 5:
      if ( !ble_connected && ( b == 'N' ) )
        bindex++;
      if ( ble_connected && ( b == 'S' ) )
        bindex++;
      break;
    case 6:
      if ( !ble_connected && ( b == 'N' ) ) {
        bindex = 0;
        ble_connected = 1;
        Serial.print(F("<connect>"));
        return(1);
      }
      if ( ble_connected && ( b == 'T' ) ) {
        bindex = 0;
        ble_connected = 0;
        Serial.print(F("<lost>"));
        return(1);
      }
      break;
    default:
      bindex = 0;
    break;
  }
  return(0);
}

// help function to show the content of the command_buff
void print_command_buff(void)
{
  int i, is_ascii;

  // printable chars begin with ' '
  is_ascii = ( command_buff.commandBuffer[0] >= 0x20 );

  if(send_debug)
  {
    Serial.print(F("<-[")); 
    for ( i = 0 ; i  < command_buff.nCurReadPos ; i++ )
    if ( is_ascii ) {
      Serial.write(command_buff.commandBuffer[i]);
    }
    else {
      Serial.print(F("0x"));
      Serial.print(command_buff.commandBuffer[i], HEX);
      Serial.print(F(" "));
    }
    Serial.println(F("]"));
  }
}

// Process any commands on either UART0 or USB COM.
int controlProtocolService()
{
  static unsigned long cmd_to;
  // ok this is where we check if there's anything happening incoming on the USB COM port or UART 0 port.
  int nRet = 1;
  unsigned char b;
  //if we have timed out waiting for a command, clear the command buffer and return.
  if(command_buff.nCurReadPos > 0 && (millis() - cmd_to) > 2000) 
  {
    Serial.print(F("lost "));
    print_command_buff();
    // clear command buffer if there was anything
    init_command_buff(&command_buff);
    return(nRet);
  } 
  //while we have something in either buffer,
  while( ble_Serial.available() && command_buff.nCurReadPos < COMMAND_MAXLEN) {
    b = ble_Serial.read();
#ifdef SHOW_BLE
    if ( send_debug ) {
      Serial.print(b, HEX);
      Serial.print(F(" "));
    }
#endif
    if ( monitor_ble(b) ) {
      if ( ble_connected ) {
        Serial.println(F(""));
        // ??? if BLE is connected before sending the config OK+NOTI1 are we in trouble?
        // try with MODE2 and AT+START
        print_state(F(" - BLE connected"));
      }
      else {
        Serial.println(F(""));
        print_state(F(" - BLE lost"));
      }
    }
    command_buff.commandBuffer[command_buff.nCurReadPos] = b;
    command_buff.nCurReadPos++;
    cmd_to = millis();
    // if it is the end for the byte string, we need to process the command
    // valid data packet or "OK" received?
    if(command_buff.nCurReadPos == command_buff.commandBuffer[0] || (command_buff.commandBuffer[0] == 'O' && command_buff.commandBuffer[1] == 'K' ))
    {
      // ok we got the end of a command;
      if(command_buff.nCurReadPos) {
        // do the command
#ifdef SHOW_BLE
        if ( send_debug ) // beautifying output
          Serial.println(F(""));
#endif
        nRet = doCommand();
        //re-initialise the command buffer for the next one.
        init_command_buff(&command_buff);
        // break out if we got a breaking command
        if(!nRet)
          return(nRet);
      }
    }
    // otherwise, if the command is not up to the maximum length, add the character to the buffer.
  }
  if ( command_buff.nCurReadPos ){
    //re-initialise the command buffer for the next one.
    init_command_buff(&command_buff);
  }
  return(nRet);
}

// process each of the services we need to be on top of.
// if bWithProtocol is true, also check for commands on both USB and UART
int doServices(unsigned char bWithProtocol)
{
  dex_tx_id_set = (settings.dex_tx_id != 0);
  if(bWithProtocol)
    return(controlProtocolService());
  return(1);
}

//format an array to decode the dexcom transmitter name from a Dexcom packet source address.
char SrcNameTable[32] = { '0', '1', '2', '3', '4', '5', '6', '7',
              '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
              'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P',
              'Q', 'R', 'S', 'T', 'U', 'W', 'X', 'Y' };

char dex_addr[6];

// convert the passed uint32 Dexcom source address into an ascii string in the passed char addr[6] array.
char *dexcom_src_to_ascii(unsigned long src)
{
  //each src value is 5 bits long, and is converted in this way.
  dex_addr[0] = SrcNameTable[(src >> 20) & 0x1F];   //the last character is the src, shifted right 20 places, ANDED with 0x1F
  dex_addr[1] = SrcNameTable[(src >> 15) & 0x1F];   //etc
  dex_addr[2] = SrcNameTable[(src >> 10) & 0x1F];   //etc
  dex_addr[3] = SrcNameTable[(src >> 5) & 0x1F];    //etc
  dex_addr[4] = SrcNameTable[(src >> 0) & 0x1F];    //etc
  dex_addr[5] = 0;  //end the string with a null character.
  return (char *)dex_addr;
}

unsigned long asciiToDexcomSrc(char addr[6])
{
  // prepare a uint32 variable for our return value
  unsigned long src = 0;
  // look up the first character, and shift it 20 bits left.
  src |= (getSrcValue(addr[0]) << 20);
  // look up the second character, and shift it 15 bits left.
  src |= (getSrcValue(addr[1]) << 15);
  // look up the third character, and shift it 10 bits left.
  src |= (getSrcValue(addr[2]) << 10);
  // look up the fourth character, and shift it 50 bits left.
  src |= (getSrcValue(addr[3]) << 5);
  // look up the fifth character
  src |= getSrcValue(addr[4]);
  //printf("asciiToDexcomSrc: val=%u, src=%u\r\n", val, src);
  return src;
}

/* getSrcValue - function to determine the encoding value of a character in a Dexcom Transmitter ID.
Parameters:
srcVal - The character to determine the value of
Returns:
uint32 - The encoding value of the character.
*/
unsigned long getSrcValue(char srcVal)
{
  unsigned char i = 0;
//  checkRam();
  for(i = 0; i < 32; i++)
  {
    if (SrcNameTable[i]==srcVal) break;
  }
  //printf("getSrcVal: %c %u\r\n",srcVal, i);
  return i & 0xFF;
}

// use function instead of macro, use pointer, volatile boolen <var> dont work on Arduino
int waitDoingServicesInterruptible(unsigned long wait_time, volatile boolean *break_flag, unsigned char bProtocolServices)
{
  unsigned long start_wait = millis();
  while ( (millis() - start_wait ) < wait_time ) {
    doServices(bProtocolServices);
    if ( *break_flag ) {
      return(1);      
    }
    delay(20);
  }
  return(0);
}

void waitDoingServices(unsigned long wait_time, unsigned char bProtocolServices)
{
  unsigned long start_wait = millis();
  while ( (millis() - start_wait ) < wait_time )
  {
    doServices(bProtocolServices);
    delay(20);
  }
}

// help function to read the Freestyle Libre sensor
// in LimiTter part of loop(), moved to reuse xBridge code
int get_nfc_reading(unsigned long ms, float *ptr)
{
  // store the current wixel milliseconds so we know how long we are waiting.
  unsigned long start = millis();
  // set the return code to timeout indication, as it is the most likely outcome.
  int nRet = 0;
  float act_glucose;

  // while we haven't reached the delay......
  while (!ms || (millis() - start) < ms ) {
    if (NFCReady == 0) {
      SetProtocol_Command(); // ISO 15693 settings
      delay(100);
      continue;
    }
    else if (NFCReady == 1) {
      for (int i=0; i<3; i++) {
        nfc_scan_count++;
        Inventory_Command(); // sensor in range?
        if (NFCReady == 2)
          break;
        delay(1000);
      }
      if (NFCReady == 1) {
        // count missed readings
        nfc_read_errors++;
        print_state(F(" - NFC timed ot"));
        return(0);
      }
    }
    else if ( NFCReady == 2 ) {
      act_glucose = Read_Memory();
#ifdef SHOW_LIMITTER
      Serial.print(F("\r\nGlucose level: "));
      Serial.println(act_glucose);
      Serial.println(F("15 minutes-trend: "));
      for (int i=0; i<16; i++)
        Serial.println(trend[i]);
      Serial.print(F("Battery level: "));
      Serial.print(batteryPcnt);
      Serial.println(F("%"));
      Serial.print(F("Battery mVolts: "));
      Serial.print(batteryMv);
      Serial.println(F("mV"));
      Serial.print(F("Sensor lifetime: "));
      Serial.print(sensorMinutesElapse);
      Serial.println(F(" minutes elapsed"));
#endif /* SHOW_LIMITTER */      
      // only for showing package content
      *ptr = act_glucose;
      return(1);
    }
  }
  // timeout
  return(nRet);
}

// wait for a NFC reading and put it to Dexcom_packet
// reuse xBridge code even if it more complicate to ensure the same behaviour
int WaitForPacket(unsigned long milliseconds, Dexcom_packet* pPkt)
{
  // store the current wixel milliseconds so we know how long we are waiting.
  unsigned long start = millis();
  // set the return code to timeout indication, as it is the most likely outcome.
  int nRet = 0;
  float glucose;

  print_state(F(" - waiting for NFC reading"));
  // while we haven't reached the delay......
  while (!milliseconds || (millis() - start) < milliseconds)
  {
    //see if anything is required to be done immediately, like process a command on the USB.
    if(!doServices(1) || (millis() - start) > 320000)
      return -1;      // cancel wait, and cancel calling function

    if ( get_nfc_reading(1000, &glucose) ) {
//      print_state(F(" - NFC reading successful done"));
      pPkt->raw = glucose*1000;    // use C casting for conversion of float to unsigned long
      return(1);
    }
  }
  // we timed out waiting for the packet.
  return nRet;
}

//function to format and send the passed Dexom_packet.
void print_packet(Dexcom_packet* pPkt)
{
  RawRecord msg;

  //prepare the message
  msg.size = sizeof(msg);
  msg.cmd_code = 0x00;
  msg.raw = pPkt->raw;
  msg.filtered = pPkt->raw;
  msg.dex_battery = 214;  // simulate good dexcom transmitter battery
  msg.my_battery = batteryPcnt;
  msg.dex_src_id = settings.dex_tx_id;
  msg.delay = abs_millis() - pPkt->ms;
  msg.function = DEXBRIDGE_PROTO_LEVEL; // basic functionality, data packet (with ack), TXID packet, beacon packet (also TXID ack).
  if ( send_debug ) {
    Serial.print(millis());
    Serial.print(F(" - sending packet with a delay of "));
    Serial.print(msg.delay/1000);
    Serial.println(F(" s"));
  }
  send_data( (unsigned char *)&msg, msg.size);
}

/*get_packet -  A function to scan the 4 Dexcom channels.  Waits on channel 0 for ~300s from last packet,
        then waits on channels 1, 2, & 3 for 500ms each.
  Parameters:
    pPkt  - Pointer to a Dexcom_packet.
    
  Uses:
    pkt_time 
        Sets this to the time the packet was aquired.
    WaitForPacket(milliseconds, pkt, channel)
        This function is called to obtain packets.  The return value determines if a packet is
        available to process, or if something else occurred.
        get_packet uses a fixed delay of 25 ms on each channel. This is more than adequate to 
        recieve a packet, which is about 4ms in length.  It cannot be much lower, due to the wixel
        processing speed.  If you go too low, it will cause the wixel to ignore input on the
        USB.  Since we need to set the Transmitter ID, this is not a good thing.
        The Dexcom Tranmsitter sends out a packet in each channel, roughly 500ms apart, every
        5 minutes.  The combination of get_packet and WaitForPacket ensures we see at least
        one of the 4 packets each 5 minutes.  WaitForPacket includes code to exclude duplicate
        packets in each cycle.  We grab the first packet with an ID we didn't see last time, and
        ignore any packets that appear with the same ID.
  Returns:
    0   -   No Packet received, or a command recieved on the USB.
    1   - Packet recieved that passed CRC.
*/
int get_packet(Dexcom_packet* pPkt)
{
  switch( WaitForPacket(3000, pPkt) )
  {
    case 1:     
      // got a packet that passed CRC
        pkt_time = millis();
        pPkt->ms = abs_millis();
        if ( abs_pkt_time != 0 )
          last_abs_pkt_time = abs_pkt_time;
        abs_pkt_time = pPkt->ms;
        return 1;
    case 0:
      // timed out
      return 0 ;
    case -1:
    {
    // cancelled by inbound data on USB (command), or channel invalid.
      return 0;
    }
  }
  return 0;
}

// Configure the BlueTooth module with a name.
// from LimiTTer setp() moved to here
void configBt() {
  // set unique LimiTTer name, max 1 chars
  send_string("AT+NAMELBridge");
  waitDoingServices(100, 1);
  // correct BLE module - answer schould be "HMSoft V540"
  // to detect fake modules which starts with 115200
  send_string("AT+VERR?");
  waitDoingServices(100, 1);
  // notify CONNECT and LOST
  send_string("AT+NOTI1");
  waitDoingServices(100, 1);
  send_string("AT+RESET");
  waitDoingServices(500, 1);
}

//Open the UART and set 9600,n,1 parameters.
// mixied code from LimiTTer and xBridge
// ??? sometimes the HM-11 is not answering here, try to solve later
// ??? ,aybe the error occures when BLE connects before ending the initial AT+ commands
void openUart()
{
  int i;
  if ( settings.uart_baudrate > 230400 ) {
    //detect HM-1x baudrate, if not currently set
    if ( send_debug )
      Serial.println(F("Determining HM-1x baudrate"));
    for( i = 0 ; i <= 8 ; i++ ) {
      init_command_buff(&command_buff);
      if(send_debug)
        Serial.print(F("trying "));
      Serial.println(uart_baudrate[i]);
      settings.uart_baudrate = uart_baudrate[i];
      ble_Serial.begin(uart_baudrate[i]);
      ble_Serial.print(F("AT"));
      waitDoingServicesInterruptible(500,&got_ok,1);
      if(got_ok) break;
    }
    if(!got_ok){
      if(send_debug)
        Serial.println(F("Could not detect baudrate of HM-1x, setting 9600"));
      settings.uart_baudrate=9600;
    }
  }
  ble_Serial.begin(settings.uart_baudrate); // Set saved baudrate
  if ( send_debug ) {
    Serial.print(millis());
    Serial.print(F(" - baudrate set to "));
    Serial.println(settings.uart_baudrate);
  }

  init_command_buff(&command_buff);
}

// help function
void print_state(String str)
{
  if ( send_debug)
  {
    Serial.print(millis());
    Serial.println(str);
  }
}

// LimiTTer code to save battery
void check_battery(void)
{
  // LimiTTer battery OK ?
  batteryPcnt = readVcc();
  if (batteryPcnt < 1)
    batteryLow = 1;
  while (batteryLow == 1)
  {
    lowBatterySleep();
    batteryPcnt = readVcc();
    if (batteryPcnt > 10)
    {
      batteryLow = 0;
      wakeUp();
      delay(100);
    }
  }
}

// main processing loop
void loop(void)
{   
  unsigned long var_sleepTime;

  send_debug = 1;     // we want to see debug
  prg_run_time = 0;   // initialise program run time in s

  // init settings
  // force to get a TXID from xDrip+
  settings.dex_tx_id = 0xFFFFFFFF;      //asciiToDexcomSrc("ABCDE");
  settings.uart_baudrate = 0xFFFFFFFF;  // 9600L;
   
  //initialise the command buffer
  init_command_buff(&command_buff);

  loop_start_time = millis();

  Serial.println(F("\r\n*********************************"));
  Serial.println(F("*** LBridge starting ***"));
  Serial.print(F("mem available: "));
  Serial.println(freeMemory());
  Serial.println(F("*********************************"));

  // give HM-1x time to settle
  delay(2000);
  // Open the UART and set it up for comms to HM-1x
  openUart();

  //configure the bluetooth module, power on was done in setup()
  configBt();

  // initialize the LimiTTer NFC module
  configNFC();

  // wait for a BLE connection to xDrip to get a TXID if needed
  Serial.println(F(""));
  print_state(F(" - initial wake up, waiting up to 40s for BLE"));
  waitDoingServicesInterruptible(40000, &ble_connected, 1);

  // read the flash stored value of our TXID.
  if(settings.dex_tx_id >= 0xFFFFFFFF) 
    settings.dex_tx_id = 0;

  // if dex_tx_id is zero, we do not have an ID to filter on.  So, we keep sending a beacon every 5 seconds until it is set.
  // Comment out this while loop if you wish to use promiscuous mode and receive all Dexcom tx packets from any source (inadvisable).
  // Promiscuous mode is allowed in waitForPacket() function (dex_tx_id == 0, will match any dexcom packet).  Just don't send the 
  // wixel a TXID packet.
  print_state(F(" - testing TXID"));

  // instead of save current TXID to EEPROM get it every time from xDrip using the Beacon mechanism
  while(settings.dex_tx_id == 0) {
    print_state(F(" - no TXID. Sending beacon."));
    // wait until we have a BLE connection
    while(!ble_connected) doServices(1);
    //send a beacon packet
    sendBeacon();
    // set flag dex_tx_id_set 
    doServices(0);
    //wait 5 seconds, beatify output
    if ( !waitDoingServicesInterruptible(5000, &dex_tx_id_set, 1) )
      Serial.println(F(""));
  }

  if ( send_debug ){
    Serial.print(millis()); Serial.print(F(" - new TXID is ")); Serial.print(settings.dex_tx_id, HEX); Serial.print(F(" ("));
    // and show first 5 char from the TXID setting in xDrips menue
    Serial.print(dexcom_src_to_ascii(settings.dex_tx_id)); Serial.println(F(")"));
  }

  // initialize to empty queue
  Pkts.read = 0;
  Pkts.write = 0;

  print_state(F(" - entering main loop."));

  while (1)
  {
    // check for low battery and go to sleep here if wrong
    check_battery();
    // wait 3 sec for NFC reading
    // if timeout = 0 or no sensor found go to sleep
    if( get_packet(&Pkts.buffer[Pkts.write]) ) {
//      if ( send_debug ) {
//        Serial.print(millis());
//        Serial.print(F(" - got packet, stored at position "));
//        Serial.println(Pkts.write);
//      }
      // so increment write position for next round...
      Pkts.write = (Pkts.write + 1) & (DXQUEUESIZE-1);
      if (Pkts.read == Pkts.write)
        Pkts.read = (Pkts.read + 1) & (DXQUEUESIZE-1); //overflow in ringbuffer, overwriting oldest entry, thus move read one up
      do_sleep = 1; // we got a packet, so we are aligned with the 5 minute interval - so go to sleep after sending out packets
    } 
    else {
      if ( send_debug ) {
        Serial.print(millis());
        Serial.print(F(" - did not receive a pkt with "));
        Serial.print(Pkts.write-Pkts.read);
        Serial.println(F(" pkts in queue"));
      }
      if ( ble_connected ) 
        sendBeacon();
      do_sleep = 1; // no NFC reading, go to sleep to save battery
    }

    //TODO: what happens if we did not receive a packet? pkt_time is still set to the last one - this will make the following checks fail...
    if (Pkts.read != Pkts.write) { // if we have a packet
      // we wait up to one minute for BLE connect
      while (!ble_connected && ((millis() - pkt_time) < 40000)) {
        if ( send_debug ) {
          Serial.print(millis());
          Serial.println(F(" - packet waiting for ble connect"));
        }
        waitDoingServicesInterruptible(10000, &ble_connected, 1);
      }

      // we got a connection, so send pending packets now - at most for two minutes after the last packet was received
      while ((Pkts.read != Pkts.write) && ble_connected && ((millis() - pkt_time) < 120000)) {
//        print_state(F(" - sending packet"));
        got_ack = 0;
        print_packet(&Pkts.buffer[Pkts.read]);
//        Serial.print(millis()); Serial.print(F(" - last packet sended "));
//        Serial.print(abs_millis() - last_abs_pkt_time);
//        Serial.println(F(" ms before"));
        waitDoingServicesInterruptible(10000, &got_ack, 1);
        if (got_ack) {
          if ( send_debug ) {
            Serial.print(millis());
            Serial.print(F(" - got ack for read position "));
            Serial.print(Pkts.read);
            Serial.print(F(" while write is "));
            Serial.print(Pkts.write);
            Serial.println(F(", incrementing read"));
          }
          Pkts.read = (Pkts.read + 1) & (DXQUEUESIZE-1); //increment read position since we got an ack for the last package
        }
      }
    }

    // can't safely sleep if we didn't get an ACK, or if we are already sleeping!
    if ( do_sleep )
    {
      dly_ms=millis();
      while((millis() - dly_ms) <= 500) {
        // allow the wixel to complete any other tasks.
        doServices(1);
      }
      // empty serial out buffer
      waitDoingServices(1000, 1);

      loop_time = millis() - loop_start_time; 
      // ensure to use 300 s for a full cycle of loop() and sleeping
      var_sleepTime = (((300000 - loop_time)/1000)+4) / 8;

//      Serial.print(F("loop time ")); Serial.print(loop_time); Serial.print(F(" ms, sleep for ")); 
//      Serial.print(var_sleepTime*8); Serial.print(F("s"));
//      Serial.print(F(", complete loop is ")); Serial.println((loop_time+var_sleepTime*8000)/1000);

      if ( sleepTime == 4 )
        goToSleep (0b100001, 4);    // for test, remove before release!
      else
        goToSleep (0b100001, var_sleepTime);    // mask for 8s, * sleepTime
      
      // waking up, power on BLE, initialze NFC
      // count programm run time up
      prg_run_time += (loop_time + var_sleepTime*8000)/1000;
      // set start point for loop 
      loop_start_time = millis();
      // clear do_sleep, cause we have just woken up.
      do_sleep = 0;
      // no BLE connection
      ble_connected = 0;  // this is done now in wakeUp()
      got_packet = 0;
      init_command_buff(&command_buff);
      Serial.println(F("******************************************************************"));
      Serial.print(F("loop ")); Serial.print(++loop_count);
      Serial.print(F(", BLE wake up errors ")); Serial.print(ble_connect_errors);
      Serial.print(F(", NFC read errors ")); Serial.print(nfc_read_errors);
      Serial.print(F(", run time ")); Serial.print(prg_run_time); Serial.println(F(""));
      wakeUp();
      waitDoingServices(250,1);
    }
  }
}


