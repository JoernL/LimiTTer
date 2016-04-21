/* LimiTTer sketch for the Arduino UNO/Pro-Mini.
   It scans the Freestyle Libre Sensor every 5 minutes
   and sends the data to the xDrip Android app. You can
   see the data with an UNO on the serial monitor, too.
   If you want another scan interval, simply change the
   sleepTime value.  
   For maximum power save you can try to enable all lines
   with the comment "BLE OFF" and set the sleep time to 22.
   For BLE OFF, VCC pin of HM11 module must be connected with
   pin 3 on the Arduino. This doesn't work stable with
   Android 4.4. But with Android > 5.x you can give it a try.

   This sketch is based on a sample sketch for the BM019 module
   from Solutions Cubed.

   Wiring for UNO / Pro-Mini:

   Arduino          BM019           BLE-HM11
   IRQ: Pin 9       DIN: pin 2
   SS: pin 10       SS: pin 3
   MOSI: pin 11     MOSI: pin 5 
   MISO: pin 12     MISO: pin4
   SCK: pin 13      SCK: pin 6
   VCC (pin 3 for BLE OFF)          VCC: pin 9 
   I/O: pin 5                       TX:  pin 2
   I/O: pin 6                       RX:  pin 4
*/

#include <SPI.h>
#include <SoftwareSerial.h>
#include <avr/sleep.h> 
#include <avr/power.h>
#include <avr/wdt.h>

const int SSPin = 10;  // Slave Select pin
const int IRQPin = 9;  // Sends wake-up pulse for BM019
const int NFCPin1 = 7; // Power pin BM019
const int NFCPin2 = 8; // Power pin BM019
const int NFCPin3 = 4; // Power pin BM019
//const int BLEPin = 3; // BLE power pin. Enable this for BLE OFF
const int MOSIPin = 11;
const int SCKPin = 13;
byte RXBuffer[24];
byte NFCReady = 0;  // used to track NFC state
int sleepTime = 36; // SleepTime ( 36 ~ 5min) Set this to 22 for BLE OFF

SoftwareSerial ble_Serial(5, 6); // RX | TX

void setup() {
    pinMode(IRQPin, OUTPUT);
    digitalWrite(IRQPin, HIGH); 
    pinMode(SSPin, OUTPUT);
    digitalWrite(SSPin, HIGH);
    pinMode(NFCPin1, OUTPUT);
    digitalWrite(NFCPin1, HIGH);
    pinMode(NFCPin2, OUTPUT);
    digitalWrite(NFCPin2, HIGH);
    pinMode(NFCPin3, OUTPUT);
    digitalWrite(NFCPin3, HIGH);
    //pinMode(BLEPin, OUTPUT); //Enable this for BLE OFF
    //digitalWrite(BLEPin, HIGH); //Enable this for BLE OFF
    pinMode(MOSIPin, OUTPUT);
    pinMode(SCKPin, OUTPUT);
    
    Serial.begin(9600);
    ble_Serial.begin(115200); // Change value to your BLE module rate 
    ble_Serial.write("AT+NAME");
    ble_Serial.write("LimiTTer");
    delay(500);
    ble_Serial.write("AT+PWRM0");
    ble_Serial.write("AT+RESET");
    delay(500);
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
    Serial.println("Protocol Set Command OK");
    NFCReady = 1; // NFC is ready
    }
  else
    {
    Serial.println("Protocol Set Command FAIL");
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
    Serial.println("Sensor in range ... OK");
    NFCReady = 2;
    }
  else
    {
    Serial.println("Sensor out of range");
    NFCReady = 1;
    }
 }
 
float Read_Memory() {

 byte oneBlock[8];
 String hexPointer = "";
 String trendValues = "";
 float currentGlucose;
 int glucosePointer;
  
 for ( int b = 3; b < 40; b++) {
 
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
  Serial.println(str);
  trendValues += str;
    
 }   
  if (RXBuffer[0] == 128) // is response code good?
    {
      hexPointer = trendValues.substring(4,6);
      glucosePointer = strtoul(hexPointer.c_str(), NULL, 16);
      Serial.println();
      Serial.print("Glucose pointer: ");
      Serial.print(glucosePointer);
      Serial.println();
            
      int ii = 0;
      for (int i=8; i<=200; i+=12) {
        if (glucosePointer == ii)
        {
          String g = trendValues.substring(i+2,i+4) + trendValues.substring(i,i+2);
          currentGlucose = Glucose_Reading(strtoul(g.c_str(), NULL ,16));
        }

        ii++;
      }
            
    NFCReady = 2;
    return currentGlucose;
    
    }
  else
    {
    Serial.print("Read Memory Block Command FAIL");
    NFCReady = 0;
    }
 }

float Glucose_Reading(unsigned int val) {
        int bitmask = 0x0FFF;
        return ((val & bitmask) / 8);
}

String Build_Packet(float glucose) {
  
// Let's build a String which xDrip accepts as a BTWixel packet

      String packet = ""; 
      unsigned long raw = glucose*733; // raw_value 
      packet = String(raw);
      packet += ' ';
      packet += String(216); // fake value
      packet += ' ';
      packet += String(100); // fake value
      Serial.println();
      Serial.print("Glucose level: ");
      Serial.print(glucose);
      Serial.println();
      return packet;
}

void Send_Packet(String packet) {
   
    if ((packet.substring(0,1) != "0"))
    {
      Serial.println();
      Serial.print("xDrip packet: ");
      Serial.print(packet);
      Serial.println();    
      ble_Serial.print(packet);
      delay(100);
    }
  }

void goToSleep(const byte interval, int time) {
 
 SPI.end();
 digitalWrite(MOSIPin, LOW);
 digitalWrite(SCKPin, LOW);
 digitalWrite(NFCPin1, LOW); // Turn off all power sources completely
 digitalWrite(NFCPin2, LOW); // for maximum power save on BM019.
 digitalWrite(NFCPin3, LOW);
 digitalWrite(IRQPin, LOW);
 //digitalWrite(BLEPin, LOW); // Enable this for BLE OFF
 //digitalWrite(5, LOW); // Enable this for BLE OFF
 //digitalWrite(6, LOW); // Enable this for BLE OFF
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
  
void wakeUp() {
    
    sleep_disable();
    power_all_enable();
    wdt_reset();
    //digitalWrite(BLEPin, HIGH); // Enable this for BLE OFF
    //digitalWrite(5, HIGH); // Enable this for BLE OFF
    //digitalWrite(6, HIGH); // Enable this for BLE OFF
    //delay(60000); //  Enable this for BLE OFF. Reconnect time
    digitalWrite(NFCPin1, HIGH);
    digitalWrite(NFCPin2, HIGH);
    digitalWrite(NFCPin3, HIGH);
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
void loop() {

  if (NFCReady == 0)
  {
    SetProtocol_Command(); // ISO 15693 settings
    delay(100);
  }
      
  else if (NFCReady == 1)
  {
    for (int i=0; i<2; i++) {
    Inventory_Command(); // sensor in range?
    if (NFCReady == 2)
      break;
    delay(1000);
    }
    if (NFCReady == 1) {
    goToSleep (0b100001, sleepTime);
    wakeUp();
    delay(100); 
    }
  }
     
  else
  {
    String xdripPacket = Build_Packet(Read_Memory());
    Send_Packet(xdripPacket);
    goToSleep (0b100001, sleepTime);
    wakeUp();
    delay(100);    
  }
  
}

