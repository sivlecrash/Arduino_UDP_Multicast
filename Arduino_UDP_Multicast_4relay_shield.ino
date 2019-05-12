#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
byte mac[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
IPAddress ip(192, 168, 182, 232);
unsigned int localPort = 3671;
byte packetBuffer[UDP_TX_PACKET_MAX_SIZE];
byte packetBufferWrite[30]; // buffer packets
EthernetUDP Udp;
IPAddress ipMulti(224, 0, 23, 12);
unsigned int portMulti = 3671;
long citac = 0;

//Individual Address - 1.1.1 Area [4 bit] . Line [4 bit] . Bus device [1 byte]
uint8_t UDParea = 10;
uint8_t UDPline = 10;
uint8_t UDPdevice = 1;

const int relayPin01 = 4;
const int relayPin02 = 5;
const int relayPin03 = 6;
const int relayPin04 = 7;
boolean relayPin01state = 0;
boolean relayPin02state = 0;
boolean relayPin03state = 0;
boolean relayPin04state = 0;

void setup() {

  pinMode(relayPin01, OUTPUT);
  pinMode(relayPin02, OUTPUT);
  pinMode(relayPin03, OUTPUT);
  pinMode(relayPin04, OUTPUT);
  digitalWrite(relayPin01, LOW); // Off
  digitalWrite(relayPin02, LOW); // Off
  digitalWrite(relayPin03, LOW); // Off
  digitalWrite(relayPin04, LOW); // Off
  
  packetBufferWrite[0] = 6;     //ok  6[HEX]
  packetBufferWrite[1] = 16;    //ok 10[HEX] - Protocol version (constant) [1byte]
  packetBufferWrite[2] = 5;     //ok  5[HEX] - Service Type ID [2byte]
  packetBufferWrite[3] = 48;    //ok 30[HEX] - Service Type ID [2byte]
  packetBufferWrite[4] = 0;     //ok  0[HEX] - UDPnet/IP action [8bit+]
  packetBufferWrite[5] = 17;    //   11[HEX] UDPnet/IP action [+4bit], Total length [4bit]
  packetBufferWrite[6] = 41;    //ok 29[HEX] - Message Code [1byte]
  packetBufferWrite[7] = 0;     //ok  0[HEX] - Additional info [1byte...]
  packetBufferWrite[8] = 188;;  //ok BC[HEX] - Frame [1bit], Reserved [1bit], Repeat [1bit], Broadcast [1bit], Priority [2bit], ACK [1bit], error [1bit]
  packetBufferWrite[9] = 224;   //ok E0[HEX] - Type Destination [1bit], Routing [3bit], Ext. Frame Format [4bit]
  UDParea <<= 4;
  packetBufferWrite[10] = UDParea | UDPline; // 1/1/1 Individual Address - Area [4 bit] . Line [4 bit] . Bus device [1 byte]
  packetBufferWrite[11] = UDPdevice;         // 1/1/1 Individual Address - Area [4 bit] . Line [4 bit] . Bus device [1 byte]
  packetBufferWrite[12] = 0;    //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
  packetBufferWrite[13] = 1;    //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
  packetBufferWrite[14] = 1;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
  packetBufferWrite[15] = 0;    //ok 0[HEX] TPCI [6 bit], APCI(Type) [2 bit+]
  packetBufferWrite[16] = 0;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
  packetBufferWrite[17] = 0;    //Data
  packetBufferWrite[18] = 0;    //Data
  packetBufferWrite[19] = 0;    //Data
  packetBufferWrite[20] = 0;    //Data
  packetBufferWrite[21] = 0;    //Data
  packetBufferWrite[22] = 0;    //Data
  packetBufferWrite[23] = 0;    //Data
  packetBufferWrite[24] = 0;    //Data
  packetBufferWrite[25] = 0;    //Data
  packetBufferWrite[26] = 0;    //Data
  packetBufferWrite[27] = 0;    //Data
  packetBufferWrite[28] = 0;    //Data
  packetBufferWrite[29] = 0;    //Data
  packetBufferWrite[30] = 0;    //Data

  Ethernet.begin(mac, ip);
  Udp.beginMulticast(ipMulti, portMulti);
  Serial.begin(9600);
  Serial.print("IP: ");
  Serial.println(ip);
  Serial.print("IP Multicast: ");
  Serial.print(ipMulti);
  Serial.print(" : ");
  Serial.println(portMulti);
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);

    //Protocol version [1byte]
    uint8_t protocol = packetBuffer[1];

    //Service Type ID [2byte]
    uint16_t stid = 0;
    stid = packetBuffer[2];
    stid <<= 8;
    stid = stid | packetBuffer[3];

    //UDPnet/IP action [12bit], Total length [4bit]
    uint16_t ipaction = 0;
    uint8_t ipaction2 = packetBuffer[5];
    uint8_t tlength = packetBuffer[5];
    ipaction2 >>= 4;
    ipaction2 <<= 4;
    ipaction = packetBuffer[4];
    ipaction <<= 8;
    ipaction = ipaction | ipaction2;
    tlength <<= 4;
    tlength >>= 4;

    //Message Code [1byte]
    uint8_t mcode = packetBuffer[6];

    //Additional info [1byte...]
    uint8_t ainfo = packetBuffer[7];

    //Control Field - Frame [1bit], Reserved [1bit], Repeat [1bit], Broadcast [1bit], Priority [2bit], ACK [1bit], error [1bit]
    uint8_t frame = packetBuffer[8];
    uint8_t reserved = packetBuffer[8];
    uint8_t repeat = packetBuffer[8];
    uint8_t broadcast = packetBuffer[8];
    uint8_t priority = packetBuffer[8];
    uint8_t ack = packetBuffer[8];
    uint8_t error = packetBuffer[8];
    frame >>= 7;
    reserved <<= 1;
    reserved >>= 7;
    repeat <<= 2;
    repeat >>= 7;
    broadcast <<= 3;
    broadcast >>= 7;
    priority <<= 4;
    priority >>= 6;
    ack <<= 6;
    ack >>= 7;
    error <<= 7;
    error >>= 7;

    //Control Field -  Type Destination [1bit], Routing [3bit], Ext. Frame Format [4bit]
    uint8_t destination = packetBuffer[9];
    uint8_t rout = packetBuffer[9];
    uint8_t efformat = packetBuffer[9];
    destination >>= 7;
    rout <<= 1;
    rout >>= 5;
    efformat <<= 4;
    efformat >>= 4;

    //Individual Address - Area [4 bit] . Line [4 bit] . Bus device [1 byte]
    uint8_t iaarea = packetBuffer[10];
    uint8_t ialine = packetBuffer[10];
    uint8_t iadevice = packetBuffer[11];
    iaarea >>= 4;
    ialine <<= 4;
    ialine >>= 4;

    //Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
    uint8_t gamain = packetBuffer[12];
    uint8_t gamiddle = packetBuffer[12];
    uint8_t gasub = packetBuffer[13];
    gamain >>= 3;
    gamiddle <<= 5;
    gamiddle >>= 5;

    //AT [1 bit], NPCI [3 bit], Length [4 bit]
    uint8_t at = packetBuffer[14];
    uint8_t npci = packetBuffer[14];
    uint8_t length2 = packetBuffer[14];
    at >>= 7;
    npci <<= 1;
    npci >>= 5;
    length2 <<= 4;
    length2 >>= 4;

    //TPCI [6 bit], APCI(Type) [4 bit], DATA/APCI [6 bit]
    uint8_t tpci = packetBuffer[15];
    uint8_t apci = 0;
    uint8_t apci1 = packetBuffer[15];
    uint8_t apci2 = packetBuffer[16];
    uint8_t data16 = packetBuffer[16];
    tpci >>= 2;
    apci1 <<= 6;
    apci1 >>= 4;
    apci2 >>= 6;
    apci = apci1 | apci2;
    data16 <<= 2;
    data16 >>= 2;

    //Data 1byte - 14byte
    uint8_t data17 = packetBuffer[17];
    uint8_t data18 = packetBuffer[18];
    uint8_t data19 = packetBuffer[19];
    uint8_t data20 = packetBuffer[20];
    uint8_t data21 = packetBuffer[21];
    uint8_t data22 = packetBuffer[22];
    uint8_t data23 = packetBuffer[23];
    uint8_t data24 = packetBuffer[24];
    uint8_t data25 = packetBuffer[25];
    uint8_t data26 = packetBuffer[26];
    uint8_t data27 = packetBuffer[27];
    uint8_t data28 = packetBuffer[28];
    uint8_t data29 = packetBuffer[29];
    uint8_t data30 = packetBuffer[30];

    //--------------relayPin01----------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 1) && (gamiddle == 0) && (gasub == 1)) {  // if "1bit" + "Read" + "1/0/1"
      Serial.print("1/0/1 - relayPin01 - Read - ");
      Serial.println(relayPin01state);
      uint8_t UDPipaction2 = 1;
      uint8_t UDPdpt = 1; // 1=1bit
      UDPipaction2 <<= 4;
      packetBufferWrite[5] = UDPipaction2 | UDPdpt;    // 11 UDPnet/IP action [+4bit], Total length [4bit]
      uint8_t UDPgaMain = 1;
      uint8_t UDPgaMiddle = 0;
      uint8_t UDPgaSub = 1;
      UDPgaMain <<= 3;
      packetBufferWrite[12] = UDPgaMain | UDPgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = UDPgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      uint8_t UDPat = 0; //0[HEX]
      uint8_t UDPnpci = 0; //0[HEX]
      UDPat <<= 7;
      UDPnpci <<= 6;
      packetBufferWrite[14] = UDPat | UDPnpci | UDPdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      uint8_t UDPapci2 = 1;
      UDPapci2 <<= 6;
      packetBufferWrite[16] = UDPapci2 | relayPin01state;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 17);
      Udp.endPacket();
    }
    if  ((tlength == 1) && (apci == 2) && (gamain == 1) && (gamiddle == 0) && (gasub == 1)) {  // if "1bit" + "Write" + "1/0/1"
      Serial.print("1/0/1 - relayPin01 - ");
      if (data16 == 0) {
        digitalWrite(relayPin01, LOW);
        relayPin01state = false;
        Serial.println("Off");
      }
      else if (data16 == 1) {
        digitalWrite(relayPin01, HIGH);
        relayPin01state = true;
        Serial.println("On");
      }
      else {
        Serial.println("?? ");
      }
    }
    //--------------relayPin02----------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 1) && (gamiddle == 0) && (gasub == 2)) {  // if "1bit" + "Read" + "1/0/2"
      Serial.print("1/0/2 - relayPin02 - Read - ");
      Serial.println(relayPin02state);
      uint8_t UDPipaction2 = 1;
      uint8_t UDPdpt = 1; // 1=1bit
      UDPipaction2 <<= 4;
      packetBufferWrite[5] = UDPipaction2 | UDPdpt;    // 11 UDPnet/IP action [+4bit], Total length [4bit]
      uint8_t UDPgaMain = 1;
      uint8_t UDPgaMiddle = 0;
      uint8_t UDPgaSub = 2;
      UDPgaMain <<= 3;
      packetBufferWrite[12] = UDPgaMain | UDPgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = UDPgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      uint8_t UDPat = 0; //0[HEX]
      uint8_t UDPnpci = 0; //0[HEX]
      UDPat <<= 7;
      UDPnpci <<= 6;
      packetBufferWrite[14] = UDPat | UDPnpci | UDPdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      uint8_t UDPapci2 = 1;
      UDPapci2 <<= 6;
      packetBufferWrite[16] = UDPapci2 | relayPin02state;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 17);
      Udp.endPacket();
    }
    if  ((tlength == 1) && (apci == 2) && (gamain == 1) && (gamiddle == 0) && (gasub == 2)) {  // if "1bit" + "Write" + "1/0/2"
      Serial.print("1/0/2 - relayPin02 - ");
      if (data16 == 0) {
        digitalWrite(relayPin02, LOW);
        relayPin02state = false;
        Serial.println("Off");
      }
      else if (data16 == 1) {
        digitalWrite(relayPin02, HIGH);
        relayPin02state = true;
        Serial.println("On");
      }
      else {
        Serial.println("?? ");
      }
    }
    //--------------relayPin03----------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 1) && (gamiddle == 0) && (gasub == 3)) {  // if "1bit" + "Read" + "1/0/3"
      Serial.print("1/0/3 - relayPin03 - Read - ");
      Serial.println(relayPin03state);
      uint8_t UDPipaction2 = 1;
      uint8_t UDPdpt = 1; // 1=1bit
      UDPipaction2 <<= 4;
      packetBufferWrite[5] = UDPipaction2 | UDPdpt;    // 11 UDPnet/IP action [+4bit], Total length [4bit]
      uint8_t UDPgaMain = 1;
      uint8_t UDPgaMiddle = 0;
      uint8_t UDPgaSub = 3;
      UDPgaMain <<= 3;
      packetBufferWrite[12] = UDPgaMain | UDPgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = UDPgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      uint8_t UDPat = 0; //0[HEX]
      uint8_t UDPnpci = 0; //0[HEX]
      UDPat <<= 7;
      UDPnpci <<= 6;
      packetBufferWrite[14] = UDPat | UDPnpci | UDPdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      uint8_t UDPapci2 = 1;
      UDPapci2 <<= 6;
      packetBufferWrite[16] = UDPapci2 | relayPin03state;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 17);
      Udp.endPacket();
    }
    if  ((tlength == 1) && (apci == 2) && (gamain == 1) && (gamiddle == 0) && (gasub == 3)) {  // if "1bit" + "Write" + "1/0/3"
      Serial.print("1/0/3 - relayPin03 - ");
      if (data16 == 0) {
        digitalWrite(relayPin03, LOW);
        relayPin03state = false;
        Serial.println("Off");
      }
      else if (data16 == 1) {
        digitalWrite(relayPin03, HIGH);
        relayPin03state = true;
        Serial.println("On");
      }
      else {
        Serial.println("?? ");
      }
    }
    //--------------relayPin04----------------//
    if  ((tlength == 1) && (apci == 0) && (gamain == 1) && (gamiddle == 0) && (gasub == 4)) {  // if "1bit" + "Read" + "1/0/4"
      Serial.print("1/0/4 - relayPin04 - Read - ");
      Serial.println(relayPin04state);
      uint8_t UDPipaction2 = 1;
      uint8_t UDPdpt = 1; // 1=1bit
      UDPipaction2 <<= 4;
      packetBufferWrite[5] = UDPipaction2 | UDPdpt;    // 11 UDPnet/IP action [+4bit], Total length [4bit]
      uint8_t UDPgaMain = 1;
      uint8_t UDPgaMiddle = 0;
      uint8_t UDPgaSub = 4;
      UDPgaMain <<= 3;
      packetBufferWrite[12] = UDPgaMain | UDPgaMiddle;   //0[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      packetBufferWrite[13] = UDPgaSub;                  //1[HEX] Group Address - Main [5 bit] / Middle [3 bit] / Sub [1 byte]
      uint8_t UDPat = 0; //0[HEX]
      uint8_t UDPnpci = 0; //0[HEX]
      UDPat <<= 7;
      UDPnpci <<= 6;
      packetBufferWrite[14] = UDPat | UDPnpci | UDPdpt;    //1[HEX] AT [1bit], NPCI [3bit], Length(DPT) [4bit]
      uint8_t UDPapci2 = 1;
      UDPapci2 <<= 6;
      packetBufferWrite[16] = UDPapci2 | relayPin04state;  //80 APCI(Type) [+2 bit], DATA/APCI [6 bit]
      Udp.beginPacket(ipMulti, portMulti);
      Udp.write(packetBufferWrite, 17);
      Udp.endPacket();
    }
    if  ((tlength == 1) && (apci == 2) && (gamain == 1) && (gamiddle == 0) && (gasub == 4)) {  // if "1bit" + "Write" + "1/0/4"
      Serial.print("1/0/4 - relayPin04 - ");
      if (data16 == 0) {
        digitalWrite(relayPin04, LOW);
        relayPin04state = false;
        Serial.println("Off");
      }
      else if (data16 == 1) {
        digitalWrite(relayPin04, HIGH);
        relayPin04state = true;
        Serial.println("On");
      }
      else {
        Serial.println("?? ");
      }
    }
    //--------------CENTRAL----------------//
    if  ((tlength == 1) && (apci == 2) && (gamain == 0) && (gamiddle == 0) && (gasub == 1)) {  // if "1bit" + "Write" + "0/0/1"
      Serial.print("0/0/1 CENTRAL - ");
      if (data16 == 0) {
        digitalWrite(relayPin01, LOW);
        digitalWrite(relayPin02, LOW);
        digitalWrite(relayPin03, LOW);
        digitalWrite(relayPin04, LOW);
        relayPin01state = false;
        relayPin02state = false;
        relayPin03state = false;
        relayPin04state = false;
        Serial.println("Off");
      }
      else if (data16 == 1) {
        digitalWrite(relayPin01, HIGH);
        digitalWrite(relayPin02, HIGH);
        digitalWrite(relayPin03, HIGH);
        digitalWrite(relayPin04, HIGH);
        relayPin01state = true;
        relayPin02state = true;
        relayPin03state = true;
        relayPin04state = true;
        Serial.println("On");
      }
      else {
        Serial.println("?? ");
      }
    }
      Serial.println(" ");
  byte packetBuffer[] = {000000000000000000000000000000000000000000000000000000000000}; //reset buffer
   }
}
