#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

byte mac[] = {
  0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA
};
IPAddress ip(192, 168, 182, 232);
unsigned int localPort = 3671;      // local port to listen on

// buffers for receiving and sending data
byte packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged";       // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

IPAddress ipMulti(224, 0, 23, 12);
unsigned int portMulti = 3671;
long citac = 0;

void setup() {
  // start the Ethernet and UDP:
  Ethernet.begin(mac, ip);
  //static uint8_t socketBeginMulticast(uint8_t protocol, IPAddress ip,uint16_t port);
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

    Serial.print(packetBuffer[0], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[1], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[2], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[3], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[4], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[5], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[6], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[7], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[8], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[9], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[10], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[11], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[12], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[13], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[14], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[15], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[16], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[17], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[18], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[19], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[20], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[21], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[22], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[23], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[24], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[25], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[26], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[27], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[28], HEX);
    Serial.print(" , ");
    Serial.print(packetBuffer[29], HEX);
    Serial.print(" , ");
    Serial.println(packetBuffer[30], HEX);

    Serial.print(packetBuffer[0], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[1], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[2], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[3], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[4], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[5], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[6], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[7], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[8], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[9], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[10], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[11], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[12], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[13], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[14], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[15], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[16], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[17], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[18], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[19], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[20], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[21], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[22], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[23], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[24], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[25], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[26], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[27], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[28], BIN);
    Serial.print(" , ");
    Serial.print(packetBuffer[29], BIN);
    Serial.print(" , ");
    Serial.println(packetBuffer[30], BIN);

    //Protocol version [1byte]
    uint8_t protocol = packetBuffer[1];

    //Service Type ID [2byte]
    uint16_t stid = 0;
    stid = packetBuffer[2];
    stid <<= 8;
    stid = stid | packetBuffer[3];

    //KNXnet/IP action [12bit], Total length [4bit]
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

    Serial.print("Protocol version: ");
    Serial.print(protocol, HEX);
    Serial.print("  Service Type ID: ");
    Serial.print(stid, HEX);
    Serial.print("  KNXnet/IP action: ");
    Serial.print(ipaction, HEX);
    Serial.print("  Total length: ");
    Serial.print(tlength, HEX);
    Serial.print("  Message Code: ");
    Serial.print(mcode, HEX);
    Serial.print("  Additional info: ");
    Serial.print(ainfo, HEX);
    Serial.print("  Frame: ");
    Serial.print(frame, HEX);
    Serial.print("  Reserved: ");
    Serial.print(reserved, HEX);
    Serial.print("  Repeat: ");
    Serial.print(repeat, HEX);
    Serial.print("  Broadcast: ");
    Serial.print(broadcast, HEX);
    Serial.print("  Priority: ");
    Serial.print(priority, HEX);
    Serial.print("  ACK: ");
    Serial.print(ack, HEX);
    Serial.print("  error: ");
    Serial.print(error, HEX);
    Serial.print("  Type destination: ");
    Serial.print(destination, HEX);
    Serial.print("  Rout: ");
    Serial.print(rout, HEX);
    Serial.print("  Ext. Frame Format: ");
    Serial.print(efformat, HEX);
    Serial.print("  Individual Address: ");
    Serial.print(iaarea, DEC);
    Serial.print(".");
    Serial.print(ialine, DEC);
    Serial.print(".");
    Serial.print(iadevice, DEC);
    Serial.print("  Group Address: ");
    Serial.print(gamain, DEC);
    Serial.print("/");
    Serial.print(gamiddle, DEC);
    Serial.print("/");
    Serial.print(gasub, DEC);
    Serial.print("  AT: ");
    Serial.print(at, HEX);
    Serial.print("  NPCI: ");
    Serial.print(npci, HEX);
    Serial.print("  Length: ");
    Serial.print(length2, HEX);
    Serial.print("  TPCI: ");
    Serial.print(tpci, HEX);
    Serial.print("  APCI(Type): ");
    Serial.print(apci, HEX);
    Serial.print("  DATA/APCI: ");
    Serial.print(data16, HEX);
    Serial.print("  DATA: ");
    Serial.print(data17, HEX);
    Serial.print(data18, HEX);
    Serial.print(data19, HEX);
    Serial.print(data20, HEX);
    Serial.print(data21, HEX);
    Serial.print(data22, HEX);
    Serial.print(data23, HEX);
    Serial.print(data24, HEX);
    Serial.print(data25, HEX);
    Serial.print(data26, HEX);
    Serial.print(data27, HEX);
    Serial.print(data28, HEX);
    Serial.print(data29, HEX);
    Serial.println(data30, HEX);

    Serial.print(" #");
    citac = citac += 1;
    Serial.print(citac);
    Serial.print("  Prio: ");
    if (priority == 0 ) {
      Serial.print("System");
    }
    else if (priority == 1 ) {
      Serial.print("Normal");
    }
    else if  (priority == 2 ) {
      Serial.print("Urgent");
    }
    else if  (priority == 3 ) {
      Serial.print("Low");
    }
    else {
      Serial.print("???");
    }
    Serial.print("  Source.addr: "); //Individual Address:
    Serial.print(iaarea, DEC);
    Serial.print(".");
    Serial.print(ialine, DEC);
    Serial.print(".");
    Serial.print(iadevice, DEC);
    Serial.print("  Dest.addr: "); //Group Address:
    Serial.print(gamain, DEC);
    Serial.print("/");
    Serial.print(gamiddle, DEC);
    Serial.print("/");
    Serial.print(gasub, DEC);
    Serial.print("  Rout: ");
    Serial.print(rout, HEX);
    Serial.print("  Type: "); //APCI(Type):
    if (apci == 0 ) {
      Serial.print("Read");
    }
    else if (apci == 1 ) {
      Serial.print("Response");
    }
    else if  (apci == 2 ) {
      Serial.print("Write");
    }
    else if  (apci == 10 ) {
      Serial.print("Memory Write");
    }
    else if  (apci == 11 ) {
      Serial.print("User Message");
    }
    else {
      Serial.print("???");
    }
    Serial.print("  DPT: "); //Total length:
    if (tlength == 1 ) {
      Serial.print("1bit ");
    }
    else if (tlength == 2 ) {
      Serial.print("1Byte ");
    }
    else if  (tlength == 3 ) {
      Serial.print("2Byte ");
    }
    else if  (tlength == 4 ) {
      Serial.print("3Byte ");
    }
    else if  (tlength == 5 ) {
      Serial.print("4Byte ");
    }
    else if  (tlength == 6 ) {
      Serial.print("5Byte ");
    }
    else if  (tlength == 7 ) {
      Serial.print("6Byte ");
    }
    else if  (tlength == 8 ) {
      Serial.print("7Byte ");
    }
    else if  (tlength == 9 ) {
      Serial.print("8Byte ");
    }
    else if  (tlength == 10 ) {
      Serial.print("9Byte ");
    }
    else if  (tlength == 11 ) {
      Serial.print("10Byte ");
    }
    else if  (tlength == 12 ) {
      Serial.print("11Byte ");
    }
    else if  (tlength == 13 ) {
      Serial.print("12Byte ");
    }
    else if  (tlength == 14 ) {
      Serial.print("13Byte ");
    }
    else if  (tlength == 15 ) {
      Serial.print("14Byte ");
    }
    else {
      Serial.print("???");
    }
    if  ((tlength == 1) && (apci == 2)) {  // if 1bit + Write
      Serial.print("  Data: ");
      Serial.print(data16, HEX);
      Serial.print(" | ");
      if (data16 == 0) {
        Serial.print(" Off ");
        Serial.print(" | ");
        Serial.print(" Decrease, Break ");
      }
      else if (data16 == 1) {
        Serial.print(" On ");
        Serial.print(" | ");
        Serial.print(" Decrease, 100% ");
      }
      else if (data16 == 2) {
        Serial.print(" Priority, Off ");
        Serial.print(" | ");
        Serial.print(" Decrease, 50% ");
      }
      else if (data16 == 3) {
        Serial.print(" Priority, On ");
        Serial.print(" | ");
        Serial.print(" Decrease, 25% ");
      }
      else if (data16 == 4) {
        Serial.print(" - ");
        Serial.print(" | ");
        Serial.print(" Decrease, 12% ");
      }
      else if (data16 == 5) {
        Serial.print(" - ");
        Serial.print(" | ");
        Serial.print(" Decrease, 6% ");
      }
      else if (data16 == 6) {
        Serial.print(" - ");
        Serial.print(" | ");
        Serial.print(" Decrease, 3% ");
      }
      else if (data16 == 7) {
        Serial.print(" - ");
        Serial.print(" | ");
        Serial.print(" Decrease, 1% ");
      }
      else if (data16 == 8) {
        Serial.print(" - ");
        Serial.print(" | ");
        Serial.print(" Increase, Break ");
      }
      else if (data16 == 9) {
        Serial.print(" - ");
        Serial.print(" | ");
        Serial.print(" Increase, 100% ");
      }
      else if (data16 == 10) {
        Serial.print(" - ");
        Serial.print(" | ");
        Serial.print(" Increase, 50% ");
      }
      else if (data16 == 11) {
        Serial.print(" - ");
        Serial.print(" | ");
        Serial.print(" Increase, 25% ");
      }
      else if (data16 == 12) {
        Serial.print(" - ");
        Serial.print(" | ");
        Serial.print(" Increase, 12% ");
      }
      else if (data16 == 13) {
        Serial.print(" - ");
        Serial.print(" | ");
        Serial.print(" Increase, 6% ");
      }
      else if (data16 == 14) {
        Serial.print(" - ");
        Serial.print(" | ");
        Serial.print(" Increase, 3% ");
      }
      else if (data16 == 15) {
        Serial.print(" - ");
        Serial.print(" | ");
        Serial.print(" Increase, 1% ");
      }
      else {
        Serial.print(" ?? ");
      }
    }
    if  ((tlength == 2) && (apci == 2)) {  // if 1Byte + Write
      float dpt5001 = (data17 * 0.392156862745098); // 0...100%
      char dpt5004 = data17; // 0...255
      signed char dpt6001 = data17; // -128...127
      Serial.print("  Data: ");
      Serial.print(data17, HEX);
      Serial.print(" | ");
      Serial.print(dpt5001); // 0...100%
      Serial.print("%");
      Serial.print(" | ");
      Serial.print(dpt5004, DEC); // 0...255
      Serial.print(" | ");
      Serial.print(dpt6001); // -128...127
    }
    if  ((tlength == 3) && (apci == 2)) {  // if 2Byte + Write
      uint16_t data2b = 0;
      data2b = data17;
      data2b <<= 8;
      data2b = data2b | data18;
      int dtp7001 = data2b; // 0...65535
      int dpt8001 = data2b; // -32768...32767
      if ( dpt8001 > 32767 ) {
        dpt8001 -= 65536;
      }
      uint8_t  dpt9001a = data17;
      uint8_t  dpt9001b = data17;
      uint16_t dpt9001c = 0;
      uint16_t dpt9001c1 = data17;
      uint16_t dpt9001c2 = data18;
      dpt9001a >>= 7;
      dpt9001b <<= 1;
      dpt9001b >>= 4;
      dpt9001c1 <<= 13;
      dpt9001c1 >>= 5;
      dpt9001c = dpt9001c1 | dpt9001c2;

      Serial.print(" ,a.b.c-DEC ");
      Serial.print(dpt9001a, DEC);
      Serial.print(" , ");
      Serial.print(dpt9001b, DEC);
      Serial.print(" , ");
      Serial.print(dpt9001c, DEC);
      Serial.print(" | ");

      Serial.print(" ,a.b.c-BIN ");
      Serial.print(dpt9001a, BIN);
      Serial.print(" , ");
      Serial.print(dpt9001b, BIN);
      Serial.print(" , ");
      Serial.print(dpt9001c, BIN);
      Serial.print(" | ");

      float dpt9001cf = dpt9001c;
      if  (dpt9001a == 1) {
        dpt9001cf = dpt9001c - 2048;
      }
      float dpt9001 = (dpt9001cf * 0.01) * pow(2, dpt9001b); // value = (cf * 0,01) 2^b
      Serial.print("  Data: ");
      Serial.print(data17, HEX);
      Serial.print(" ");
      Serial.print(data18, HEX);
      Serial.print(" | ");
      Serial.print(dtp7001);
      Serial.print(" | ");
      Serial.print(dpt8001);
      Serial.print(" | ");
      Serial.print(dpt9001);
      Serial.print(" | ");
      Serial.print(" ,a.b.c ");
      Serial.print(dpt9001a);
      Serial.print(" , ");
      Serial.print(dpt9001b);
      Serial.print(" , ");
      Serial.print(dpt9001cf);
      Serial.print(" | ");
    }
    if  ((tlength == 4) && (apci == 2)) {  // if 3Byte + Write
      uint32_t dptxx3b = 0; // 0...16777215
      uint32_t data3b17 = data17;
      uint32_t data3b18 = data18;
      uint32_t data3b19 = data19;
      data3b17 <<= 16;
      data3b18 <<= 8;
      data3b19 = data19;
      dptxx3b = data3b17 | data3b18 | data3b19;
      signed long dptxx3bs = dptxx3b; // -8388608...8388607
      uint8_t dpt250600r = data17; // Red
      uint8_t dpt250600g = data18; // Green
      uint8_t dpt250600b = data19; // Blue
      Serial.print("  Data: ");
      Serial.print(data17, HEX);
      Serial.print(" ");
      Serial.print(data18, HEX);
      Serial.print(" ");
      Serial.print(data19, HEX);
      Serial.print(" | ");
      Serial.print(dptxx3b, DEC);
      Serial.print(" | ");
      Serial.print(dptxx3bs, DEC);
      Serial.print(" | ");
      Serial.print(dpt250600r, DEC);
      Serial.print(", ");
      Serial.print(dpt250600g, DEC);
      Serial.print(", ");
      Serial.print(dpt250600b, DEC);
      Serial.print(" | ");
    }
    if  ((tlength == 5) && (apci == 2)) {  // if 4Byte + Write
      uint32_t dpt12001 = 0; // 0...4294967295
      uint32_t data4b17 = data17;
      uint32_t data4b18 = data18;
      uint32_t data4b19 = data19;
      uint32_t data4b20 = data20;
      data4b17 <<= 24;
      data4b18 <<= 16;
      data4b19 <<= 8;
      data4b20 = data20;
      dpt12001 = data4b17 | data4b18 | data4b19 | data4b20;
      signed long dpt13001 = dpt12001; // -2147483648...2147483647
      Serial.print("  Data: ");
      Serial.print(data17, HEX);
      Serial.print(" ");
      Serial.print(data18, HEX);
      Serial.print(" ");
      Serial.print(data19, HEX);
      Serial.print(" ");
      Serial.print(data20, HEX);
      Serial.print(" | ");
      Serial.print(dpt12001, DEC);
      Serial.print(" | ");
      Serial.print(dpt13001, DEC);
      Serial.print(" | ");
    }
    if  ((tlength == 6) && (apci == 2)) {  // if 5Byte + Write
      Serial.print("  Data: ");
      Serial.print(data17, HEX);
      Serial.print(" ");
      Serial.print(data18, HEX);
      Serial.print(" ");
      Serial.print(data19, HEX);
      Serial.print(" ");
      Serial.print(data20, HEX);
      Serial.print(" ");
      Serial.print(data21, HEX);
      Serial.print(" | ");
    }
    if  ((tlength == 7) && (apci == 2)) {  // if 6Byte + Write
      uint16_t dpt251600r12 = data17; //[12 bit]
      uint16_t dpt251600r12b = data18;
      uint8_t dpt251600mr = data18; //[1 bit] Shall specify whether the colour information red in the field R is valid or not.
      uint8_t dpt251600mg = data18; //[1 bit] Shall specify whether the colour information red in the field G is valid or not.
      uint8_t dpt251600mb = data18; //[1 bit] Shall specify whether the colour information red in the field B is valid or not.
      uint8_t dpt251600mw = data18; //[1 bit] Shall specify whether the colour information red in the field W is valid or not.
      uint8_t dpt251600r = data19; //[8 bit] Colour Level Red
      uint8_t dpt251600g = data20; //[8 bit] Colour Level Green
      uint8_t dpt251600b = data21; //[8 bit] Colour Level Blue
      uint8_t dpt251600w = data22; //[8 bit] Colour Level White
      dpt251600r12 <<= 4;
      dpt251600r12b >>= 4;
      dpt251600r12 = dpt251600r12 | dpt251600r12b;
      dpt251600mr <<= 4;
      dpt251600mr >>= 7;
      dpt251600mg <<= 5;
      dpt251600mg >>= 7;
      dpt251600mb <<= 6;
      dpt251600mb >>= 7;
      dpt251600mw <<= 7;
      dpt251600mw >>= 7;
      uint8_t dpt242600c = data17; //[8 bit] 0...1, 0...1
      uint16_t dpt242600xa = data18; //[16 bit] x-coordinate of the colour information 0...65535
      uint16_t dpt242600ya = data20; //[16 bit] y-coordinate of the colour information 0...65535
      uint8_t dpt242600b = data22; //[8 bit] Brightness of the colour 0...100%
      dpt242600xa <<= 8;
      dpt242600xa = dpt242600xa | data19;
      dpt242600ya <<= 8;
      dpt242600ya = dpt242600ya | data21;
      Serial.print("  Data: ");
      Serial.print(data17, HEX);
      Serial.print(" ");
      Serial.print(data18, HEX);
      Serial.print(" ");
      Serial.print(data19, HEX);
      Serial.print(" ");
      Serial.print(data20, HEX);
      Serial.print(" ");
      Serial.print(data21, HEX);
      Serial.print(" ");
      Serial.print(data22, HEX);
      Serial.print(" | ");
      Serial.print(dpt251600r12, BIN);
      Serial.print(", ");
      Serial.print(dpt251600mr, DEC);
      Serial.print(", ");
      Serial.print(dpt251600mg, DEC);
      Serial.print(", ");
      Serial.print(dpt251600mb, DEC);
      Serial.print(", ");
      Serial.print(dpt251600mw, DEC);
      Serial.print(", ");
      Serial.print(dpt251600r, DEC);
      Serial.print(", ");
      Serial.print(dpt251600g, DEC);
      Serial.print(", ");
      Serial.print(dpt251600b, DEC);
      Serial.print(", ");
      Serial.print(dpt251600w, DEC);
      Serial.print(" | ");
      Serial.print(dpt242600c, BIN);
      Serial.print(", ");
      Serial.print(dpt242600xa, DEC);
      Serial.print(", ");
      Serial.print(dpt242600ya, DEC);
      Serial.print(", ");
      Serial.print(dpt242600b, DEC);
      Serial.print(" | ");
    }
    if  ((tlength == 8) && (apci == 2)) {  // if 7Byte + Write
      Serial.print("  Data: ");
      Serial.print(data17, HEX);
      Serial.print(" ");
      Serial.print(data18, HEX);
      Serial.print(" ");
      Serial.print(data19, HEX);
      Serial.print(" ");
      Serial.print(data20, HEX);
      Serial.print(" ");
      Serial.print(data21, HEX);
      Serial.print(" ");
      Serial.print(data22, HEX);
      Serial.print(" ");
      Serial.print(data23, HEX);
      Serial.print(" | ");
    }
    if  ((tlength == 9) && (apci == 2)) {  // if 8Byte + Write
      Serial.print("  Data: ");
      Serial.print(data17, HEX);
      Serial.print(" ");
      Serial.print(data18, HEX);
      Serial.print(" ");
      Serial.print(data19, HEX);
      Serial.print(" ");
      Serial.print(data20, HEX);
      Serial.print(" ");
      Serial.print(data21, HEX);
      Serial.print(" ");
      Serial.print(data22, HEX);
      Serial.print(" ");
      Serial.print(data23, HEX);
      Serial.print(" ");
      Serial.print(data24, HEX);
      Serial.print(" | ");
    }
    if  ((tlength == 10) && (apci == 2)) {  // if 9Byte + Write
      Serial.print("  Data: ");
      Serial.print(data17, HEX);
      Serial.print(" ");
      Serial.print(data18, HEX);
      Serial.print(" ");
      Serial.print(data19, HEX);
      Serial.print(" ");
      Serial.print(data20, HEX);
      Serial.print(" ");
      Serial.print(data21, HEX);
      Serial.print(" ");
      Serial.print(data22, HEX);
      Serial.print(" ");
      Serial.print(data23, HEX);
      Serial.print(" ");
      Serial.print(data24, HEX);
      Serial.print(" ");
      Serial.print(data25, HEX);
      Serial.print(" | ");
    }
    if  ((tlength == 11) && (apci == 2)) {  // if 10Byte + Write
      Serial.print("  Data: ");
      Serial.print(data17, HEX);
      Serial.print(" ");
      Serial.print(data18, HEX);
      Serial.print(" ");
      Serial.print(data19, HEX);
      Serial.print(" ");
      Serial.print(data20, HEX);
      Serial.print(" ");
      Serial.print(data21, HEX);
      Serial.print(" ");
      Serial.print(data22, HEX);
      Serial.print(" ");
      Serial.print(data23, HEX);
      Serial.print(" ");
      Serial.print(data24, HEX);
      Serial.print(" ");
      Serial.print(data25, HEX);
      Serial.print(" ");
      Serial.print(data26, HEX);
      Serial.print(" | ");
    }
    if  ((tlength == 12) && (apci == 2)) {  // if 11Byte + Write
      Serial.print("  Data: ");
      Serial.print(data17, HEX);
      Serial.print(" ");
      Serial.print(data18, HEX);
      Serial.print(" ");
      Serial.print(data19, HEX);
      Serial.print(" ");
      Serial.print(data20, HEX);
      Serial.print(" ");
      Serial.print(data21, HEX);
      Serial.print(" ");
      Serial.print(data22, HEX);
      Serial.print(" ");
      Serial.print(data23, HEX);
      Serial.print(" ");
      Serial.print(data24, HEX);
      Serial.print(" ");
      Serial.print(data25, HEX);
      Serial.print(" ");
      Serial.print(data26, HEX);
      Serial.print(" ");
      Serial.print(data27, HEX);
      Serial.print(" | ");
    }
    if  ((tlength == 13) && (apci == 2)) {  // if 12Byte + Write
      Serial.print("  Data: ");
      Serial.print(data17, HEX);
      Serial.print(" ");
      Serial.print(data18, HEX);
      Serial.print(" ");
      Serial.print(data19, HEX);
      Serial.print(" ");
      Serial.print(data20, HEX);
      Serial.print(" ");
      Serial.print(data21, HEX);
      Serial.print(" ");
      Serial.print(data22, HEX);
      Serial.print(" ");
      Serial.print(data23, HEX);
      Serial.print(" ");
      Serial.print(data24, HEX);
      Serial.print(" ");
      Serial.print(data25, HEX);
      Serial.print(" ");
      Serial.print(data26, HEX);
      Serial.print(" ");
      Serial.print(data27, HEX);
      Serial.print(" ");
      Serial.print(data28, HEX);
      Serial.print(" | ");
    }
    if  ((tlength == 14) && (apci == 2)) {  // if 13Byte + Write
      Serial.print("  Data: ");
      Serial.print(data17, HEX);
      Serial.print(" ");
      Serial.print(data18, HEX);
      Serial.print(" ");
      Serial.print(data19, HEX);
      Serial.print(" ");
      Serial.print(data20, HEX);
      Serial.print(" ");
      Serial.print(data21, HEX);
      Serial.print(" ");
      Serial.print(data22, HEX);
      Serial.print(" ");
      Serial.print(data23, HEX);
      Serial.print(" ");
      Serial.print(data24, HEX);
      Serial.print(" ");
      Serial.print(data25, HEX);
      Serial.print(" ");
      Serial.print(data26, HEX);
      Serial.print(" ");
      Serial.print(data27, HEX);
      Serial.print(" ");
      Serial.print(data28, HEX);
      Serial.print(" ");
      Serial.print(data29, HEX);
      Serial.print(" | ");
    }
    if  ((tlength == 15) && (apci == 2)) {  // if 14Byte + Write
      Serial.print("  Data: ");
      Serial.print(data17, HEX);
      Serial.print(" ");
      Serial.print(data18, HEX);
      Serial.print(" ");
      Serial.print(data19, HEX);
      Serial.print(" ");
      Serial.print(data20, HEX);
      Serial.print(" ");
      Serial.print(data21, HEX);
      Serial.print(" ");
      Serial.print(data22, HEX);
      Serial.print(" ");
      Serial.print(data23, HEX);
      Serial.print(" ");
      Serial.print(data24, HEX);
      Serial.print(" ");
      Serial.print(data25, HEX);
      Serial.print(" ");
      Serial.print(data26, HEX);
      Serial.print(" ");
      Serial.print(data27, HEX);
      Serial.print(" ");
      Serial.print(data28, HEX);
      Serial.print(" ");
      Serial.print(data29, HEX);
      Serial.print(" ");
      Serial.print(data30, HEX);
      Serial.print(" | ");
    }
    Serial.println(" ");
    byte packetBuffer[] = {000000000000000000000000000000000000000000000000000000000000}; //reset buffer
  }
}
