//Testing out CAN Bus ONLY
#include <FlexCAN.h> //Can Bus

//======================START GLOBAL VARIABLES==============================
//------------Global Variables------------
int RPM;
int SPEED;
int OIL_TEMP;
int COOLANT_TEMP;
int AIR_TEMP;
int INTAKE_TEMP;
int BATT_VOLTAGE;
int FUEL_PRESSURE;
int FUEL_RATE;
int THROTTLE_POS;

//------------Global Time Variables------------
unsigned long currentMillis = 0;
//========================END GLOBAL VARIABLES==============================

//=====================START CAN BUS RELATED VARIABLES======================
//------------CAN PIDs------------
/* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
#define MODE1               0x01        //Show current data
#define MODE2               0x02        //Show freeze frame data
#define MODE3               0x03        //Show stored Diagnostic Trouble Codes
#define MODE4               0x04        //Clear Diagnostic Trouble Codes and stored values

#define PID_SUPPORTED       0x00
#define PID_MONITOR_STATUS  0x01
#define PID_SPEED           0x0D
#define PID_RPM             0x0C
#define PID_OIL_TEMP        0x5C
#define PID_COOLANT_TEMP    0x05
#define PID_INTAKE_TEMP     0x0F
#define PID_AIR_TEMP        0x46
#define PID_BATT_VOLTAGE    0x42
#define O2_VOLTAGE_1        0x24
#define O2_VOLTAGE_2        0x25
#define PID_FUEL_PRESSURE   0x0A
#define PID_FUEL_RATE       0x5E
#define PID_THROTTLE_POS    0x11

#define MODE1_RESPONSE      0x41
#define MODE3_RESPONSE      0x43
#define MODE4_RESPONSE      0x44
#define PID_REQUEST         0x7DF
#define PID_REPLY           0x7E8
#define DTC_PID             0x00

//------------DIAGNOSTIC SERIAL DUMP------------
static uint8_t hex[17] = "0123456789abcdef";
static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
  uint8_t working;
  while ( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working >> 4 ] );
    Serial.write( hex[ working & 15 ] );
  }
  Serial.write('\r');
  Serial.write('\n');
}

//------------CAN Variables------------
byte PID_CALL;
byte PID;
byte MODE;
byte PID_RESPONSE;
CAN_message_t can_MsgRx, can_MsgTx;
bool CANWaiting = false;
int CAN_Retry = 0;

//------------Structure Arrray Initialisation------------
struct OBD {
  String Name;
  word   PID;
  bool   Active;
  word   MODE;
  int*   Data;
  int    NoReply;
};

//------------DTC Variables------------
int DTC = 2;
int numDTC;
int DTC1Raw;
int DTC2Raw;
char DTC1 [5];
char DTC2 [5];
int SeenDTC = 0;
int DTCp1;
int DTCp2;
int DTC_CLEARED = 0;

//------------CAN Time Variables------------

//For CAN
unsigned long CAN_Request_Millis = 10;
unsigned long CAN_Timeout_Millis = 10;
unsigned long CAN_Timeout = 100;
unsigned long CAN_DeadConnection = 1000;
bool CAN_SNIFF = true;
unsigned long CAN_Wait_Duration = 0;
unsigned long CAN_Timeout_Duration = 0;

//------------Set Up Structure Array------------
int Row = 0;
OBD OBDArray[] = {{"RPM", PID_RPM, true, MODE1, &RPM, 0},
  {"Speed", PID_SPEED, true, MODE1, &SPEED, 0},
  {"Coolant Temp", PID_COOLANT_TEMP, true, MODE1, &COOLANT_TEMP, 0},
  {"Oil Temp", PID_OIL_TEMP, true, MODE1, &OIL_TEMP, 0},
  {"Air Temp", PID_AIR_TEMP, true, MODE1, &AIR_TEMP, 0},
  {"Intake Temp", PID_INTAKE_TEMP, true, MODE1, &INTAKE_TEMP, 0},
  {"Throttle Pos", PID_THROTTLE_POS, true, MODE1, &THROTTLE_POS, 0},
  {"Fuel Pressure", PID_FUEL_PRESSURE, true, MODE1, &FUEL_PRESSURE, 0},
  {"Fuel Rate", PID_FUEL_RATE, true, MODE1, &FUEL_RATE, 0},
  {"Batt Voltage", PID_BATT_VOLTAGE, true, MODE1, &BATT_VOLTAGE, 0},
  {"Trouble Codes", DTC_PID, true, MODE3, &numDTC, 0}
};
byte Arraysize = sizeof(OBDArray) / sizeof(OBDArray[0]);
//=====================END CAN BUS RELATED VARIABLES=====================

//====== END SET UP VARIABLES =================================================================================

void setup(void) {//+++++++++++++++++++++++++++++++++++++++START SETUP+++++++++++++++++++++++++++++++++++++++++++

  Serial.begin(9600);
  Can0.begin(500000);
  delay(500);
  //Load EEPROM and update Active PIDs in table
}//++++++++++++++++++++++++++++++++++++++++++++++++++++++++END SETUP+++++++++++++++++++++++++++++++++++++++++++++

void loop(void) {//+++++++++++++++++++++++++++++++++++++++START LOOP+++++++++++++++++++++++++++++++++++++++++++++
  currentMillis = millis();

  //----------READ CAN BUS----------
  Get_CAN_Data();
  if (CAN_SNIFF != true)Print_CAN_Data();

}//++++++++++++++++++++++++++++++++++++++++++++++++++++++++END LOOP++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++START FUNCTIONS+++++++++++++++++++++++++++++++++++++++++++++

//=====================================START CAN BUS FUNTIONS====================================================
//-------------GET CAN DATA-------------
void Get_CAN_Data() {

  CAN_Wait_Duration = currentMillis - CAN_Request_Millis; //how long since we last requested something?
  CAN_Timeout_Duration = currentMillis - CAN_Timeout_Millis; //how long since we last received something?
  if (CAN_Timeout_Duration > CAN_DeadConnection) CAN_SNIFF = true; //else CAN_SNIFF = false; // if we haven't heard from CAN bus for >1s then stop CAN link and start sniffing for re-connection

  //************************************************************************************************CAN SNIFFING MODE
  //Are we sniffing for CAN?

  if (CAN_SNIFF == true) {
    if (CANWaiting == true) { // IS IT THE RESPONSE WE'RE WAITING FOR?? IF SO, SAVE IT AND MOVE ON
      if (Can0.available() > 0) {//Is a response waiting?
        Can0.read(can_MsgRx);
        CAN_Timeout_Millis = currentMillis;
        if ((can_MsgRx.id == PID_REPLY) && (can_MsgRx.buf[2] == PID_RPM)) { //Is this message the response we're looking for?
          CANWaiting = false; //Reset waiting flag
          CAN_SNIFF = false;//PConfirmed data received, stop sniffing
        }
      }
      else { //Still waiting - check for timeout
        //      CAN_Wait_Duration = currentMillis - CAN_Request_Millis;
        if (CAN_Wait_Duration > CAN_Timeout) { //Waited too long
          //Show Error
          Serial.println("OBD NO CONNECTION");
          //If it has just dropped a single packet then give it another try
          CANWaiting = false;
        }
      }
    }
    if (CANWaiting == false && CAN_SNIFF == true) {
      //Request next Data
      CAN_Request(MODE1, PID_RPM);
      CAN_Request_Millis = currentMillis;
      CANWaiting = true;
    }
  }
  //****************************************************************************************************CAN NORMAL COMMUNICATION
  //Are we waiting for a response?
  if ((CANWaiting == true) && (CAN_SNIFF == false)) { // IS IT THE RESPONSE WE'RE WAITING FOR?? IF SO, SAVE IT AND MOVE ON
    if (Can0.available() > 0) {//Is a response waiting?
      Can0.read(can_MsgRx);
      if ((can_MsgRx.id == PID_REPLY) && ((can_MsgRx.buf[2] == OBDArray[Row].PID) || can_MsgRx.buf[1] == MODE3_RESPONSE)) //Is this message the response we're looking for?
      {
        CAN_Timeout_Millis = currentMillis;
        //Yes
        if (can_MsgRx.buf[1] == MODE3_RESPONSE) {//Is the message an error code?
          //Serial.print("CAN ERROR RECEIVED: "); hexDump(8, can_MsgRx.buf); //Diagnostic - Dump message to Serial useful explanation: https://www.csselectronics.com/screen/page/simple-intro-obd2-explained/language/en
          numDTC = CAN_ERR_Receive(can_MsgRx.buf[2], can_MsgRx.buf[3], can_MsgRx.buf[4], can_MsgRx.buf[5], can_MsgRx.buf[6]);
        }
        else {//Normal PID Received
          //Serial.print("CAN RECEIVED: "); hexDump(8, can_MsgRx.buf); //Diagnostic - Dump message to Serial useful explanation: https://www.csselectronics.com/screen/page/simple-intro-obd2-explained/language/en
          *(OBDArray[Row].Data) = CAN_Receive_Parse(can_MsgRx.buf[2], can_MsgRx.buf[3], can_MsgRx.buf[4]);
        }
        CANWaiting = false; //Reset waiting flag
        NextPID();//Prepare next PID
      }
    }

    else { //Still waiting - check for timeout
      if (CAN_Wait_Duration > CAN_Timeout) //Waited too long
      {
        if (OBDArray[Row].Name != "Trouble Codes") { //If waiting for an error code then skip this part
          OBDArray[Row].NoReply ++; //Add to the no reply counter for this PID
          if (OBDArray[Row].NoReply > 100) { //if >100 no replys then this PID is not available, so stop requesting it and show an error
            //Switch off this PID Request
            OBDArray[Row].Active = false;
            //Show Error
            Serial.print("OBD Error - ");
            Serial.println(OBDArray[Row].Name);
            //Try Next PID
            NextPID();
            CAN_Retry = 0;
            CANWaiting = false;
          }

          //If it has just dropped a single packet then give it another try
          if (CAN_Retry == 2) {
            //Try Next PID
            NextPID();
            CAN_Retry = 0;
            CANWaiting = false;
          }
          else {
            CAN_Retry ++; //Retry this PID
          }
        }
        else { //If waiting for an error code don't bother retrying
          NextPID();
          CAN_Retry = 0;
          CANWaiting = false;
        }
      }
    }
  }

  if (CANWaiting == false && CAN_SNIFF == false) {
    //Request next Data
    CAN_Request(OBDArray[Row].MODE, OBDArray[Row].PID);
    CAN_Request_Millis = currentMillis;
    CANWaiting = true;
  }
}

//-------------REQUEST CAN DATA-------------
void CAN_Request(byte CAN_MODE, byte PID_CALL) {
  can_MsgTx.ext = 0;
  can_MsgTx.id = PID_REQUEST;
  can_MsgTx.len = 8;

  can_MsgTx.buf[0] = 0x02; //Length in number of bytes of remaining data (2 bytes = 02, 3 bytes = 03 etc)
  can_MsgTx.buf[1] = CAN_MODE;
  can_MsgTx.buf[2] = PID_CALL;
  can_MsgTx.buf[3] = 0;
  can_MsgTx.buf[4] = 0;
  can_MsgTx.buf[5] = 0;
  can_MsgTx.buf[6] = 0;
  can_MsgTx.buf[7] = 0;

  can_MsgTx.flags.extended = 0;
  can_MsgTx.flags.remote = 0;

  //Serial.print("CAN SENT: "); hexDump(8, can_MsgTx.buf); //Diagnostic - Dump message to Serial//@@@@@@@@@@@@@@@@@@@@@@@@@@@CAN DIAGNOSTICS@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //Serial.println(currentMillis / 10);
  Can0.write(can_MsgTx); //Send message
}

//-------------NEXT PID TO RUN-------------
void NextPID() {
  //Start with next row unless at end of table, then start from 0
  for (int i = 0; i < (Arraysize - 1); i++) {

    if ((i + Row) == (Arraysize - 1)) Row = 0;
    else Row = (Row + 1);
    if (OBDArray[Row].Active == true) break;
  }
}

//-------------SERIAL PRINT CAN DATA-------------
void Print_CAN_Data() {
  Serial.print("RPM: ");
  Serial.print(RPM);
  Serial.print(" | SPEED: ");
  Serial.print(SPEED);
  Serial.print(" | COOLANT: ");
  Serial.print(COOLANT_TEMP);
  Serial.print(" | OIL: ");
  Serial.print(OIL_TEMP);
  Serial.print(" | AMBIENT: ");
  Serial.print(AIR_TEMP);
  Serial.print(" | INTAKE: ");
  Serial.print(INTAKE_TEMP);
  Serial.print(" | TPS: ");
  Serial.print(THROTTLE_POS);
  Serial.print(" | FUEL P: ");
  Serial.print(FUEL_PRESSURE);
  Serial.print(" | FUEL R: ");
  Serial.print(FUEL_RATE);
  Serial.print(" | BATT: ");
  Serial.print(BATT_VOLTAGE);
  Serial.print(" | DTC: ");
  Serial.println(numDTC);
}

//-------------PROCESS DTC INCOMING-------------
int CAN_ERR_Receive(byte pkt2, byte pkt3, byte pkt4, byte pkt5, byte pkt6) {
  numDTC = pkt2;

  if (numDTC > 0) {
    DTCp1 = pkt3;
    DTCp2 = pkt4;
    if (DTCp1 < 10) DTC1Raw = (DTCp1 * 100) + DTCp2;
    else DTC1Raw = (DTCp1 * 1000) + DTCp2;
    sprintf (DTC1, "P%04u", DTC1Raw);
  }
  else sprintf (DTC1, "");

  if (numDTC > 1) {
    DTCp1 = pkt5;
    DTCp2 = pkt6;
    if (DTCp1 < 10) DTC2Raw = (DTCp1 * 100) + DTCp2;
    else DTC2Raw = (DTCp1 * 1000) + DTCp2;
    sprintf (DTC2, "P%04u", DTC2Raw);
  }
  else sprintf (DTC2, "");
  return numDTC;
}

//-------------PROCESS NORMAL PID INCOMING-------------
int CAN_Receive_Parse(byte PID_Received, int Data_1, int Data_2) {
  switch (PID_Received) //Switch means look at different cases
  { /* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */

    case PID_SPEED:         // A                  [km]  int SPEED =
      return  Data_1 * 1; //CHECK SPEED IS IN KPH AND CONVERSION!!!!
      break;

    case PID_RPM:              //   ((A*256)+B)/4    [RPM]
      return  ((Data_1 * 256) + Data_2) / 4;
      break;

    case PID_OIL_TEMP:     //     A-40              [degree C]
      return  Data_1 - 40;
      break;

    case PID_COOLANT_TEMP:     //     A-40              [degree C]
      return  Data_1 - 40;
      break;

    case PID_INTAKE_TEMP:     //     A-40              [degree C]
      return  Data_1 - 40;
      break;

    case PID_AIR_TEMP:     //     A-40              [degree C]
      return  Data_1 - 40;
      break;

    case PID_BATT_VOLTAGE:            //
      return ((Data_1 * 256) + Data_2);
      break;

    case PID_FUEL_PRESSURE:     //                   [kPa]
      return  Data_1 * 3;
      break;

    case PID_FUEL_RATE:     //                   [l/h]
      return  ((Data_1 * 256) + Data_2) / 2;
      break;

    case PID_THROTTLE_POS:
      return int((Data_1 * 100) / 255);
      break;

    case PID_MONITOR_STATUS:
      return Data_1;// & Data_2 & can_MsgRx.buf[5];
      break;
  }
}
//=========================================END CAN BUS FUNCTIONS====================================================
