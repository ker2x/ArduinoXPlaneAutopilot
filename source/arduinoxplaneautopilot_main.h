//-------------------------------------------------------------------
#ifndef __arduinoxplaneautopilot_main_H__
#define __arduinoxplaneautopilot_main_H__
//-------------------------------------------------------------------
 
#include <arduino.h>
#include <w5100.h>
#include <socket.h>
#include <util.h>
#include <EthernetUdp.h>
#include <EthernetServer.h>
#include <EthernetClient.h>
#include <Ethernet.h>
#include <Dns.h>
#include <Dhcp.h>
#include <SPI.h>

#include "kerpid.h"
 
//-------------------------------------------------------------------
 
//-------------------------------------------------------------------
 
// Put yout declarations here
#define UDP_TX_PACKET_MAX_SIZE 1024
char RXBuffer[UDP_TX_PACKET_MAX_SIZE];
char TXBuffer[UDP_TX_PACKET_MAX_SIZE];
 
EthernetClient ethernet;
EthernetUDP udp;
byte mac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02 };
unsigned int localPort = 49003;      // local port to listen on
 
float fzero = 0.0F;

long idx_joy = 8L;
long idx_thr = 25L;
float *speed, *throttle;
float speedPID_In, speedPID_Out;
float speedPID_Target;

float *AoA, *elev;
float AoAPID_In, AoAPID_Out;
float AoAPID_Target;

float *VVI;
float VVIPID_In, VVIPID_Out;
float VVIPID_Target;

float *pitch, *roll;
float rollPID_In, rollPID_Out;
float rollPID_Target;

/*
P_Param:  the bigger the number the harder the controller pushes.
I_Param:  the SMALLER the number (except for 0, which turns it off,)  the more quickly the controller reacts to load changes, but the greater the risk of oscillations.
D_Param: the bigger the number  the more the controller dampens oscillations (to the point where performance can be hindered)
*/

KerPID speedPID(&speedPID_In, &speedPID_Out, &speedPID_Target,0.01,0.05,0.002, DIRECT);

KerPID AoAPID(&AoAPID_In, &AoAPID_Out, &AoAPID_Target, 0.6, 1.0, 0.001, DIRECT);

KerPID VVIPID(&VVIPID_In, &VVIPID_Out, &VVIPID_Target, 0.005, 0.002, 0.0, DIRECT);

KerPID rollPID(&rollPID_In, &rollPID_Out, &rollPID_Target, 0.005, 0.002, 0.0, DIRECT);
 
//-------------------------------------------------------------------
 
//===================================================================
// -> DO NOT WRITE ANYTHING BETWEEN HERE...
// 		This section is reserved for automated code generation
// 		This process tries to detect all user-created
// 		functions in main_sketch.cpp, and inject their
// 		declarations into this file.
// 		If you do not want to use this automated process,
//		simply delete the lines below, with "&MM_DECLA" text
//===================================================================
//---- DO NOT DELETE THIS LINE -- @MM_DECLA_BEG@---------------------
void loop();
void setup();
//---- DO NOT DELETE THIS LINE -- @MM_DECLA_END@---------------------
// -> ...AND HERE. This space is reserved for automated code generation!
//===================================================================
 
 
//-------------------------------------------------------------------
#endif
//-------------------------------------------------------------------
 
 
