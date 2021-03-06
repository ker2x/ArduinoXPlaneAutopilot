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
#include "linearservo.h"
 
//-------------------------------------------------------------------
 
//-------------------------------------------------------------------
 
// Put your declarations here
#define UDP_TX_PACKET_MAX_SIZE 256
char RXBuffer[UDP_TX_PACKET_MAX_SIZE];
char TXBuffer[UDP_TX_PACKET_MAX_SIZE];
char CharBuffer[6];

char SBuffer[36]; // Receive buffer for serial port
int debugloop = 0;
 
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

float *AoA; //, *elev;
//float AoAPID_In, AoAPID_Out;
//float AoAPID_Target;


float *VVI;
float VVIPID_In, VVIPID_Out;
float VVIPID_Target;

float *pitch, *roll;
float rollPID_In, rollPID_Out;
float rollPID_Target;

float *hding;
float hding_v, hding_vt;
float hdingPID_In, hdingPID_Out;
float hdingPID_Target;

float *alt;
float altPID_In, altPID_Out;
float altPID_Target;

float *slip;
float slipPID_In, slipPID_Out;
float slipPID_Target;

typedef enum {
	CLIMB_VVI,
	CLIMB_ALT
} CLIMB_MODE_E;
	
CLIMB_MODE_E CLIMB_MODE = CLIMB_VVI;

float elev, elev_Target;
LinearServo elevServo(0.025);

float ailrn, ailrn_Target;
LinearServo ailrnServo(0.025);


/* VALUES FOR C337 SKYMASTER */
KerPID speedPID(&speedPID_In, &speedPID_Out, &speedPID_Target,0.01,0.05,0.002, DIRECT);
KerPID altPID(&altPID_In, &altPID_Out, &altPID_Target, 2.0, 0.01, 0.001, DIRECT);
KerPID VVIPID(&VVIPID_In, &VVIPID_Out, &VVIPID_Target, 0.0004, 0.0003, 0.00005, DIRECT);
KerPID rollPID(&rollPID_In, &rollPID_Out, &rollPID_Target, 0.008, 0.004, 0.001, DIRECT);
KerPID hdingPID(&hdingPID_In, &hdingPID_Out, &hdingPID_Target, 0.2, 0.001, 0.0005, DIRECT);
KerPID slipPID(&slipPID_In, &slipPID_Out, &slipPID_Target, 0.1, 0.05, 0.005, DIRECT);


/* VALUES FOR C172SP (NEED UPDATE)
KerPID speedPID(&speedPID_In, &speedPID_Out, &speedPID_Target, 0.25, 0.1, 0.005, DIRECT);
KerPID VVIPID(&VVIPID_In, &VVIPID_Out, &VVIPID_Target, 0.0002, 0.00015, 0.0001, DIRECT);
KerPID altPID(&altPID_In, &altPID_Out, &altPID_Target, 1.0, 0.05, 0.0, DIRECT);
KerPID rollPID(&rollPID_In, &rollPID_Out, &rollPID_Target, 0.01, 0.001, 0.001, DIRECT);
KerPID hdingPID(&hdingPID_In, &hdingPID_Out, &hdingPID_Target, 0.2, 0.001, 0.0005, DIRECT);
KerPID slipPID(&slipPID_In, &slipPID_Out, &slipPID_Target, 0.1, 0.05, 0.005, DIRECT);
*/

/* Values for 777 Jardesign : NOT WORKING (yet)
KerPID speedPID(&speedPID_In, &speedPID_Out, &speedPID_Target, 0.25, 0.1, 0.005, DIRECT);
KerPID VVIPID(&VVIPID_In, &VVIPID_Out, &VVIPID_Target, 0.00005, 0.00001, 0.00001, DIRECT);
KerPID altPID(&altPID_In, &altPID_Out, &altPID_Target, 1.0, 0.05, 0.0, DIRECT);
KerPID rollPID(&rollPID_In, &rollPID_Out, &rollPID_Target, 0.01, 0.001, 0.001, DIRECT);
KerPID hdingPID(&hdingPID_In, &hdingPID_Out, &hdingPID_Target, 0.2, 0.001, 0.0005, DIRECT);
KerPID slipPID(&slipPID_In, &slipPID_Out, &slipPID_Target, 0.1, 0.05, 0.005, DIRECT);
*/
 
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
int freeRam ();
void setupBuffer();
void setupPID();
void loop();
void setup();
//---- DO NOT DELETE THIS LINE -- @MM_DECLA_END@---------------------
// -> ...AND HERE. This space is reserved for automated code generation!
//===================================================================
 
 
//-------------------------------------------------------------------
#endif
//-------------------------------------------------------------------
 
 
