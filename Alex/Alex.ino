#include "serialize.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "packet.h"
#include "constants.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "buffer.h"

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

#define BUF_LEN 500
volatile TBuffer recvBuffer;
volatile TBuffer sendBuffer;
volatile TDirection dir = STOP;
/*
   Alex's configuration constants
*/

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV      175

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.41

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
//#define LF                  6   // Left forward pin
//#define LR                  5   // Left reverse pin
//#define RF                  10  // Right forward pin
//#define RR                  9 // Right reverse pin

// PI, for calculating turn circumference
#define PI 3.1415926535897932384626433832795
// pins for PWM compare match
#define LF               OCR0A
#define LR               OCR0B
#define RF               OCR1BL
#define RR               OCR1AL

//#define ALEX_LENGTH to be measured
#define ALEX_LENGTH         20.5
//#define ALEX_BREADTH to be measured
#define ALEX_BREADTH        12.5

float alexDiagonal = 0.0;
float alexCirc = 0.0;
/*
      Alex's State Variables
*/

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Left and right encider ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variable to keep track of whether we have moved a command distance
volatile unsigned long deltaDist;
volatile unsigned long newDist;

//Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

/*

   Alex Communication Routines.

*/

TResult readPacket(TPacket *packet)
{
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);

}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDists
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command =  RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  statusPacket.params[10] = getDist(); //encapsulate distance in the packet so that we can print it later
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprint(char *format, ...) {
  va_list args;
  char buffer[128];
  
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);

}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
 
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  if (dir == FORWARD) {
    leftForwardTicks++;
    leftRevs = leftForwardTicks/COUNTS_PER_REV;
    forwardDist = (unsigned long)((double)leftForwardTicks/COUNTS_PER_REV*WHEEL_CIRC);
  }
  else if (dir == BACKWARD) {
    leftReverseTicks++;
    leftRevs = leftForwardTicks/COUNTS_PER_REV;
    reverseDist = (unsigned long)((double)leftReverseTicks/COUNTS_PER_REV*WHEEL_CIRC);
  }
  else if (dir == LEFT) {
    leftReverseTicksTurns++;
    leftRevs = leftForwardTicks/COUNTS_PER_REV;
  }
  else if (dir == RIGHT) {
    leftForwardTicksTurns++;
    leftRevs = leftForwardTicks/COUNTS_PER_REV;
  }
}

void rightISR()
{
  if (dir == FORWARD) {
    rightForwardTicks++;
    rightRevs = rightForwardTicks/COUNTS_PER_REV;
  }
  else if (dir == BACKWARD) {
    rightReverseTicks++;
    rightRevs = rightForwardTicks/COUNTS_PER_REV;
  }
  else if (dir == LEFT) {
    rightForwardTicksTurns++;
    rightRevs = rightForwardTicks/COUNTS_PER_REV;
  }
  else if (dir == RIGHT) {
    rightReverseTicksTurns++;
    rightRevs = rightForwardTicks/COUNTS_PER_REV;
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EICRA |= 0b00001010;
  EIMSK |= 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR(INT0_vect) {
  leftISR();
}

ISR(INT1_vect) {
  rightISR();
}

// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial() //REPLACED WITH BAREMETAL
{
  initBuffer(&sendBuffer, BUF_LEN);
  initBuffer(&recvBuffer, BUF_LEN);
  UCSR0C = 0b00000110;
  UCSR0A = 0;
  UBRR0H = 0;
  UBRR0L = 103; //to make baud rate = 9600
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  UCSR0B = 0b10011000; //this starts the serial comms
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.
ISR(USART_RX_vect) { //ISR for receiving the char
  unsigned char data = UDR0;

  writeBuffer(&recvBuffer, data);
}

ISR(USART_UDRE_vect) { //UDRE Flag indicates whether the transmit buffer is ready to receive new data
  unsigned char data;

  if (readBuffer(&sendBuffer, &data) == BUFFER_OK) {
    UDR0 = data; //grab the data
  } else {
    UCSR0B &= ~0b00100000; //reset UDRIE0 flag
  }
}

int readSerial(char *buffer)
{
  int count = 0;

  for (count = 0; dataAvailable(&recvBuffer); count += 1) { //read in the data sequentially
    readBuffer(&recvBuffer, (unsigned char*)&buffer[count]);
  }

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  for (int i = 0; i < len; i += 1) {
    writeBuffer(&sendBuffer, buffer[i]);
  }
  UCSR0B |= 0b00100000; //renable UDRIE0 flag
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  DDRD |= 0b01100000;
  DDRB |= 0b110;

  TCNT0 = 0;
  TCCR0A = 0b1;
  TCCR0B = 0b1;
  OCR0A = 0;
  OCR0B = 0;

  TCNT1L = 0;
  TCNT1H = 0;
  TCCR0B = 0b00000001;
  TCCR1B = 0b00000001;
  OCR1AL = 0;
  OCR1AH = 0;
  OCR1BL = 0;
  OCR1BH = 0;
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  TCCR0A |= 0b10100000;
  TCCR1A |= 0b10100000;
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if (speed < 0.0)
    speed = 0;

  if (speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  dir = FORWARD;
  int val = pwmVal(speed);

  if(dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;
    
  newDist = forwardDist + deltaDist;

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  // set PWM compare match(duty cycle)
  LF = val; //LF
  RF = val; //RF
  LR = 0; //LR
  RR = 0; //RR

}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  dir = BACKWARD;
  int val = pwmVal(speed);

  if(dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;
    
  newDist = reverseDist + deltaDist;

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  LR = val;
  RR = val;
  LF = 0;
  RF = 0;
  
}

//New function to estimate number of wheel ticks needed to turn an angle

unsigned long computeDeltaTicks(float ang) {
  //We will assume that angular distance moved = linear distance moved in one wheel revolution. This is (probably) incorrect but simplifies calculation.
  //# of wheel revs to make one full 360 turn is alexCirc / WHEEL_CIRC
  //This is for 360 degrees. For ang degrees it will be (ang * alexCirc) / (360 * WHEEL_CIRC)
  //To convert to ticks, we multiply by COUNTS_PER_REV
  unsigned long ticks = (unsigned long)(ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC);
  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;
  int val = pwmVal(speed);

  if(ang == 0){
    deltaTicks = 9999999;
  }
  else{
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = leftReverseTicksTurns + deltaTicks;

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  LR = val;
  RF = val;
  LF = 0;
  RR = 0;

}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;
  int val = pwmVal(speed);

  if(ang == 0){
    deltaTicks = 9999999;
  }
  else{
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = rightReverseTicksTurns + deltaTicks;
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  RR = val;
  LF = val+10; //offset to account for diff in motors based on our own testing
  LR = 0;
  RF = 0;

}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  RR = 0;
  LF = 0;
  LR = 0;
  RF = 0;
}

/*
   Alex's setup and run codes

*/

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((float) command->params[0], (float) command->params[1]);
      break;
    // Implement code for other commands here
    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_STOP:
      sendOK();
      dbprint("Dist: %ld", getDist());
      stop();
      break;

    case COMMAND_GET_STATS:
      sendOK();
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command->params[0]);
      break;

    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {


        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  }
}

void setupUltrasonic() { //incorporated ultrasonic sensor to calculate distance from front wall as improvement, coded in baremetal
  DDRD |= 0b10000000; //we use pin 7 for the trigger pin
  DDRB &= ~0b00000001; //we use pin 8 for the echo pin
}

void setMSTimer() {
  TCCR2A = 0b00000010; //CTC mode
  TIMSK2 = 0b00000010; //interrupt A
  TCNT2 = 0;
  OCR2A = 16;
  TCCR2B = 0; //we start the timer later
}

volatile long ctr2 = 0; //counter variable

ISR (TIMER2_COMPA_vect) {
  ctr2 += 1;
}

void delayms(int ms) {
  TCCR2B = 0b1; //start timer, prescaler = 1
  while (ctr2 < ms) {} //wait for time to elapse
  TCCR2B = 0; // stop timer
  ctr2 = 0;
}

unsigned long getDist() {
  unsigned long distance;
  unsigned long duration;
  PORTD &= ~0b10000000; //write low to trigger
  delayms(2);

  //write high for 10 microseconds
  PORTD |= 0b10000000; //write high to trigger
  delayms(10);
  PORTD &= ~0b10000000; //write low to trigger

  duration = pulseIn(8, HIGH, 40000);
  distance = duration * 0.034 / 2; //speed of sound assumed to be 340m/s
  return distance;
}

void setup() {
  // put your setup code here, to run once:
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;

  cli();
  setupEINT();
  setupUltrasonic();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  setMSTimer();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK) {
    handlePacket(&recvPacket);
  }
  else if (result == PACKET_BAD) {
    sendBadPacket();
  }
  else if (result == PACKET_CHECKSUM_BAD) {
    sendBadChecksum();
  }

  //Check distance
  if (deltaDist > 0) {
    if (dir == FORWARD) {
      if (forwardDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == BACKWARD) {
      if (reverseDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }
  //Turn angle
  if (deltaTicks > 0) {

    if (dir == LEFT) {

      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == RIGHT) {
      if (rightReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
}
