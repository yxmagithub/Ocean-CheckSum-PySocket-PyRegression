/*
 * Programmer's note: I use #warning to mark the TODO list
 *
 * Receive input from Raspberry Pi and send output to Dive Plan motor controller only.
 * THIS IS DIVE PLAN
 */

//#define UNO
#undef UNO
#define MEGA2560
//#undef MEGA2560

#include <stdint.h>
#ifdef UNO
#include <SoftwareSerial.h>
#endif
#include <E:/MyWorkSpace/ArduinoPrj/Arduino/TecnadyneMotorController/TecnadyneMotorController.ino>
#warning Implement with manual RS485 mode???

// Make this address unique for each Arduino.
const uint8_t ardu_address[]={0xA0,0xA1,0xA2};//A0:rudder,A1:DP, A2:Thruster
uint8_t motorcontrol_rs485_address[]={0x60,0x61,0x62};//60:rudder MC; 61:DP MC; 
uint8_t thruser_ardu_rs485_address=0x80;

// Change these values for each Arduino (Although motor_preferred_pwm should probably stay the same across all Arduinos).
uint8_t motor_address = 0x61;//motor control rs485 address
uint8_t motor_clockwise = 0x00;
uint8_t motor_counterclockwise = 0x01;
//uint8_t motor_preferred_pwm = 0xa0;
uint8_t motor_preferred_pwm = 0x07F;//127 is for 0.5051rpm, 330ms for 1 degree, 
uint8_t motor_state = 0; //0 not running, 1 running

enum ReadState{
  HEADERPT1,
  HEADERPT2,
  ADDRESS,
  COMMAND,
  FOOTERPT1,
  FOOTERPT2,
  CHECKSUM,
  SENDCMD
};

// This is the reading state. As each part of an input string is read, this value is incremented.
ReadState read_state = HEADERPT1;

/*
 * 0x00 - Wait for header pt 1.
 * 0x01 - Wait for header pt 2.
 * 0x02 - Wait for address.
 * 0x03 - Wait for command.
 * 0x04 - Read data. Wait for footer pt 1.
 * 0x05 - Wait for footer pt 2.
 * 0x06 - Wait for checksum.
 */

// Stores the command read from an input string.
uint8_t cmd = 0x00;
uint8_t temp_cmd = 0x00;
/*
 * 0x00 - Do nothing.
 * 0x01 - Reset actuator.
 *        Parameters: none.
 *        Expected reply: none.
 * 0x02 - Move actuator to specified angle.
 *        Parameters:
 *        b1 -> Sign (0 is positive, 1 is negative)
 *        b2 -> Angle (0 to 90)
 *        Expected reply: none.
 * 0x03 - Direction and PWM for actuator.
 *        Parameters:
 *        b1 -> Direction (0x00 or 0x01).
 *        b2 -> PWM (0x00 - 0xfe).
 *        Expected reply: none.
 * 0x04 - Get info.
 *        Parameters: none
 *        Expected reply:
 *        b1-b4 -> Current angle.
 */

/*
 * Example input string from raspi: 10 01 a0 03 00 fe 10 03 c5
 * 10 01: Header.
 * a0: Address of arduino (can range from a0-af).
 * 03: Command (direction and PWM).
 * 00 fe: Data (direction=0x00, PWM is on full power).
 * 10 03: Footer.
 * c5: Checksum.
 */ 

// Stores data read from an input string.
const size_t dta_maxlen = 16; // Max of 16 bytes will be read in.
uint8_t dta_buf[dta_maxlen];
uint8_t temp_dta_buf[dta_maxlen];
size_t dta_len = 0;
size_t temp_dta_len = 0;
bool ready_to_read_dta = false;

// Calculates checksum of incoming bytes one at a time.
uint8_t flying_bcc = 0x00;

// Deal with rotary encoder.
int pinA = 6;  // Connected to CLK on KY-040
int pinB = 7;  // Connected to DT on KY-040
//int encoderPosCount = 0; 
int pinALast = 0;
int aVal = 0;
boolean bCW=true;
float current_angle = 0.0;
float previous_angle = 0.0;
float desiredAngle=0.0;
int cmdFlag = 999;

unsigned long time; //milli second since program beginning.
unsigned long preTime=0;
unsigned int deltaTime=0;
unsigned int counterms=0;

/*
* @brief This is the subroutine to send out the command string 
* @brief Motor Controller
* Function Name: 
* Arguments: 
*/
void setDesiredAngle(){
    // Check motor status if necessary.
    // if(...)
    uint8_t dAngle = 0;
    #warning implement

    // Spin motor if necessary.
    #warning implement properly
    switch(cmd) {
      case 0x01:
        Serial.println("Received reset command. Command not implemented.");
        cmd = 0x00;
        break;

      case 0x02:
        dAngle = dta_buf[0] ? -dta_buf[1] : dta_buf[1];
        desiredAngle = float(dAngle);
        cmdFlag = 2;
        Serial.print("\ndesiredAngle: ");
        Serial.print(desiredAngle, 4);
        break;

      case 0x03:
        Serial.println("Received pwm command.");
        Serial.print("Direction: ");
        Serial.println(dta_buf[0], HEX);
        Serial.print("PWM: ");
        Serial.println(dta_buf[1], HEX);
        break;

      case 0x04:
        Serial.println("Received data request.");
        break;
    }

}

/*startMotorOneTimeatCheckSum()*
*
* only one time start cmd is sent out, and during the checksum
*****************************************************************************************************************************/
void actMotorOneTimeatCheckSum()
{
  if(motor_address == motorcontrol_rs485_address[1])
  {
    if( (motor_state == 0) && (cmdFlag!=999)) 
    {
      if ( (desiredAngle>current_angle) && (abs(desiredAngle - current_angle) >=0.9 ) ) // only 1 deg take effect
      {
        bCW = true;
        setSpeedPWM(motor_address, motor_clockwise, motor_preferred_pwm); 
        Serial.println("\nStart Motor Clockwise");
        motor_state = 1;
        cmdFlag = 999;//clr cmdFlag
        counterms+=1;
      }
      else if( (desiredAngle <current_angle ) && (abs(desiredAngle - current_angle) >=0.9 )) 
      {
        bCW = false;
        setSpeedPWM(motor_address, motor_counterclockwise, motor_preferred_pwm); 
        Serial.println("\nStart Motor CounterClockwise");
        motor_state = 1;
        cmdFlag = 999;//clr cmdFlag
        counterms+=1;
      }
      else{
        cmdFlag= 999;//clr cmdFlag
        read_state = HEADERPT1;
      }
    }
  }
  else
    ;//Serial.println("\n WWWRONG Motor Control rs485 address");
}
/*setup()
*run once
*****************************************************************************************************************************/
void setup()
{
  pinALast = digitalRead(pinA); 
  Serial.begin(57600);
  #ifdef MEGA2560
  Serial1.begin(57600);
  #endif
  //enable the pull up resister
  // pinMode (pinA,INPUT);
  // pinMode (pinB,INPUT);
  pinMode(pinA, INPUT_PULLUP);
  pinMode (pinB,INPUT_PULLUP);
  bCW=true;
  current_angle = 0.0;
  previous_angle = 0.0;
  desiredAngle=0.0;
  counterms = 0;
  cmdFlag = 999;
  size_t dta_len = 0;
  size_t temp_dta_len = 0;
  bool ready_to_read_dta = false;
  uint8_t flying_bcc = 0x00;

}

void loop()
{
  time = millis();
  deltaTime = time - preTime;
  /*  Serial.print("\n\t deltaTime is: ");
  Serial.print(deltaTime, DEC);
  */
  if (Serial1.available() > 0) 
  {
    // read the incoming byte:
    uint8_t in_byte = Serial1.read();
    // Update flying bcc.
    if(read_state != 0x06) {
      uint8_t buf[2];
      buf[0] = flying_bcc;
      buf[1] = in_byte;
      #warning ^ might need to revise that
      flying_bcc = getBCC(buf, 2);
    }
    Serial.print("\nGot byte: ");
    Serial.print(in_byte, HEX);
    Serial.println();
    #warning delete all that

    // Progress through states until an entire command is read.
    switch(read_state) {
      // Wait for first part of header.
      case HEADERPT1:
        //Serial.print("\nSTATE is HEADERPT1");
        if(in_byte == 0x10)
          read_state = HEADERPT2;
        break;

      // Wait for second part of header.
      case HEADERPT2:
        //Serial.print("\nSTATE is HEADERPT2");
        if(in_byte == 0x01)
          read_state = ADDRESS;
        else
          read_state = HEADERPT1;
        break;

      // Wait for address.
      case ADDRESS:
        if(in_byte == ardu_address[1]) {
          // Only proceed to next state if command is addressed to this Arduino.
          read_state = COMMAND;
          motor_address = motorcontrol_rs485_address[in_byte-0xA0];
        }
        else
          read_state = HEADERPT1;
        break;

      // Wait for command.
      case COMMAND:
        //Serial.print("\nSTATE is COMMAND");
        switch(in_byte) {
          // Known commands.
          case 0x01:
          case 0x02:
          case 0x03:
          case 0x04:
            temp_cmd = in_byte;
            read_state = FOOTERPT1;
            temp_dta_len = 0;
            ready_to_read_dta = false;
            break;

          // Command was not recognized.
          default:
            read_state = HEADERPT1;
        }
        break;

      // Read data and wait for footer pt 1.
      case FOOTERPT1:
        //Serial.print("\nSTATE is FOOTERPT1");
        ready_to_read_dta = true;
        if(in_byte == 0x10) {
          read_state = FOOTERPT2;
        }
        else {
          // Do not change states. Store data later.
        }
        break;

      // Wait for footer pt 2.
      case FOOTERPT2:
        Serial.print("\nSTATE is FOOTERPT2");
        if(in_byte == 0x10) {
          // This is an instance of DLE stuffing (0x10 followed by another 0x10), so go back to reading data
          // and ensure that the current in_byte is set to 0x10.
          read_state = FOOTERPT1;
          in_byte = 0x10;
        } else if(in_byte == 0x03) {
          read_state = CHECKSUM;
        } else if(in_byte == 0x01) { // A new command is being sent before the old one has finished processing.
          flying_bcc = 0x11; // Since we are starting a new command, we need to instantiate the flying_bcc.
          read_state = ADDRESS;
        } else {
          read_state = HEADERPT1;
        }
        break;

      // Validate checksum. 
      case CHECKSUM:
        Serial.print("\nSTATE is CHECKSUM");
        #warning just temporary. Delete (true)
        if(true) { //(flying_bcc == in_byte) {
          cmd = temp_cmd;
          for(int i = 0; i < temp_dta_len; ++i) {
            dta_buf[i] = temp_dta_buf[i];
          }
          dta_len = temp_dta_len;
        // Send strings to motor controller.
          //Serial.print("\nWe send the command");
          setDesiredAngle();
          actMotorOneTimeatCheckSum();
        }
        break;
      default:
        read_state = HEADERPT1;//better recover to HEADERRPT1
        break;
    }

    if(read_state == HEADERPT1)
      flying_bcc = 0x00;

    // Store data if needed.
    if(read_state == FOOTERPT1 && ready_to_read_dta) {
      if(temp_dta_len < dta_maxlen) {
        temp_dta_buf[temp_dta_len++] = in_byte;
      }
      else {
        read_state = HEADERPT1;
      }      
      Serial.print("Storing data... ");
      for(int i = 0; i < temp_dta_len; ++i) {
        Serial.print(temp_dta_buf[i], HEX);
        Serial.print(" ");
      }
      Serial.print("\n");
    }
  }
  if(motor_state == 1)
  {
    previous_angle = current_angle;
    if(bCW == true){
      current_angle= current_angle + float(deltaTime)*0.003;
    }
    else if(bCW == false){
      current_angle = current_angle - float(deltaTime)*0.003;
    }
    /*    Serial.print("\n\tCurrent angle: ");
    Serial.print(current_angle, 4);
    Serial.print("\tPrevious angle: ");
    Serial.println(previous_angle, 4);*/
    if (current_angle >= 90.0 || current_angle <= -90.0)
    {
      uint8_t motor_dir = 0x00;
      //current_angle>0 ? motor_dir=0x01 : motor_dir=0x00;
      motor_dir = (bCW == true) ? 0x01 : 0x00;
      setSpeedPWM(motor_address, motor_dir, 0x00);
      Serial.println("\nSSSTOP Motor at Position Limit");
      motor_state =0;
      read_state = HEADERPT1;
      counterms=0;
    }
    else if((current_angle>desiredAngle) && (previous_angle<=desiredAngle))
    {
      setSpeedPWM(motor_address, motor_counterclockwise, 0x00);
      Serial.println("\nSSSTOP Motor Clockwise by CounterClockwise");
      motor_state =0;
      read_state = HEADERPT1;
      counterms=0;
    }
    else if ((current_angle<desiredAngle) &&(previous_angle>=desiredAngle))
    { 
      setSpeedPWM(motor_address, motor_clockwise, 0x00);
      Serial.println("\nSSSTOP Motor CounterClockwise by Clockwise");
      motor_state =0;
      read_state = HEADERPT1;
      counterms=0;
    }
  }
  /*
  *timer event
  *5 seconds
  *****************************************************************************************************************************/
  if ((time%5000)==0) 
  {
    //broadcast the ping message 
    uint8_t buf[] = {0x10, 0x01, 0x6F, 0x71,0x00,0x00,0x00,0x01,0x10,0x03,0x05};
      //Serial.write(buf, len);
    for(int i = 0; i < 11; ++i) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.print("\n");
    Serial.write(buf, 11);
    Serial.print("\n"); 
  }
  preTime = time;
}


