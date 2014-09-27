// DC Motor Closed-Loop PD Control code
// by Dennis Schissler - June, 2009  (further credit below)
// 
// Features:
//    -- Makes use of the L293 motor driver; may easily be adapted to another driver
//    -- Motor pwm'ing implemented by direct control of the ATmega Timer2
//    -- Closed-loop feedback via encoder wheel on the DC motor
//    -- Hardware counter implemented using Timer1 of the ATmega for high-frequency capture of encoder counts
//             - no interrupt necessary!
//    -- PD control system implemented for motor control
//
//
// My main source of info for implementing motor pwm'ing with the L293D was this document:
//   http://www.arduino.cc/playground/Main/DirectionalMotorControlWithAL293D
// The author is not listed so I can't provide specific credit.
// The code is not particularly well-documented so I have attempted to remedy that in my own code below.
//
// Here is a link to the ATmega48/88/168 datasheet.  I happen to be using the ATmega168.
// This resource in invaluable in gaining an understanding of the hardware counters/timers.  
// All page # references below are to this document unless otherwise specified
//   http://www.atmel.com/dyn/resources/prod_documents/doc2545.pdf
//
// Here is a link to the L293D motor driver datasheet
//   http://www.robokitsworld.com/datasheets/l293d.pdf
//
// User 'mem' from the Arduino forum was very helpful in giving me tips on implementing a hardware
// counter for the motor encoder.  This link in particular was exactly what was needed:
//   http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1231326297/all
//
// Other links that were helpful:
//   http://letsmakerobots.com/node/2074
//   http://thecodebender.com/journal/2009/2/21/we-just-cant-leave-things-well-enough-alone.html
//   http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235060559/8#8
//   http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234764073
//   http://abigmagnet.blogspot.com/2008/10/dc-motor-control-part-one.html
//   http://mil.ufl.edu/~achamber/servoPWMfaq.html
//
//
// For the PD control:  the P is for position and the D is for derivative (or velocity).  Other systems also make use of I (integral, which 
//                      gives you a PID controller) but it is not particularly helpful in this application.  You may want to explore it 
//                      for your own application.  Computing the PD gains is left to you.  These are highly dependent upon the specific 
//                      motor, load, encoder resolution, and sampling frequency.  You could just knob twiddle until you find a workable combination.
//                      There are also analytical methods out there for determining these gains.
//
//
// Specific application notes:
//        The code here is tailored for a system that drives a part along a slider rod (linear motion).  In my case, this is done via a 
//        worm gear and a linkage.  I have roughly a 40:1 gear ratio.  A homing move is implemented to find a hard stop at the end of
//        travel.  The part is repeatedly moved away from and then back to the home position.  I added control input for experimental use.
//        Slew distance, acceleration, acceleration ramp - all of these may be specifically tailored to your own application.
//
//        My motor is a smallish DC motor with a 32v supply.  The quadrature encoder provides digital output.  I am using only 1 of the outputs
//        in this case - you could use the 2nd output for additional accuracy.  Note that some encoders output an analog signal.  I used one
//        of these initially and was able to square up the signal using a Schmitt trigger.  However, I had noise coupling problems as soon
//        as any pwm was input to the motor which yielded spurious encoder counts.  I'm not an EE so I abandoned this for the much easier-to-use
//        digital output encoder.  My motor/encoder may be found in some printer and/or scanner products where DC motors are used.
//
//        The Timer1 counter has a 16-bit register so if you expect encoder counts higher than 16 bits, you will need to deal with this in 
//        the code.  My application runs well below this so I did not need to account for it in this code.
//
//        In the motor_forward subroutine, I zero out any positive move errors since in my application all forward moves position
//        the driven part to a hard stop.  You can remove this positive move error check and modification if your application is different.
//
//        I am checking the encoder position every 3ms or 333Hz (sample_freq).  I have placed some debug lines within this sample countdown period to 
//        ensure that we have sufficient processor bandwidth.  It's best to apply control at the highest frequency possible.  If you have other interrupts 
//        or other processes happening during the motor move, you will need to slow down the sample frequency.
//        Note that the encoder counts are being refreshed at the speed of the counter (really fast).  I am only speaking of the frequency
//        by which I am computing position and velocity errors and applying control gains
//
//        My L293D / Arduino / Motor connections are as follows:
//
//              Arduino digital pin 5 to motor encoder output pin
//              Arduino digital pin 11 to L293D pin 7 (motor pwm)
//              Arduino digital pin 12 to L293D pin 2 (motor direction)
//              Motor + pin to L293D pin 6 (pins 3 & 6 may be swapped to flip the motor direction)
//              Motor - pin to L293D pin 3
//              Motor encoder +5v and ground pins suitably connected
//              L293D pin 1 connected to +5v (enable)
//              L293D pins 4,5,12,13 connnected to ground
//              L293D pin 8 connected to +32v (motor power)
//              L293D pin 9 connected to +5v (logic power)
//              Filter caps added per:  http://letsmakerobots.com/node/2074


#include <avr/io.h>  

#define MOTOR_DIR 12                            // Non PWM pin for direction control
#define MOTOR_PWM 11                            // PWM controlled pin.  Pin 11 must be used since we are using Timer2 for pwm control
#define ENCODER_READ 5                          // The encoder digital output must be connected to pin 5 for Timer/Counter1

unsigned int encoderPos;                        // used to track the ABSOLUTE encoder position
unsigned long next_speed_check_time;            // keep track of the next sample measurement time  
int encoder_end_pos;                            // the last actual encoder position sampled
int move_error;                                 // used to account for previous move errors
float last_ref_pos;                             // the previous reference encoder position for the PD calculation
float last_ref_vel;                             // the previous reference velocity for the PD calculation
int pwm = 0;                                    // motor pwm
float accel_decel;                              // the ramp acceleration and deceleration in counts/sample^2
int accel_decel_ramp;                           // the acceleration & deceleration ramp distance in samples
int slew_distance;                              // the slew distance in samples
float KP;                                       // position gain for the PD calculation
float KD;                                       // derivative gain for the PD calculation
int enc_pos_target; // the expected encoder movement during the specified move
int homing = 0;                                 // used to track if we are doing a homing move
	                                            // 0 = no homing move; 1 = doing a homing move; 2 = completed homing move
int home_count = 0;                             // used to track the number of samples of no movement for homing move
int inByte = 0;                                 // variable to hold incoming serial data for keypress


void setup() {  
  Serial.begin(19200);

  CLKPR=0;                                   // set the clock prescale register to 0, p. 35-36

  pinMode(ENCODER_READ, INPUT);              // initialize ENCODER_READ pin as an INPUT pin; used for digital encoder reads
  pinMode(MOTOR_DIR, OUTPUT);     		   // initialize MOTOR_DIR pin as an OUTPUT pin; used to control the motor direction
  pinMode(MOTOR_PWM, OUTPUT);     		   // initialize MOTOR_PWM pin as an OUTPUT pin; used to control the motor pwm

  // initialize the OUTPUT's to a LOW value
  digitalWrite(MOTOR_DIR, LOW);
  digitalWrite(MOTOR_PWM, LOW);

  // Beware:  Here there be dragons...
  //
  // There are 2 registers for Timer2 that need to be initialized:  TCCR2A and TCCR2B
  //
  // Use phase correct PWM Mode and set Output Compare mode
  // PWM fequency is ~30kHz
  TCCR2A = (0<<WGM21) | (1<<WGM20) | (0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0);

  // See p. 154 in Atmel datasheet, WGM21 and WGM22 control the counting sequence of the counter,
  // the source for maximum (TOP) counter value, and what type of waveform generation to be used 
  // 
  // Per Table 15-8 on p. 155, the following settings:
  // WGM20 = 1
  // WGM21 = 0
  // WGM22 = 0 (set below in TCCR2B)
  // set "PWM, Phase Correct" mode
  // P. 147, In Phase Correct mode, the counter counts up from BOTTOM to the OCR2A value (max = 255) and then from OCR2A back down to 
  // the BOTTOM.  This dual-slope operation has lower maximum frequency than the single slope operation (called Fast PWM mode)
  // but is better for motor control applications.

  // p. 152 in Atmel datasheet, set:
  // COM2A1 = 0
  // COM2A0 = 0
  // for Normal port operation, OC2A disconnected

  // p. 153 in Atmel datasheet, set:
  // COM2B1 = 0
  // COM2B0 = 0
  // for Normal port operation, OC2B disconnected

  TCCR2B = (0<<WGM22) | (0<<CS22) | (0<<CS21) | (1<<CS20) | (0<<FOC2A) | (0<<FOC2B);

  // p. 144 in Atmel datasheet, WGM22=0 for "Normal" mode
  // Also per Table 15-8 on p. 155, this is set to 0 combined with above
  //    WGM20=1 & WGM21=0 settings to set "PWM, Phase Correct" mode. 

  // p. 155 in Atmel datasheet, FOC2A = 0 when TCCR2B is written when operating in PWM mode.  This is Force Output Compare A
  // p. 155 in Atmel datasheet, FOC2B = 0 when TCCR2B is written when operating in PWM mode.  This is Force Output Compare B

  // p. 156 in Atmel datasheet, to set "no prescaling", the following bits must be set as follows:
  // CS22 = 0 
  // CS21 = 0
  // CS20 = 1

  counterStart();           			 // initialize the Timer1 counter for tracking the encoder counts
  
  Serial.println("Press '6' to go forwards, '4' to go backwards, and '5' to home");

}  


void loop() 
{  

  // check if there are any incoming serial bytes available to read
  if (Serial.available() > 0) 
  {  
	// then read the first available byte
	inByte = Serial.read();

	switch (inByte) 
	{
	case 54:                             // User pressed '6', move forward
	  TCNT1 = 0;                         // set the encoder counting hardware register to 0
	                                     // need to reset since we count up for both motor direction moves
	  slew_distance = 47;                // set the slew distance in samples
	  motor_forward();                   // turn the motor forward 
	  break;  

	case 52:                             // User pressed '4', move backward
	  TCNT1 = 0;                         // set the encoder counting hardware register to 0
	                                     // need to reset since we count up for both motor direction moves
	  slew_distance = 47;                // set the slew distance in samples
	  motor_backward();                  // turn the motor backward
	  break;  

	case 53:                             // User pressed '5', home
	  TCNT1 = 0;                         // set the encoder counting hardware register to 0
	                                     // need to reset since we count up for both motor direction moves
	  encoderPos = 2000;                 // set a default absolute encoder position; want to ensure non-negative values
	  motor_home();                      // home the motor
	  break;  

	default:  
	  Serial.print(inByte, BYTE);
	  Serial.println(" is not a valid key");

	}
  }
}


void motor_backward() 
{
  accel_decel = 0.25;                // the ramp acceleration and deceleration in counts/sample^2
  accel_decel_ramp = 25;             // set the acceleration & deceleration ramp distance in samples
  KP = 2.88;                         // set position gain; my values - yours will very likely be different
  KD = 15.86;                        // set derivative gain; my values - yours will very likely be different
  float accel_decel_dist = 81.25;    // the expected encoder movement during accel and decel; computed offline via spreadsheet

  OCR2A=0;                           // zero the pwm 
  // OCR2A explanation:
  //
  // First of all, the '2' in OCR2A is for Timer2 which is the timer we use for pwm control
  // on p. 140 of Atmel datasheet, OCR2A is the output compare register (8-bit)
  // it is compared with the timer/counter value at all times.  The result of the 
  // compare can be used by the waveform generator to generate a PWM or variable frequency output
  // on the Output Compare pin (OC2A)
  //
  // This sets the TOP of the counter
  // The counter length effectively sets the PWM frequency
  //
  // Computing PWM Frequency:  see p. 148
  // In this setup the prescaler is set to 1.  The clock runs at 16MHz.  The formula for computing the PWM frequency is:
  // F(pwm) = F(clk)/(N*510)
  // where N is the prescale
  // In this case F(pwm) = 16000000/(1*510) = 31373 KHz
  // General consensus is that motor PWM's should be at least 20KHz so we should be OK
  //
  // OCR2A sets the duty cycle.  If it were 128 then the duty cycle would be 50%.  We are basically just adjusting how high
  // the timer counts up and thus affecting the duty cycle.

  encoder_end_pos = getCount();        // get the current encoder position; to be used for PD control
  last_ref_pos = encoder_end_pos;      // initialize the 'last' reference position to the current encoder position
  last_ref_vel = 0;                    // initialize the 'last' velocity reference to 0

  // subtract accel, decel, and slew distances from the current position, and add any previous move error 
  // to determine the expected final position
  enc_pos_target = encoderPos - (2 * accel_decel_dist) - ((slew_distance - 1) * (accel_decel * accel_decel_ramp)) + move_error;  
  
  Serial.print("Encoder position target for the move is: ");
  Serial.println(enc_pos_target - move_error); 
  
  // Set the direction bit to the correct value
  digitalWrite(MOTOR_DIR, HIGH);

  // Set output to PWM.  Since we are using "Compare Output Mode, Phase Correct PWM Mode", table 15-4 applies, p. 153
  // See p. 147 for additional info on Phase Correct PWM Mode
  TCCR2A |= ((1<<COM2A1) | (1<<COM2A0));
  // p. 153 of Atmel datasheet, Table 15-4:
  // COM2A1 = 1
  // COM2A0 = 1
  // This sets OC2A on Compare Match when up-counting and clears OC2A on Compare Match when down-counting.
  // This is considered "inverting Compare Output mode"

  next_speed_check_time = millis();    // initialize the frequency timer to the current time
  encoder_end_pos = getCount();        // get the current encoder position; to be used for PD control
  last_ref_pos = encoder_end_pos;      // initialize the 'last' reference position to the current encoder position
  last_ref_vel = 0;                    // initialize the 'last' velocity reference to 0

  // Do Acceleration Ramp

  adjust_pwm(accel_decel, accel_decel_ramp);  

  // Do Slew

  adjust_pwm(0, slew_distance);

  // Do Deceleration Ramp  

  adjust_pwm(-accel_decel, accel_decel_ramp);  

  // Stop the motor 

  motor_stop();

  encoderPos = encoderPos - getCount();       // update the absolute encoder position; subtracting since moving backward
  move_error = enc_pos_target - encoderPos;   // compute the move error; positive means we moved too far
  
  Serial.print("Move error was: ");
  Serial.println(move_error);

  Serial.print("End encoder position:");
  Serial.println(encoderPos);
  Serial.println();
}


void motor_forward() 
{
  accel_decel = 0.25;                  // the ramp acceleration and deceleration in counts/sample^2
  accel_decel_ramp = 25;               // set the acceleration & deceleration ramp distance in samples
  KP = 2.88;                           // set position gain; my values - yours will very likely be different
  KD = 15.86;                          // set derivative gain; my values - yours will very likely be different
  float accel_decel_dist = 81.25;      // the expected encoder movement during accel and decel; computed offline via spreadsheet

  OCR2A=0;                             // zero the pwm; see motor_backward subroutine for the OCR2A explanation

  encoder_end_pos = getCount();        // get the current encoder position; to be used for PD control
  last_ref_pos = encoder_end_pos;      // initialize the 'last' reference position to the current encoder position
  last_ref_vel = 0;                    // initialize the 'last' velocity reference to 0

  // add accel, decel, slew distances, and any previous move error to the current position to determine the expected final position
  enc_pos_target = encoderPos + (2 * accel_decel_dist) + ((slew_distance - 1) * (accel_decel * accel_decel_ramp)) + move_error;  
  
  Serial.print("Encoder position target for the move is: ");
  Serial.println(enc_pos_target - move_error); 
  
  // Set the direction bit to the correct value
  digitalWrite(MOTOR_DIR, LOW);

  // Since we are using "Compare Output Mode, Phase Correct PWM Mode", table 15-4 applies, p. 153
  // See p. 147 for additional info on Phase Correct PWM Mode
  // Set output to PWM (inverted of motor_backward function)
  TCCR2A |= ((1<<COM2A1) | (0<<COM2A0));

  // p. 153 of Atmel datasheet, Table 15-4:
  // COM2A1 = 1
  // COM2A0 = 0
  // This clears OC2A on Compare Match when up-counting and sets OC2A on Compare Match when down-counting.
  // This is considered "non-inverting Compare Output mode", p. 147

  next_speed_check_time = millis();    	   // initialize the frequency timer to the current time

  // Do Acceleration Ramp

  adjust_pwm(accel_decel, accel_decel_ramp);  

  // Do Slew

  adjust_pwm(0, slew_distance);

  // Do Deceleration Ramp  

  adjust_pwm(-accel_decel, accel_decel_ramp);  

  // Stop the motor 

  motor_stop();

  encoderPos = encoderPos + getCount();       // update the absolute encoder position; adding since moving forward
  move_error = encoderPos - enc_pos_target;   // compute the move error; positive means we moved too far
  
  Serial.print("Move error was: ");
  Serial.println(move_error);  
  
  if (move_error > 0)                         // In this application, the forward move always goes to home
	                                          // Since home is a solid wall, it's not possible to move beyond it
	                                          // and thus we need to zero out any move error.  These positive errors may
	                                          // be a result of backlash or possibly some problem with counting
	                                          // We may want to consider resetting the encoderPos as well (to 2000).  I'm not sure
  {
	move_error = 0;
	Serial.println("We hit the wall");
  }
  
  Serial.print("End encoder position:");
  Serial.println(encoderPos);
  Serial.println();
}


void motor_home()                    // homing against a hard stop
{
  accel_decel = 0.08;                // the ramp acceleration and deceleration in counts/sample^2; move slow for homing
  accel_decel_ramp = 25;             // set the acceleration & deceleration ramp distance in samples
  slew_distance = 200;               // set the maximum slew distance in samples for the homing move
  KP = 0.96;                         // set position gain; soft gain for homing
  KD = 5.29;                         // set derivative gain; soft gain for homing
  homing = 1;                        // signal that we are performing a homing move
  encoder_end_pos = 2000;            // initialize the encoder position to a safe positive value
  OCR2A=0;                           // zero the pwm;  see motor_backward subroutine for the OCR2A explanation

  // Set the directional bit to the correct value
  digitalWrite(MOTOR_DIR, LOW);

  // Set output to PWM (same as forward move, since we always move forward for homing)
  TCCR2A |= ((1<<COM2A1) | (0<<COM2A0));

  // p. 153 of Atmel datasheet, Table 15-4:
  // COM2A1 = 1
  // COM2A0 = 0
  // This clears OC2A on Compare Match when up-counting and sets OC2A on Compare Match when down-counting.
  // This is considered "non-inverting Compare Output mode", p. 147

  next_speed_check_time = millis();    // initialize the frequency timer to the current time
  encoder_end_pos = getCount();        // get the current encoder position; to be used for PD control
  last_ref_pos = encoder_end_pos;      // initialize the 'last' reference position to the current encoder position
  last_ref_vel = 0;                    // initialize the 'last' velocity reference to 0

  // Do Acceleration Ramp

  adjust_pwm(accel_decel, accel_decel_ramp);  

  if (homing < 2)                      // check if we've completed home move; '2' = finished
  {
	// Do Slew

	adjust_pwm(0, slew_distance);
  }

  motor_stop();

  if (homing == 2)  
  {
	encoderPos = 2000;                  // set the home position to absolute encoder position 2000; helps to avoid negative positions
	Serial.println("Home Found");
  }
  else Serial.println("WARNING!!!  DID NOT FIND HOME!!!");

  homing = 0;                           // reset the homing flag
  home_count = 0;                       // reset the homing count flag
  move_error = 0;                       // zero out any previous move error
}


void motor_stop() 
{
  // Disconnect the PWM
  TCCR2A &= ~((1<<COM2A1) | (1<<COM2A0));
  // this format clears COM2A1 and COM2A0, disconnecting OC2A; see table 15-4, p. 153

  pwm = 0;                               // reset the pwm variable to 0
  OCR2A = 0; 							// reset the pwm register to 0

  // Put the motor control outputs to a safe 'LOW'
  digitalWrite(MOTOR_DIR, LOW);
  digitalWrite(MOTOR_PWM, LOW);  
}


// perform the slew or ramp motor move, applying PD gain control
void adjust_pwm(float acceleration, int control_distance)         // acceleration in samples/sec^2; control_distance is in # samples
{
  // The PD correction formula is:
  // pwm correction = Kp * poserror + Kd * velerror

  float poserror;                                   // the computed position error
  float velerror;                                   // the computed velocity error
  float ref_vel;                                    // the reference velocity
  float ref_pos;                                    // the reference position
  int encoder_start_pos;                            // the starting encoder position for this sample
  int sample_freq = 5;                              // sample frequency in ms; OK frequency is actually 1/time, so in this case 200Hz
  int sample_count = 1;                             // tracks the number of samples
  int home_count_max = 20;                          // used for homing; the number of zero-movement samples required to say we've homed
  int interval = 0;  							   // the end time for the sample period
  int cycle = 0;                                    // Debug variable to check that there is sufficient bandwidth for the given sample frequency

  while (sample_count <= control_distance)  		// Keep the move going for the length of the control distance
  {      
	OCR2A = pwm;                                    // Write the current pwm to the pwm register
	encoder_start_pos = encoder_end_pos;            // store the current encoder position for the next sample point
	next_speed_check_time = next_speed_check_time + sample_freq; // compute the next sample time based on the sample frequency

	// Now we sample at the sample frequency

	interval = next_speed_check_time - millis();    // compute the end time for this sample period

	while (interval > 0)                            // hang out until the sample period has elapsed
	{
	  cycle = cycle + 1;                            // debug used to check if we have enough processor bandwidth for given sample freq
	  interval = next_speed_check_time - millis();  // compute the end time for this sample period
	}

	if (cycle < 2)                                  // check on the processor bandwidth for the given sample freq; if less than 2 cycles than we are cutting it too close
	{
	  Serial.print("WARNING:  Cycles = ");
	  Serial.println(cycle);
	}
	cycle = 0;                                      // reset the cycle debug variable

	encoder_end_pos = getCount();                   // get the current encoder position
	sample_count = sample_count + 1;                // increment the number of samples

	// Check for the special case of a homing move
	if (homing > 0)                                 // homing = 1 or 2 means we are homing
	{
	  if (encoder_end_pos == encoder_start_pos)     // check if the encoder has moved since the last sample point
	  {  
	    if (encoder_end_pos != 2000)                // don't include the startup accel (where there is no movement) as part of the calc
	    {        
	      home_count++;                             // increment the number of times we've seen no movement
	      Serial.println(home_count);
	      if (home_count > home_count_max)          // check if we've met the required number of zero-movement samples to say we've reached the home position
	      {
	        homing = 2;            				 // movement has ceased sufficiently so we've found the home position; '2' signals homing complete
	        break;    // break out of the 'while (sample_count < control_distance)' loop back to the motor_home routine
	      }
	    }
	  }
	}

	ref_vel = last_ref_vel + acceleration;          // compute the current reference velocity for the sample
	last_ref_vel = ref_vel;                         // store the current reference velocity for the next sample point
	ref_pos = ref_vel + last_ref_pos;               // compute the current reference position for the sample
	last_ref_pos = ref_pos;                         // store the current reference position for the next sample point

	//Serial.println(ref_pos);
	//Serial.println(encoder_end_pos);

	velerror = ref_vel - (encoder_end_pos - encoder_start_pos);   // compute the velocity error for the PD calculation
	poserror = ref_pos - encoder_end_pos;  					   // compute the position error for the PD calculation

	pwm = pwm + (KP * poserror) + (KD * velerror);                // update the pwm based on the PD calculation   

	//Serial.print("PWM = ");
	//Serial.println(pwm);

	if (pwm > 255) pwm = 255;                  	// clamp maximum pwm to 255
	if (pwm < 0) pwm = 0;                      	// make sure pwm's don't go negative
  }
}


// call this to initialize the counter
void counterStart()
{
  // hardware counter setup, see p. 107 for info on the 16-bit Timer1 Timer/Counter
  TCCR1A=0;                              // reset timer/countern control register A
  TCCR1B=0;                              // reset timer/countern control register B
  TCNT1=0;                               // initialize the counter value to 0; this register holds the current count
  
  // set timer/counter1 hardware as a counter; it counts events on pin Tn (Arduino pin 5)
  // normal mode, wgm10 .. wgm13 = 0, see p. 131, table 13-4
  TCCR1B = TCCR1B | 7; // Counter Clock source = pin Tn (Arduino pin 5) , start counting now
  // 7 in binary is 0111; OR-ing will set CS10,11,12 to 1's
  // External clock source on Tn pin. Clock on rising edge., see table 13-5, p. 132
}


// call this to get the current count
unsigned int getCount()
{
  unsigned int count;      					// this variable returns the current encoder count from the counting register
  TCCR1B = TCCR1B & ~7;        				// Gate Off  / Counter Tn stopped, see table 13-5, p. 132; we need to disable counting just prior to our read
	                                           // this operation clears the bits (CS10,11,12)
 
  count = TCNT1;                               // read the counting register
  TCCR1B = TCCR1B | 7;      				   // re-start counting by resetting the bits (CS10,11,12)
  return count;                                // return the retreived count to the calling function
}


