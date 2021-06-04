const byte numChars = 32;       //Number of characters for received  
char receivedChars[numChars];   // an array to store the received data

static int index = 0; //Used to determine the index of characters in the array for turning them into an integer type
long int val = 0;  //Current set up in showNewNumber prevents outside of -90000 and 90000

boolean Move_flag = false; //Determines whether a actuator is able to move
boolean neg_flag = false;  //Used to determine if the serial input is negative
boolean newData = false;  //Used to determine whether newData had been entered into the serial monitor
boolean EndMark_flag = false; //Used to determine if the end of the serial input is detected

boolean X_flag = false;  //Actuator selection flag for X axis
boolean Y_flag = false;  //Actuator selection flag for Y axis
boolean test_flag = false; //Selection flag for test case 

boolean NEGnum_flag = false; //Used to determine which direction the actuators move

long int dataNumber = 0; //Used for taking serial monitor input and moving the actuator to a new encoder location
int character = 0;  //Used for Actuator Axis selection, X and Y only.
long int convert = 0.919; //number of encoder counts per micron used for PPC mode 
long int correction = 0; //correction value to allow for negative overshoot 

int state = 0;  //State of the system (Zeroing, Joystick, PPC)

//Joystick Analog reading pins
const int VRxpin = A0;
const int VRypin = A1;

//Joystick Switch Pin
const int SWpin = 2;

//850 G on 1-8 side of L293D
int en1 = 8;      //Enables Pulse Width Modulation (PWM) to control the speed of the motors, Pin 1 on L293D
int in1 = 51;     //First Driver Input for HIGH/LOW inputs from Arduino, Pin 2 on L293D
int in2 = 50;     //Second Driver Input for HIGH/LOW inputs from Arduino, Pin 7 on L293D
int revlimA = 22; //Reverse Limit Switch for Actuator A, terminal 18 
int forlimA = 23; //Forward Limit Switch for Actuator A, terminal 17
static int pinAA = 20; //Encoder Channel A for Actuator A, terminal 19. Used for hardware interrupts
static int pinAB = 21; //Encoder Channel B for Actuator A, terminal 20. Used for hardware interrupts
volatile long int EncAencoderPos = 0; //Variable value for current encoder reading. Provides signed reading of a 32-bit number

//850 G on 9-16 side of L293D
int en2 = 9;      //Enables Pulse Width Modulation (PWM) to control the speed of the motors, Pin 9 on L293D
int in3 = 52;     //First Driver Input for HIGH/LOW inputs from Arduino, Pin 10 on L293D
int in4 = 53;     //Second Driver Input for HIGH/LOW inputs from Arduino, Pin 15 on L293D
int revlimB = 24; //Reverse Limit Switch for Actuator B, terminal 18
int forlimB = 25; //Forward Limit Switch for Actuator B, terminal 17
static int pinBA = 18; //Encoder Channel A for Actuator B, terminal 19. Used for hardware interrupts
static int pinBB = 19; //Encoder Channel B for Actuator B, terminal 20. Used for hardware interrupts
volatile long int EncBencoderPos = 0; //Variable value for current encoder reading. Provides signed reading of a 32-bit number

//---------------------Actuator A Channel A Encoder ------------------------

void AencChA(){
/* Description: Counts each time Actuator A’s encoder triggers a change in 
 *             its Channel A’s voltage
 */
  cli(); //stop interrupts happening before we read pin values
  if (digitalRead(pinAA) == HIGH){
    if(digitalRead(pinAB) == LOW){
      EncAencoderPos --; //Moving In, index - 1
      sei();             //restart interrupts
    }
    else{
      EncAencoderPos ++; //Moving Out, index + 1
      sei();             //restart interrupts
    }
  }
  else{
    if(digitalRead(pinAB) == HIGH){
      EncAencoderPos --; //Moving In, index - 1
      sei();             //restart interrupts
    }
    else {
      EncAencoderPos ++; //Moving Out, index + 1
      sei();             //restart interrupts
    }
  }
}

//---------------------Actuator A Channel B Encoder ------------------------

void AencChB(){
/* Description: Counts each time Actuator A’s encoder triggers a change in 
 *             its Channel B’s voltage
 */
  cli(); //stop interrupts happening before we read pin values
  if (digitalRead(pinAB) == HIGH){
    if(digitalRead(pinAA) == HIGH){
      EncAencoderPos --;  //Moving In, index - 1
      sei();              //restart interrupts
    }
    else{
      EncAencoderPos ++;  //Moving Out, index + 1
      sei();              //restart interrupts
    }
  }
  else{
    if(digitalRead(pinAA) == LOW){
      EncAencoderPos --; //Moving In, index - 1
      sei();             //restart interrupts
    }
    else {
      EncAencoderPos ++;  //Moving Out, index + 1
      sei();              //restart interrupts
    }
  }
}

//---------------------Actuator B Channel A Encoder ------------------------

void BencChA(){
/* Description: Counts each time Actuator B’s encoder triggers a change in 
 *             its Channel A’s voltage 
 */ 
  cli(); //stop interrupts happening before we read pin values
  if (digitalRead(pinBA) == HIGH){
    if(digitalRead(pinBB) == LOW){
      EncBencoderPos --; //Moving In, index - 1
      sei();             //restart interrupts
    }
    else{
      EncBencoderPos ++;  //Moving Out, index + 1
      sei();              //restart interrupts
    }
  }
  else{
    if(digitalRead(pinBB) == HIGH){
      EncBencoderPos --; //Moving In, index - 1
      sei();             //restart interrupts
    }
    else {
      EncBencoderPos ++; //Moving Out, index + 1
      sei();             //restart interrupts
    }
  }
}

//---------------------Actuator B Channel B Encoder ------------------------

void BencChB(){
/* Description: Counts each time Actuator B’s encoder triggers a change in 
 *             its Channel B’s voltage 
 */
  cli();//stop interrupts happening before we read pin values
  if (digitalRead(pinBB) == HIGH){
    if(digitalRead(pinBA) == HIGH){
      EncBencoderPos --;  //Moving In, index - 1
      sei();              //restart interrupts
    }
    else{
      EncBencoderPos ++;  //Moving Out, index + 1
      sei();              //restart interrupts
    }
  }
  else{
    if(digitalRead(pinBA) == LOW){
      EncBencoderPos --; //Moving In, index - 1
      sei();             //restart interrupts
    }
    else {
      EncBencoderPos ++; //Moving Out, index + 1
      sei();             //restart interrupts
    }
  }
}

//---------------------Actuator A Basic Movement Functions ------------------------

void MoveAOut(int speed){  
/* Description: Moves Actuator A out at a set speed determined by the Speed 
 *             Differential function
 */
  analogWrite(en1, speed); //Writes to Enable 1 pin 8 bit value for speed
  digitalWrite(in1, LOW);  //Writes Input 1 a low value
  digitalWrite(in2, HIGH); //Writes Input 2 a high value
}

void MoveAIn(int speed){   
/*Description: Moves Actuator A in at a set speed determined by the Speed 
 *             Differential function
 */
  analogWrite(en1, speed); //Writes to Enable 1 pin 8 bit value for speed 
  digitalWrite(in1, HIGH); //Writes Input 1 a high value
  digitalWrite(in2, LOW);  //Writes Input 2 a low value
}

void StopA(){  
/* Description: Stops actuator motion by with a preset zero for the speed
 */  
  //Stops movement for Actuator A
  analogWrite(en1, 0);     //Enable 1 pin 8 bit value for speed set to zero
  digitalWrite(in1, LOW);  //Writes Input 1 a low value
  digitalWrite(in2, LOW);  //Writes Input 2 a low value
}

//---------------------Actuator B Basic Movement Functions ------------------------

void MoveBOut(int speed){
/* Description: Moves Actuator B out at a set speed determined by the Speed 
 *             Differential function
 */
  analogWrite(en2, speed); //Writes to Enable 2 pin 8 bit value for speed
  digitalWrite(in3, LOW);  //Writes Input 3 a low value
  digitalWrite(in4, HIGH); //Writes Input 4 a high value
}

void MoveBIn(int speed){
/* Description: Moves Actuator B in at a set speed determined by the Speed 
 *             Differential function
 */
  analogWrite(en2, speed); //Writes to Enable 2 pin 8 bit value for speed 
  digitalWrite(in3, HIGH); //Writes Input 3 a high value
  digitalWrite(in4, LOW);  //Writes Input 4 a low value
}

void StopB(){
/* Description: Stops actuator motion by with a preset zero for the speed
 */  
  analogWrite(en2, 0);     //Enable 2 pin 8 bit value for speed set to zero
  digitalWrite(in3, LOW);  //Writes Input 3 a low value
  digitalWrite(in4, LOW);  //Writes Input 4 a low value
}

void setup() {
  // set up Joystick pins for input
  pinMode(VRxpin, INPUT);
  pinMode(VRypin, INPUT);
  pinMode(SWpin, INPUT);
  digitalWrite(SWpin, HIGH);
    
  //Set up pins for Actuator A for output
  pinMode(en1, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  //Set up pins for Actuator B for output
  pinMode(en2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //Set up limit switches for both Actuators
  pinMode(forlimA, INPUT_PULLUP);
  pinMode(revlimA, INPUT_PULLUP);
  pinMode(forlimB, INPUT_PULLUP);
  pinMode(revlimB, INPUT_PULLUP);
  
  //Actuator A, Encoder Output
  pinMode(pinAA,INPUT_PULLUP);
  pinMode(pinAB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinAA),AencChA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinAB),AencChB,CHANGE);

  //Actuator B, Encoder Output
  pinMode(pinBA,INPUT_PULLUP);
  pinMode(pinBB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinBA),BencChA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinBB),BencChB,CHANGE);
  
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");
}

void loop() {
    int SW_read;   //Variable to read switch pin
    int X_dir = 0; //Variable for Joystick in X direction
    int Y_dir = 0; //Variable for Joystick in Y direction
    
    SW_read = digitalRead(SWpin); //Read switch pins digital input
    stateChange(SW_read,state);  //Check switch to alter state   
    if(SW_read == LOW){           //If Switch button is pressed
        while(SW_read == LOW){    //While the button is pressed
          SW_read = digitalRead(SWpin);  //Read the switch pin
          if(SW_read == HIGH){    //If the button is released
            break;                //Break the loop and continue with code
          }
        }
    }
    switch (state){
      case 0:
          Serial.println("Initial State: Click Enter to start Zeroing the actuators");
          while (Serial.available() == 0){} //Wait till Serial Monitor detects an input
          Serial.read();               //Remove newline created when clicking enter
          Serial.print("Initializing the X Axis Actuator\n");
          XZeroing(revlimB,forlimB);  //Zeroing the X axis while monitoring both limit switches
          Serial.print("Initializing the Y Axis Actuator\n");
          YZeroing(revlimA,forlimA);  //Zeroing the Y axis while monitoring both limit switches
          state++;                    //Increase state variable to move to case 1 (Joystick)
          Serial.println("Done, IFM in Joystick state");
          Serial.println("To change to programable path mode click the joystick");
          delay(50);
      break;

      case 1:
          //Change Satblock to act as a flag instead
          X_dir = SatBlock(analogRead(VRypin)); //Reads the analog input from VRy and put it through a Saturation Block`
          Y_dir = SatBlock(analogRead(VRxpin)); //Reads the analog input from VRx and put it through a Saturation Block
          //Serial.println(X_dir);
          //Serial.println(Y_dir);
          LimitSwitchA(revlimA,forlimA,Y_dir);  //Moves Actuator A (Y_axis) while checking for limit switch activation
          LimitSwitchB(revlimB,forlimB,-X_dir);  //Moves Actuator B (X_axis) while checking for limit switch activation
          delay(50);
      break;
            
      case 2:
          StopA();
          StopB();
          InitPPCselect();
          if (X_flag == true){  //If X selected
              Serial.println("Enter a target value for X axis in Microns");
              while (Serial.available() == 0){}  //Wait till Serial Monitor detects an input
              MoveXEncoder();
          }
          else if(Y_flag == true){  //If Y selected
              Serial.println("Enter a target value for Y axis in Microns");
              while (Serial.available() == 0){}  //Wait till Serial Monitor detects an input
              MoveYEncoder();
          }
          else if(test_flag == true){ //if in test mode is selected
              Serial.println("Select an Actuator to test");
              InitPPCselect();
                if(X_flag == true){ //if x actuator has been selected
                  testXactuator();
                }
                else if(Y_flag == true){ //if Y actuator has been selected
                  testYactuator();
                }
          }
          delay(50);
      break;
    }
}

void stateChange(int sw, int s){
/* Description: Checks if the joystick switch has been activated and which 
 *              state the Arduino is currently in to switch to the next mode. 
 *              The inputs are the Switch read value (int sw) and the current 3
 *              state (int s).
 */  
  if (sw == LOW){
      if(s == 0){
        state++;  //Add 1 to state to move to Joystick Case
        Serial.println("Joystick Case");
        delay(300); 
      }
      else if(s == 1){
        state++;  //Add 1 to state to move to PPC Case
        Serial.println("Path Code Case");
        //Display current encoder counts on switch
        delay(300);
      }
      else if(s == 2){
        state--;  //Subtract 1 from state to move to Joystick Case
        Serial.println("Joystick Case");
        delay(300);
      }
  }
}

void InitPPCselect(){
/* Description: Waits for an Actuator to be selected in the Programmable Path mode. 
 *              This function also allows for the joystick switch to be activated 
 *              in order for the user to switch to the Joystick mode.
 */  
  int SW_read;        //Variable to store switch read (HIGH/LOW)
  int switch_flag = 0; //Flag activated during switch activation
  
  Serial.println("Select Actuator to move (X/Y)or click the joystick to enter joystick control mode");
  while (Serial.available() == 0){  //While waiting for an input to be detected
      SW_read = digitalRead(SWpin); //Read switch pins digital input
      if(SW_read == LOW){           //If Switch button is pressed
        Serial.print("Switching to Joystick State\n"); 
        state = 2;         //Change to Joystick state
        switch_flag = 1; //Activate switch flag
        break;
        while(SW_read == LOW){  //While the button is pressed
          SW_read = digitalRead(SWpin); //Read the button pin
          if(SW_read == HIGH){  //If the switch reads a high
            break;   //Break the loop
          }
        }
        break; //Break the loop
      }
  }
  if (switch_flag == 0){ //If the switch is not activated and serial monitor detected an input
    while (newData != true){  //If newData has not been detected
      recvWithEndMarker();  //Cycle through to read next character from serial port
    }  
    selectActuator();  //Determine Actuator selected
    if (character == 0x58 || character == 0x78){  //If character is equal to ASCCI "X"
      X_flag = true;   //Set X_flag to move to target step
    }
    else if(character == 0x59 ||character == 0x79){  //If character is equal to ASCCI "Y"
      Y_flag = true;   //Set Y_flag to move to target step
    }
    else if(character == 0x54 ||character == 0x74){  //If character is equal to ASCCI "T"
      test_flag = true;  // Set test_flag to move to test mode
    }
  }
  else{
    switch_flag = 0;
  }
}

//---------------------General Actuator Movement Functions------------------

void YZeroing(int rlimsw, int flimsw){
/* Description: Zeros the Y axis actuator (Actuator A) while monitoring both
 *              limit switches. It uses the functions MoveAOutToTarget and 
 *              MoveAIn.
 */
  int x = 0;      //Variable for reading LimitSwitch
  long int y = 0; //Variable for encoder target input
  x = digitalRead(rlimsw);   //Read limit switch output
  while (x == LOW){          //While the reverse limit is not activated
    x = digitalRead(rlimsw); //Read limit switch output
    if(x == LOW){            //If the reverse limit switch is not activated
      MoveAIn(175);          //Move A at a set speed -----Change if Actuator B is moving faster
    }
    else if (x == HIGH){     //If the reverse limit switch is activated
      Serial.print("Triggered Setpoint\n");
      break;                 //Break while loop
    }
  }
  StopA();             //Stop movement of Actuator A
  delay(500);          //Delay between print
  EncAencoderPos = 0;  //Set Encoder position to zero
  Serial.println("Encoder has been initialized");
  delay(500);          //Delay between prints
  y = 3500;            //Number of encoder counts
  dataNumber = 3500*0.919; //convert to microns for serial print
  Serial.println("Moving Actuator to Zero Mark");
  MoveAOutToTarget(flimsw,y); //Move Actuator A to target while checking forward limit switch
}

void XZeroing(int rlimsw, int flimsw){ 
/* Description: Zeros the X axis actuator (Actuator B) while monitoring both
 *              limit switches. It uses the functions MoveBOutToTarget and 
 *              MoveBIn.
 */
  int x = 0;      //Variable for reading LimitSwitch
  long int y = 0; //Variable for encoder target input
  x = digitalRead(rlimsw);   //Read limit switch output 
  while (x == LOW){          //While the reverse limit is not activated
    x = digitalRead(rlimsw); //Read limit switch output
    if(x == LOW){            //If the reverse limit switch is not activated
      MoveBIn(175);          //Move B at a set speed------------------------ Actuator has had trouble moving at 150 previously
    }
    else if (x == HIGH){     //If the reverse limit switch is activated
      Serial.print("Triggered Setpoint\n");
      break;                 //Break while loop
    }
  }
  StopB();            //Stop movement of Actuator B
  delay(500);         //Delay between prints
  EncBencoderPos = 0; //Set Encoder position to zero
  Serial.print("Encoder has been initialized\n");
  delay(500);         //Delay between prints
  y = 3500;           //Number of encoder counts
  dataNumber = 3500*0.919; //convert to microns for serial print 
  Serial.println("Moving Actuator to Zero Mark");
  MoveBOutToTarget(flimsw,y); //Move Actuator B to target while checking forward limit witch
}

void MoveYEncoder(){
/* Description: Moves the Y axis actuator when the Arduino is in the Programmable Path mode.
 *              It uses recWithEndMarker, showNewNumber, MoveAOutToTarget and MoveAInToTarget
 *              functions.
 */
    long int y = 0;
    while (newData != true){ //If newData has not been detected
      recvWithEndMarker();   //Cycle through to read next character from serial port
    }
    showNewNumber();       //Turn characters entered into an integer
    y = dataNumber/0.919;        //Load y with new integer and change units to encoder counts
    Y_flag = false;        //Actuator selection flag changed to prevent re-entry
    if(Move_flag == true){       // If Move_Flag is true
      if (NEGnum_flag == false){ //And the number is not negative
        MoveAOutToTarget(forlimA,y); //Move Actuator to target and check if the forward limit switch is entered
        Move_flag = false;       //Once completed, disable Move_flag
      }
      else {                     //If the number is negative

        MoveAInToTarget(revlimA,y); //Move Actuator to target and check if the forward limit switch is entered
        Move_flag = false;    //Once completed, disable Move_flag
        //NEGnum_flag = false;  //Do not reset flag to enable corrective re-entry into MoveAOutToTarget
      }
    }
}

void MoveXEncoder(){ 
/* Description: Moves the X axis actuator when the Arduino is in the Programmable Path mode.
 *              It uses recWithEndMarker, showNewNumber, MoveBOutToTarget and MoveBInToTarget
 *              functions.
 */
  long int x = 0;
  while (newData != true){  //If newData has not been detected
    recvWithEndMarker();    //Cycle through to read next character from serial port
  }
  
  showNewNumber();          //Turn characters entered into an integer
  x = dataNumber/0.919;     //Load x with new integer and change units to encoder counts 
  X_flag = false;           //Actuator selection flag changed to prevent re-entry
  
  if(Move_flag == true){    //If actuator is allowed to move
    if(NEGnum_flag == false){  //If the dataNumber is negative
      MoveBOutToTarget(forlimB,x);  //Move Actuator B out to target while checking forward limit switch 
      Move_flag = false;            //Move flag is turned off
    }
    else {
      MoveBInToTarget(revlimB,x);  //Move Actuator B into target while checking reverse limit switch
      Move_flag = false;           //Once completed, disable Move_flag
      //NEGnum_flag = false;       //Do not reset flag to enable corrective re-entry into MoveAOutToTarget
    }
  }
}

//------------------------Encoder Movement for Actuator A------------------------
//------------------------(Y axis)-----------------------------------------------

void MoveAOutToTarget(int flimsw,long int target)  { 
/* Descrription: : Moves Actuator A using the MoveAOut function and detects whether 
 *                 the target encoder count is reached (long int target), or the forward 
 *                 limit switch (int flimsw) has been activated to stop actuator
 *                 motion using the StopA function.
 */ 
long int x = 0;  //New encoder count to reach
int y = 0;       //Limit switch variable
int f = 0;       //Speed Differential variable
  
  if(NEGnum_flag != true){//if  not entering as a negative correction & print as usual
      Serial.print("Moving Y actuator forward  ");
      Serial.print(dataNumber); 
      Serial.print(" Microns \n");
	
      x =  EncAencoderPos + target; //Make new total encoder count to reach
    }
  else{ // if entering as a negative correction do not print and set X to the correction value 
      x = EncBencoderPos + correction;
      Serial.print("Correction of   ");
      Serial.print(correction);
      Serial.print(" Microns required \n");
    }

  while (EncAencoderPos < x){  //While the current encoder count is less than the new count
    y = digitalRead(flimsw);   //Read forward limit switch
    if (y == LOW){             //If forward limit switch is not active
      f = SpeedDifferential(EncAencoderPos);  //Calculated required extrusion speed
      MoveAOut(f);             //Set moving out speed to speed found above
      delay(2);
      //Serial.println(EncAencoderPos);
    }
    else if(y == HIGH){ //If forward limit switch is not active
      Serial.println("Forward Limit Switch Activated, Stopping");
      break;           //Exit out of loop
    }
  } 
  StopA();     //Stop movement of actuator
  delay(250);  //Delay for print
  NEGnum_flag = false; //reset flags 
  //Serial.println(f);
  Serial.print("target found at ");
  Serial.print(EncAencoderPos*0.919);
  Serial.print(" Microns \n");
}

void MoveAInToTarget(int rlimsw,long int target){
/* Description: Moves Actuator A using the MoveAIn function and detects whether 
 *              the target encoder count is reached (long int target), or the reverse 
 *              limit switch (int rlimsw) has been activated to stop actuator 
 *              motion using the StopA function.
 */
  Serial.print("Moving Y actuator backward "); 
  Serial.print(dataNumber);
  Serial.print(" Microns \n");
  long int x = 0;  //New encoder count to reach 
  int z = 0;       //Limit switch variable
  float f = 0;     //Speed Differential variable
  
  x =  EncAencoderPos + target; //Make new total encoder count to reach
  
  while (EncAencoderPos > x){ //While the current encoder count is less than the new count
    z = digitalRead(rlimsw);  //Read reverse limit switch
    if (z == LOW){            //If reverse limit switch is not active
      f = SpeedDifferential(EncAencoderPos);  //Calculated required extrusion speed
      MoveAIn(0.5*f);      //Set moving in speed to 50% the speed found above because the actuators move faster in the reverse direction 
      delay(2);
      //Serial.println(EncAencoderPos);
    }
    else if(z == HIGH){  //If forward limit switch is not active
      Serial.println("Reverse Limit Switch Activated, Stopping");
      break;             //Exit out of loop
    }
  }
  StopA();     //Stop movement of actuator
  delay(250);  //Delay for print
  
  correction = (x - EncBencoderPos) * 0.919; //calculates overshoot value in microns
  
//  Serial.print("target found at ");
//  Serial.print(EncAencoderPos*0.919);
//  Serial.print(" Microns \n"); 
  MoveAOutToTarget(forlimA,correction);  
}

void testYactuator(){
  /*Description: Tests Y actuator repeatability across a chosen
   * encoder count range 
   */
 Serial.println("Enter a target value for Y axis repeatability in encoder counts");
  long int y = 0;
  long int rep=0;
  //int k=0;
    while (newData != true){ //If newData has not been detected
      recvWithEndMarker();   //Cycle through to read next character from serial port
    }
    showNewNumber();        //Turn characters entered into an integer
    y = dataNumber;         //Load y with new integer
    Y_flag = false;         //Actuator selection flag changed to prevent re-entry

    if (Move_flag == true);{      //If Move_flag is true
      if (NEGnum_flag == false){  //And the number is not negative
        int k = 0;
        rep = (88100 - EncAencoderPos) / y;
        Serial.print("test will repeat ");
        Serial.print(rep-20);
        Serial.print( "times \n");
        for (k = 0; k <= (rep-20); k++){
          MoveAOutToTarget(forlimA,y);
          delay(50);
        }
      }
      else {                      //If the number is negative
        int k = 0;
        rep=(EncAencoderPos)/-y;
        Serial.print("test will repeat ");
        Serial.print(rep-20);
        Serial.print( "times \n");
        for (k = 0; k <= (rep-20); k++){
          MoveAInToTarget(revlimA,y);
          delay(50);
        }
      }
    NEGnum_flag = false;    //Disable number flag  
    }
  Move_flag = false;        //Once completed, disable Move_flag
  test_flag = false;        //Once completed, disable test_flag
}

//------------------------Encoder Movement for Actuator B------------------------
//-----------------------------------(X axis)------------------------------------

void MoveBOutToTarget(int flimsw,long int target)  { // Moving shaft out 
/* Description: : Moves Actuator B using the MoveBOut function and detects whether 
 *                 the target encoder count is reached (long int target), or the forward 
 *                 limit switch (int flimsw) has been activated to stop actuator
 *                 motion using the StopB function.
 */
  long int x = 0;  //New encoder count to reach
  int y = 0;       //Limit switch variable
  int f = 0;       //Speed Differential variable
  
  if (NEGnum_flag != true){      //if  not entering as a negative correction print as usual 
      Serial.print("Moving X actuator forward  ");
      Serial.print(dataNumber);
      Serial.print(" Microns \n");
  
      x =  EncBencoderPos + target; //Make new total encoder count to reach
  }
    else{     // if entering as a negative correction do not print and set X to the correction value 
      x = EncBencoderPos + correction;
      Serial.print("Correction of   ");
      Serial.print(correction);
      Serial.print(" Microns required \n");
    }
    
  while (EncBencoderPos < x){  //While the current encoder count is less than the new count
    y = digitalRead(flimsw);   //Read forward limit switch
    if (y == LOW){             //If forward limit switch is not active
      f = SpeedDifferential(EncBencoderPos);  //Calculated required extrusion speed
      MoveBOut(f);             //Set moving out speed to speed found above
      delay(2);
      //Serial.println(EncBencoderPos);
    }
    else if(y == HIGH){ //If forward limit switch is not active
      Serial.println("Forward Limit Switch Activated, Stopping");
      break;            //Exit out of loop
    }
  } 
  StopB();     //Stop movement of actuator
  delay(250);  //Delay for print
  NEGnum_flag = false; //reset flags 
  
  Serial.print("target found at ");
  Serial.print(EncBencoderPos*0.919);
  Serial.print(" Microns \n");
}

void MoveBInToTarget(int rlimsw,long int target){
/* Description: Moves Actuator B using the MoveBIn function and detects whether 
 *              the target encoder count is reached (int target), or the reverse 
 *              limit switch (int rlimsw) has been activated to stop actuator 
 *              motion using the StopB function.
 */
  Serial.print("Moving X actuator backward "); 
  Serial.print(dataNumber);
  Serial.print(" Microns \n");
  long int x = 0;  //New encoder count to reach
  int z = 0;       //Limit switch variable
  float f = 0;     //Speed Differential variable
  
  x =  EncBencoderPos + target; //Make new total encoder count to reach
  
  while (EncBencoderPos > x){ //While the current encoder count is less than the new count
    z = digitalRead(rlimsw);  //Read reverse limit switch
    if (z == LOW){            //If reverse limit switch is not active
      f = SpeedDifferential(EncBencoderPos);  //Calculated required extrusion speed
      MoveBIn(0.5*f);        //Set moving in speed to 20% the speed found above because the actuators move faster in the reverse direction 
      delay(2);
      }
    else if(z == HIGH){ //If forward limit switch is not active
      Serial.println("Reverse Limit Switch Activated, Stopping");
      break;            //Exit out of loop
    }
  }
  StopB();      //Stop movement of actuator
  delay(250);   //Delay for print  
  
  correction = (x - EncBencoderPos) * 0.919; //calculates overshoot value in microns
  
//  Serial.print("target found at ");
//  Serial.print(EncBencoderPos*0.919);
//  Serial.print(" Microns \n"); 
  MoveBOutToTarget(forlimB,correction);
}

void testXactuator(){
/* Description: Tests the repeatability of the Y actuator by moving it a set number of encoder 
counts and then repeating that action a desired number of times  
 */
  Serial.println("Enter a target value for X axis repeatability in encoder counts");
  long int y = 0;
  long int rep = 0;
  //long int k = 0;
    while (newData != true){ //If newData has not been detected
      recvWithEndMarker();   //Cycle through to read next character from serial port
    }
    showNewNumber();        //Turn characters entered into an integer
    y = dataNumber;       //Load y with new integer
    X_flag = false;         //Actuator selection flag changed to prevent re-entry 
    
    if (Move_flag == true);{       // If Move_Flag is true
      if (NEGnum_flag == false){ //And the number is not negative
        int k = 0;
        rep= (88100 - EncBencoderPos)/y;
        Serial.print("test will repeat ");
        Serial.print(rep-20);
        Serial.print(" times \n");
        for (k = 0; k <= (rep - 20); k++) {          
          MoveBOutToTarget(forlimB,y);
          delay(50);
          k++;
        }
      }
      else {                     //If the number is negative
        int k = 0;
        rep= (EncBencoderPos)/-y;
        Serial.print("test will repeat ");
        Serial.print(rep-20);
        Serial.print(" times \n");
        for (k = 0; k <= (rep - 20); k++) {          
          MoveBInToTarget(revlimB,y);
          delay(50);
          k++;
        }      
      }
      
    NEGnum_flag = false;  //Disable number flag 
    }
   Move_flag = false;     //Once completed, disable Move_flag
   test_flag = false;     //Once completed, disable test_flag
}

//---------------------------Joystick Motion-----------------------------

void LimitSwitchA(int rlimsw, int flimsw,int speed){
/* Description: Moves the Y axis actuator in the Joystick state while 
 *              checking whether the limit switches (int rlimsw & int
 *              flimsw) have been activated. The speed (int speed) is 
 *              a flag for which direction the joystick is pointing.
 */
  int x = 0;  //Forward Limit Switch reader variable
  int y = 0;  //Reverse Limit Switch reader variable
  int f = 0;  //Speed Differential calculated value variable
  x = digitalRead(flimsw);  //Read forward limit switch
  y = digitalRead(rlimsw);  //Read reverse limit switch
  if (speed > 0){
    if (x == LOW){      //Move out Actuator A
      f = SpeedDifferential(EncAencoderPos);  //Calculate speed from Encoder A Position
      MoveAOut(2*f);
    }
    else if(x == HIGH){ //Front Limit Switch Activated for Actuator A
      Serial.print("Y Axis forward limit reached, Reverse Now! \n");
      StopA();          //Stop Actuator B motion
    }
  }
  else if (speed < 0){
    if (y == LOW){      //Move in Actuator A
      f = SpeedDifferential(EncAencoderPos);  //Calculate speed from Encoder A Position
      MoveAIn(2*f);       //Move Actuator A out by calculated speed
    }
    else if (y == HIGH){ //Reverse Limit Switch Activated for Actuator A
      Serial.print("Y Axis reverse limit reached, Forward Now! \n");
      StopA();           //Stop Actuator A motion
    }
  }
  else {
    StopA();  //Stop Actuator A motion
  } 
}

void LimitSwitchB(int rlimsw, int flimsw,int speed){
  /* Description: Moves the X axis actuator in the Joystick state while 
 *              checking whether the limit switches (int rlimsw & int
 *              flimsw) have been activated. The speed (int speed) is 
 *              a flag for which direction the joystick is pointing.
 */
  int x = 0;  //Forward Limit Switch reader variable
  int y = 0;  //Reverse Limit Switch reader variable
  int f = 0;  //Speed Differential calculated value variable
  x = digitalRead(flimsw);  //Read forward limit switch
  y = digitalRead(rlimsw);  //Read reverse limit switch
  if (speed > 0){        
    if (x == LOW){  //Move out Actuator B
      f = SpeedDifferential(EncBencoderPos); //Calculate speed from Encoder B Position
      MoveBOut(1.5*f);  //Move Actuator B out by calculated speed
    }
    else if(x == HIGH){ //Reverse Limit Switch Activated
      Serial.print("X Axis forward limit reached, Reverse Now!\n");
      StopB();  //Stop Actuator B motion
    }
  }
  else if (speed < 0){
    if (y == LOW){        //Move in Actuator B
      f = SpeedDifferential(EncBencoderPos); //Calculate speed from Encoder B Position
      MoveBIn(1.5*f);         //Move Actuator B in by calculated speed
    }
    else if (y == HIGH){  //Forward Limit Switch Activated
      Serial.print("X Axis forward limit reached, Forward Now! \n");
      StopB();           //Stop Actuator B motion
    }
  }
  else {
    StopB(); //Stop Actuator B motion
  }
}

//---------------------------Saturation Block-----------------------------

int SatBlock(float x) { 
/* Description: Receives input from the Joystick and acts as flag to determine 
 *              which axis moves. float x is a floating number with a range between 
 *              0-1023
 */
  float y;  //Initializes a floating number for return
  float z;  //Initializes a floating number for return
    if (x >= 383 && x <= 640){  //x equals zero if input is between 383 and 640
      x = 0;                    
      return x;
    }
    else if (x < 383){          //If input is less than 383
      y =  1;//*((x - 383)/383); //Run math to change input into percentage then multiply by -255
      return y;
    }
    else if (x > 640){          //If input is greater than 640
      z = -1;//*((x - 640)/383); //Run math to change input into percentage then multiply by -255
      return z;
    }

}

//---------------------Serial Monitor Read/Decode------------------------

void recvWithEndMarker() {
/* Description: Reads from the Serial Monitor then input entered and 
 *              organizes the input into an array
 */
    static byte ndx = 0;   //Storage Index
    char endMarker = '\n'; //End marker from Serial Inputs
    char rc;               // Temporary char storage
    
    if (Serial.available() > 0) { //Wait for Serial port to read entered input
        rc = Serial.read();       //Reads 1 byte from Serial port
        if (rc != endMarker) {    //If end marker not detected, index data
            receivedChars[ndx] = rc;
            ndx++;                //Increase storage index
            if (ndx >= numChars) {//If Storage Index larger than array size
                ndx = numChars - 1; //Set index to last array space for overwrite
            }    
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            index = ndx;     //Make char sorter index equal to storage index
            ndx = 0;         //Set storage index to zero for next data 
            newData = true;  //Set newData flag to true
        }
    }
}

void showNewNumber() {
/* Description: Receives the array constructed by the recWithEndMarker function 
 *              and attempts to construct an integer from the ASCII values 
 *              present in the array  
 */
    if (newData == true) { //If newData is ready
        dataNumber = 0;     
        int i = 0;  //Index for while loop to be compared to array index
        int x = 0;  //Variable to hold next array input
        while (i < index+1){     //While i is less than the character storage index + 1
          if (receivedChars[i] >= 0x30 && receivedChars[i] <= 0x39){
            //If the character in the array location is between 0 and 9 in Hex
            val = val*10;                 //Multiply value by 10
            x =  receivedChars[i] - 0x30; //Subtract character by 30 hex
            //Serial.println(x);          //Print digit 
            val = val + x;                //Add digit to value
            i++;                          //increase index by 1
            x = 0;                        //Set x equal to zero for next array location
          }
          else if(receivedChars[i] == 0x2D){
            Serial.println("Negative Number");
            neg_flag = true;     //If a negative sign is detected, set neg_flag to true
            NEGnum_flag = true;  //and set NEGnum_flag to true
            i++;                 //Increase index by 1
          }
          else if(receivedChars [i] == '\0'){ //If the array location contains a NULL
            i++;   //Increase index by 1
            x = 0; //Set x equal to zero for next iteration
          }
          else{
            Serial.print("Yo, your number is inappropriate, write a new one\n"); //Reasonable Response
            val = 0;  //Bad number detected, set value to 0 and exit loop
            break;    //Exit out of the loop
          }
        }
        if (neg_flag == true){ //If negative number detected, multiply value by -1 
          val = val*-1; 
          if(val < -90000){   //If less than -90000, 
            Serial.print("Number too small, Enter a larger one\n");
            dataNumber = 0;  //Sets number to zero for next iteration
            val = 0;         //Sets value to zero for next iteration
            Move_flag = false;
            neg_flag = false;
          } 
          else{ //If with threshold
            dataNumber = val; //Set value equal to dataNumber
            val = 0;          //Set value to zero for next iteration
            Move_flag = true; //Allows selected actuator to move
          }
        }
        else{ //If negative number not detected
          if(val > 90000){ //If number is larger than 90000
            Serial.print("Number too big, Enter a smaller one\n");
            dataNumber = 0;    //Sets number to zero for next iteration
            val = 0;           //Sets number to zero for next iteration
            Move_flag = false; //Prevents selected actuator from moving
          } 
          else{
            dataNumber = val; //Set value equal to dataNumber
            val = 0;          //Set value to zero for next iteration
            Move_flag = true; //Allows selected actuator to move
          }
        }
        
        newData = false;  //Set newData to false to allow for next data input
        neg_flag = false; //Set neg_flag to allow for next calculation of a negative value
    }
}

void selectActuator() { 
/* Description: Acts similar to the showNewNumber but instead checks whether 
 *              the array holds an X or a Y. 
 */
 
    if (newData == true) { //If newData is ready
        character = 0;     //Variable set to zero for next iteration
        if (receivedChars[0] == 0x58 || receivedChars[0] == 0x78){  //Compares receivedChar to X in hex
          Serial.println("X Axis Selected");
          character = receivedChars[0]; // X placed into character 
        }
        else if(receivedChars[0] == 0x59 || receivedChars[0] == 0x79){ //Compares receivedChar to Y in hex
          Serial.println("Y Axis Selected");
          character = receivedChars[0]; // Y placed into character
        }
        else if(receivedChars[0] == 0x54 || receivedChars[0] == 0x74){ //Compares receivedChar to T in hex
          Serial.println("Test mode Selected");
          character = receivedChars[0]; // T placed into character
        }
        else{
          Serial.print("That letter is not one of the availible options, try again.\n"); //Reasonable print for failed character
        }
      } 
      newData = false;  //Set newData to false to allow for next data input
}

int SpeedDifferential(float x){
/*Description: Receives the current encoder count and calculates the speed for encoders.
 *             NOTE: The numbers below can be manipulated as long as the final result 
 *             (float y) results in 255 as the maximum possible output.  The 88100 in 
 *             the denominator of float m represents the maximum amount encoder counts
 *             the actuator can travel. This has been verified through multiple tests
 *             at varying speeds.
 *             
 *             As a general rule, if 255 is wanted to be the maximum speed at full extension,
 *             the slope must coincide with the selected y intercept to maintain a linear
 *             relationship.
 */
  float b = 61;           //Y-Intercept, Set speed is 150 for 10.5V or 175 for 5V
  float m = 169/88100.0; //Slope of speed differential, 80.0
  float y;                //Final speed output variable
  y = m * x + b;    
  return y;

 //At 5V, b = 175, m =  80.0/88100
 //At 10.5V, try to use b = 150, m =  105.0/88100
 //---- 10.5V can be manipilated to be slower at start (best lower limit estimate is b = 72 and m = 183/88100
 //At 12V to keep actuator voltage away from it's 12V max b=130 and m=100 keeping m+b below 255
}
