//Declare pin functions on Redboard
#define stp 12
#define dir 13
#define led1 52
#define led2 53

//Declare variables for functions
char PIDinput;
int x;

void setup() {
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  resetEDPins(); //Set step and direction pins to default states
  Serial.begin(9600); //Open Serial 
  while(Serial.read()>=0){}; //drop received incoming data
}

void loop() {
  while(Serial.available()){
      PIDinput = Serial.read();
      if (PIDinput == '1')
      {
        digitalWrite(led2, LOW);
        digitalWrite(led1,HIGH);
         StepForwardDefault();
      }
      else if(PIDinput =='2')
      {
        digitalWrite(led1,LOW);
        digitalWrite(led2, HIGH);
        ReverseStepDefault();
      }
      else if (PIDinput == '0')
      {
        digitalWrite(led1, LOW);
        digitalWrite(led2, LOW);
      }
      resetEDPins();
  }
}

//Reset Easy Driver's pins to default states
void resetEDPins()
{
  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
}

//Default microstep mode function
void StepForwardDefault()
{
  digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  digitalWrite(stp,HIGH); //Trigger one step forward
  delay(1);
  digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
  delay(1);
}

//Reverse default microstep mode function
void ReverseStepDefault()
{
  digitalWrite(dir, HIGH); //Pull direction pin high to move in "reverse"
  digitalWrite(stp,HIGH); //Trigger one step
  delay(1);
  digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
  delay(1);
}
