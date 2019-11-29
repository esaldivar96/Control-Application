/*
 * Enrique Daniel Saldívar Carranza
 * Program to make the indentification of a DC engine with encoder
 * Intrusrions on how to use MonsterShield: https://www.instructables.com/id/Monster-Motor-Shield-VNH2SP30/
 * Engine being used: https://www.pololu.com/product/4753
*/

const float  _CPR = 3200.0;                       // Counts per revolution
const unsigned int _sample_t = 10;                // Sample time ms
const unsigned int _total_samples = 1000;         // Total samples to take
const unsigned int _time_diff_to_change = 1000;   // Time diff in ms for rand to change 

unsigned int pwm = 0;
unsigned long time;                       // Time in ms since the arduino has been running (overflow in around 50 days)
unsigned long prev_time_to_change = 0;    // For comparison purposes
unsigned long prev_time_to_sample = 0;
unsigned int pulse_counter = 0;
unsigned int sample_counter = 0;
unsigned int ms_per_rev = 0; 
float RPM = 0;

// Seting up Monster Shield
unsigned int PWM_pin = 3;      // Pin that will control de speed of the motor with PWM - D5 in MS
unsigned int dir_pin_7  = 7;   // Pin to specify for direction MS - D7 in MS
unsigned int dir_pin_8  = 8;   // Pin to specify for direction MS - D8 in MS
unsigned int enable_pin = 4;   // Pin for the motor enable - D0 in MS

void setup() {

  Serial.begin(9600);

  pinMode(PWM_pin, OUTPUT);       // Sets the PWM pin as output
  pinMode(dir_pin_7, OUTPUT);     // Sets the direction pin as output
  pinMode(dir_pin_8, OUTPUT);     // Sets the direction pin as output
  pinMode(enable_pin, OUTPUT);    // Sets the enable pin as output
  
  digitalWrite(dir_pin_7, HIGH);  // CW
  digitalWrite(dir_pin_8, LOW);   // CW
  digitalWrite(enable_pin, HIGH); // Motor is enabled

  attachInterrupt(digitalPinToInterrupt(2),interrupt_routine,RISING);

  //Serial.print("Sample time in ms: ");
  //Serial.println(_sample_t);
}

void loop() {

  time = millis();

  // To change pwm value
  if(time - prev_time_to_change >= _time_diff_to_change) {
    prev_time_to_change = time;
    //pwm = random(25,200);
    pwm = 150;
    analogWrite(PWM_pin, pwm);  // analogWrite values from 0 to 255 for the PWM
  }

  // To print the output as a sample
  if(time - prev_time_to_sample >= _sample_t && sample_counter <= _total_samples) {
    prev_time_to_sample = time;
    ms_per_rev = (_sample_t * _CPR) / pulse_counter;
    RPM = 60000.0 / (float)(ms_per_rev);
    Serial.print(pwm); 
    Serial.print(",");
    Serial.println(int(RPM)); 
    pulse_counter = 0;
    sample_counter++;
  }

}

void interrupt_routine() {

  pulse_counter++;
  
}


//
