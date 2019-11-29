/*
 * Enrique Daniel Saldívar Carranza
 * Program to control a DC engine with encoder
 * PID of the form : M(n) = M(n-1) + AE(n) + BE(n-1) + CE(n-2)
 * Intrusrions on how to use MonsterShield: https://www.instructables.com/id/Monster-Motor-Shield-VNH2SP30/
 * Engine being used: https://www.pololu.com/product/4753
*/

const float  _CPR = 3200.0;                       // Counts per revolution
const unsigned int _sample_t = 10;                // Sample time ms
const int _A = 950;
const int _B = -60;
const int _C = 5;
 
int pwm = 0;
int counter = 0;
int pwm_avg = 0;
int rpm_avg = 0;
float RPM = 0;

unsigned long time;                       // Time in ms since the arduino has been running (overflow in around 50 days)
unsigned long prev_time_to_sample = 0;
unsigned int pulse_counter = 0;
unsigned int ms_per_rev = 0; 

unsigned const int REF = 30;   // Reference that the motor is going to follow
int m_1 = 0;
int e = 0;
int e_1 = 0;
int e_2 = 0;


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

  // Analize data at sample time
  if(time - prev_time_to_sample >= _sample_t) {
    prev_time_to_sample = time;
    
    calculate_RPM();
    calculate_PID();
    if(counter == 100) {
      print_average();
    }

    pulse_counter = 0;
  }

}

void print_average() {

  pwm_avg = pwm_avg/counter;
  rpm_avg = rpm_avg/counter;

  Serial.print(RPM);
  Serial.print(" --- ");
  Serial.println(pwm);

  pwm_avg = 0;
  rpm_avg = 0;
  counter = 0;
  
}

void calculate_RPM() {

  if(pulse_counter != 0) {
    ms_per_rev = (_sample_t * _CPR) / pulse_counter;
    RPM = 60000.0 / (float)(ms_per_rev);  // Calculate RPM
  }
  else {
    RPM = 0;
  }
  
}

void calculate_PID() {
  
  m_1 = pwm;
  e_2 = e_1;
  e_1 = e;
  e = REF - RPM;

  pwm = m_1 + _A * e + _B * e_1 + _C * e_2; // Apply control laws
  
  if(pwm > 255) {
    pwm = 255;
  }
  else if(pwm < 0) {
    pwm = 0;
  }
  analogWrite(PWM_pin, pwm);
  
  pwm_avg += pwm;
  rpm_avg += int(RPM);
  counter++;
  
}

void interrupt_routine() {

  pulse_counter++;
  
}





//
