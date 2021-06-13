// Pini de control PWM ai motorului M1
#define M1_h_A_pin 11
#define M1_h_B_pin 10

// Pini de control PWM ai motorului M2
#define M2_h_A_pin 3
#define M2_h_B_pin 9

// Valoare maxima PWM
#define PWM_MAX 255

// Valoare maxima factor de umplere
#define DUTY_CYCLE_MAX 100

// Definitia ID-urilor celor doua motoare
typedef enum Motor{
  MOTOR1 = 0,
  MOTOR2 = 1,
}Motor;

// Definitia directiei de miscare
typedef enum Direction{
  FORWARD = 0,
  BACKWARD = 1,
}Direction;


// TODO 4: Implementati functia pentru miscarea unui motor
//         intr-o anumita directie cu duty_cycle-ul primit
//         ca parametru
void setPWM(int m,  int d, int duty_cycle){
  int pwn_width = map(duty_cycle,0,DUTY_CYCLE_MAX,0,PWM_MAX);
  switch(m){
  	case MOTOR1:
    if(d == FORWARD){
   		analogWrite(M1_h_A_pin,pwn_width);
    	analogWrite(M1_h_B_pin,0);
    } else {
     	analogWrite(M1_h_B_pin,pwn_width);
        analogWrite(M1_h_A_pin,0);
    }
    break;
    case MOTOR2:
    if(d == FORWARD){
   		analogWrite(M2_h_A_pin,pwn_width);
    	analogWrite(M2_h_B_pin,0);
    } else {
     	analogWrite(M2_h_B_pin,pwn_width);
        analogWrite(M2_h_A_pin,0);
    }
    break;
  }
}


void setup() {
  // TODO 3: Setati pini pentru utilizarea pinilor de PWM
  pinMode(M1_h_A_pin,OUTPUT);
  pinMode(M1_h_B_pin,OUTPUT);
  pinMode(M2_h_A_pin,OUTPUT);
  pinMode(M2_h_B_pin,OUTPUT);
}



void loop() { 
  
  
  // TODO 5: Implementati o simulare de accelerare a motorarelor
  //         pe directia inainte in 10 pasi pe o durata de 
  //         o secunda de la 0% la 100% duty-cycle
  // Nota: consultati functia delay din documentatie
  
  int duty_cycle_m1 = 0;
  int duty_cycle_m2 = 0;
  int wait_time_low = 100;
  int wait_time_high = 2000;

  int i = 0;
  for(i = 0; i < 10; i++){
    duty_cycle_m1 = duty_cycle_m1 + 10;
    setPWM(MOTOR1, FORWARD, duty_cycle_m1);
    
    duty_cycle_m2 = duty_cycle_m2 + 10;
    setPWM(MOTOR2, FORWARD, duty_cycle_m2);
    
    delay(wait_time_low);
  }
  delay(wait_time_high);
  
  // TODO 6: Implementati o simulare de decelerare a motorarelor
  //         pe directia inainte in 10 pasi pe o durata de 
  //         o secunda de la 0% la 100% duty-cycle 

  for(i = 0; i < 10; i++){
    duty_cycle_m1 = duty_cycle_m1 - 10;
    setPWM(MOTOR1, FORWARD, duty_cycle_m1);
    
    duty_cycle_m2 = duty_cycle_m2 - 10;
    setPWM(MOTOR2, FORWARD, duty_cycle_m2);
    
    delay(wait_time_low);
  }
  delay(wait_time_high);
  
  // TODO 7: Realizati TODO-urile 5 si 6 pentru directia inapoi

  for(i = 0; i < 10; i++){
    duty_cycle_m1 = duty_cycle_m1 + 10;
    setPWM(MOTOR1, BACKWARD, duty_cycle_m1);
    
    duty_cycle_m2 = duty_cycle_m2 + 10;
    setPWM(MOTOR2, BACKWARD, duty_cycle_m2);
    
    delay(wait_time_low);
  }
  delay(wait_time_high);
  
  for(i = 0; i < 10; i++){
    duty_cycle_m1 = duty_cycle_m1 - 10;
    setPWM(MOTOR1, BACKWARD, duty_cycle_m1);
    
    duty_cycle_m2 = duty_cycle_m2 - 10;
    setPWM(MOTOR2, BACKWARD, duty_cycle_m2);
    
    delay(wait_time_low);
  }
  delay(wait_time_high);
}