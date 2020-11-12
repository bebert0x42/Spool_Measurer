/* Beta test code pour projet PIC IMT11 SpoolMeasurer */
/* Programme de test du materiel*/

#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>


// directions
#define FORWARD   HIGH
#define BACKWARD  LOW

// debounce time (milliseconds)
#define DEBOUNCE_TIME  200

// PINs for A4988 driver
#define PIN_STEP  9
#define PIN_DIR   8
#define PIN_ENABLE_DRIVER   10


#define PIN_CLICK  A2 

//Pin du Joystick
const int PIN_X = A0; // analog pin connected to X output
const int PIN_Y = A1; // analog pin connected to Y output
const int PIN_FOURCHE = 2; // analog pin connected to Y output

//Variable des bouton joystick
bool X_PLUS = 0;
bool X_MOIN = 0;
bool Y_PLUS = 0;
bool Y_MOIN = 0;
bool CLICK = 0;

// Variable de statu
bool MOTOR_RUN = 0;
boolean PREVIOUS_MEASURE = LOW;
bool MOTOR_ENABLE = 0;

// lookup table speed - ticks (interrupts)
const int speed_ticks[] = {-1, 400,375,350,325,300,275,250,225,200,175,150,125,110,100,90,85,80,75,70,65,60,55,50};
size_t max_speed = (sizeof(speed_ticks) / sizeof(speed_ticks[0])) -1 ; // nombre de variable dans le tableau


// global variables
LiquidCrystal_I2C lcd(0x27, 16, 2);// définit le type d'écran lcd pour ma part j'utilise un 16 x 2

int measurement = 0;
volatile unsigned int counter;


int actual_speed;
int actual_direction;
int target_speed;

int ticks;
int tick_count;

int button;
boolean debounce;
int previous_time;

// custom LCD square symbol for progress bar
byte square_symbol[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};

// string constants
char forward_arrow[] = "-->";
char backward_arrow[] = "<--";

void pulsecount(){
    counter++;
}

void setup() {
  
  digitalWrite(PIN_ENABLE_DRIVER, HIGH);// Désactivation du moteur
  // init the timer1, interrupt every 0.1ms
  Timer1.initialize(20);
  Timer1.attachInterrupt(timerIsr);
  

 
  lcd.init();// initialize the lcd 
  //lcd.init();
  lcd.backlight();

  // Pin Fourche
  pinMode(PIN_FOURCHE, INPUT);
  
  // pins DRIVER
  pinMode(PIN_ENABLE_DRIVER, OUTPUT);
  attachInterrupt(0, pulsecount, RISING);
  
  digitalWrite(PIN_ENABLE_DRIVER, HIGH);// Désactivation du moteur
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  
  // initial values
  actual_speed = 0;
  target_speed = max_speed/2;
  actual_direction = BACKWARD;
  tick_count = 0;
  ticks = -1;
  debounce = false;

  digitalWrite(PIN_DIR, actual_direction);
    
  updateLCD();
}
  
void loop() {
  
  // check if debounce active
  /*if(debounce) {
    button = btnNONE;
    if(millis() > previous_time + DEBOUNCE_TIME) debounce = false;
  } else button = read_buttons();
  */
  //measure_filament();

  read_buttons() ;//Read X and Y on joystick
  if (MOTOR_RUN){
    increase_speed();
  }
  else {
    decrease_speed();
  }
  /*// check which button was pressed
  switch(button) {
    
    case btnUP:
      increase_speed();
      break;
    case btnDOWN:
      decrease_speed();
      break;
    case btnLEFT:
      change_direction(BACKWARD);
      break;
    case btnRIGHT:
      change_direction(FORWARD);
      break;
    case btnSELECT:
      emergency_stop();
      break;
  }*/
  
  // finally update the LCD
  updateLCD();
}

// increase speed if it's below the max (70)
void increase_speed() {
  
  if(actual_speed < target_speed) {
    actual_speed += 1;
    tick_count = 0;
    ticks = speed_ticks[actual_speed];
  }
  else if (actual_speed > target_speed){
    actual_speed -= 1;
    tick_count = 0;
    ticks = speed_ticks[actual_speed];
  }
  measure_filament();

}

// decrease speed if it's above the min (0)
void decrease_speed() {
  
  if(actual_speed > 0) {
    actual_speed -= 1;
    tick_count = 0;
    ticks = speed_ticks[actual_speed];
  }

}


// emergency stop: speed 0
void emergency_stop() {
  actual_speed = 0;
  tick_count = 0;
  ticks = speed_ticks[actual_speed / 5];
}

// update LCD
void updateLCD() {
  
  // print first line:
  // Speed: xxxRPM --> (or <--)
  lcd.setCursor(0,0);
  lcd.print("Speed: ");
  lcd.print(actual_speed);
  lcd.print("RPM ");

  lcd.setCursor(13,0);
  if(actual_direction == FORWARD) lcd.print(forward_arrow);
  else lcd.print(backward_arrow);
  
      /*// print second line:
      // progress bar [#####         ]
      // 15 speed steps: 0 - 5 - 10 - ... - 70
      lcd.setCursor(0,1);
      lcd.print("[");
      
      for(int i = 1; i <= 14; i++) {
        
        if(actual_speed > (5 * i) - 1) lcd.write(byte(0));
        else lcd.print(" ");
      }
      
      lcd.print("]");*/
 
  lcd.setCursor(0,1);
  lcd.print("        ");
  lcd.setCursor(0,1);
  lcd.print(counter);
  lcd.setCursor(8,1);
  lcd.print(target_speed);
} 

// timer1 interrupt function
void timerIsr() {
  
  if(actual_speed == 0) return;

  digitalWrite(PIN_ENABLE_DRIVER, LOW);// Activation du moteur
  MOTOR_ENABLE = 1;
  
  tick_count++;
  
  if(tick_count == ticks) {  
    
    // make a step
    digitalWrite(PIN_STEP, HIGH);
    digitalWrite(PIN_STEP, LOW);
    
    tick_count = 0;
  }
}


// read Joystick connected to a single analog pin
int measure_filament() {
  int CURRENT_MEASURE = digitalRead(PIN_FOURCHE);
  if (PREVIOUS_MEASURE != CURRENT_MEASURE) {
         if( CURRENT_MEASURE == HIGH ) // If input only changes from LOW to HIGH
       {
             measurement += 1;
       }
  }
  PREVIOUS_MEASURE = CURRENT_MEASURE;
}



// read Joystick connected to a single analog pin
int read_buttons() {

  int XValue = analogRead(PIN_X);     // Read the analog value from The X-axis from the joystick
  int YValue = analogRead(PIN_Y);     // Read the analog value from The Y-axis from the joystick
  

  if (!Y_PLUS & !Y_MOIN){
      if (YValue < 10){ // joystick Y - -> reduce speed
            /*if (motorSpeed >= motorMaxSpeed){
              motorSpeed= motorMaxSpeed;
            }
            else{
               motorSpeed = motorSpeed +100 ;
            }*/
            Y_MOIN = 1;
            Y_PLUS = 0;
      }
      else if (YValue > 800 ){ // joystick Y +  -> rise speed
             /*if (motorSpeed > 0){
              motorSpeed = motorSpeed -100;
             }*/
             Y_MOIN = 0;
             Y_PLUS = 1;
      }
      }
      else if (YValue < 800 && YValue > 50 && actual_speed > 0 ){        // Y en home position      
        Y_MOIN = 0;
        Y_PLUS = 0;
     } 


    if (XValue < 400 && XValue > 10){ // joystick X - -> Stop motor
        if (MOTOR_RUN && target_speed > 5 ){
          target_speed = target_speed - 1;
      }
    }


    else if (XValue < 10){ // joystick X - -> Stop motor
        if (MOTOR_RUN){
          MOTOR_RUN = 0;
        }
        else if ( actual_speed <= 0){
          Y_MOIN = 0;
          Y_PLUS = 0;
          MOTOR_ENABLE = 0;
          digitalWrite(PIN_ENABLE_DRIVER, HIGH);
        }
        
      }


      
   if (XValue > 800){ // joystick X + -> Start motor or Speed
      if (!MOTOR_RUN){
        target_speed = max_speed/2;
        MOTOR_RUN = 1;
      }
      if (MOTOR_RUN){
        if (target_speed < max_speed){
          target_speed += 1;
        }
      }
        
      }
}
