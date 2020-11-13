/* Beta test code pour projet PIC IMT11 SpoolMeasurer */
/* Programme de test du materiel*/

#include <TimerOne.h>
#include <LiquidCrystal_I2C.h>

//******* Déclaration des Pin de l'arduino **********

	// PIN pour le Driver/Moteur A4988
	#define PIN_STEP  9
	#define PIN_DIR   8
	#define PIN_ENABLE_DRIVER 10

	LiquidCrystal_I2C lcd(0x27, 16, 2);// définit le type d'écran lcd avec connexion I2C sur les PIN SLA SCL

	//Pin du Joystick
	const int PIN_X 		= A0; // analog pin connected to X output
	const int PIN_Y 		= A1; // analog pin connected to Y output
	const int PIN_CLICK 	= 3;// Digital pin connected to Y output
	//Pin Fourche Optique
	const byte PIN_FOURCHE 	= 2; //Digital Pin connected to optical switch

//******* Déclaration des Variables de statu *************
	//Variable des bouton joystick
	bool X_PLUS 			= 0;
	bool X_MOIN 			= 0;
	bool Y_PLUS 			= 0;
	bool Y_MOIN 			= 0;
	bool CLICK 				= 0;
	bool commande_waiting	= 1;// Variable pour attendre que le joystick se remete en 0
	bool commande_push		= 0;// Variable pour push une commande joystick

	// Variable de statu
	bool MOTOR_RUN 			= 0;
	bool MOTOR_ENABLE 		= 0;

	// Variable de de fenetre IHM
	bool window_Menu 		= 1;
	bool window_Manual_run 	= 0;
	bool window_Manual_init	= 0;
	bool window_Manu_paused	= 0;
	bool window_Auto_init 	= 0;
	bool window_Auto_run 	= 0;
	bool window_Auto_paused	= 0;
	bool window_Finish 		= 0;
	bool window_Fail 		= 0;


//******* Variable pour le fonctinnement moteur **********
	int actual_speed;
	int actual_direction;
	int target_speed;
	int ticks;
	int tick_count;
	// Constante de direction du moteur
	#define FORWARD   HIGH
	#define BACKWARD  LOW

//*********** Parametre de vitesse du moteur ***************

	const int speed_ticks[] = {-1, 400,375,350,325,300,275,250,225,200,175,150,125,110,100,90,85,80,75,70,65,60,55,50};
	size_t max_speed = (sizeof(speed_ticks) / sizeof(speed_ticks[0])) -1 ; // nombre de variable dans le tableau



//******* Variable pour la fonction de mesure **********

	volatile unsigned int  counter_steps=0;// Compteur du nombre de steps de la roue codeuse
	volatile float measurement = 0; //mesure a afficher sur l'IHM
	volatile float measurement_target = 0; //La mesure ciblé
	//                           IIIII
	//                           IIIII
	//     Variable pour         IIIII
	//      la calibration     IIIIIIIII
	//                          IIIIIII
	//                           IIII
	//                            II
	const float perimeter_gear = 0.0339; //périmetre de la roue denté de mesure en mm
	const int encoder_hole = 2;//nombre de fenetre sur la roue codeuse




/*
 * =============================================================================================
 * ========================================= Setup =============================================
 * =============================================================================================
*/
void setup() {

	digitalWrite(PIN_ENABLE_DRIVER, HIGH);// Désactivation du moteur avant le setup

	// initial values
  	actual_speed = 0;
  	target_speed = max_speed/2;
  	actual_direction = BACKWARD;
  	tick_count = 0;
  	ticks = -1;
	
	// Init du Timer1, interrupt toute les 0.1ms pour lancer un step moteur
	Timer1.initialize(20);
	Timer1.attachInterrupt(timerMotor);
	
	// Init de l'interrupt de compteur de la fourche optique en fonction du soulevement du signal
	attachInterrupt(digitalPinToInterrupt(PIN_FOURCHE), measure_filament, RISING);

	//Initialisation du LCD I2C
	lcd.init();
	lcd.backlight();

	//Initialisation des PIN
  	
	// pins DRIVER
  	pinMode(PIN_ENABLE_DRIVER, OUTPUT);
	digitalWrite(PIN_ENABLE_DRIVER, HIGH);// Désactivation du moteur
  	pinMode(PIN_STEP, OUTPUT);
  	pinMode(PIN_DIR, OUTPUT);
	digitalWrite(PIN_DIR, actual_direction);

  	// Pin Fourche ??????????????????????????????????????????????????????
  	pinMode(PIN_FOURCHE, INPUT_PULLUP);

}
/*
 * =============================================================================================
 * ======================================== Main Loop ==========================================
 * =============================================================================================
*/
void loop() {

	measurement =	counter_steps * perimeter_gear/encoder_hole;
	ruuning_program();	// Boucle programme principal
	read_joystick();	// Lectue Joystick
	updateLCD();		// Mise a jours du LCD

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
  
}

/********************************* Programme principal ****************************************
 * 
 *  - fonctionement du programme suivant la séquence IHM (voir fichier)
 * 
************************************************************************************************/
void ruuning_program() {
	if (!commande_push || !commande_waiting){
		return;
	}

	if (window_Menu){
		counter_steps = 0;
		MOTOR_RUN = 0;
		if(Y_MOIN){
			resetIHM();
			window_Manual_init = 1;
		}
		else if(Y_PLUS){
			resetIHM();
			window_Auto_init = 1;
		}
	}
	else if(window_Manual_init)
	{
		if(Y_MOIN){
			resetIHM();
			window_Menu = 1;
		}
		else if(Y_PLUS){
			resetIHM();
			window_Manual_run = 1;
			MOTOR_RUN = 1;
			
		}
	}
	else if(window_Manual_run)
	{
		if(Y_MOIN){
			MOTOR_RUN = 0;
			resetIHM();
			window_Manu_paused = 1;
		}
	}
	else if(window_Manu_paused)
	{
		if(Y_MOIN){
			resetIHM();
			window_Finish = 1;
		}
		if(Y_PLUS){
			resetIHM();
			MOTOR_RUN = 1 ;
			window_Manual_run = 1;
		}
	}
	else if(window_Finish)
	{
		if(Y_MOIN || Y_PLUS){
			resetIHM();
			window_Menu = 1;
		}
	}
	else if(window_Auto_init){
		if(Y_MOIN){
			resetIHM();
			window_Menu = 1;
		}
		else if(Y_PLUS){
			resetIHM();
			window_Auto_run = 1;
			MOTOR_RUN = 1;
		}
	//a poursuivre
	}
  	else if(window_Auto_run){

	//a poursuivre
	}
	else if(window_Auto_paused){

	//a poursuivre
	}
	else if(window_Fail){

	//a poursuivre
	}

	
	commande_waiting = 0;
 
}
/********************************* Fonction Diminution de la vitesse Moteur *********************
 * 
 * - Augmentation de la vitesse moteur en fonction des valeurs du tableau speed_ticks
 * 
************************************************************************************************/
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
}

/********************************* Fonction Diminution de la vitesse Moteur *********************
 * 
 * - Diminution de la vitesse moteur en fonction des valeurs du tableau speed_ticks
 * 
************************************************************************************************/
void decrease_speed() {
  
  if(actual_speed > 0) {
    actual_speed -= 1;
    tick_count = 0;
    ticks = speed_ticks[actual_speed];
  } 
}

/********************************* Fonction arret d'urgence Moteur ******************************
 * 
 * - Non utilisé
 * 
************************************************************************************************/
void emergency_stop() {
  actual_speed = 0;
  tick_count = 0;
  ticks = speed_ticks[actual_speed / 5];
}

/********************************* Fonction TimerMotor *****************************************
 *
 * - Fonction lancé toute les 20ms avec au Timer1
 * - Fait tourner le moteur un tour en fonction de la vitesse
 * 
************************************************************************************************/
void timerMotor() {
	if(actual_speed == 0) return;

	digitalWrite(PIN_ENABLE_DRIVER, LOW);// Activation du moteur
	MOTOR_ENABLE = 1;//Flag motor en marche

	tick_count++;

	if(tick_count == ticks) {  
		// 1 step du moteur
		digitalWrite(PIN_STEP, HIGH);
		digitalWrite(PIN_STEP, LOW);

		tick_count = 0;
	}
}



/********************************* Fonction de compteur de tour par interupt *******************
 *
 *  - Ajoute 1 à la variable compteur_hole
 * - la roue codeuse comporte 2 Hole --> + 2 au compteur_hole = 1 tours
 * 
************************************************************************************************/
void measure_filament() {
	if (digitalRead(PIN_FOURCHE)){
		counter_steps += 1;
	}
}


/********************************* Fonction de Lecture du Joystick *****************************
 *
 *  - Lit le joystick en X et Y sur les pin Analog
 * - Mets à jours l'IHM en fonction des fenetre dans lequel ce trouve l'utilisateur
 * - Mets à jours les Flags Y_PLUS et Y_MOINS
 * 
************************************************************************************************/

void read_joystick() {

	int XValue = analogRead(PIN_X);     // Read the analog value from The X-axis from the joystick
	int YValue = analogRead(PIN_Y);     // Read the analog value from The Y-axis from the joystick

	// ************************* Analyse de l'axe Y *********************************

  	if (!Y_PLUS && !Y_MOIN){
		if (YValue < 10){ // joystick Y - -> 
			Y_MOIN = 1;
			Y_PLUS = 0;
		}
		else if (YValue > 800 ){ // joystick Y +  -> 
			Y_MOIN = 0;
			Y_PLUS = 1;
		}
	}
	else if (YValue < 800 && YValue > 50){        // Y en home position      
		Y_MOIN = 0;
		Y_PLUS = 0;
	} 

	// ************************* Analyse de l'axe X *********************************

    if (XValue < 400 && XValue > 10){ // joystick X - -> reduce motor speed
		if (MOTOR_RUN && target_speed > 5 ){
			target_speed = target_speed - 1;
      	}
    }
    else if (XValue < 10){ // joystick X - -> Stop motor
        if (MOTOR_RUN && target_speed > 5 ){
			target_speed = target_speed - 1;
      	}
		else if (MOTOR_RUN && target_speed <= 5 ){
          	MOTOR_RUN = 0;
        }
        else if ( actual_speed <= 0){
			Y_MOIN = 0;
			Y_PLUS = 0;
			MOTOR_ENABLE = 0;
			digitalWrite(PIN_ENABLE_DRIVER, HIGH);
        }
    }
   	else if (XValue > 800){ // joystick X + -> Start motor or Speed
      	if (!MOTOR_RUN){
			target_speed = max_speed/2;
			//MOTOR_RUN = 1;
		}
		if (MOTOR_RUN){
			if (target_speed < max_speed){
				target_speed += 1;
			}
      	}
    }

	// ************************* Analyse des Commandes ***************************
	// Vérifier q'une commande est activé et mettre le programme en attente de 
	// la prochaine commande.


	

	if (commande_waiting)
	{
		if (Y_PLUS || Y_MOIN)
		{
			commande_push = 1;
		}
	}
	else if(!commande_waiting)
	{
		if (!Y_PLUS && !Y_MOIN)
		{
			commande_waiting = 1;
			commande_push = 0;
		}
	}

	
}

/********************************* Update LCD **************************************************
 *
 * - Fonction qui met a jours l'IHM en fonction de la fenetre dans lequels on est
 * - Nom des fenetres :
 * 		- window_Menu
 * 		- window_Manual_run
 * 		- window_Auto_init
 * 		- window_Auto_run
 * 		- window_Paused
 * 		- window_Finish
 * 		- window_Fail
 *  
 * 
************************************************************************************************/
void updateLCD() {
	lcd.setCursor(0,0);
	lcd.print(commande_push);
	lcd.print(commande_waiting);

	if(window_Fail)
	{
		//********| System Failure |*********//
		//********|<-Abort  Retry->|*********//
		lcd.setCursor(0, 0);
		lcd.print(" System Failure ");
		lcd.setCursor(0, 1);
		lcd.print("<-Abort  Retry->");
	}
	else if (window_Menu)
	{
		//********| Spool Measurer |*********//
		//********|<-Manual  Auto->|*********//
		lcd.setCursor(0, 0);
		lcd.print(" Spool Measurer ");
		lcd.setCursor(0, 1);
		lcd.print("<-Manual  Auto->");
	}
	else if (window_Manual_run)
	{
		//********|M.   Lgt:  1.93m|*********//
		//********|  Speed:0   rpm |*********//
		lcd.setCursor(0, 0);
		lcd.print("M.   Lgt:");
		lcd.setCursor(9, 0);
		lcd.print("      ");
		lcd.setCursor(9, 0);
		lcd.print(measurement);
		lcd.setCursor(15, 0);
		lcd.print("m");
		lcd.setCursor(0, 1);
		lcd.print("  Speed:");
		lcd.setCursor(9, 1);
		lcd.print("    ");
		lcd.setCursor(9, 1);
		lcd.print(actual_speed);
		lcd.setCursor(12, 1);
		lcd.print("rpm ");
	}
	else if (window_Manual_init)
	{
		//********|M.   Lgt:  1.93m|*********//
		//********|<-Menu   Start->|*********//
		lcd.setCursor(0, 0);
		lcd.print("M.   Lgt:");
		lcd.setCursor(9, 0);
		lcd.print("      ");
		lcd.setCursor(9, 0);
		lcd.print(measurement);
		lcd.setCursor(15, 0);
		lcd.print("m");
		lcd.setCursor(0, 1);
		lcd.print("<-Menu   Start->");
		
	}
	else if (window_Auto_init)
	{
		//********| Choose length! |*********//
		//********| Lgt:  23.01 m  |*********//
		lcd.setCursor(0, 0);
		lcd.print(" Choose length! ");
		lcd.setCursor(0, 1);
		lcd.print(" Lgt: ");
		lcd.setCursor(6, 1);
		lcd.print("      ");
		lcd.setCursor(6, 1);
		lcd.print(measurement);
		lcd.setCursor(12, 1);
		lcd.print(" m  ");
	}
	else if (window_Auto_run)
	{
		//********| 002.22m/230.00m|*********//
		//********|  Speed:0   rpm |*********//
		lcd.setCursor(0, 0);
		lcd.print(" ");
		lcd.setCursor(1, 0);
		lcd.print("      ");
		lcd.setCursor(1, 0);
		lcd.print(measurement);
		lcd.setCursor(7, 0);
		lcd.print("m/");
		lcd.setCursor(9, 0);
		lcd.print("      ");
		lcd.setCursor(9, 0);
		lcd.print(measurement_target);
		lcd.setCursor(15, 0);
		lcd.print("m");

		lcd.setCursor(0, 1);
		lcd.print("  Speed:");
		lcd.setCursor(9, 1);
		lcd.print("    ");
		lcd.setCursor(9, 1);
		lcd.print(actual_speed);
		lcd.setCursor(12, 1);
		lcd.print("rpm ");
	}
	else if (window_Auto_paused || window_Manu_paused)
	{
		//********|     Paused     |*********//
		//********|     23.01 m    |*********//
		lcd.setCursor(0, 0);
		lcd.print("     PAUSED     ");
		lcd.setCursor(0, 1);
		lcd.print("    ");
		lcd.setCursor(4, 1);
		lcd.print("      ");
		lcd.setCursor(4, 1);
		lcd.print(measurement);
		lcd.setCursor(9, 1);
		lcd.print(" m    ");
	}
	else if (window_Finish)
	{
		//********|   Finished !   |*********//
		//********|     23.01 m    |*********//
		lcd.setCursor(0, 0);
		lcd.print("   Finished !   ");
		lcd.setCursor(0, 1);
		lcd.print("    ");
		lcd.setCursor(4, 1);
		lcd.print("      ");
		lcd.setCursor(4, 1);
		lcd.print(measurement);
		lcd.setCursor(9, 1);
		lcd.print(" m    ");
	}
	else
	{
		//********| System Failure |*********//
		//********|<-Abort  Retry->|*********//
		lcd.setCursor(0, 0);
		lcd.print(" System Failure ");
		lcd.setCursor(0, 1);
		lcd.print("<-Abort  Retry->");
	}

	/*lcd.setCursor(0,0);
	lcd.print("Speed: ");
	lcd.print(actual_speed);
	lcd.print("RPM ");

	lcd.setCursor(0,1);
	lcd.print(counter_steps);
	lcd.print("   ");*/
	
} 

/********************************* Reset ecran IHM **************************************************
 *
 * - Fonction remet toute les variables d'écrans LCD à 0
*
************************************************************************************************/
void resetIHM() {
	window_Menu 		= 0;
	window_Manual_run 	= 0;
	window_Manual_init	= 0;
	window_Manu_paused	= 0;
	window_Auto_init 	= 0;
	window_Auto_run 	= 0;
	window_Auto_paused	= 0;
	window_Finish 		= 0;
	window_Fail 		= 0;
}