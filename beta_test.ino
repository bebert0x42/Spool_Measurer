



/* Beta test code pour projet PIC IMT11 SpoolMeasurer */
/* Programme de test du materiel*/
/*
Liste des Bug et truc a mettre en place :
- Bug : quand on fait trop vite back back back le moteur ne se coupe pas


A faire :

- Calibration plus précise de la roues ? (actuelement 260cm réél mesuré 260+/- 3cm)
- reduire le frame rate du LCD (avec millis())
- Intégration du Servo moteur
- Montage electrique
- Création boitier electrique
- Finir les fixations
- réduire les tiges filetés


Fait/traité :
- Bug : Sur Auto_Init la modification de la valeure cible ne fonctionne pas bien (a cause des variables commande_push et commande_waiting)
- Add : Ajout d'un fine tunning lors du choix de la valeurs de la target a mesurer
- Add : Ajout d'une fonction reset target dans Window_Auto_init en clickan sur le joystick
- Bug : Sur Autu_run le joystick gauche vas sur Finished et non sur pause idem pour le droit.
- Bug : Sur l'ecran Manual Run, si on met la vitesse à 0 on ne peut plus la remonté directement sans passer par un autre menu
- Bug : Reset du compteur Timout APRES la mise en pause en auto et manu
- Bug : Sur tout les ecrans + et moins sont a désactivé sauf les ecrans de run
- Add : Ajout des execptions timeout
- Bug : Une fois la target atteint le programme continue (sur la fenetre Auto_Run)
- Bug : Si la vitesse de rotation est à 0 le timeout déclenche quand même
- Add : Passage en 1/8 de steps pour limiter le bruit moteur (configurable)
- Test : Test a 24V, le couple est un peu meilleur mais pas un gain important
- Test : L'alimentation 12V 2A suiffit (max 12V 1.2A utilisé)
- Add : Modification de la rampe de vitesse et dimminution de la vitesse max
- Bug : Lors de l'abort si la roue tourne le programme reprend (sur la fenetre Fail)
- Bug : le moteur n'est pas coupé lors de l'abort
- Bug : Affichage des RPM
- Add : définition complete des vitesses
- Add : Vérification de la limite d'améprage avec une alimentation plus forte 1.2 A max, 1A sur le driver
- Add : vérification en condition réel
- Add : Définition/calibration complete des distance_deceleration
- Bug : Apres une mesure, l'ancienne vitesse est en mémoire
- Add : Click sur le joystick sur les ecran window Manu reset le compteur
- BUG #1 Mesure non fiable, soit mettre un condensateur 100nf soit modifier le code pour une vérification de la lecture
		Update de la fonction measurement


*/

#include <TimerOne.h>
#include <LiquidCrystal_I2C.h>
#include <ServoTimer2.h>


//******* Déclaration des Pin de l'arduino **********

	// PIN pour le Driver/Moteur A4988
	#define PIN_STEP 9
	#define PIN_DIR 8
	#define PIN_ENABLE_DRIVER 10

	LiquidCrystal_I2C lcd(0x27, 16, 2); // définit le type d'écran lcd avec connexion I2C sur les PIN SLA SCL

	//Pin du Joystick
	const int PIN_X = A0;	 // analog pin connected to X output
	const int PIN_Y = A1;	 // analog pin connected to Y output
	const int PIN_CLICK = 3; // Digital pin connected to Y output
	//Pin Fourche Optique
	const byte PIN_FOURCHE = 2; //Digital Pin connected to optical switch
	const int PIN_SERVO = 4;	// PWM Pin pour le servo moteur

//******* Déclaration des Variables de statu *************
	//Variable des bouton joystick
	bool X_PLUS = 0;
	bool X_MOIN = 0;
	bool Y_PLUS = 0;
	bool Y_MOIN = 0;
	bool CLICK = 0;
	// Variable pour attendre que le joystick se remete en 0
	bool commande_waiting = 1;
	// Variable pour push une commande joystick
	bool commande_push = 0;
	int debug_var = 0; // variable de debugage

//****** Variable de statu ******

	// Démare/arrete le moteur a une vitesse donnée
	bool MOTOR_RUN = 0;
	// Active/Désactive le moteur et son couple
	bool MOTOR_ENABLE = 0;

//****** Variable de de fenetre IHM******

	//********| Spool Measurer |*********//
	//********|<-Manual  Auto->|*********//
	bool window_Menu = 1;
	//********|M.   Lgt:  1.93m|*********//
	//********|  Speed:0   rpm |*********//
	bool window_Manual_run = 0;
	//********|M.   Lgt:  1.93m|*********//
	//********|<-Menu   Start->|*********//
	bool window_Manual_init = 0;
	//********|     Paused     |*********//
	//********|     23.01 m    |*********//
	bool window_Manu_paused = 0;
	//********| Choose length! |*********//
	//********| Lgt:  23.01 m  |*********//
	bool window_Auto_init = 0;
	//********| 002.22m/230.00m|*********//
	//********|  Speed:0   rpm |*********//
	bool window_Auto_run = 0;
	//********|     Paused     |*********//
	//********|     23.01 m    |*********//
	bool window_Auto_paused = 0;
	//********|   Finished !   |*********//
	//********|     23.01 m    |*********//
	bool window_Finish = 0;
	//********| System Failure |*********//
	//********|<-Abort  Retry->|*********//
	bool window_Fail = 0;

//******* Variable pour le fonctinnement moteur **********
	int actual_speed = 1;
	int actual_direction;
	int target_speed;
	int ticks;
	int tick_count;
	// Constante de direction du moteur
	#define FORWARD HIGH
	#define BACKWARD LOW

//*********** Parametre de vitesse du moteur ***************

	const int speed_ticks[] = {-1, 600, 575, 550, 525, 500, 475, 450, 425, 400, 375, 350, 325, 300, 275, 240, 200, 170, 135, 120, 110, 95, 85, 75};
	size_t max_speed = (sizeof(speed_ticks) / sizeof(speed_ticks[0])) - 1; // nombre de variable dans le tableau
	const int microsteps = 8;											   // 1 Full step, 2 Half, 4 1/4 de step, 8 1/8 de steps

//******* Variable pour la fonction de mesure **********

	//Compteur du nombre de steps de la roue codeuse
	unsigned int counter_steps = 0;
	//Mesure de la longueure filament à l'instant T (afficher sur l'IHM)
	float measurement = 0;
	//La mesure ciblé de longueure de filament
	float measurement_target = 0;

	//                           IIIII
	//                           IIIII
	//     Variable pour         IIIII
	//      la calibration     IIIIIIIII
	//                          IIIIIII
	//                           IIII
	//                            II

	//périmetre de la roue denté de mesure en mm
	const float perimeter_gear = 0.0339;
	//nombre de fenetre sur la roue codeuse
	const int encoder_hole = 2;
	//Course de la bobine lors de la décélération de 60 RPM à 0
	const float distance_deceleration = 0.50;
	//Variable de temporisation pour éviter les faux signaux de la fourche optique
	static volatile unsigned long debounce = 0;
	//Temps de latence de la fourche optique pour s'assurer d'une bonne mesure
	const int latence_fourche = 500;

//******* Variable pour la fonction de TimeOut **********

	//Variable du dernier temps de mesure via millis()
	unsigned long previous_time_measurement = 0;
	//Temps en millisecond avant timeout (0.500s)
	const unsigned long timeout = 1000;
	//Variable du dernier temps de mesure via millis() + Timeout
	unsigned long check_time_measurement = 0;

//******* Variable pour la fonction guide du servo moteur **********

	ServoTimer2 servo_guide; //Servo name is myservo

	// Position du millieu du servo moteur en degres pendant le fonctionnement
	const int position_middle = 1500;
	// Course du servo moteur en microseconde pendant le fonctionnement
	const int position_variation = 600;
	// Position de départ du servo moteur en microseconde  pendant le fonctionnement
	const int position_start = position_middle - position_variation / 2;
	// Position de fin du servo moteur en microseconde pendant le fonctionnement
	const int position_end = position_middle + position_variation / 2;
	// Position actuel du servo moteur en microseconde (position de départ au démarage)
	int position_servo = position_start;
	// Dernier temps en millis() que le servo moteur a bougé
	unsigned long position_tempo = 0;
	// Temps entre chaque mouvement d'une microseconde du servo moteur (en fonction de la vitesse du moteur)
	unsigned int position_running_time = speed_ticks[actual_speed];
	// Sens de direction du servo moteur
	bool direction_servo = 0;

/*
 * =============================================================================================
 * ========================================= Setup =============================================
 * =============================================================================================
*/
void setup()
{

	digitalWrite(PIN_ENABLE_DRIVER, HIGH); // Désactivation du moteur avant le setup

	// initial values
	actual_speed = 0;
	target_speed = max_speed / 2;
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
	digitalWrite(PIN_ENABLE_DRIVER, HIGH); // Désactivation du moteur
	pinMode(PIN_STEP, OUTPUT);
	pinMode(PIN_DIR, OUTPUT);
	digitalWrite(PIN_DIR, actual_direction);

	// Pin Fourche
	pinMode(PIN_FOURCHE, INPUT_PULLUP);
	pinMode(PIN_CLICK, INPUT_PULLUP);

	//init du servo Moteur Guide
	servo_guide.attach(PIN_SERVO); // attaches the servo signal pin
	servo_guide.write(position_start);
}
/*
 * =============================================================================================
 * ======================================== Main Loop ==========================================
 * =============================================================================================
*/
void loop()
{

	measurement = counter_steps * perimeter_gear / encoder_hole;

	read_joystick();   					// Lectue Joystick
	running_program(); 					// Boucle programme principal
	updateLCD();	   					// Mise a jours du LCD
	check_Target();						// Vérification si la mesure est proche de la cible
	servoGuide_Running();				// Rotation du Servo moteur Guide
	if (MOTOR_ENABLE && !window_Fail) 	// Activation du moteur
	{
		digitalWrite(PIN_ENABLE_DRIVER, LOW);
	}
	else 								// Désactivation du moteur
	{
		digitalWrite(PIN_ENABLE_DRIVER, HIGH);
	}

	if (MOTOR_RUN && !window_Fail) 		//Augmentation de la vitesse à la target
	{
		increase_speed();
		
	}
	else 								//Diminution de la vitesse du moteur jusqu'à l'arret
	{
		decrease_speed();
	}

	//Vérification que nous ne somme pas en TimeOut (plus de mesure depuis X secondes)
	check_time_measurement = previous_time_measurement + timeout;

	if (actual_speed > 0 && check_time_measurement < millis())
	{
		if (window_Auto_run || window_Manual_run)
		{
			window_Fail = 1;
			emergency_stop();
		}
	}
	else if (actual_speed <= 0 && check_time_measurement < millis())
	{
		previous_time_measurement = millis();
	}
}

/********************************* Programme principal ****************************************
 * 
 *  - fonctionement du programme suivant la séquence IHM (voir fichier)
 * 
************************************************************************************************/
void running_program()
{
	if (!commande_push || !commande_waiting)
	{			// exclusion de windows fail ?
		return; //<- A tester VS le fonctionnement du programme en auto
	}

	if (window_Fail)
	{
		if (Y_MOIN)
		{ //abort du process et retour au menu principal
			resetIHM();
			MOTOR_RUN = 0;
			MOTOR_ENABLE = 0;
			window_Finish = 1;
		}
		if (Y_PLUS)
		{ //Retry avec reset du compteur timeout
			previous_time_measurement = millis();
			if (window_Auto_run || window_Auto_paused || window_Auto_init)
			{
				MOTOR_RUN = 1;
				window_Fail = 0;
			}
			else if (window_Manu_paused || window_Manual_init || window_Manual_run)
			{
				MOTOR_RUN = 1;
				window_Fail = 0;
			}
		}
	}
	else if (window_Menu)
	{
		counter_steps = 0;
		MOTOR_RUN = 0;
		if (Y_MOIN)
		{
			resetIHM();
			window_Manual_init = 1;
			target_speed = max_speed / 2;
		}
		else if (Y_PLUS)
		{
			resetIHM();
			window_Auto_init = 1;
			target_speed = max_speed / 2;
		}
	}
	else if (window_Manual_init)
	{
		if (Y_MOIN)
		{
			resetIHM();
			window_Menu = 1;
		}
		else if (Y_PLUS)
		{
			resetIHM();
			window_Manual_run = 1;
			MOTOR_RUN = 1;
			previous_time_measurement = millis();
		}
	}
	else if (window_Manual_run)
	{
		if (Y_MOIN)
		{
			MOTOR_RUN = 0;
			resetIHM();
			window_Manu_paused = 1;
		}
	}
	else if (window_Manu_paused)
	{
		if (Y_MOIN)
		{
			resetIHM();
			window_Finish = 1;
		}
		if (Y_PLUS)
		{
			resetIHM();
			MOTOR_RUN = 1;
			window_Manual_run = 1;
			previous_time_measurement = millis();
		}
	}
	else if (window_Finish)
	{
		if (Y_MOIN || Y_PLUS)
		{
			resetIHM();
			window_Menu = 1;
			MOTOR_RUN = 0;
			MOTOR_ENABLE = 0;
		}
	}
	else if (window_Auto_init)
	{
		if (Y_MOIN)
		{
			resetIHM();
			window_Menu = 1;
		}
		else if (Y_PLUS && measurement_target > 0)
		{
			resetIHM();
			window_Auto_run = 1;
			MOTOR_RUN = 1;
			previous_time_measurement = millis();
		}
	}
	else if (window_Auto_run)
	{
		if (Y_MOIN)
		{
			MOTOR_RUN = 0;
			resetIHM();
			window_Auto_paused = 1;
		}
	}
	else if (window_Auto_paused)
	{
		if (Y_MOIN)
		{
			resetIHM();
			window_Finish = 1;
		}
		if (Y_PLUS)
		{
			resetIHM();
			MOTOR_RUN = 1;
			window_Auto_run = 1;
			previous_time_measurement = millis();
		}
	}
	commande_waiting = 0;
}

/********************************* Fonction de check de la mesure VS Targe *********************
 * 
 * - Permet de vérifier si la valeur measurement est proche de la target pour diminuer la vitesse
 * du moteur
 * - Passe window_Finish à 1 si la target est ok et coupe le moteur
 * 
************************************************************************************************/
void check_Target()
{

	if (measurement_target > 0 && window_Auto_run)
	{
		/* Code pour le fonctionnement du moteur en fonction de la cible a mesurer*/
		if (measurement >= measurement_target)
		{
			MOTOR_RUN = 0;
			resetIHM();
			window_Finish = 1;
		}
		else if (measurement + distance_deceleration >= measurement_target)
		{
			target_speed = 1;
		}
	}
}

/********************************* Fonction Diminution de la vitesse Moteur *********************
 * 
 * - Augmentation de la vitesse moteur en fonction des valeurs du tableau speed_ticks
 * 
************************************************************************************************/
void increase_speed()
{

	if (actual_speed < target_speed)
	{
		actual_speed += 1;
		tick_count = 0;
		ticks = speed_ticks[actual_speed] / microsteps;
	}
	else if (actual_speed > target_speed)
	{
		actual_speed -= 1;
		tick_count = 0;
		ticks = speed_ticks[actual_speed] / microsteps;
	}
}

/********************************* Fonction Diminution de la vitesse Moteur *********************
 * 
 * - Diminution de la vitesse moteur en fonction des valeurs du tableau speed_ticks
 * 
************************************************************************************************/
void decrease_speed()
{

	if (actual_speed > 0)
	{
		actual_speed -= 1;
		tick_count = 0;
		ticks = speed_ticks[actual_speed] / microsteps;
	}
}

/********************************* Fonction arret d'urgence Moteur ******************************
 * 
 * - Non utilisé
 * 
************************************************************************************************/
void emergency_stop()
{
	actual_speed = 0;
	tick_count = 0;
	//ticks = speed_ticks[actual_speed / 5]/microsteps;
}

/********************************* Fonction TimerMotor *****************************************
 *
 * - Fonction lancé toute les 20ms avec au Timer1
 * - Fait tourner le moteur un tour en fonction de la vitesse
 * 
************************************************************************************************/
void timerMotor()
{
	if (actual_speed == 0)
		return;

	digitalWrite(PIN_ENABLE_DRIVER, LOW); // Activation du moteur
	MOTOR_ENABLE = 1;					  //Flag motor en marche

	tick_count++;

	if (tick_count == ticks)
	{
		// 1 step du moteur
		digitalWrite(PIN_STEP, HIGH);
		digitalWrite(PIN_STEP, LOW);

		tick_count = 0;
	}
}

/********************************* Fonction ServoGuide_Running *****************************************
 *
 * - Fonction pour mettre en mouvement le servo moteur du guide en fonction de la vitesse du moteur
 * - Fait tourner le servo moteur suivant les paramettres 
 * 
************************************************************************************************/
void servoGuide_Running()
{
	if (MOTOR_RUN && MOTOR_ENABLE)
	{
		position_running_time = speed_ticks[actual_speed] * 2 ;

		if ((position_tempo + position_running_time) < millis())
		{
			position_tempo = millis();
			if (!direction_servo && position_servo < position_end)
			{
				position_servo = position_servo + 20;
			}
			else if (!direction_servo && position_servo >= position_end)
			{
				direction_servo = !direction_servo;
			}
			else if (direction_servo && position_servo > position_start)
			{
				position_servo = position_servo - 20;
			}
			else if (direction_servo && position_servo <= position_start)
			{
				direction_servo = !direction_servo;
			}
			
			servo_guide.write(position_servo);
		}
	}
}

/********************************* Fonction de compteur de tour par interupt *******************
 *
 * - Ajoute 1 à la variable compteur_hole
 * - la roue codeuse comporte 2 Hole --> + 2 au compteur_hole = 1 tours
 * 
************************************************************************************************/
void measure_filament()
{
	// Vérifier à nouveau que le codeur envoie un bon signal puis vérifier que le temps est supérieur à 1000 microsecondes et vérifier à nouveau que le signal est correct.
	if (digitalRead(PIN_FOURCHE) && (micros() - debounce > latence_fourche) && digitalRead(PIN_FOURCHE))
	{
		counter_steps += 1;
		debounce = micros();
		previous_time_measurement = millis();
	}
}

/********************************* Fonction de Lecture du Joystick *****************************
 *
 *  - Lit le joystick en X et Y sur les pin Analog
 * - Mets à jours l'IHM en fonction des fenetre dans lequel ce trouve l'utilisateur
 * - Mets à jours les Flags Y_PLUS et Y_MOINS
 * 
************************************************************************************************/

void read_joystick()
{

	int XValue = analogRead(PIN_X);				  // Read the analog value from The X-axis from the joystick
	int YValue = analogRead(PIN_Y);				  // Read the analog value from The Y-axis from the joystick
	bool clickJoystick = !digitalRead(PIN_CLICK); // Read the digital value from Joystick Click

	// ************************* Analyse de l'axe Y *********************************

	if (!Y_PLUS && !Y_MOIN)
	{
		if (YValue < 10)
		{ // joystick Y - ->
			Y_MOIN = 0;
			Y_PLUS = 1;
		}
		else if (YValue > 800)
		{ // joystick Y +  ->
			Y_MOIN = 1;
			Y_PLUS = 0;
		}
	}
	else if (YValue < 800 && YValue > 50)
	{ // Y en home position
		Y_MOIN = 0;
		Y_PLUS = 0;
	}

	// ************************* Analyse de l'axe X *********************************

	if (XValue > 600 && XValue < 950) // joystick X - légée -> reduce motor speed ou longueur
	{
		if (window_Auto_run || window_Manual_run)
		{
			if (MOTOR_RUN && target_speed > 5)
			{
				target_speed = target_speed - 1;
			}
		}
		else if (window_Auto_init)
		{
			if (measurement_target > 0)
			{

				measurement_target = measurement_target - 0.01;
				return;
			}
		}
	}
	else if (XValue > 800)
	{ // joystick X - -> Stop motor ou choisir la longueur
		if (window_Auto_init)
		{
			if (measurement_target >= 1)
			{
				measurement_target = measurement_target - 1;
				return;
			}
		}
		else if (window_Auto_run || window_Manual_run)
		{
			if (MOTOR_RUN && target_speed > 5)
			{
				target_speed = target_speed - 1;
			}
			else if (MOTOR_RUN && target_speed <= 5)
			{
				MOTOR_RUN = 0;
			}
			else if (actual_speed <= 0)
			{
				MOTOR_ENABLE = 0;
			}
		}
	}
	else if (XValue < 450 && XValue > 2) // joystick X + légée ->  longueur + 0.1
	{
		if (window_Auto_init)
		{
			if (measurement_target > 0)
			{
				measurement_target = measurement_target + 0.01;
				return;
			}
		}
	}
	else if (XValue < 2)
	{ // joystick X + -> Start motor or Speed OR longueure
		if (window_Auto_init)
		{
			if (measurement_target < 400)
			{
				measurement_target = measurement_target + 1;
				return;
			}
		}
		else if (window_Auto_run || window_Manual_run)
		{
			if (!MOTOR_RUN)
			{
				target_speed = max_speed / 2;
				MOTOR_RUN = 1;
			}
			if (MOTOR_RUN)
			{
				if (target_speed < max_speed)
				{
					target_speed += 1;
				}
			}
		}
	}
	else if (clickJoystick) // Si on click sur le joystick
	{
		if (window_Auto_init || window_Manual_run || window_Manual_init) // reset de la target sur cette page
		{
			measurement_target = 0;
			counter_steps = 0;
			return;
		}
	}

	// ** Analyse des Commandes **
	// Vérifier q'une commande est activé et mettre le programme en attente de
	// la prochaine commande.
	if (commande_waiting)
	{
		if (Y_PLUS || Y_MOIN)
		{
			commande_push = 1;
		}
	}
	else if (!commande_waiting)
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
void updateLCD()
{
	//lcd.setCursor(0,0);
	//lcd.print(debug_var);

	if (window_Fail)
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
		lcd.print("M. Lgt : ");
		lcd.setCursor(9, 0);
		lcd.print("      ");
		lcd.setCursor(9, 0);
		lcd.print(measurement);
		//lcd.setCursor(15, 0);
		lcd.print("m            ");
		lcd.setCursor(0, 1);
		lcd.print("Speed : ");
		lcd.setCursor(8, 1);
		lcd.print("         ");
		lcd.setCursor(8, 1);
		lcd.print(actual_speed * 2.7);
		//lcd.setCursor(13, 1);
		lcd.print("rpm    ");
	}
	else if (window_Manual_init)
	{
		//********|M.   Lgt:  1.93m|*********//
		//********|<-Menu   Start->|*********//
		lcd.setCursor(0, 0);
		lcd.print("M. Lgt : ");
		lcd.setCursor(9, 0);
		lcd.print("         ");
		lcd.setCursor(9, 0);
		lcd.print(measurement);
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
		lcd.print(measurement_target);
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
		lcd.setCursor(8, 1);
		lcd.print(actual_speed * 2.7);
		lcd.setCursor(13, 1);
		lcd.print("rpm ");
	}
	else if (window_Auto_paused)
	{
		//********|     Paused     |*********//
		//********|     23.01 m    |*********//
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
		lcd.print("     PAUSED     ");
	}
	else if (window_Manu_paused)
	{
		//********|     Paused     |*********//
		//********|     23.01 m    |*********//
		lcd.setCursor(0, 0);
		lcd.print("M. Lgt : ");
		lcd.setCursor(9, 0);
		lcd.print("      ");
		lcd.setCursor(9, 0);
		lcd.print(measurement);

		lcd.setCursor(0, 1);
		lcd.print("     PAUSED     ");
	}
	else if (window_Finish)
	{
		//********|   Finished !   |*********//
		//********|     23.01 m    |*********//
		lcd.setCursor(0, 0);
		lcd.print("    Finished !   ");
		lcd.setCursor(0, 1);
		lcd.print("    ");
		lcd.setCursor(5, 1);
		lcd.print("      ");
		lcd.setCursor(4, 1);
		lcd.print(measurement);
		lcd.print("m         ");
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
}

/********************************* Reset ecran IHM **************************************************
 *
 * - Fonction remet toute les variables d'écrans LCD à 0
*
************************************************************************************************/
void resetIHM()
{
	window_Menu = 0;
	window_Manual_run = 0;
	window_Manual_init = 0;
	window_Manu_paused = 0;
	window_Auto_init = 0;
	window_Auto_run = 0;
	window_Auto_paused = 0;
	window_Finish = 0;
	window_Fail = 0;
}