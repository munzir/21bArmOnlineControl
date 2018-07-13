// 01-singlearm



// #include "../../common/initModules.h"
// #include "../../common/motion.h"
//initModules stuff moved here
#include <unistd.h> //for usleep thing
#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <string>
#include <time.h>


using namespace std;

#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION

somatic_d_t daemon_cx;
// ach_channel_t some_channel_name;
somatic_motor_t singlearm;

//Intialize as false. User must tap "s" to start the arm
bool start = false;
double currTime = 0.0;
double dt = 0.001;
const int dof = 7;
double wf = 0.558048373585;
time_t startTime;

// Initialize variables a, b, and qref
Eigen::Matrix<double, 7, 4> a, b;
// Eigen::Matrix<double, 7, 1> dqref;
double dqref [7] = {0, 0, 0, 0, 0, 0, 0};
double q0 [7] = {0, 0, 0, 0, 0, 0, 0};

/* ********************************************************************************************* */
void controlArm(){
	
	// Check if the user has started the arm
	if (start){
		//get current time
		currTime = difftime(time(0), startTime);
		
		// Update qref value
		for (int joint = 0; joint < dof; joint++) {
		    for (int l = 1; l <= 4; l++) {
		      dqref[joint] = 0 + a(joint,l-1)*cos(wf*l*currTime)
     		 + b(joint, l-1)*sin(wf*l*currTime);   
		  }
		}
		
		somatic_motor_cmd(&daemon_cx, &singlearm, VELOCITY, dqref, 7, NULL);
		
		return;
	}
	else {
		cout << "You must start the program by pressing 's' " << endl;
		return;
	}
}

void readKeyboard(){
	
	char input;
	cin>>input;

	switch(input)
	{
		case 's':
			{
				start = true;
				cout << "Arm is ready to move. Press 'x' to lock" << endl;
				break;
			}
		case 'x':
			{	start = false;
				cout << "Arm is soft locked. Press 's' to unlock" << endl;
				break;
			}
	}

	return;
}

//I think we should just iterater the control while reading file instead of reading the whole thing.
/*void controlArm_Tianhang(){


		int row = 0;
		int col;
		string delimiter = " ";
		size_t pos = 0;
		string value;

		string line;
		ifstream qdots("dataQdot.text");
		//get each line of the text and put it into variable "line"
		while (getline(qdots, line)){
			if (row != 0){
				col = 0; //start from zeroth column for each row


				double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

				//delimit by commas in the line and put values in velData matrix
				while ((pos = line.find(delimiter)) != string::npos) {
				    //get first value after delimiting
				    value = line.substr(0, pos);
				    //add delimited value to matrix
				    dq[col] = atof(value.c_str());
				    //erase first value + delimiter from line
				    line.erase(0, pos + delimiter.length());
				    //to access next column in velData matrix in the next loop
				    ++col;
				}
				++row;

				somatic_motor_cmd(&daemon_cx, &singlearm, VELOCITY, dq, 7, NULL);
			}
		}
}*/

/* ********************************************************************************************* */
/// Initializes the arm
static void initArm (somatic_d_t& daemon_cx, somatic_motor_t& arm, const char* armName) {	

	// Get the channel names
	char cmd_name [16], state_name [16];
	sprintf(cmd_name, "%s-cmd", armName);
	sprintf(state_name, "%s-state", armName);

	// Initialize the arm with the daemon context, channel names and # of motors
	somatic_motor_init(&daemon_cx, &arm, 7, cmd_name, state_name);
	usleep(1e5);

	// Set the min/max values for valid and limits values
	double** limits [] = {
		&arm.pos_valid_min, &arm.vel_valid_min, 
		&arm.pos_limit_min, &arm.pos_limit_min, 
		&arm.pos_valid_max, &arm.vel_valid_max, 
		&arm.pos_limit_max, &arm.pos_limit_max};

		**limits[1] = -1024.1;
		**limits[2] = -1024.1;
		**limits[3] = -1024.1;
		**limits[4] = -1024.1;
		**limits[5] = 1024.1;
		**limits[6] = 1024.1;
		**limits[7] = 1024.1;
		**limits[8] = 1024.1;

	// for(size_t i = 0; i < 4; i++) aa_fset(*limits[i], -1024.1, 7);
	// for(size_t i = 4; i < 8; i++) aa_fset(*limits[i], 1024.1, 7);
	
	// Update and reset them
	somatic_motor_update(&daemon_cx, &arm);
	somatic_motor_cmd(&daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
	usleep(1e5);

	cout << "************************" << endl;
	cout << "Arm has been initialized" << endl;
	cout << "************************" << endl;
}


void setInitPosition(){
		somatic_motor_cmd(&daemon_cx, &singlearm, POSITION, q0, 7, NULL);
}

/* ********************************************************************************************* */
/// Continuously process the joystick data and sets commands to the modules
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	startTime = time(0);

	// Unless an interrupt or terminate message is received, process the new message
	while(!somatic_sig_received) {


		// Read the keyboard data - TO BE IMPLEMENTED IN THE FUTURE
		// readKeyboard();

		// Control the arm
		// controlArm_Tianhang();
		// controlArm();

		// Free buffers allocated during this cycle
		aa_mem_region_release(&daemon_cx.memreg);	
		usleep(1e4);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}


/* ********************************************************************************************* */
void init () {
	
	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "01-singlearm";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the arm
	initArm(daemon_cx, singlearm, "singlearm");

	// Set variables a, b
  	a << -0.009, -0.36, 0.311, -0.362,
        0.095, -0.132, -0.363, 0.474,
        -0.418, -0.25, -0.12, 0.119,
        0.023, 0.113, 0.497, 0.213,
        -0.23, -0.237, 0.153, -0.147,
        0.366, 0.366, 0.302, -0.373,
        -0.247, -0.166, 0.315, 0.031;

    b <<  -0.051, 0.027, 0.003, -0.332,
        -0.292, 0.358, -0.056, -0.436,
        -0.355, 0.039, -0.397, -0.445,
        0.328, 0.256, -0.36, 0.143,
        0.428, 0.093, 0.035, -0.28,
        -0.39, -0.085, 0.388, 0.46,
        -0.046, 0.135, -0.428, 0.387;

     setInitPosition();

	// dqref = {0, 0, 0, 0, 0, 0, 0};

	// Initialize the joystick channel
	/*int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);*/
}

/* ********************************************************************************************* */
void destroy() {

	// Halt the Schunk modules
	somatic_motor_cmd(&daemon_cx, &singlearm, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);

	//
	cout << "Destroyed and halted" << endl;
}

/* ********************************************************************************************* */
int main() {

	init();
	run();
	destroy();

	return 0;
}