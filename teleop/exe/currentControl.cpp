// 00-singlearm


// #include "../../common/initModules.h"
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
#include <ctime>

#include <fstream>
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

using namespace std;

#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION
#define CURRENT SOMATIC__MOTOR_PARAM__MOTOR_CURRENT
#define DOF 7

// Intialize variables
somatic_d_t daemon_cx;
somatic_motor_t singlearm;

bool start = true;
const int dof = 7;

double wf = 0.558048373585;

double currTime = 0.0;
double prevTime = 0.0;
double duration = 0.0;
double currentTime = 0.0;
clock_t startTime;
clock_t st;
double prev_Vel[7];
double qddot[7];
double prev_time;
bool flag = true;

int jj = 2;
//time_t startTime;

Eigen::Matrix<double, 7, 4> a, b;
double qRef[7] 		= {-0.187, 	 0.085,   -0.09,  -0.014,  -0.053,   0.187,   0.066} ;
double dqRef[7] 	= {0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000} ;
double current[7] 	= {0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000} ;

// Global variables for file objects
ofstream dataQ;
ofstream dataDotQ;
ofstream dataCur;
ofstream dataTimeStamp;
ofstream dataDDotQ;

// Global variables for dart objects
dart::dynamics::SkeletonPtr robotArm;
dart::utils::DartLoader dl;

// Global variables for states - joint positions and velocities
Eigen::Matrix<double, DOF, 1> qPos;
Eigen::Matrix<double, DOF, 1> qVel;
Eigen::Matrix<double, DOF, 1> qCur;
Eigen::Matrix<double, DOF, 1> qref;
Eigen::Matrix<double, DOF, 1> dqref;

// Global variables for M, Cg, ddq_cmd, and gain parameters
Eigen::Matrix<double, DOF, 1> eigCurrent;
Eigen::Matrix<double, DOF, DOF> Mass;
Eigen::Matrix<double, DOF, 1> Cg;
Eigen::Matrix<double, DOF, 1> ddq_cmd;
Eigen::Matrix<double, DOF, DOF> mKp; 
Eigen::Matrix<double, DOF, DOF> mKd;

/* ********************************************************************************************* */
void controlArm(){

		duration = clock() - st;
		currentTime = (duration *100)/CLOCKS_PER_SEC;

		currTime = (difftime(time(0), startTime));

		if(currentTime - prevTime >= 0.02) {
			double timeStep = currentTime - prevTime;

			prevTime = currentTime;
			somatic_motor_update(&daemon_cx, &singlearm);
			cout << "pos: "; for(int i=0; i<7; i++) { cout << singlearm.pos[i] <<  ", "; } cout << endl;
			cout << "vel: "; for(int i=0; i<7; i++) { cout << singlearm.vel[i] <<  ", "; } cout << endl;
			cout << "cur: "; for(int i=0; i<7; i++) { cout << singlearm.cur[i] <<  ", "; } cout << endl;

			if(flag == true){
				for(int i=0; i<7; i++) { prev_Vel[i] = singlearm.vel[i]; } cout << endl;
				prev_time = currentTime;
				cout << "Seconds: " << currentTime << " s" << endl;
				flag = false;
			}
			
			else{


				for(int i=0; i<7; i++) { dataQ 		<< singlearm.pos[i] <<  ", "; }	dataQ 	 << endl;
				for(int i=0; i<7; i++) { dataDotQ 	<< singlearm.vel[i] <<  ", "; } dataDotQ << endl;
				for(int i=0; i<7; i++) { dataCur 	<< singlearm.cur[i] <<  ", "; } dataCur  << endl;

				dataTimeStamp << currentTime << endl;

				for(int i=0; i<7; i++) 
				{ 
					qddot[i] = (singlearm.vel[i] - prev_Vel[i])/(currentTime - prev_time); 
				} 
		

				cout << "qddot: "; for(int i=0; i<7; i++) { cout << qddot[i] <<  ", "; } cout << endl;
				cout<< "current time: "<<currentTime<<endl;

				for(int i=0; i<7; i++) { dataDDotQ << qddot[i] <<  ", "; } dataDDotQ << endl;

				for(int i=0; i<7; i++) { prev_Vel[i] = singlearm.vel[i]; } cout << endl;
				prev_time = currentTime;
			
			}
		}

		for (int i = 0; i < dof; i++) {
				qRef[i] = 0;
		}

		// Update qref value
		for (int joint = 0; joint < dof; joint++) {
		    for (int l = 1; l <= 4; l++) {

		    	qRef[joint] = qRef[joint] + (a(joint, l-1)/(wf*l))*sin(wf*l*currTime)
                    - (b(joint, l-1)/(wf*l))*cos(wf*l*currTime);

                dqRef[joint] = a(joint, l-1)*cos(wf*l*currTime)
                    + b(joint, l-1)*sin(wf*l*currTime);

			    // dqref[joint] = dqref[joint] + a(joint,l-1)*cos(wf*l*currTime)
	      // 			+ b(joint, l-1)*sin(wf*l*currTime);
		  }
		}
		
		for(int i=0; i<7; i++) { 
			qPos(i)  = singlearm.pos[i] ;
			qVel(i)  = singlearm.vel[i] ;
			qCur(i)  = singlearm.cur[i] ;
			qref(i)  = qRef[i];
			dqref(i) = dqRef[i];
		}

		robotArm->setPositions(qPos);
		robotArm->setVelocities(qVel);

		// I want M and Cg
		Mass = robotArm->getMassMatrix();
		Cg 	 = robotArm->getCoriolisAndGravityForces();

		// I want ddq_ref = kp(q - qref) + kd(dq - qdref)
		ddq_cmd = mKp*(qref - qPos) + mKd*(dqref - qVel);

		// I compute current = M*ddq_ref + Cg
		eigCurrent = Mass*ddq_cmd + Cg;
		for(int i=0; i<7; i++) { 
			current[i]  = eigCurrent(i);
		}

		somatic_motor_cmd(&daemon_cx, &singlearm, CURRENT, current, 7, NULL);

		// somatic_motor_cmd(&daemon_cx, &singlearm, POSITION, qRef, 7, NULL);
		
		return;
}

// void readKeyboard(){
// }
// static inline void aa_fset( double *dst, double val, size_t n ) {
//     for( size_t i = 0; i < n; i ++ )
//         dst[i] = val;
// }

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
		// **limits[0] = -1024.1;
		// **limits[1] = -1024.1;
		// **limits[2] = -1024.1;
		// **limits[3] = -1024.1;
		// **limits[4] = 1024.1;
		// **limits[5] = 1024.1;
		// **limits[6] = 1024.1;
		// **limits[7] = 1024.1;
	for(size_t i = 0; i < 4; i++) aa_fset(*limits[i], -1024.1, 7);
	for(size_t i = 4; i < 8; i++) aa_fset(*limits[i], 1024.1, 7);
	// Update and reset them
	somatic_motor_update(&daemon_cx, &arm);
	somatic_motor_cmd(&daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
	usleep(1e5);
}

/* ********************************************************************************************* */
void init () {
		
	dataQ.open("/home/munzir/Documents/Software/experiments/teleop/build/dataQ.txt");
	dataDotQ.open("/home/munzir/Documents/Software/experiments/teleop/build/dataDotQ.txt");
	dataCur.open("/home/munzir/Documents/Software/experiments/teleop/build/dataCur.txt");
	dataTimeStamp.open("/home/munzir/Documents/Software/experiments/teleop/build/dataTimeStamp.txt");

	dataDDotQ.open("/home/munzir/Documents/Software/experiments/teleop/build/dataDDotQ.txt");
	
	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt));
	// dopt.ident = "00-singlearm";
	dopt.ident = "currentControl";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the arm
	initArm(daemon_cx, singlearm, "singlearm");

	// Set variables a, b
    // a << -0.009, -0.36, 0.311, -0.362,
   		//      -0.0, 0.0, -0.0, -0.0,
   		//      -0.0, 0.0, -0.0, -0.0,
   		//      -0.0, 0.0, -0.0, -0.0,
   		//      -0.0, 0.0, -0.0, -0.0,
   		//      -0.0, 0.0, -0.0, -0.0,
   		//      -0.0, 0.0, -0.0, -0.0;

    //  b <<  -0.051, 0.027, 0.003, -0.332,
   		//      -0.0, 0.0, -0.0, -0.0,
   		//      -0.0, 0.0, -0.0, -0.0,
   		//      -0.0, 0.0, -0.0, -0.0,
   		//      -0.0, 0.0, -0.0, -0.0,
   		//      -0.0, 0.0, -0.0, -0.0,
  		//      -0.0, 0.0, -0.0, -0.0;

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

     // Set initial position
     somatic_motor_cmd(&daemon_cx, &singlearm, POSITION, qRef, 7, NULL);
     usleep(3e6);
     //somatic_motor_update(&daemon_cx, &singlearm);
			//cout << "vel: "; for(int i=0; i<7; i++) { cout << singlearm.cur[i] <<  ", "; } cout << endl;


     // Using robot pointer to parse URDF
     robotArm = dl.parseSkeleton("/home/munzir/Documents/Software/experiments/teleop/exe/krangArm.urdf");
     robotArm->setGravity(Eigen::Vector3d (0, -9.81, 0) );	// remember to set g either in y or z axis

     // Setting up proportional and derivative gain parameters
     mKp = Eigen::Matrix<double, DOF, DOF>::Identity();
     mKd = Eigen::Matrix<double, DOF, DOF>::Identity();
     mKp = mKp*750.0;
     mKd = mKd*250.0;
}

/* ********************************************************************************************* */
// Continuously process the data and set commands to the modules
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	startTime = time(0);
	st = clock();

	// Unless an interrupt or terminate message is received, process the new message
	while(!somatic_sig_received) {

		// Read the keyboard data - TO BE IMPLEMENTED IN THE FUTURE
		// readKeyboard();
		
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
void destroy() {

	// Halt the Schunk modules
	somatic_motor_cmd(&daemon_cx, &singlearm, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);

	cout << "Destroyed daemons and halted modules" << endl;

	dataQ.close();
	dataDotQ.close();
	dataCur.close();
	dataTimeStamp.close();

	dataDDotQ.close();
}

/* ********************************************************************************************* */
int main() {
	init();
	
	run();

	destroy();

	return 0;
}