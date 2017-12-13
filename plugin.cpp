

#include <algorithm>
#include <iostream>
using namespace std;

#include "v_repExtCHAI3D.h"
#include "luaFunctionData.h"
#include "v_repLib.h"

#include "chai3d.h"
using namespace chai3d;

#ifdef _WIN32
#include <shlwapi.h>
#pragma comment(lib, "Shlwapi.lib")
#endif

#ifdef __APPLE__
#include <unistd.h>
#endif

#define PLUGIN_VERSION 1





//---------------------------------------------------------------------------
//
//  globals
//
//---------------------------------------------------------------------------
// Define pointer for the haptic
cHapticDeviceHandler* handler;
// Define haptic object
cGenericHapticDevicePtr hapticDevice;

// a flag to indicate if the haptic simulation currently running
bool simulationRunning = false;
// a flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;
// a global variable to store the position [m] of the haptic device
cVector3d hapticDevicePosition;

simInt v_repsphere;
simInt v_repsphereR;
simInt v_repcuboid;
simInt v_repsensor;
simInt v_repMass;

cVector3d Fcm, Fcs;
simFloat* posS;
simFloat* posH;
simFloat* velM;
cVector3d xs;
//Vm;
//cVector3d linearVelocity;
simFloat* lVM;
simFloat* angularVelocity;
cVector3d linearVelocity;
simFloat* lVS;
simFloat* angularVelocityS;
cVector3d linearVelocityS;
float Vs;
float Fs;
float Fcm_n;
float Fcs_n;
float vm;
float vs;
simFloat* tor;
simFloat* fcs;
int vHandle;
int fHandle;
int FF;
int VS;
int VM;
float xM;
float xS;

float vmOldx;
float vmOldy;
float vmOldz;

float vsOldx;
float vsOldy;
float vsOldz;

simFloat* force;
simFloat* torque;

int pHandle;
int XS;
int XM;


float fOldx;
float fOldy;
float fOldz;
cVector3d xm;
cVector3d xm_old;
cVector3d applyForce;

void updateHaptics(void);
void ppArch(cVector3d Vm, cVector3d Vs, cVector3d xm, cVector3d xs);
void pfArch(cVector3d Vm, cVector3d Vs, cVector3d xm, cVector3d xs, cVector3d FSlave);



float c1;		// tune
float c2;		// tune
float c3;		// tune
float c4;		// tune

float Mm;		// tune
float Bm;		// tune
float Km;		// tune

float Ms;		// tune
float Bs;		// tune
float Ks;		// tune

float Zcm;
float Zcs;


void updateHaptics(void)
{
	// retrieve position of haptic device
	hapticDevice->getPosition(xm);





	// scale the position in such a way we can cover more space in v-rep
	xm *= 20;
	// set reference MASTER sphere position
	posS[0] = xm(0);
	posS[1] = xm(1);
	posS[2] = xm(2);
	simSetObjectPosition(v_repsphereR, -1, posS);





	// get position of slave (Xs)
	//posH = new simFloat(3);
	simGetObjectPosition(v_repsphere, -1, posH);
	xs(0) = posH[0];
	xs(1) = posH[1];
	xs(2) = posH[2];





	//MASTER VELOCITY
	linearVelocity = (xm - xm_old) / simGetSimulationTimeStep();
	xm_old = xm;
	// print velocity
	std::cout << "Vm : " << linearVelocity(0) << " " << linearVelocity(1) << " " << linearVelocity(2) << std::endl;
	float VmNorm = sqrt(pow(linearVelocity(0), 2) + pow(linearVelocity(1), 2) + pow(linearVelocity(2), 2));
	// low pass filter (threshold is not 0 since force sensor will receive ever a force,even small: e-15
	//float alpha = 0.05;	// set filter parameter (frequency selector)
	//
	//	// filter on vx
	//	linearVelocity(0) = vmOldx + alpha*(linearVelocity(0) - vmOldx);
	//	vmOldx = linearVelocity(0);
	//	// filter on vy
	//	linearVelocity(1) = vmOldy + alpha*(linearVelocity(1) - vmOldy);
	//	vmOldy = linearVelocity(1);
	//	// filter on vz
	//	linearVelocity(2) = vmOldz + alpha*(linearVelocity(2) - vmOldz);
	//	vmOldz = linearVelocity(2);






	// linear velocity of slave
	simGetVelocity(v_repsphere, lVS, angularVelocityS);
	linearVelocityS(0) = lVS[0];
	linearVelocityS(1) = lVS[1];
	linearVelocityS(2) = lVS[2];
	// print slave velocity
	std::cout << "Vs: " << linearVelocityS(0) << " " << linearVelocityS(1) << " " << linearVelocityS(2) << std::endl;
	// compute norm of slave velocity
	float VsNorm = sqrt(pow(linearVelocityS(0), 2) + pow(linearVelocityS(1), 2) + pow(linearVelocityS(2), 2));
	 //low pass filter (threshold is not 0 since force sensor will receive ever a force,even small: e-15
		//// filter on vx
		//linearVelocityS(0) = vsOldx + alpha*(linearVelocityS(0) - vsOldx);
		//vsOldx = linearVelocityS(0);
		//// filter on vy
		//linearVelocityS(1) = vsOldy + alpha*(linearVelocityS(1) - vsOldy);
		//vsOldy = linearVelocityS(1);
		//// filter on vz
		//linearVelocityS(2) = vsOldz + alpha*(linearVelocityS(2) - vsOldz);
		//vsOldz = linearVelocityS(2);






	// force on slave (reaction force)
	// set force vector applied to haptic
	// read result of force sensor (Fs)
	simReadForceSensor(v_repsensor, force, torque);
	applyForce(0) = 0;
	applyForce(1) = 0;
	applyForce(2) = 0;			// z component is affected by gravity, therefore we don't consider it!
	// compute norm of unfiltered force to compare it with filtered one
	Fs = sqrt(pow(force[0], 2) + pow(force[1], 2));

	// low pass filtering and scaling (threshold is not 0 since force sensor will receive ever a force, even small: e-15
	if (Fs > 0.5) {
		// set filter parameter (frequency selector)
		float gamma = 0.002;
		// filter on fx
		applyForce(0) = fOldx + gamma*(force[0] - fOldx);
		fOldx = applyForce(0);
		// filter on fy
		applyForce(1) = fOldy + gamma*(force[1] - fOldy);
		fOldy = applyForce(1);
	}
	else {
		applyForce(0) = 0;
		applyForce(1) = 0;
		applyForce(2) = 0;
		fOldx = 0;
		fOldy = 0;
	}
	// retrieve new norm of filtered Fs
	float FsN = sqrt(pow(applyForce(0), 2) + pow(applyForce(1), 2));
	// print filtered force
	fHandle = simGetObjectHandle("graF");
	FF = simSetGraphUserData(fHandle, "FsN", FsN);





	// NOTE: choosing KD higher we achieve stability with few (almost zero) oscillations. In this way slave "anticipates" master during the motion.
	//       Instead, with a lower KD we achieve stability with more oscillations but slave doesn't "anticipate" master

	// P-P
	ppArch(linearVelocity, linearVelocityS, xm, xs);

	// P-F
	//pfArch(linearVelocity, linearVelocityS, xm, xs, applyForce);



}



void ppArch(cVector3d Vm, cVector3d Vs, cVector3d xm, cVector3d xs) {

	Mm = 0;
	Bm = 0.5;	// KDm
	Km = 2;		// KPm
	Ms = 0;
	Bs = 600;	// KDs
	Ks = 300;	// KPs

	Zcm = Mm + Bm + Km;
	Zcs = Ms + Bs + Ks;



	// compute control forces basing on previous tuning
	Fcm(0) = Bm*(Vs(0) - Vm(0)) + Km*(xs(0) - xm(0));
	Fcm(1) = Bm*(Vs(1) - Vm(1)) + Km*(xs(1) - xm(1));
	Fcm(2) = Bm*(Vs(2) - Vm(2)) + Km*(xs(2) - xm(2));

	Fcs(0) = Bs*(Vm(0) - Vs(0)) + Ks*(xm(0) - xs(0));
	Fcs(1) = Bs*(Vm(1) - Vs(1)) + Ks*(xm(1) - xs(1));
	Fcs(2) = Bs*(Vm(2) - Vs(2)) + Ks*(xm(2) - xs(2));


	// print forces
	std::cout << "Fcm x: " << Fcm(0) << std::endl;
	std::cout << "Fcm y: " << Fcm(1) << std::endl;
	std::cout << "Fcm z: " << Fcm(2) << std::endl;


	// set control forces
	fcs[0] = Fcs(0);
	fcs[1] = Fcs(1);
	fcs[2] = Fcs(2) + 9.81*66*2.2;
	tor[0] = 0;
	tor[1] = 0;
	tor[2] = 0;



	simHandleDynamics(simGetSimulationTimeStep());
	simAddForceAndTorque(v_repsphere, fcs, tor);
	hapticDevice->setForce(Fcm);


	// compare norm of velocities
	vm = sqrt(pow(Vm(0), 2) + pow(Vm(1), 2) + pow(Vm(2), 2));
	vs = sqrt(pow(Vs(0), 2) + pow(Vs(1), 2) + pow(Vs(2), 2));
	vHandle = simGetObjectHandle("graV");
	VS = simSetGraphUserData(vHandle, "VS", vs);
	VM = simSetGraphUserData(vHandle, "VM", vm);


	// compare norm of position
	xM = sqrt(pow(xm(0), 2) + pow(xm(1), 2) + pow(xm(2), 2));
	xS = sqrt(pow(xs(0), 2) + pow(xs(1), 2) + pow(xs(2), 2));
	pHandle = simGetObjectHandle("graP");
	XS = simSetGraphUserData(pHandle, "XS", xS);
	XM = simSetGraphUserData(pHandle, "XM", xM);

}





void pfArch(cVector3d Vm, cVector3d Vs, cVector3d xm, cVector3d xs, cVector3d FSlave) {

	// I keep blue ball fixed to better visualize vrep scene. Position of haptic is still sent to slave!!!
	posS[0] = 0;
	posS[1] = 0;
	posS[2] = 0;
	simSetObjectPosition(v_repsphereR, -1, posS);



	Mm = 0;
	Bm = 0;	// KDm; check if tuning Bm = 0 or 0.5 system is stable or not
	Km = 0;		// KPm
	Ms = 0;
	Bs = 600;	// KDs
	Ks = 300;	// KPs

	Zcm = Mm + Bm + Km;
	Zcs = Ms + Bs + Ks;



	// compute control forces basing on previous tuning
	Fcm = -FSlave - Bm*Vm;
	Fcs = Bs*(Vm - Vs) + Ks*(xm - xs);



	// print forces
	std::cout << "Fcm x: " << Fcm(0) << std::endl;
	std::cout << "Fcm y: " << Fcm(1) << std::endl;
	std::cout << "Fcm z: " << Fcm(2) << std::endl;


	// set control forces
	fcs[0] = Fcs(0);
	fcs[1] = Fcs(1);
	fcs[2] = Fcs(2) + 9.81*66*2.2;
	tor[0] = 0;
	tor[1] = 0;
	tor[2] = 0;


	simHandleDynamics(simGetSimulationTimeStep());
	simAddForceAndTorque(v_repsphere, fcs, tor);
	hapticDevice->setForce(Fcm);


	// compare norm of velocities
	vm = sqrt(pow(Vm(0), 2) + pow(Vm(1), 2) + pow(Vm(2), 2));
	vs = sqrt(pow(Vs(0), 2) + pow(Vs(1), 2) + pow(Vs(2), 2));
	vHandle = simGetObjectHandle("graV");
	VS = simSetGraphUserData(vHandle, "VS", vs);
	VM = simSetGraphUserData(vHandle, "VM", vm);


	// compare norm of position
	xM = sqrt(pow(xm(0), 2) + pow(xm(1), 2) + pow(xm(2), 2));
	xS = sqrt(pow(xs(0), 2) + pow(xs(1), 2) + pow(xs(2), 2));
	pHandle = simGetObjectHandle("graP");
	XS = simSetGraphUserData(pHandle, "XS", xS);
	XM = simSetGraphUserData(pHandle, "XM", xM);



}
//---------------------------------------------------------------------------
//
//  LUA interface
//
//---------------------------------------------------------------------------



LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

#define CONCAT(x,y,z)     x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)



// definitions for LUA_START_COMMAND
#define LUA_START_COMMAND "simExtCHAI3D_start"
const int inArgs_START[] = {
    3,
    sim_lua_arg_int,   0,
    sim_lua_arg_float, 0,
    sim_lua_arg_float, 0,
};



void LUA_START_CALLBACK(SLuaCallBack* p)
{

	// create a haptic device handler
	handler = new cHapticDeviceHandler();
	// get access to the first available haptic device found
	handler->getDevice(hapticDevice, 0);
	// open connection to haptic device
	hapticDevice->open();


	// get the sphere (slave) on V-REP scene
	v_repsphere = simGetObjectHandle("Sphere");
	// get the sphere (haptic) on V-REP scene
	v_repsphereR = simGetObjectHandle("SphereR");
	// get the cuboid on V-REP scene
	v_repcuboid = simGetObjectHandle("CuboidY");
	// get the force sensor on V-REP scene
	v_repsensor = simGetObjectHandle("Accelerometer_forceSensor");
	// get accelerometer mass
	v_repMass = simGetObjectHandle("Accelerometer_mass");

	// initialize filter variables
	fOldx = 0;
	fOldy = 0;
	fOldz = 0;




	//Old master position to retrieve the haptic velocity
	xm_old(0) = 0;
	xm_old(1) = 0;
	xm_old(2) = 0;


	tor = new simFloat(3);
	fcs = new simFloat(3);
	posH = new simFloat(3);
	posS = new simFloat(3);
	lVS = new simFloat(3);
	lVM = new simFloat(3);
	angularVelocity = new simFloat(3);
	angularVelocityS = new simFloat(3);
	force = new simFloat(3);
	torque = new simFloat(3);

	vmOldx = 0;
	vmOldy = 0;
	vmOldz = 0;

	vsOldx = 0;
	vsOldy = 0;
	vsOldz = 0;

}



// definitions for LUA_RESET_COMMAND
#define LUA_RESET_COMMAND "simExtCHAI3D_reset"
const int inArgs_RESET[] = {
    0
};



void LUA_RESET_CALLBACK(SLuaCallBack* p)
{



}





// definitions for LUA_READ_POSITION_COMMAND
#define LUA_READ_POSITION_COMMAND "simExtCHAI3D_readPosition"
const int inArgs_READ_POSITION[] = {
    1,
    sim_lua_arg_int, 0
};



void LUA_READ_POSITION_CALLBACK(SLuaCallBack* p)
{

	updateHaptics();

}





///  \brief V-REP shared library initialization.

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{


  char curDirAndFile[1024];
#ifdef _WIN32
  GetModuleFileName(NULL,curDirAndFile,1023);
  PathRemoveFileSpec(curDirAndFile);
#elif defined (__linux) || defined (__APPLE__)
  if (getcwd(curDirAndFile, sizeof(curDirAndFile)) == NULL) strcpy(curDirAndFile, "");
#endif
  std::string currentDirAndPath(curDirAndFile);
  std::string temp(currentDirAndPath);
#ifdef _WIN32
  temp+="\\v_rep.dll";
#elif defined (__linux)
  temp+="/libv_rep.so";
#elif defined (__APPLE__)
  temp+="/libv_rep.dylib";
#endif
  vrepLib=loadVrepLibrary(temp.c_str());
  if (vrepLib==NULL)
  {
    std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'CHAI3D' plugin.\n";
    return(0);
  }
  if (getVrepProcAddresses(vrepLib)==0)
  {
    std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'CHAI3D' plugin.\n";
    unloadVrepLibrary(vrepLib);
    return(0);
  }

  int vrepVer;
  simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
  if (vrepVer<30103)
  {
    std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'CHAI3D' plugin.\n";
    unloadVrepLibrary(vrepLib);
    return(0);
  }

  // register LUA commands

  std::vector<int> inArgs;

  CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_START,inArgs);
  simRegisterCustomLuaFunction(LUA_START_COMMAND,strConCat("number result=",LUA_START_COMMAND,"(number deviceIndex,number toolRadius,number workspaceRadius)"),&inArgs[0],LUA_START_CALLBACK);

  CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_RESET,inArgs);
  simRegisterCustomLuaFunction(LUA_RESET_COMMAND,strConCat("",LUA_RESET_COMMAND,"()"),&inArgs[0],LUA_RESET_CALLBACK);

  CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_READ_POSITION,inArgs);
  simRegisterCustomLuaFunction(LUA_READ_POSITION_COMMAND,strConCat("table_3 position=",LUA_READ_POSITION_COMMAND,"(number deviceIndex)"),&inArgs[0],LUA_READ_POSITION_CALLBACK);

  return PLUGIN_VERSION;
}



///  \brief V-REP shared library disconnect.

VREP_DLLEXPORT void v_repEnd()
{
  // stop haptic thread and cleanup

  unloadVrepLibrary(vrepLib);
}



///  \brief V-REP shared library message processing callback.

VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{
  int   errorModeSaved;
  void *retVal = NULL;


  cVector3d applyForce = { 0,0,0 };
  if (message == sim_message_eventcallback_simulationended)
  { // simulation ended. Destroy all BubbleRob instances:
	//allBubbleRobs.clear();
	  hapticDevice->setForce(applyForce);

  }

  simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
  simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);
  simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved);

  return retVal;
}
