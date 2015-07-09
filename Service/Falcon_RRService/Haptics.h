/*
	File:			Haptics.h
	Author:			Gregory Grebe
	Description:	Handle the Falcon	
*/

#ifndef HAPTICS_H
#define HAPTICS_H

#include <hdl/hdl.h>
#include <hdlu/hdlu.h>

// Blocking values
const bool bNonBlocking = false;
const bool bBlocking = true;

class HapticsClass
{

//	Define callback functions as friends
	friend HDLServoOpExitCode ContactCB(void *data);
	friend HDLServoOpExitCode GetStateCB(void *data);

public:
	// Constructor
	HapticsClass();

	// Destructor
	~HapticsClass();

	// Initialize
	void init();

	// Clean up
	void uninit();

	// Get position
	void getPosition(double (&pos)[3]);

	// Get state of device button
	bool isButtonDown();

	// Get state of all buttons
	int getButtonStatus();

	// Set only the Z force
	void applyZForce(double force);

	//set the forces for all servos
	void setForces(double *forces);

	// Get the device workspace
	void getDeviceWorkspace(double(&workspace)[6]);
	
	// synchFromServo() is called from the application thread when it wants to exchange
	// data with the HapticsClass object. HDAL manages the thread synchronization
	// on behalf of the application.
	void synchFromServo();

	// Get ready state of device
	bool isDeviceCalibrated();

private:
	// Move data between servo and app variables
	void synch();

	// Check error result; display message and abort, if any
	void testHDLError(const char* str);

	// Matrix multiply
	void vecMultMatrix(double srcVec[3], double mat[16], double dstVec[3]);

	// Nothing happens until initialization is done
	bool m_inited;

	// Variables used only by servo thread
	double m_positionServo[3];
	bool m_buttonServo;
	int m_buttonStatusServo;
	double m_forceServo[3];

	// Variables used only by application thread
	double m_positionApp[3];
	bool m_buttonApp;
	int m_buttonStatusApp;

	// Handle to device
	HDLDeviceHandle m_deviceHandle;

	// Handle to Contact Callback
	HDLServoOpExitCode m_servoOp;

	// Device workspace dimensions
	double m_workspaceDims[6];

	// Transformation from Device coordinates to Application coordinates
	double m_transformMat[16];
};


#endif //HAPTICS_H