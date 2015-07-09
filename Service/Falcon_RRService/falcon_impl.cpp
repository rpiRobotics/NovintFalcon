#include "falcon_impl.h"
#include "Vector3.h"


Falcon_impl::Falcon_impl(void)
{
	Haptics.init();
	
	Sleep(100);
	if(!Haptics.isDeviceCalibrated())
		MessageBox(NULL, 
					// The next two lines are one long string
					"Please home the device by extending\n"
					"then pushing the arms all the way in.",
					"Not Homed",
					MB_OK);

}


Falcon_impl::~Falcon_impl(void)
{
	Haptics.uninit();
}

RR_SHARED_PTR<RobotRaconteur::RRArray<double > > Falcon_impl::get_position(void)
{
	double pos[3];
	Haptics.synchFromServo();
	Haptics.getPosition(pos);
	RR_SHARED_PTR<RRArray<double> > returnPos = AttachRRArrayCopy(pos, 3);
	return returnPos;
}
void Falcon_impl::set_position(RR_SHARED_PTR<RobotRaconteur::RRArray<double > > value)
{
	throw runtime_error("Read only property");
}

int32_t Falcon_impl::get_button_status()
{
	// Update the information from the servo thread
	Haptics.synchFromServo();
	// get the buttons status from the Haptics object
	int buttons = Haptics.getButtonStatus();
	return buttons;
}

void Falcon_impl::set_button_status(int32_t value)
{
	throw runtime_error("Read only property");
}


RR_SHARED_PTR<ControllerInput > Falcon_impl::get_controller_input(void)
{
	// Update info from the Haptics object
	Haptics.synchFromServo();
	int buttons = Haptics.getButtonStatus();
	double pos[3];
	Haptics.getPosition(pos);

	Vector3 vector(pos[0], pos[1], pos[2]);
	/*
	 * Values are hard coded... 
	 *		0.06 is the max device workspace of the falcon.
	 *		either make that available or call the function to get the info
	 */
	double maxPos = 0.06;
	double threshold = 0.2 * maxPos;
	Vector3 normalized = vector.Normalize(threshold, maxPos);

	int32_t posInt[3];
	posInt[0] = floor(normalized.X * 10000);
	posInt[1] = floor(normalized.Y * 10000);
	posInt[2] = floor(normalized.Z * 10000);

	// Create our Input Class
	RR_SHARED_PTR<ControllerInput > input = RR_MAKE_SHARED<ControllerInput >();
	input->center_button = (buttons & HDL_BUTTON_1) == HDL_BUTTON_1 ? 1 : 0;
	input->left_button	 = (buttons & HDL_BUTTON_2) == HDL_BUTTON_2 ? 1 : 0;
	input->top_button	 = (buttons & HDL_BUTTON_3) == HDL_BUTTON_3 ? 1 : 0;
	input->right_button  = (buttons & HDL_BUTTON_4) == HDL_BUTTON_4 ? 1 : 0;
	
	input->positionX = posInt[0];
	input->positionY = posInt[1];
	input->positionZ = posInt[2];
	
	return input;
}

void Falcon_impl::set_controller_input(RR_SHARED_PTR<ControllerInput > value)
{
	throw runtime_error("Read only property");
}

void Falcon_impl::setForce(RR_SHARED_PTR<RobotRaconteur::RRArray<double > >force)
{
	if (force->Length() != 3){
		throw runtime_error("Incorrect number of forces");
	}
	double *forces = force->ptr();
	Haptics.setForces(forces);
}