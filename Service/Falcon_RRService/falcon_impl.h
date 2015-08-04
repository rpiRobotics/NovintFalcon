#undef SendMessage
#include <RobotRaconteur.h>
#include "falcon_service.h"
#include "falcon_service_stubskel.h"
#include <boost/enable_shared_from_this.hpp>
#include "Haptics.h"

#pragma once

using namespace RobotRaconteur;
using namespace boost;
using namespace std;
using namespace falcon_service;

class Falcon_impl : public Falcon, public boost::enable_shared_from_this<Falcon_impl>
{
public:
	Falcon_impl(void);
	//Properties
	virtual RR_SHARED_PTR<ControllerInput > get_controller_input();
	virtual void set_controller_input(RR_SHARED_PTR<ControllerInput > value);

	virtual RR_SHARED_PTR<RobotRaconteur::RRArray<double > > get_position();
	virtual void set_position(RR_SHARED_PTR<RobotRaconteur::RRArray<double > > value);

	virtual int32_t get_button_status();
	virtual void set_button_status(int32_t value);

	virtual int32_t get_deadzone_enabled();
	virtual void set_deadzone_enabled(int32_t enabled);

	virtual int32_t get_deadzone_feedback_enabled();
	virtual void set_deadzone_feedback_enabled(int32_t enable);
	

	virtual int32_t get_deadzone_size();
	virtual void set_deadzone_size(int32_t percent);

	// Functions
	virtual void setForce(RR_SHARED_PTR<RobotRaconteur::RRArray<double > > force);

	void getWorkspaceDims(double(&workspace)[6]){ Haptics.getDeviceWorkspace(workspace); }

	~Falcon_impl(void);

private:
	HapticsClass Haptics;
	bool _deadzoneEnabled;
	int32_t _deadzoneSize;
	bool _deadzoneFeedbackEnabled;
};

