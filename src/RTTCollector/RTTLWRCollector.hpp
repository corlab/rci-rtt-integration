#pragma once

#include "../NestedTaskContext.hpp"

#include <iostream>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <rci/dto/JointAngles.h>
#include <rci/dto/JointVelocities.h>
#include <rci/dto/JointTorques.h>

#include <nemo/Mapping.h>
#include <nemo/Vector.h>

#include "RCICollector.hpp"

class RTTCollector: public NestedTaskContext {
public:
	RTTCollector(std::string const& name);

	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();
};
