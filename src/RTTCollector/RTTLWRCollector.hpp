#pragma once

#include "../NestedTaskContext.hpp"

#include <iostream>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <rci/dto/JointAngles.h>
#include <rci/dto/JointVelocities.h>
#include <rci/dto/JointTorques.h>
#include <rci/dto/JointImpedance.h>

#include <nemo/Mapping.h>
#include <nemo/Vector.h>

#include "../RTTCollector/RCICollector.hpp"

class RTTLWRCollector: public NestedTaskContext {
public:
	RTTLWRCollector(std::string const& name);

	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();
};
