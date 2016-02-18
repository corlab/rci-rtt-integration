#pragma once

#include <iostream>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <rci/dto/JointAngles.h>
#include <rci/dto/JointVelocities.h>
#include <rci/dto/JointTorques.h>

#include <nemo/Mapping.h>
#include <nemo/Vector.h>

#include "RCISplitter.hpp"

class RTTSplitter: public RTT::TaskContext {
public:
	RTTSplitter(std::string const& name);

	bool connectPortTo(std::string portOut, std::string PortIn,
			RTT::ConnPolicy connP);
	void configureJointNodes();

	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

protected:
	void configureJointNodesDefault();
	RTT::base::PortInterface* findNestedPort(
			std::vector<std::string>& nestedPath, TaskContext* context);

	std::vector<boost::shared_ptr<RTT::TaskContext> > registeredSplitterNodes;
};
