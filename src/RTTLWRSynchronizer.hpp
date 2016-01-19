#pragma once

#include <iostream>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <rci/dto/JointAngles.h>

#include <nemo/Mapping.h>
#include <nemo/Vector.h>

#include "RTTLWRJoint.hpp"

class RTTLWRSynchronizer: public RTT::TaskContext {
public:
	RTTLWRSynchronizer(std::string const& name);

	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

protected:
	// override this to implement custom joint-node chain.
	void configureJointNodes();
	bool connectPortTo(std::string portOut, std::string PortIn, RTT::ConnPolicy connP);

	RTT::OutputPort<rci::JointAnglesPtr> cmdJntPos_Port;
	RTT::OutputPort<rci::JointImpedancePtr> cmdJntImp_Port;
	RTT::OutputPort<rci::JointTorquesPtr> cmdJntTrq_Port;

	RTT::InputPort<rci::JointAnglesPtr> currJntPos_Port;
	RTT::InputPort<rci::JointTorquesPtr> currJntTrq_Port;

	rci::JointAnglesPtr sendJntPos;
	rci::JointImpedancePtr sendJntImp;
	rci::JointTorquesPtr sendJntTrq;

	rci::JointAnglesPtr currJntPos;
	rci::JointTorquesPtr currJntTrq;

	RTT::FlowStatus currJntPos_flow;
	RTT::FlowStatus currJntTrq_flow;

	std::vector<RTTLWRJointPtr> registeredJointNodes;

private:
	void configureJointNodesDefault();
	RTT::base::PortInterface* findNestedPort(std::vector<std::string>& nestedPath, TaskContext* context);

};
