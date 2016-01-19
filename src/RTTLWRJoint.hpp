#pragma once

#include "LWRJoint.hpp"

#include <iostream>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <nemo/Mapping.h>
#include <nemo/Vector.h>

class RTTLWRJoint;
typedef boost::shared_ptr<RTTLWRJoint> RTTLWRJointPtr;

class RTTLWRJoint: public RTT::TaskContext {
public:

	RTTLWRJoint(const std::string &name);
	~RTTLWRJoint();

	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	rtt::lwr::LWRJointPtr joint;

protected:
	RTT::InputPort<rci::JointAnglesPtr> INPUT_JntCmd;
	RTT::InputPort<rci::JointImpedancePtr> INPUT_JntImp;
	RTT::InputPort<rci::JointTorquesPtr> INPUT_JntTrq;

	RTT::OutputPort<rci::JointAnglesPtr> OUTPUT_JntPos;
	RTT::OutputPort<rci::JointTorquesPtr> OUTPUT_JntTorq;
	RTT::OutputPort<rci::JointTorquesPtr> OUTPUT_EstExtJntTorq;

	rci::JointAnglesPtr tmpJntAngles;
	rci::JointImpedancePtr tmpJntImpedance;
	rci::JointTorquesPtr tmpJntTorques;
};
