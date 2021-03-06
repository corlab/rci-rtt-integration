#include "RTTLWRJoint.hpp"

#include <boost/pointer_cast.hpp>

#include <rci/dto/JointAngles.h>
#include <rci/dto/JointVelocities.h>
#include <rci/dto/JointImpedance.h>

#include <rtt/Component.hpp>

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace rci;
using namespace boost;

#define l(lvl) log(lvl) << "[" << "RTTLWRJoint." << this->getName() << "] "

RTTLWRJoint::RTTLWRJoint(const std::string &name) :
		TaskContext(name), joint(), INPUT_JntCmd("INPUT_JntCmd"), INPUT_JntImp(
				"INPUT_JntImp"), INPUT_JntTrq("INPUT_JntTrq"),

		OUTPUT_JntPos("OUTPUT_JntPos"), OUTPUT_JntVel("OUTPUT_JntVel"), OUTPUT_JntTorq(
				"OUTPUT_JntTorq"), OUTPUT_EstExtJntTorq("OUTPUT_EstExtJntTorq") {

	// perhaps move into configured section
	joint = rtt::lwr::LWRJoint::create("LWR4 Joint Node");

	this->ports()->addPort(INPUT_JntCmd).doc(
			"Reading joint position commands.");
	this->ports()->addPort(INPUT_JntImp).doc(
			"Reading joint impedance commands.");
	this->ports()->addPort(INPUT_JntTrq).doc("Reading joint torque commands.");

	this->ports()->addPort(OUTPUT_JntPos).doc(
			"Sending joint position feedback.");
	this->ports()->addPort(OUTPUT_JntVel).doc(
			"Sending joint velocity feedback.");
	this->ports()->addPort(OUTPUT_JntTorq).doc(
			"Sending joint torque feedback.");
	this->ports()->addPort(OUTPUT_EstExtJntTorq).doc(
			"Sending estimated external joint torque feedback.");

	tmpJntAngles = rci::JointAngles::fromDeg(0.0);
	tmpJntVelocities = rci::JointVelocities::fromDeg_s(0.0);
	tmpJntImpedance = rci::JointImpedance::create(nemo::RealVector(0.0));
	tmpJntTorques = rci::JointTorques::fromNm(0.0);

	OUTPUT_JntPos.setDataSample(tmpJntAngles);
	OUTPUT_JntVel.setDataSample(tmpJntVelocities);
	OUTPUT_JntTorq.setDataSample(tmpJntTorques);
	OUTPUT_EstExtJntTorq.setDataSample(tmpJntTorques);

	l(Info) << " created!" << endlog();
}

RTTLWRJoint::~RTTLWRJoint() {
}

bool RTTLWRJoint::configureHook() {
	this->joint->reset();
	l(Info) << " configured!" << endlog();
	return true;
}

void RTTLWRJoint::updateHook() {
	// Persist the received commands CMD in
	if ((INPUT_JntCmd.connected())
			&& (INPUT_JntCmd.read(tmpJntAngles) == RTT::NewData)) {
		this->joint->setJointPosition(tmpJntAngles);
	}

	if ((INPUT_JntImp.connected())
			&& (INPUT_JntImp.read(tmpJntImpedance) == RTT::NewData)) {
		this->joint->setJointImpedance(tmpJntImpedance);
	}

	if ((INPUT_JntTrq.connected())
			&& (INPUT_JntTrq.read(tmpJntTorques) == RTT::NewData)) {
		this->joint->setJointTorque(tmpJntTorques);
	}

	// Send out latest sensor read FB out
	if (OUTPUT_JntPos.connected()) {
		OUTPUT_JntPos.write(this->joint->getJointPosition());
	}
	if (OUTPUT_JntVel.connected()) {
		OUTPUT_JntVel.write(this->joint->getVelocity());
	}
	// TODO check if this creation is an issue for RT...!
	if (OUTPUT_JntTorq.connected()) {
		OUTPUT_JntTorq.write(
				rci::JointTorques::fromNm(this->joint->getTorque()->Nm(0))); //actual measured torques
	}
	if (OUTPUT_EstExtJntTorq.connected()) {
		OUTPUT_EstExtJntTorq.write(
				rci::JointTorques::fromNm(this->joint->getTorque()->Nm(1))); //estimated external torques
	}
}

bool RTTLWRJoint::startHook() {
	l(Info) << " started!" << endlog();
	return true;

}

void RTTLWRJoint::stopHook() {
	l(Info) << " stopping!" << endlog();
}

void RTTLWRJoint::cleanupHook() {
	this->joint->reset();
	l(Info) << " cleaning up!" << endlog();
}

//ORO_CREATE_COMPONENT_LIBRARY()
//ORO_CREATE_COMPONENT(RTTLWRJoint)
ORO_LIST_COMPONENT_TYPE(RTTLWRJoint)
