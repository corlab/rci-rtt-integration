#include "LWRJoint.hpp"

using namespace std;
using namespace boost;
using namespace nemo;
using namespace rci;

namespace rtt {
namespace lwr {

LWRJoint::LWRJoint(std::string name) :
		rci::ResourceNode(name), rci::Controlled(), rci::Sensing(), rci::PositionControlled(), ImpedanceControlled(), rci::PositionSensing(), rci::TorqueSensing(
				2), _once(true), limitMin(), limitMax(), jointLimitsDefined(
				false) {
	this->_latestJointPosition = JointAngles::fromRad(0.0);
	this->torques = JointTorques::fromNm(0.0);
	this->_lastCommandedImpedance = JointImpedancePtr(
			new JointImpedance(JointStiffness(50.0), JointDamping(0.7)));
}

LWRJoint::LWRJoint(const JointAngles& limMin, const JointAngles& limMax,
		std::string name) :
		rci::ResourceNode(name), rci::Controlled(), rci::Sensing(), rci::PositionControlled(), ImpedanceControlled(), rci::PositionSensing(), rci::TorqueSensing(
				2), _once(true), limitMin(limMin), limitMax(limMax), jointLimitsDefined(
				true) {
	this->_latestJointPosition = JointAngles::fromRad(0.0);
	this->torques = JointTorques::fromNm(RealVector(0.0, 0.0));
	this->_lastCommandedImpedance = JointImpedancePtr(
			new JointImpedance(JointStiffness(500.0), JointDamping(0.7)));
}

LWRJointPtr LWRJoint::create(std::string name) {
	return LWRJointPtr(new LWRJoint(name));
}

LWRJointPtr LWRJoint::create(const JointAngles& limitMin,
		const JointAngles& limitMax, std::string name) {
	return LWRJointPtr(new LWRJoint(limitMin, limitMax, name));
}

bool LWRJoint::isConverged() const {
	return false;
}

void LWRJoint::reset() {
	// Empty latest commands
	this->_lastCommandedPosition = JointAnglesPtr();
	this->_lastCommandedImpedance = JointImpedancePtr(
			new JointImpedance(JointStiffness(50.0), JointDamping(0.7)));
	this->_lastCommandedTorque = JointTorquesPtr();

// Reset latest sensor reads as in constructor
	this->_latestJointPosition = JointAngles::fromRad(0.0);
	this->torques = JointTorques::fromNm(RealVector(0.0, 0.0));
}

bool LWRJoint::setJointPosition(JointAnglesPtr position) {
	recursive_mutex::scoped_lock scoped_lock(angleCommandMutex);
	if (position == NULL) {
		throw std::runtime_error(
				"[LWRJoint::setJointPosition] Got null pointer!");
	}

	// check joint limits
	if (jointLimitsDefined) {
		for (unsigned int i = 0; i < position->getDimension(); i++) {
			if ((position->asDouble(i) < limitMin.asDouble(i))
					|| (position->asDouble(i) > limitMax.asDouble(i))) {
				return false;
			}
		}
	}

	if (this->_lastCommandedPosition == NULL) {
		this->_lastCommandedPosition = JointAngles::copy(*position);
	} else {
		this->_lastCommandedPosition->setFromRad(0, position->rad(0));
	}

	return true;
}

bool LWRJoint::setJointImpedance(JointImpedancePtr impedance) {
	recursive_mutex::scoped_lock scoped_lock(impedanceCommandMutex);
	if (impedance == NULL) {
		throw std::runtime_error(
				"[LWRJoint::setJointImpedance] Got null pointer!");
	}

	// Successfull
	if (this->_lastCommandedImpedance == NULL) {
		this->_lastCommandedImpedance = JointImpedance::copy(*impedance);
	} else {
		this->_lastCommandedImpedance->setValue(0, impedance->asDouble(0));
		this->_lastCommandedImpedance->setValue(1, impedance->asDouble(1));
	}

	return true;
}

bool LWRJoint::setJointTorque(rci::JointTorquesPtr position) {
	recursive_mutex::scoped_lock scoped_lock(torqueCommandMutex);
	if (position == NULL) {
		throw std::runtime_error(
				"[LWRJoint::setJointTorque] Got null pointer!");
	}

	// check joint torque limits TODO

	if (this->_lastCommandedTorque == NULL) {
		this->_lastCommandedTorque = JointTorques::copy(*position);
	} else {
		this->_lastCommandedTorque->setFromNm(0, position->Nm(0));
	}

	return true;
}

void LWRJoint::updateJointPosition(JointAnglesPtr angle) {
	recursive_mutex::scoped_lock scoped_lock(angleStatusMutex);
	if (this->_latestJointPosition == NULL) {
		this->_latestJointPosition = JointAngles::copy(*angle);
	} else {
		this->_latestJointPosition->setFromRad(0, angle->rad(0));
	}
}

void LWRJoint::updateTorque(rci::JointTorquesPtr torque) {
	recursive_mutex::scoped_lock scoped_lock(torqueStatusMutex);
	if (this->torques == NULL) {
		this->torques = JointTorques::copy(*torque);
	} else {
		this->torques->setFromNm(0, torque->Nm(0));
	}
}

// TODO why this??
cca::DataTransferObjectPtr LWRJoint::get() const {
	return this->getJointPosition();
}

void LWRJoint::update() {
	std::cerr << "Update is not necessary on this sensor." << std::endl;
}

JointAnglesPtr LWRJoint::getJointPosition() const {
	recursive_mutex::scoped_lock scoped_lock(angleStatusMutex);
	return this->_latestJointPosition;
}

JointTorquesPtr LWRJoint::getTorque() const {
	recursive_mutex::scoped_lock scoped_lock(torqueStatusMutex);
	return this->torques;
}

LWRJoint::~LWRJoint() {
}

std::string LWRJoint::print() const {
	ostringstream outstream(ostringstream::out);
	outstream.precision(3); // Precision when printing double values
	outstream << "<LWRJoint \"" << this->nodename << "\"";
	outstream << ">";
	return outstream.str();
}

JointAnglesPtr LWRJoint::getLastPositionCommand() const {
	recursive_mutex::scoped_lock scoped_lock(angleCommandMutex);

	if (this->_lastCommandedPosition) {
		return this->_lastCommandedPosition;
	}

	// If we don`t have a command yet, we return the latest sensor value
	if (this->_latestJointPosition) {
		// TODO: Log
		return this->_latestJointPosition;
	}

	throw runtime_error("No position command received yet.");
}

JointTorquesPtr LWRJoint::getLastTorqueCommand() const {
	recursive_mutex::scoped_lock scoped_lock(torqueCommandMutex);

	if (this->_lastCommandedTorque) {
		return this->_lastCommandedTorque;
	}

	// If we don`t have a command yet, we return the latest sensor value
	if (this->torques) {
		return this->torques;
	}

	throw runtime_error("No torque command received yet.");
}

JointImpedancePtr LWRJoint::getLastImpedanceCommand() const {
	recursive_mutex::scoped_lock scoped_lock(impedanceCommandMutex);

	// automatic adaption of joint stiffness to limits
	if (jointLimitsDefined) {
		double pos = _latestJointPosition->rad(0);
		double diff = min(pos - limitMin.rad(0), limitMax.rad(0) - pos);
		double cmdStiffness = _lastCommandedImpedance->getStiffness().asDouble(
				0);

		double softlim = 0.1;
		double maxStiff = 100.0;

		if (maxStiff > cmdStiffness) {
			if (diff <= 0) {
				return JointImpedancePtr(
						new JointImpedance(JointStiffness(maxStiff),
								JointDamping(1.0)));
			} else if (diff <= softlim) {
				double param = diff / softlim;
				param *= param;
				double stiffness = param * cmdStiffness
						+ (1 - param) * maxStiff;
				return JointImpedancePtr(
						new JointImpedance(JointStiffness(stiffness),
								_lastCommandedImpedance->getDamping()));
			}
		}
	}

	return _lastCommandedImpedance;
}

}
}
