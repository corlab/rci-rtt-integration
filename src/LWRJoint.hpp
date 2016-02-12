#pragma once

#include <iostream>
#include <fstream>

#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <rci/ResourceNode.h>
#include <rci/Controlled.h>
#include <rci/Configurable.h>
#include <rci/Sensing.h>

namespace rtt {
namespace lwr {

class LWRJoint;
typedef boost::shared_ptr<LWRJoint> LWRJointPtr;

/**
 * Node class, representing a joint node of the KUKA LWR IV.
 *
 * A lwr joint is sensing
 * * joint position
 * * a two-dim joint torque: the actual joint torque (index=0) and an estimated external joint torque (index=1)
 *
 * A lwr joint can be controlled via
 * * PositionControl - position of single joint
 * * ImpedanceControl - control stiffness and damping of joint
 *
 */
class LWRJoint: public rci::ResourceNode,
		public rci::Controlled,
		public rci::Sensing,
		public rci::PositionControlled,
		public rci::ImpedanceControlled,
		public rci::TorqueControlled,
		public rci::PositionSensing,
		public rci::TorqueSensing,
		public rci::VelocityControlled,
		public rci::VelocitySensing {

public:

	LWRJoint(std::string name = "LWR4 Joint Node");
	LWRJoint(const rci::JointAngles& limitMin, const rci::JointAngles& limitMax,
			std::string name = "LWR4 Joint Node");
	virtual ~LWRJoint();

	static LWRJointPtr create(std::string name = "LWR4 Joint Node");
	static LWRJointPtr create(const rci::JointAngles& limitMin,
			const rci::JointAngles& limitMax, std::string name =
					"LWR4 Joint Node");

	/**
	 * Returns, if controller is converged.
	 * @return True, if controller is converged after last command.
	 */
	bool isConverged() const;
	cca::DataTransferObjectPtr get() const;
	void update();

	void reset();

	/**
	 * Commanding a joint position.
	 *
	 * @param position
	 * 			Position command
	 * @return Return true, if successfull. (e.g. not exceeding joint limits)
	 * @todo Check for correct control mode?
	 */
	bool setJointPosition(rci::JointAnglesPtr position);

	/**
	 * Set new joint impedance.
	 * @param impedamce Desired joint impedance
	 * @todo Check for correct control mode?
	 */
	bool setJointImpedance(rci::JointImpedancePtr impedance);

	/**
	 * Sets Torque position
	 *
	 * Sets a position on a joint/actuator (joint space control).
	 */
	bool setJointTorque(rci::JointTorquesPtr position);

	/**
	 * Sets position
	 *
	 * Sets a position on a joint/actuator (joint space control).
	 */
	virtual bool setJointVelocity(rci::JointVelocitiesPtr velocity);

	/**
	 * Updates internal representation of joint position / angle
	 */
	virtual void updateJointPosition(rci::JointAnglesPtr angle);

	/**
	 * Updates internal representation of torques
	 */
	virtual void updateTorque(rci::JointTorquesPtr torque);

	/**
	 * Updates internal representation of joint position / angle
	 */
	virtual void updateJointVelocity(rci::JointVelocitiesPtr vel);

	/**
	 * Returns current joint position.
	 * @return Current joint position
	 */
	virtual rci::JointAnglesPtr getJointPosition() const;

	virtual rci::JointVelocitiesPtr getVelocity() const;

	virtual rci::JointTorquesPtr getTorque() const;

	/**
	 * Returns latest position command. Note, that this is the latest valid
	 * commanded position. If the position command exceeds limits and is
	 * therefore rejected, it ..
	 */
	virtual rci::JointAnglesPtr getLastPositionCommand() const;

	virtual rci::JointVelocitiesPtr getLastVelocityCommand() const;

	virtual rci::JointImpedancePtr getLastImpedanceCommand() const;

	virtual rci::JointTorquesPtr getLastTorqueCommand() const;

	/**
	 * Print
	 */
	std::string print() const;

private:
	bool _once;

	const rci::JointAngles limitMin, limitMax;
	bool jointLimitsDefined;

	mutable boost::recursive_mutex angleCommandMutex;
	mutable boost::recursive_mutex angleStatusMutex;
	mutable boost::recursive_mutex velocityCommandMutex;
	mutable boost::recursive_mutex velocityStatusMutex;
	mutable boost::recursive_mutex impedanceCommandMutex;
	mutable boost::recursive_mutex torqueCommandMutex;
	mutable boost::recursive_mutex torqueStatusMutex;
};

}
}
