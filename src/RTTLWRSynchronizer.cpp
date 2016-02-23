#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include "RTTLWRSynchronizer.hpp"
#include <rtt/extras/PeriodicActivity.hpp>
#include <rtt/Activity.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace rci;
using namespace boost;

//#define 	ORO_SCHED_RT   0 /** Hard real-time */
//#define 	ORO_SCHED_OTHER   1 /** Soft real-time */

#define l(lvl) log(lvl) << "[" << this->getName() << "] "

RTTLWRSynchronizer::RTTLWRSynchronizer(string const& name) :
		TaskContext(name), cmdJntPos_Port("cmdJntPos"), cmdJntImp_Port(
				"cmdJntImp"), cmdJntTrq_Port("cmdJntTrq"), currJntPos_Port(
				"currJntPos"), currJntVel_Port("currJntVel"), currJntTrq_Port(
				"currJntTrq"), currJntPos_flow(RTT::NoData), currJntVel_flow(
				RTT::NoData), currJntTrq_flow(RTT::NoData) {

	this->ports()->addPort(cmdJntPos_Port).doc(
			"Sending joint position commands to robot.");
	this->ports()->addPort(cmdJntImp_Port).doc(
			"Sending joint impedance commands to robot.");
	this->ports()->addPort(cmdJntTrq_Port).doc(
			"Sending joint impedance commands to robot.");

	this->ports()->addPort(currJntPos_Port).doc(
			"Receiving current joint position.");
	this->ports()->addPort(currJntVel_Port).doc(
			"Receiving current joint velocity.");
	this->ports()->addPort(currJntTrq_Port).doc(
			"Receiving current joint torques.");

	this->addOperation("connectPortTo", &RTTLWRSynchronizer::connectPortTo,
			this, OwnThread).doc("Connect (also nested) peer ports").arg(
			"portOut", "relative path for OutputPort").arg("portIn",
			"relative path for InputPort").arg("connP",
			"ConnPolicy used for the connection");

	// init. local variable storage
	sendJntPos = rci::JointAngles::create(7, 0.0);

	sendJntImp = JointImpedancePtr(
			new JointImpedance(nemo::RealVector(nemo::dim(7), 50.0),
					nemo::RealVector(nemo::dim(7), 0.7)));

	sendJntTrq = rci::JointTorques::create(7, 0.0);

	currJntPos = rci::JointAngles::create(7, 0.0);
	currJntVel = rci::JointVelocities::create(7, 0.0);
	currJntTrq = rci::JointTorques::create(7, 0.0);

	// init. all output ports with sample data
	cmdJntPos_Port.setDataSample(rci::JointAngles::create(7, 0.0));
	cmdJntImp_Port.setDataSample(
			JointImpedancePtr(
					new JointImpedance(JointStiffness(50.0),
							JointDamping(0.7))));
	cmdJntTrq_Port.setDataSample(rci::JointTorques::create(7, 0.0));
}

void RTTLWRSynchronizer::configureJointNodesDefault() {
	for (int i = 0; i < 7; i++) {
		RTTLWRJointPtr tmpPtr(
				new RTTLWRJoint("joint_" + lexical_cast<std::string>(i)));
		this->addPeer((TaskContext*) tmpPtr.get(),
				"joint_" + lexical_cast<std::string>(i));
		registeredJointNodes.push_back(tmpPtr);
	}
}

RTT::base::PortInterface* RTTLWRSynchronizer::findNestedPort(
		std::vector<std::string>& nestedPath, TaskContext* context) {
	if (context == NULL) {
		log(Error) << "[RTTLWRSynchronizer] context is NULL -> return"
				<< endlog();
		return NULL;
	}

	if (nestedPath.size() == 1) {
		return context->getPort(nestedPath[0]);
	} else if (nestedPath.size() > 1) {
		std::string potentialPeer = nestedPath[0];
		nestedPath.erase(nestedPath.begin());
		return findNestedPort(nestedPath, context->getPeer(potentialPeer));
	} else {
		log(Error)
				<< "[RTTLWRSynchronizer] something wrong in findNestedPort(...)!"
				<< endlog();
		return NULL;
	}
}

bool RTTLWRSynchronizer::connectPortTo(std::string portOut, std::string portIn,
		ConnPolicy connP) {
	// 1) extract peers and ports from path
	vector<string> portOut_strs;
	boost::split(portOut_strs, portOut, boost::is_any_of("."));

	vector<string> portIn_strs;
	boost::split(portIn_strs, portIn, boost::is_any_of("."));

	if ((portOut_strs[0] == "this") || (portOut_strs[0] == "This")
			|| (portOut_strs[0] == this->getName())) {
		if (!this->hasPeer(portIn_strs[0])) {
			l(Warning)
					<< "If the connection can't be established, check if "
					<< portOut_strs[0] << " and " << portIn_strs[0]
					<< " are peers." << endlog();
		}
		portOut_strs.erase(portOut_strs.begin());
	} else if ((portIn_strs[0] == "this") || (portIn_strs[0] == "This")
			|| (portIn_strs[0] == this->getName())) {
		if (!this->hasPeer(portOut_strs[0])) {
			l(Warning)
					<< "If the connection can't be established, check if "
					<< portIn_strs[0] << " and " << portOut_strs[0]
					<< " are peers." << endlog();
		}
		portIn_strs.erase(portIn_strs.begin());
	}

	base::PortInterface* portOut_ptr = findNestedPort(portOut_strs, this);

	base::PortInterface* portIn_ptr = findNestedPort(portIn_strs, this);

	bool retVal = portOut_ptr->connectTo(portIn_ptr, connP);

	if (!retVal) {
		l(Error) << "Could not establish connection between " << portOut
				<< " and " << portIn << endlog();
	} else {
		l(Info) << "Connection between " << portOut << " and " << portIn
				<< " established!" << endlog();
	}

	return retVal;
}

void RTTLWRSynchronizer::configureJointNodes() {
	configureJointNodesDefault();
}

bool RTTLWRSynchronizer::configureHook() {
	configureJointNodes();

	for (std::vector<RTTLWRJointPtr>::iterator it =
			registeredJointNodes.begin(); it != registeredJointNodes.end();
			++it) {
		(*it)->setActivity(
				new Activity(ORO_SCHED_RT, os::HighestPriority, 0.1));
		(*it)->configure();

	}
	log(Info) << "RTTLWRSynchronizer configured !" << endlog();
	return true;
}

void RTTLWRSynchronizer::updateHook() {
	/** trigger all nodes manually... dont know if this is blocking...
	 *	for (std::vector<RTTLWRJointPtr>::iterator it =
	 *			registeredJointNodes.begin(); it != registeredJointNodes.end();
	 *			++it) {
	 *		(*it)->trigger();
	 *  }
	 */

	// collect input from nodes and distribute to robot
	for (int i = 0; i < registeredJointNodes.size(); i++) {
		// read joint position command
		sendJntPos->setValue(i,
				registeredJointNodes[i]->joint->getLastPositionCommand()->rad(
						0));

		// read joint impedance command
		sendJntImp->setValue(i * 2,
				registeredJointNodes[i]->joint->getLastImpedanceCommand()->asDouble(
						0));
		sendJntImp->setValue(i * 2 + 1,
				registeredJointNodes[i]->joint->getLastImpedanceCommand()->asDouble(
						1));
		// read joint torque command
		sendJntTrq->setValue(i,
				registeredJointNodes[i]->joint->getLastTorqueCommand()->Nm(0));

//		log(Error) << "RTTLWRSynchronizer torques read: " << sendJntTrq->Nm(i)
//				<< endlog();
	}
	/** TODO make everything relative to the CONTROL_MODE!
	 * If not take current from nodes!
	 */

//	l(Error) << "RTTLWRSynchronizer sendJntPos = " << sendJntPos->print()
//			<< endlog();
//	l(Error) << "RTTLWRSynchronizer sendJntTrq = " << sendJntTrq->print()
//			<< endlog();

//	log(Error) << "[RTTLWRSynchronizer] cmdJntPos_Port.write(sendJntPos) " << sendJntPos->print() << endlog();
//	log(Error) << "[RTTLWRSynchronizer] cmdJntTrq_Port.write(sendJntTrq) " << sendJntTrq->print() << endlog();

	if (cmdJntPos_Port.connected()) {
		cmdJntPos_Port.write(sendJntPos);
	}
	if (cmdJntImp_Port.connected()) {
		cmdJntImp_Port.write(sendJntImp);
	}
	if (cmdJntTrq_Port.connected()) {
		cmdJntTrq_Port.write(sendJntTrq);
	}

	// collect input from robot and distribute to nodes

	// collect from robot
	if (currJntPos_Port.connected()) {
		currJntPos_flow = currJntPos_Port.read(currJntPos);
	}
	if (currJntVel_Port.connected()) {
		currJntVel_flow = currJntVel_Port.read(currJntVel);
	}
	if (currJntTrq_Port.connected()) {
		currJntTrq_flow = currJntTrq_Port.read(currJntTrq);
	}
	//TODO add exstimated external torques

	// updated nodes with FB
	for (int i = 0; i < registeredJointNodes.size(); i++) {
		// write joint position FB to nodes
		if (currJntPos_flow == RTT::NewData) {
			registeredJointNodes[i]->joint->updateJointPosition(
					JointAngles::fromRad(currJntPos->rad(i)));
		}
		// write joint velocity FB to nodes
		if (currJntVel_flow == RTT::NewData) {
			registeredJointNodes[i]->joint->updateJointVelocity(
					JointVelocities::fromRad_s(currJntVel->rad_s(i)));
		}
		// write joint torques FB to nodes
		if (currJntTrq_flow == RTT::NewData) {
			registeredJointNodes[i]->joint->updateTorque(
					JointTorques::fromNm(currJntTrq->Nm(i)));
		}
		//TODO add estimated external torques
	}
}

bool RTTLWRSynchronizer::startHook() {
	for (std::vector<RTTLWRJointPtr>::iterator it =
			registeredJointNodes.begin(); it != registeredJointNodes.end();
			++it) {
		(*it)->start();
	}
	log(Info) << "RTTLWRSynchronizer started !" << endlog();
	return true;

}

void RTTLWRSynchronizer::stopHook() {
	for (std::vector<RTTLWRJointPtr>::iterator it =
			registeredJointNodes.begin(); it != registeredJointNodes.end();
			++it) {
		(*it)->stop();
	}
	log(Info) << "RTTLWRSynchronizer executes stopping !" << endlog();
}

void RTTLWRSynchronizer::cleanupHook() {
	for (std::vector<RTTLWRJointPtr>::iterator it =
			registeredJointNodes.begin(); it != registeredJointNodes.end();
			++it) {
		(*it)->cleanup();
	}
	log(Info) << "RTTLWRSynchronizer cleaning up !" << endlog();
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(RTTController)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */

ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(RTTLWRSynchronizer)
