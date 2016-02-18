#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include "RTTSplitter.hpp"
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

#define dims 7

RTTSplitter::RTTSplitter(string const& name) :
		TaskContext(name) {

	this->addOperation("connectPortTo", &RTTSplitter::connectPortTo,
			this, OwnThread).doc("Connect (also nested) peer ports").arg(
			"portOut", "relative path for OutputPort").arg("portIn",
			"relative path for InputPort").arg("connP",
			"ConnPolicy used for the connection");
}

void RTTSplitter::configureJointNodesDefault() {
	for (int i = 0; i < dims; i++) {
		shared_ptr<RCISplitter<rci::JointAngles> > jaSplitter(
				new RCISplitter<rci::JointAngles>("JointAngles_Splitter", dims));
		this->addPeer((TaskContext*) jaSplitter.get(), jaSplitter->getName());
		registeredSplitterNodes.push_back(jaSplitter);

		shared_ptr<RCISplitter<rci::JointVelocities> > jvSplitter(
				new RCISplitter<rci::JointVelocities>(
						"JointVelocities_Splitter", dims));
		this->addPeer((TaskContext*) jvSplitter.get(), jvSplitter->getName());
		registeredSplitterNodes.push_back(jvSplitter);

		shared_ptr<RCISplitter<rci::JointTorques> > jtSplitter(
				new RCISplitter<rci::JointTorques>("JointTorques_Splitter", dims));
		this->addPeer((TaskContext*) jtSplitter.get(), jtSplitter->getName());
		registeredSplitterNodes.push_back(jtSplitter);
	}
}

RTT::base::PortInterface* RTTSplitter::findNestedPort(
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

void RTTSplitter::configureJointNodes() {
	configureJointNodesDefault();
}

bool RTTSplitter::connectPortTo(std::string portOut, std::string portIn,
		ConnPolicy connP) {
	// 1) extract peers and ports from path
	vector<string> portOut_strs;
	boost::split(portOut_strs, portOut, boost::is_any_of("."));

	vector<string> portIn_strs;
	boost::split(portIn_strs, portIn, boost::is_any_of("."));

	if ((portOut_strs[0] == "this") || (portOut_strs[0] == "This")
			|| (portOut_strs[0] == this->getName())) {
		if (!this->hasPeer(portIn_strs[0])) {
			log(Warning)
					<< "[RTTLWRSynchronizer] If the connection can't be established, check if "
					<< portOut_strs[0] << " and " << portIn_strs[0]
					<< " are peers." << endlog();
		}
		portOut_strs.erase(portOut_strs.begin());
	} else if ((portIn_strs[0] == "this") || (portIn_strs[0] == "This")
			|| (portIn_strs[0] == this->getName())) {
		if (!this->hasPeer(portOut_strs[0])) {
			log(Warning)
					<< "[RTTLWRSynchronizer] If the connection can't be established, check if "
					<< portIn_strs[0] << " and " << portOut_strs[0]
					<< " are peers." << endlog();
		}
		portIn_strs.erase(portIn_strs.begin());
	}

	base::PortInterface* portOut_ptr = findNestedPort(portOut_strs, this);

	base::PortInterface* portIn_ptr = findNestedPort(portIn_strs, this);

	bool retVal = portOut_ptr->connectTo(portIn_ptr, connP);

	if (!retVal) {
		log(Error)
				<< "[RTTLWRSynchronizer] Could not establish connection between "
				<< portOut << " and " << portIn << endlog();
	} else {
		log(Info) << "[RTTLWRSynchronizer] Connection between " << portOut
				<< " and " << portIn << " established!" << endlog();
	}

	return retVal;
}

bool RTTSplitter::configureHook() {
	configureJointNodes();

	for (std::vector<boost::shared_ptr<RTT::TaskContext> >::iterator it =
			registeredSplitterNodes.begin();
			it != registeredSplitterNodes.end(); ++it) {
		(*it)->setActivity(
				new Activity(ORO_SCHED_RT, os::HighestPriority, 0.1));
		(*it)->configure();

	}
	l(Info)<< "configured !" << endlog();
	return true;
}

void RTTSplitter::updateHook() {
}

bool RTTSplitter::startHook() {
	for (std::vector<boost::shared_ptr<RTT::TaskContext> >::iterator it =
			registeredSplitterNodes.begin();
			it != registeredSplitterNodes.end(); ++it) {
		(*it)->start();
	}
	l(Info)<< "started !" << endlog();
	return true;
}

void RTTSplitter::stopHook() {
	for (std::vector<boost::shared_ptr<RTT::TaskContext> >::iterator it =
			registeredSplitterNodes.begin(); it != registeredSplitterNodes.end();
				++it) {
			(*it)->stop();
		}
	l(Info)<< "executes stopping !" << endlog();
}

void RTTSplitter::cleanupHook() {
	for (std::vector<boost::shared_ptr<RTT::TaskContext> >::iterator it =
			registeredSplitterNodes.begin(); it != registeredSplitterNodes.end();
				++it) {
			(*it)->cleanup();
		}
	l(Info)<< "cleaning up !" << endlog();
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

ORO_LIST_COMPONENT_TYPE(RTTSplitter)
