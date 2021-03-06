#include "RTTLWRCollector.hpp"

#include "../RTTCollector/RTTLWRCollector.hpp"

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/extras/PeriodicActivity.hpp>
#include <rtt/Activity.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace rci;
using namespace boost;

#define l(lvl) log(lvl) << "[" << this->getName() << "] "

#define dims 7

RTTLWRCollector::RTTLWRCollector(string const& name) :
		NestedTaskContext(name) {
}

void NestedTaskContext::configureJointNodes() {
	for (int i = 0; i < dims; i++) {
		shared_ptr<RCICollector<rci::JointAngles> > jaCollector(
				new RCICollector<rci::JointAngles>("JointAngles_Collector",
				dims));
		this->addPeer((TaskContext*) jaCollector.get(), jaCollector->getName());
		registeredNodes.push_back(jaCollector);

		shared_ptr<RCICollector<rci::JointVelocities> > jvCollector(
				new RCICollector<rci::JointVelocities>(
						"JointVelocities_Collector", dims));
		this->addPeer((TaskContext*) jvCollector.get(), jvCollector->getName());
		registeredNodes.push_back(jvCollector);

		shared_ptr<RCICollector<rci::JointTorques> > jtCollector(
				new RCICollector<rci::JointTorques>("JointTorques_Collector",
				dims));
		this->addPeer((TaskContext*) jtCollector.get(), jtCollector->getName());
		registeredNodes.push_back(jtCollector);
	}
}

bool RTTLWRCollector::configureHook() {
	configureJointNodes();

	for (std::vector<boost::shared_ptr<RTT::TaskContext> >::iterator it =
			registeredNodes.begin(); it != registeredNodes.end(); ++it) {
		(*it)->setActivity(
				new Activity(ORO_SCHED_RT, os::HighestPriority, 0.1));
		(*it)->configure();

	}
	l(Info) << "configured !" << endlog();
	return true;
}

void RTTLWRCollector::updateHook() {
}

bool RTTLWRCollector::startHook() {
	this->startH();
	l(Info) << "started !" << endlog();
	return true;
}

void RTTLWRCollector::stopHook() {
	this->stopH();
	l(Info) << "executes stopping !" << endlog();
}

void RTTLWRCollector::cleanupHook() {
	this->cleanupH();
	l(Info) << "cleaning up !" << endlog();
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

ORO_LIST_COMPONENT_TYPE(RTTLWRCollector)
