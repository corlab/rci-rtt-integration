#pragma once

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

class NestedTaskContext: public RTT::TaskContext {
public:
	NestedTaskContext(std::string const& name) :
			RTT::TaskContext(name) {

		this->addOperation("connectPortTo", &NestedTaskContext::connectPortTo,
				this, RTT::OwnThread).doc("Connect (also nested) peer ports").arg(
				"portOut", "relative path for OutputPort").arg("portIn",
				"relative path for InputPort").arg("connP",
				"ConnPolicy used for the connection");

	}

	~NestedTaskContext() {

	}

	bool connectPortTo(std::string portOut, std::string portIn,
			RTT::ConnPolicy connP) {
		// 1) extract peers and ports from path
		std::vector<std::string> portOut_strs;
		boost::split(portOut_strs, portOut, boost::is_any_of("."));

		std::vector<std::string> portIn_strs;
		boost::split(portIn_strs, portIn, boost::is_any_of("."));

		if ((portOut_strs[0] == "this") || (portOut_strs[0] == "This")
				|| (portOut_strs[0] == this->getName())) {
			if (!this->hasPeer(portIn_strs[0])) {
				RTT::log(RTT::Warning)
						<< "[NestedTaskContextIF] If the connection can't be established, check if "
						<< portOut_strs[0] << " and " << portIn_strs[0]
						<< " are peers." << RTT::endlog();
			}
			portOut_strs.erase(portOut_strs.begin());
		} else if ((portIn_strs[0] == "this") || (portIn_strs[0] == "This")
				|| (portIn_strs[0] == this->getName())) {
			if (!this->hasPeer(portOut_strs[0])) {
				RTT::log(RTT::Warning)
						<< "[NestedTaskContextIF] If the connection can't be established, check if "
						<< portIn_strs[0] << " and " << portOut_strs[0]
						<< " are peers." << RTT::endlog();
			}
			portIn_strs.erase(portIn_strs.begin());
		}

		RTT::base::PortInterface* portOut_ptr = findNestedPort(portOut_strs,
				this);

		RTT::base::PortInterface* portIn_ptr = findNestedPort(portIn_strs,
				this);

		bool retVal = portOut_ptr->connectTo(portIn_ptr, connP);

		if (!retVal) {
			RTT::log(RTT::Error)
					<< "[NestedTaskContextIF] Could not establish connection between "
					<< portOut << " and " << portIn << RTT::endlog();
		} else {
			RTT::log(RTT::Info) << "[NestedTaskContextIF] Connection between "
					<< portOut << " and " << portIn << " established!"
					<< RTT::endlog();
		}

		return retVal;
	}

	/**
	 * Implement to set the nodes
	 */
	virtual void configureJointNodes();

	void startH() {
		for (std::vector<boost::shared_ptr<RTT::TaskContext> >::iterator it =
				registeredNodes.begin(); it != registeredNodes.end(); ++it) {
			(*it)->start();
		}
	}

	void stopH() {
		for (std::vector<boost::shared_ptr<RTT::TaskContext> >::iterator it =
				registeredNodes.begin(); it != registeredNodes.end(); ++it) {
			(*it)->stop();
		}
	}

	void cleanupH() {
		for (std::vector<boost::shared_ptr<RTT::TaskContext> >::iterator it =
				registeredNodes.begin(); it != registeredNodes.end(); ++it) {
			(*it)->cleanup();
		}
	}

protected:
	RTT::base::PortInterface* findNestedPort(
			std::vector<std::string>& nestedPath, TaskContext* context) {
		if (context == NULL) {
			RTT::log(RTT::Error)
					<< "[NestedTaskContextIF] context is NULL -> return"
					<< RTT::endlog();
			return NULL;
		}

		if (nestedPath.size() == 1) {
			return context->getPort(nestedPath[0]);
		} else if (nestedPath.size() > 1) {
			std::string potentialPeer = nestedPath[0];
			nestedPath.erase(nestedPath.begin());
			return findNestedPort(nestedPath, context->getPeer(potentialPeer));
		} else {
			RTT::log(RTT::Error)
					<< "[NestedTaskContextIF] something wrong in findNestedPort(...)!"
					<< RTT::endlog();
			return NULL;
		}
	}

	std::vector<boost::shared_ptr<RTT::TaskContext> > registeredNodes;
};
