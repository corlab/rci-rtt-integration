#pragma once

#include <iostream>

#include <boost/type_traits.hpp>
#include <boost/static_assert.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>

#include <rci/dto/JointValues.h>

#include <nemo/Mapping.h>
#include <nemo/Vector.h>
#include <nemo/Matrix.h>

template<typename DERIVED>
class RCICollector: public RTT::TaskContext {

public:
	RCICollector(std::string name, int dims) :
			RTT::TaskContext(name), portToPublishTo("Collected_Out"), dims(dims) {
		BOOST_STATIC_ASSERT(
				(boost::is_base_of<rci::JointValues, DERIVED>::value));

	}

	~RCICollector() {
	}

	bool startHook() {
		RTT::log(RTT::Info) << "[" << this->getName() << "] started!"
				<< RTT::endlog();
		return true;

	}

	void stopHook() {
		RTT::log(RTT::Info) << "[" << this->getName() << "] stopping!"
				<< RTT::endlog();
	}

	void cleanupHook() {
		RTT::log(RTT::Info) << "[" << this->getName() << "] cleaning up!"
				<< RTT::endlog();
	}

	bool configureHook() {
		nemo::RealVector initvec(nemo::dim(7), 0.0);
		collectedDataToBePublished = boost::shared_ptr<DERIVED>(
				new DERIVED(initvec));
		collectedSingleData = boost::shared_ptr<DERIVED>(new DERIVED(0.0));

		this->ports()->addPort(portToPublishTo).doc(
				"Output for collected Data.");
		portToPublishTo.setDataSample(collectedDataToBePublished);

		for (int i = 0; i < collectedDataToBePublished->getDimension(); i++) {
			boost::shared_ptr<RTT::InputPort<boost::shared_ptr<DERIVED> > > tmpPort(
					new RTT::InputPort<boost::shared_ptr<DERIVED> >(
							"Collect_In_"
									+ boost::lexical_cast<std::string>(i)));
			this->ports()->addPort(*tmpPort).doc(
					"Input for Data to be collected.");
			portsToCollectFrom.push_back(tmpPort);
			toBeCollected_flows.push_back(RTT::NoData);
		}
		return true;
	}

	void resetHook() {
		nemo::RealVector initvec(nemo::dim(7), 0.0);
		collectedDataToBePublished = boost::shared_ptr<DERIVED>(
				new DERIVED(initvec));
		collectedSingleData = boost::shared_ptr<DERIVED>(new DERIVED(0.0));
		for (int i = 0; i < toBeCollected_flows.size(); i++) {
			toBeCollected_flows[i] = RTT::NoData;
		}
	}

	void updateHook() {
		for (int i = 0; i < portsToCollectFrom.size(); i++) {
			if (portsToCollectFrom[i]->connected()) {
				toBeCollected_flows[i] = portsToCollectFrom[i]->read(
						collectedSingleData);
				if (toBeCollected_flows[i] == RTT::NoData) {
//					RTT::log(RTT::Warning) << "[" << this->getName()
//							<< "] Port " << portsToCollectFrom[i]->getName()
//							<< " receives no data but is connected! Skipping publishing process!"
//							<< RTT::endlog();
					return;
				}
				if (toBeCollected_flows[i] == RTT::NewData) {
					if (collectedSingleData->getDimension() != 1) {
						RTT::log(RTT::Error) << "[" << this->getName() << "] "
								<< "Dimension mismatch. Expecting " << 1
								<< " dimension and not "
								<< collectedSingleData->getDimension()
								<< "dimensions!" << RTT::endlog();
						return;
					}
				}

				// this means it publishes even if it only has old data for a long time... TODO (not sure if this is good or bad)
				collectedDataToBePublished->setValue(i,
						collectedSingleData->asDouble(0));

//			} else {
//				RTT::log(RTT::Warning) << "[" << this->getName() << "] Port "
//						<< portsToCollectFrom[i]->getName()
//						<< " is not connected! Skipping publishing process!"
//						<< RTT::endlog();
//				return;
			}
		}

		if (portToPublishTo.connected()) {
			portToPublishTo.write(collectedDataToBePublished);
		} //else {
//			RTT::log(RTT::Warning) << "[" << this->getName() << "] Port "
//					<< portToPublishTo.getName()
//					<< " is not connected! Skipping publishing process (out)!"
//					<< RTT::endlog();
//		}
	}

protected:
	boost::shared_ptr<DERIVED> collectedDataToBePublished;
	boost::shared_ptr<DERIVED> collectedSingleData;
	RTT::OutputPort<boost::shared_ptr<DERIVED> > portToPublishTo;
	std::vector<boost::shared_ptr<RTT::InputPort<boost::shared_ptr<DERIVED> > > > portsToCollectFrom;
	std::vector<RTT::FlowStatus> toBeCollected_flows;
	int dims;
};

