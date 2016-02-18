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
			RTT::TaskContext(name), portToSplitFrom("ToSplit_In"), toBeSplitted_flow(
					RTT::NoData), dims(dims) {
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
		this->ports()->addPort(portToSplitFrom).doc(
				"Input for Data to be splitted.");

		nemo::RealVector initvec(nemo::dim(7), 0.0);
		dataToBeSplitted = boost::shared_ptr<DERIVED>(new DERIVED(initvec));

		dataToBePublished.resize(dims);
		for (int i = 0; i < dataToBePublished.size(); i++) {
			boost::shared_ptr<RTT::OutputPort<boost::shared_ptr<DERIVED> > > tmpPort(
					new RTT::OutputPort<boost::shared_ptr<DERIVED> >(
							"Splitted_Out_"
									+ boost::lexical_cast<std::string>(i)));
			tmpPort->setDataSample(dataToBeSplitted);
			this->ports()->addPort(*tmpPort).doc("Output for splitted Data.");
			portsToSplitTo.push_back(tmpPort);
			dataToBePublished[i] = boost::shared_ptr<DERIVED>(new DERIVED(0.0));
		}
		return true;
	}

	void resetHook() {
		nemo::RealVector initvec(nemo::dim(7), 0.0);
		dataToBeSplitted = boost::shared_ptr<DERIVED>(new DERIVED(initvec));

		for (int i = 0; i < dataToBePublished.size(); i++) {
			dataToBePublished[i] = boost::shared_ptr<DERIVED>(new DERIVED(0.0));
		}
	}

	void updateHook() {
//		if (portToSplitFrom.connected()) {
//			toBeSplitted_flow = portToSplitFrom.read(dataToBeSplitted);
//			if (toBeSplitted_flow == RTT::NewData) {
//				if (dataToBeSplitted->getDimension() != dims) {
//					RTT::log(RTT::Error) << "[" << this->getName() << "] "
//							<< "Dimension mismatch. Expecting " << dims
//							<< " dimensions and not "
//							<< dataToBeSplitted->getDimension() << "dimensions!"
//							<< RTT::endlog();
//					return;
//				}
//				for (int i = 0; i < dataToBePublished.size(); i++) {
//					dataToBePublished[i]->setValue(0,
//							dataToBeSplitted->asDouble(i));
//				}
//				for (int i = 0; i < portsToSplitTo.size(); i++) {
//					if (portsToSplitTo[i]->connected())
//						portsToSplitTo[i]->write(dataToBePublished[i]);
//				}
//			}
//		}
	}

protected:
	boost::shared_ptr<DERIVED> dataToBeSplitted;
	RTT::InputPort<boost::shared_ptr<DERIVED> > portToSplitFrom;
	std::vector<boost::shared_ptr<RTT::OutputPort<boost::shared_ptr<DERIVED> > > > portsToSplitTo;
	RTT::FlowStatus toBeSplitted_flow;
	int dims;
};

