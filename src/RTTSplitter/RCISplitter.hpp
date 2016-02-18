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

#define l(lvl) RTT::log(lvl) << "[" << this->getName() << "] "

template<typename DERIVED>
class RCISplitter: public RTT::TaskContext {

public:
	RCISplitter(std::string name, int dims) : RTT::TaskContext(name),
			portToSplitFrom("ToSplit_In"), toBeSplitted_flow(RTT::NoData), dims(
					dims) {
		BOOST_STATIC_ASSERT(
				(boost::is_base_of<rci::JointValues, DERIVED>::value));

	}

	~RCISplitter() {
	}

//	void setAdjacencyMatrix(const nemo::Matrix<int>& adj) {
//		adjacencyMatrix = adj;
//	}
//
//	void generatePortsAccordingToAdjacencyMatrix(const nemo::Matrix<int>& adj,
//			boost::shared_ptr<DERIVED> sample) {
//		// set right amount of input ports
//		portsToSplitFrom.resize(adj.rows());
//		// set same amount of storage variables
//		for (int i = 0; i < portsToSplitTo.size(); i++)
//			dataToBeSplitted.push_back(sample);
//		tmpDataToModify = sample;
//
//		// set right amount of output ports
//		portsToSplitTo.resize(adj.cols());
//		// set sample data of output ports
//		for (int i = 0; i < portsToSplitTo.size(); i++)
//			portsToSplitTo[i].setDataSample(sample);
//	}

	bool startHook() {
		l(RTT::Info)<< " started!" << RTT::endlog();
		return true;

	}

	void stopHook() {
		l(RTT::Info) << " stopping!" << RTT::endlog();
	}

	void cleanupHook() {
		l(RTT::Info) << " cleaning up!" << RTT::endlog();
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
		if (portToSplitFrom.connected()) {
			toBeSplitted_flow = portToSplitFrom.read(dataToBeSplitted);
			if (toBeSplitted_flow == RTT::NewData) {
				if (dataToBeSplitted->getDimension() != dims) {
					RTT::log(RTT::Error) << "[" << this->getName() << "] "
					<< "Dimension mismatch. Expecting " << dims
					<< " dimensions and not "
					<< dataToBeSplitted->getDimension() << "dimensions!"
					<< RTT::endlog();
					return;
				}
				for (int i = 0; i < dataToBePublished.size(); i++) {
					dataToBePublished[i]->setValue(0,
							dataToBeSplitted->asDouble(i));
				}
				for (int i = 0; i < portsToSplitTo.size(); i++) {
					if (portsToSplitTo[i]->connected())
					portsToSplitTo[i]->write(dataToBePublished[i]);
				}
			}
		}
	}

//	void readStep() {
//		// iterate through all input ports
//		for (int in = 0; in < portsToSplitFrom.size(); in++) {
//			portsToSplitFrom[in].read(dataToBeSplitted[in]);
////			dataToBeSplitted[in]->asDoubleVector();
////			dataToBeSplitted[in]->getDimension();
////			dataToBeSplitted[in]->asDouble();
//
//// check distribution for every output port
//			for (int ou = 0; ou < portsToSplitTo.size(); ou++) {
//				int adjVal = adjacencyMatrix(in, ou);
//				if ((adjVal > -1)
//						&& (dataToBeSplitted[in]->getDimension() > adjVal)) {
//
////					tmpDataToModify = boost::make_shared<DERIVED>(
////							*dataToBeSplitted[in]); // not sure if this is valid in RT-Loop TODO
////					tmpDataToModify->setValue(
////							dataToBeSplitted[in]->asDouble(adjVal)); // setValues(...); TODO
////					portsToSplitTo[ou].write(tmpDataToModify);
//					portsToSplitTo[ou].write(DERIVED::create(1, adjVal));
//				}
//			}
//
////			boost::shared_ptr<DERIVED> dd = boost::static_pointer_cast<
////					boost::shared_ptr<DERIVED> >(dataToBeSplitted[in]);
//
////			boost::shared_ptr<DERIVED> tmp = portsToSplitFrom[in]
////			dataToBeSplitted[in] = dynamic_cast<rci::JointValuesPtr>();
//		}
//	}

protected:
	boost::shared_ptr<DERIVED> dataToBeSplitted;
	std::vector<boost::shared_ptr<DERIVED> > dataToBePublished;
//	boost::shared_ptr<DERIVED> tmpDataToModify;
//	std::vector<RTT::InputPort<boost::shared_ptr<DERIVED> > > portsToSplitFrom;
	RTT::InputPort<boost::shared_ptr<DERIVED> > portToSplitFrom;
	std::vector<boost::shared_ptr<RTT::OutputPort<boost::shared_ptr<DERIVED> > > > portsToSplitTo;
//	nemo::Matrix<int> adjacencyMatrix;
	RTT::FlowStatus toBeSplitted_flow;
	int dims;

//	ORO_LIST_COMPONENT_TYPE(RCISplitter)
};

