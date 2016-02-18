
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#



include_directories(include
  ${RCI_INCLUDE_DIRS}
  ${RSC_INCLUDE_DIRS}
#  ${Eigen_INCLUDE_DIR}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${NemoMath_INCLUDE_DIRS}
  )





# Creates a component library libmyrobot-<target>.so
# and installs in the directory lib/orocos/myrobot/
#


orocos_component(RCIRTTIntegration RTTLWRSynchronizer.hpp RTTLWRSynchronizer.cpp RTTLWRJoint.hpp RTTLWRJoint.cpp LWRJoint.hpp LWRJoint.cpp RTTSplitter/RTTLWRSplitter.cpp RTTSplitter/RTTLWRSplitter.hpp RTTSplitter/RCISplitter.hpp NestedTaskContext.hpp RTTCollector/RCICollector.hpp RTTCollector/RTTLWRCollector.hpp RTTCollector/RTTLWRCollector.cpp)

#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/myrobot/ directory
#
orocos_install_headers(RTTLWRSynchronizer.hpp)
#
# You may add multiple orocos_install_headers statements.

target_link_libraries(RCIRTTIntegration ${RCI_LIBRARIES} ${RSC_LIBRARIES} ${NemoMath_LIBRARIES})