# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Authors: Giulio Romualdi


add_centroidal_mpc_walking_application(
  NAME walking
  SOURCES src/Main.cpp src/WholeBodyQPBlock.cpp src/CentroidalMPCBlock.cpp
  HEADERS include/CentroidalMPCWalking/WholeBodyQPBlock.h include/CentroidalMPCWalking/CentroidalMPCBlock.h
  LINK_LIBRARIES   YARP::YARP_dev BipedalLocomotion::ParametersHandlerYarpImplementation
  BipedalLocomotion::RobotInterfaceYarpImplementation BipedalLocomotion::FloatingBaseEstimators BipedalLocomotion::IK BipedalLocomotion::ContinuousDynamicalSystem BipedalLocomotion::System BipedalLocomotion::SystemYarpImplementation BipedalLocomotion::ReducedModelControllers BipedalLocomotion::Planners BipedalLocomotion::ManifConversions BipedalLocomotion::ContactDetectors BipedalLocomotion::SimplifiedModelControllers
  )

# install_ini_files(${CMAKE_CURRENT_SOURCE_DIR}/config)

# install(FILES script/blf_joint_position_tracking_plot_dataset.py
#   DESTINATION "${CMAKE_INSTALL_PREFIX}/share/BipedalLocomotionFramework/python")

# install(FILES script/blf-joint-position-tracking-script.sh
#   PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
#   DESTINATION "${CMAKE_INSTALL_BINDIR}")
