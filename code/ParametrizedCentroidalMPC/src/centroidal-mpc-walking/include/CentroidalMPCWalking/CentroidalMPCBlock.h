/**
 * @file CentoidalMPCBlock.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the GNU Lesser
 * General Public License v2.1 or any later version.
 */

#ifndef CENTROIDAL_MCP_WALKING_CENTROIDAL_MCP_BLOCK_H
#define CENTROIDAL_MCP_WALKING_CENTROIDAL_MCP_BLOCK_H

#include <BipedalLocomotion/Math/Wrench.h>
#include <map>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include <BipedalLocomotion/ReducedModelControllers/ParametrizedCentroidalMPC.h>
#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>

#include <BipedalLocomotion/IK/CoMTask.h>
#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/SE3Task.h>

#include <BipedalLocomotion/FloatingBaseEstimators/LeggedOdometry.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <BipedalLocomotion/System/Advanceable.h>



namespace CentroidalMPCWalking
{

struct CentoidalMPCInput
{
    Eigen::Vector3d com;
    Eigen::Vector3d dcom;
    Eigen::Vector3d angularMomentum;

    BipedalLocomotion::Math::Wrenchd totalExternalWrench;

    manif::SE3d leftFoot;
    manif::SE3d rightFoot;

    bool isValid{false};
};

} // namespace CentroidalMPCWalking

namespace CentroidalMPCWalking
{
class CentroidalMPCBlock : public BipedalLocomotion::System::Advanceable<
                              CentoidalMPCInput,
                              BipedalLocomotion::ReducedModelControllers::ParametrizedCentroidalMPCState>
{
    typename CentroidalMPCBlock::Output m_output;

    BipedalLocomotion::ReducedModelControllers::ParametrizedCentroidalMPC m_controller;
    


    BipedalLocomotion::Contacts::ContactPhaseList m_phaseList;

    bool m_inputValid{false};
    double m_currentTime{0};
    BipedalLocomotion::Contacts::ContactPhaseList::const_iterator m_phaseIt;

public:
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>
                        handler) override;

    const Output& getOutput() const override;

    bool setInput(const Input& input) override;

    bool advance() override;

    bool isOutputValid() const override;
};
} // namespace CentroidalMPCBlock

#endif // CENTROIDAL_MCP_WALKING_CENTROIDAL_MCP_BLOCK_H
