/**
 * @file Main.cpp
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the GNU Lesser
 * General Public License v2.1 or any later version.
 */

#include <CentroidalMPCWalking/CentroidalMPCBlock.h>
#include <CentroidalMPCWalking/WholeBodyQPBlock.h>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/AdvanceableRunner.h>
#include <BipedalLocomotion/System/Barrier.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/QuitHandler.h>
#include <BipedalLocomotion/System/SharedResource.h>
#include <BipedalLocomotion/System/YarpClock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <memory>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>



// pit tuning stuff
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPidControl.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IRemoteVariables.h>
#include <yarp/os/Value.h>
#include <yarp/os/LogStream.h>







int main(int argc, char* argv[])
{
    constexpr auto errorPrefix = "[main]";

    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::YarpClockFactory>());

    std::string temp;
    BipedalLocomotion::log()->warn("[main] Please remember to check the mass of the robot. It is "
                                   "hardcoded in the blocks");
    std::cin >> temp;    

    // initialize yarp network
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        BipedalLocomotion::log()->error("[main] Unable to find YARP network.");
        return EXIT_FAILURE;
    }

    // prepare and configure the resource finder
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();

    rf.setDefaultConfigFile("centroidal_mpc_walking.ini");
    rf.configure(argc, argv);

    auto handler = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    handler->set(rf);

    // -------------------------------------Handle PIDs gains tuning---------------------------------------//
    bool tune_pids{false};
    if (!handler->getParameter("use_pidParams", tune_pids))
    {
        BipedalLocomotion::log()->warn("{} Not able to obtain use_pidParams from config file.", errorPrefix);
    } 

    if (tune_pids)
    {
        typedef std::map<std::string, int> AxisMap;
        typedef std::map<std::string, yarp::dev::Pid> PIDmap;

        yarp::dev::IPidControl* pidInterface{nullptr};
        yarp::dev::IEncodersTimed* encodersInterface{nullptr};
        yarp::dev::IAxisInfo* axisInfo{nullptr};

        auto ptr = handler->getGroup("ROBOT_INTERFACE").lock();  // do I need to unlock for the WBQPBlock device ?
        if (ptr == nullptr)
        {
            BipedalLocomotion::log()->error("{} Robot interface options is empty.", errorPrefix);
            return EXIT_FAILURE;
        }
        bool ok = true;
        std::vector<std::string> jointsList;
        ok = ok && ptr->getParameter("joints_list", jointsList);

        std::vector<std::string> controlBoards;
        ok = ok && ptr->getParameter("remote_control_boards", controlBoards);

        std::string robotName;
        ok = ok && ptr->getParameter("robot_name", robotName);


        if (!ok)
        {
            BipedalLocomotion::log()->error(" {} Unable to get all the parameters for PidDriver from configuration file.",
                                    errorPrefix);
            return EXIT_FAILURE;
        }

        yarp::os::Property options;
        options.put("device", "remotecontrolboardremapper");

        options.addGroup("axesNames");
        yarp::os::Bottle& bottle = options.findGroup("axesNames").addList();
        for (const auto& joint : jointsList)
            bottle.addString(joint);

        yarp::os::Bottle remoteControlBoards;

        yarp::os::Bottle& remoteControlBoardsList = remoteControlBoards.addList();
        for (const auto& controlBoard : controlBoards)
            remoteControlBoardsList.addString("/" + robotName + "/" + controlBoard);

        options.put("remoteControlBoards", remoteControlBoards.get(0));
        options.put("localPortPrefix", "/PidremoteControlBoard");
        yarp::os::Property& remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
        remoteControlBoardsOpts.put("writeStrict", "on");

        yarp::dev::PolyDriver PidTuningdevice(options);

        if (!PidTuningdevice.isValid())
        {
            
            BipedalLocomotion::log()->error(" {} Could not open the pid tuning polydriver object..",
                                    errorPrefix);
            return EXIT_FAILURE;
        }

        if (!PidTuningdevice.view(pidInterface) || !pidInterface) {
            BipedalLocomotion::log()->error("{} Device not available to implement PID Interface",
                                                errorPrefix);
            return EXIT_FAILURE;
        }
        if (!PidTuningdevice.view(encodersInterface) || !encodersInterface) {
            BipedalLocomotion::log()->error("{} Device not available to implement encoders Interface",
                                                errorPrefix);
            return EXIT_FAILURE;
        }
        if (!PidTuningdevice.view(axisInfo) || !axisInfo) {
            BipedalLocomotion::log()->error("{} Device not available to implement encoders axis information",
                                                errorPrefix);
            return EXIT_FAILURE;
        }
        int axes;
        if (!encodersInterface->getAxes(&axes))
        {
            BipedalLocomotion::log()->error("{} Error getting the axes of the torso device",
                                                errorPrefix);
            return EXIT_FAILURE;
        }

        yarp::dev::Pid current_gains;
        yarp::dev::Pid desired_gains;
        PIDmap pid_map;
        AxisMap axis_map;

        for (int ax = 0; ax < axes; ax++)
        {
            std::string axisName;
            yarp::dev::Pid currentAxispid;

            if (axisInfo->getAxisName(ax, axisName)) {
                std::pair<AxisMap::iterator, bool> result = axis_map.insert(AxisMap::value_type(axisName, ax));
                if (!result.second) {
                    BipedalLocomotion::log()->error("{} Error getting the axes from the pid tuning device",
                                                errorPrefix);
                    return EXIT_FAILURE;
                }

                if (pidInterface->getPid(yarp::dev::VOCAB_PIDTYPE_POSITION, ax, &currentAxispid))
                {
                    std::pair<PIDmap::iterator, bool> result = pid_map.insert(PIDmap::value_type(axisName, currentAxispid));
                    if (!result.second) {
                        yError("Error while inserting an item in the output map");
                        return false;
                    }
                }
                

            } else {
                BipedalLocomotion::log()->error("{} Error while retrieving the current info of axis {}",
                                                errorPrefix,
                                                axisName);
                return EXIT_FAILURE;
            }
        }
        
        yarp::os::Property prop;
        prop.fromConfigFile("pidParams.ini");
        auto defaultGroup = prop.findGroup("DEFAULT");



        
        if (defaultGroup.isNull() || defaultGroup.size() == 1)
        {
            BipedalLocomotion::log()->warn("{} the pid parameters configuration file doesn't have a default group, using current values",
                                                errorPrefix);
        } else{
        for (int i = 1; i < defaultGroup.size(); ++i){
            std::string jointName;
            bool ok = false;
            yarp::os::Value subgroup = defaultGroup.get(i);
            ok = subgroup.isList() && subgroup.asList()->get(0).isString();
            if (ok){
                yarp::os::Bottle* jointValues;
                jointValues = subgroup.asList();
                if ((jointValues->get(0).isString()) && (jointValues->size() == 4)) {
                    jointName = jointValues->get(0).toString();
                } else {
                    BipedalLocomotion::log()->error("{} Invalid joint in the pidParams file {}",
                                                errorPrefix,
                                                jointValues->toString());
                    return false;
                }
                for (int g = 1; g < 4; ++g) {
                    yarp::os::Value gain = jointValues->get(g);
                    if(!(gain.isFloat64())&& !(gain.isInt32())){
                        BipedalLocomotion::log()->error("{} The gains for joint {} are not numeric values",
                                                    errorPrefix,
                                                    jointName);
                        return EXIT_FAILURE;
                    }
                }
                
                desired_gains.setKp(jointValues->get(1).asFloat64());
                desired_gains.setKi(jointValues->get(2).asFloat64());
                desired_gains.setKd(jointValues->get(3).asFloat64());          
            }
            int ax;
            for (const auto& [key, value] : axis_map) {
                if (jointName == key)
                {
                    ax = value;
                }
                
            }
            for (const auto& [key, value] : pid_map) {
                if (jointName == key && !(desired_gains == value))
                {
                    if (!pidInterface->getPid(yarp::dev::VOCAB_PIDTYPE_POSITION, ax, &current_gains))
                    {
                        BipedalLocomotion::log()->error("{} Error while setting the new pid gaines for joint {}",
                                                errorPrefix,
                                                jointName);
                        return EXIT_FAILURE;
                    }
                    desired_gains.setMaxInt(current_gains.max_int);
                    desired_gains.setMaxOut(current_gains.max_output);
                    desired_gains.setOffset(current_gains.offset);
                    desired_gains.setScale(current_gains.scale);
                    desired_gains.setStictionValues(current_gains.stiction_up_val, current_gains.stiction_down_val);
                    if (!pidInterface->setPid(yarp::dev::VOCAB_PIDTYPE_POSITION, ax, desired_gains))
                    {
                        BipedalLocomotion::log()->error("{} Error while setting the new pid gaines for joint {}",
                                                errorPrefix,
                                                jointName);
                        return EXIT_FAILURE;
                    }
                    BipedalLocomotion::log()->info("{} New gain was set for joint {}",
                                                errorPrefix,
                                                jointName);   
                    
                }     
            }
        }
        }

        // TODO, unlock handler
        PidTuningdevice.close();
        pid_map.clear();
        axis_map.clear();

    } else{
        BipedalLocomotion::log()->warn("{} Not going to use pidParams file for tuning." ,
                                                errorPrefix);   
    }
    
    // -------------------------------------Handle the two runners---------------------------------------//
    std::string temp2;
    BipedalLocomotion::log()->warn("{}Please make sure the current PID gains are matching the configuration file",
                                    errorPrefix);
    std::cin >> temp2;

    // create the module
    auto wholeBodyQPBlock = std::make_unique<CentroidalMPCWalking::WholeBodyQPBlock>();
    if (!wholeBodyQPBlock->initialize(handler))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the whole body block.",
                                        errorPrefix);
        return EXIT_FAILURE;
    }

    auto input0 = BipedalLocomotion::System::SharedResource<
        CentroidalMPCWalking::WholeBodyQPBlock::Input>::create();
    auto output0 = BipedalLocomotion::System::SharedResource<
        CentroidalMPCWalking::WholeBodyQPBlock::Output>::create();

    BipedalLocomotion::System::AdvanceableRunner<CentroidalMPCWalking::WholeBodyQPBlock>
        wholeBodyRunner;

    if (!wholeBodyRunner.initialize(handler->getGroup("WHOLE_BODY_RUNNER")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the whole body runner.",
                                        errorPrefix);
        return EXIT_FAILURE;
    }

    wholeBodyRunner.setInputResource(input0);
    wholeBodyRunner.setOutputResource(output0);
    wholeBodyRunner.setAdvanceable(std::move(wholeBodyQPBlock));

    ///////// centroidal
    auto centoidalMPCBlock = std::make_unique<CentroidalMPCWalking::CentroidalMPCBlock>();
    if (!centoidalMPCBlock->initialize(handler->getGroup("CENTROIDAL_MPC")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the centoidal mpc block.",
                                        errorPrefix);
        return EXIT_FAILURE;
    }

    BipedalLocomotion::System::AdvanceableRunner<CentroidalMPCWalking::CentroidalMPCBlock>
        centroidalMPCRunner;

    if (!centroidalMPCRunner.initialize(handler->getGroup("CENTOIDAL_MPC_RUNNER")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the centoidal runner.",
                                        errorPrefix);
        return EXIT_FAILURE;
    }

    centroidalMPCRunner.setInputResource(output0);
    centroidalMPCRunner.setOutputResource(input0);
    centroidalMPCRunner.setAdvanceable(std::move(centoidalMPCBlock));

    BipedalLocomotion::System::handleQuitSignals([&]() {
        centroidalMPCRunner.stop();
        wholeBodyRunner.stop();
    });

    // Run the threads
    BipedalLocomotion::System::Barrier barrier(2);

    auto threadWBC = wholeBodyRunner.run(barrier);
    auto threadMPC = centroidalMPCRunner.run(barrier);

    while (wholeBodyRunner.isRunning() && centroidalMPCRunner.isRunning())
    {
        using namespace std::chrono_literals;
        constexpr auto delay = 200ms;
        BipedalLocomotion::clock().sleepFor(delay);
    }

    centroidalMPCRunner.stop();
    wholeBodyRunner.stop();

    if (threadMPC.joinable())
    {
        threadMPC.join();
        threadMPC = std::thread();
    }

    if (threadWBC.joinable())
    {
        threadWBC.join();
        threadWBC = std::thread();
    }

    return EXIT_SUCCESS;
}
