<h1 align="center">
  Online Non-linear Centroidal MPC for Humanoid Robots Payload Carrying with Contact-Stable Force Parametrization
</h1>

<div align="center">


Mohamed Elobaid, Giulio Romualdi, Gabriele Nava, Lorenzo Rapetti, Hosameldin Awadalla Omer Mohamed, Daniele Pucci

</div>

<p align="center">


</p>

<div align="center">
  IEEE International Conference of Robotics and Automation (ICRA), 2023
</div>

<div align="center">
  <a href="#installation"><b>installation</b></a> |
  <a href="#usage"><b>usage</b></a> |
  <a href="https://arxiv.org/abs/2305.10917"><b>pre-print</b></a> |
  <a href="https://youtu.be/QQG_kwMi3o0"><b>youtube</b></a>
</div>



https://github.com/ami-iit/paper_elobaid_2023_icra_walking_with_payloads/assets/36491318/fdda31c2-f015-4539-8252-d29681b9eefb



### Abstract
In this paper we consider the problem of allowing a humanoid robot that is subject to a persistent disturbance, in the form of a payload-carrying task, to follow given planned footsteps. To solve this problem, we combine an online nonlinear centroidal Model Predictive Controller - MPC  with a contact stable force parametrization. The cost function of the MPC is augmented with terms handling the disturbance and regularizing the parameter. The performance of the resulting controller is validated  both in simulations and on the humanoid robot iCub. Finally, the effect of using the parametrization on the computational time of the controller is briefly studied.


### Installation

The main difference with respect to [the original centroidal-mpc](https://github.com/ami-iit/paper_romualdi_2022_icra_centroidal-mpc-walking) is not on the application side, but on the `bipedal-locomotion-framework` side. In our case we build a version of the framework that implements the parametrized Reduced model controllers component. 

#### General dependencies and robotology-superbuild

It is recommended that you follow the installation guidelines for the [robotology-superbuild](https://github.com/robotology/robotology-superbuild)

Make sure that you set in the CMake options, the following [profiles](https://github.com/robotology/robotology-superbuild/blob/master/doc/cmake-options.md#profile-cmake-options) are set

```
- ROBOTOLOGY_USES_GAZEBO
- ROBOTOLOGY_ENABLE_DYNAMICS
- ROBOTOLOGY_ENABLE_DYNAMICS_FULL_DEPS
```

#### Ad-hoc installation guidelines

Once the robotology-superbuild installation is followed correctly. You need to follow the following instructions


clone this repo

then clone this repo 

```
git clone https://github.com/ami-iit/paper_elobaid_2023_icra_walking_with_payloads.git
```

then go to the `bipedal-locomotion-framework` repo

```
cd /where/you/cloned/robotology-superbuild/src/bipedal-locomotion-framework/src
git checkout feature/CentroidalMPC
```

You need to replace the contents of the folder `ReducedModelControllers` in the bipedal-locomotion-framework with the folder of the same name found in this repo, then

```
cd /where/you/cloned/robotology-superbuild/builde/src/bipedal-locomotion-framework
cmake .
[sudo] make && make install
```
the `sudo` option in case the `CMAKE_INSTALL_PREFIX` was not set to a local path.

Now we clone and build the application provided in this repo

```
cd /where/you/cloned/this/repo/application && mkdir build
cd build
ccmake ..
[sudo] make && make install
```

Finally, to be able to use the gazebo worlds provided by this repo, you need to set the following environment variables

```
export YARP_DATA_DIRS=$YARP_DATA_DIRS:/where/you/installed/this/repo/application/world/centroidal_mpc_ergocub

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/where/you/installed/this/repo/application/world/centroidal_mpc_ergocub
```


### Usage

To use the application and reproduce the experiments, follow the instructions below


1. set up the robot name

```
export YARP_ROBOT_NAME="icubGazeboV3"
```

2.  run a `yarpserver`

```
yarpserver --write
```

3. launch the gazebo world

```
export YARP_CLOCK=/clock
gazebo -slibgazebo_yarp_clock.so /where/you/installed/this/repo/application/world/world
```

4. run a `yarprobotinterface`

```
 YARP_CLOCK=/clock yarprobotinterface --config launch-wholebodydynamics.xml
```

4. run the application using the correct config files

```
cd /where/you/installed/this/repo/application/src/centroidal-mpc-walking/config/robots/iCubGazeboV3

/where/you/installed/the/application/cwm 
```

to plot the data, use matlab to run the script `plot_data.m`

### Citing this work

If you find the work useful, please consider citing:

```bibtex
@ARTICLE{mebbaid2023ICRA,
  author={Mohamed Elobaid, Giulio Romualdi, Gabriele Nava, Lorenzo Rapetti, Hosameldin Awadalla Omer Mohamed, Daniele Pucci},
  title={Online Non-linear Centroidal MPC for Humanoid Robots Payload Carrying with Contact-Stable Force Parametrization},
  booktitle={2023 International Conference on Robotics and Automation (ICRA)},
  year={2023}}
```

### Maintainer

This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/mebbaid.png" width="40">](https://github.com/mebbaid) | [@mebbaid](https://github.com/mebbaid) |
