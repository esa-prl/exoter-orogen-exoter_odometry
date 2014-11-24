/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

//#define DEBUG_PRINTS 1

using namespace exoter_odometry;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    /******************************************/
    /*** General Internal Storage Variables ***/
    /******************************************/

    /** Default size for the std_vector for the Cartesian 6DoF velocities variables **/
    vectorCartesianVelocities = std::vector< Eigen::Matrix <double, 6, 1> , Eigen::aligned_allocator < Eigen::Matrix <double, 6, 1> > > (2);
    vectorCartesianVelocities[0].setZero(); vectorCartesianVelocities[1].setZero();

    /** Weighting Matrix Initialization **/
    WeightMatrix =  base::NaN<double>() * Eigen::Matrix<double, 6*exoter::NUMBER_OF_WHEELS, 6*exoter::NUMBER_OF_WHEELS, 1>::Identity();

    /** Uncertainty matrices for the motion model **/
    cartesianVelCov = base::NaN<double>() * Eigen::Matrix< double, 6, 6 >::Identity();
    modelVelCov = base::NaN<double>() * Eigen::Matrix< double, RobotMotionModel::MODEL_DOF, RobotMotionModel::MODEL_DOF >::Identity();

    /***************************/
    /** Input port variables  **/
    /***************************/
    orientationSamples.orientation = Eigen::Quaterniond(base::NaN<double>()*Eigen::Vector4d::Ones());
    orientationSamples.cov_orientation = base::NaN<double>() * Eigen::Matrix3d::Ones();

    /***************************/
    /** Output port variables **/
    /***************************/
    base::samples::RigidBodyState poseInit;
    poseInit.invalidate();
    pose = poseInit;

}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::joints_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Joints &joints_samples_sample)
{
    double delta_t = static_cast<const double>(_joints_samples_period.value());
    //double delta_t = joints_samples_sample.time.toSeconds() - jointsSamples.time.toSeconds();

    /** Get the Joints values  **/
    jointsSamples = joints_samples_sample;
    jointPositions.setZero();
    jointVelocities.setZero();


    /** Mechanical joints ordered by jointsName **/
    register int jointIdx = 0;
    for(std::vector<std::string>::const_iterator it = jointsNames.begin();
    it != jointsNames.end(); it++)
    {
        base::JointState const &state(jointsSamples[*it]);

        /** Avoid NaN values in position **/
        if (std::isfinite(state.position))
            jointPositions[jointIdx] = state.position;
        else
            jointPositions[jointIdx] = 0.00;

        /** Set to zero in case of a driving wheel **/
        if((*it).find("drive") != std::string::npos)
            jointPositions[jointIdx] = 0.00;

        /** Avoid NaN values in velocity **/
        if (std::isfinite(state.speed))
            jointVelocities[jointIdx] = state.speed;
        else
            throw std::runtime_error("[EXOTER_ODOMETRY JOINT_SAMPLES] Joint speed cannot be NaN.");

        jointIdx++;

    }

    /** Fill the rest of jointVelocities (unknown quantities) **/
    jointVelocities.block<exoter::NUMBER_OF_WHEELS*exoter::SLIP_VECTOR_SIZE,1> (exoter::EXOTER_JOINT_DOF, 0) = Eigen::Matrix<double, exoter::NUMBER_OF_WHEELS*exoter::SLIP_VECTOR_SIZE, 1>::Identity() * base::NaN<double>();
    jointVelocities.block<exoter::NUMBER_OF_WHEELS*exoter::CONTACT_POINT_DOF,1> (exoter::EXOTER_JOINT_DOF+(exoter::NUMBER_OF_WHEELS*exoter::SLIP_VECTOR_SIZE), 0) = Eigen::Matrix<double, exoter::NUMBER_OF_WHEELS*exoter::CONTACT_POINT_DOF, 1>::Identity() * base::NaN<double>();

    #ifdef DEBUG_PRINTS
    std::cout<<"[EXOTER_ODOMETRY JOINT_SAMPLES] Received time-stamp:\n"<<jointsSamples.time.toMicroseconds()<<"\n";
    std::cout<<"[EXOTER_ODOMETRY JOINT_SAMPLES] Joint Positions:\n"<<jointPositions<<"\n";
    std::cout<<"[EXOTER_ODOMETRY JOINT_SAMPLES] Joint Velocities:\n"<<jointVelocities<<"\n";
    #endif

    #ifdef DEBUG_PRINTS
    /** Check the time difference between joint and orientation samples **/
    base::Time diffTime = jointsSamples.time - orientationSamples.time;

    std::cout<<"[EXOTER_ODOMETRY JOINT_SAMPLES] [ON] ("<<jointsSamples.time.toMicroseconds()<<")\n";
    std::cout<<"[EXOTER_ODOMETRY JOINT_SAMPLES] diffTime:\n"<<diffTime.toSeconds()<<"\n";
    #endif

    /** Perform the Motion Model and the Dead-reckoning **/
    if (orientationSamples.time.toSeconds() != 0.00)
    {
        this->updateOdometry(delta_t);
    }
}

void Task::orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    double delta_t = static_cast<const double>(_orientation_samples_period.value());
    //double delta_t = orientation_samples_sample.time.toSeconds() - orientationSamples.time.toSeconds();

    /** Invalidate every step in order to set it from the beginning **/
    deltaPose.invalidate();

    /** Delta quaternion (rotation k-1 - rotation k) **/
    deltaPose.orientation = orientationSamples.orientation.inverse() * orientation_samples_sample.orientation; /** (T0_k-1)^-1 * T0_k **/
    deltaPose.cov_orientation = orientation_samples_sample.cov_orientation - orientationSamples.cov_orientation;

    /** Angular velocity **/
    Eigen::AngleAxisd deltaAngleaxis(deltaPose.orientation);
    Eigen::Vector3d angular_velocity = (deltaAngleaxis.angle() * deltaAngleaxis.axis())/delta_t;

    /** Fill the Cartesian Velocities **/
    cartesianVelocities.block<3,1> (0,0) = Eigen::Matrix<double, 3, 1>::Identity() * base::NaN<double>();
    cartesianVelocities.block<3,1> (3,0) = angular_velocity;//!Angular velocities come from gyros

    /** Fill the Cartesian velocity covariance **/
    cartesianVelCov.block<3,3>(3,3) = deltaPose.cov_orientation / (delta_t);// * delta_t);

    /** Get the orientation readings  **/
    orientationSamples = orientation_samples_sample;

    #ifdef DEBUG_PRINTS
    std::cout<<"[EXOTER_ODOMETRY INERTIAL_SAMPLES] Received time-stamp:\n"<<orientationSamples.time.toMicroseconds()<<"\n";
    std::cout<<"[EXOTER_ODOMETRY INERTIAL_SAMPLES] Cartesian Velocities:\n"<<cartesianVelocities<<"\n";
    #endif

    #ifdef DEBUG_PRINTS
    /** Check the time difference between orientation and joint samples **/
    base::Time diffTime = orientationSamples.time - jointsSamples.time;
    std::cout<<"[EXOTER_ODOMETRY INERTIAL_SAMPLES] diffTime:\n"<<diffTime.toSeconds()<<"\n";
    #endif
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    ::base::samples::RigidBodyState poseInit;

    /************************/
    /** Read configuration **/
    /************************/
    urdfFile = _urdf_file.value();
    wheelRadius = _wheelRadius.value();
    modelType = _modelType.value();
    centerOfMass = _robot_CoM.value();
    iirConfig = _iir_filter.value();
    jointsNames = _jointsNames.value();

    /** Info with regard to the Dynamic Weighting Matrix **/
    if (centerOfMass.dynamicOn)
        RTT::log(RTT::Warning)<<"[EXOTER_ODOMETRY] Dynamic Weight Matrix [ON]"<<RTT::endlog();
    else
        RTT::log(RTT::Warning)<<"[EXOTER_ODOMETRY] Dynamic Weight Matrix [OFF]"<<RTT::endlog();

    if (iirConfig.iirOn)
        RTT::log(RTT::Warning)<<"[EXOTER_ODOMETRY] Infinite Impulse Response Filter [ON]"<<RTT::endlog();
    else
        RTT::log(RTT::Warning)<<"[EXOTER_ODOMETRY] Infinite Impulse Response Filter [OFF]"<<RTT::endlog();

    /*********************************************/
    /** Configure the Motion Model of the Robot **/
    /*********************************************/

    if (modelType == NUMERICAL)
    {
        /** Robot Kinematics Model **/
        robotKinematics.reset(new exoter::ExoterKinematicKDL (urdfFile, wheelRadius));
        RTT::log(RTT::Warning)<<"[EXOTER_ODOMETRY] Numerical Model selected"<<RTT::endlog();
    }
    else if (modelType == ANALYTICAL)
    {
        /** Robot Kinematics Model **/
        robotKinematics.reset(new exoter::ExoterKinematicModel (wheelRadius));
        RTT::log(RTT::Warning)<<"[EXOTER_ODOMETRY] Analytical Model selected"<<RTT::endlog();
    }
    else
        throw std::runtime_error("[EXOTER_ODOMETRY] Invalid Odometry Model.");


    /** Create the Motion Model **/
    bool motionModelStatus;
    motionModel =  RobotMotionModel(motionModelStatus, RobotMotionModel::LOWEST_POINT , robotKinematics);

    /****************/
    /** IIR Filter **/
    /****************/
    Eigen::Matrix <double, localization::NORDER_BESSEL_FILTER+1, 1> besselBCoeff, besselACoeff;
    besselBCoeff = iirConfig.feedForwardCoeff;
    besselACoeff = iirConfig.feedBackCoeff;

    /** Create the Bessel Low-pass filter with the right coefficients **/
    bessel.reset(new localization::IIR<localization::NORDER_BESSEL_FILTER, 3> (besselBCoeff, besselACoeff));

    /*******************************/
    /** Inertial Noise Covariance **/
    /*******************************/

    /** Angular velocity coming from gyros **/
    cartesianVelCov.setZero(); modelVelCov.setZero();

    /***************************/
    /** Input port variables **/
    /***************************/
    orientationSamples.position.setZero();
    orientationSamples.cov_position.setZero();
    orientationSamples.orientation.setIdentity();
    orientationSamples.cov_orientation.setZero();


    /*************************************/
    /** Reset initial Starting position **/
    /*************************************/

    /** Set the initial world to navigation frame transform **/
    poseInit.invalidate();
    poseInit.sourceFrame = _odometry_source_frame.get();
    poseInit.targetFrame = _odometry_target_frame.get();

    /** Staring zero position **/
    poseInit.position.setZero();
    poseInit.velocity.setZero();

    /** Assume well known starting position **/
    poseInit.cov_position = Eigen::Matrix3d::Zero();
    poseInit.cov_velocity = Eigen::Matrix3d::Zero();

    #ifdef DEBUG_PRINTS
    std::cout<< "[EXOTER_ODOMETRY]\n";
    std::cout<< "******** Initial Position *******"<<"\n";
    std::cout<< poseInit.position<<"\n";
    std::cout<< poseInit.cov_position<<"\n";
    #endif

    /** Orientation with respect to the relative navigation frame**/
    poseInit.orientation = Eigen::Quaterniond::Identity();
    poseInit.angular_velocity.setZero();

    /** Assume very well known initial attitude **/
    poseInit.cov_orientation = Eigen::Matrix3d::Zero();
    poseInit.cov_angular_velocity = Eigen::Matrix3d::Zero();

    /** Get the Initial pose and uncertainty **/
    pose = poseInit.getTransform();
    poseCov << poseInit.cov_position, Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Zero(), poseInit.cov_orientation;

    #ifdef DEBUG_PRINTS
    std::cout<< "[EXOTER_ODOMETRY]\n";
    std::cout<< "******** Initial Position *******"<<"\n";
    std::cout<< pose.translation()<<"\n";
    std::cout<< "Initial Uncertainty"<<"\n";
    std::cout<< poseInit.cov_position<<"\n";
    Eigen::Vector3d euler;
    euler[2] = poseInit.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
    euler[1] = poseInit.orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
    euler[0] = poseInit.orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
    std::cout<< "******** Initial Attitude *******"<<"\n";
    std::cout<< "Initial Roll: "<<euler[0]*R2D<<" Init Pitch: "<<euler[1]*R2D<<" Init Yaw: "<<euler[2]*R2D<<"\n";
    std::cout<< "Initial Uncertainty"<<"\n";
    std::cout<< poseInit.cov_orientation<<"\n";
    std::cout<< "Initial Complete Uncertainty"<<"\n";
    std::cout<< poseCov<<"\n";
    #endif

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
WeightingMatrix Task::dynamicWeightMatrix (CenterOfMassConfiguration &centerOfMass, base::Orientation &orientation)
{
    std::vector< Eigen::Affine3d > fkRobot; /** Robot kinematics */
    std::vector< base::Matrix6d > cov; /** Covariance of the kinematics **/
    WeightingMatrix weightLocal; /** Local variable to return */
    std::vector< int > contactPoints; /** Contact points */
    std::vector< Eigen::Matrix<double, 3, 1> , Eigen::aligned_allocator < Eigen::Matrix<double, 3, 1> > > chainPosition; /** Chain position of contact points **/
    Eigen::Matrix<double, exoter::NUMBER_OF_WHEELS, 1> forces; /** forces to calculate */
    double theoretical_g = 9.81; /** It is not important to be exactly the real theoretical g at the location **/

    /** Set to the identity **/
    weightLocal.setIdentity();

    if (centerOfMass.dynamicOn)
    {
        /** Get the Forward Kinematics from the model  **/
        motionModel.getKinematics(fkRobot, cov);

        /** Points in contact **/
        contactPoints = motionModel.getPointsInContact();

        /** Resize the chainPosition **/
        chainPosition.resize(contactPoints.size());

        /** Form the chainPosition vector **/
        for (std::vector<int>::size_type i = 0; i < contactPoints.size(); ++i)
        {
            chainPosition[i] = fkRobot[i].translation();
        }

        /** Compute the forces **/
        //exoter::BodyState::forceAnalysis(centerOfMass.coordinates, chainPosition, static_cast<Eigen::Quaterniond>(orientation), theoretical_g, forces);

        /** Compute the percentages **/
        for (register int i=0; i<static_cast<int>(exoter::NUMBER_OF_WHEELS); ++i)
        {
            if (forces[i] > 0.00)
                centerOfMass.percentage[i] = forces[i] / theoretical_g;
            else
                centerOfMass.percentage[i] = 0.001; //Almost zero value to this wheel.zero cannot be because there is not solution to the motion model
        }
    }

    /** Form the weighting Matrix (static or dynamic it needs to be created) **/
    for (register int i=0; i<static_cast<int>(exoter::NUMBER_OF_WHEELS); ++i)
    {
        weightLocal.block<6,6>(6*i, 6*i) = centerOfMass.percentage[i] * Eigen::Matrix<double, 6, 6>::Identity();
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[WEIGHT-MATRIX] forces:\n"<< forces <<"\n";
    std::cout<<"[WEIGHT-MATRIX] percentage:\n"<< centerOfMass.percentage<<"\n";
    std::cout<<"[WEIGHT-MATRIX] Weighting Matrix:\n"<< weightLocal<<"\n";
    #endif

    return weightLocal;
}

void Task::updateOdometry (const double &delta_t)
{

    /** Update the Motion Model (Forward Kinematics and Contact Points) **/
    this->motionModel.updateKinematics(jointPositions);

    /** Compute dynamic Weight matrix depending on the attitude **/
    base::Pose tempPose( pose );
    this->WeightMatrix = dynamicWeightMatrix(centerOfMass, tempPose.orientation);

    /** In case of no noise in properties, set to zero to get the noise from the Motion Model **/
    if (!_orientation_samples_noise_on.value())
        cartesianVelCov.setZero();

    /** Solve the navigation kinematics **/
    this->motionModel.navSolver(jointPositions, cartesianVelocities, jointVelocities,
                                cartesianVelCov, modelVelCov, WeightMatrix);

    /** Bessel IIR Low-pass filter of the linear cartesianVelocities from the Motion Model **/
    if (iirConfig.iirOn)
    {
        Eigen::Matrix<double, 3, 1> velocity = cartesianVelocities.block<3, 1>(0,0);
        Eigen::Matrix<double, 3, 3> velocityCov = cartesianVelCov.block<3, 3>(0,0);
        cartesianVelocities.block<3, 1>(0,0) = this->bessel->perform(velocity, velocityCov, false);

        /** Store the filtered velocity uncertainty (Uncertainty propagation is time-correlated by the IIR) **/
        cartesianVelCov.block<3, 3>(0,0) = velocityCov;
    }

    /** Update the Cartesian velocities on the std_vector **/
    this->vectorCartesianVelocities[1] = this->vectorCartesianVelocities[0];
    this->vectorCartesianVelocities[0] = cartesianVelocities;

    /** Complete the delta pose **/
    deltaPose.position = delta_t * ((vectorCartesianVelocities[1].block<3,1>(0,0) + vectorCartesianVelocities[0].block<3,1>(0,0))/2.0);
    deltaPose.cov_position = cartesianVelCov.block<3, 3>(0,0) * delta_t;// * delta_t;

    /** Take uncertainty on delta orientation from the motion model **/
    if (!_orientation_samples_noise_on.value())
    {
        deltaPose.cov_orientation = cartesianVelCov.block<3, 3>(3,3) * delta_t;// * delta_t;
    }

    /** Perform the velocities integration to get the pose (Dead Reckoning) **/
    Eigen::Matrix<double, 6, 6> deltaPoseCov;
    deltaPoseCov<< deltaPose.cov_position, Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Zero(), deltaPose.cov_orientation;
    localization::DeadReckon::updatePose(delta_t, pose, poseCov,
                                        deltaPose.getTransform(), deltaPoseCov,
                                        pose, poseCov, true);

    /** Compute the Model Sensitivity Analysis **/
    if (_output_debug.value())
    {
        Eigen::Matrix<double, 3+exoter::EXOTER_JOINT_DOF, 1> parameter;
        Eigen::Matrix<double, 3+exoter::EXOTER_JOINT_DOF, 1> Tstate, TCov;
        parameter.block<3, 1> (0,0) = cartesianVelocities.block<3,1>(3,0); //!Angular velocities
        parameter.block<exoter::EXOTER_JOINT_DOF, 1> (3,0) = jointVelocities.block<exoter::EXOTER_JOINT_DOF, 1>(0,0);//!Joint velocities

        Tstate = modelAnalysis.solve(cartesianVelocities.block<3,1> (0,0), cartesianVelCov.block<3,3>(0,0), parameter, TCov);

        sensitivity.time = jointsSamples.time; //!timestamp
        sensitivity.Tstate = Tstate;
        sensitivity.TCovariance = TCov;
    }

    /** Out port the information **/
    this->outputPortSamples (jointPositions, cartesianVelocities, jointVelocities, sensitivity);


    return;
}


void Task::outputPortSamples(const Eigen::Matrix< double, RobotMotionModel::MODEL_DOF, 1  > &jointPositions,
                                const Eigen::Matrix< double, 6, 1  > &cartesianVelocities,
                                const Eigen::Matrix< double, RobotMotionModel::MODEL_DOF, 1  > &jointVelocities,
                                const exoter_odometry::SensitivityAnalysis &sensitivityAnalysis)
{
    base::samples::RigidBodyState poseOut;
    std::vector<Eigen::Affine3d> fkRobot;

    /***************************************/
    /** Port out the OutPorts information **/
    /***************************************/

    /** The Motion Model Estimated pose **/
    //pose.copyToRigidBodyState(poseOut);
    poseOut.setTransform(pose);
    poseOut.sourceFrame = _odometry_source_frame.value();
    poseOut.targetFrame = _odometry_target_frame.value();
    poseOut.time = jointsSamples.time; //!timestamp;
    poseOut.cov_position = poseCov.block<3,3>(0,0);
    poseOut.cov_orientation = poseCov.block<3,3>(3,3);
    poseOut.velocity = poseOut.orientation * cartesianVelocities.block<3,1>(0,0);
    poseOut.cov_velocity = (poseOut.orientation.matrix().transpose() * cartesianVelCov.block<3,3>(0,0).inverse() * poseOut.orientation.matrix()).inverse();
    poseOut.angular_velocity = cartesianVelocities.block<3,1> (3,0);
    poseOut.cov_angular_velocity = cartesianVelCov.block<3,3>(3,3);
    _pose_samples_out.write(poseOut);


    /** The Delta pose of this step **/
    deltaPose.time = jointsSamples.time;
    deltaPose.sourceFrame = _delta_odometry_source_frame.value();
    deltaPose.targetFrame = _delta_odometry_target_frame.value();
    deltaPose.velocity =  cartesianVelocities.block<3,1>(0,0);
    deltaPose.cov_velocity = cartesianVelCov.block<3,3>(0,0);
    deltaPose.angular_velocity = cartesianVelocities.block<3,1>(3,0);
    deltaPose.cov_angular_velocity = cartesianVelCov.block<3,3>(3,3);
    _delta_pose_samples_out.write(deltaPose);

    /** Debug information **/
    if (_output_debug.value())
    {
        std::vector<Eigen::Affine3d> fkRobot;
        std::vector<base::Matrix6d> fkRobotCov;
        this->motionModel.getKinematics(fkRobot, fkRobotCov);

        /** Forward kinematics information. Set of contact points. **/
        exoter_odometry::RobotContactPointsRbs robotKineRbs;

        robotKineRbs.time = poseOut.time;
        robotKineRbs.rbsChain.resize(fkRobot.size());

        /** For the movement of the points with respect to the body center **/
        for (register size_t i=0; i<fkRobot.size(); ++i)
        {
            robotKineRbs.rbsChain[i].invalidate();
            robotKineRbs.rbsChain[i].time = robotKineRbs.time;
            robotKineRbs.rbsChain[i].setTransform(fkRobot[i]);
            robotKineRbs.rbsChain[i].cov_position = fkRobotCov[i].topLeftCorner<3,3>();
            robotKineRbs.rbsChain[i].cov_orientation = fkRobotCov[i].bottomRightCorner<3,3>();
        }

        _fkchains_rbs_out.write(robotKineRbs);

        /** Sensitivity Analysis **/
        _sensitivity_analysis.write(sensitivityAnalysis);

        /** Output port **/
        base::VectorXd weightOut(centerOfMass.percentage);
        _weighting_vector.write(weightOut);
    }


    #ifdef DEBUG_PRINTS
    std::cout<<"[EXOTER_ODOMETRY OUTPUT_PORTS]: poseOut.position\n"<<poseOut.position<<"\n";
    std::cout<<"[EXOTER_ODOMETRY OUTPUT_PORTS]: poseOut.cov_position\n"<<poseOut.cov_position<<"\n";
    std::cout<<"[EXOTER_ODOMETRY OUTPUT_PORTS]: poseOut.velocity\n"<<poseOut.velocity<<"\n";
    Eigen::Vector3d euler;
    euler[2] = poseOut.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
    euler[1] = poseOut.orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
    euler[0] = poseOut.orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
    std::cout<<"[EXOTER_ODOMETRY OUTPUT_PORTS]: Pose Orientation\n";
    std::cout<<"Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
    std::cout<<"[EXOTER_ODOMETRY OUTPUT_PORTS]: Pose cov_orientation\n"<<poseOut.cov_orientation<<"\n";
    #endif

    #ifdef DEBUG_PRINTS
    std::cout<<"[EXOTER_ODOMETRY OUTPUT_PORTS]: deltaPose.position\n"<<deltaPose.position<<"\n";
    std::cout<<"[EXOTER_ODOMETRY OUTPUT_PORTS]: deltaPose.cov_position\n"<<deltaPose.cov_position<<"\n";
    Eigen::Vector3d deltaEuler;
    deltaEuler[2] = deltaPose.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
    deltaEuler[1] = deltaPose.orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
    deltaEuler[0] = deltaPose.orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
    std::cout<< "[EXOTER_ODOMETRY OUTPUT_PORTS]: Delta Pose Orientation\n";
    std::cout<< "******** Delta Rotation *******"<<"\n";
    std::cout<< "Roll: "<<deltaEuler[0]*R2D<<" Pitch: "<<deltaEuler[1]*R2D<<" Yaw: "<<deltaEuler[2]*R2D<<"\n";
    std::cout<<"[EXOTER_ODOMETRY OUTPUT_PORTS]: Delta pose cov_orientation\n"<<deltaPose.cov_orientation<<"\n";
    std::cout<<"[EXOTER_ODOMETRY OUTPUT_PORTS]\n ******************** END ******************** \n";
    #endif


    return;
}


