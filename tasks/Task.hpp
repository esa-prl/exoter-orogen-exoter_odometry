/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef EXOTER_ODOMETRY_TASK_TASK_HPP
#define EXOTER_ODOMETRY_TASK_TASK_HPP

#include "exoter_odometry/TaskBase.hpp"

/** Eigen **/
#include <Eigen/Core>/** Eigen core library **/
#include <Eigen/Dense> /** Algebra and transformation matrices **/
#include <Eigen/StdVector> /** STL container with Eigen types **/

/** Exoter dependencies includes **/
#include <exoter/ExoterKinematicModel.hpp> /** Analytical model **/
#include <exoter/ExoterKinematicKDL.hpp> /** KDL model **/
#include <exoter/Configuration.hpp> /** Dedicated Exoter variables and constants **/

/** Odometry include for the Motion Model **/
#include <odometry/MotionModel.hpp>

/** Localization Library includes **/
#include <localization/tools/Analysis.hpp>
#include <localization/filters/IIR.hpp>
#include <localization/core/DeadReckon.hpp>

/** Boost **/
#include <boost/shared_ptr.hpp> /** Shared pointers **/

namespace exoter_odometry {


    /** Data types definition **/
    typedef odometry::KinematicModel< double, exoter::NUMBER_OF_WHEELS, exoter::EXOTER_JOINT_DOF, exoter::SLIP_VECTOR_SIZE, exoter::CONTACT_POINT_DOF > RobotKinematicModel;
    typedef odometry::MotionModel< double, exoter::NUMBER_OF_WHEELS, exoter::EXOTER_JOINT_DOF, exoter::SLIP_VECTOR_SIZE, exoter::CONTACT_POINT_DOF > RobotMotionModel;
    typedef Eigen::Matrix<double, 6*exoter::NUMBER_OF_WHEELS, 6*exoter::NUMBER_OF_WHEELS> WeightingMatrix;


    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare the Exoter Odometry class
    The component computes the robot pose based on
    a complete motion model.
    Robot joints positions are needed to compute
    the forward kinematics of robot chains.
    Angular and robot joints
    rates are needed to compute the movement.

    The corresponding C++ class can be edited in tasks/Task.hpp and
    tasks/Task.cpp, and will be put in the exoter_odometry namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','exoter_odometry::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

        virtual void joints_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Joints &joints_samples_sample);

        virtual void orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample);

    protected:

        /**************************/
        /*** Property Variables ***/
        /**************************/

        std::string urdfFile;

        double wheelRadius;

        ModelType modelType;

        /** Center of Mass location of the robot **/
        CenterOfMassConfiguration centerOfMass;

        /** IIR filter configuration structure **/
        IIRCoefficients iirConfig;

        /** Order of Joints by Name **/
        std::vector<std::string> jointsNames;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

        /** Joint, Slip and Contact Angle positions NOTE: The order of the storage needs to be coincident if used as input for the motionModel **/
        Eigen::Matrix< double, RobotMotionModel::MODEL_DOF, 1  > jointPositions;

        /** Joint, Slip and Contact Angle velocities NOTE: The order of the storage needs to be coincident if used as input for the motionModel **/
        Eigen::Matrix< double, RobotMotionModel::MODEL_DOF, 1  > jointVelocities;

        /** Linear and Angular velocities NOTE: The order of the storage needs to be coincident if used as input for the motionModel **/
        Eigen::Matrix< double, 6, 1  > cartesianVelocities;

        /** Buffer for the storage of cartesianVelocities variables  (for integration assuming constant acceleration) **/
        std::vector< Eigen::Matrix <double, 6, 1> , Eigen::aligned_allocator < Eigen::Matrix <double, 6, 1> > > vectorCartesianVelocities;

        /** Robot Kinematic Model **/
        boost::shared_ptr< RobotKinematicModel > robotKinematics;

        /** Robot Motion Model **/
        RobotMotionModel  motionModel;

        /** Covariance Joints, Slip and Contact Angle velocities NOTE: The order of the storage needs to be coincident if used as input for the motionModel **/
        Eigen::Matrix< double, RobotMotionModel::MODEL_DOF, RobotMotionModel::MODEL_DOF > modelVelCov;

        /** Covariance Linear and Angular velocities NOTE: The order of the storage needs to be coincident to be used as input for the motionModel **/
        Eigen::Matrix< double, 6, 6  > cartesianVelCov;

        /** Bessel Low-pass IIR filter for the Motion Model velocities
         * Specification of the Order and Data dimension is required */
        boost::shared_ptr< localization::IIR<localization::NORDER_BESSEL_FILTER, localization::NUMAXIS> > bessel;

        /** Sensitivity analysis **/
        localization::Analysis <3, 3+exoter::EXOTER_JOINT_DOF> modelAnalysis; //! DoF of the analysis is 8

        /** Weighting Matrix for the Motion Model  **/
        WeightingMatrix WeightMatrix;

        /** Motion Model sensitivity analysis **/
        exoter_odometry::SensitivityAnalysis sensitivity;

        /** Delta pose step **/
        ::base::samples::RigidBodyState deltaPose;

        /***************************/
        /** Input port variables **/
        /***************************/

        ::base::samples::Joints jointsSamples;

        ::base::samples::RigidBodyState orientationSamples;


        /***************************/
        /** Output port variables **/
        /***************************/

        /** Body Center w.r.t the World Coordinate system (using statistical Motion Model and IMU orientation) */
        Eigen::Affine3d pose;
        Eigen::Matrix<double, 6, 6> poseCov;


    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "exoter_odometry::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        /** @brief Performs the odometry update
         */
        void updateOdometry(const double &delta_t);

        /** \brief Store the variables in the Output ports
         */
        void outputPortSamples(const Eigen::Matrix< double, RobotMotionModel::MODEL_DOF, 1  > &jointPositions,
                                const Eigen::Matrix< double, 6, 1  > &cartesianVelocities,
                                const Eigen::Matrix< double, RobotMotionModel::MODEL_DOF, 1  > &jointVelocities,
                                const exoter_odometry::SensitivityAnalysis &sensitivityAnalysis);

    protected:

        /** Weight matrix for the Asguard Robot **/
        WeightingMatrix dynamicWeightMatrix (CenterOfMassConfiguration &centerOfMass, base::Orientation &orientation);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    };
}

#endif

