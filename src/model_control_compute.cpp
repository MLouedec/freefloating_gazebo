#include <freefloating_gazebo/model_control_compute.h>
#include <freefloating_gazebo/hydro_link.h>

namespace ffg
{


void ModelControlCompute::Init(ros::NodeHandle &nh, ros::Duration&_dt, const std::vector<std::string>&_controlled_axes, std::string default_mode/* = "position"*/)
{
    velocity_error_ << 0,0,0,0,0,0;
    s_error_        << 0,0,0,0,0,0;

    // init dt from rate
    dt = _dt;

    // wrench setpoint
    wrench_sp_subscriber =
            nh.subscribe("body_wrench_setpoint", 1, &ModelControlCompute::WrenchSPCallBack, this);
    // measure
    state_subscriber =
            nh.subscribe("state", 1, &ModelControlCompute::MeasureCallBack, this);

    // deal with controlled axes
    const size_t n = _controlled_axes.size();
    //axes.resize(n);

    //const std::vector<std::string> axes3D{"x", "y", "z", "roll", "pitch", "yaw"};


    if(true/*n*/)//If we can actually control something --> That's what n meant, we need to replace it;
    {
        if(default_mode == "position"){
            // position setpoint
            position_sp_subscriber =
                    nh.subscribe("body_position_setpoint", 1, &ModelControlCompute::PositionSPCallBack, this);
        }
        else if(default_mode == "velocity"){
            // velocity setpoint
            velocity_sp_subscriber =
                    nh.subscribe("body_velocity_setpoint", 1, &ModelControlCompute::VelocitySPCallBack, this);
        }
        //TODO initialize setpoint (angular desired value in both cases);
    }

    // initialisation of the parameters (very bad now)
    param_estimated << 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1;
    // get whether or not we use dynamic reconfigure
    //TODO: Learn how to use it (Maybe change GetGains function)
    ros::NodeHandle control_node(nh, "controllers");
    bool use_dynamic_reconfig;
    control_node.param("config/body/dynamic_reconfigure", use_dynamic_reconfig, true);

    // initialisation of the gains
    this->GetGains(control_node);

}

bool ModelControlCompute::UpdateError()
{
    if(state_received)
    {
        if(setpoint_velocity_ok)
        {
            //std::stringstream ss2;ss2 << "Erreur en vitesse mis a jour";
            //Let's update the velocity_error_
            velocity_error_ = velocity_setpoint_ - velocity_measure_;
            //ROS_INFO("%s; %f %f %f %f %f %f",ss2.str().c_str(), velocity_error_[0],velocity_error_[1],velocity_error_[2],velocity_error_[3],velocity_error_[4],velocity_error_[5]);
            pose_error_ << 0, 0, 0, 0, 0, 0;
        }
        else if(setpoint_position_ok)
        {
            velocity_error_ << 0, 0, 0, 0, 0, 0;
            //Let's update the pose_error_
            Eigen::Quaterniond pose_ang_measure_ = pose_ang_measure_inv_.inverse();
            Eigen::Vector3d attitude_error_;

            attitude_error_ = pose_ang_measure_.w()*pose_ang_setpoint_.vec() - pose_ang_setpoint_.w()*pose_ang_measure_.vec()
                    + pose_ang_setpoint_.vec().cross( pose_ang_measure_.vec() );
            pose_error_ << pose_ang_measure_inv_.toRotationMatrix() * (pose_lin_setpoint_ - pose_lin_measure_) , attitude_error_;
        }else
        {
            return false;
        }

        //Let's update the s_error_
        Eigen::DiagonalMatrix<double, 6> Lambda;
        Lambda.diagonal() << lp, lp, lp, lo, lo, lo;
        s_error_ = velocity_error_ + Lambda * pose_error_;
        return true;
    }
    else
        return false;

}

void ModelControlCompute::UpdateParam()
{
    std::stringstream ss2;
    ss2 << "Parameters: ";
    param_prev = param_estimated;
    Eigen::Matrix<double,22,6> rt =  regressor.transpose();
    param_estimated = param_prev + dt.toSec()*KL*regressor.transpose()*s_error_;//TODO inverser KL
    std::cout << "Parameters: \n" << param_estimated << std::endl;
//    std::cout << "Parameters" << param_estimated(0)param_estimated(1),param_estimated(2),param_estimated(3),param_estimated(4),param_estimated(5),
//                                                                 param_estimated(6),param_estimated(7),param_estimated(8),param_estimated(9),param_estimated(10),param_estimated(11),
//                                                                 param_estimated(12),param_estimated(13),param_estimated(14),param_estimated(15),param_estimated(16),param_estimated(17),
//                                                                 param_estimated(18),param_estimated(19),param_estimated(20),param_estimated(21));
}

void ModelControlCompute::UpdateWrench()
{
    //Let's Calculate the regressor matrix
    Eigen::Vector6d v = velocity_measure_;

    Eigen::Matrix6d lin_dampling_regressor =  v.asDiagonal();

    Eigen::Matrix6d quad_dampling_regressor = v.array().square().matrix().asDiagonal();


    Eigen::Vector6d acc = (velocity_measure_ - vel_prev)/dt.toSec();
    vel_prev = velocity_measure_;
    v = velocity_measure_;
    Eigen::Matrix6d  added_effect_regressor;
    added_effect_regressor << acc(0), v(1)*v(5), -v(2)*v(4), 0, 0, 0,
            -v(0)*v(5), acc(1), v(2)*v(3), 0, 0, 0,
            v(0)*v(4), -v(1)*v(3), acc(2), 0, 0, 0,
            0, v(1)*v(2), -v(1)*v(2), acc(3), v(4)*v(5), -v(4)*v(5),
            -v(0)*v(2), 0, v(0)*v(2), -v(3)*v(5), acc(4), v(3)*v(5),
            v(0)*v(1), -v(0)*v(1), 0, v(3)*v(4), -v(3)*v(4), acc(5);

    Eigen::Matrix<double, 6,4> grav_regressor;
    Eigen::Vector3d e3(0.0 ,0.0 ,1.0);
    grav_regressor << pose_ang_measure_inv_.toRotationMatrix()*e3, Eigen::MatrixXd::Zero(3,3),
            Eigen::MatrixXd::Zero(3,1), skew(pose_ang_measure_inv_.toRotationMatrix()*e3) ;

    regressor << lin_dampling_regressor, quad_dampling_regressor, added_effect_regressor, grav_regressor;

    //Let's update the wrench
    Eigen::Vector6d wrench;
    Eigen::Vector6d wrench_pid;
    Eigen::Vector6d wrench_model;

    wrench_pid =  KD*s_error_+ K * pose_error_;
    wrench_model = regressor*param_estimated;
    wrench = KD*s_error_+ K * pose_error_ + regressor*param_estimated;

    tf::wrenchEigenToMsg(wrench,wrench_command_);
    //std::stringstream ss2;
    //ss2 << "Updating Wrench";
    //ROS_INFO("%s; %f %f %f %f %f %f",ss2.str().c_str(), wrench[0],wrench[1],wrench[2],wrench[3],wrench[4],wrench[5]);


}


void ModelControlCompute::GetGains(const ros::NodeHandle &control_node)
{
    //Updates directly the variables
    control_node.param("Lamba/l/lo",lo,0.0);
    control_node.param("Lamba/l/lp",lp,0.0);
    //lo et lp are used to create the Lambda matrix just before use

    control_node.param("K/k/ko",ko,0.0);
    control_node.param("K/k/kp",kp,0.0);
    //We have to call SetGainsK to update them

    double KD_d1,KD_d2;
    control_node.param("KD/diagonal/d1",KD_d1,0.0);
    control_node.param("KD/diagonal/d2",KD_d2,0.0);

    int n_blocks;
    control_node.param("KL/blocks",n_blocks,4);
    std::vector<double> KL_diag;
    for(int i = 1; i <= n_blocks; i ++)
    {
        double k;
        control_node.param("KL/diagonal/b"+std::to_string(i), k, 0.0);
        KL_diag.push_back(k);

    }

    SetGainsK();
    SetGainsKD(KD_d1,KD_d2);
    SetGainsKL(KL_diag);
}


void ModelControlCompute::PositionSPCallBack(const geometry_msgs::PoseStampedConstPtr& _msg)
{
    setpoint_position_ok = true;

    pose_lin_setpoint_ = Eigen::Vector3d(_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z);
    pose_ang_setpoint_ = Eigen::Quaterniond(_msg->pose.orientation.w, _msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z);
}

void ModelControlCompute::VelocitySPCallBack(const geometry_msgs::TwistStampedConstPtr & _msg)
{
    setpoint_velocity_ok = true;
    Eigen::Vector3d velocity_lin_setpoint_ = Eigen::Vector3d(_msg->twist.linear.x, _msg->twist.linear.y, _msg->twist.linear.z);
    Eigen::Vector3d velocity_ang_setpoint_ = Eigen::Vector3d(_msg->twist.angular.x, _msg->twist.angular.y, _msg->twist.angular.z);
    velocity_setpoint_ << velocity_lin_setpoint_, velocity_ang_setpoint_;
}

void ModelControlCompute::MeasureCallBack(const nav_msgs::OdometryConstPtr &_msg)
{
    state_received = true;
    // positions are expressed in the world frame, rotation is inversed
    pose_lin_measure_ = Eigen::Vector3d(_msg->pose.pose.position.x, _msg->pose.pose.position.y, _msg->pose.pose.position.z);
    pose_ang_measure_inv_ = Eigen::Quaterniond(_msg->pose.pose.orientation.w, _msg->pose.pose.orientation.x, _msg->pose.pose.orientation.y, _msg->pose.pose.orientation.z).inverse();

    // change velocities from world to body frame
    Eigen::Vector3d velocity_lin_measure_ = pose_ang_measure_inv_.toRotationMatrix()*Eigen::Vector3d(_msg->twist.twist.linear.x, _msg->twist.twist.linear.y, _msg->twist.twist.linear.z);
    Eigen::Vector3d velocity_ang_measure_ = pose_ang_measure_inv_.toRotationMatrix()*Eigen::Vector3d(_msg->twist.twist.angular.x, _msg->twist.twist.angular.y, _msg->twist.twist.angular.z);
    velocity_measure_ << velocity_lin_measure_, velocity_ang_measure_;// TODO be sure it can be done
}

}


