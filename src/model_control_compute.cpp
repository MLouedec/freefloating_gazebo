#include <freefloating_gazebo/model_control_compute.h>
#include <freefloating_gazebo/hydro_link.h>

namespace ffg
{


void ModelControlCompute::Init(ros::NodeHandle &nh, ros::Duration&_dt, const std::vector<std::string>&_controlled_axes, std::string default_mode/* = "position"*/)
{
    velocity_error_ << 0,0,0,0,0,0;
    s_error_ << 0,0,0,0,0,0;

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

    // get whether or not we use dynamic reconfigure
    //bool use_dynamic_reconfig;
    ros::NodeHandle control_node(nh, "controllers");
    //control_node.param("controllers/config/body/dynamic_reconfigure", use_dynamic_reconfig, true);

    UpdateGains(control_node);//Get gains from parameters

    if(n)//If we can actually control something
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

    // initialisation of the gains
    this->UpdateGains(nh);

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
    param_prev = param_estimated;
    //double  dt = dt.toSec();
    Eigen::Matrix<double,22,6> rt =  regressor.transpose();
    param_estimated = param_prev + dt.toSec()*KL*regressor.transpose()*s_error_;//TODO inverser KL
}

void ModelControlCompute::UpdateWrench()
{
    //Let's Calculate the regressor matrix
    std::stringstream ss2;
    ss2 << "Updating Wrench";
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
    ROS_INFO("%s; %f %f %f %f %f %f",ss2.str().c_str(), wrench[0],wrench[1],wrench[2],wrench[3],wrench[4],wrench[5]);


}


void ModelControlCompute::UpdateGains(const ros::NodeHandle &control_node)
{
    control_node.param("Lamba/l/lo",lo,0.0);
    control_node.param("Lamba/l/lp",lp,0.0);

    control_node.param("K/k/ko",ko,0.0);
    control_node.param("K/k/kp",kp,0.0);

    std::vector<double> KD_diag;
    if( !control_node.getParam("KD/diagonal",KD_diag) ){
        ROS_ERROR("Failed to get KD from server");
    }

    std::vector<double> KL_diag;
    if( !control_node.getParam("KL/diagonal",KL_diag) ){
        ROS_ERROR("Failed to get KL from server");
    }

    K.diagonal() << kp, kp, kp, ko, ko, ko;

    KD.diagonal() <<    KD_diag[0], KD_diag[1], KD_diag[2], KD_diag[3], KD_diag[4], KD_diag[5];

    KL.diagonal() <<    KL_diag[0], KL_diag[1], KL_diag[2], KL_diag[3], KL_diag[4],KL_diag[5], KL_diag[6], KL_diag[7], KL_diag[8], KL_diag[9],KL_diag[10], KL_diag[11],
                        KL_diag[12], KL_diag[13], KL_diag[14],KL_diag[15], KL_diag[16], KL_diag[17], KL_diag[18], KL_diag[19],KL_diag[20], KL_diag[21];
    //= 0.0000001*Eigen::Matrix<double, 22, 22>::Identity();

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


