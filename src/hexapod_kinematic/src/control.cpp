
// ROS Hexapod Controller Node
// Copyright (c) 2016, Kevin M. Ochs
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the Kevin Ochs nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL KEVIN OCHS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Author: Kevin M. Ochs


#include <control.h>

static const double PI = atan(1.0)*4.0;

//==============================================================================
// Constructor
//==============================================================================

Control::Control( void )
{
    ros::param::get( "NAME_SPACE", NAME_SPACE);
    ros::param::get( "NUMBER_OF_LEGS", NUMBER_OF_LEGS );
    ros::param::get( "NUMBER_OF_LEG_SEGMENTS", NUMBER_OF_LEG_JOINTS );
    ros::param::get( "NUMBER_OF_HEAD_SEGMENTS", NUMBER_OF_HEAD_JOINTS );
    ros::param::get( "BODY_MAX_ROLL", BODY_MAX_ROLL );
    ros::param::get( "BODY_MAX_PITCH", BODY_MAX_PITCH );
    ros::param::get( "BODY_MAX_YAW", BODY_MAX_YAW );
    ros::param::get( "STANDING_BODY_HEIGHT", STANDING_BODY_HEIGHT );    
    ros::param::get( "MAX_BODY_ROLL_COMP", MAX_BODY_ROLL_COMP );
    ros::param::get( "MAX_BODY_PITCH_COMP", MAX_BODY_PITCH_COMP );
    ros::param::get( "COMPENSATE_INCREMENT", COMPENSATE_INCREMENT );
    ros::param::get( "COMPENSATE_TO_WITHIN", COMPENSATE_TO_WITHIN );
    ros::param::get( "MASTER_LOOP_RATE", MASTER_LOOP_RATE );
    ros::param::get( "VELOCITY_DIVISION", VELOCITY_DIVISION );
    current_time_odometry_ = ros::Time::now();
    last_time_odometry_ = ros::Time::now();
    current_time_cmd_vel_ = ros::Time::now();
    last_time_cmd_vel_ = ros::Time::now();
    // Find out how many servos/joints we have
    for( int i = 0; i < 18; i++ )
    {
        ros::param::get( ("/JOINTS/" + joint_map_key_array_[i] + "/name"), joint_names_array_[i] );
        ros::param::get( ("/JOINTS/" + joint_map_key_array_[i] + "/sign"), joint_orientation_array_[i] );
    }

    for (int i = 0; i < 18; i++) {
        // std::cout << joint_names_array_[i] << std::endl;     
        ROS_INFO_STREAM("Joint_name["<<i<<"]: "<<joint_names_array_[i]<<"");
    }

    prev_hex_state_ = false;
    hex_state_ = false;
    imu_init_stored_ = false;
    imu_roll_lowpass_ = 0.0;
    imu_pitch_lowpass_ = 0.0;
    imu_yaw_lowpass_ = 0.0;
    imu_roll_init_ = 0.0;
    imu_pitch_init_ = 0.0;

    // Topics we are subscribing
    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>( "/hexa_haibot/cmd_vel", 1, &Control::cmd_velCallback, this );
    body_scalar_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>( "/body_scalar", 1, &Control::bodyCallback, this );
    state_sub_ = nh_.subscribe<std_msgs::Bool>( "/state", 1, &Control::stateCallback, this );
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>( "/imu/data", 1, &Control::imuCallback, this );

    // Topics we are publishing
    sounds_pub_ = nh_.advertise<hexapod_msgs::Sounds>( "/sounds", 10 );
    // joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>( "/hexa_haibot/joint_states", 10 );
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>( "hexa_haibot/odometry/calculated", 50 );
    twist_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>( "/twist", 50 );

    jpc_1 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[0] + "_position_controller/command", 10);
    jpc_2 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[1] + "_position_controller/command", 10);
    jpc_3 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[2] + "_position_controller/command", 10);
    jpc_4 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[3] + "_position_controller/command", 10);
    jpc_5 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[4] + "_position_controller/command", 10);
    jpc_6 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[5] + "_position_controller/command", 10);
    jpc_7 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[6] + "_position_controller/command", 10);
    jpc_8 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[7] + "_position_controller/command", 10);
    jpc_9 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[8] + "_position_controller/command", 10);
    jpc_10 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[9] + "_position_controller/command", 10);
    jpc_11 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[10] + "_position_controller/command", 10);
    jpc_12 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[11] + "_position_controller/command", 10);
    jpc_13 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[12] + "_position_controller/command", 10);
    jpc_14 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[13] + "_position_controller/command", 10);
    jpc_15 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[14] + "_position_controller/command", 10);
    jpc_16 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[15] + "_position_controller/command", 10);
    jpc_17 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[16] + "_position_controller/command", 10);
    jpc_18 = nh_.advertise<std_msgs::Float64>( NAME_SPACE + joint_names_array_[17] + "_position_controller/command", 10);

    // Send service request to the imu to re-calibrate
    // imu_calibrate_ = nh_.serviceClient<std_srvs::Empty>("/imu/calibrate");
    // imu_calibrate_.call( calibrate_ );
}

//==============================================================================
// Getter and Setters
//==============================================================================

void Control::setHexActiveState( bool state )
{
    hex_state_ = state;
}

bool Control::getHexActiveState( void )
{
    return hex_state_;
}

void Control::setPrevHexActiveState( bool state )
{
    prev_hex_state_ = state;
}

bool Control::getPrevHexActiveState( void )
{
    return prev_hex_state_;
}

//==============================================================================
// Odometry Publisher
//==============================================================================
void Control::publishOdometry( const geometry_msgs::Twist &gait_vel )
{
    // compute odometry in a typical way given the velocities of the robot

    // calculate time elapsed
    current_time_odometry_ = ros::Time::now();
    double dt = ( current_time_odometry_ - last_time_odometry_ ).toSec();

    double vth = gait_vel.angular.z;
    double delta_th = vth * dt;
    pose_th_ += delta_th;

    double vx = gait_vel.linear.x;
    double vy = gait_vel.linear.y;
    double delta_x = ( vx * cos( pose_th_ ) - vy * sin( pose_th_ ) ) * dt;
    double delta_y = ( vx * sin( pose_th_ ) + vy * cos( pose_th_ ) ) * dt;
    pose_x_ += delta_x;
    pose_y_ += delta_y;

    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( pose_th_ );

    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_odometry_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = pose_x_;
    odom_trans.transform.translation.y = pose_y_;
    odom_trans.transform.translation.z = body_.position.z;
    odom_trans.transform.rotation = odom_quat;

    // Uncomment odom_broadcaster to send the transform. Only used if debugging calculated odometry.
    // odom_broadcaster.sendTransform( odom_trans );

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_odometry_;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // set the position
    odom.pose.pose.position.x = pose_x_;
    odom.pose.pose.position.y = pose_y_;
    odom.pose.pose.position.z = body_.position.z;
    odom.pose.pose.orientation = odom_quat;

    odom.pose.covariance[0] = 0.00001;  // x
    odom.pose.covariance[7] = 0.00001;  // y
    odom.pose.covariance[14] = 0.00001; // z
    odom.pose.covariance[21] = 1000000000000.0; // rot x
    odom.pose.covariance[28] = 1000000000000.0; // rot y
    odom.pose.covariance[35] = 0.001; // rot z

    // set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    odom.twist.covariance = odom.pose.covariance; // needed?

    odom_pub_.publish( odom );
    last_time_odometry_ = current_time_odometry_;
}

//==============================================================================
// Twist Publisher
//==============================================================================
void Control::publishTwist( const geometry_msgs::Twist &gait_vel )
{
    geometry_msgs::TwistWithCovarianceStamped twistStamped;
    twistStamped.header.stamp = ros::Time::now();
    twistStamped.header.frame_id = "odom";

    twistStamped.twist.twist.linear.x = gait_vel.linear.x;
    twistStamped.twist.twist.linear.y = gait_vel.linear.y;
    twistStamped.twist.twist.angular.z = gait_vel.angular.z;

    twistStamped.twist.covariance[0] = 0.00001;  // x
    twistStamped.twist.covariance[7] = 0.00001;  // y
    twistStamped.twist.covariance[14] = 0.00001; // z
    twistStamped.twist.covariance[21] = 1000000000000.0; // rot x
    twistStamped.twist.covariance[28] = 1000000000000.0; // rot y
    twistStamped.twist.covariance[35] = 0.001; // rot z

    twist_pub_.publish( twistStamped );
}


void Control::publishJointCommand( const hexapod_msgs::LegsJoints &legs)
{
    std_msgs::Float64 joint_cmd_0;
    joint_cmd_0.data = (joint_orientation_array_[0] * legs.leg[0].coxa);
    jpc_1.publish( joint_cmd_0 );

    std_msgs::Float64 joint_cmd_1;
    joint_cmd_1.data = (joint_orientation_array_[1] * legs.leg[0].femur);
    jpc_2.publish( joint_cmd_1 );

    std_msgs::Float64 joint_cmd_2;
    joint_cmd_2.data = (joint_orientation_array_[2] * legs.leg[0].tibia);
    jpc_3.publish( joint_cmd_2 );

    std_msgs::Float64 joint_cmd_3;
    joint_cmd_3.data = (joint_orientation_array_[3] * legs.leg[1].coxa);
    jpc_4.publish( joint_cmd_3 );

    std_msgs::Float64 joint_cmd_4;
    joint_cmd_4.data = (joint_orientation_array_[4] * legs.leg[1].femur);
    jpc_5.publish( joint_cmd_4 );

    std_msgs::Float64 joint_cmd_5;
    joint_cmd_5.data = (joint_orientation_array_[5] * legs.leg[1].tibia);
    jpc_6.publish( joint_cmd_5 );

    std_msgs::Float64 joint_cmd_6;
    joint_cmd_6.data = (joint_orientation_array_[6] * legs.leg[2].coxa);
    jpc_7.publish( joint_cmd_6 );

    std_msgs::Float64 joint_cmd_7;
    joint_cmd_7.data = (joint_orientation_array_[7] * legs.leg[2].femur);
    jpc_8.publish( joint_cmd_7 );

    std_msgs::Float64 joint_cmd_8;
    joint_cmd_8.data = (joint_orientation_array_[8] * legs.leg[2].tibia);
    jpc_9.publish( joint_cmd_8 );

    std_msgs::Float64 joint_cmd_9;
    joint_cmd_9.data = (joint_orientation_array_[9] * legs.leg[3].coxa);
    jpc_10.publish( joint_cmd_9 );

    std_msgs::Float64 joint_cmd_10;
    joint_cmd_10.data = (joint_orientation_array_[10] * legs.leg[3].femur);
    jpc_11.publish( joint_cmd_10 );

    std_msgs::Float64 joint_cmd_11;
    joint_cmd_11.data = (joint_orientation_array_[11] * legs.leg[3].tibia);
    jpc_12.publish( joint_cmd_11 );

    std_msgs::Float64 joint_cmd_12;
    joint_cmd_12.data = (joint_orientation_array_[12] * legs.leg[4].coxa);
    jpc_13.publish( joint_cmd_12 );

    std_msgs::Float64 joint_cmd_13;
    joint_cmd_13.data = (joint_orientation_array_[13] * legs.leg[4].femur);
    jpc_14.publish( joint_cmd_13 );

    std_msgs::Float64 joint_cmd_14;
    joint_cmd_14.data = (joint_orientation_array_[14] * legs.leg[4].tibia);
    jpc_15.publish( joint_cmd_14 );

    std_msgs::Float64 joint_cmd_15;
    joint_cmd_15.data = (joint_orientation_array_[15] * legs.leg[5].coxa);
    jpc_16.publish( joint_cmd_15 );

    std_msgs::Float64 joint_cmd_16;
    joint_cmd_16.data = (joint_orientation_array_[16] * legs.leg[5].femur);
    jpc_17.publish( joint_cmd_16 );

    std_msgs::Float64 joint_cmd_17;
    joint_cmd_17.data = (joint_orientation_array_[17] * legs.leg[5].tibia);
    jpc_18.publish( joint_cmd_17 );
}



//==============================================================================
// Topics we subscribe to
//==============================================================================
//==============================================================================
// cmd_vel callback
//==============================================================================

void Control::cmd_velCallback( const geometry_msgs::TwistConstPtr &cmd_vel_msg )
{
    cmd_vel_incoming_.linear.x = cmd_vel_msg->linear.x;
    cmd_vel_incoming_.linear.y = cmd_vel_msg->linear.y;
    cmd_vel_incoming_.angular.z = cmd_vel_msg->angular.z;
}

//==============================================================================
// Override IMU and manipulate body orientation callback
//==============================================================================

void Control::bodyCallback( const geometry_msgs::TwistStampedConstPtr &body_scalar_msg )
{
    ros::Time current_time = ros::Time::now();
    double time_delta = current_time.toSec() - body_scalar_msg->header.stamp.toSec();
    if ( time_delta < 1.0 ) // Don't move if timestamp is stale over a second
    {
        // To prevent violent motion changes the values are ran through a low pass filter
        body_.orientation.roll = ( body_scalar_msg->twist.angular.x * BODY_MAX_ROLL )* 0.01 + ( body_.orientation.roll * ( 1.0 - 0.01 ) );
        body_.orientation.pitch = ( body_scalar_msg->twist.angular.y * BODY_MAX_PITCH ) * 0.01 + ( body_.orientation.pitch * ( 1.0 - 0.01 ) );
        body_.orientation.yaw = ( body_scalar_msg->twist.angular.z * BODY_MAX_YAW ) * 0.01 + ( body_.orientation.yaw * ( 1.0 - 0.01 ) );
    }
}


//==============================================================================
// Active state callback - currently simple on/off - stand/sit
//==============================================================================

void Control::stateCallback( const std_msgs::BoolConstPtr &state_msg )
{
    if(state_msg->data == true )
    {
        if( getHexActiveState() == false )
        {
            // Activating hexapod
            body_.position.y = 0.0;
            body_.position.z = 0.0;
            body_.position.x = 0.0;
            body_.orientation.pitch = 0.0;
            body_.orientation.yaw = 0.0;
            body_.orientation.roll = 0.0;
            setHexActiveState( true );
            sounds_.stand = true;
            sounds_pub_.publish( sounds_ );
            sounds_.stand = false;
        }
    }

    if( state_msg->data == false )
    {
        if( getHexActiveState() == true )
        {
            // Sit down hexapod
            body_.position.y = 0.0;
            body_.position.x = 0.0;
            body_.orientation.pitch = 0.0;
            body_.orientation.yaw = 0.0;
            body_.orientation.roll = 0.0;
            setHexActiveState( false );
            sounds_.shut_down = true;
            sounds_pub_.publish( sounds_ );
            sounds_.shut_down = false;
        }
    }
}


//==============================================================================
// IMU callback to auto-level body if on non level ground
//==============================================================================

void Control::imuCallback( const sensor_msgs::ImuConstPtr &imu_msg )
{
        const geometry_msgs::Vector3 &lin_acc = imu_msg->linear_acceleration;

        if( imu_init_stored_ == false )
        {
            imu_roll_init_ = atan2( lin_acc.x, sqrt( lin_acc.y * lin_acc.y + lin_acc.z * lin_acc.z ) );
            imu_pitch_init_ = -atan2( lin_acc.y, lin_acc.z );
            imu_pitch_init_ = ( imu_pitch_init_ >= 0.0 ) ? ( PI - imu_pitch_init_ ) : ( -imu_pitch_init_ - PI );
            imu_init_stored_ = true;
        }

        // low-pass filter to smooth out noise
        imu_roll_lowpass_ = lin_acc.x * 0.01 + ( imu_roll_lowpass_ * ( 1.0 - 0.01 ) );
        imu_pitch_lowpass_ = lin_acc.y * 0.01 + ( imu_pitch_lowpass_ * ( 1.0 - 0.01 ) );
        imu_yaw_lowpass_ = lin_acc.z * 0.01 + ( imu_yaw_lowpass_ * ( 1.0 - 0.01 ) );

        double imu_roll = atan2( imu_roll_lowpass_, sqrt( imu_pitch_lowpass_ * imu_pitch_lowpass_ + imu_yaw_lowpass_ * imu_yaw_lowpass_ ) );
        double imu_pitch = -atan2( imu_pitch_lowpass_, imu_yaw_lowpass_ );
        imu_pitch = ( imu_pitch >= 0.0 ) ? ( PI - imu_pitch ) : ( -imu_pitch - PI );

        double imu_roll_delta = imu_roll_init_ - imu_roll;
        double imu_pitch_delta = imu_pitch_init_ - imu_pitch;

        if( ( std::abs( imu_roll_delta ) > MAX_BODY_ROLL_COMP ) || ( std::abs( imu_pitch_delta ) > MAX_BODY_PITCH_COMP ) )
        {
            sounds_.auto_level = true;
            sounds_pub_.publish( sounds_ );
            sounds_.auto_level = false;
        }

        if( imu_roll_delta < -COMPENSATE_TO_WITHIN )
        {
            if( body_.orientation.roll < MAX_BODY_ROLL_COMP )
            {
                //body_.orientation.roll = body_.orientation.roll + COMPENSATE_INCREMENT;
            }
        }

        if( imu_roll_delta > COMPENSATE_TO_WITHIN )
        {
            if( body_.orientation.roll > -MAX_BODY_ROLL_COMP )
            {
                //body_.orientation.roll = body_.orientation.roll - COMPENSATE_INCREMENT;
            }
        }

        if( imu_pitch_delta < -COMPENSATE_TO_WITHIN )
        {
            if( body_.orientation.pitch < MAX_BODY_PITCH_COMP )
            {
                //body_.orientation.pitch = body_.orientation.pitch + COMPENSATE_INCREMENT;
            }
        }

        if( imu_pitch_delta > COMPENSATE_TO_WITHIN )
        {
            if( body_.orientation.pitch > -MAX_BODY_PITCH_COMP )
            {
                //body_.orientation.pitch = body_.orientation.pitch - COMPENSATE_INCREMENT;
            }
        }
}

//==============================================================================
// Partitions up the cmd_vel to the speed of the loop rate
//==============================================================================

void Control::partitionCmd_vel( geometry_msgs::Twist *cmd_vel )
{
    // Instead of getting delta time we are calculating with a static division
    double dt = VELOCITY_DIVISION;
                                                                                                                            //      x
    double delta_th = cmd_vel_incoming_.angular.z * dt;                                                                     //      |
    double delta_x = ( cmd_vel_incoming_.linear.x * cos( delta_th ) + cmd_vel_incoming_.linear.y * sin( delta_th ) ) * dt;  //  y---origin
    double delta_y = ( cmd_vel_incoming_.linear.x * sin( delta_th ) - cmd_vel_incoming_.linear.y * cos( delta_th ) ) * dt;  //
    cmd_vel->linear.x = delta_x;
    cmd_vel->linear.y = delta_y;
    cmd_vel->angular.z = delta_th;
}

