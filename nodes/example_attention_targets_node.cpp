#include <ros/ros.h>
#include <semio_msgs_ros/AttentionTargets.h>
#include <semio_msgs_ros/Humanoids.h>
#include <semio/recognition/attention_recognizer.h>
#include <semio/recognition/humanoid_source_virtual.h>

struct TestPose
{
    Eigen::Vector3d head_rotation;
    Eigen::Vector3d neck_rotation;
    Eigen::Vector3d torso_rotation;
    Eigen::Translation3d position;
};

class ExampleAttentionTargetsNode
{
protected:
    typedef semio_msgs_ros::AttentionTargets _AttentionTargetsMsg;
    typedef semio_msgs_ros::AttentionTarget _AttentionTargetMsg;
    typedef semio_msgs_ros::Humanoids _HumanoidsMsg;
    typedef semio_msgs_ros::Humanoid _HumanoidMsg;
    typedef semio_msgs_ros::HumanoidJoint _HumanoidJointMsg;

    ros::Publisher _attention_targets_pub;
    ros::Publisher _humanoids_pub;

    semio::HumanoidSource::Ptr _humanoid_source_ptr;

    semio::AttentionTargetArray _attention_targets;

    std::vector<TestPose> _test_poses;

public:
    ExampleAttentionTargetsNode( ros::NodeHandle & nh_rel, semio::HumanoidSource::Ptr humanoid_source_ptr )
    :
        _attention_targets_pub( nh_rel.advertise<_AttentionTargetsMsg>( "attention_targets", 10 ) ),
        _humanoids_pub( nh_rel.advertise<_HumanoidsMsg>( "humanoids", 10 ) ),
        _humanoid_source_ptr( humanoid_source_ptr )
    {
        _attention_targets.emplace( "center-low", Eigen::Translation3d( -1, 0, 0 ) );
        _attention_targets.emplace( "left", Eigen::Translation3d( 1, -3, 0 ) );
        _attention_targets.emplace( "right", Eigen::Translation3d( 1, 3, 0 ) );
        _attention_targets.emplace( "center", Eigen::Translation3d( -1, 0, 0.6858 ) );
        _attention_targets.emplace( "center-high", Eigen::Translation3d( -1, 0, 1 ) );

        _test_poses.push_back( std::move( TestPose{ Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 180 ), Eigen::Translation3d( 2, 0, 0 ) } ) );
        _test_poses.push_back( std::move( TestPose{ Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 180 + 45 ), Eigen::Translation3d( 2, 0, 0 ) } ) );
        _test_poses.push_back( std::move( TestPose{ Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 180 - 45 ), Eigen::Translation3d( 2, 0, 0 ) } ) );
        _test_poses.push_back( std::move( TestPose{ Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 180 + 90 ), Eigen::Translation3d( 2, 0, 0 ) } ) );
        _test_poses.push_back( std::move( TestPose{ Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 180 - 90 ), Eigen::Translation3d( 2, 0, 0 ) } ) );
        _test_poses.push_back( std::move( TestPose{ Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 180 ), Eigen::Translation3d( 2, -2, 0 ) } ) );
        _test_poses.push_back( std::move( TestPose{ Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 180 ), Eigen::Translation3d( 2, 2, 0 ) } ) );

        for( double const & torso_yaw : { 90, 45, 0, -45, -90 } )
        {
            for( double const & neck_yaw : { 45, 30, 15, 0, -15, -30, -45 } )
            {
                for( double const & head_yaw : { 45, 30, 15, 0, -15, -30, -45 } )
                {
                    _test_poses.push_back( std::move( TestPose{ Eigen::Vector3d( 0, 0, head_yaw ), Eigen::Vector3d( 0, 0, neck_yaw ), Eigen::Vector3d( 0, 0, 180 + torso_yaw ), Eigen::Translation3d( 1, 0, 0 ) } ) );
                }
            }
        }
    }

    void spin()
    {
        ros::Duration sleep_interval( 0.5 );
        size_t test_pose_idx = 1;
        for( auto const & test_pose : _test_poses )
        {
            if( !ros::ok() ) break;

            ros::spinOnce();

            std::cout << "pose " << test_pose_idx << "/" << _test_poses.size() << std::endl;

            // update virtual humanoid with new test position
            auto humanoid_source_virtual_ptr( std::dynamic_pointer_cast<semio::HumanoidSourceVirtual>( _humanoid_source_ptr ) );
            humanoid_source_virtual_ptr->setPosition( test_pose.position );
            humanoid_source_virtual_ptr->setOrientation(
                semio::HumanoidJoint::JointType::HEAD,
                Eigen::AngleAxisd( test_pose.head_rotation.x() * M_PI/180, Eigen::Vector3d::UnitX() ) *
                Eigen::AngleAxisd( test_pose.head_rotation.y() * M_PI/180, Eigen::Vector3d::UnitY() ) *
                Eigen::AngleAxisd( test_pose.head_rotation.z() * M_PI/180, Eigen::Vector3d::UnitZ() ) );
            humanoid_source_virtual_ptr->setOrientation(
                semio::HumanoidJoint::JointType::NECK,
                Eigen::AngleAxisd( test_pose.neck_rotation.x() * M_PI/180, Eigen::Vector3d::UnitX() ) *
                Eigen::AngleAxisd( test_pose.neck_rotation.y() * M_PI/180, Eigen::Vector3d::UnitY() ) *
                Eigen::AngleAxisd( test_pose.neck_rotation.z() * M_PI/180, Eigen::Vector3d::UnitZ() ) );
            humanoid_source_virtual_ptr->setOrientation(
                semio::HumanoidJoint::JointType::TORSO,
                Eigen::AngleAxisd( test_pose.torso_rotation.x() * M_PI/180, Eigen::Vector3d::UnitX() ) *
                Eigen::AngleAxisd( test_pose.torso_rotation.y() * M_PI/180, Eigen::Vector3d::UnitY() ) *
                Eigen::AngleAxisd( test_pose.torso_rotation.z() * M_PI/180, Eigen::Vector3d::UnitZ() ) );

            // get humanoids
            semio::HumanoidArray const & humanoids( _humanoid_source_ptr->update() );

            // publish humanoids
            _HumanoidsMsg humanoids_msg;

            humanoids_msg.humanoids.reserve( humanoids.size() );

            for( auto const & humanoid : humanoids )
            {
                auto & joints( humanoid.joints_ );

                _HumanoidMsg humanoid_msg;

                humanoid_msg.id = humanoid.id_;
                humanoid_msg.tracking_state = static_cast<uint32_t>( humanoid.tracking_state_ );
                humanoid_msg.joints.reserve( joints.size() );

                for( auto const & joint_item : joints )
                {
                    semio::HumanoidJoint const & joint( joint_item.second );

                    _HumanoidJointMsg joint_msg;

                    joint_msg.type = static_cast<size_t>( joint.joint_type_ );
                    joint_msg.position_confidence = joint.position_confidence_;
                    joint_msg.orientation_confidence = joint.orientation_confidence_;
                    joint_msg.position.x = joint.position_.x();
                    joint_msg.position.y = joint.position_.y();
                    joint_msg.position.z = joint.position_.z();
                    joint_msg.orientation.w = joint.orientation_.w();
                    joint_msg.orientation.x = joint.orientation_.x();
                    joint_msg.orientation.y = joint.orientation_.y();
                    joint_msg.orientation.z = joint.orientation_.z();

                    humanoid_msg.joints.push_back( std::move( joint_msg ) );
                }
                humanoids_msg.humanoids.push_back( std::move( humanoid_msg ) );
            }

            _humanoids_pub.publish( std::move( humanoids_msg ) );

            // publish attention targets
            _AttentionTargetsMsg attention_targets_msg;

            attention_targets_msg.targets.reserve( _attention_targets.size() );

            for( auto const & attention_target : _attention_targets )
            {
                _AttentionTargetMsg attention_target_msg;

                attention_target_msg.name = attention_target.name_;
                attention_target_msg.position.x = attention_target.position_.x();
                attention_target_msg.position.y = attention_target.position_.y();
                attention_target_msg.position.z = attention_target.position_.z();

                attention_targets_msg.targets.push_back( std::move( attention_target_msg ) );
            }

            _attention_targets_pub.publish( attention_targets_msg );

            sleep_interval.sleep();
            test_pose_idx++;
        }
    }
};

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "example_attention_targets_node" );
    ros::NodeHandle nh_rel( "~" );

    ExampleAttentionTargetsNode example_attention_targets_node( nh_rel, std::make_shared<semio::HumanoidSourceVirtual>() );
    example_attention_targets_node.spin();

    return 0;
}
