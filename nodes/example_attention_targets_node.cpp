#include <ros/ros.h>
#include <semio_msgs_ros/AttentionTargets.h>
#include <semio/recognition/attention_recognizer.h>
#include <semio/ros/humanoid_source_adapter.h>
#include <semio/ros/humanoid_sink_adapter.h>
#include <semio/recognition/humanoid_source_virtual.h>

//! Stores a partial pose (head/neck/torso rotations and the torso position) of a virtual humanoid
struct TestPose
{
    //! Rotation of the head joint for the test pose (relative to global frame)
    Eigen::Vector3d head_rotation;
    //! Rotation of the neck joint for the test pose (relative to global frame)
    Eigen::Vector3d neck_rotation;
    //! Rotation of the torso joint for the test pose (relative to global frame)
    Eigen::Vector3d torso_rotation;
    //! Position of the torso joint for the test pose (relative to global frame)
    Eigen::Vector3d position;
};

//! Example node demonstrating how to publish targets for attention recognition
class ExampleAttentionTargetsNode
{
protected:
    //! ROS message for a vector of attention targets
    typedef semio_msgs_ros::AttentionTargets _AttentionTargetsMsg;
    //! ROS message for a single attention target
    typedef semio_msgs_ros::AttentionTarget _AttentionTargetMsg;

    //! NodeHandle used to interface with ROS
    ros::NodeHandle & _nh_rel;
    //! Publisher for our attention targets
    ros::Publisher _attention_targets_pub;

    //! Pointer to the input source for humanoids
    semio::HumanoidSource::Ptr _humanoid_source_ptr;
    //! Pointer to the output sink for humanoids
    semio::HumanoidSink::Ptr _humanoid_sink_ptr;

    //! Whether the input source is a semio::HumanoidSourceVirtual
    bool _is_virtual_source;

    //! The list of attention targets to publish
    semio::AttentionTargetArray _attention_targets;

    //! The list of test poses to use if #_is_virtual_source == true
    std::vector<TestPose> _test_poses;

public:
    /**
    @param nh_rel @copybrief _nh_rel
    @param humanoid_source_ptr @copybrief _humanoid_source_ptr
    @param humanoid_sink_ptr @copybrief _humanoid_sink_ptr
    */
    ExampleAttentionTargetsNode( ros::NodeHandle & nh_rel, semio::HumanoidSource::Ptr humanoid_source_ptr, semio::HumanoidSink::Ptr humanoid_sink_ptr )
    :
        _nh_rel( nh_rel ),
        _attention_targets_pub( nh_rel.advertise<_AttentionTargetsMsg>( "attention_targets", 10 ) ),
        _humanoid_source_ptr( humanoid_source_ptr ),
        _humanoid_sink_ptr( humanoid_sink_ptr ),
        _is_virtual_source( std::dynamic_pointer_cast<semio::HumanoidSourceVirtual>( _humanoid_source_ptr ) )
    {
        size_t const cols( _nh_rel.param<int>( "cols", 13 ) );
        size_t const rows( _nh_rel.param<int>( "rows", 7 ) );
        double const horizontal_spacing( _nh_rel.param<double>( "h_spacing", 15 ) );
        double const vertical_spacing( _nh_rel.param<double>( "v_spacing", 15 ) );
        double const radius( _nh_rel.param<double>( "radius", 2 ) );

        std::cout << "using " << cols << "x" << rows << " @ " << horizontal_spacing << "x" << vertical_spacing << std::endl;
        Eigen::Vector3d const center( radius, 0, 0 );

        double const yaw_range( horizontal_spacing * static_cast<double>( cols - 1 ) / 2.0 );
        double const pitch_range( vertical_spacing * static_cast<double>( rows - 1 ) / 2.0 );

        for( size_t col = 0; col < cols; ++col )
        {
            for( size_t row = 0; row < rows; ++row )
            {
                double const yaw( yaw_range - static_cast<double>( col ) * horizontal_spacing );
                double const pitch( -pitch_range + static_cast<double>( row ) * vertical_spacing );

                std::stringstream name_stream;
                name_stream << ( col * rows + row );
                _attention_targets.emplace(
                    name_stream.str(),
                    (
                        Eigen::Affine3d(
                            Eigen::Translation3d( center ) *
                            Eigen::Quaterniond( Eigen::AngleAxisd( M_PI, Eigen::Vector3d::UnitZ() ) )
                        ) *
                        Eigen::Affine3d(
                            Eigen::Quaterniond(
                                Eigen::AngleAxisd( yaw * M_PI/180, Eigen::Vector3d::UnitZ() ) *
                                Eigen::AngleAxisd( pitch * M_PI/180, Eigen::Vector3d::UnitY() )
                            ) *
                            Eigen::Translation3d( radius, 0, 0 )
                        )
                    ).translation()
                );
            }
        }

        if( _is_virtual_source )
        {
            _test_poses.push_back( std::move( TestPose{ Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 180 ), Eigen::Vector3d( 2, 0, 0 ) } ) );
            _test_poses.push_back( std::move( TestPose{ Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 180 + 45 ), Eigen::Vector3d( 2, 0, 0 ) } ) );
            _test_poses.push_back( std::move( TestPose{ Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 180 - 45 ), Eigen::Vector3d( 2, 0, 0 ) } ) );
            _test_poses.push_back( std::move( TestPose{ Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 180 + 90 ), Eigen::Vector3d( 2, 0, 0 ) } ) );
            _test_poses.push_back( std::move( TestPose{ Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 180 - 90 ), Eigen::Vector3d( 2, 0, 0 ) } ) );
            _test_poses.push_back( std::move( TestPose{ Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 180 ), Eigen::Vector3d( 2, -2, 0 ) } ) );
            _test_poses.push_back( std::move( TestPose{ Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 180 ), Eigen::Vector3d( 2, 2, 0 ) } ) );

            for( double const & torso_yaw : { 90, 45, 0, -45, -90 } )
            {
                for( double const & neck_yaw : { 45, 30, 15, 0, -15, -30, -45 } )
                {
                    for( double const & head_yaw : { 45, 30, 15, 0, -15, -30, -45 } )
                    {
                        _test_poses.push_back( std::move( TestPose{ Eigen::Vector3d( 0, 0, head_yaw ), Eigen::Vector3d( 0, 0, neck_yaw ), Eigen::Vector3d( 0, 0, 180 + torso_yaw ), Eigen::Vector3d( 1, 0, 0 ) } ) );
                    }
                }
            }
        }
    }

    //! Publish all relevant data
    void publishData()
    {
        //! - Publish humanoids from our input source to our output sink
        _humanoid_sink_ptr->publish( _humanoid_source_ptr->update() );

        //----------
        //! - Convert attention targets to ROS message
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
        //----------

        //! - Publish attention targets
        _attention_targets_pub.publish( attention_targets_msg );
    }

    //! Main loop
    /**
    - If #_is_virtual_source == true
      - Update our humanoid source with the next test pose
    - publishData()
    */
    void spin()
    {
        if( _is_virtual_source )
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

                publishData();

                sleep_interval.sleep();
                test_pose_idx++;
            }
        }
        else
        {
            ros::Rate loop_rate( 30 );

            while( ros::ok() )
            {
                ros::spinOnce();

                publishData();

                loop_rate.sleep();
            }
        }
    }
};

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "example_attention_targets_node" );
    //! - Create NodeHandle with relative namespace
    ros::NodeHandle nh_rel( "~" );

    //! - Create semio::ros::HumanoidSourceAdapter; default to virtual source
    semio::ros::HumanoidSourceAdapter humanoid_source_adapter( nh_rel, "virtual" );
    //! - Create semio::ros::HumanoidSinkAdapter; default to ROS source
    semio::ros::HumanoidSinkAdapter humanoid_sink_adapter( nh_rel, "ros" );

    //! - Create ExampleAttentionTargetsNode; pass node handle, humanoid source, and humanoid sink
    ExampleAttentionTargetsNode example_attention_targets_node( nh_rel, humanoid_source_adapter.getHumanoidSource(), humanoid_sink_adapter.getHumanoidSink() );
    //! - Start main loop ExampleAttentionTargetsNode::spin()
    example_attention_targets_node.spin();

    return 0;
}
