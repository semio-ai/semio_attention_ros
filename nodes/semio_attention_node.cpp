#include <iostream>

#include <ros/ros.h>

#include <semio_msgs_ros/AttentionRecognitionResult.h>
#include <semio_msgs_ros/AttentionTargets.h>

#include <semio/recognition/attention_recognizer.h>
#include <semio/recognition/humanoid_source_NiTE.h>

class SemioAttentionNode
{
public:
    typedef semio_msgs_ros::AttentionRecognitionResult _AttentionRecognitionResultMsg;
    typedef semio_msgs_ros::AttentionRecognitionHumanoidItem _AttentionRecognitionHumanoidItemMsg;
    typedef semio_msgs_ros::AttentionRecognitionJointItem _AttentionRecognitionJointItemMsg;
    typedef semio_msgs_ros::AttentionRecognitionTopNItem _AttentionRecognitionTopNItemMsg;
    typedef semio_msgs_ros::AttentionTargets _AttentionTargetsMsg;
    typedef semio_msgs_ros::AttentionTarget _AttentionTargetMsg;

    ros::NodeHandle nh_rel_;
    ros::Publisher result_pub_;
    ros::Subscriber targets_sub_;

    semio::AttentionRecognizer attention_recognizer_;
    semio::HumanoidSourceNiTE humanoid_source_;

    SemioAttentionNode( ros::NodeHandle & nh_rel )
    :
        nh_rel_( nh_rel ),
        result_pub_( nh_rel_.advertise<_AttentionRecognitionResultMsg>( "result", 10 ) ),
        targets_sub_( nh_rel_.subscribe( "targets", 10, &SemioAttentionNode::targetsCB, this ) )
    {
        //
    }

    void spin()
    {
        ros::Rate loop_rate( 30 );

        while( ros::ok() )
        {
            attention_recognizer_.getHumanoids() = humanoid_source_.update();
            semio::AttentionRecognitionResult const & result = attention_recognizer_.calculateResult();

            _AttentionRecognitionResultMsg result_msg;

            result_msg.humanoids.reserve( result.size() );

            for( auto humanoids_it = result.cbegin(); humanoids_it != result.cend(); ++humanoids_it )
            {
                std::map<semio::HumanoidJoint::JointType, semio::TopNList<std::string> > const & joints = humanoids_it->second;

                _AttentionRecognitionHumanoidItemMsg humanoid_msg;
                humanoid_msg.id = static_cast<uint32_t>( humanoids_it->first );
                humanoid_msg.joints.reserve( joints.size() );

                for( auto joints_it = joints.cbegin(); joints_it != joints.cend(); ++joints_it )
                {
                    semio::TopNList<std::string> const & top_n_list = joints_it->second;

                    _AttentionRecognitionJointItemMsg joint_msg;
                    joint_msg.id = static_cast<uint32_t>( joints_it->first );
                    joint_msg.top_n_list.reserve( top_n_list.size() );

                    for( auto list_it = top_n_list.cbegin(); list_it != top_n_list.cend(); ++list_it )
                    {
                        _AttentionRecognitionTopNItemMsg top_n_item_msg;
                        top_n_item_msg.likelihood = list_it->value_;
                        top_n_item_msg.target_name = list_it->data_;

                        joint_msg.top_n_list.emplace_back( std::move( top_n_item_msg ) );
                    }

                    humanoid_msg.joints.emplace_back( std::move( joint_msg ) );
                }

                result_msg.humanoids.emplace_back( std::move( humanoid_msg ) );
            }

            result_pub_.publish( result_msg );

            loop_rate.sleep();
        }
    }

    void targetsCB( _AttentionTargetsMsg::ConstPtr const & msg_ptr )
    {
        auto const & targets_msg = *msg_ptr;

        semio::AttentionTargetArray attention_targets;

        for( auto msg_it = targets_msg.targets.cbegin(); msg_it != targets_msg.targets.cend(); ++msg_it )
        {
            auto const & position = msg_it->position;
            auto const & name = msg_it->name;

            attention_targets.emplace( semio::AttentionTarget( name, Eigen::Translation3d( position.x, position.y, position.z ) ) );
        }

        attention_recognizer_.getTargets() = attention_targets;
    }
};

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "semio_attention_node" );
    ros::NodeHandle nh_rel( "~" );

    SemioAttentionNode semio_attention_node( nh_rel );
    semio_attention_node.spin();

    return 0;
}
