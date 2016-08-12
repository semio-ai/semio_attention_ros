#include <ros/ros.h>

#include <semio_msgs_ros/AttentionRecognitionResult.h>
#include <semio_msgs_ros/AttentionTargets.h>

#include <semio/recognition/attention_recognizer.h>
#include <semio/ros/humanoid_source_adapter.h>

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

    semio::HumanoidSource::Ptr humanoid_source_ptr_;
    semio::AttentionRecognizer attention_recognizer_;

    SemioAttentionNode( ros::NodeHandle & nh_rel, semio::HumanoidSource::Ptr humanoid_source_ptr )
    :
        nh_rel_( nh_rel ),
        result_pub_( nh_rel_.advertise<_AttentionRecognitionResultMsg>( "result", 1000 ) ),
        targets_sub_( nh_rel_.subscribe( "targets", 10, &SemioAttentionNode::targetsCB, this ) ),
        humanoid_source_ptr_( humanoid_source_ptr )
    {
        //
    }

    void spin()
    {
        ros::Rate loop_rate( 30 );

        while( ros::ok() )
        {
            ros::spinOnce();

            attention_recognizer_.getHumanoids() = humanoid_source_ptr_->update();
            semio::AttentionRecognitionResult const & result = attention_recognizer_.calculateResult();

            _AttentionRecognitionResultMsg result_msg;

            result_msg.humanoids.reserve( result.size() );

            for( auto const & humanoid_item : result )
            {
                semio::AttentionSourceMap const & joints( humanoid_item.second );

                _AttentionRecognitionHumanoidItemMsg humanoid_msg;
                humanoid_msg.id = static_cast<uint32_t>( humanoid_item.first );
                humanoid_msg.joints.reserve( joints.size() );

                for( auto const & joint_item : joints )
                {
                    semio::AttentionTopNList const & top_n_list( joint_item.second );

                    _AttentionRecognitionJointItemMsg joint_msg;
                    joint_msg.id = static_cast<uint32_t>( joint_item.first );
                    joint_msg.top_n_list.reserve( top_n_list.size() );

                    for( auto const & list_item : top_n_list )
                    {
                        _AttentionRecognitionTopNItemMsg top_n_item_msg;
                        top_n_item_msg.likelihood = list_item.value_;
                        top_n_item_msg.target_name = list_item.data_;

                        joint_msg.top_n_list.push_back( std::move( top_n_item_msg ) );
                    }

                    humanoid_msg.joints.push_back( std::move( joint_msg ) );
                }

                result_msg.humanoids.push_back( std::move( humanoid_msg ) );
            }

            result_pub_.publish( std::move( result_msg ) );

            loop_rate.sleep();
        }
    }

    void targetsCB( _AttentionTargetsMsg::ConstPtr const & msg_ptr )
    {
        auto const & targets_msg = *msg_ptr;

        semio::AttentionTargetArray attention_targets;

        for( auto const & target_msg : targets_msg.targets )
        {
            auto const & name = target_msg.name;
            auto const & position = target_msg.position;

            attention_targets.emplace( name, Eigen::Translation3d( position.x, position.y, position.z ) );
        }

        attention_recognizer_.getTargets() = attention_targets;
    }
};

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "semio_attention_node" );
    ros::NodeHandle nh_rel( "~" );

    semio::ros::HumanoidSourceAdapter humanoid_source_adapter( nh_rel );

    SemioAttentionNode semio_attention_node( nh_rel, humanoid_source_adapter.getHumanoidSource() );
    semio_attention_node.spin();

    return 0;
}
