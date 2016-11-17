#include <ros/ros.h>

#include <semio_msgs_ros/AttentionRecognitionResult.h>
#include <semio_msgs_ros/AttentionTargets.h>

#include <semio/recognition/attention_recognizer.h>
#include <semio/ros/humanoid_source_adapter.h>

//! Simple ROS wrapper around semio::AttentionRecognizer
class SemioAttentionNode
{
public:
    //! ROS message for the result of attention recognition for all humanoids
    typedef semio_msgs_ros::AttentionRecognitionResult _AttentionRecognitionResultMsg;
    //! ROS message for the result of attention recognition for a single humanoid
    typedef semio_msgs_ros::AttentionRecognitionHumanoidItem _AttentionRecognitionHumanoidItemMsg;
    //! ROS message for the result of attention recognition for a single humanoid joint
    typedef semio_msgs_ros::AttentionRecognitionJointItem _AttentionRecognitionJointItemMsg;
    //! ROS message for the result of attention recognition for a single attention target
    typedef semio_msgs_ros::AttentionRecognitionTopNItem _AttentionRecognitionTopNItemMsg;
    //! ROS message for a vector of attention targets
    typedef semio_msgs_ros::AttentionTargets _AttentionTargetsMsg;
    //! ROS message for a single attention target
    typedef semio_msgs_ros::AttentionTarget _AttentionTargetMsg;

    //! NodeHandle copy used to interface with ROS
    ros::NodeHandle nh_rel_;
    //! Attention recognition result publisher
    ros::Publisher result_pub_;
    //! Attention targets subscriber
    ros::Subscriber targets_sub_;

    //! Pointer to the input source for humanoids
    semio::HumanoidSource::Ptr humanoid_source_ptr_;
    //! Semio attention recognizer
    semio::AttentionRecognizer attention_recognizer_;

    /**
    @param nh_rel @copybrief nh_rel_
    @param humanoid_source_ptr @copybrief humanoid_source_ptr_
    */
    SemioAttentionNode( ros::NodeHandle & nh_rel, semio::HumanoidSource::Ptr humanoid_source_ptr )
    :
        nh_rel_( nh_rel ),
        result_pub_( nh_rel_.advertise<_AttentionRecognitionResultMsg>( "result", 100 ) ),
        targets_sub_( nh_rel_.subscribe( "targets", 10, &SemioAttentionNode::targetsCB, this ) ),
        humanoid_source_ptr_( humanoid_source_ptr )
    {
        //
    }

    //! Main loop
    void spin()
    {
        ros::Rate loop_rate( 30 );

        while( ros::ok() )
        {
            //! - Trigger ROS callbacks
            ros::spinOnce();

            //! - Pass humanoids to attention recognizer
            attention_recognizer_.getHumanoids() = humanoid_source_ptr_->update();
            //! - Calculate attention recognition result
            semio::AttentionRecognitionResult const & result = attention_recognizer_.calculateResult();

            //----------
            //! - Convert attention recognition result to ROS message
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

                    for( auto const & list_item : top_n_list.values() )
                    {
                        _AttentionRecognitionTopNItemMsg top_n_item_msg;
                        top_n_item_msg.likelihood = list_item.getValue();
                        top_n_item_msg.target_name = list_item.getData();

                        joint_msg.top_n_list.push_back( std::move( top_n_item_msg ) );
                    }

                    humanoid_msg.joints.push_back( std::move( joint_msg ) );
                }

                result_msg.humanoids.push_back( std::move( humanoid_msg ) );
            }
            //----------

            //! - Publish attention recognition result
            result_pub_.publish( std::move( result_msg ) );

            loop_rate.sleep();
        }
    }

    //! ROS callback for attention targets
    /**
    @param msg_ptr ConstPtr to the attention target message
    */
    void targetsCB( _AttentionTargetsMsg::ConstPtr const & msg_ptr )
    {
        auto const & targets_msg = *msg_ptr;

        //----------
        //! - Convert attention target message to semio::AttentionTargetArray
        semio::AttentionTargetArray attention_targets;

        for( auto const & target_msg : targets_msg.targets )
        {
            auto const & name = target_msg.name;
            auto const & position = target_msg.position;

            attention_targets.emplace( name, Eigen::Vector3d( position.x, position.y, position.z ) );
        }
        //----------

        //! - Update attention recognizer's list of targets
        attention_recognizer_.getTargets() = attention_targets;
    }
};

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "semio_attention_node" );
    //! - Create NodeHandle with relative namespace
    ros::NodeHandle nh_rel( "~" );

    //! - Create semio::ros::HumanoidSourceAdapter
    semio::ros::HumanoidSourceAdapter humanoid_source_adapter( nh_rel );

    //! - Create SemioAttentionNode; pass node handle and humanoid source
    SemioAttentionNode semio_attention_node( nh_rel, humanoid_source_adapter.getHumanoidSource() );
    //! - Start main loop SemioAttentionNode::spin()
    semio_attention_node.spin();

    return 0;
}
