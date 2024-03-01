using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

/// <summary>
/// ゴールをパブリッシュするスクリプト
/// </summary>
public class GoalPublisher : MonoBehaviour
{
    [SerializeField] string goalTopicName;
    [SerializeField] string initialPoseTopicName;
    [SerializeField] string cancelTopicName;
    [SerializeField] GoalSelector goalSelector;
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(goalTopicName);
        ros.RegisterPublisher<PoseWithCovarianceStampedMsg>(initialPoseTopicName);
        ros.RegisterPublisher<BoolMsg>(cancelTopicName);
    }

    public void SendInitialPose() {
        PoseWithCovarianceStampedMsg msg = new PoseWithCovarianceStampedMsg();
        msg.header.stamp.sec = 0;
        msg.header.stamp.nanosec = 0;
        msg.header.frame_id = "map";
        msg.pose.pose.position.x = 0.0;
        msg.pose.pose.position.y = 0.0;
        msg.pose.pose.position.z = 0.0;
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = 0.0;
        msg.pose.pose.orientation.w = 1.0;
        ros.Publish(initialPoseTopicName, msg);
        Debug.Log("Initial Pose: " + msg.pose);
    }

    public void SendGoalPose() {
        PoseStampedMsg msg = new PoseStampedMsg();
        msg.pose = goalSelector.GoalPose;
        msg.header.stamp.sec = 0;
        msg.header.stamp.nanosec = 0;
        msg.header.frame_id = "map";
        ros.Publish(goalTopicName, msg);
        Debug.Log("Goal Pose: " + msg.pose);
    }

    public void SendCancel()
    {
        BoolMsg msg = new BoolMsg();
        msg.data = true;
        ros.Publish(cancelTopicName, msg);
        Debug.Log("Cancel");
    }
}
