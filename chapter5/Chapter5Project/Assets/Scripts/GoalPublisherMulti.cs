using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

/// <summary>
/// 複数台にゴールをパブリッシュするスクリプト
/// </summary>
public class GoalPublisherMulti : MonoBehaviour
{
    [SerializeField] string goalTopicName;
    [SerializeField] string initialPoseTopicName;
    [SerializeField] string cancelTopicName;
    [SerializeField] GoalSelectorMulti goalSelectorMulti;
    [SerializeField] MultiNavigationController multiNavigationController;
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        foreach (var navigationRobot in multiNavigationController.NavigationRobots)
        {
            ros.RegisterPublisher<PoseStampedMsg>("/" + navigationRobot.rosNamespace + goalTopicName);
            ros.RegisterPublisher<PoseWithCovarianceStampedMsg>("/" + navigationRobot.rosNamespace + initialPoseTopicName);
            ros.RegisterPublisher<BoolMsg>("/" + navigationRobot.rosNamespace + cancelTopicName);
        }
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
        msg.pose.pose.orientation.w = 0.1;
        var topicName = "/" + multiNavigationController.SelectedRobot.rosNamespace + initialPoseTopicName;
        ros.Publish(topicName, msg);
        Debug.Log("Initial Pose: " + msg.pose);
    }

    public void SendGoalPose() {
        PoseStampedMsg msg = new PoseStampedMsg();
        msg.pose = goalSelectorMulti.GoalPose;
        msg.header.stamp.sec = 0;
        msg.header.stamp.nanosec = 0;
        msg.header.frame_id = "map";
        var topicName = "/" + multiNavigationController.SelectedRobot.rosNamespace + goalTopicName;
        ros.Publish(topicName, msg);
        Debug.Log("Goal Pose: " + msg.pose);
    }

    public void SendCancel()
    {
        BoolMsg msg = new BoolMsg();
        msg.data = true;
        var topicName = "/" + multiNavigationController.SelectedRobot.rosNamespace + cancelTopicName;
        ros.Publish(topicName, msg);
        Debug.Log("Cancel");
    }
}
