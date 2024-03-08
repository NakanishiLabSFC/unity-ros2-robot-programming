using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Tf2;

public class TFPoseCalculatorMulti : MonoBehaviour
{
    [SerializeField] MapTransformer mapTransformer;
    [SerializeField] MultiNavigationController multiNavigationController;
    [SerializeField] string parentFrameBaseName;
    [SerializeField] string targetFrameBaseName;
    ROSConnection ros;
    string topicName = "/tf";
    Transform parentFrameTransform; 

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TFMessageMsg>(topicName, ReceiveTFMsg);
        var empty = new GameObject();
        parentFrameTransform = empty.transform;

        foreach (var robot in multiNavigationController.NavigationRobots)
        {
            robot.ParentFrameName = robot.rosNamespace + "/" + parentFrameBaseName;
            robot.TargetFrameName = robot.rosNamespace + "/" + targetFrameBaseName;
        }
    }

    void Update()
    {
        foreach (var robot in multiNavigationController.NavigationRobots)
        {
            parentFrameTransform.position = robot.parentFramePose.position;
            parentFrameTransform.rotation = robot.parentFramePose.rotation;

            robot.turtlebot3Obj.transform.position = TFUtility.GetRelativePosition(parentFrameTransform, robot.targetFramePose.position) - mapTransformer.OriginPos;
            robot.turtlebot3Obj.transform.rotation = TFUtility.GetRelativeRotation(parentFrameTransform, robot.targetFramePose.rotation);
        }
        
    }

    void ReceiveTFMsg(TFMessageMsg msg)
    {
        foreach (var robot in multiNavigationController.NavigationRobots)
        {
            for (int i = 0; i < msg.transforms.Length; i++)
            {
                if (msg.transforms[i].child_frame_id == robot.ParentFrameName) {
                    robot.parentFramePose = TFUtility.ConvertToUnityPose(msg.transforms[i].transform.translation, msg.transforms[i].transform.rotation);
                } else if (msg.transforms[i].child_frame_id == robot.TargetFrameName) {
                    robot.targetFramePose = TFUtility.ConvertToUnityPose(msg.transforms[i].transform.translation, msg.transforms[i].transform.rotation);
                } 
            }
        }  
    }
}
