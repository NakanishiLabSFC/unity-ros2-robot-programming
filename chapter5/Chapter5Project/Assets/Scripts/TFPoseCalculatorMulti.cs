using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Tf2;

public class TFPoseCalculatorMulti : MonoBehaviour
{
    [SerializeField] MapTransformer mapTransformer;
    [SerializeField] MultiNavigationController multiNavigationController;
    [SerializeField] string mapChildFrameBaseName = "odom";
    [SerializeField] string odomChildFrameBaseName = "base_footprint";
    ROSConnection ros;
    string topicName = "/tf";
    Transform mapFrameTransform; 

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TFMessageMsg>(topicName, ReceiveTFMsg);
        var empty = new GameObject();
        mapFrameTransform = empty.transform;

        foreach (var robot in multiNavigationController.NavigationRobots)
        {
            robot.mapFrameName = robot.rosNamespace + "/" + mapChildFrameBaseName;
            robot.odomFrameName = robot.rosNamespace + "/" + odomChildFrameBaseName;
        }
    }

    void Update()
    {
        foreach (var robot in multiNavigationController.NavigationRobots)
        {
            mapFrameTransform.position = robot.mapFramePose.position;
            mapFrameTransform.rotation = robot.mapFramePose.rotation;

            robot.turtlebot3Obj.transform.position = TFUtility.GetRelativePosition(mapFrameTransform, robot.odomFramePose.position) - mapTransformer.OriginPos;
            robot.turtlebot3Obj.transform.rotation = TFUtility.GetRelativeRotation(mapFrameTransform, robot.odomFramePose.rotation);
        }
        
    }

    void ReceiveTFMsg(TFMessageMsg msg)
    {
        foreach (var robot in multiNavigationController.NavigationRobots)
        {
            for (int i = 0; i < msg.transforms.Length; i++)
            {
                if (msg.transforms[i].child_frame_id == robot.mapFrameName) {
                    robot.mapFramePose = TFUtility.ConvertToUnityPose(msg.transforms[i].transform.translation, msg.transforms[i].transform.rotation);
                } else if (msg.transforms[i].child_frame_id == robot.odomFrameName) {
                    robot.odomFramePose = TFUtility.ConvertToUnityPose(msg.transforms[i].transform.translation, msg.transforms[i].transform.rotation);
                } 
            }
        }  
    }
}
