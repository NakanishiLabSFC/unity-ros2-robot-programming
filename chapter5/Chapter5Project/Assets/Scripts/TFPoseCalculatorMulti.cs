using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using RosMessageTypes.Tf2;

public class TFPoseCalculatorMulti : MonoBehaviour
{
    [SerializeField] MapTransformer mapTransformer;
    // [SerializeField] GameObject turtlebot3Obj;
    [SerializeField] string parentFrameName;
    [SerializeField] string targetFrameName;
    // public class Pose {
    //     public Vector3 position = new Vector3();
    //     public Quaternion rotation = new Quaternion();
    // }
    ROSConnection ros;
    string topicName = "/tf";
    Transform mapTransform; 
    // Pose mapFramePose = new Pose();
    // Pose odomFramePose = new Pose();


    [SerializeField] MultiNavigationController multiNavigationController;


    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TFMessageMsg>(topicName, ReceiveTFMsg);
        var empty = new GameObject();
        mapTransform = empty.transform; 
    }

    void Update()
    {
        for (int i = 0; i < multiNavigationController.NavigationRobots.Count; i++)
        {
            var robot = multiNavigationController.NavigationRobots[i];

            mapTransform.position = robot.mapFramePose.position;
            mapTransform.rotation = robot.mapFramePose.rotation;

            robot.turtlebot3Obj.transform.position = GetRelativePosition(robot.odomFramePose.position) - mapTransformer.OriginPos;
            robot.turtlebot3Obj.transform.rotation = GetRelativeRotation(robot.odomFramePose.rotation);
        }
        
    }

    void ReceiveTFMsg(TFMessageMsg msg)
    {
        for (int i = 0; i < multiNavigationController.NavigationRobots.Count; i++)
        {
            var robot = multiNavigationController.NavigationRobots[i];
            
            var parentFrameID = robot.rosNamespace + "/" + parentFrameName;
            var targetFrameID = robot.rosNamespace + "/" + targetFrameName;

            for (int j = 0; j < msg.transforms.Length; j++)
            {

                if (msg.transforms[j].child_frame_id == parentFrameID) {
                    robot.mapFramePose = ConvertToUnityPose(msg.transforms[j].transform.translation, msg.transforms[j].transform.rotation);

                } else if (msg.transforms[j].child_frame_id == targetFrameID) {
                    robot.odomFramePose = ConvertToUnityPose(msg.transforms[j].transform.translation, msg.transforms[j].transform.rotation);
                } 
            }
        }
          
    }

    Pose ConvertToUnityPose(Vector3Msg rosPosMsg, QuaternionMsg rosQuaternionMsg)
    {
        Pose unityPose = new Pose();

        Vector3<Unity.Robotics.ROSTCPConnector.ROSGeometry.FLU> rosPos = rosPosMsg.As<FLU>();
        Vector3 unityPos = rosPos.toUnity;
        unityPose.position = unityPos;

        Quaternion<Unity.Robotics.ROSTCPConnector.ROSGeometry.FLU> rosQuaternion = rosQuaternionMsg.As<FLU>();
        Quaternion unityQuaternion = rosQuaternion.toUnity;
        unityPose.rotation = unityQuaternion;

        return unityPose;
    }

    Vector3 GetRelativePosition(Vector3 worldPosition)
    {
        return mapTransform.TransformPoint(worldPosition);
    }

    public Quaternion GetRelativeRotation(Quaternion worldRotation)
    {
        return worldRotation * mapTransform.rotation;
    }
}
