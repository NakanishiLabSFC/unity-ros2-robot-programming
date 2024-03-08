using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using RosMessageTypes.Tf2;

public class TFPoseCalculator : MonoBehaviour
{
    [SerializeField] MapTransformer mapTransformer;
    [SerializeField] GameObject turtlebot3Obj;
    [SerializeField] string parentFrameName;
    [SerializeField] string targetFrameName;
    public class Pose {
        public Vector3 position = new Vector3();
        public Quaternion rotation = new Quaternion();
    }
    ROSConnection ros;
    string topicName = "/tf";
    Transform mapTransform; 
    Pose mapFramePose = new Pose();
    Pose odomFramePose = new Pose();
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TFMessageMsg>(topicName, ReceiveTFMsg);
        var empty = new GameObject();
        mapTransform = empty.transform; 
    }

    void Update()
    {
        mapTransform.position = mapFramePose.position;
        mapTransform.rotation = mapFramePose.rotation;

        turtlebot3Obj.transform.position = GetRelativePosition(odomFramePose.position) - mapTransformer.OriginPos;
        turtlebot3Obj.transform.rotation = GetRelativeRotation(odomFramePose.rotation);
    }

    void ReceiveTFMsg(TFMessageMsg msg)
    {
        if (msg.transforms[0].header.frame_id == parentFrameName) {
            mapFramePose = ConvertToUnityPose(msg.transforms[0].transform.translation, msg.transforms[0].transform.rotation);
        } else if (msg.transforms[0].header.frame_id == targetFrameName) {
            odomFramePose = ConvertToUnityPose(msg.transforms[0].transform.translation, msg.transforms[0].transform.rotation);
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
