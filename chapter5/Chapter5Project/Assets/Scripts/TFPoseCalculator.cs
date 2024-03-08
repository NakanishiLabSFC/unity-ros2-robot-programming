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
    ROSConnection ros;
    string topicName = "/tf";
    Transform parentFrameTransform; 
    Pose parentFramePose = new Pose();
    Pose targetFramePose = new Pose();
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TFMessageMsg>(topicName, ReceiveTFMsg);
        var empty = new GameObject();
        parentFrameTransform = empty.transform; 
    }

    void Update()
    {
        parentFrameTransform.position = parentFramePose.position;
        parentFrameTransform.rotation = parentFramePose.rotation;

        turtlebot3Obj.transform.position = TFUtility.GetRelativePosition(parentFrameTransform, targetFramePose.position) - mapTransformer.OriginPos;
        turtlebot3Obj.transform.rotation = TFUtility.GetRelativeRotation(parentFrameTransform, targetFramePose.rotation);
    }

    void ReceiveTFMsg(TFMessageMsg msg)
    {
        for (int i = 0; i < msg.transforms.Length; i++)
            {
            if (msg.transforms[i].child_frame_id == parentFrameName) {
                parentFramePose = TFUtility.ConvertToUnityPose(msg.transforms[i].transform.translation, msg.transforms[i].transform.rotation);
            } else if (msg.transforms[i].child_frame_id == targetFrameName) {
                targetFramePose = TFUtility.ConvertToUnityPose(msg.transforms[i].transform.translation, msg.transforms[i].transform.rotation);
            }  
        } 
    }
}

public static class TFUtility {
    public static Pose ConvertToUnityPose(Vector3Msg rosPosMsg, QuaternionMsg rosQuaternionMsg)
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

    public static Vector3 GetRelativePosition(Transform t, Vector3 worldPosition)
    {
        return t.TransformPoint(worldPosition);
    }

    public static Quaternion GetRelativeRotation(Transform t, Quaternion worldRotation)
    {
        return worldRotation * t.rotation;
    }
}
