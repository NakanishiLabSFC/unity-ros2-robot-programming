using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using RosMessageTypes.Tf2;

public class TFPoseCalculator : MonoBehaviour
{
    [SerializeField] GameObject turtlebot3Obj;
    [SerializeField] string mapChildFrameName = "odom";
    [SerializeField] string odomChildFrameName = "base_footprint";
    [SerializeField] float offsetTurtlebot3Height = 0.19f;
    ROSConnection ros;
    string topicName = "/tf";
    Transform mapFrameTransform; 
    Pose mapFramePose = new Pose();
    Pose odomFramePose = new Pose();
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TFMessageMsg>(topicName, ReceiveTFMsg);
        var empty = new GameObject();
        mapFrameTransform = empty.transform; 
    }

    void Update()
    {
        mapFrameTransform.position = mapFramePose.position;
        mapFrameTransform.rotation = mapFramePose.rotation;
        // odomフレームの座標をmapフレーム基準に変換する
        var pos = TFUtility.GetRelativePosition(mapFrameTransform, odomFramePose.position);
        turtlebot3Obj.transform.position = new Vector3(
            pos.x, offsetTurtlebot3Height/2, pos.z
        );
        turtlebot3Obj.transform.rotation = TFUtility.GetRelativeRotation(mapFrameTransform, odomFramePose.rotation);
    }

    void ReceiveTFMsg(TFMessageMsg msg)
    {
        for (int i = 0; i < msg.transforms.Length; i++)
            {
            if (msg.transforms[i].child_frame_id == mapChildFrameName) {
                mapFramePose = TFUtility.ConvertToUnityPose(msg.transforms[i].transform.translation, msg.transforms[i].transform.rotation);
            } else if (msg.transforms[i].child_frame_id == odomChildFrameName) {
                odomFramePose = TFUtility.ConvertToUnityPose(msg.transforms[i].transform.translation, msg.transforms[i].transform.rotation);
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

    public static Vector3 GetRelativePosition(Transform t, Vector3 pos)
    {
        return t.TransformPoint(pos);
    }

    public static Quaternion GetRelativeRotation(Transform t, Quaternion rot)
    {
        return rot * t.rotation;
    }
}
