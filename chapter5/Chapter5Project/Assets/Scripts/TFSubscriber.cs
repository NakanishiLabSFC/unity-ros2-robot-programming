using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using RosMessageTypes.Tf2;

/// <summary>
/// TFトピックをサブスクライブするスクリプト
/// </summary>
public class TFSubscriber : MonoBehaviour
{
    [SerializeField] string topicName = "tf";
    [SerializeField] MapTransformer mapTransformer;
    [SerializeField] GameObject turtlebot3Obj;
    public class Pose {
        public Vector3 position;
        public Quaternion rotation;
    }
    ROSConnection ros;
    Pose pose = new Pose();
    Transform baseTransform; 
    bool receivedFisrtMsg;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TFMessageMsg>(topicName, ReceiveTFMsg);
        var empty = new GameObject();
        baseTransform = empty.transform; 
    }

    void Update()
    {
        turtlebot3Obj.transform.position = pose.position;
        turtlebot3Obj.transform.rotation = pose.rotation;
    }

    void ReceiveTFMsg(TFMessageMsg msg)
    {
        for (int i = 0; i < msg.transforms.Length; i++)
        {
            // ロボット本体の姿勢情報が格納されたフレームを抽出
            if (msg.transforms[i].child_frame_id == "base_footprint")
            {
                Vector3Msg rosPosMsg = msg.transforms[i].transform.translation;
                Vector3<Unity.Robotics.ROSTCPConnector.ROSGeometry.FLU> rosPos = rosPosMsg.As<FLU>();
                Vector3 unityPos = rosPos.toUnity;  // 座標変換
                // 初期位置からの相対位置を計算
                Vector3 adjustedPos = baseTransform.InverseTransformPoint(unityPos);

                QuaternionMsg rosQuaternionMsg = msg.transforms[i].transform.rotation;
                Quaternion<Unity.Robotics.ROSTCPConnector.ROSGeometry.FLU> rosQuaternion = rosQuaternionMsg.As<FLU>();
                Quaternion unityQuaternion = rosQuaternion.toUnity;  // 座標変換
                // 初期角度からの相対角度を計算
                Quaternion adjustedRot = unityQuaternion * Quaternion.Inverse(baseTransform.rotation);

                pose.position = adjustedPos - mapTransformer.OriginPos;
                pose.rotation = adjustedRot;

                if (!receivedFisrtMsg) {
                    // 初期位置を保存
                    baseTransform.position = unityPos;
                    baseTransform.rotation = unityQuaternion;
                    receivedFisrtMsg = true;
                }
            }
        }
    }
}