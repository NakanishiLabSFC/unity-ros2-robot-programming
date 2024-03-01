using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using RosMessageTypes.Tf2;
using System.Collections.Generic;

/// <summary>
/// 複数台のTFトピックをサブスクライブするスクリプト
/// </summary>
public class TFSubscriberMulti : MonoBehaviour
{
    [SerializeField] string topicName = "tf";
    [SerializeField] MapTransformer mapTransformer;
    [SerializeField] MultiNavigationController multiNavigationController;
    public class Pose {
        public Vector3 position;
        public Quaternion rotation;
    }
    public class SimulatorInfo {
        public Transform baseTransform = new GameObject().transform;
        public bool receivedFisrtMsg;
        public Pose initialPose;
        public Pose pose = new Pose();
    }
    ROSConnection ros;
    List<SimulatorInfo> simulatorInfos;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TFMessageMsg>(topicName, ReceiveTFMsg);

        simulatorInfos = new List<SimulatorInfo>(multiNavigationController.NavigationRobots.Count);
        for (int i = 0; i < multiNavigationController.NavigationRobots.Count; i++)
        {
            simulatorInfos.Add(new SimulatorInfo());
        }
    }

    void Update()
    {
        for (int i = 0; i < multiNavigationController.NavigationRobots.Count; i++)
        {
            var navigationRobot = multiNavigationController.NavigationRobots[i];
            navigationRobot.turtlebot3Obj.transform.position = simulatorInfos[i].pose.position;
            navigationRobot.turtlebot3Obj.transform.rotation = simulatorInfos[i].pose.rotation;
        }
    }

    void ReceiveTFMsg(TFMessageMsg msg)
    {
        for (int i = 0; i < multiNavigationController.NavigationRobots.Count; i++)
        {
            for (int j = 0; j < msg.transforms.Length; j++)
            {
                // ロボット本体の姿勢情報が格納されたフレームを抽出
                var frameID = multiNavigationController.NavigationRobots[i].rosNamespace + "/" + "base_footprint";
                if (msg.transforms[j].child_frame_id == frameID)
                {
                    Vector3Msg rosPosMsg = msg.transforms[j].transform.translation;
                    Vector3<Unity.Robotics.ROSTCPConnector.ROSGeometry.FLU> rosPos = rosPosMsg.As<FLU>();
                    Vector3 unityPos = rosPos.toUnity;  // 座標変換
                    // 初期位置からの相対位置を計算
                    Vector3 adjustedPos = simulatorInfos[i].baseTransform.InverseTransformPoint(unityPos);

                    QuaternionMsg rosQuaternionMsg = msg.transforms[j].transform.rotation;
                    Quaternion<Unity.Robotics.ROSTCPConnector.ROSGeometry.FLU> rosQuaternion = rosQuaternionMsg.As<FLU>();
                    Quaternion unityQuaternion = rosQuaternion.toUnity;  // 座標変換
                    // 初期角度からの相対角度を計算
                    Quaternion adjustedRot = unityQuaternion * Quaternion.Inverse(simulatorInfos[i].baseTransform.rotation);

                    simulatorInfos[i].pose.position = adjustedPos - mapTransformer.OriginPos;
                    simulatorInfos[i].pose.rotation = adjustedRot;

                    if (!simulatorInfos[i].receivedFisrtMsg) {
                        // 初期位置を保存
                        simulatorInfos[i].baseTransform.position = unityPos;
                        simulatorInfos[i].baseTransform.rotation = unityQuaternion;
                        simulatorInfos[i].receivedFisrtMsg = true;
                    }
                }
            }
        }
    }
}
