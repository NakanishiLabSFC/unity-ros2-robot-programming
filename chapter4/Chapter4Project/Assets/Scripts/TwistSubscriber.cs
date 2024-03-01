using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

/// <summary>
/// Twist型のメッセージをサブスクライブするスクリプト
/// </summary>
public class TwistSubscriber : MonoBehaviour
{
    [SerializeField] string topicName = "/cmd_vel";
    ROSConnection ros;
    (Vector3 linear, Vector3 angular) twist = (new Vector3(), new Vector3());
    public (Vector3 linear, Vector3 angular) Twist {
        get { return twist; }
    }
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, ReceiveTwistMsg);
    }

    void ReceiveTwistMsg(TwistMsg msg)
    {
        // 移動速度と回転速度をUnityの座標系に変換
        Vector3 linear = msg.linear.From<FLU>();
        Vector3 angular = msg.angular.From<FLU>();
        angular.y *= -1f;

        twist.linear = linear;
        twist.angular = angular;
    }
}
