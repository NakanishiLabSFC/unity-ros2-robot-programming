using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

/// <summary>
/// Twist型のメッセージをパブリッシュするスクリプト
/// </summary>
public class TwistPublisher : MonoBehaviour
{
    [SerializeField] string topicName;
    [SerializeField] float publishMessageFrequency = 0.5f;  // パブリッシュする時間間隔
    ROSConnection ros;
    float timeElapsed;
    TwistMsg twist = new TwistMsg();

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName); 
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishMessageFrequency)  // 一定周期でパブリッシュする
        {
            ros.Publish(topicName, twist);
            timeElapsed = 0;
        }
    }

    public void SetTwistMsgValue(Vector3 linearVector, Vector3 angularVector) {
        // 移動速度と回転速度をROSの座標系に変換
        Vector3<FLU> rosLinear = linearVector.To<FLU>();
        Vector3<FLU> rosAngular = angularVector.To<FLU>();
        rosAngular.z *= -1f;

        twist.linear = rosLinear;
        twist.angular = rosAngular;
    }
}
