using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// String型のメッセージをサブスクライブするスクリプト
/// </summary>
public class SimpleSubscriber : MonoBehaviour
{
    ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        // サブスクライバーを作成
        ros.Subscribe<StringMsg>("/simple_topic", ReceiveStringMsg);
    }

    void ReceiveStringMsg(StringMsg msg)
    {   
        Debug.Log(msg);
    }
}
