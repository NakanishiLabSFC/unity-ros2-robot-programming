using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using System;

/// <summary>
/// String型のメッセージをパブリッシュするスクリプト
/// </summary>
public class SimplePublisher : MonoBehaviour
{
    ROSConnection ros;
    float publishMessageFrequency = 1f;
    float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        // パブリッシャーを作成
        ros.RegisterPublisher<StringMsg>("/simple_topic");
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;
        // 一定周期でパブリッシュする
        if (timeElapsed > publishMessageFrequency)
        {
            StringMsg msg = new StringMsg();
            var dt = DateTime.Now;
            msg.data = String.Format("{0}:{1}:{2}", new string[] {dt.Hour.ToString(), dt.Minute.ToString(), dt.Second.ToString()});
            ros.Publish("/simple_topic", msg);
            timeElapsed = 0;
        }
    }
}
