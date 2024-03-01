using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Chapter2Interfaces;

/// <summary>
/// CalculateCylinderVolume型のサービスを実行するクライアントのスクリプト
/// </summary>
public class SimpleClient : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        // サービスを登録
        ros.RegisterRosService<CalculateCylinderVolumeRequest, CalculateCylinderVolumeResponse>("calculate_cylinder_volume");   
        // リクエストを生成して送信
        CalculateCylinderVolumeRequest request = new CalculateCylinderVolumeRequest(2.0, 3.0);
        ros.SendServiceMessage<CalculateCylinderVolumeResponse>("calculate_cylinder_volume", request, CalculateCylinderVolumeCallback);
    }

    void CalculateCylinderVolumeCallback(CalculateCylinderVolumeResponse response)
    {
        Debug.Log("V = " + response.v);
    }
}
