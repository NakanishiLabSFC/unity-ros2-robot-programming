using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Chapter2Interfaces;

/// <summary>
/// CalculateCylinderVolume型のサービスを実行するサーバーのスクリプト
/// </summary>
public class SimpleServer : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        // サービスを登録
        ros.ImplementService<CalculateCylinderVolumeRequest, CalculateCylinderVolumeResponse>("calculate_cylinder_volume", CalculateCylinderVolumeCallback);
    }

    CalculateCylinderVolumeResponse CalculateCylinderVolumeCallback(CalculateCylinderVolumeRequest request)
    {
        Debug.Log("Request: " + request.r + ", " + request.h);
        CalculateCylinderVolumeResponse response = new CalculateCylinderVolumeResponse();
        response.v = 2 * Mathf.PI * Mathf.Pow((float)request.r, 2) * (float)request.h;
        return response;
    }
}
