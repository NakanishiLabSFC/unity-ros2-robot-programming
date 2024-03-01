using RosMessageTypes.Chapter4Interfaces;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

/// <summary>
/// サーボを回転させるサービスを呼び出すスクリプト
/// </summary>
public class ServoPositionServiceCaller : MonoBehaviour
{
    [SerializeField] string serviceName;
    [SerializeField] int servoStep;
    [SerializeField] int minServoPosition;
    [SerializeField] int maxServoPosition;
    [SerializeField] int cameraStep;
    [SerializeField] int minCameraAngle;
    [SerializeField] int maxCameraAngle;
    [SerializeField] GameObject cameraObj;
    ROSConnection ros;
    int servoPosition = 90;
    int cameraAngle;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<ServoPositionServiceRequest, ServoPositionServiceResponse>(serviceName);
    }

    void CallServoPositionService(int pos) {
        ServoPositionServiceRequest positionServiceRequest = new ServoPositionServiceRequest(pos);
        ros.SendServiceMessage<ServoPositionServiceResponse>(serviceName, positionServiceRequest, ServoPositionServiceCallback);
    }

    void ServoPositionServiceCallback(ServoPositionServiceResponse response)
    {
        Debug.Log(response);
    }

    public void ServoUp() {
        // サーボの回転
        servoPosition -= servoStep;
        if (servoPosition >= maxServoPosition) {
            servoPosition = maxServoPosition;
        }
        CallServoPositionService(servoPosition);
        // Unityのカメラの回転
        cameraAngle += cameraStep;
        if (cameraAngle >= maxCameraAngle) {
            cameraAngle = maxCameraAngle;
        }
        cameraObj.transform.rotation = Quaternion.Euler(-cameraAngle, 0, 0);
    }

    public void ServoDown() {
        // サーボの回転
        servoPosition += servoStep;
        if (servoPosition <= minServoPosition) {
            servoPosition = minServoPosition;
        }
        CallServoPositionService(servoPosition);
        // Unityのカメラの回転
        cameraAngle -= cameraStep;
        if (cameraAngle <= minCameraAngle) {
            cameraAngle = minCameraAngle;
        }
        cameraObj.transform.rotation = Quaternion.Euler(-cameraAngle, 0, 0);
    }
}