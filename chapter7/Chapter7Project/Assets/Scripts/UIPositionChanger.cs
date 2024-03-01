using UnityEngine;

/// <summary>
/// カメラの視線に基づいてUIの位置を調整するスクリプト
/// </summary>
public class UIPositionChanger : MonoBehaviour
{
    [SerializeField] GameObject uiCanvasObj;
    [SerializeField] Transform centerEyeCamera;
    [SerializeField] float distanceFromCamera = 1.0f;

    void Update()
    {
        if (OVRInput.GetDown(OVRInput.Button.Two))
        {
            uiCanvasObj.SetActive(!uiCanvasObj.activeSelf);
            ChangeUIPosition();
        }
    }

    void ChangeUIPosition()
    {
        Vector3 cameraPositionXZ = new Vector3(centerEyeCamera.position.x, 0f, centerEyeCamera.position.z);
        Vector3 uiPosition = cameraPositionXZ + centerEyeCamera.forward * distanceFromCamera;
        uiPosition.y = centerEyeCamera.position.y;
        uiCanvasObj.transform.position = uiPosition;
        uiCanvasObj.transform.LookAt(centerEyeCamera);
        uiCanvasObj.transform.Rotate(0f, 180f, 0f);
    }
}