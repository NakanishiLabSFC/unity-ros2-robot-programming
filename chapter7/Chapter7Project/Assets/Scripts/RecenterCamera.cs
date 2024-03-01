using UnityEngine;

/// <summary>
/// OVRCameraのリセンターを行うスクリプト
/// </summary>
public class RecenterCamera : MonoBehaviour
{
    [SerializeField] Transform OVRCameraRig;
    [SerializeField] Transform recenterOrigin;

    public void OnRecenter()
    {
      Vector3 originPosition = recenterOrigin.localPosition;
      float yawOffset = -recenterOrigin.localEulerAngles.y;
      float yawAngle = -yawOffset * Mathf.Deg2Rad;  // 角度をラジアンに変換

      // 座標変換を行い新しい位置を計算
      float sinAngle = Mathf.Sin(yawAngle);
      float cosAngle = Mathf.Cos(yawAngle);
      float xOffset = originPosition.z * sinAngle - originPosition.x * cosAngle;
      float zOffset = -originPosition.z * cosAngle - originPosition.x * sinAngle;

      // OVRCameraRigの位置と角度を更新
      OVRCameraRig.localPosition = new Vector3(xOffset, 0, zOffset);
      OVRCameraRig.localEulerAngles = new Vector3(0, yawOffset, 0);
    }
}