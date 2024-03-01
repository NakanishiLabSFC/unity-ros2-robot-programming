using UnityEngine;

/// <summary>
/// パブリッシュするTwistメッセージを管理するスクリプト
/// </summary>
public class TwistController : MonoBehaviour
{
    [SerializeField] TwistPublisher twistPublisher;
    [SerializeField] float linearSpeed;  // 並進速度
    [SerializeField] float angularSpeed;  // 回転速度
    Vector3 linear = Vector3.zero;
    Vector3 angular = Vector3.zero;

    void Update()
    {
        if (Input.GetKey(KeyCode.UpArrow)) {
            SetForwardVelocity();
        }
        else if (Input.GetKey(KeyCode.DownArrow)) {
            SetBackwardVelocity();
        }
        if (Input.GetKey(KeyCode.LeftArrow)) {
            SetTurnLeftVelocity();
        }
        else if (Input.GetKey(KeyCode.RightArrow)) {
            SetTurnRightVelocity();
        }

        if (Input.GetKeyUp(KeyCode.UpArrow) || Input.GetKeyUp(KeyCode.DownArrow) || Input.GetKeyUp(KeyCode.LeftArrow) || Input.GetKeyUp(KeyCode.RightArrow)) {
            SetEmptyVelocity();  // どのキーも押されていないときは空のデータをセットする
        }

        twistPublisher.SetTwistMsgValue(linear, angular);
    }

    public void SetEmptyVelocity() {
        linear = Vector3.zero;
        angular = Vector3.zero;
    }

    public void SetForwardVelocity() {
        linear.z = linearSpeed;
    }

    public void SetBackwardVelocity() {
        linear.z = -linearSpeed;
    }

    public void SetTurnRightVelocity() {
        angular.y = angularSpeed;
    }

    public void SetTurnLeftVelocity() {
        angular.y = -angularSpeed;
    }
}
