using UnityEngine;

/// <summary>
/// Unityで動作するロボットのシミュレーターのスクリプト
/// </summary>
public class Turtlesim : MonoBehaviour
{
    [SerializeField] TwistSubscriber twistSubscriber;
    [SerializeField] TFSubscriber tfSubscriber;
    [SerializeField] TurtlesimManipulationMode turtlesimManipulationMode;
    enum TurtlesimManipulationMode {
        Twist, 
        TF
    }
    Vector3 linearSpeed;
    Vector3 angularSpeed;

    void Update() {
        if (turtlesimManipulationMode == TurtlesimManipulationMode.Twist) {
            // サブスクライブしたTwistメッセージを取得
            var twist = twistSubscriber.Twist;
            linearSpeed = twist.linear;
            angularSpeed = twist.angular;
            Move(linearSpeed, angularSpeed);
        } else if (turtlesimManipulationMode == TurtlesimManipulationMode.TF) {
            // サブスクライブしたTFメッセージを取得
            var pose = tfSubscriber.Pose;
            this.gameObject.transform.position = new Vector3(pose.position.x, 0.075f, pose.position.z);
            this.gameObject.transform.rotation = pose.rotation;
        }
    }

    void Move(Vector3 linear, Vector3 angular)
    {
        transform.Translate(Vector3.forward * linear.z * Time.deltaTime);
        transform.Rotate(angular);
    }
}
