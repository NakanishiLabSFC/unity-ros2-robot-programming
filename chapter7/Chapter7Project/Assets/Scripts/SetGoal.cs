using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

/// <summary>
/// VRでゴールを指定するスクリプト
/// </summary>
public class SetGoal : MonoBehaviour
{
    [SerializeField] GameObject directionIndicatorPrefab;
    [SerializeField] GameObject goalMarkerPrefab;
    [SerializeField] ControllerRayHit getLaserPosition;
    [SerializeField] GoalPublisher goalPublisher;
    bool isTriggerOn;
    GameObject previousGoalMarkerObj;
    GameObject directionIndicator;
    Vector3 raycastHitPoint;
    Vector3 raycastHitPointOnButtonDown;
    Quaternion directionRotation;
    Vector3 goalPosition;
    Quaternion goalRotation;
    PoseMsg goalPose;
    public PoseMsg GoalPose
    {
        get { return goalPose; }
    }

    void Start()
    {
        directionIndicator = Instantiate(directionIndicatorPrefab);
        directionIndicator.SetActive(false);
    }

    void Update()
    {
        raycastHitPoint = getLaserPosition.HitPosition;
        // コントローラのトリガーが引かれた時
        if (OVRInput.GetDown(OVRInput.Button.PrimaryIndexTrigger, OVRInput.Controller.RTouch))
        {
            raycastHitPointOnButtonDown = raycastHitPoint;
            // ゴールの向きを表すインジケータを表示
            directionIndicator.SetActive(true);
            directionIndicator.transform.position = raycastHitPoint;
            isTriggerOn = true;
        }

        // コントローラのトリガーが引かれている間
        if (isTriggerOn)
        {
            RotateDirectionIndicatorObj();

            if (OVRInput.GetUp(OVRInput.Button.PrimaryIndexTrigger, OVRInput.Controller.RTouch))
            {
                // ゴール位置の送信
                CreateGoalPoseMsg();
                goalPublisher.SendGoalPose();

                directionIndicator.SetActive(false);  // 方向を示すオブジェクトを表示
                // ゴールのマーカーを作成
                GameObject goalMarkerObj = Instantiate(goalMarkerPrefab, raycastHitPointOnButtonDown, goalRotation);
                if (previousGoalMarkerObj != null) Destroy(previousGoalMarkerObj);
                previousGoalMarkerObj = goalMarkerObj;
                isTriggerOn = false;
            }
        }
    }

    void RotateDirectionIndicatorObj()
    {
        Vector2 stickInput = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.RTouch);
        var angle = Mathf.Atan2(stickInput.x, stickInput.y) * Mathf.Rad2Deg;
        directionRotation = Quaternion.Euler(0f, angle, 0f);
        directionIndicator.transform.rotation = directionRotation;
    }

    void CreateGoalPoseMsg()
    {
        goalPosition = raycastHitPointOnButtonDown;
        goalRotation = directionRotation;
        Vector3<FLU> rosPosition = goalPosition.To<FLU>();
        Quaternion<FLU> rosRotation = goalRotation.To<FLU>();
        PoseMsg pose = new PoseMsg();
        pose.position.x = rosPosition.x;
        pose.position.y = rosPosition.y;
        pose.position.z = rosPosition.z;
        pose.orientation.x = rosRotation.x;
        pose.orientation.y = rosRotation.y;
        pose.orientation.z = rosRotation.z;
        pose.orientation.w = rosRotation.w;
        goalPose = pose;
    }
}