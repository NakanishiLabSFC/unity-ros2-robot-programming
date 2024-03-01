using UnityEngine;
using TMPro;
using UnityEngine.EventSystems;
using System.Linq;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using System.Collections.Generic;
using UnityEngine.UI;

/// <summary>
/// 複数台にパブリッシュするゴールを設定するスクリプト
/// </summary>
public class GoalSelectorMulti : MonoBehaviour
{
    [SerializeField] GameObject arrowObj;
    [SerializeField] GameObject cursorObj;
    [SerializeField] GameObject goalMarkerPrefab;
    [SerializeField] GameObject mapObj;
    [SerializeField] TextMeshProUGUI goalPosRotText;
    [SerializeField] MapTransformer mapTransformer;
    [SerializeField] MultiNavigationController multiNavigationController;
    bool isMouseButtonDown;
    GameObject previousGoalMarkerObj;
    List<GameObject> previousGoalMarkerObjs = new List<GameObject>();
    Vector3 raycastHitPos;
    Vector3 raycastHitPointOnMouseButtonDown;
    Quaternion arrowRotation;
    Vector3 goalPosition;
    Quaternion goalRotation; 
    PoseMsg goalPose;
    public PoseMsg GoalPose {
        get { return goalPose; }
    }
    
    void Start()
    {
        cursorObj.SetActive(false);
        arrowObj.SetActive(false);
        for (int i = 0; i < multiNavigationController.NavigationRobots.Count; i++)
        {
            previousGoalMarkerObjs.Add(new GameObject());
        }
    }

    void Update()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit[] hits = Physics.RaycastAll(ray);
        RaycastHit mapHit = hits.FirstOrDefault(hit => hit.collider.gameObject == mapObj);
        if (mapHit.collider != null)
        {
            cursorObj.SetActive(true);
            var p = mapHit.point;
            p.y = 0f;
            raycastHitPos = p;
        } else {
            cursorObj.SetActive(false);
            return;
        }
        // カーソールの位置を示すオブジェクトの座標を更新
        cursorObj.transform.position = isMouseButtonDown ? raycastHitPointOnMouseButtonDown : raycastHitPos;

        if (Input.GetMouseButtonDown(0))
        {
            // ボタン等のUIを操作中は除外
            if (EventSystem.current.IsPointerOverGameObject()) return;
            raycastHitPointOnMouseButtonDown = raycastHitPos;
            // 向きを決めるための矢印を表示する
            arrowObj.SetActive(true);
            arrowObj.transform.position = raycastHitPos;
            isMouseButtonDown = true;
        }
        
        if (isMouseButtonDown)
        {
            RotateArrowObj();

            if (Input.GetMouseButtonUp(0))
            {
                CreateGoalPoseMsg();
                arrowObj.SetActive(false);
                // ゴールのマーカーを作成
                GameObject goalMarkerObj = Instantiate(goalMarkerPrefab, raycastHitPointOnMouseButtonDown, goalRotation);
                goalMarkerObj.GetComponentInChildren<Image>().color = multiNavigationController.SelectedRobot.color;
                var index = multiNavigationController.NavigationRobots.IndexOf(multiNavigationController.SelectedRobot);
                if (previousGoalMarkerObjs[index] != null) Destroy(previousGoalMarkerObjs[index]);
                previousGoalMarkerObjs[index] = goalMarkerObj;
                isMouseButtonDown = false;
            }
        }
    }

    void RotateArrowObj() {
        var cursorDirection = raycastHitPos - arrowObj.transform.position;
        cursorDirection.y = 0;
        arrowRotation = (cursorDirection == Vector3.zero) ? Quaternion.identity : Quaternion.LookRotation(cursorDirection, Vector3.up);
        arrowObj.transform.rotation = Quaternion.Lerp(arrowObj.transform.rotation, arrowRotation, 0.1f);
    }
    
    void CreateGoalPoseMsg() {
        goalPosition = raycastHitPointOnMouseButtonDown;
        // マップの原点だけずらす
        goalPosition.x += mapTransformer.OriginPos.x;
        goalPosition.z += mapTransformer.OriginPos.z;
        goalRotation = arrowRotation;
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

        goalPosRotText.text = "Goal: \n{\n" + 
        "Pos: " + rosPosition.ToString("f2") + ",\n" + 
        "Rot: " + rosRotation.ToString("f2") + "\n}";
    }
}
