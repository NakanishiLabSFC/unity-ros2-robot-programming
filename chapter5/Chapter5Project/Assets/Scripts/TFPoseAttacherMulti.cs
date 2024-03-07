using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TFPoseAttacherMulti : MonoBehaviour
{
    // [SerializeField] GameObject turtlebot3Obj;
    [SerializeField] MapTransformer mapTransformer;
    [SerializeField] MultiNavigationController multiNavigationController;
    // [SerializeField] string parentFrameName;
    [SerializeField] string targetFrameName;
    // GameObject parentFrameObj;
    // GameObject targetFrameObj;
    // public class SimulatorInfo {
    //     public Transform baseTransform = new GameObject().transform;
    //     public bool receivedFisrtMsg;
    //     public Pose initialPose;
    //     public Pose pose = new Pose();
    // }
    // List<SimulatorInfo> simulatorInfos;

    void Start() {
        // simulatorInfos = new List<SimulatorInfo>(multiNavigationController.NavigationRobots.Count);
        // for (int i = 0; i < multiNavigationController.NavigationRobots.Count; i++)
        // {
        //     simulatorInfos.Add(new SimulatorInfo());
        // }
    }
    
    void Update()
    {
        foreach (var robot in multiNavigationController.NavigationRobots)
        {
            if (robot.ParentFrameObj == null) {
                var parentFrameObj = GameObject.Find(robot.rosNamespace);

                // 親フレームのゲームオブジェクトが見つかった場合
                if (parentFrameObj != null) {
                    robot.ParentFrameObj = parentFrameObj;

                    var childrenObjs = parentFrameObj.GetComponentsInChildren<Transform>();

                    
                    foreach (var obj in childrenObjs)
                    {
                        if (obj.gameObject.name == targetFrameName) {
                            robot.FrameObj = obj.gameObject;
                        }
                    }
                    
                }
            }
        }


        foreach (var robot in multiNavigationController.NavigationRobots)
        {
            if (robot.ParentFrameObj ==  null || robot.FrameObj == null) continue;

            robot.turtlebot3Obj.transform.position = robot.FrameObj.transform.position - mapTransformer.OriginPos;
            robot.turtlebot3Obj.transform.rotation = robot.FrameObj.transform.rotation;
        }
        
        

            
            
        
    }
}
