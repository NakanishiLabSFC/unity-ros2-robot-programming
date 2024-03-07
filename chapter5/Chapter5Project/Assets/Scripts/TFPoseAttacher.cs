using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class TFPoseAttacher : MonoBehaviour
{
    [SerializeField] GameObject turtlebot3Obj;
    [SerializeField] MapTransformer mapTransformer;
    [SerializeField] string parentFrameName;
    [SerializeField] string targetFrameName;
    GameObject parentFrameObj;
    GameObject targetFrameObj;
    bool existParentFrame;
    
    void Update()
    {
        if (!existParentFrame) {
            parentFrameObj = GameObject.Find(parentFrameName);
            if (parentFrameObj != null) {
                // 親フレームのゲームオブジェクトが見つかった場合
                var objs = parentFrameObj.GetComponentsInChildren<Transform>();
                foreach (var obj in objs)
                {
                    if (obj.gameObject.name == targetFrameName) {
                        targetFrameObj = obj.gameObject;
                    }
                }
                existParentFrame = true;
            } 
        }

        if (targetFrameObj != null)
        {
            turtlebot3Obj.transform.position = targetFrameObj.transform.position - mapTransformer.OriginPos;
            turtlebot3Obj.transform.rotation = targetFrameObj.transform.rotation;
        }
    }
}
