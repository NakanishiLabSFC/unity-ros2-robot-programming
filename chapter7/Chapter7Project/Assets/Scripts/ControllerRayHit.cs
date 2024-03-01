using UnityEngine;

/// <summary>
/// マップとコントローラーのレイの衝突を検出するスクリプト
/// </summary>
public class ControllerRayHit : MonoBehaviour
{
    [SerializeField] LayerMask collisionLayer;  // 衝突をチェックするレイヤーマスク
    [SerializeField] float maxDistance = 10f;  // レイの最大距離
    [SerializeField] Transform rayOrigin;
    [SerializeField] GameObject rayCastHitPointObj;
    GameObject rayCastHitPoint;
    Vector3 hitPosition;
    public Vector3 HitPosition {
        get { return hitPosition; }
    }
    
    void Start()
    {
        rayCastHitPoint = Instantiate(rayCastHitPointObj);
        rayCastHitPoint.SetActive(false);
    }

    void Update()
    {
        Vector3 rayDirection = rayOrigin.forward;
        RaycastHit hit;
        if (Physics.Raycast(rayOrigin.position, rayDirection, out hit, maxDistance, collisionLayer)) {
            hitPosition = hit.point;
            rayCastHitPoint.SetActive(true);
            rayCastHitPoint.transform.position = new Vector3 (hitPosition.x, 1f, hitPosition.z);
        } else {
            rayCastHitPoint.SetActive(false);
        }
    }
}