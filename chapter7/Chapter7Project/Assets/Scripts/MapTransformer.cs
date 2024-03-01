using UnityEngine;

/// <summary>
/// マップの見え方を調整するスクリプト
/// </summary>
public class MapTransformer : MonoBehaviour
{
    [SerializeField] GameObject mapObj;
    [Header("マップのメタ情報")]
    [SerializeField] Vector2 pixelSize;
    [SerializeField] float resolution;
    [SerializeField] Vector2 origin;
    Vector3 originPos;
    void Start()
    {
        ReadMap();
    }

#if UNITY_EDITOR
    [ContextMenu("Transform Map")]
    void ReadMap()
    {
        var scaleX = resolution * pixelSize.x * 0.1f;
        var scaleZ = resolution * pixelSize.y * 0.1f;
        mapObj.transform.localScale = new Vector3(scaleX, 1f, scaleZ);
        mapObj.transform.rotation = Quaternion.Euler(new Vector3(0f, 90f, 0f));
        originPos = new Vector3(
            ((pixelSize.y / 2f * resolution) + (origin.y)) * -1f,
            0f,
            ((pixelSize.x / 2f * resolution) + (origin.x))
        );
        Debug.Log("Map Origin: " + originPos);
        mapObj.transform.position = originPos;
    }
#endif
}