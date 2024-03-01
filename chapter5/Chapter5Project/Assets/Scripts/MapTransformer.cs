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
    [SerializeField] float mapSizeScale = 0.6f;
    Vector3 originPos;
    public Vector3 OriginPos {
        get { return originPos; }
    }

    void Start()
    {
        ReadMap();
        AdjustCamera();
    }

    #if UNITY_EDITOR
    [ContextMenu("Transform Map")]
    void ReadMap() {
        var scaleX = resolution * pixelSize.x * 0.1f;
        var scaleZ = resolution * pixelSize.y * 0.1f;
        mapObj.transform.localScale = new Vector3(scaleX, 1f, scaleZ);
        mapObj.transform.rotation = Quaternion.Euler(new Vector3(0f, 90f, 0f));
        originPos = new Vector3(
            ((pixelSize.y/2f*resolution)+(origin.y))*-1f, 
            0f, 
            ((pixelSize.x/2f*resolution)+(origin.x))
        );
        Debug.Log("Map Origin: " + originPos);
    }
    [ContextMenu("Adjust Camera")]
    void AdjustCamera() {
        Vector3 mapSize = mapObj.GetComponent<Renderer>().bounds.size;
        float cameraSize = Mathf.Max(mapSize.x * mapSizeScale, mapSize.y * mapSizeScale);
        Camera.main.orthographicSize = cameraSize;
    }
    #endif
}
