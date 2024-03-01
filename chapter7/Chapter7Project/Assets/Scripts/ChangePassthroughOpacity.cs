using UnityEngine;

/// <summary>
/// パススルーの不透明度を変更するスクリプト
/// </summary>
public class ChangePassthroughOpacity : MonoBehaviour
{
    [SerializeField] OVRPassthroughLayer passthroughLayer;

    public void ChangeTransparency(float value)
    {
        passthroughLayer.textureOpacity = value;
    }
}