using UnityEngine;
using UnityEngine.UI;

// Turtlesimのカメラ画像を表示するスクリプト
public class TurtlesimCameraImageRenderer : MonoBehaviour
{
    [SerializeField] Camera cam;
    [SerializeField] Image targetImage;
    [SerializeField] LayerMask visibleLayerMask;
    RenderTexture renderTexture;
    Texture2D texture;

    void Start()
    {        
        renderTexture = new RenderTexture(Screen.width, Screen.height, 24);  // カメラの映像を取得するためのRenderTextureを作成
        cam.targetTexture = renderTexture;
        texture = new Texture2D(Screen.width, Screen.height, TextureFormat.RGBA32, false);  // テクスチャを作成
    }

    void Update()
    {
        cam.Render();  // カメラの画像をRenderTextureに描画
        // RenderTextureからテクスチャにデータを読み込む
        RenderTexture.active = renderTexture;
        texture.ReadPixels(new Rect(0, 0, Screen.width, Screen.height), 0, 0);
        texture.Apply();
        targetImage.sprite = Sprite.Create(texture, new Rect(0, 0, texture.width, texture.height), Vector2.one * 0.5f);  // Imageコンポーネントにテクスチャを設定
    }
}
