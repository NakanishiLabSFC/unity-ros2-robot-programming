using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using UnityEngine.UI;

/// <summary>
/// 画像のトピックをサブスクライブしてImageコンポーネントに表示するスクリプト
/// </summary>
public class ImageSubscriber : MonoBehaviour
{
    [SerializeField] Image targetImage;
    [SerializeField] string topicName;
    ROSConnection ros;
    Texture2D texture;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<CompressedImageMsg>(topicName, ReceiveMsg);
        texture = new Texture2D(1, 1);
    }

    void ReceiveMsg(CompressedImageMsg compressedImage)
    {
        byte[] imageData = compressedImage.data;
        RenderTexture(imageData);
    }

    void RenderTexture(byte[] data)
    {        
        texture.LoadImage(data);
        targetImage.sprite = Sprite.Create(texture, new Rect(0, 0, texture.width, texture.height), Vector2.one * 0.5f);
    }
}
