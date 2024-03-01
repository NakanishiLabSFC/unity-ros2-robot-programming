using UnityEngine;

/// <summary>
/// コイン本体のスクリプト
/// </summary>
public class Coin : MonoBehaviour
{
    [SerializeField] AudioClip pickedUpAudio;
    MeshRenderer meshRenderer;
    AudioSource audioSource;
    bool pickedUp;

    void Start()
    {
        audioSource = GetComponent<AudioSource>();
        meshRenderer = GetComponent<MeshRenderer>();
    }

    void Update() {
        transform.Rotate(0f, 0f ,180f*Time.deltaTime);  
    }

    public void OnPickedUp() {
        if (pickedUp) return;   // すでに取得済みの場合は実行しない
        pickedUp = true;
        audioSource.PlayOneShot(pickedUpAudio);
        meshRenderer.enabled = false;
        Destroy(gameObject, 1.5f);
    }
}
