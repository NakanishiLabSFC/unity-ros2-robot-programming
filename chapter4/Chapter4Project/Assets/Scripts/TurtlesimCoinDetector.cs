using UnityEngine;
using TMPro;

/// <summary>
/// Turtlesimとコインの衝突処理を実行するスクリプト
/// </summary>
public class TurtlesimCoinDetector : MonoBehaviour
{
    [SerializeField] TextMeshProUGUI coinCountText;
    int coinCount;
    
    void OnTriggerEnter(Collider other) {
        GameObject target = other.gameObject;
        if (target.tag == "Coin") {
            target.GetComponent<Coin>().OnPickedUp();
            coinCount ++;
            coinCountText.text = coinCount.ToString();
        }
    }
}
