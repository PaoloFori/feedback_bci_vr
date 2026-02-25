using UnityEngine;

public class KeepAlive : MonoBehaviour {
    void Awake() {
        // Dice a Unity: "Non distruggere questo oggetto quando carichi una nuova scena"
        DontDestroyOnLoad(this.gameObject);
    }
}