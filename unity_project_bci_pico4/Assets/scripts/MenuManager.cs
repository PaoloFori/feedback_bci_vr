using UnityEngine;
using UnityEngine.SceneManagement;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;

public class MenuManager : MonoBehaviour {

    private bool shouldLoadScene = false;
    private string targetScene = "";

    void Start() {
        ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>("/vr/load_scene", OnRosCommandReceived);
    }
    
    public void LoadMI() {
        SceneManager.LoadScene("BCI_MI"); 
    }

    public void LoadCVSA() {
        SceneManager.LoadScene("BCI_CVSA");
    }

    public void LoadHybrid() {
        SceneManager.LoadScene("BCI_HYBRID");
    }

    void OnRosCommandReceived(StringMsg msg) {
        targetScene = msg.data;
        if (!string.IsNullOrEmpty(targetScene) && targetScene != "Menu") {
            shouldLoadScene = true; 
        }
    }

    void Update() {
        if (shouldLoadScene) {
            shouldLoadScene = false;
            SceneManager.LoadScene(targetScene);
        }
    }
}