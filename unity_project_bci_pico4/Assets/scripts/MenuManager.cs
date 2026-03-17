using UnityEngine;
using UnityEngine.SceneManagement;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;

public class MenuManager : MonoBehaviour {

    private string ros_command = "";
    private bool has_new_command = false;

    void Start() {
        ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>("/vr/load_scene", OnRosCommandReceived);
    }

    void OnRosCommandReceived(StringMsg msg) {
        ros_command = msg.data;
        if (!string.IsNullOrEmpty(ros_command) && ros_command != "Menu") {
            has_new_command = true; 
        }
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

    void Update() {
        if (has_new_command) {
            has_new_command = false;
            ProcessRosCommand(ros_command);
        }
    }

    void ProcessRosCommand(string command) {
        Debug.Log("Scene to load: " + command);

        switch (command) {
            case "mi":
                LoadMI();
                break;
            case "cvsa":
                LoadCVSA();
                break;
            case "hybrid":
                LoadHybrid();
                break;
            default:
                Debug.LogWarning("ROS command not recognized: " + command);
                break;
        }
    }
}