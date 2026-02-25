using UnityEngine;
using UnityEngine.SceneManagement;

public class MenuManager : MonoBehaviour {
    
    public void LoadMI() {
        SceneManager.LoadScene("BCI_MI"); 
    }

    public void LoadCVSA() {
        SceneManager.LoadScene("BCI_CVSA");
    }

    public void LoadHybrid() {
        SceneManager.LoadScene("BCI_HYBRID");
    }
}