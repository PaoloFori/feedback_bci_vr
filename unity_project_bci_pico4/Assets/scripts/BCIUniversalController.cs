using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using NeuroOutput = RosMessageTypes.Rosneuro.NeuroOutputMsg;
using EventMsg = RosMessageTypes.Rosneuro.NeuroEventMsg;

public class BCIUniversalController : MonoBehaviour{
    // paradigm and objects
    public enum Paradigma { MI, CVSA, Hybrid}
    public Paradigma tipoParadigma;
    public GameObject cuboL, cuboR;
    public GameObject fixationCross;
    public GameObject centerPoint;
    private Material matL, matR;
    private AudioSource audioL, audioR;
    public AudioSource audioSourceCues;
    public AudioClip audioCueLeft;
    public AudioClip audioCueRight;
    public AudioClip audioCueRest;

    // boolean for events
    private bool inCF = false;
    
    // parameters for feedback
    [Header("Parametri Fluidità")]
    public float smoothTime = 0.15f; 

    private float targetL, targetR;
    private float currentL, currentR;
    private float velocityL, velocityR; 

    // code for the events
    private const int OFF = 32768;
    private const int FIXATION = 786;
    private const int HIT = 897;
    private const int MISS = 898;
    private const int TIMEOUT = 899;
    private const int CUE_BL_CVSA = 730; // bottom left
    private const int CUE_BR_CVSA = 731; // bottom right
    private const int CUE_LH_MI = 769; // left hand motor imagery
    private const int CUE_RH_MI = 770; // right hand motor imagery
    private const int CUE_BH = 771; // both hands
    private const int CUE_BF = 773; // both feet
    private const int CF = 781;
    private const int REST = 783;

    void Start(){
        ROSConnection.GetOrCreateInstance().Subscribe<NeuroOutput>("/cvsa/neuroprediction/integrated", callback_data);
        ROSConnection.GetOrCreateInstance().Subscribe<EventMsg>("/events/bus", callback_events);

        // check if the objects are assigned
        if (cuboL == null || cuboR == null || fixationCross == null || centerPoint == null 
            || audioSourceCues == null || audioCueLeft == null || audioCueRight == null 
            || audioCueRest == null){
            Debug.LogError("Please assign all GameObjects in the inspector.");
            return;
        }

        // memorize elements for performance
        matL = cuboL.GetComponent<MeshRenderer>().material;
        matR = cuboR.GetComponent<MeshRenderer>().material;

        // At the beginning, ensure all hide
        fixationCross.SetActive(false);
        centerPoint.SetActive(false);
        ResetPosition();
        switch (tipoParadigma){
            case Paradigma.MI:
                ResetMaterial_MI();
                break;
            case Paradigma.CVSA:
            case Paradigma.Hybrid:
                audioL = cuboL.GetComponent<AudioSource>();
                audioR = cuboR.GetComponent<AudioSource>();
                ResetAudio();
                ResetMaterial_CVSA();
                break;
        }
    }

    void callback_data(NeuroOutput msg){
        if (msg.softpredict.data.Length < 2) return;
        targetL = (float)msg.softpredict.data[0];
        targetR = (float)msg.softpredict.data[1];
    }

    void callback_events(EventMsg msg){
        int code = msg.@event;

        switch (code){
            case FIXATION:
                fixationCross.SetActive(true);
                break;
            case OFF + FIXATION:
                fixationCross.SetActive(false);
                break;
            case CUE_LH_MI:
            case CUE_BL_CVSA:
            case CUE_BH:
                centerPoint.SetActive(true);
                audioSourceCues.PlayOneShot(audioCueLeft);
                break;
            case CUE_RH_MI:
            case CUE_BR_CVSA:
            case CUE_BF:
                centerPoint.SetActive(true);
                audioSourceCues.PlayOneShot(audioCueRight);
                break;
            case REST:
                centerPoint.SetActive(true);
                audioSourceCues.PlayOneShot(audioCueRest);
                break;
            case CF:
                inCF = true;
                break;
            case OFF + CF:
                inCF = false;
                break;
            case HIT:
                centerPoint.GetComponent<UnityEngine.UI.Image>().color = Color.green;
                break;
            case MISS:
                centerPoint.GetComponent<UnityEngine.UI.Image>().color = Color.red;
                break;
            case TIMEOUT:
                centerPoint.GetComponent<UnityEngine.UI.Image>().color = Color.yellow;
                break;
            case OFF + HIT:
            case OFF + MISS:
            case OFF + TIMEOUT:
                centerPoint.SetActive(false);
                centerPoint.GetComponent<UnityEngine.UI.Image>().color = Color.white;
                switch (tipoParadigma){
                    case Paradigma.MI:
                        ResetPosition();
                        break;
                    case Paradigma.CVSA:
                        ResetAudio();
                        ResetMaterial_CVSA();
                        break;
                    case Paradigma.Hybrid:
                        ResetPosition();
                        ResetAudio();
                        ResetMaterial_CVSA();
                        break;
                }
                break;
        }
    }

    // for unity update rate
    void Update(){

        // SmoothDamp calcola la transizione perfetta
        currentL = Mathf.SmoothDamp(currentL, targetL, ref velocityL, smoothTime);
        currentR = Mathf.SmoothDamp(currentR, targetR, ref velocityR, smoothTime);

        // execute application of the paradigm
        if(inCF){
            switch (tipoParadigma){
                case Paradigma.MI:
                    ApplyVerticalMovement(currentL, currentR);
                    break;
                case Paradigma.CVSA:
                    ApplyMaterial(currentL, currentR);
                    ApplyAudio(currentL, currentR);
                    break;
                case Paradigma.Hybrid:
                    ApplyVerticalMovement(currentL, currentR);
                    ApplyMaterial(currentL, currentR);
                    ApplyAudio(currentL, currentR);
                    break;
            }
        }
    }


    void ResetAudio(){
        audioL.volume = 0f;
        audioR.volume = 0f;
    }

    void ResetMaterial_MI(){
        matL.SetFloat("_Sharpness", 1.0f);
        matR.SetFloat("_Sharpness", 1.0f);
    }

    void ResetMaterial_CVSA(){
        matL.SetFloat("_Sharpness", 0.5f);
        matR.SetFloat("_Sharpness", 0.5f);
    }

    void ResetPosition(){
        cuboL.transform.localPosition = new Vector3(-1.5f, 1.65f, 4f);
        cuboR.transform.localPosition = new Vector3(1.5f, 1.65f, 4f);
    }

    void ApplyVerticalMovement(float L, float R) {
        float escursione = 1.0f;
        float offsetL = Mathf.Max(0, (L - 0.5f) * 2.0f); 
        float offsetR = Mathf.Max(0, (R - 0.5f) * 2.0f);

        float yL = 1.65f + (offsetL * escursione);
        float yR = 1.65f + (offsetR * escursione);

        cuboL.transform.localPosition = new Vector3(-1.5f, yL, 4f);
        cuboR.transform.localPosition = new Vector3(1.5f, yR, 4f);
    }

    void ApplyMaterial(float L, float R){
        matL.SetFloat("_Sharpness", L);
        matR.SetFloat("_Sharpness", R);
    }

    void ApplyAudio(float L, float R){
        audioL.volume = L;
        audioR.volume = R;
    }
}