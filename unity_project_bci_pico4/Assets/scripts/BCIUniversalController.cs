using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections;
using RosMessageTypes.Std;
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

    private bool has_new_input = false;

    private const float init_percentage = 0.5f;
    private readonly Vector3 initPosL = new Vector3(-1.5f, 1.65f, 4f);
    private readonly Vector3 initPosR = new Vector3(1.5f, 1.65f, 4f);

    // code for the events
    private const int OFF           = 32768;
    private const int FIXATION      = 786;
    private const int HIT           = 897;
    private const int MISS          = 898;
    private const int TIMEOUT       = 899;
    private const int CUE_BL_CVSA   = 730; // bottom left
    private const int CUE_BR_CVSA   = 731; // bottom right
    private const int CUE_LH_MI     = 769; // left hand motor imagery
    private const int CUE_RH_MI     = 770; // right hand motor imagery
    private const int CUE_BH_MI     = 771; // both hands
    private const int CUE_BF_MI     = 773; // both feet
    private const int CUE_L_CVSA_MI = 750; // left MI and CVSA
    private const int CUE_R_CVSA_MI = 751; // right MI and CVSA
    private const int CF            = 781;
    private const int REST          = 783;

    void Start(){
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
                ROSConnection.GetOrCreateInstance().Subscribe<NeuroOutput>("/mi/neuroprediction/integrated/normalized", callback_data);
                ResetMaterial_MI();
                break;
            case Paradigma.CVSA:
                ROSConnection.GetOrCreateInstance().Subscribe<NeuroOutput>("/cvsa/neuroprediction/integrated/normalized", callback_data);
                audioL = cuboL.GetComponent<AudioSource>();
                audioR = cuboR.GetComponent<AudioSource>();
                audioL.spatialBlend = 0.0f;
                audioR.spatialBlend = 0.0f;
                audioL.panStereo = -1f;  
                audioR.panStereo =  1f;
                ResetAudio();
                ResetMaterial_CVSA();
                break;
            case Paradigma.Hybrid:
                ROSConnection.GetOrCreateInstance().Subscribe<NeuroOutput>("/hybrid/neuroprediction/integrated/normalized", callback_data);
                audioL = cuboL.GetComponent<AudioSource>();
                audioR = cuboR.GetComponent<AudioSource>();
                audioL.spatialBlend = 0.0f;
                audioR.spatialBlend = 0.0f;
                audioL.panStereo = -1f;  
                audioR.panStereo =  1f;
                ResetAudio();
                ResetMaterial_MI();
                break;
        }

        // set VR ready
        ROSConnection.GetOrCreateInstance().RegisterPublisher<BoolMsg>("/vr/status/ready");
        StartCoroutine(SendReadySignalDelayed());
    }

    IEnumerator SendReadySignalDelayed(){
        yield return new WaitForSeconds(1.0f);
        
        BoolMsg msg = new BoolMsg(true);
        ROSConnection.GetOrCreateInstance().Publish("/vr/status/ready", msg);
    }

    void callback_data(NeuroOutput msg){
        if (msg.softpredict.data.Length < 2) return;
        targetL = (float)msg.softpredict.data[0];
        targetR = (float)msg.softpredict.data[1];

        has_new_input = true;
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
            case CUE_L_CVSA_MI:
            case CUE_BH_MI:
                centerPoint.SetActive(true);
                audioSourceCues.PlayOneShot(audioCueLeft);
                break;
            case CUE_RH_MI:
            case CUE_BR_CVSA:
            case CUE_R_CVSA_MI:
            case CUE_BF_MI:
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
                        ResetMaterial_MI();
                        break;
                }
                break;
        }
    }

    // for unity update rate
    void Update(){

        if (!inCF) return;

        // SmoothDamp calcola la transizione perfetta
        currentL = Mathf.SmoothDamp(currentL, targetL, ref velocityL, smoothTime);
        currentR = Mathf.SmoothDamp(currentR, targetR, ref velocityR, smoothTime);

        // execute application of the paradigm
        if(has_new_input){
            has_new_input = false;
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
                    // ApplyMaterial(currentL, currentR);
                    ApplyAudio(currentL, currentR);
                    break;
            }
        }
    }


    void ResetAudio(){
        currentL = 0f;
        currentR = 0f;
        targetL = 0f;
        targetR = 0f;
        audioL.volume = 0f;
        audioR.volume = 0f;
    }

    void ResetMaterial_MI(){
        matL.SetFloat("_Sharpness", 1.0f);
        matR.SetFloat("_Sharpness", 1.0f);
    }

    void ResetMaterial_CVSA(){
        currentL = init_percentage;
        currentR = init_percentage;
        targetL = init_percentage;
        targetR = init_percentage;
        matL.SetFloat("_Sharpness", init_percentage);
        matR.SetFloat("_Sharpness", init_percentage);
    }

    void ResetPosition(){
        currentL = init_percentage;
        currentR = init_percentage;
        targetL = init_percentage;
        targetR = init_percentage;
        velocityL = 0f;
        velocityR = 0f;
        cuboL.transform.localPosition = initPosL;
        cuboR.transform.localPosition = initPosR;
    }

    void ApplyVerticalMovement(float L, float R) {
        float escursione = 1.0f;
        float offsetL = Mathf.Max(0, (L - init_percentage) * (1.0f/init_percentage)); 
        float offsetR = Mathf.Max(0, (R - init_percentage) * (1.0f/init_percentage));

        float yL = initPosL.y + (offsetL * escursione);
        float yR = initPosR.y + (offsetR * escursione);

        cuboL.transform.localPosition = new Vector3(initPosL.x, yL, initPosL.z);
        cuboR.transform.localPosition = new Vector3(initPosR.x, yR, initPosR.z);
    }

    void ApplyMaterial(float L, float R){
        float max_value_L = Mathf.Max(L, init_percentage);
        float max_value_R = Mathf.Max(R, init_percentage);
        matL.SetFloat("_Sharpness", max_value_L);
        matR.SetFloat("_Sharpness", max_value_R);
    }

    void ApplyAudio(float L, float R){
        audioL.volume = Mathf.Max(0, (L - init_percentage) * (1.0f/init_percentage));
        audioR.volume = Mathf.Max(0, (R - init_percentage) * (1.0f/init_percentage));
    }
}