using UnityEngine;

public class VRCalibrationManager : MonoBehaviour
{
    [Header("Setup VR")]
    [Tooltip("L'oggetto padre della telecamera (es. XR Origin o Camera Rig)")]
    public Transform xrOrigin;
    [Tooltip("La telecamera VR vera e propria (Main Camera)")]
    public Transform vrCamera;
    [Tooltip("Il punto esatto e la direzione in cui vuoi che stia la testa (es. TargetTesta)")]
    public Transform targetHeadPosition;

    [Header("Impostazioni Movimento")]
    [Tooltip("Velocità con cui avvicini o allontani il giocatore")]
    public float moveSpeed = 0.5f;

    void Start()
    {
        // All'avvio, carichiamo SIA la posizione SIA la rotazione salvate!
        if (PlayerPrefs.HasKey("SavedPosX"))
        {
            // Ripristina Posizione
            Vector3 savedPos = new Vector3(
                PlayerPrefs.GetFloat("SavedPosX"),
                PlayerPrefs.GetFloat("SavedPosY"),
                PlayerPrefs.GetFloat("SavedPosZ")
            );
            xrOrigin.position = savedPos;

            // Ripristina Rotazione (Asse Y)
            Vector3 savedRot = xrOrigin.eulerAngles;
            savedRot.y = PlayerPrefs.GetFloat("SavedRotY");
            xrOrigin.eulerAngles = savedRot;

            Debug.Log("Calibrazione caricata: Posizione e Direzione ripristinate!");
        }
    }

    void Update()
    {
        // 1. RE-CENTER TOTALE (Tasto Spazio) - Centra posizione E direzione
        if (Input.GetKeyDown(KeyCode.Space))
        {
            RecenterCamera();
        }

        // 2. AVVICINARE IL GIOCATORE AL TAVOLO (Freccia Su)
        if (Input.GetKey(KeyCode.UpArrow))
        {
            xrOrigin.position += targetHeadPosition.forward * moveSpeed * Time.deltaTime;
        }

        // 3. ALLONTANARE IL GIOCATORE DAL TAVOLO (Freccia Giù)
        if (Input.GetKey(KeyCode.DownArrow))
        {
            xrOrigin.position -= targetHeadPosition.forward * moveSpeed * Time.deltaTime;
        }

        // 4. ALZARE IL GIOCATORE (E)
        if (Input.GetKey(KeyCode.E))
        {
            xrOrigin.position += Vector3.up * moveSpeed * Time.deltaTime;
        }

        // 5. ABBASSARE IL GIOCATORE (Q)
        if (Input.GetKey(KeyCode.Q))
        {
            xrOrigin.position -= Vector3.up * moveSpeed * Time.deltaTime;
        }

        // 6. SALVARE PER IL FUTURO (Tasto Invio / Enter)
        if (Input.GetKeyDown(KeyCode.Return))
        {
            SaveCalibration();
        }
    }

    void RecenterCamera()
    {
        if (xrOrigin != null && vrCamera != null && targetHeadPosition != null)
        {
            // STEP 1: ALLINEARE LA ROTAZIONE
            // Calcoliamo la differenza di gradi (sull'asse Y) tra dove guarda il giocatore e dove vogliamo che guardi
            float angleDifference = targetHeadPosition.eulerAngles.y - vrCamera.eulerAngles.y;
            // Ruotiamo la stanza (XR Origin) usando la testa del giocatore come "perno" centrale
            xrOrigin.RotateAround(vrCamera.position, Vector3.up, angleDifference);

            // STEP 2: ALLINEARE LA POSIZIONE
            // Calcoliamo lo spostamento necessario (da fare DOPO aver ruotato!)
            Vector3 offset = targetHeadPosition.position - vrCamera.position;
            xrOrigin.position += offset;

            Debug.Log("Centrato e orientato verso i cubi!");
        }
    }

    void SaveCalibration()
    {
        // Salva le coordinate spaziali
        PlayerPrefs.SetFloat("SavedPosX", xrOrigin.position.x);
        PlayerPrefs.SetFloat("SavedPosY", xrOrigin.position.y);
        PlayerPrefs.SetFloat("SavedPosZ", xrOrigin.position.z);
        
        // Salva la rotazione della stanza
        PlayerPrefs.SetFloat("SavedRotY", xrOrigin.eulerAngles.y);
        
        PlayerPrefs.Save(); 

        Debug.Log("<color=green>Posizione e Rotazione salvate con successo!</color>");
    }
}