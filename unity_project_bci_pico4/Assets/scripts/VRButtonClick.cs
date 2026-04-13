using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.UI;

public class VRButtonClick : MonoBehaviour
{
    private XRRayInteractor rayInteractor;
    private bool triggerWasPressed = false;

    void Start()
    {
        rayInteractor = GetComponent<XRRayInteractor>();
    }

    void Update()
    {
        InputDevice rightHand = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        rightHand.TryGetFeatureValue(CommonUsages.triggerButton, out bool triggerPressed);

        if (triggerPressed && !triggerWasPressed)
        {
            if (rayInteractor.TryGetCurrentUIRaycastResult(out var result))
            {
                Button button = result.gameObject.GetComponentInParent<Button>();
                if (button != null)
                {
                    Debug.Log("Clicking: " + button.gameObject.name);
                    button.onClick.Invoke();
                }
            }
        }

        triggerWasPressed = triggerPressed;
    }
}