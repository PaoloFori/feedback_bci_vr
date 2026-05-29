# Feedback BCI VR

This package acts as the core protocol controller and visualization bridge for the Brain-Computer Interface Virtual Reality experiments. It manages the strict experimental timeline on the **ROS side** and provides the immersive graphic components on the **Unity (VR) side**.

---

## 🧠 ROS Domain (Protocol Manager)
Within the ROS ecosystem, this package hosts the state-machine logic that orchestrates the flow of the BCI experiment.

### Key Nodes
* **`training_node` (C++)**: The backbone of the experimental protocol. Its responsibilities dynamically change based on whether it is running in `calibration` or `evaluation` modality.
  
  **Topic Interactions**:
  * **(Publishes)** `/events/bus` (`rosneuro_msgs::NeuroEvent`): Used in all modalities to broadcast state-machine markers (e.g., `781` for continuous feedback, `786` for fixation, Hit `897`, Miss `898`, Timeout `899`).
  * **(Subscribes)** `/vr/status/ready` (`std_msgs::Bool`): Used in all modalities to block the protocol execution until the VR environment signals it is fully loaded.

  **Modality: Calibration**
  * In this mode, the subject is meant to train the system or acclimate to the environment.
  * The node **fakes the classifier output**. It employs mathematical autopilots (`LinearPilot` for active tasks, `SinePilot` for rest) to synthetically generate optimal probability streams over time.
  * Thresholds are hardcoded to `[1.0, 1.0]` — the autopilot drives probabilities from `0.5` to `1.0`, so the cube reaches the visual goal exactly when `is_target_hit` fires.
  * **(Publishes)** `/{paradigm}/neuroprediction/integrated/raw` (`rosneuro_msgs::NeuroOutput`): The autopilot-generated raw probabilities.
  * **(Publishes)** `/{paradigm}/neuroprediction/integrated/normalized` (`rosneuro_msgs::NeuroOutput`): Per-class linear stretch of the raw probabilities using `normalize_input`. With calibration thresholds `[1.0, 1.0]`, slope = 1 (passthrough), so normalized = raw.

  **Modality: Evaluation**
  * In this mode, the user actively controls the BCI. The system relies on real-time signal processing, sLDA classification, and the `rosneuro_integrator` node.
  * **(Subscribes)** `/{paradigm}/neuroprediction/integrated/raw` (`rosneuro_msgs::NeuroOutput`): Listens to the buffer state published by `rosneuro_integrator`.
  * Reads `thresholds` from the ROS parameter server (passed from `evaluation.launch`). Evaluates when `raw[i] >= thresholds[i]` to declare Hit, Miss, or Timeout.
  * The normalized topic (`/{paradigm}/neuroprediction/integrated/normalized`) is published directly by `rosneuro_integrator` and consumed by Unity — `training_node` does not publish it in evaluation mode.

  **Normalization (`normalize_input`)**
  * Applied in calibration mode when publishing to the normalized topic.
  * Formula (per class `i`, with `p_rest = 1/n_classes`):
    ```
    slope_i = (1 - p_rest) / (threshold_i - p_rest)
    normalized_i = clamp(p_rest + (raw_i - p_rest) * slope_i, 0, 1)
    ```
  * Maps `raw_i = threshold_i → normalized_i = 1.0`. Identical to the formula used by `rosneuro_integrator`, ensuring visual and detection consistency.

* **`scene_manager.py`**: A lifecycle utility node that reads the user's configured `paradigm` at launch and signals the Unity application to dynamically load the appropriate Scene via the `/vr/load_scene` topic.

**Launch configuration (`evaluation.launch`)**
The `thresholds` parameter (e.g., `[0.95, 0.75]`) is passed to both:
* `integrator` node: used to compute `integrated/normalized` (raw = threshold → normalized = 1.0).
* `training_node`: used by `is_target_hit` to check `raw >= threshold`.

---

## 🕶️ Unity Domain (Virtual Reality)
The directory `unity_project_bci_pico4` contains the complete Unity project designed to render zero-latency VR feedback. It communicates bidirectionally with ROS using the `ros_tcp_endpoint`.

### VR Mechanisms & User Feedback
* **ROS Integration**: The Unity application maintains a robust WebSocket/TCP connection with the ROS Master to receive both discrete state-machine transitions (from the `training_node`) and continuous probability vectors (from the integrator).
* **Normalized probability mapping**: Unity subscribes to `/{paradigm}/neuroprediction/integrated/normalized`. The formula `offset = max(0, (normalized − 0.5) × 2)` maps `normalized ∈ [0.5, 1.0]` to a visual range `[0, 1]`. The cube (or material/audio) reaches its goal position exactly when `normalized = 1.0`, which coincides with `raw = threshold` — the same moment `training_node` detects a Hit.
* **Cube initial positions**: Read from the Unity editor at `Start()` via `cuboL.transform.localPosition` and `cuboR.transform.localPosition`. Positions can be adjusted in the Inspector without modifying the script.
* **Dynamic Visual Feedback (The Cubes)**: The core interaction revolves around **two dynamic cubes** (positioned respectively on the left and right sides) that provide continuous, real-time feedback to the user based on the selected paradigm:
  * **Motor Imagery (MI)**: The user applies spatial motor imagery to control the vertical axis. A successful MI intent makes the selected target cube **move up or down** toward its destination.
  * **Covert Visual Spatial Attention (CVSA)**: The user focuses visual attention spatially toward the left or the right cube. This focus controls the transparency/alpha channel. A successful CVSA hit causes the target cube to **materialize** (become fully opaque) from a transparent state.
  * **Hybrid Approach**: Fuses both challenges simultaneously. The user must sustain peripheral CVSA focus on the target side (left or right) to **materialize and visually lock** the corresponding cube, while simultaneously applying MI to **translate it vertically** toward the target position.

### VR Device & Unity Configuration
* **Tested Hardware:** The entire tracking and feedback system has been developed and comprehensively tested using a **Pico 4** VR headset.
* **OpenXR & Project Settings:**
    1. Go to `Edit` -> `Project Settings` -> `XR Plugin Management`.
    2. In the **Windows (PC)** tab, ensure **OpenXR** is selected.
    3. Under OpenXR, make sure to add a compatible **Controller Profile** (e.g., *Oculus Touch Controller Profile*, which maps correctly for the Pico 4 controllers).
    4. If streaming from a PC, ensure your chosen VR runtime (e.g., SteamVR or Pico Streaming Assistant) is active before hitting Play.
* **Audio Setup:** To correctly hear the spatial auditory feedback and event sound cues when testing directly inside the Unity Editor, make sure to deactivate the **"Mute Audio"** button (or select "Unmute") located at the top of the **Game** window tab.
* **Scene Adjustments:** Some minor environmental tweaks might be necessary depending on the subject's anatomy and sitting position. For instance, adjusting the vertical position of the **"Finish bar"** in the Inspector for the **MI** and **Hybrid** paradigms to ensure they represent a comfortable visual threshold.
* **Network Setup:** Ensure that the IP address perfectly matches between the **'Robotics'** / **'ROS_manager'** script fields inside the Unity scene and the `ros_tcp_endpoint` on the ROS side to guarantee seamless telemetry rendering.
* **Subject Calibration (`VRCalibrationManager`)**: A `manager_calibration` GameObject (with `VRCalibrationManager` script and a `target_head` reference) is present in all 4 scenes at identical world-space positions, ensuring consistent calibration across scenes. Key bindings:
  * `Space` — recenter: aligns `XR Origin` so the virtual head position and orientation match `target_head`.
  * `↑ / ↓` — move forward / backward toward the table.
  * `E / Q` — raise / lower the player height.
  * `Enter` — save calibration to `PlayerPrefs` (persists across scenes and sessions).
  
  Calibration workflow per session: enter any BCI scene → `Space` → fine-tune with `↑↓ E Q` → `Enter`. On subsequent entries the saved position is auto-restored by `Start()`.
