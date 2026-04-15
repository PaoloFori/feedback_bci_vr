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
  * The node **Fakes the Classifier Output**. It employs mathematical autopilots (`LinearPilot` for active tasks, `SinePilot` for rest) to synthetically generate optimal probability streams over time.
  * **(Publishes)** `/{paradigm}/neuroprediction/integrated/normalized` (`rosneuro_msgs::NeuroOutput`): Streams the locally faked probabilities into the VR environment.

  **Modality: Evaluation**
  * In this mode, the user actively controls the BCI, and the system relies entirely on real-time continuous signal processing, QDA classification, and integration. 
  * **(Subscribes)** `/{paradigm}/neuroprediction/integrated/normalized` (`rosneuro_msgs::NeuroOutput`): Listens to the true output arriving from the `rosneuro_integrator`.
  * Evaluates when the received probabilities exceed the configured thresholds to dynamically declare a target Hit or Timeout.

* **`scene_manager.py`**: A lifecycle utility node that reads the user's configured `paradigm` at launch and signals the Unity application to dynamically load the appropriate Scene via the `/vr/load_scene` topic.

---

## 🕶️ Unity Domain (Virtual Reality)
The directory `unity_project_bci_pico4` contains the complete Unity project designed to render zero-latency VR feedback. It communicates bidirectionally with ROS using the `ros_tcp_endpoint`.

### VR Mechanisms & User Feedback
* **ROS Integration**: The Unity application maintains a robust WebSocket/TCP connection with the ROS Master to receive both discrete state-machine transitions (from the `training_node`) and continuous probability vectors (from the QDA classifiers).
* **Dynamic Visual Feedback (The Cubes)**: The core interaction revolves around **two dynamic cubes** (positioned respectively on the left and right sides) that provide continuous, real-time feedback to the user based on the selected paradigm:
  * **Motor Imagery (MI)**: The user applies spatial motor imagery to control the vertical axis. A successful MI intent makes the selected target cube **move up or down** toward its destination.
  * **Covert Visual Spatial Attention (CVSA)**: The user focuses visual attention spatially toward the left or the right cube. This focus controls the transparency/alpha channel. A successful CVSA hit causes the target cube to **materialize** (become fully opaque) from a transparent state.
  * **Hybrid Approach**: Fuses both challenges simultaneously. The user must sustain peripheral CVSA focus on the target side (left or right) to **materialize and visually lock** the corresponding cube, while simultaneously applying MI to **translate it vertically** toward the target position.

### Software Configuration
* **Unity Version:** Developed and tested on Unity **2022 LTS**.
* **VR Runtime:** Fully tested with **SteamVR** (via OpenXR plugin). Ensure the SteamVR runtime is active before launching the Unity project.

---

**Note on Setup**: Ensure your `ros_tcp_endpoint` IP matches the Unity network config (`ROS_manager`) before launching the system to guarantee seamless feedback rendering.
