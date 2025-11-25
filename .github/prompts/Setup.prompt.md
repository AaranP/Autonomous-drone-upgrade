---
mode: ask
---
Define the task to achieve, including specific requirements, constraints, and success criteria.

## Hardware Roles

### 1. Server / Onboard Computer / Drone Computer
*   **Device:** Raspberry Pi 5
*   **Operating System:** Ubuntu 25.10
*   **Role:** This device is physically onboard the drone. It functions as the **ROS Master host**. All ROS nodes running on the drone will connect to this ROS Master.

### 2. Client / Ground Station / Ground Computer
*   **Device:** A more powerful computer (e.g., desktop PC, laptop)
*   **Operating Systems:** Can be Windows, Linux, or Apple M-series (macOS).
*   **Role:** This computer serves as the ground control station. It will run client-side ROS nodes, visualization tools (like Rviz), and development environments. It connects to the ROS Master hosted on the Raspberry Pi 5.

## Development Focus

When making code changes, providing suggestions, or generating new content, please consider accommodating all mentioned platforms (Raspberry Pi 5, Windows, Linux, Apple M-series). However, the **primary build target and focus for development and Docker images is `amd64` architecture**. Where platform-specific instructions are necessary, please provide them for `amd64` first, followed by `arm64` (for Raspberry Pi) and other relevant architectures if applicable.