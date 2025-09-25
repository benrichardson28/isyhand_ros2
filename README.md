## Welcome to the ISyHand Driver

👉 **More info:** [ISyHand Website](https://isyhand.is.mpg.de/)

---
This repository contains a C++ driver for the ISyHand, a ROS2 wrapper, and urdf model of the ISyHand.v6.
The ROS2 wrapper uses the associated urdf model in [assets]() to set joint limits, efforts, and velocities in the driver. 

### Software Setup



<!--
---

### 🔌 Hardware Setup
- Connect **5 V power** to the hand (Dynamixels should light up on boot).
- Connect the **Micro‑USB** cable (avoid multiple USB extensions).
- Use [Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/rplus1/dynamixel_wizard/) to find the correct port.  
  ➡️ Put that port into `main.py` or `ros_example.py`.  
  ⚠️ You **cannot** have Dynamixel Wizard open while using the API (the port will be busy).
- On Ubuntu, find the hand by ID at `/dev/serial/by-id` (persistent across reboots).
- `sudo chmod 666 /dev/serial/by-id/(your_id)` to give serial port permissions.
- Official support: **Python** and **C++** and **ROS/ROS2**. 
  Other languages can use the [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/).
- To improve latency on Ubuntu:  
  - [Adjust USB Latency Settings](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)  
  - Tune the [Dynamixel Python SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/288)  
  - Set *Return Delay Time* (Control Table Register 9) from 250 µs to **0 µs**.
- If you are using the full hand, you can raise the current limit from 300 mA to 550 mA in the API for increased strength!
---

### 🤖 Functionality
- Leap Node allows commanding joint angles in different scalings.
- You can read **position, velocity, and current**.
- **Query limits:**  
  - Position only: ≤ 500 Hz  
  - Position + velocity + current: ≤ 500 Hz  
  (Higher rates can slow USB communication.)
- Default control: **PID** (up to current limit).  
  Velocity and current control also supported—see the [motor manual](https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/).
- Current limits:  
  - Lite: ≈ 300 mA  
  - Full: up to ≈ 550 mA
  - **By default the API is at 300, you can raise it to 550mA on the Full hand!!!**
- Jittery hand? ➡️ Lower P/D values.  
  Weak hand? ➡️ Raise P/D values.

---
-->
### 🛠️ Troubleshooting
- Motor off by 90°/180°/270° → **Remount the horn.**
- No motors show up → Check **serial port permissions**.
- Some motors missing → Verify **IDs** and **U2D2 connections**.
- Overload error (motors flashing red) → **Power cycle**. If frequent, **lower current limits**.
- Jittery motors → Lower P/D values.
- Inaccurate motors → Raise P/D values.

---

### Support
- Questions/issues: **richardson@is.mpg.de** or **grueninger@is.mpg.de**
- **License:**  
  - Code: MIT License  
  - CAD: CC BY‑NC‑SA (non‑commercial use with attribution)
- Provided **as‑is**, without warranty.

**If you use ISyHand in research, please cite:**
```bibtex
@inproceedings{Richardson25-HR-ISyHand,
  title = {ISyHand: A Dexterous Multi-finger Robot Hand with an
Articulated Palm},
  booktitle = {Proceedings of the IEEE-RAS International
Conference on Humanoid Robots (Humanoids)},
  address = {Seoul, Korea},
  month = sep,
  year = {2025},
  note = {Benjamin A. Richardson, Felix Gr{\"u}ninger and Lukas Mack
contributed equally to this publication},
  slug = {richardson25-hr-isyhand},
  author = {Richardson, Benjamin A. and Gr{\"u}ninger, Felix and
Mack, Lukas and Stueckler, Joerg and Kuchenbecker, Katherine J.},
  month_numeric = {9}
}

