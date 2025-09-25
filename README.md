## Welcome to the ISyHand Driver

This repository contains a C++ driver for the ISyHand, a ROS2 wrapper, and urdf model of the ISyHand.v6.
The ROS2 wrapper uses the associated urdf model in [assets](https://github.com/benrichardson28/isyhand_ros2/tree/master/assets) to set joint limits, efforts, and velocities in the driver. 

More information about the ISyHand can be found [here](https://isyhand.is.mpg.de/).
---

## Running the driver.
### Connect the ISyHand
- Power the hand and connect it via USB to the computer.
- Find the USB port using the [Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)

### On Ubuntu using docker (tested on Ubuntu 22)
Install [docker](https://docs.docker.com/engine/install/ubuntu/) and follow the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user).
#### Create a folder that will act as your ROS2 workspace if one doesn't exist:
- Clone this repository into the src folder of the workspace.
- From the repository directory, start and enter the docker container by running: `./src/isyhand_driver/scripts/run.sh <usb port from above, defaults to /dev/ttyUSB0>`. 
** `run.sh` automatically mounts the device to /dev/ttyUSB0 inside the container. **
- Run `colcon build` to build the package.
- Source the package with `source "$WORKSPACE_DIR/install/setup.bash"`.
**_NOTE_** If the package is already built, the docker entrypoint will automatically do the sourcing. 



#### How to use
- `ros2_example.py` is an example script that queries the LEAP service and also publishes out a pose for the hand to move to.  You can build off of this for your own project.
- For more info I recommend checking the ROS2 docs and the other docs for LEAP Hand.




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



