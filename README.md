## Welcome to the ISyHand Driver

This repository contains a C++ driver for the ISyHand, a ROS2 wrapper, and urdf model of the ISyHand.v6.
The ROS2 wrapper uses the associated urdf model in [assets](https://github.com/benrichardson28/isyhand_ros2/tree/master/assets) to set joint limits, efforts, and velocities in the driver. 

More information about the ISyHand can be found [here](https://isyhand.is.mpg.de/).
---

### Running the driver.
#### Connect the ISyHand
- Power the hand and connect it via USB to the computer.
- Find the USB port using the [Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/).

#### Run the docker container (tested on Ubuntu 22)
- Install [docker](https://docs.docker.com/engine/install/ubuntu/) and follow the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user).
- Clone this repository with the `--recurse-submodules` flag into `<workspace>/src` (workspace can be an existing ros2 workspace or a new folder). `git clone --recurse-submodules https://github.com/benrichardson28/isyhand_ros2.git <workspace>/src`
  > This installs Dynamixel SDK from https://github.com/ROBOTIS-GIT/DynamixelSDK.git (main branch; commit 54cabbf) and
  > the isyhand ros interface package. 
- From the repository directory, start and enter the docker container by running: `./src/isyhand_driver/scripts/run.sh <usb port from above, defaults to /dev/ttyUSB0>`. 
  > 1. To enter the container with another shell, simply call `run.sh` again.
  > 2. `run.sh` automatically mounts the device to /dev/ttyUSB0 inside the container.

#### Build the package
- Run `colcon build` to build the package.
- Source the package with `source install/setup.bash`.
  > If the package has already been built, the docker entrypoint will automatically source the package. 

#### How to use
- To launch the driver with the ros2 wrapper, run `ros2 launch isyhand_driver isyhand_driver.launch`.
- While the driver is running, running `ros2 launch isyhand_driver test_motion.launch` will generate a sinusoidal signal that will be sent to the joints. 

---
### Support
- Questions/issues: **richardson@is.mpg.de** or **grueninger@is.mpg.de**
- **License:**  
  - Code: MIT License  
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





