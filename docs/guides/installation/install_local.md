# Local Installation

This guide is designed to set up an environment using Windows and WSL2. 

!!! warning
    Marked as obsolete and not expected to work. Currently, Windows only

## Requirements

This guide expects you to have a working Formula-Student-Driverless-Simulator Installation up and running.
If that is not the case, please follow the instructions [here](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.2.0/getting-started/#from-release-binaries). We advise you to install from release binaries.

You also need a GitLab-Account using your @strohmleitung-Mail. Please register [here](https://gitlab.com/users/sign_up) and send your username to the "Head of Driverless". He will clear you for the Strohm-und-Söhne Developing Group. 

## Install WSL2

***If you are using Linux, you can skip this step.***

Open a PowerShell-Console with elevated rights and run this command:

```ps1
wsl --install -d Ubuntu-22.04
```

You now can connect to your Linux Distro with:

```ps1
wsl
```

Run Upgrade and Update on your Linux distribution.

```bash
sudo apt update
sudo apt upgrade
```

Make sure you have the latest version, ```Ubuntu 22.04 Jammy```, running.

```bash
lsb_release -a
```

## Install ROS2 on WSL-Ubuntu

Please ensure to have a running environment of the humble-hawksbill ROS2 distribution.

If not, follow the instructions [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#) 

Also configure your environment as described [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)

## Running this Project

Clone the repository with following command (on your respective branch):

!!! note 
    Make sure to clone this Repository into your user-home-directory (~)

```bash
cd ~
git clone --recurse-submodules -b <branch> https://gitlab.com/strohm-und-soehne/driverless/autonomous-system.git
```

!!! note
    Please replace ```<branch>``` by your desired branch to clone - default is ```main```

We need to install AirSim first.

```bash
cd ~/autonomous-system/src/fsds/fsds_ros2_bridge/assets/
AirSim/setup.sh
```

Now we need to build our ROS2-Workspace.

!!! note
    If you use Linux, you may want to change the default "host"-parameter in the /fsds_ros2_bridge/launch/fsds_ros2_bridge.launch.py to "localhost"

```bash
cd ~/autonomous-system
colcon build
```
!!! note 
    When rebuilding the Workspace, always make sure you are in the root folder of this project (~/autonomous-system).

Despite some Error-Output for package fsds_ros2_bridge, your project should now look like this:

```
├── build
├── install
├── log
├── src
```

Before you are able to start any Nodes, you need to run:
```bash
source install/setup.bash
```
!!! note 
    Replace .bash with your Shell (e.g. .zsh .sh)

You are now ready and set to run the bridge using:
```bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py
```
!!! note 
    Make sure your Simulator is up and running in the background

!!! tip

    If you don't want to source your setup-file every time you open a new Terminal, run this:
    ```bash 
    echo "source ~/autonomous-system/install/setup.bash" >> ~/.bashrc
    ```
    !!! note 
        Replace .bash with your Shell (e.g. .zsh .sh)