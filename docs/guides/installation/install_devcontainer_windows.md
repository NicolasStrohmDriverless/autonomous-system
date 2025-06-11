# DevContainer Installation

This guide is designed to set up an environment using Docker Desktop and Visual Studio Code inside of WSL2.
Using this setup is the best way to develop on Windows because it has the best performance and is the most convenient.

If you don't use Windows, please follow [this guide](install_devcontainer.md) instead.

## Requirements

This guide expects you to have a working Formula-Student-Driverless-Simulator Installation up and running.
If that is not the case, please follow the instructions [here](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.2.0/getting-started/#from-release-binaries). We advise you to install from release binaries.

You also need a GitLab-Account using your ```@strohmleitung```-Mail. Please register [here](https://gitlab.com/users/sign_up) and send your username to the "Head of Driverless". He will clear you for the Strohm-und-Söhne Developing Group. 

## Install Docker Desktop

Please install the respective Docker Desktop Application for [Windows](https://docs.docker.com/desktop/install/windows-install).

## Install WSL2

### Enable WSL2
Run the following command in PowerShell as Administrator to enable WSL2.
```bash
wsl --install
```
Now restart your Computer.

### Install Ubuntu-22.04
After that, run the following command in PowerShell to install the Ubuntu-22.04 Distribution.
```bash
wsl --install Ubuntu-22.04
```

### Set up Ubuntu-22.04
You will be prompted for a username and password. Please enter your desired username and password.

### Exiting the Ubuntu-Shell

If you want to exit the Ubuntu-Shell, just type ```exit``` and hit Enter.

### Set Ubuntu-22.04 as Default

If you want to set this Distribution as your default, run the following command in PowerShell (not WSL).
```bash
wsl --set-default Ubuntu-22.04
```

### Starting the Ubuntu-Shell

If you set the Ubuntu-22.04 as your default, you can just type ```wsl``` in PowerShell to start the Ubuntu-Shell.
Otherwise, you can start the Ubuntu-Shell by typing ```wsl -d Ubuntu-22.04``` in PowerShell.

### Update Ubuntu-22.04

After you started the Ubuntu-Shell, run the following commands to update the Ubuntu-22.04 Distribution.
```bash
sudo apt update
sudo apt upgrade
```

For more information on how to install Ubuntu, please follow the guide [here](https://learn.microsoft.com/en-us/windows/wsl/install).

## Set up Git for Ubuntu

### Install Git
Open a Ubuntu Shell.

Git is most likely already installed. To be sure, run
```bash
git --version
```
If this prints a version number, you are good to go. If not, run the following command in the Ubuntu-Shell.
```bash
sudo apt update
sudo apt install git
```

#### Add authentication for GitLab

!!! note

    You only need this if you can't push or pull. If you can, just ignore this.

For accessing the GitLab-Repository, you need to authenticate yourself. Therefor, you need to set up an ssh key for your GitLab-Account. In addition to this, you need to configure git to use it.

Configure Git to use the Windows credential manager by running the following command in the Ubuntu-Shell.
```bash
git config --global core.sshcommand "ssh.exe"
```

Make sure that you have ssh installed on Windows by running the following command in PowerShell.
```powershell
ssh -V
```
If it is not installed, please enable it by following [this guide](https://learn.microsoft.com/en-us/windows/terminal/tutorials/ssh).

Enable ssh-agent to run on startup. This makes sure your ssh key can be used by git and ssh  Run these commands in PowerShell with Administrator permissions:
```powershell
Get-Service -Name ssh-agent | Set-Service -StartupType Manual
Start-Service ssh-agent
```

Follow the instructions by GitLab to generate an ssh key. You can find the instructions [here](https://docs.gitlab.com/ee/user/ssh.html#generate-an-ssh-key-pair).
Then you need to [add the public key to your GitLab-Account](https://docs.gitlab.com/ee/user/ssh.html#add-an-ssh-key-to-your-gitlab-account).

After that, add the ssh key to the ssh-agent by running the following command in PowerShell.
```powershell
ssh-add <path-to-private-key>
```
Configure SSH to use the ssh key for GitLab by creating a config file at `C:\Users\<Windows-username>\.ssh\config`.
```
Host gitlab.com
  PreferredAuthentications publickey
  IdentityFile "<path-to-private-key>"
```

## Configure Docker for interop with WSL

Open Docker Desktop and open the settings tab. Then navigate to "Resources" and "WSL Integration". Enable the WSL2-Integration and select the Ubuntu-22.04 Distribution.
If you have set up the Ubuntu-22.04 as your default, you check the "Enable integration with my default WSL distro" checkbox.

Now click on "Apply & Restart".

## Install Visual Studio Code

Install Visual Studio Code for Windows [here](https://code.visualstudio.com/download) and follow the SetUp-Wizard.

## Configure Visual Studio Code

When you open Visual Studio Code, please navigate to the "Extension"-Section on the left of your screen and install [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-container).
After that, install [WSL](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl).

Alternatively, press **Ctrl+Shift+P** and enter following Commands (only one at the time)

```
ext install ms-vscode-remote.remote-container
ext install ms-vscode-remote.remote-wsl
```

## X11

In order to run visual applications on Windows, please download [VcXsrv](https://sourceforge.net/projects/vcxsrv/).

!!! note

    If you want to start VcXsrv, search for XLaunch

!!! note

    Unfortunately, XMing is too old to support newer connection Methods.

## Clone the Repository

Open a WSL2 Shell and navigate to your respective directory where you want to clone the repository to.

Copy & Paste these commands and hit Enter. 

```bash
cd ~
git clone --recurse-submodules -b <branch> git@gitlab.com:strohm-und-soehne/driverless/autonomous-system.git
cd autonomous-system
code .
```

!!! note 
    Please replace ```<branch>``` by your desired branch to clone - default is ```main```

Visual Studio may show a prompt warning you that the host wsl.localhost is not known. This is normal and you can just select the option to always trust it.

You should now have the ```autonomous-system```-Repository opened in Visual Studio Code.

### Configure Git

You also need to configure your Git username and email, otherwise Visual Studio Code will throw errors when you try to create a commit. Run the following commands in a Terminal of your choice after installation.

```bash
git config --add user.name <your-name>
git config --add user.email <your-email>
```

??? note

    You can also use
    ```bash
    git config --add user.name <your-name> --global
    git config --add user.email <your-email> --global
    ```
    to configure these settings globally. This helps if you regularly clone this repository, as you don't have to configure it again.
    On the flip side, these settings will be used for all your repositories. So be careful with this.

## Opening the DevContainer

Since you already have the DevContainer-Extension installed, you can just hit **Ctrl+Shift+P** and search for ```Dev Containers: Reopen in Container```

Now select default.

??? Note
    The FSDS option is there for legacy reasons. Choosing this option will add support for the FSDS-Bridge and the FSDS Simulator.
    This is not needed for the autonomous system right now and will only slow down the build process.

This should automatically start the Docker Creation and open a new Visual Studio Code Window.

!!! note
    The first time you open the DevContainer, it will take a while to build the Docker Image. Please be patient.

## Quickly open this project again

First of all, start Docker Desktop.


There are multiple ways to open this project again:

- Use the project manager in Visual Studio Code. You can find it on the left side of the screen. Just save this project once and it will appear there. This is recommended because it is by far the easiest.
- Use the "open recent" feature in Visual Studio Code. You can find it in the start screen of Visual Studio Code. Just click on the project you want to open. Alternatively, you can find it in the "File" menu.
- Open the project folder in Visual Studio Code. You can do this by navigating to the project folder in the terminal and typing `code .`. After that, you can open the DevContainer again by pressing `Ctrl+Shift+P` and searching for `Dev Containers: Reopen in Container`.
- Open this project from the Visual Studio Code start screen. You can do this by clicking on `Open Folder` and selecting the project folder. After that, you can open the DevContainer again by pressing `Ctrl+Shift+P` and searching for `Dev Containers: Reopen in Container`.

## What now?

### Simulator

!!! note

    The simulator is not used currently. This is mostly here for legacy reasons

Great! Now you are ready and set to start to develop on our Autonomous System. If you want to try out simulator and ROS2 in action, please run the following command in the Visual Studio Code Terminal.

```bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py
```

!!! warning
    Quick heads up: The FS-Driverless-Simulator can be configured by the ``settings.json``-File in its root directory. Whenever you change settings on the Simulator also change them in ``src/fsds/fsds_ros2_bridge/assets/settings.json``. Copy & Paste should do the job.

### Additional Guides

Please also look into following Guides next:

- A fast entry into the basics of Git can be found [here](https://rogerdudler.github.io/git-guide/index.de.html).
- If you want to learn more about ROS, I would kindly redirect you to the ROS2 humble documentation [tutorial site](https://docs.ros.org/en/humble/Tutorials.html).