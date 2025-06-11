# DevContainer Installation

This guide is designed to set up an environment using Docker Desktop and Visual Studio Code.
If you use Windows, please follow [this guide](install_devcontainer_windows.md) instead.

## Requirements

This guide expects you to have a working Formula-Student-Driverless-Simulator Installation up and running.
If that is not the case, please follow the instructions [here](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.2.0/getting-started/#from-release-binaries). We advise you to install from release binaries.

You also need a GitLab-Account using your ```@strohmleitung```-Mail. Please register [here](https://gitlab.com/users/sign_up) and send your username to the "Head of Driverless". He will clear you for the Strohm-und-Söhne Developing Group. 

## Install Docker Desktop

=== "Linux"

    Please install the respective Docker Desktop Application for your [Linux Distro](https://docs.docker.com/desktop/install/linux-install)

    !!! note

        As a Linux user you are not required to install Docker Desktop as you may run into permission problems because of the creation of a Linux VM. In this case you can also use the core functionalities by [installing docker engine](https://docs.docker.com/engine/install/).

=== "Mac"

    Please install the respective Docker Desktop Application for [Mac](https://docs.docker.com/desktop/install/mac-install)

## Install Visual Studio Code

Select a package suited for your OS [here](https://code.visualstudio.com/download) and follow the SetUp-Wizard.

## Configure Visual Studio Code

When you open Visual Studio Code, please navigate to the "Extension"-Section on the left of your screen and install [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-container)

Alternatively, press **Ctrl+Shift+P** and enter following Command

```
ext install ms-vscode-remote.remote-container
```

## Clone the Repository

=== "Linux"

    In order to clone the repository, you need to have git installed on your System. For installation follow the guide [here](https://git-scm.com/download/linux).

=== "Mac"

    In order to clone the repository, you need to have git installed on your System. For installation follow the guide [here](https://git-scm.com/download/mac).

Another configuration is the SSH-Key needed to establish a connection between GitLab and your Computer. Please follow these two Instructions from GitLab. [First](https://docs.gitlab.com/ee/user/ssh.html#generate-an-ssh-key-pair) here, second [there](https://docs.gitlab.com/ee/user/ssh.html#add-an-ssh-key-to-your-gitlab-account).

Open a Command-Shell or Terminal and Navigate to your respective Directory where you want to clone the Repository to. 

Copy & Paste these commands and hit Enter.

=== "Linux"

    ```bash
    git clone --recurse-submodules -b <branch> https://gitlab.com/strohm-und-soehne/driverless/autonomous-system.git
    cd autonomous-system
    code .
    ```

=== "Mac"

    ```bash
    git clone --recurse-submodules -b <branch> https://gitlab.com/strohm-und-soehne/driverless/autonomous-system.git
    cd autonomous-system
    code .
    ```

!!! note 
    Please replace ```<branch>``` by your desired branch to clone - default is ```main```

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

You should now have the ```autonomous-system```-Repository opened in Visual Studio Code.

## Opening the DevContainer

=== "Linux"

    Since you already have the DevContainer-Extension installed, you can just hit **Ctrl+Shift+P** and search for ```Dev Containers: Reopen in Container```

    Now select default.

    ??? Note
        The FSDS option is there for legacy reasons. Choosing this option will add support for the FSDS-Bridge and the FSDS Simulator.
        This is not needed for the autonomous system right now and will only slow down the build process.

    This should automatically start the Docker Creation and open a new Visual Studio Code Window.

    Because some extensions clash with the host OS you have to run a small bash-script to make the container compatible with Linux.

    ```bash
    .devcontainer/linux_extensions.sh
    ``` 

    Reloading your VSCode-Window is advised.

=== "Mac"

    Since you already have the DevContainer-Extension installed, you can just hit **Ctrl+Shift+P** and search for ```Dev Containers: Reopen in Container```

    Now select default.

    ??? Note
        The FSDS option is there for legacy reasons. Choosing this option will add support for the FSDS-Bridge and the FSDS Simulator.
        This is not needed for the autonomous system right now and will only slow down the build process.

    This should automatically start the Docker Creation and open a new Visual Studio Code Window.

## What now?

Great! Now you are ready and set to start to develop on our Autonomous System. If you want to try out simulator and ROS2 in action, please run the following command in the Visual Studio Code Terminal.

```bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py
```

!!! warning
    Quick heads up: The FS-Driverless-Simulator can be configured by the ``settings.json``-File in its root directory. Whenever you change settings on the Simulator also change them in ``src/fsds/fsds_ros2_bridge/assets/settings.json``. Copy & Paste should do the job.

Please also look into following Guides next:

- A fast entry into the basics of Git can be found [here](https://rogerdudler.github.io/git-guide/index.de.html).
- If you want to learn more about ROS, I would kindly redirect you to the ROS2 humble documentation [tutorial site](https://docs.ros.org/en/humble/Tutorials.html).