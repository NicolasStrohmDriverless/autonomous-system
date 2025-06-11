# Autonomous System

The [Autonomous System repository](https://gitlab.com/strohm-und-soehne/driverless/autonomous-system) is located on our Teams Gitlab Group. It is the central hub for implementing and documenting the Software of our Autonomous System. Inside the [Install Guide](../guides/installation/install_devcontainer.md) you will find all necessary information to get the Docker Container and a Simulator running on your device.

## Structure

Our repository consist of different folders and files all serving a specific purpose. This should give you a small overview.

```
.
├── .devcontainer             # Configuration for the docker environment
├── .drawio                   # Diagrams describing different system parts
├── docker                    # Docker files for deployment
├── docs                      # Source files for our documentation
├── src                       # Source-Code for our ROS2 Workspace
│   ├── drivers               # Already existing drivers of hardware we use
│   ├── fsds                  # Simulator API
│   └── subsystems            # Implementation of our own ROS2 Nodes
│       ├── package 1         # packages containing source and unit test code
│       └── package 2
├── .gitlab-ci.yml            # Pipeline Configuration for GitLab
```