# CheatSheet

## Git

### Glossary

<table>
    <tr>
        <td>Commit</td>
        <td>a submitted state of the code base</td>
    </tr>
    <tr>
        <td>Branch</td>
        <td>a reference to a commit; also tracks history of commits before the durrent one</td>
    </tr>
    <tr>
        <td>Tag</td>
        <td>a reference (standard) or an object (annotated)</td>
    </tr>
    <tr>
        <td>HEAD</td>
        <td>a place where your working directory is now</td>
    </tr>
</table>

### Setup

<table>
    <tr>
        <td width="35%"><code>git init &lt;project name&gt;</code></td>
        <td width="65%">Create a new local repository in the current directory. If &lt;project name&gt; is provided, Git will create a new directory named &lt;project name&gt; and will initialize a repository inside it.</td>
    </tr>
    <tr>
        <td><code>git clone &lt;project url&gt;</code></td>
        <td>Downloads a project with the entire history from the remote repository.</td>
    </tr>
</table>

### Common commands

<table>
    <tr>
        <td width="35%"><code>git status</code></td>
        <td width="65%">Displays the status of your working directory. Options include new, staged, and modified files. It will retrieve branch name, current commit identifier, and changes pending commit.</td>
    </tr>
    <tr>
        <td><code>git add &lt;file&gt;</code></td>
        <td>Add a file to the staging area. Use. in place of the full file path to add all changed files from the current directory down into the directory tree.</td>
    </tr>
    <tr>
        <td><code>git diff &lt;file&gt;</code></td>
        <td>Show changes between working directory and staging area.</td>
    </tr>
    <tr>
        <td><code>git diff --staged &lt;file&gt;</code></td>
        <td>Shows any changes between the staging area and the repository.</td>
    </tr>
    <tr>
        <td><code>git checkout -- &lt;file&gt;</code></td>
        <td>Discard changes in working directory. This operation is unrecoverable. </td>
    </tr>
    <tr>
        <td><code>git reset [&lt;path&gt;...]</code></td>
        <td>Revert some paths in the index (or the whole index) to their state in HEAD.</td>
    </tr>
    <tr>
        <td><code>git commit</code></td>
        <td>Create a new commit from changes added to the staging area. The commit must have a message! </td>
    </tr>
    <tr>
        <td><code>git rm &lt;file&gt;</code></td>
        <td>Remove file from working directory and staging area.</td>
    </tr>
</table>

### Stashing work

<table>
    <tr>
        <td><code>git stash</code></td>
        <td>Put current changes in your working directory into stash for later use</td>
    </tr>
        </tr>
    <tr>
        <td><code>git stash pop</code></td>
        <td>Apply stored stash content into working directory, and clear stash.</td>
    </tr>
        </tr>
    <tr>
        <td><code>git stash drop</code></td>
        <td>Delete a specific stash from all your previous stashes.</td>
    </tr>
</table>

### Branching

<table>
    <tr>
        <td width="35%"><code>git branch [-a]</code></td>
        <td width="65%">List all local branches in repository. With -a: show all branches (with remote).</td>
    </tr>
    <tr>
        <td><code>git branch [branch_name]</code></td>
        <td>Create new branch, referencing the current **HEAD**.</td>
    </tr>
    <tr>
        <td><code>git rebase [branch_name]</code></td>
        <td>Set the base of the current branch to HEAD of [branch_name] and apply your changes on top</td>
    </tr>
    <tr>
        <td><code>git checkout [-b] [branch_name]</code></td>
        <td>Switch working directory to the specified branch. With -b: Git will create the specified branch if it does not exist.</td>
    </tr>
    <tr>
        <td>git merge [branch_name]<code></code></td>
        <td> Join specified [branch_name] branch into your current branch (the one you are on currently). Possible change conflicts will have to be resolved manually.</td>
    </tr>
    <tr>
        <td><code>git branch -d [branch_name]</code></td>
        <td>remove selected branch, if it is already merged into any other. <code>-D</code> instead of <code>-d</code> forces deletion</td>
    </tr>
</table>

### Inspect history

<table>
    <tr>
        <td width="35%"><code>git log [-n count]</code></td>
        <td width="65%">List commit history of current branch. <code>-n count</code> limits list to last n commits</td>
    </tr>
    <tr>
        <td><code>git log --oneline --graph --decorate</code></td>
        <td>An overview with reference labels and history graph. One commit per line.</td>
    </tr>
    <tr>
        <td><code>git log ref ..</code></td>
        <td>List commits that are present on the current branch and not merged into ref. A ref can be a branch name or a tag name.</td>
    </tr>
    <tr>
        <td><code>git log ..ref</code></td>
        <td>List commit that are present on ref and not merged into current branch</td>
    </tr>
    <tr>
        <td><code>git reflog</code></td>
        <td>List operations (e.g. checkouts or commits) made on local repository</td>
    </tr>
</table>

### Tagging commits

<table>
    <tr>
        <td width="35%">git tag<code></code></td>
        <td width="65%">Lists all tags</td>
    </tr>
    <tr>
        <td><code>git tag [name] [commit sha]</code></td>
        <td>Create a tag reference named <code>name</code> for current commit. Add <code>commit sha</code> to tag a specific commit instead of current one</td>
    </tr>
    <tr>
        <td><code>git tag -a [name] [commit sha]</code></td>
        <td>Create a tag object named <code>name</code> for current commit.</td>
    </tr>
    <tr>
        <td><code>git tag -d [name] </code></td>
        <td>Remove a tag from local repository.</td>
    </tr>
</table>

### Reverting Changes

<table>
    <tr>
        <td width="35%"><code>git reset [--hard] [target reference]</code></td>
        <td width="65%">Switches the current branch to the target reference, leaving a difference as an uncommitted change. When <code>--hard</code> is used, all changes are discarded. It's easy to lose uncommitted changes with <code>--hard</code>.</td>
    </tr>
    <tr>
        <td><code>git revert [commit sha]</code></td>
        <td>Create a new commit, reverting changes from the specified commit. It generates an <code>inversion</code> of changes.</td>
    </tr>
</table>

### Synchronizing repositories

<table>
    <tr>
        <td width="35%"><code>git fetch [remote]</code></td>
        <td width="65%">Fetch changes from the <code>remote</code>, but not update tracking branches.</td>
    </tr>
    <tr>
        <td><code>git fetch --prune [remote]</code></td>
        <td>Delete remote Refs that were removed from the <code>remote</code> repository.</td>
    </tr>
    <tr>
        <td><code>git pull [remote]</code></td>
        <td>Fetch changes from the <code>remote</code> and merge current branch with its upstream.</td>
    </tr>
    <tr>
        <td><code>git push [--tags] [remote]</code></td>
        <td>Push local changes to the <code>remote</code>. Use <code>--tags</code> to push tags.</td>
    </tr>
    <tr>
        <td><code>git push -u [remote] [branch]</code></td>
        <td>Push local branch to <code>remote</code> repository. Set its copy as an upstream.</td>
    </tr>
</table>

### Ignoring files

To ignore files, create a .gitignore file in your repository with a line for each pattern. File ignoring will work for the current and sub directories where .gitignore file is placed. In this example, all files are ignored in the logs directory (excluding the .gitkeep file), whole tmp directory and all files *.swp.

```.gitignore
/logs *
!logs/.gitkeep
/tmp
*.swp
```

## ROS2

### Command Line Interface

All ROS 2 CLI tools start with the prefix `ros2` followed by a command, a verb and (possibly) positional/optional arguments.

```ros2 <command> <verb> <optional arguments>```

For any tool, the documentation is accessible with, `ros2 command --help`
and similarly for verb documentation, `ros2 command verb -h`

#### Actions

A ROS 2 action enables asynchronous communication for long-running tasks between nodes, allowing for feedback, cancellation, and final result reporting. It’s useful for tasks like navigation or manipulation that require real-time updates and control.

Allows to manually send a goal and displays debugging information about actions.

<table>
    <tr>
        <td><code>ros2 action info [action]</code></td>
        <td>Output Information about an action</td>
    </tr>
    <tr>
        <td><code>ros2 action list</code></td>
        <td>Output a list of action names.</td>
    </tr>
    <tr>
        <td><code>ros2 action send_goal [action_name] [action_type] [goal]</code></td>
        <td>Send an action goal</td>
    </tr>
</table>

#### Rosbags

A ROS bag is a file format in ROS used to record and store message data from topics for later playback or analysis. It allows capturing real-time data during system operation for debugging, testing, or simulation purposes.

<table>
    <tr>
        <td><code>ros2 bag info [bag_path]</code></td>
        <td>Output information of a bag.</td>
    </tr>
    <tr>
        <td><code>ros2 bag play [bag_path]</code></td>
        <td>Play a bag.</td>
    </tr>
    <tr>
        <td><code>ros2 bag record {topics}</code></td>
        <td>Record a bag.</td>
    </tr>
</table>

#### Components

A ROS component is a node that can be dynamically loaded into a running process to improve modularity and efficiency. It allows multiple nodes to share resources within a single process, reducing overhead and enabling faster communication.

<table>
    <tr>
        <td width ="45%"><code>ros2 component list</code></td>
        <td>Output a list of running containers and components.</td>
    </tr>
    <tr>
        <td><code>ros2 component load [container_node_name] [package_name] [plugin_name]</code></td>
        <td>Load a component into a container node.</td>
    </tr>
    <tr>
        <td><code>ros2 component standalone [package_name] [plugin_name]</code></td>
        <td>Run a component into its own standalone container node.</td>
    </tr>
    <tr>
        <td><code>ros2 component types</code></td>
        <td>Output a list of components registered in the ament index.</td>
    </tr><tr>
        <td><code>ros2 component unload [container_node_name] [component_uid]</code></td>
        <td>Unload a component from a container node.</td>
    </tr>
</table>

#### Daemon


A ROS daemon is a background process that runs continuously to perform specific tasks or manage system resources. It operates without direct user interaction, supporting functions like managing nodes or handling communication between components.

<table>
    <tr>
        <td><code>ros2 daemon start</code></td>
        <td>Start the daemon if it isn’t running.</td>
    </tr>
    <tr>
        <td><code>ros2 daemon status</code></td>
        <td>Output the status of the daemon.</td>
    </tr>
    <tr>
        <td><code>ros2 daemon stop</code></td>
        <td>Stop the daemon if it is running.</td>
    </tr>
</table>

#### ROS2 Health

A tool to check ROS setup and other potential issues such as network, package versions, rmw middleware etc.

<table>
    <tr>
        <td><code>ros2 doctor -r</code></td>
        <td>Output report of all checks.</td>
    </tr>
    <tr>
        <td><code>ros2 doctor -rf</code></td>
        <td>Output report of failed checks only.</td>
    </tr>
    <tr>
        <td><code>ros2 doctor -iw</code></td>
        <td>Include warnings as failed checks.</td>
    </tr>
</table>

#### Extension Points

In ROS 2, extension points are hooks that allow developers to extend or customize the behavior of core functionalities without modifying the underlying code. They enable plugins or additional modules to be integrated seamlessly into the system, enhancing flexibility and modularity.

<table>
    <tr>
        <td><code>ros2 extension_points</code></td>
        <td>Lists extension points</td>
    </tr>
</table>


#### Extensions

In ROS 2, extensions are additional packages or plugins that enhance or customize the functionality of the core system. They allow developers to add features, tools, or capabilities, such as new message types, transport layers, or middleware implementations, without modifying the base framework.

<table>
    <tr>
        <td><code>ros2 extensions</code></td>
        <td>Lists extensions</td>
    </tr>
</table>

#### Interfaces

In ROS 2, an interface defines the structure of communication between nodes, specifying how data is exchanged. It includes three types: messages (for topic-based communication), services (for request-response interactions), and actions (for long-running tasks). Each interface defines the format and type of data being passed.

On command line each can be filtered with either of the following options, ‘--only-actions’, ‘--only-msgs’, ‘--only-srvs’.

<table>
    <tr>
        <td><code>ros2 interface list</code></td>
        <td>List all interface types available.</td>
    </tr>
    <tr>
        <td><code>ros2 interface package [package_name]</code></td>
        <td>Output a list of available interface types within one package.</td>
    </tr>
    <tr>
        <td><code>ros2 interface packages</code></td>
        <td>Output a list of packages that provide interfaces.</td>
    </tr>
    <tr>
        <td><code>ros2 interface proto [type]</code></td>
        <td>Print the prototype (body) of an interfaces.</td>
    </tr>
    <tr>
        <td><code>ros2 interface show [type]</code></td>
        <td>Output the interface definition.</td>
    </tr>
</table>

#### Launching

Allows to run a launch file in an arbitrary package without to ‘cd’ there first.

In ROS 2, a launch file is used to start and configure multiple nodes and their parameters simultaneously. It simplifies system setup by automating the process of running nodes, setting configurations, and managing dependencies in a single file, often written in Python or XML.

<table>
    <tr>
        <td><code>ros2 launch [package] [launch-file]</code></td>
        <td>Launches configuration desribed in the Launch-file.</td>
    </tr>
</table>

#### Lifecycle

In ROS 2, a lifecycle refers to a managed node's state-based system, allowing nodes to transition through specific phases, such as unconfigured, inactive, active, and finalized. This helps control node behavior, ensuring proper initialization, cleanup, and shutdown, making the system more predictable and reliable.

<table>
    <tr>
        <td><code>ros2 lifecycle get {node_names}</code></td>
        <td>Get lifecycle state for one or more nodes.</td>
    </tr>
    <tr>
        <td><code>ros2 lifecycle list</code></td>
        <td>Output a list of available transitions.</td>
    </tr>
    <tr>
        <td><code>ros2 lifecycle nodes</code></td>
        <td>Output a list of nodes with lifecycle.</td>
    </tr>
    <tr>
        <td><code>ros2 lifecycle set [node_name] [transition]</code></td>
        <td>Trigger lifecycle state transition.</td>
    </tr>
</table>

#### Multicasting

In ROS 2, multicast refers to a network communication method that allows data to be sent from one source to multiple destinations simultaneously.

<table>
    <tr>
        <td><code>ros2 multicast receive</code></td>
        <td>Receive a single UDP multicast packet.</td>
    </tr>
    <tr>
        <td><code>ros2 multicast send</code></td>
        <td>Send a single UDP multicast packet.</td>
    </tr>
</table>

#### Nodes

In ROS 2, a node is a fundamental unit of computation that performs a specific task within a robotic system. Nodes communicate with each other using topics, services, or actions, allowing for modular design and parallel processing. 

<table>
    <tr>
        <td><code>ros2 node info [node_name]</code></td>
        <td>Output information about a node.</td>
    </tr>
    <tr>
        <td><code>ros2 node list</code></td>
        <td>Output a list of available nodes.</td>
    </tr>
</table>

#### Parameters

In ROS 2, parameters are configurable settings that allow users to modify a node's behavior at runtime. They can be used to adjust various aspects, such as sensor thresholds or operational modes, without changing the code. 

<table>
    <tr>
        <td><code>ros2 param delete [node_name] [parameter_name]</code></td>
        <td>Delete a parameter.<td>
    </tr>
    <tr>
        <td><code>ros2 param describe [node_name] [parameter_names]</code></td>
        <td>Show descriptive information about declared parameters.</td>
    </tr>
    <tr>
        <td><code>ros2 param dump [node_name]</code></td>
        <td>Dump the parameters of a given node in yaml format, either in terminal or in a file.</td>
    </tr>
    <tr>
        <td><code>ros2 param get [node_name] [parameter_name]</code></td>
        <td>Get parameter.</td>
    </tr>
    <tr>
        <td><code>ros2 param list</code></td>
        <td>Output a list of available parameters.</td>
    </tr>
    <tr>
        <td><code>ros2 param set [node_name] [parameter_name] [value]</code></td>
        <td>Set a parameter</td>
    </tr>
</table>

#### Packages

In ROS 2, packages are the fundamental building blocks for organizing and distributing software. Each package can contain nodes, libraries, configuration files, and other resources necessary for a specific functionality or application.

<table>
    <tr>
        <td><code>ros2 pkg create [package_name]</code></td>
        <td>Create a new ROS2 package.</td>
    </tr>
    <tr>
        <td><code>ros2 pkg executables [package_name]</code></td>
        <td>Output a list of package specific executables.</td>
    </tr>
    <tr>
        <td><code>ros2 pkg list</code></td>
        <td>Output a list of available packages.</td>
    </tr>
    <tr>
        <td><code>ros2 pkg xml [package_name]</code></td>
        <td>Output the information contained in the package xml manifest.</td>
    </tr>
</table>

#### Running Executables

In ROS 2, Executables can be run on their own without Launch-file.

<table>
    <tr>
        <td><code>ros2 run [package] [executable]</code></td>
        <td>Allows to run an executable in an arbitrary package without having to ‘cd’ there first.</td>
    </tr>
</table>

#### Services

<table>
    <tr>
        <td><code>ros2 service call [service_name] [service_type]</code></td>
        <td>Call a service.</td>
    </tr>
    <tr>
        <td><code>ros2 service find [service_type]</code></td>
        <td>Output a list of services of a given type.</td>
    </tr>
    <tr>
        <td><code>ros2 service list</code></td>
        <td>Output a list of service names.</td>
    </tr>
    <tr>
        <td><code>ros2 service type [service_name]</code></td>
        <td>Output service’s type.</td>
    </tr>
</table>

#### Topics

In ROS 2, topics are a communication mechanism used for publishing and subscribing to messages between nodes. A node can publish data to a specific topic, while other nodes can subscribe to receive updates from that topic.

<table>
    <tr>
        <td><code>ros2 topic bw [topic_name]</code></td>
        <td>Display bandwidth used by topic.</td>
    </tr>
    <tr>
        <td><code>ros2 topic delay [topic_name]</code></td>
        <td>Display delay of topic from timestamp in header.</td>
    </tr>
    <tr>
        <td><code>ros2 topic echo [topic_name]</code></td>
        <td>Output messages of a given topic to screen.</td>
    </tr>
    <tr>
        <td><code>ros2 topic find [topic_type]</code></td>
        <td>Find topics of a given type.</td>
    </tr>
    <tr>
        <td><code>ros2 topic hz [topic_name]</code></td>
        <td>Display publishing rate of topic.</td>
    </tr>
    <tr>
        <td><code>ros2 topic bw [topic_name]</code></td>
        <td>Display bandwidth used by topic.</td>
    </tr>
    <tr>
        <td><code>ros2 topic info [topic_name]</code></td>
        <td>Output information about a given topic.</td>
    </tr>
    <tr>
        <td><code>ros2 topic list</code></td>
        <td>Output list of active topics.</td>
    </tr>
    <tr>
        <td><code>ros2 topic pub [topic_name] [message_type] {values}</code></td>
        <td>Publish data to a topic.</td>
    </tr>
    <tr>
        <td><code>ros2 topic type [topic_name]</code></td>
        <td>Output topic’s type.</td>
    </tr>
</table>

### colcon

colcon stand for collective construction and is a command line tool to improve the workflow of building, testing and using multiple software packages. It automates the process, handles the ordering and sets up the environment to use the packages.

#### Environment Variables

<table>
    <tr>
        <td><code>CMAKE_COMMAND</code></td>
        <td>The full path to the CMake executable.</td>
    </tr>
    <tr>
        <td><code>COLCON_ALL_SHELLS</code></td>
        <td>Flag to enable all shell extensions.</td>
    </tr>
    <tr>
        <td><code>COLCON_COMPLETION_LOGFILE</code></td>
        <td>Set the logfile for completion time.</td>
    </tr>
    <tr>
        <td><code>COLCON_DEFAULTS_FILE</code></td>
        <td>Set path to the yaml file containing the default values for the command line arguments (default: $COLCON_HOME/defaults.yaml).</td>
    </tr>
    <tr>
        <td><code>COLCON_DEFAULT_EXECUTOR</code></td>
        <td>Select the default executor extension.</td>
    </tr>
    <tr>
        <td><code>COLCON_EXTENSION_BLACKLIST</code></td>
        <td>Blacklist extensions which should not be used.</td>
    </tr>
    <tr>
        <td><code>COLCON_HOME</code></td>
        <td>Set the configuration directory (default: `/.colcon`).</td>
    </tr>
    <tr>
        <td><code>COLCON_LOG_LEVEL</code></td>
        <td>Set the log level (<code>debug—10</code>, <code>info—20</code>, <code>warn—30</code>, <code>error—40</code>, <code>critical—50</code>, or any other positive numeric value).</td>
    </tr>
    <tr>
        <td><code>COLCON_LOG_PATH</code></td>
        <td>Set the log directory (default: <code>$COLCON_HOME/log</code>).</td>
    </tr>
    <tr>
        <td><code>CTEST_COMMAND</code></td>
        <td>The full path to the CTest executable.</td>
    </tr>
    <tr>
        <td><code>POWERSHELL_COMMAND</code></td>
        <td>The full path to the PowerShell executable.</td>
    </tr>
</table>

#### Build Commands

Building packages.

<table>
    <tr>
        <td><code>colcon build</code></td>
        <td>Build your whole workspace.</td>
    </tr>
    <tr>
        <td><code>colcon build --packages-selected {package_name}</code></td>
        <td>Build single or multiple package/s excluding dependencies.</td>
    </tr>
</table>

#### Other Commands

<table>
    <tr>
        <td><code>colcon list</code></td>
        <td>List all packages in the workspace.</td>
    </tr>
    <tr>
        <td><code>colcon test</code></td>
        <td>Test the whole workspace.</td>
    </tr>
    <tr>
        <td><code>colcon test --packages-select {packages}</code></td>
        <td>Only test selected packages.</td>
    </tr>
    <tr>
        <td><code>colcon test --packages-above {packages}</code></td>
        <td>Test selected packages and packages depending on it.</td>
    </tr>
    <t>r>
        <td><code>colcon test-result --all</code></td>
        <td>Print out all test-results</td>
    </tr>
</table>

#### Important Flags

<table>
    <tr>
        <td><code>--symlink-install</code></td>
        <td>Use ‘symlinks’ instead of installing (copying) files where possible.</td>
    </tr>
    <tr>
        <td><code>--continue-on-error</code></td>
        <td>Continue other packages when a package fails to build. Packages recursively depending on the failed package are skipped.</td>
    </tr>
    <tr>
        <td><code>--event-handlers console direct+</code></td>
        <td>Show output on console.</td>
    </tr>
    <tr>
        <td><code>--event-handlers console cohesion+</code></td>
        <td>Show output on console after a package has finished.</td>
    </tr>
    <tr>
        <td><code>--packages-select</code></td>
        <td>Build only specific package(s).</td>
    </tr>
    <tr>
        <td><code>--packages-above</code></td>
        <td>Build specific package(s) and other packages that recursively depend on it.</td>
    </tr>
    <tr>
        <td><code>--packages-skip</code></td>
        <td>Skip package(s).</td>
    </tr>
    <tr>
        <td><code>--packages-skip-build-finished</code></td>
        <td>Skip a set of packages which have finished to build previously.</td>
    </tr>
</table>