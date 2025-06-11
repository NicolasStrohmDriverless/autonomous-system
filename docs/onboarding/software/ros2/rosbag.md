# Rosbag

## 6.1 Setup

You’ll be recording your keyboard input in the `turtlesim` system to save and replay later on, so begin by starting up the `/turtlesim` and `/teleop_turtle` nodes.

Open a new terminal and run:

```bash
ros2 run turtlesim turtlesim_node
```

Open another terminal and run:

```bash
ros2 run turtlesim turtle_teleop_key
```

It is recommended to create a own folder for storing rosbags, e.g. `bags`.

## 6.2 Choose a topic

ros2 bag can only record data from published messages in topics. To see the list of your system’s topics, open a new terminal and run the command:

```bash
ros2 topic list
```

Which will return:

```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

In the topics tutorial, you learned that the /turtle_teleop node publishes commands on the `/turtle1/cmd_vel` topic to make the turtle move in turtlesim.

To see the data that `/turtle1/cmd_vel` is publishing, run the command:

```bash
ros2 topic echo /turtle1/cmd_vel
```

Nothing will show up at first because no data is being published by the teleop. Return to the terminal where you ran the teleop and select it so it’s active. Use the arrow keys to move the turtle around, and you will see data being published on the terminal running ros2 topic echo.

```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
```

## 6.3 ros2 bag record

### 6.3.1 Record a single topic

To record the data published to a topic use the command syntax:

```bash
ros2 bag record <topic_name>
```

Before running this command on your chosen topic, open a new terminal and move into the `bags` directory you created earlier, because the rosbag file will save in the directory where you run it.

Run the command:

```
ros2 bag record /turtle1/cmd_vel
```

You will see the following messages in the terminal (the date and time will be different):

```
[INFO] [rosbag2_storage]: Opened database 'rosbag2_2019_10_11-05_18_45'.
[INFO] [rosbag2_transport]: Listening for topics...
[INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/cmd_vel'
[INFO] [rosbag2_transport]: All requested topics are subscribed. Stopping discovery...
```

Now ros2 bag is recording the data published on the `/turtle1/cmd_vel` topic. Return to the teleop terminal and move the turtle around again. The movements don’t matter, but try to make a recognizable pattern to see when you replay the data later.

### 6.3.2 Record multiple topics

In order to record multiple topics just list different topic names using whitespace.

Enter following and move interact with your turtle:

```bash
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose
```

!!! info
    `-o <name>` defines a custom name for your bag.

Stop the recording with <kbd>Ctrl</kbd>+<kbd>C</kbd>.

## 6.4 ros2 bag info

You can see details about your recording by running:

```bash
ros2 bag info <bag_file_name>
```

Running this command on the subset bag file will return a list of information on the file:

```
ros2 bag info subset
Files:             subset.db3
Bag size:          228.5 KiB
Storage id:        sqlite3
Duration:          48.47s
Start:             Oct 11 2019 06:09:09.12 (1570799349.12)
End                Oct 11 2019 06:09:57.60 (1570799397.60)
Messages:          3013
Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 9 | Serialization Format: cdr
                   Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 3004 | Serialization Format: cdr
```

## 6-5 ros2 bag play

Before replaying the bag file, enter <kbd>Ctrl</kbd>+<kbd>C</kbd> in the terminal where the teleop is running. Then make sure your turtlesim window is visible so you can see the bag file in action.

Enter the command:

```bash
ros2 bag play subset
```

The terminal will return the message:

```
[INFO] [rosbag2_storage]: Opened database 'subset'.
```

Your turtle will follow the same path you entered while recording (though not 100% exactly; turtlesim is sensitive to small changes in the system’s timing).

Your turtle will follow the same path you entered while recording (though not 100% exactly; turtlesim is sensitive to small changes in the system’s).