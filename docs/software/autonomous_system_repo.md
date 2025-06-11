# Autonomous System Repository

Here all ros packages included in the main Autonomous System Repository are explained. Key functions and configurations are also taken into account.

## Drivers

## FSDS

## Subsystems

### can_frame_translator

This package exists to translate CAN-Messages (CanFrame) from the `ros2_socketcan`-package to ROS-Messages and vice versa.

CAN-Messages are data packs consisting of (mostly) 8 data bytes and a header. The data can hold multiple types of information named signals. How a Message needs to be split and decoded is defined by the CAN-ID. Lower ID-Number results in higher priority.

#### CanFrameEncoder

To achieve this the namespace `CanFrameEncoder` holds functions to en- and decode parts of a message in little endian format. The en- and decoding can be set by defining a `start_bit` in the message, `length` of the signal in bits and `scale` to modify the original value.

- 
  ```cpp
  void CanFrameEncoder::encodeUnsigned(uint8_t *frame, uint32_t data, int start_bit, int length, float scale);
  ```
- 
  ```cpp
  void CanFrameEncoder::encodeSigned(uint8_t *frame, int32_t data, int start_bit, int length, float scale);
  ```
- 
  ```cpp
  void CanFrameEncoder::encodeFloat(uint8_t *frame, float data, int start_bit, int length, float scale);
  ```
- 
  ```cpp
  float CanFrameEncoder::decodeUnsigned(const uint8_t *frame, int start_bit, int length, float scale);
  ```
- 
  ```cpp
  float CanFrameEncoder::decodeSigned(const uint8_t *frame, int start_bit, int length, float scale);
  ```

#### CanToRos

This Node subscribes to the receiver of the `ros2_socketcan`-package. On every received message the function `onCANFrameReceived` gets called. It checks on the CAN-ID and initiates a sequence of handler functions defined in `can_id_map_`. The handler functions are given the publisher message type in template format, the desired `publisher`, `start_bit`, `length` and `scale`.

```cpp
can_id_map_ = {
    {0x500, {std::bind(&CanToRos::handleMessage<std_msgs::msg::UInt32>, this, _1, test_UInt32_publisher_, 8, 8, 1.0f),
             std::bind(&CanToRos::handleMessage<std_msgs::msg::Int32>, this, _1, test_Int32_publisher_, 16, 8, 1.0f)}},
    {0x501, {std::bind(&CanToRos::handleMessage<std_msgs::msg::Float32>, this, _1, test_Float32_publisher_, 0, 32, 1.0f)}},
    {0x502, {std::bind(&CanToRos::handleMessage<std_msgs::msg::Int32>, this, _1, test_Int32_publisher_, 0, 32, 1.0f)}}
};
```

#### RosToCan

The `RosToCan`-Node subscribes to a number of topics used inside the ROS2-Environment and publishes to transceiver of the `ros2_socketcan`-package. Based on a set of `can_ids_` a set of buffers and timers is created. Each is taking in asynchronous changes but publish synchronously timed to the transceiver (every 100 milliseconds). Subscriptions can be created standardized via the `subscription_list`-Vector. It takes in entries of the following format:

```cpp
{<identifier|string>, <config|SubscriptionConfig>}
```

```cpp
struct SubscriptionConfig{
    std::string topic;
    uint32_t can_id;
    int bit_position;
    int bit_length;
    float scale;
    enum SubscriptionTyping typing;
};

enum SubscriptionTyping{
    Bool,
    UInt8,
    Int8,
    Int16,
    Float32,
};
```

### descriptions

The description package is used to define `tf2`s. A transform-frame (`tf2`) describes the relation between two **links** in a 3D space. This type of transformation can either be static (defined one) or dynamic (adjusted at runtime).

A link name has to follow these guidelines:

1. lower case letters
2. whitespaces are replaced by hyphens (`-`)

#### Static Transformations through launch files

The description of such transformations is defined in specific launch files. 

Examples are `base-link.launch.py` and `odom.launch.py`.

Following is a code snippet that calls the `static_transformation_publisher`-Node and defines the arguments of the transformation

```python
Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=[
          '--x', '0', '--y', '0', '--z', '0',
          '--yaw', '0', '--pitch', '0', '--roll',
          '0', '--frame-id', 'parent_frame', '--child-frame-id', 'this_frame']
  ),
```

#### Transformations using `urdf`

`urdf` is a xml-file-format that can be used to describe robots and their links. This description can then be read and interpreted as transformations.

An example for defining such relations is the file `urdf/nora11.urdf.xacro`. 

The launch file `robot_urdf.launch.py` takes `urdf` as input and launches the `robot_state_publisher` Node which creates `tf2`s based on the defined links.

#### Combining everything

The file `tf_statics.launch.py` is here to take in every other launch file related to static transformations and launches them at the same time. This makes it easier to configure relations between components. 

!!! warning inline end "Non-functional"

    This option is not in a finished version! Currently proper Configuration as well as in- and outputs are missing.

Additionally a sensor-fusion localization can be activated by setting the parameter `ekf:=true`. This will activate the `ekf_node` from `robot_localization`-package. Configuration can be done in the params folder. 

> Further information can be found at the packages [Github](https://github.com/cra-ros-pkg/robot_localization/tree/ros2)