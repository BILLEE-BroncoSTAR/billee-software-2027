# Debugging in ROS 2

## rqt_console

Launch the console GUI to view and filter log messages from running nodes:

```bash
ros2 run rqt_console rqt_console
```

### Logger Levels

| Level | Description |
|-------|-------------|
| **Fatal** | Indicates the system is going to terminate to protect itself from detriment. |
| **Error** | Indicates significant issues that won't necessarily damage the system, but are preventing it from functioning properly. |
| **Warn** | Indicates unexpected activity or non-ideal results that might represent a deeper issue, but don't harm functionality outright. |
| **Info** | Indicates event and status updates that serve as visual verification that the system is running as expected. |
| **Debug** | Usually hidden by default; used for detailed diagnostic output. |

### Setting the Log Level

```bash
ros2 run <pkg> <node> --ros-args --log-level <level>
```

Sets the default log level for a given node.

---

## Recording Log Data

### Recording Commands

Record a single topic:

```bash
ros2 bag record <topic>
```

Record multiple topics:

```bash
ros2 bag record <topic_1> <topic_2> ... <topic_n>
```

Record with a custom bag name:

```bash
ros2 bag record -o <name> <topic>
```

### Interacting with Bags

View bag metadata:

```bash
ros2 bag info <bag>
```

Play back a recorded bag:

```bash
ros2 bag play <bag>
```