# Useful ROS2 Commands

A quick reference for common ROS2 CLI commands, organized by concept.

---

## Nodes

### Running Nodes

```bash
ros2 run <package> <node>
```

### Node Info

| Command | Description |
|---|---|
| `ros2 node list` | List running nodes |
| `ros2 node info <node>` | Get info about a node |

---

## Topics

### RQT Graph

Visualize a graph of the entire system's plumbing:

```bash
ros2 run rqt_graph rqt_graph
```

Then: **Plugins → Introspection → Node Graph**

### Topic Info

| Command | Description |
|---|---|
| `ros2 topic list -t` | List topics with types |
| `ros2 topic echo <topic>` | Show data published to a topic |
| `ros2 topic info <topic> --verbose` | Show node names/namespaces, topic type, and QoS profiles |
| `ros2 topic hz <topic>` | Show topic data publish rate |

### Publishing Topics

```bash
ros2 topic pub <topic> <msg> <args>
```

**Example:**
```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

### Searching by Topic Type

```bash
ros2 topic find <topic_type>
```

---

## Messages

### Showing Message Schema

```bash
ros2 interface show <message>
```

---

## Services

### Service Info

| Command | Description |
|---|---|
| `ros2 service list -t` | List services with types |
| `ros2 service type <service>` | Get the type of a service |
| `ros2 service find <type>` | Find services by type |

### Service Types

Since services are request/response, service types define both the request and the response.

```bash
ros2 interface show <type>
```

**Example:**
```
float32 x
float32 y
float32 theta
string name
---
string name
```

> Everything below the `---` is the response.

### Calling Services

```bash
ros2 service call <service> <service_type> <args>
```

---

## Parameters

### Parameter Info

| Command | Description |
|---|---|
| `ros2 param list` | List parameters |
| `ros2 param get <node> <param>` | Get type + current value of a param on a running node |
| `ros2 param dump <node>` | Get all current parameter values |

### Setting Parameters

```bash
ros2 param set <node> <param_name> <value>
```

Bulk-load preset parameters:
```bash
ros2 param load <node> <param.yaml>
```

---

## Actions

### Action Info

| Command | Description |
|---|---|
| `ros2 action list -t` | List actions with types |
| `ros2 action info <action>` | Get info about an action |
| `ros2 node info <node>` | Useful for debugging/inspecting actions on a node |
| `ros2 interface show <action>` | Show action schema |

### Sending a Goal

```bash
ros2 action send_goal <action> <action_type> <value>
```

## Packages

### Creating a Package

`ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package>`
`ros2 pkg create --build-type ament_python --license Apache-2.0 <package>`