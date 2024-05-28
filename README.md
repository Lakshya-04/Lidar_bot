# Lidar_bot Package

## Git Commands

When in the `Lidar_bot` package space, use the following commands to manage your Git repository:

1. **Add all changes**:
    ```sh
    git add .
    ```

2. **Check the status**:
    ```sh
    git status
    ```

3. **Commit your changes**:
    ```sh
    git commit -m "commit message"
    ```

4. **Push your changes**:
    ```sh
    git push
    ```
    If you encounter errors, use:
    ```sh
    git push origin main --force
    ```

## Simulation Control

### Joystick Control

To control the robot using a joystick in the simulation, run the following commands:

1. Launch the simulation:
    ```sh
    ros2 launch dexter launch_sim.launch.py
    ```

2. Start the joystick node:
    ```sh
    ros2 run joy joy_node
    ```

3. Launch the teleop node with the Xbox configuration:
    ```sh
    ros2 launch teleop_twist_joy teleop-launch.py joy_config:=xbox
    ```

### Keyboard Control

To control the robot using the keyboard in the simulation, run the following commands:

1. Launch the simulation:
    ```sh
    ros2 launch dexter launch_sim.launch.py
    ```

2. Start the teleop keyboard node:
    ```sh
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
To view arduino interface 
screen /dev/ttyACM0 57600

Launch Rviz2 Node
rviz2 -d ~/dev_ws/src/Lidar_bot/config/view.rviz 