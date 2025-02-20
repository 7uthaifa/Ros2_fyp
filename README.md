To Get the Packages:
git clone https://github.com/7uthaifa/Ros2_fyp.git

Network Configuration:
- WIFI Name: hamze
- WIFI Password: talin(10)  # This name and password are known to the Raspberry Pi

1. Check connection:
   ping 192.168.100.176  # Depends on the IP you got (use Angry IP Scanner)
2. Connect via SSH:
   ssh pi@192.168.100.176
3. Raspberry Pi Password: 1234

Testing Movement:
- On Development PC:
   ros2 run whag_bot omni_teleop
- On Raspberry Pi:
   ros2 run whag_bot cmd_to_pwm_node
   
Joystick Control:
1. Start joystick control:
   ros2 launch whag_bot_joystick joystick.launch.py
2. Run command to PWM node:
   ros2 run whag_bot cmd_to_pwm_node
   
Visualization & Simulation:
1. Launch RViz:
   ros2 launch whag_description display.launch.xml
2. Start Gazebo simulation:
   ros2 launch whag_bringup whag_gazebo.launch.xml
   
Line Follower Parameter:
ros2 param set /line_follower_node angular_divisor 200.0

Shutdown Raspberry Pi:
sudo shutdown -h now
