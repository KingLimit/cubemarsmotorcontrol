# CubeMars Motor Controller Supper

This project uses an STM32 to control a CubeMars motor over CAN, with real-time input from an Xbox controller. The controller is used to command motion, while a PD control loop maintains the motor’s position when no input is given, allowing it to resist external disturbances and return to its commanded position if moved by hand.

The control is velocity-based from the controller, with filtering, position rate limiting, and gain ramping to reduce twitching and sudden jumps. The system is designed to feel responsive during user input while still maintaining stable position holding when disturbed.

## Hardware
- STM32 Nucleo-F446RE  
- Waveshare CAN Bus Shield  
- CubeMars AK-series motor  
  - AK70-10 KV100 Tested
  - AK80-64 KV80 Tested
 
## Software
- STM32CubeMX used to create and edit initial .ioc file and generate project  
- STM32CubeIDE used to edit main.c and flash to board
- Python script used for Xbox controller input and UART communication  
  - pygame for controller input  
  - pyserial for serial communication to STM32

## Controller (Xbox)
- Right Trigger → Clockwise motion (trigger-scaled velocity)  
- Left Trigger → Counterclockwise motion (trigger-scaled velocity)
- Right Bumper → Cycle Motor Number Up
- Left Bumper → Cycle Motor Number Down
- A Button → Deadman switch (must be held for motion)  
- Y Button → Reset to home position  
- D-Pad Up → Set home position  
- D-Pad Down → Unlock motion control  
- Start Button → Enable motor  
- Back Button → Disable motor  
- X Button → Lower speed setting  
- B Button → Raise speed setting  

## Notes
CAN communication is handled through the Waveshare shield, with PB8/PB9 used for RX/TX. Basic filtering is applied to accept all messages.

Motion is velocity-based and integrated into position for smooth control. Position commands are rate-limited and gains are ramped to prevent sudden jumps and reduce jitter.

The motor will only move when it is enabled, homed, unlocked, and the deadman button is held.

Gains (Kp, Kd) and filtering parameters can be adjusted depending on the setup and how stiff or compliant you want the motor to feel.
