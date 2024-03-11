**CHANGELOG**:  <br> 
&ensp;&ensp;1. CHANGED several button bindings for efficiency and convenience  <br> 
&ensp;&ensp;2. REMOVED unused/detrimental code; namely, the AutoAlign command and example autonomous code  <br> 
&ensp;&ensp;3. ADDED HeldAutoArm.java for held arm PIDs  <br> 
&ensp;&ensp;4. RENAMED AutoArm.java to InstantAutoArm.java  <br> 
&ensp;&ensp;5. RENAMED TeleopIntake.java to TeleopJukebox.java  <br> 
&ensp;&ensp;6. ADDED a fix to the runJukebox instant command within Jukebox.java  <br> 
&ensp;&ensp;7. ADDED a fix to brake-related issues for the arms  <br> 
&ensp;&ensp;8. FIXED an issue with the (now deleted) AutoAlign command interfering with the swerve code  <br>   
**ISSUES & POTENTIAL ERRORS**:  <br> 
&ensp;&ensp;1. The code for the vision extension of swerve drive **does not work**  <br> 
&ensp;&ensp;2. The current default position **is incorrect**  <br> 
&ensp;&ensp;3. *Potential* issue with arm angle offset values bugging the code  <br> 
&ensp;&ensp;4. The original PID within the Arms.java instant command method **has accuracy issues**  <br> 
&ensp;&ensp;5. The new MotionMagic PID **does not run** its PID, instead only returning a message  <br> 
&ensp;&ensp;6. *Potential* issue with the swerve subsystem's instant command not functioning properly  <br>   
**TODO LIST**:  <br> 
&ensp;&ensp;1. FIX the new MotionMagic PID instant command within the Arms.java subsystem  <br> 
&ensp;&ensp;2. FIX the limelight additions in the TeleopSwerve.java command  <br> 
&ensp;&ensp;3. CREATE a viable vision code  <br>   
**UNUSED CODE**:  <br> 
&ensp;&ensp;1. InstantAutoArm.java command  <br>   
**NOTES**:  <br> 
&ensp;&ensp;1. Ethan is currently working on the autonomous code; as such, issues and to-dos relating to the autonomous code has been removed  <br> 
&ensp;&ensp;2. **ALL** code *ABSOLUTELY* needs to be peer-reviewed **AND** tested!!!  <br>   
**CREDITS**:  <br> 
&ensp;&ensp;1. Code Created By: Drake Nguyen, Evan Wang, and Ethan Jiang  <br> 
&ensp;&ensp;2. Robot Created By: FRC Robotics Team 6934 (ACHS Scorpions)  <br>   

**DRIVER'S GUIDE**:  <br> 
&ensp;&ensp;1. **DRIVE CONTROLLER** (**PORT 0**):  <br> 
&ensp;&ensp;&ensp;&ensp;* Field-Centric Driving: *MOVE* Left Joystick (x & y)  <br> 
&ensp;&ensp;&ensp;&ensp;* Field-Centric Strafing: *MOVE* Right Joystick (x)  <br> 
&ensp;&ensp;&ensp;&ensp;* Robot-Centric Driving: *HOLD* Left Bumper + *MOVE* Left Joystick (x & y)  <br> 
&ensp;&ensp;&ensp;&ensp;* Robot-Centric Strafing: *HOLD* Left Bumper + *MOVE* Right Joystick (x)  <br> 
&ensp;&ensp;&ensp;&ensp;* Slow-Mode (ALL Driving & ALL Strafing): *HOLD* Right Bumper + *USE* Driving/Strafing Controls  <br> 
&ensp;&ensp;&ensp;&ensp;* Reset Gyro (Field-Centric Driving ONLY): *PRESS* Y-Button  <br> 
&ensp;&ensp;&ensp;&ensp;* Align to Target: *HOLD* X-Button  <br> 
&ensp;&ensp;&ensp;&ensp;* Get Driver Info: *PRESS* Start-Button  <br> 
&ensp;&ensp;2. **WEAPONS CONTROLLER** (**PORT 1**): <br> 
&ensp;&ensp;&ensp;&ensp;* Arm Movement: *MOVE* Left Joystick (y)  <br> 
&ensp;&ensp;&ensp;&ensp;* Climber Movement: *MOVE* Right Joystick (y)  <br> 
&ensp;&ensp;&ensp;&ensp;* Slow-Mode (Arm ONLY): *HOLD* Left Joystick **BUTTON**  <br> 
&ensp;&ensp;&ensp;&ensp;* Arm PID to Reset Position: *PRESS* Y-Button **IF INSTANT PID**  <br> 
&ensp;&ensp;&ensp;&ensp;* Arm PID to Reset Position: *HOLD* Y-Button **IF HELD PID**  <br> 
&ensp;&ensp;&ensp;&ensp;* Arm PID to Trap Position: *PRESS* A-Button **IF INSTANT PID**  <br> 
&ensp;&ensp;&ensp;&ensp;* Arm PID to Trap Position: *HOLD* A-Button **IF HELD PID**  <br> 
&ensp;&ensp;&ensp;&ensp;* Arm PID to Amp Position: *PRESS* B-Button **IF INSTANT PID**  <br> 
&ensp;&ensp;&ensp;&ensp;* Arm PID to Amp Position: *HOLD* B-Button **IF HELD PID**  <br> 
&ensp;&ensp;&ensp;&ensp;* Arm PID to Speaker Position: *PRESS* X-Button **IF INSTANT PID**  <br> 
&ensp;&ensp;&ensp;&ensp;* Arm PID to Speaker Position: *HOLD* X-Button **IF HELD PID**  <br> 
&ensp;&ensp;&ensp;&ensp;* Intake Note: *HOLD* Right Bumper  <br> 
&ensp;&ensp;&ensp;&ensp;* Manually Shoot Note: *HOLD* Right Trigger  <br> 
&ensp;&ensp;&ensp;&ensp;* Automatically Shoot Note: *PRESS* Left Bumper  <br> 
&ensp;&ensp;&ensp;&ensp;* Get Weapons Info: *PRESS* Start-Button  <br>   

**LAST UPDATED**: 3/8/24
