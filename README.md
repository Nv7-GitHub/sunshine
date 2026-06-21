# Sunshine Combat Robot
This is the code for a 1lb melty-brain combat robot. This is a combat robot whose body spins and the wheel speeds are modulated while spinning for translation. There are 3 main components to this robot:

- Robot brain (sunshine_brain): ESP32-S3, which reads the sensors, does the processing, and sends the telemetry via ESP-now to the receiver and receives motion commands and safety-related commands (e.g. enable, disable, etc.)
- Receiver (sunshine_receiver): ESP32-S3, which receives the telemetry and sends it to the host app as well as sends out telemetry commands from the host app
- Host app (sunshine_app): Graphs data, displays telemetry, saves logs to files, handles replay (has to interact with sunshine_brain code for this) and simulation

## Tech stack
All embedded code will be implemented via platform IO. The host app code will be Tauri with React for the frontend. For replay (explained below), the brain code that is above the IO layer has to be very encapsulated so that it can be called from the Tauri rust code.

## Replay
The core concept of this project is IO layers and replay. The core sensors: **time (precise)**, wheel eRPM, ESP-NOW RSSI, magnetometer reading, battery voltage, control inputs from the driver, and accelerometer readings are all sampled at 1kHz and logged via telemetry. 

All the brain code processes these inputs (which are below the IO layer) and runs code that affects the **state**. The state is every variable that changes over time as a result of previous inputs. This should just be the external and internal states of any filters being used. 

The telemetry packets are sent out at 50 Hz, so each packet should include two state snapshots — one at the start of the packet and one at its midpoint (the 10th of its 20 samples), giving the real state at 100 Hz — and then the 20 IO layer samples that happened since the start state. The two states let the host both initialize the filters and double-check the replay against the real state twice per packet. This will minimize data usage because the host app can simply receive the state to initialize the filters and then run replay from there on. In the log files, the real state is likewise logged at 100 Hz (two snapshots per 50 Hz frame; theoretically you could even do it once but I want to be able to always have something to double-check the replay against and/or a way to handle changed code since you collected the telemetry) and the data below the IO layer is logged at the full 1kHz. Variables are NOT logged — they are a pure function of state + inputs, so the host always recomputes them (both the real and replayed versions).

The replay should use the ACTUAL robot code and just feed it a different source of IO layers instead of the raw sensors. So if you make a code change, you could re-run the code using the logged telemetry data and see if it fixes a bug that is occurring, for example.

This IO layer concept is important because it lets you implement simulation, where instead of having the receiver managing state you just implement a basic simulation of the motors and how it affects the sensors in an ideal world and then you can simulate the robot fully on the computer, once again just feeding the simulated sensor data into the IO layer.

In addition to the **state** and **inputs** (data below the IO layer) you also have the **variables** - data that is calculated from the state or inputs without any historical components. This is also logged at 50 Hz, but never sent over telemetry, just logged by the host app. This is once again so that if code changes occur you don't lose the data that was produced by the old code, which is necessary for determining if changes actually led to fixes. 

Replay is designed with AI assistance in mind. Develop Claude skills to aid with debugging code with validation via replay and simulation in parallel to this. Design everything with debugging and ugpradeability in mind. Changing what's in the state, what the inputs are, etc. should be easy and backwards-compatible (so old log data should still be usable). Adding a new variable (especially to the state and variables, which I anticipate changing a lot) should be very easy and well documented for future AI sessions along with the codebase in general.

## Robot brain

The robot brain has the two components mentioned before: below the IO layer, and above the IO layer. Everything above the IO layer has to be able to be run both on the host computer as well as the robot, so it should never interact with hardware directly and should be able to easily be compiled and run on a variety of host computer platforms. For below the IO layer, here is the hardware map:

## Hardware map
IMU is an ADXL375, 11mm from the center of the robot (this is important for calculating ang vel from the centripetal accel). The IMU is rotated 45 degrees so it saturates less.
- IO8: IMU_INT1
- IO9: IMU_INT2
- IO10: IMU_CS
- IO11: IMU_MOSI
- IO12: IMU_SCK
- IO13: IMU_MISO

Mag is LIS3MDL
- IO14: MAG_DRDY
- IO15: MAG_MOSI
- IO16: MAG_SCK
- IO17: MAG_MISO
- IO18: MAG_CS
- IO21: MAG_INT

There is an LED connected to IO38 that can be PWMed. It's connected to the gate of a low side switch so you drive it high to turn on the LEDs.

IO7 is connected with a voltage divider (2kohm high, one kohm low) to the battery positive terminal

IO4 is the signal pin of the left wheel ESC (S1), IO5 is for the right wheel ESC (S2). Use bidirectional DShot 600.

USB is connected, so you can use USB for debugging info

## Core splitting
There are 3 overall tasks running on the brain: telemetry, navigation, and control. 

### Telemetry
Telemetry should be running on the ESP32 core that is meant for telemetry. The data is getting sent out at a fixed 50 Hz and should be received from the transmitter at 500 Hz. This core should receive state (the history, or maybe it buffers the history itself but as explained earlier in each packet there's the state and then the 50 subsequent inputs). This core also uses the RSSI from the receiver and sends it over to the control core as this is one of the inputs. The receiver and transmitter should be configured to send at the absolute max strength, and the transmitter should be sending without waiting for acknowledgement or anything. For the telemetry it should be robust since any missing data will mess up replay, so determine a protocol that is able to do that without causing congestion so the packets with RSSI and control inputs can be sent at the 500 Hz. This is in a very RF noisy environment so account for that appropriately.

IMPORTANT: When the robot is disabled in the host app, it HAS to disable in real life. This is a strict requirement as it is critical to safety.

### Navigation
Navigation is the 1kHz sensor read and filter update loop. All the sensors can be read at 1kHz or faster (make sure to configure correctly) and all are connected via SPI so it should be fast. The filter is as follows:
- The accelerometer data is used to determine angular velocity. When being transmitted, this is an int16
- The magnetometer data goes through a butterworth filter with a cutoff roughly in the range of what Earth's magnetic field should be oscillating at. Calculate the minimum speed needed before zero DC leaks through and set that as the minimum speed to use this sensor, as there will be a lot of DC offset due to current draw from the ESC and various metallic objects. The raw reading is also an int16 for telemetry. The filter will be part of the state (and needs appropriate history). 
- The wheel ERPM is not going to be used for anything navigation-related. However, it can be used for torque-limiting and for looking at things like wheel slip in the logged data after. Log this as a float16.
- The RSSI is from the actual ESP-NOW receiving data. This is an int8. This is eventually going to be worked into the filter but I think the shape of it's curve depends on antenna stuff so I will collect some data not using it initially and try to analyze it before incorporating it into anything. 
- The host computer sends x, y, theta, and throttle. X/Y/Theta are all int8 and throttle is uint8. 
- The battery voltage can be represented as a int8, relative to 7.6V (the robot is 2S). The range needs to go to 5.0V and be symmetrical.
- Put the applied DShot signals into the inputs too, but quantize down to uint8 for each since I just want to see the overall shape at 1kHz, it is never going to be used anywhere.
- Time should be of the necessary precision to be accurate down to a decimal OF the control loop (so since the loop is at 1kHz, it should be accurate to ten-thousandths of a second) which means a uint32 is needed.

Make sure to consider endian-ness correctly for telemetry or use some protocol.

The overall Kalman filter will use the magnetometer as an absolute reference (since with the DC filtered out from the butterworth it should actually represent absolute position), the accelerometer gives velocity for the kalman filter, and eventually the RSSI will either give velocity or position but that is to be determined. 

The theta control defines an offset from the absolute position from the magnetometer. That way the driver can rotate the zero heading reference. 

The kalman filter will need to be tuned somewhat. Develop necessary code (this will probably happen during bringup) and include instructions to tune it. 

### Control
Control also happens in the same 1kHz loop. It is CRITICAL that this tight 1kHz loop does not overrun, and I should know about it when it is happening. The inputs include precise time so the host app should be receiving this, and of course loop overruns should be included in the USB output along with any initialization errors for the sensors. Make sure to have a distinct LED pattern if there are initialization errors too with the sensors, since usually I won't have USB connected during init so I would connect USB once I see that pattern (so the error should be continuously printed). 

During control, the LED has to be turned on from -3 deg to +3 deg (of course relative to the driver-controlled zero heading). 

The X and Y controls will be used for translational control. For the translational drift, use triangle waves in a portion of the rotation such that it can drive. The amplitude of the wave should depend on the magnitude of the X/Y control. Write a document on how to tune this (what to change and how that will affect the motion and what I should look for). I am thinking of a triangle wave instead of the usual square because the melty should be traction limited and a triangle wave should limit slip. Maybe a trapezoidal wave instead of a triangle if that would be more optimal.

There are two modes of control. There is DISABLED, which is critical to safety and the host app will control this. Then there is TANK, where the robot is driven around like a traditional robot, so throttle controls forwards backwards and the x control (side-to-side) should determine turning. Y and theta are unused. Finally, there is MELTY where throttle controls target angular speed (for now open-loop), X is for side-to-side, Y is forwards backwards, and theta controls the offset. This is holonomic unlike the previous tank drive mode so you can of course mix the X and Y. In the app design mockup (included below) there is only ENABLED and DISABLED so this needs to be changed to have DISABLED, TANK, MELTY.

## Bringup
There is no way that the entire codebase can be put onto the device and everything will work, so bringup needs to occur where each specific function has to be individually tested. Design a clean way of doing this. Here are the levels of bringup:
1. Low-level sensors: Read the accelerometer, plot via USB, make sure that works, same for the magnetometer, voltage sensor. etc.
2. Low-level control: Test the bidirectional DShot and make sure everything spins up correctly
3. Telemetry link: Test the ESP-NOW connection between the brain and the receiver, including sending the inputs and receiving the RSSI data. At this point the host app has to get in the loop too, and I should be able to plot accel, rssi, etc.
4. Navigation tuning: The kalman filter and other parameters must be tuned so that the device can localize well. At this point I will use the tank mode to spin the robot, MELTY mode will still not be used.
5. Drift tuning: MELTY mode is finally enabled, heading control should be good (so the LED should be solid and still), and I can tune the drift profile

Make sure it is an easy process and write instructions.

# Receiver
The receiver is simply the other side of the link from the robot brain. It should receive the necessary info and send it via USB (it is an ESP32-S3 so should have plenty of bandwidth) to the host app and send data. The accompanying protocols must be developed. 

# Host app
The UI mockup has been made:

## UI Mockup
Fetch this design file, read its readme, and implement the relevant aspects of the design. https://api.anthropic.com/v1/design/h/ER_Esv7z8Q_IkM8_wpP-5Q?open_file=Sunshine+Dashboard.html
Implement: Sunshine Dashboard.html

Note that this mockup has some unnecesary info plotted on top (e.g. CPU usage doesn't make sense, but seeing the bandwidth over the connection or the RSSI would be useful up there), and the variables are fully made up. Instead of ENABLE DISABLE there should be DISABLE, TANK, MELTY. In addition to x-y and throttle there is also the theta control. The current keybinds are:
W/S for Y, A/D for X, left/right arrow for theta, and up/down arrow for throttle (where throttle adds up instead of being like when you let go it goes to zero). This should be HEAVILY low-pass filtered (even more than the current mockup) so that I can basically emulate having a controller by pulsing WASD and the left/right arrow keys.

The connections section of the UI doesn't make sense. Robot is 2S so of course adjust battery indicator colors accordingly. Get rid of the E-STOP button as that's the same as DISABLE. Inputs menu is unneeded. Retain the same graph controls but fix it because right now it's a little jerky and awkward (way too sensitive to the scroll). 

You should be able to do replay, simulation, and connect to the real robot through the UI. Implement that stuff into where the "CONNECTIONS" area is currently. For replay you would pick a log file, simulation just spins up a simulation, and connecting to the robot would connect to the receiver and then look at whether the receiver is connected and all. Error handling will need to be coordinated across all devices in the chain in a robust and extendable manner.

## Logging
The code needs to log a decent amount of data at 50 Hz (where each 50 Hz frame has those 20 inputs). Use binary so the file is small and of course design around the backwards compatibility aspect so that old files can be used with versions of the software that have new variables.

## Plotting
You should be able to plot both the actual 50 Hz data as well as your replayed data for the state variables (this applies when connected to the real robot as well as when replaying from a data file, in simulation it doesn't really make sense but they can probably just be the same thing but one is at 50 Hz). The only reason we replay even when connected to the real robot is so that we can see what's going on at 1kHz, so the replayed data should look like a high-resolution version of the 50 Hz data. Make sure to design the graphing UI such that it can handle the high density of points / quantize since of course rendering at 1kHz when you are showing a minute of data would probably cause lag issues, so consider that when implementing the graphing. 

## Inputs, State, and Variables
All things being plotted have their own units. You can already see this system well implemented in the mockup.

The inputs should all be plotted under the inputs dropdown. Most of the state (e.g. the kalman filter internal covariances, butterworth history) don't make sense to plot so only things like position, velocity, and the magnetometer data after the filter really make sense to plot.

Even though the control outputs are logged at 1kHz as part of the inputs, they will get re-calculated during replay and so there should be a way of viewing the new control outputs too. That's why even though during telemetry they are part of the inputs in the app graphing they should be part of the STATE since they have separate values received vs replayed. We basically have a Real State and a Replayed State which of course includes the 50 Hz things above but also the 1kHz wheel applied values.

The variables are only present in the replayed values, as they are never sent from the robot. Think of all important values. e.g. velocity calculated from the accelerometer would be a variable that should be plotted. All possibly useful info should be plottable.

The app has to call the actual C code to emulate the control and state estimation. This will require calling of C code from Rust. 

## Simulation
Use the basic electrical model of a brushed motor. (Back-EMF source, resistance, etc.). The KV of the motors is 1100KV (this should be an easily editable constant though). I don't know the exact resistance but lets say each phase is 75mohm. The battery is a 2S 650mah 120C LiPo. Don't model battery draining but model internal resistance using like the supply current derived from the stator current in the motor, pick a reasonable internal resistance for a battery with these specs. For the magnetomer, just do ideal earth's magnetic field, same for accelerometer. here are some more physical parameters you need:
- Wheel Diameter: 44mm
- The wheels are 81mm apart (so radius from center is 40.5mm). 
- The MoI of the robot is 1213859.17531 g-mm^2
- Wheel MoI is 6407.44019 g-mm^2
Use metric for everything.

I believe the max body RPM is somewhere around 4,000, you should double check this to ensure I gave you the right-specs.

# General Notes
Everything should be implemented robustly. If everything is connected and setup but then I unplug and replug the receiver, the host app should handle that gracefully and make a new log file. There also should be an easy way to disable logging from the host app. If the robot momentarily loses power or disconnects and then reconnects the host app and receiver should handle this gracefully. The disabled aspect is critical to safety. The code should be very well documented (in MD files) such that future AI sessions can read an info document and know what files to edit. There should be instructions for tuning meant for humans. Also make it easy to label log files (e.g. during comp it should be based on match, but always include date and stuff)

IMPLEMENT SKILLS TO AID WITH DEBUGGING!
