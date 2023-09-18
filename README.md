# AgOpenGPSSimPoC
Proof of concept simulator ("digital twin"?) made with Godot game engine for AgOpenGPS.

Uses Godot game engine (and it's physics engine) to simulate tractor's behavior in a virtual world. Steering commands are received from and latitude/longitude/height/heading/roll/etc are sent to AgOpenGPS (or actually AgIO) using UDP-protocol. From the AgOpenGPS' (AgIO's) perspective this basically emulates All-In-One-board's (V4.1) UDP-communication as implemented in teensy-code. This emulation, however, implements only minimum set of features to get this working somehow (as this is just a PoC after all).

Made with Godot version 4.1.1, which you can download here: https://downloads.tuxfamily.org/godotengine/4.1.1/ This should be multi-platform but I have only tested this in windows 10 for now. Requirements should not be very high, I'd guess any mid-range laptop manufactured during the last 10 years or so should probably be enough to run this.

You can drive around the virtual world using keys and UI-controls in the left upper corner.

Controls:
- W,S,A,D: control tractor's speed and steering
- Space: Set tractor's target speed to 0 and steering straight
- Enter: Engage/disengage automatic steering
- F1: Switch to "First person flyer" camera (use CTRL to activate/deactivate mouse look)
- F2: Switch to follow camera
- F3: Switch to tractor driver's view
- F12: Full screen toggle
- CTRL: Activate/deactivate mouse look for "First person flyer" camera. This needs to be deactivated before you can use your mouse for other purposes again.
- U,H,J,K & Y,I (& O,L for roll if using 6 dof-mode): Fly around with the "First person flyer" camera
- Tab: Switch between "FPS" and "6 dof" flying modes (for "First person flyer" camera)
- Mouse wheel & plus & minus keys: Change flying speed of "First person flyer" camera
- B: Create a new 300 kg ball (physics object)
- Mouse buttons (when flying with "First person flyer" camera): Activate physics "manipulators"

Notes:
- You need to set your ethernet's IP-address to 192.168.5.126 and connect your tablet to your computer using an ethernet-cable (or using a router or something). Due to IP-address-issues it is currently not possible (or maybe using a router and port forwarding?) to run AgOpenGPS(/AgIO) and simulator on the same computer.
- You may need to allow firewall to pass Godot Engine's traffic as this uses UDP-protocol for communication.
- Simulated area is located in the middle of South Atlantic ocean (near zero latitude/longitude). This was done to make calculations far easier (this being just a PoC...).
- Based on dual antenna setup (although only one antenna can be seen, because teensy-code calculates heading/roll internally and therefore there's no need for the location of the second antenna here). You need to configure AgOpenGPS/AgIO accordingly.
- Tractor model was loosely based on Valmet 405 (as seen on the video https://youtu.be/IhYAJ6jgmnk?feature=shared). This tractor has a wheelbase 2.26 m, antenna location is 28 cm back, 258 cm up and 97 cm left (I remember there being some strange discrepancy about the sideway coordinates when testing the real setup and now looking at the simulated tractor and setting in AgOpenGPS they do not seem to match... Not investigating this further now, though. If trying this, just use -97 to sideways shift-setting).
- Do not use this as a base for anything serious. This was hacked together in few days, is super ugly and contains several dirty hacks to get things working on a rudimentary level.
