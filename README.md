# Teleoperation-schemes
Three basic teleoperation schemes are implemented: Position-Position, Force-Position, Force/Position-Position. The three teleoperation schemes has been implemented as a vrep plugin. The master robot is the Haptic device that can be easily manage by the human, the slave robot is a 3DOFs robot represented in vrep with a red ball. 

# Prerequisites 
What you need:
  - Windows operating system
  - Visual Studio 2013
  - Vrep
  - OpenHaptics API : https://3dsystems.teamplatform.com/pages/102774?t=r4nk8zvqwa91
  - Chai3D for Visual Studio : http://www.chai3d.org/download/releases
  
The software are tested using Chai3D with Visual Studio Community 2013 and using Vrep ProEdu (32 bit) on Windows 8 (64 bit) laptop.

# Installation
  a. Compile all the CHAI3D resources by running the Chai3d project, depending ob your version of Visual Studio.  
  b. Copy the folder $(CHAI3DPROJECTPATH)/modules/V-REP in the $(CHAI3DPROJECTPATH)/modules with another name
  c. Substitute the Visual studio project in  $(CHAI3DPROJECTPATH)/modules/(YOURPROJECTNAME) with the .cpp file of this repository.

# Running
1. Build the solution in debug mode of Vrep plugin in Visualstudio.
2. Copy the files present in the folder $(CHAI3DPROJECTPATH)/bin/Debug in the vrep folder.
3. Connect the haptic device to the laptop and perform haptic diagnostic using the Diagnostic program. 
4. Open vrep and load the scene. Check on the terminal that there are no errors reported. 
5. Run the scene. 
