# Go2OccMapAbstractionROS2ws
Unitree Go2 ROS2 workspace containing all the tools needed for conducting realworld testing of the radiation aware occupancy map abstraction based framework for unknown environment exploration.

---
🧩 Included Packages

🔹 Bringup and System Integration
    bringup_go2 (Main Bringup Package)
    The primary entry point for the system.
    	•	Launch files for starting the full exploration stack
    	•	Robot configuration and parameter loading
    	•	TF tree initialisation
    	•	Middleware and namespace configuration
    	•	Optional visualisation and debugging tools

🔹 SLLIDAR Package for using the SLLIDAR S2 2D LiDAR

🔹 Marvelmind driver and ROS2 radiation sensor node for radiation emulation

🔹 Platform and Interface Packages

---

⚠️ Development Status
	•	Workspace structure is functional but evolving
	•	Package APIs and launch files may change
	•	Some experimental or legacy nodes may still be present
	•	Documentation is partial and will be expanded

This repository should currently be treated as a research workspace, not a production-ready robotics stack.

🚀 Running the System

Important: The workspace is still under active development and cleanup.

---

🔧 Planned Improvements
	•	Workspace and package refactoring
	•	Clear separation between simulation and hardware bringup
	•	Consolidation of launch files
	•	Parameter documentation
	•	Minimal working examples for each major subsystem

---

📄 Licence and Usage

Licensing will be finalised once the workspace structure stabilises.
Please contact the author before redistributing or reusing this codebase.

---

📫 Contact

Author: David Batty
Email: dwbatty@liverpool.ac.uk
Affiliation: University of Liverpool — School of Engineering
Platform: Unitree Go2
Research Area: Radiation-aware autonomous exploration and navigation
