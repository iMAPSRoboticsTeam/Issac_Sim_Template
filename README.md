# Issac_Sim_Template

This template is used to help get started on developing simulations with Issac Sim.

# How To Use

## Create An Extension

First create a new extension by opening the extension window 
Window >> Extensions

Next click the green plus sign on the top left of the window, Select "New Extension Template Project"
Save this to any location

Give it a project name (No Spaces) and use iMAPS for the ext.id name

VS Code should now open.

## Clone The Issac_Sim_Template

Next, open a git bash terminal

Click on the three dots in the upper left corner of VS Code

Click "New Terminal" a terminal will open on the bottom of VS Code window. In this window, click on the down arrow next to the white plus sign, click Git Bash. 
> Assuming you have git installed on your windows machine click source control in VS Code to downolad.

This is a git bash terminal, here we can clone the repo into the extension by typing.

```
git clone https://github.com/iMAPSRoboticsTeam/Issac_Sim_Template.git
```

## Run the Setup Script

After the repo has been cloned, open file explorer, locate and double click the setup.bat file inside the repo. It should be inside your extension folder.

After you run this script the code template should now be ready to use. Issac Sim should now have a new extension window.

# Getting Started

This template sets up the developer with all the necessary functions to create and build simulations through a GUI. 

Inside the Extension.py file is the code for the GUI. Here you can copy the code below while changing the names of the assets that are stored in their respective dictionaries. 

```
dict = {
    "label": "Load Simulation", ## Give Simulation a Name
    "type": "button",
    "text": "Load",
    "tooltip": "Edit ToolTip",
    "on_clicked_fn": self._load_new_world,
}
self._buttons["Load New Simulator"] = btn_builder(**dict) # Give button a unique name
self._buttons["Load New Simulator"].enabled = True


dict = {
    "label": "Sim Reset", # Give reset button a name
    "type": "button",
    "text": "Reset",
    "tooltip": "Reset Simulation",
    "on_clicked_fn": self._on_reset,
}
self._resetButtons["Sim Reset"] = btn_builder(**dict) # Create a unique name for button
self._resetButtons["Sim Reset"].enabled = False
```


The Simulations are created within their own respected python files. The SimulationModule.py is an example of how to setup a simulation module for a specific purpose. Inside this file is where the robot and world are loaded into Issac Sim and where the simulation function (function that is used to control the simulation) are placed. 
This file stores a class that will be called within the Simulation.py file. This file is where the simulation runs and will call the simulation function within the respected simulation module file. The ArticulationAction class is called inside this file and is used to drive the joints inside the loaded robot file. 

# Good Luck

If you get stuck or need help understanding the code better, please email me 
```
cleydig@email.sc.edu
```

or text me. Issac Sim can be very complicated but hopfully this template will help you better understand the proccess with robotic simulation development.

Good Luck!







