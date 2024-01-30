import omni.isaac.core.utils.stage as stage 
from omni.isaac.core.robots import Robot
import numpy as np
import os

class SimulationModule(): # Rename to unique module name

    def setup_scene(self, sim):
        
        usd_path = "Path_to_file"
        world = sim.get_world()
        world.scene.add_default_ground_plane()
        stage.add_reference_to_stage(usd_path=usd_path,
            prim_path="/World/Robot")
        world.scene.add(Robot(
            prim_path="/World/Robot", 
            name="Robot", 
            scale=np.array([0.01,0.01,0.01]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            position=np.array([0.0, 0.0, 0.3]),)
        )   
        return

    def setup_post_load(self, sim, func):
        sim._world = sim.get_world()
        sim._robot = sim._world.scene.get_object("Robot")
        sim._world.add_physics_callback("sending_actions", callback_fn=func)
        
        return
    
    def simulation(self):
        pass