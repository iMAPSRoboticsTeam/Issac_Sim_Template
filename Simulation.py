from .SimulationBase import SimulationBase
from .SimulationModule import SimulationModule
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np


class Simulation(SimulationBase):
    def __init__(self) -> None:
        super().__init__()
        self._loadWorld = None
        self.simMod = SimulationModule() 
        ## Add new Simulation Modules
        return
       
    # Setup Scene
    def setup_scene(self, loadWorld):
        if loadWorld == "Simulation":
            self.simMod.setup_post_load(self, self.send_actions)
        # elif loadWorld == "NewSim":
            # load new sim module

        return
    
    # Setup Post Load
    async def setup_post_load(self, loadWorld):
        if loadWorld == "Simulation":
            self.simMod.setup_post_load(self, self.send_actions)
        # elif loadWorld == "NewSim":
            # load new sim module

        return
    
    def send_actions(self, step_size):
        
        self.simMod.simulation()
        self._robot.apply_action(ArticulationAction(joint_positions=None,
                        joint_efforts=None,
                        joint_velocities=np.array([5,5,5,5])*-1))
                                                        
        return
    
