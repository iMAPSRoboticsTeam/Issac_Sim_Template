from .SimulationBase import SimulationBase


class Simulation(SimulationBase):
    def __init__(self) -> None:
        super().__init__()

        return
       
    # Setup Scene
    def setup_scene(self, loadWorld):
        if loadWorld == "TestDrive":
            # Setup Respected Scene from Object
            pass

        return
    
    # Setup Post Load
    async def setup_post_load(self, loadWorld):
        if loadWorld == "TestDrive":
            # Setup Respected Post Scene from Object
            pass

        return
    
