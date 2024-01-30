from abc import abstractmethod
import omni.ext
from omni.isaac.ui.ui_utils import setup_ui_headers, get_style, btn_builder, state_btn_builder, cb_builder
import omni.ui as ui
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.core import World
from .Simulation import Simulation
import weakref
import os
import asyncio


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class Extension(omni.ext.IExt, Simulation):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id: str):
        self._menuItems = None
        self._buttons = None
        self._resetButtons = None
        self._GolbalReset = None
        self._ext_id = ext_id
        self.name = "Robot"
        self.robot = Simulation()
        self.extra_frames = []
        self.start_Extension()

        return

    def start_Extension(
            self,
            menu_name = None,
            submenu_name = None,
            name = "Simulation Suite",
            title = "Simulation",
            doc_link = None,
            overview = "This is a complete simulation package",
            file_path = os.path.abspath(__file__),
            number_of_extra_frames = 1,
    ):
        menu_items = [MenuItemDescription(name=name, onclick_fn=lambda a=weakref.proxy(self): a._menu_callback())]
        if menu_name == "" or menu_name is None:
            self._menu_items = menu_items
        elif submenu_name == "" or submenu_name is None:
            self._menu_items = [MenuItemDescription(name=menu_name, sub_menu=menu_items)]
        else:
            self._menu_items = [
                MenuItemDescription(
                    name=menu_name, sub_menu=[MenuItemDescription(name=submenu_name, sub_menu=menu_items)]
                )
            ]
        add_menu_items(self._menu_items, name)

        self._buttons = dict()
        self._resetButtons = dict()
        self._GolbalReset = dict()
        
        self._build_ui(
            name=name,
            title=title,
            doc_link=doc_link,
            overview=overview,
            file_path=file_path,
            number_of_extra_frames=number_of_extra_frames,
            ext_id=self._ext_id,
            )
         
    """

    USER INTERFACE
    
    """
        
    def _build_ui(self, 
                  name, 
                  title, 
                  doc_link, 
                  overview, 
                  file_path, 
                  number_of_extra_frames, 
                  ext_id,
                  ):
        self._window = ui.Window(name, width=360, height=0, visible=True, dockPreference=ui.DockPreference.LEFT_BOTTOM)
        with self._window.frame:
            with ui.VStack(spacing=5, height=10):
                #title = ("StakeBot Simulation")
                #doc_link = ("https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.ui/docs/index.html")
                # overview = (
                #     "This is a complete control pannel for simulating and testing the StakeBot\n"
                # )
                setup_ui_headers(ext_id, file_path, title, doc_link, overview)

                frame = ui.CollapsableFrame(
                    title="Simulation Name",
                    height=0,
                    collasped=False,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
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
                
                dict = {
                            "label": "Reset Simulation",
                            "type": "button",
                            "text": "Reset Simulation",
                            "tooltip": "Reset EtherBot Simulation",
                            "on_clicked_fn": self._reset_all,
                        }
                self._GolbalReset["Reset"] = btn_builder(**dict)
    """
    
    
    """

    def _load_new_world(self): # rename function to unique world name
        self.robot._loadWorld = "Simulation" # Rename this to unique world name
        async def _on_load_world_async():
            await self.robot.load_world_async()
            await omni.kit.app.get_app().next_update_async()
            self.robot._world.add_stage_callback("stage_event_1", self.on_stage_event)
            self._enable_all_buttons(self._buttons, False)
            self._resetButtons["Sim Reset"].enabled = True
            self.post_load_button_event()
            self.robot._world.add_timeline_callback("stop_reset_event", self._reset_on_stop_event)
            
            
        asyncio.ensure_future(_on_load_world_async())
        return
    
    def _menu_callback(self):
        self._window.visible = not self._window.visible
        return
    
    def _enable_all_buttons(self, type, flag):
        for btn_name, btn in type.items():
            if isinstance(btn, omni.ui._ui.Button):
                btn.enabled = flag
        return
    
    def on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.CLOSED):
            if World.instance() is not None:
                self.robot._world_cleanup()
                self.robot._world.clear_instance()
                if hasattr(self, "_buttons"):
                    if self._buttons is not None:
                        self._enable_all_buttons(self._buttons, False)
                       
        return
    
    def _reset_on_stop_event(self, e):
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            try:
                self.post_clear_button_event()
            except:
                pass
        return
    
    def _on_reset(self):
        async def _on_reset_async():          
            await self.robot.reset_async()
            await omni.kit.app.get_app().next_update_async()
            self.post_reset_button_event()
            self.robot._world.clear_instance()
        asyncio.ensure_future(_on_reset_async())
        return
    
    @abstractmethod
    def post_reset_button_event(self):
        return
    
    @abstractmethod
    def post_load_button_event(self):
        return
    
    @abstractmethod
    def post_clear_button_event(self):
        return
    

    def _sample_window_cleanup(self):
        remove_menu_items(self._menu_items, self.name)
        self._window = None
        self._menu_items = None
        self._buttons = None
        self._resetButtons = None
        self._GolbalReset = None
        return
    
    def shutdown_cleanup(self):
        return

    def on_shutdown(self):
        if self.robot._world is not None:
            self.robot._world_cleanup()
        if self._menu_items is not None:
            self._sample_window_cleanup()
        if self._buttons is not None:
            self._enable_all_buttons(self._buttons, False)
        if self._resetButtons is not None:
            self._enable_all_buttons(self._resetButtons, False)
        if self._GolbalReset is not None:
            self._GolbalReset["Reset"].enabled = False
        
        self.shutdown_cleanup()
        
        if self.robot._world is not None:
            self.robot._world.clear_instance()
            self.robot._world.clear()

        return
    
    def _reset_all(self):
        async def _on_reset_all_async():
            try:
                if self.robot._world is not None:
                    await self.robot._world.stop_async()
                    await omni.kit.app.get_app().next_update_async()
                    self.robot._world.clear_instance()
                    self.robot._world.clear()
                self._enable_all_buttons(self._buttons, True)
                self._enable_all_buttons(self._resetButtons, False)
            except:
                pass
          
        asyncio.ensure_future(_on_reset_all_async())
        return
