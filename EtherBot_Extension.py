import omni.ext
from .UserInterface import UserInterface
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from .EtherBot_Sim import EtherBotSimulation
import weakref
import os


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class AdventinnousExtension(omni.ext.IExt, EtherBotSimulation):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id: str):
        self._menuItems = None
        self._buttons = None
        self._ext_id = ext_id
        self.EtherBot = EtherBotSimulation()
        self._ui = UserInterface()
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
            window_width = 360,
            keep_window_open = True,
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
        
        self._ui._build_ui(
            name=name,
            title=title,
            doc_link=doc_link,
            overview=overview,
            file_path=file_path,
            number_of_extra_frames=number_of_extra_frames,
            window_width=window_width,
            keep_window_open=keep_window_open,
            ext_id=self._ext_id,
            )
       

    def on_shutdown(self):
        print("shutdown")
