from omni.isaac.ui.ui_utils import setup_ui_headers, get_style, btn_builder, state_btn_builder, cb_builder
import omni.ui as ui

class UserInterface():
    def __init__(self):
        pass

    def _build_ui(self, 
                  name, 
                  title, 
                  doc_link, 
                  overview, 
                  file_path, 
                  number_of_extra_frames, 
                  window_width,
                  keep_window_open,
                  ext_id,
                  ):
        self._window = ui.Window(name, width=window_width, height=0, visible=keep_window_open, dockPreference=ui.DockPreference.LEFT_BOTTOM)
        with self._window.frame:
            with ui.VStack(spacing=5, height=10):
                #title = ("StakeBot Simulation")
                #doc_link = ("https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.ui/docs/index.html")
                # overview = (
                #     "This is a complete control pannel for simulating and testing the StakeBot\n"
                # )
                setup_ui_headers(ext_id, file_path, title, doc_link, overview)
                
                # frame = ui.CollapsableFrame(
                #     title="Motor Simulation",
                #     height=0,
                #     collasped=False,
                #     style=get_style(),
                #     style_type_name_override="CollapsableFrame",
                #     horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                #     vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                #)
