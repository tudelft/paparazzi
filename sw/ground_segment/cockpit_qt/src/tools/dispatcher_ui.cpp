#include "dispatcher_ui.h"

DispatcherUi::DispatcherUi(PprzApplication* app, PprzToolbox* toolbox) : PprzTool(app, toolbox),
    selected_ac_id("")
{
    connect(this, &DispatcherUi::ac_selected, [=](QString ac_id){
        selected_ac_id = ac_id;
    });

}
