#include "plugin.h"
#include "MotorAction.h"
#include <cedar/processing/ElementDeclaration.h>

void pluginDeclaration(cedar::aux::PluginDeclarationListPtr plugin)
{
    cedar::proc::ElementDeclarationPtr summation_decl
    (
        new cedar::proc::ElementDeclarationTemplate <MotorAction>("Utilities")
    );
    plugin->add(summation_decl);
}
