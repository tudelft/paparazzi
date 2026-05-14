#ifndef WATCHER_H
#define WATCHER_H

#include <QObject>
#include "PprzToolbox.h"
#include "PprzApplication.h"
#include "pprz_dispatcher.h"

class Watcher : public PprzTool
{
    Q_OBJECT
public:
    explicit Watcher(PprzApplication* app, PprzToolbox* toolbox);
    void setToolbox(PprzToolbox* toolbox) override;

signals:

private:

};

#endif // WATCHER_H
