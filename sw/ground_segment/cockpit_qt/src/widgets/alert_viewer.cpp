#include "alert_viewer.h"
#include <QScrollBar>
#include "PprzApplication.h"
#include "PprzToolbox.h"
#include "tools/speaker.h"
#include "common/gcs_utils.h"

#include "tools/AircraftManager.h"
#include "tools/dispatcher_ui.h"


AlertViewer::AlertViewer(QWidget *parent) : QWidget(parent), lastText("")
{
    auto layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    view = new QTextEdit(this);
    view->setReadOnly(true);
    view->setMinimumHeight(40); // Allow dock widget to shrink vertically
    layout->addWidget(view);
    
    for (auto ac : AircraftManager::get()->getAircrafts()) {
        connect(ac->getStatus(), &AircraftStatus::text_message, this, &AlertViewer::addMessage);
    }
    
    connect(DispatcherUi::get(), &DispatcherUi::new_ac_config, this, [=](QString ac_id) {
        auto ac = AircraftManager::get()->getAircraft(ac_id);
        connect(ac->getStatus(), &AircraftStatus::text_message, this, &AlertViewer::addMessage);
    });

    if (appConfig() && !appConfig()->value("EDIT_MODE").toBool()) {
        addMessage("Waiting for telemetry...");
    }
}

void AlertViewer::addMessage(const QString& text)
{
    if (text != lastText) {
        QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
        QString formatted = QString("%1 %2").arg(timestamp, text);
        view->append(formatted);
        lastText = text;
        
        QScrollBar *sb = view->verticalScrollBar();
        sb->setValue(sb->maximum());

        if (pprzApp() && pprzApp()->toolbox() && pprzApp()->toolbox()->speaker()) {
            pprzApp()->toolbox()->speaker()->addSentence(text);
        }
    }
}


/* --- HISTORICAL OCAML COMMENTS: speech.ml ---
 * If the os is Darwin, then use "say"
 ------------------------------------------
 * If the os is Linux, use "spd-say"
 ------------------------------------------
 * Add more cases here to enhance support
 ------------------------------------------
*/
