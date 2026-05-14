#include "speaker.h"
#include "pprz_dispatcher.h"
#include "gcs_utils.h"
#include "AircraftManager.h"

Speaker::Speaker(PprzApplication* app, PprzToolbox* toolbox) : PprzTool(app, toolbox),
    locale("en_GB")
{
     speechProcess = new QProcess(this);
    connect(speechProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
            [this](int exitCode, QProcess::ExitStatus exitStatus){ 
                (void)exitCode; (void)exitStatus; 
                this->say(); 
            });
}

void Speaker::addSentence(QString txt, int priority, int validity) {
    if(!speech()) {
        return;
    }
    sentences.append({txt, priority, QTime::currentTime().addMSecs(validity*1000)});
    say();
}

#include <QProcess>
#include <QSysInfo>

void Speaker::say() {
    if(!speech()) {
        return;
    }

    if (speechProcess->state() != QProcess::NotRunning) {
        return;
    }

    auto now = QTime::currentTime();

    auto it = QMutableListIterator(sentences);
    while(it.hasNext()) {
        auto next = it.next();
        if(next.expiration < now) {
            it.remove();
        }
    }

    if(!sentences.isEmpty()) {
        int prio = -1;
        int index = 0;

        for(int i=0; i<sentences.size(); ++i) {
            if(sentences[i].priority > prio) {
                prio = sentences[i].priority;
                index = i;
            }
        }

        auto s = sentences.takeAt(index);
        
#if defined(Q_OS_MAC)
        speechProcess->start("say", QStringList() << s.text);
#elif defined(Q_OS_LINUX)
        speechProcess->start("espeak", QStringList() << s.text);
#else
        qDebug() << "Current OS not supported by -speech option:" << s.text;
#endif
    }
}

void Speaker::enableSpeech(bool s) {
    setSpeech(s);
    if(s) {
        for(auto &msg: messages) {
            bindMessage(msg);
        }
    }
}

void Speaker::setLocale(QString l) {
    locale = l;
}

void Speaker::registerMessage(QString name, QString text, int period, int expire, int priority, bool onChange, QString postprocessing) {
    struct Message msg = {name, text, period*1000, priority, expire, onChange, postprocessing};
    messages.append(msg);
    if(speech()) {
        bindMessage(msg);
    }
}

void Speaker::bindMessage(struct Message msg) {

    auto def = msg.name.split(":");
    auto msg_class = def[0];

    auto bid = PprzDispatcher::get()->bind(def[1], this,
        [=](QString sender, pprzlink::Message pprz_msg){

            QString ac_id;
            if(msg_class == "ground") {
                try {
                    pprz_msg.getField("ac_id", ac_id);
                }  catch (pprzlink::no_such_field& e) {
                    return;
                }
            } else if(msg_class == "telemetry") {
                ac_id = sender;
            }

            QString msg_id = msg.name+ac_id;


            auto now = QTime::currentTime();

            if(msg_times.contains(msg_id)) {
                if(msg_times[msg_id].addMSecs(msg.timeout) < now) {
                    handleMsg(msg, ac_id, pprz_msg);
                    msg_times[msg_id] = now;
                }
            } else {
                handleMsg(msg, ac_id, pprz_msg);
                msg_times[msg_id] = now;
            }

        });

    bids.append(bid);

}

void Speaker::handleMsg(struct Message msg, QString ac_id, pprzlink::Message pprz_msg) {
    auto def = msg.name.split(":");
    auto field = def[2];

    auto t = pprz_msg.getDefinition().getField(field).getType();
    auto bt = t.getBaseType();

    if(!t.isArray()) {
        if(bt == pprzlink::BaseType::FLOAT) {
            make_sentence<float>(msg, field, pprz_msg, ac_id);
        } else if(bt == pprzlink::BaseType::DOUBLE) {
            make_sentence<double>(msg, field, pprz_msg, ac_id);
        } else if(bt == pprzlink::BaseType::UINT8) {
            make_sentence<uint8_t>(msg, field, pprz_msg, ac_id);
        } else if(bt == pprzlink::BaseType::UINT16) {
            make_sentence<uint16_t>(msg, field, pprz_msg, ac_id);
        } else if(bt == pprzlink::BaseType::UINT32) {
            make_sentence<uint32_t>(msg, field, pprz_msg, ac_id);
        } else if(bt == pprzlink::BaseType::INT8) {
            make_sentence<int8_t>(msg, field, pprz_msg, ac_id);
        } else if(bt == pprzlink::BaseType::INT16) {
            make_sentence<int16_t>(msg, field, pprz_msg, ac_id);
        } else if(bt == pprzlink::BaseType::INT32) {
            make_sentence<int32_t>(msg, field, pprz_msg, ac_id);
        } else if(bt == pprzlink::BaseType::STRING) {
            make_sentence<QString>(msg, field, pprz_msg, ac_id);
        } else {
            qDebug() << "type not supported yet";
        }
    } else {
        qDebug() << "arrays not supported yet";
    }
}

template<typename T>
QString Speaker::value_to_string(T val) {
    return QString::number(val);
}

QString Speaker::value_to_string(double val) {
    return QString::number(val, 'f', 0);
}

QString Speaker::value_to_string(float val) {
    return QString::number(val, 'f', 0);
}

QString Speaker::value_to_string(QString val) {
    return  val;
}

QString Speaker::postprocess(QString process, QString valstr, QString ac_id) {
    if(process == "block") {
        auto fp = pprzApp()->toolbox()->aircraftManager()->getAircraft(ac_id)->getFlightPlan();
        auto b = fp->getBlock(valstr.toUInt());
        return b->getName();
    } else {
        qDebug() << "no postprocess " << process;
        return valstr;
    }
}

template<typename T>
void Speaker::make_sentence(struct Message msg, QString field, pprzlink::Message pprz_msg, QString ac_id) {
    T val;
    pprz_msg.getField(field, val);
    auto str_val = value_to_string(val);

    QString msg_id = msg.name+ac_id;

    if(msg.onChange && last_values.contains(msg_id) && last_values[msg_id] == str_val) {
        return;
    }

    if(msg.onChange) {
        last_values[msg_id] = str_val;
    }

    if(msg.postprocessing != "") {
        str_val = postprocess(msg.postprocessing, str_val, ac_id);
    }

    QString text = msg.text.replace("{value}", str_val);
    auto ac_name = pprzApp()->toolbox()->aircraftManager()->getAircraft(ac_id)->name();
    text = text.replace("{AC}", ac_name);
    addSentence(text, msg.priority, msg.expire);
}


void Speaker::setToolbox(PprzToolbox* toolbox) {
    PprzTool::setToolbox(toolbox);
    //connect(PprzDispatcher::get(), &PprzDispatcher::waypoint_moved, this, &AircraftManager::moveWaypoint);
}
