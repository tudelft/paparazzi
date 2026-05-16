#ifndef SPEAKER_H
#define SPEAKER_H

#include <QObject>

#include "PprzToolbox.h"
#include "PprzApplication.h"
#include "pprz_dispatcher.h"



class Speaker : public PprzTool
{
    Q_OBJECT
public:
    explicit Speaker(PprzApplication* app, PprzToolbox* toolbox);
    void setToolbox(PprzToolbox* toolbox) override;
    void addSentence(QString txt, int priority=0, int validity=30);
    void registerMessage(QString name, QString text, int period, int expire, int priority, bool onChange, QString postprocessing);
    void setLocale(QString l);
signals:

public slots:
    void enableSpeech(bool s, bool quiet = false);

private:

    struct Sentence {
        QString text;
        int priority;
        QTime expiration;
    };

    struct Message {
        QString name;
        QString text;
        int timeout;
        int expire;
        int priority;
        bool onChange;
        QString postprocessing;
    };


    void bindMessage(struct Message msg);
    void handleMsg(struct Message msg, QString sender, pprzlink::Message pprz_msg);

    template<typename T>
    QString value_to_string(T val);

    QString value_to_string(double val);
    QString value_to_string(float val);
    QString value_to_string(QString val);

    QString postprocess(QString process, QString valstr, QString ac_id);

    template<typename T>
    void make_sentence(struct Message msg, QString field, pprzlink::Message pprz_msg, QString ac_id);

    void say();

    
    QString locale;

    QList<Sentence> sentences;
    QList<Message> messages;
    QList<long> bids;
    QMap<QString, QTime> msg_times;
    QMap<QString, QString> last_values;

    QTimer* timer;
    class QProcess* speechProcess;
    void onSpeechFinished(int exitCode);

};

#endif // SPEAKER_H
