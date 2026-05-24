#include <QApplication>
#include <QCommandLineParser>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QCheckBox>
#include <QTimer>
#include <QIcon>
#include <QFile>
#include <QDebug>
#include <cmath>
#include <vector>

#include "../../ground_segment/linux_desktop_utils.h"
#include "pprzlinkQt/Message.h"
#include "pprzlinkQt/MessageDictionary.h"
#include "pprzlinkQt/IvyQtLink.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SENDING_PERIOD_MS 5000

class GaiaWindow : public QMainWindow {
    Q_OBJECT

public:
    GaiaWindow(const QString &ivyBus, double timeScale, double windSpeed, double windDir, double windUp, bool gpsOff, QWidget *parent = nullptr);
    ~GaiaWindow();

private slots:
    void sendWorldEnv();

private:
    void setupUI(double initTimeScale, double initWindSpeed, double initWindDir, double initWindUp, bool initGpsOff);
    void setupIvy(const QString &ivyBus);

    QDoubleSpinBox *m_spinTimeScale;
    QSlider *m_sliderWindDir;
    QLabel *m_lblWindDirVal;
    QSlider *m_sliderWindSpeed;
    QLabel *m_lblWindSpeedVal;
    QSlider *m_sliderWindUp;
    QLabel *m_lblWindUpVal;
    QCheckBox *m_chkGpsOff;
    QTimer *m_timer;

    pprzlink::MessageDictionary *m_dict;
    pprzlink::IvyQtLink *m_link;

    double m_irContrast = 266.0;
};

GaiaWindow::GaiaWindow(const QString &ivyBus, double timeScale, double windSpeed, double windDir, double windUp, bool gpsOff, QWidget *parent)
    : QMainWindow(parent), m_dict(nullptr), m_link(nullptr)
{
    setWindowTitle("Gaia");
    resize(300, 200);

    setupUI(timeScale, windSpeed, windDir, windUp, gpsOff);
    setupIvy(ivyBus);

    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &GaiaWindow::sendWorldEnv);
    m_timer->start(SENDING_PERIOD_MS);
}

GaiaWindow::~GaiaWindow()
{
    if (m_link) {
        m_link->stop();
        delete m_link;
    }
    if (m_dict) {
        delete m_dict;
    }
}

void GaiaWindow::setupUI(double initTimeScale, double initWindSpeed, double initWindDir, double initWindUp, bool initGpsOff)
{
    QWidget *central = new QWidget(this);
    QVBoxLayout *vbox = new QVBoxLayout(central);

    // Time scale
    QHBoxLayout *hboxTS = new QHBoxLayout();
    hboxTS->addWidget(new QLabel("Time scale:"));
    m_spinTimeScale = new QDoubleSpinBox();
    m_spinTimeScale->setDecimals(1);
    m_spinTimeScale->setRange(0.5, 10.0);
    m_spinTimeScale->setSingleStep(0.5);
    m_spinTimeScale->setValue(initTimeScale);
    connect(m_spinTimeScale, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &GaiaWindow::sendWorldEnv);
    hboxTS->addWidget(m_spinTimeScale);
    vbox->addLayout(hboxTS);

    // Wind direction
    QHBoxLayout *hboxWD = new QHBoxLayout();
    hboxWD->addWidget(new QLabel("Wind dir:"));
    m_sliderWindDir = new QSlider(Qt::Horizontal);
    m_sliderWindDir->setRange(0, 359);
    m_sliderWindDir->setSingleStep(1);
    m_sliderWindDir->setValue(static_cast<int>(initWindDir));
    m_lblWindDirVal = new QLabel(QString::number(initWindDir, 'f', 0));
    m_lblWindDirVal->setMinimumWidth(30);
    connect(m_sliderWindDir, &QSlider::valueChanged, this, [this](int value) {
        m_lblWindDirVal->setText(QString::number(value));
        sendWorldEnv();
    });
    hboxWD->addWidget(m_sliderWindDir);
    hboxWD->addWidget(m_lblWindDirVal);
    vbox->addLayout(hboxWD);

    // Wind speed (store *10 in slider to keep 0.1 precision)
    QHBoxLayout *hboxWS = new QHBoxLayout();
    hboxWS->addWidget(new QLabel("Wind speed:"));
    m_sliderWindSpeed = new QSlider(Qt::Horizontal);
    m_sliderWindSpeed->setRange(0, 300); // 0 to 30.0
    m_sliderWindSpeed->setSingleStep(1);
    m_sliderWindSpeed->setValue(static_cast<int>(initWindSpeed * 10));
    m_lblWindSpeedVal = new QLabel(QString::number(initWindSpeed, 'f', 1));
    m_lblWindSpeedVal->setMinimumWidth(30);
    connect(m_sliderWindSpeed, &QSlider::valueChanged, this, [this](int value) {
        m_lblWindSpeedVal->setText(QString::number(value / 10.0, 'f', 1));
        sendWorldEnv();
    });
    hboxWS->addWidget(m_sliderWindSpeed);
    hboxWS->addWidget(m_lblWindSpeedVal);
    vbox->addLayout(hboxWS);

    // Wind up (store *10 in slider for -10 to 10)
    QHBoxLayout *hboxWU = new QHBoxLayout();
    hboxWU->addWidget(new QLabel("Wind up:"));
    m_sliderWindUp = new QSlider(Qt::Horizontal);
    m_sliderWindUp->setRange(-100, 100); // -10.0 to 10.0
    m_sliderWindUp->setSingleStep(1);
    m_sliderWindUp->setValue(static_cast<int>(initWindUp * 10));
    m_lblWindUpVal = new QLabel(QString::number(initWindUp, 'f', 1));
    m_lblWindUpVal->setMinimumWidth(30);
    connect(m_sliderWindUp, &QSlider::valueChanged, this, [this](int value) {
        m_lblWindUpVal->setText(QString::number(value / 10.0, 'f', 1));
        sendWorldEnv();
    });
    hboxWU->addWidget(m_sliderWindUp);
    hboxWU->addWidget(m_lblWindUpVal);
    vbox->addLayout(hboxWU);

    // GPS availability
    m_chkGpsOff = new QCheckBox("GPS OFF");
    m_chkGpsOff->setChecked(initGpsOff);
    connect(m_chkGpsOff, &QCheckBox::toggled, this, &GaiaWindow::sendWorldEnv);
    vbox->addWidget(m_chkGpsOff);

    setCentralWidget(central);
}

void GaiaWindow::setupIvy(const QString &ivyBus)
{
    QString phome = qgetenv("PAPARAZZI_HOME");
    if (phome.isEmpty()) phome = QString("/home/%1/paparazzi").arg(qgetenv("USER"));
    QString xmlPath = phome + "/var/messages.xml";

    if (!QFile::exists(xmlPath)) {
        qWarning() << "Gaia: message dictionary not found at" << xmlPath << ". Ivy telemetry disabled.";
        return;
    }

    try {
        m_dict = new pprzlink::MessageDictionary(xmlPath);
        m_link = new pprzlink::IvyQtLink(*m_dict, "gaia", this);
        m_link->start(ivyBus);

        // Listen for WORLD_ENV_REQ and respond. This mirrors `Ground_Pprz.message_answerer my_id "WORLD_ENV"`
        std::vector<pprzlink::MessageDefinition> defs = m_dict->getMsgsForClass("ground");
        for (const auto& def : defs) {
            if (def.getName() == "WORLD_ENV_REQ") {
                m_link->BindMessage(def, this, [this](QString sender, pprzlink::Message msg) {
                    Q_UNUSED(sender);
                    Q_UNUSED(msg);
                    this->sendWorldEnv();
                });
                break;
            }
        }
    } catch (const std::exception &e) {
        qWarning() << "Gaia: Exception during Ivy init:" << e.what();
    }
}

void GaiaWindow::sendWorldEnv()
{
    if (!m_link || !m_dict) return;

    try {
        auto def = m_dict->getDefinition("WORLD_ENV");
        pprzlink::Message msg(def);
        msg.setSenderId("gaia");

        double windSpeed = m_sliderWindSpeed->value() / 10.0;
        double windDirDeg = m_sliderWindDir->value();
        double windDirRad = M_PI / 2.0 - (windDirDeg * M_PI / 180.0);

        double windEast = -windSpeed * cos(windDirRad);
        double windNorth = -windSpeed * sin(windDirRad);
        double windUp = m_sliderWindUp->value() / 10.0;

        uint8_t gpsAvail = m_chkGpsOff->isChecked() ? 0 : 1;

        msg.addField("wind_east", static_cast<float>(windEast));
        msg.addField("wind_north", static_cast<float>(windNorth));
        msg.addField("wind_up", static_cast<float>(windUp));
        msg.addField("ir_contrast", static_cast<float>(m_irContrast));
        msg.addField("time_scale", static_cast<float>(m_spinTimeScale->value()));
        msg.addField("gps_availability", gpsAvail);

        m_link->sendMessage(msg);

    } catch (const std::exception &e) {
        qWarning() << "Gaia: failed to send WORLD_ENV:" << e.what();
    }
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setApplicationVersion("1.0");
    app.setDesktopFileName(QStringLiteral("paparazzi_gaia"));
    app.setApplicationName(QStringLiteral("Gaia"));

    QString iconPath = ":/penguin_icon_sim.png";
    QIcon icon(iconPath);
    installLinuxDesktopIntegration(app.desktopFileName(), "Paparazzi Gaia", "World environment simulator", iconPath, "paparazzi-gaia");
    app.setWindowIcon(icon);

    QCommandLineParser parser;
    parser.setApplicationDescription("Paparazzi Gaia simulator component");
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption busOption(QStringList() << "b", "Ivy Bus (Default: 127.255.255.255:2010)", "bus", "127.255.255.255:2010");
    parser.addOption(busOption);
    
    QCommandLineOption timeOption(QStringList() << "t", "Set time scale (default: 1.0)", "timeScale", "1.0");
    parser.addOption(timeOption);
    
    QCommandLineOption windSpeedOption(QStringList() << "w", "Set wind speed (0-30m/s)", "windSpeed", "0.0");
    parser.addOption(windSpeedOption);
    
    QCommandLineOption windDirOption(QStringList() << "d", "Set wind direction 0-359 deg", "windDir", "0.0");
    parser.addOption(windDirOption);
    
    QCommandLineOption windUpOption(QStringList() << "u", "Set wind updraft (-10 to 10m/s)", "windUp", "0.0");
    parser.addOption(windUpOption);
    
    QCommandLineOption gpsOffOption(QStringList() << "g", "Turn off GPS");
    parser.addOption(gpsOffOption);

    parser.process(app);

    QString ivyBus = parser.value(busOption);
    double timeScale = parser.value(timeOption).toDouble();
    double windSpeed = parser.value(windSpeedOption).toDouble();
    double windDir = parser.value(windDirOption).toDouble();
    double windUp = parser.value(windUpOption).toDouble();
    bool gpsOff = parser.isSet(gpsOffOption);

    GaiaWindow window(ivyBus, timeScale, windSpeed, windDir, windUp, gpsOff);
    window.show();

    return app.exec();
}

#include "gaia.moc"
