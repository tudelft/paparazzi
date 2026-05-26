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
#include <QDial>
#include <QFrame>
#include <QIcon>
#include <QFile>
//#include <QDebug>
#include <QPainter>
#include <QPolygon>
#include <QSpinBox>
#include <QGridLayout>
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

class ArrowDial : public QDial {
    Q_OBJECT
public:
    explicit ArrowDial(QWidget* parent = nullptr) : QDial(parent) {
        setMinimumSize(100, 100);
        setWrapping(true);
        setNotchesVisible(true);
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        
        painter.translate(width() / 2.0, height() / 2.0);
        double side = qMin(width(), height());
        painter.scale(side / 200.0, side / 200.0);
        
        QColor edgeColor(255, 255, 255, 255);
        QColor fillColor(0, 0, 0, 255);
        
        QPen pen = painter.pen();
        pen.setWidth(1);
        pen.setColor(edgeColor);
        painter.setPen(pen);
        painter.setBrush(fillColor);
        
        painter.setRenderHint(QPainter::Antialiasing);
        
        painter.save();
        
        double minimum = this->minimum();
        double maximum = this->maximum();
        double val = this->value();
        double percent = (val - minimum) / (maximum - minimum);
        double angle = 360.0 * percent;
        
        painter.rotate(angle);
        
        QPolygon polygon;
        polygon << QPoint(0, 80)
                << QPoint(8, -80)
                << QPoint(0, -75)
                << QPoint(-8, -80);
        
        painter.drawPolygon(polygon, Qt::OddEvenFill);
        painter.restore();
        
        if (notchesVisible()) {
            painter.save();
            for (int i = 0; i < 4; i++) {
                painter.drawRect(-2, 85, 4, 13);
                painter.rotate(90);
            }
            painter.restore();
            
            painter.save();
            painter.rotate(45);
            for (int i = 0; i < 4; i++) {
                painter.drawRect(-1, 90, 2, 8);
                painter.rotate(90);
            }
            painter.restore();
            
            painter.save();
            for (int i = 0; i < 360; i += 10) {
                if (i % 90 != 0) {
                    painter.drawRect(-1, 94, 2, 4);
                }
                painter.rotate(10);
            }
            painter.restore();
        }
    }
};

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

    QSlider *m_sliderTimeScale;
    QDoubleSpinBox *m_spinTimeScaleVal;
    ArrowDial *m_dialWindDir;
    QSlider *m_sliderWindSpeed;
    QDoubleSpinBox *m_spinWindSpeedVal;
    QSlider *m_sliderWindUp;
    QDoubleSpinBox *m_spinWindUpVal;
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
    resize(500, 250);

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
    QHBoxLayout *mainLayout = new QHBoxLayout(central);

    // Left side: sliders
    QFrame *slidersFrame = new QFrame();
    slidersFrame->setFrameShape(QFrame::StyledPanel);
    QVBoxLayout *slidersLayout = new QVBoxLayout(slidersFrame);

    auto createSliderBlock = [this, slidersLayout](const QString& text, double min, double max, double step, int decimals, double current, QSlider*& slider, QDoubleSpinBox*& spinBox) {
        QHBoxLayout* textLayout = new QHBoxLayout();
        QLabel* titleLabel = new QLabel(text);
        spinBox = new QDoubleSpinBox();
        spinBox->setDecimals(decimals);
        spinBox->setRange(min, max);
        spinBox->setSingleStep(step);
        spinBox->setValue(current);
        //spinBox->setButtonSymbols(QAbstractSpinBox::NoButtons); //Whatever you fancy
        spinBox->setMinimumWidth(40);
        spinBox->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        // By default spinboxes show up/down arrows
        textLayout->addWidget(titleLabel);
        textLayout->addWidget(spinBox);
        
        slider = new QSlider(Qt::Horizontal);
        int factor = pow(10, decimals);
        slider->setRange(min * factor, max * factor);
        slider->setSingleStep(step * factor);
        slider->setValue(current * factor);
        
        slidersLayout->addLayout(textLayout);
        slidersLayout->addWidget(slider);
        
        QFrame* hline = new QFrame();
        hline->setFrameShape(QFrame::HLine);
        hline->setFrameShadow(QFrame::Plain);
        slidersLayout->addWidget(hline);
        
        // Connect slider and spinbox
        connect(slider, &QSlider::valueChanged, this, [spinBox, factor](int value) {
            spinBox->setValue(value / static_cast<double>(factor));
        });
        connect(spinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [slider, factor](double value) {
            slider->setValue(value * factor);
        });
    };

    // Time scale
    createSliderBlock("Time scale", 0.1, 10.0, 0.1, 1, initTimeScale, m_sliderTimeScale, m_spinTimeScaleVal);
    connect(m_sliderTimeScale, &QSlider::valueChanged, this, [this](int) { sendWorldEnv(); });

    // Wind speed
    createSliderBlock("Wind speed (m/s)", 0.0, 30.0, 0.1, 1, initWindSpeed, m_sliderWindSpeed, m_spinWindSpeedVal);
    connect(m_sliderWindSpeed, &QSlider::valueChanged, this, [this](int) { sendWorldEnv(); });

    // Wind up
    createSliderBlock("Vertical up/down draft (m/s)", -10.0, 10.0, 0.1, 1, initWindUp, m_sliderWindUp, m_spinWindUpVal);
    connect(m_sliderWindUp, &QSlider::valueChanged, this, [this](int) { sendWorldEnv(); });

    // GPS availability
    m_chkGpsOff = new QCheckBox("Emulate GPS signal unavailable");
    m_chkGpsOff->setChecked(initGpsOff);
    connect(m_chkGpsOff, &QCheckBox::toggled, this, &GaiaWindow::sendWorldEnv);
    slidersLayout->addWidget(m_chkGpsOff, 0, Qt::AlignHCenter);

    mainLayout->addWidget(slidersFrame);

    // Right side: Dial (Angle Selector)
    QWidget *angleSelectorWidget = new QWidget();
    QVBoxLayout *angleSelectorLayout = new QVBoxLayout(angleSelectorWidget);
    
    QGridLayout *gridLayout = new QGridLayout();
    
    QLabel *lbl0 = new QLabel("0");
    gridLayout->addWidget(lbl0, 0, 1, 1, 1, Qt::AlignHCenter | Qt::AlignBottom);
    
    QLabel *lbl90 = new QLabel("90");
    gridLayout->addWidget(lbl90, 1, 2, 1, 1, Qt::AlignLeft | Qt::AlignVCenter);
    
    QLabel *lbl270 = new QLabel("270");
    gridLayout->addWidget(lbl270, 1, 0, 1, 1, Qt::AlignRight | Qt::AlignVCenter);
    
    QLabel *lbl180 = new QLabel("180");
    gridLayout->addWidget(lbl180, 2, 1, 1, 1, Qt::AlignHCenter | Qt::AlignTop);

    m_dialWindDir = new ArrowDial();
    m_dialWindDir->setRange(0, 359);
    m_dialWindDir->setValue(static_cast<int>(initWindDir));
    QSizePolicy sp(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    sp.setHorizontalStretch(0);
    sp.setVerticalStretch(0);
    m_dialWindDir->setSizePolicy(sp);
    
    gridLayout->addWidget(m_dialWindDir, 1, 1, 1, 1);
    angleSelectorLayout->addLayout(gridLayout);
    
    QHBoxLayout *spinBoxLayout = new QHBoxLayout();
    QLabel *spinBoxLabel = new QLabel("Wind direction (°)");
    spinBoxLayout->addWidget(spinBoxLabel);
    
    QSpinBox *spinBox = new QSpinBox();
    spinBox->setWrapping(true);
    spinBox->setRange(0, 359);
    spinBox->setValue(static_cast<int>(initWindDir));
    spinBoxLayout->addWidget(spinBox);
    
    angleSelectorLayout->addLayout(spinBoxLayout);
    
    connect(m_dialWindDir, &QDial::valueChanged, spinBox, &QSpinBox::setValue);
    connect(spinBox, &QSpinBox::valueChanged, m_dialWindDir, &QDial::setValue);
    connect(m_dialWindDir, &QDial::valueChanged, this, &GaiaWindow::sendWorldEnv);
    
    mainLayout->addWidget(angleSelectorWidget);

    //resize(500, 300);
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
        double windDirDeg = m_dialWindDir->value();
        double windDirRad = windDirDeg * M_PI / 180.0;

        double windEast = -windSpeed * sin(windDirRad);
        double windNorth = -windSpeed * cos(windDirRad);
        double windUp = m_sliderWindUp->value() / 10.0;

        uint8_t gpsAvail = m_chkGpsOff->isChecked() ? 0 : 1;

        msg.addField("wind_east", static_cast<float>(windEast));
        msg.addField("wind_north", static_cast<float>(windNorth));
        msg.addField("wind_up", static_cast<float>(windUp));
        msg.addField("ir_contrast", static_cast<float>(m_irContrast));
        msg.addField("time_scale", static_cast<float>(m_sliderTimeScale->value() / 10.0));
        msg.addField("gps_availability", gpsAvail);

        m_link->sendMessage(msg);

    } catch (const std::exception &e) {
        qWarning() << "Gaia: failed to send WORLD_ENV:" << e.what();
    }
}

int main(int argc, char *argv[])
{
    // Set metadata BEFORE application instantiation to prevent XDG portal double-registration 
    // root cause ("Connection already associated with an application ID").
    QCoreApplication::setApplicationVersion("1.0");
    QGuiApplication::setDesktopFileName(QStringLiteral("paparazzi_gaia"));
    QCoreApplication::setApplicationName(QStringLiteral("Gaia"));

    QApplication app(argc, argv);

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
    
    QCommandLineOption windUpOption(QStringList() << "u", "Set wind vertical draft (-10 to 10m/s)", "windUp", "0.0");
    parser.addOption(windUpOption);
    
    QCommandLineOption gpsOffOption(QStringList() << "g", "Emulate GPS signal unavailable (default: false)");
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
