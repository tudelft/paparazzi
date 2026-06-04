/**
 * @file plotter.cpp
 * @brief Real-time telemetry plotter application for Paparazzi UAV.
 * @details This application connects to the Paparazzi Ivy bus, parses telemetry 
 *          messages dynamically, and visualizes the data via Qt Charts in real-time. 
 *          It is designed to be highly robust and memory-safe for long-running operations.
 */

#include <QApplication>
#include <QChartView>
#include <QCheckBox>
#include <QGraphicsLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QLineSeries>
#include <QMainWindow>
#include <QMenuBar>
#include <QSlider>
#include <QSpinBox>
#include <QValueAxis>
#include <QProxyStyle>
#include <QPlainTextEdit>
#include <QInputDialog>
#include <QRegularExpression>
#include <mutex>

#include "../linux_desktop_utils.h"
#include "pprzlinkQt/IvyQtLink.h"

/**
 * @struct PlotConfig
 * @brief Configuration and runtime state for an actively plotted curve.
 * @details This structure bundles the telemetry metadata (sender, message name, field) 
 *          with the Qt rendering components (QLineSeries, action menus) and an internal 
 *          buffer. Buffering points incoming rapidly prevents excessive redraw calls 
 *          on the QChart side, allowing high update rates gracefully.
 */
struct PlotConfig {
    QString senderName;
    QString className;
    QString msgName;
    QString fieldName;
    double coef;
    QLineSeries* series;
    QList<QPointF> buffer;
    int fieldIndex = -1;
    bool discrete = false;
    QAction* avgAction = nullptr;
    QAction* stdevAction = nullptr;
    int arrayIndex = -1;
    QLabel* legendLabel = nullptr;
};

/**
 * @class PlotterWindow
 * @brief The main GUI manager encompassing the plotting canvas, tools, and Ivy messaging.
 * @details This class is responsible for spawning the main window layout, registering 
 *          drag-and-drop operations for curve associations, processing parsed variables 
 *          into chart instances, and maintaining boundary rules (auto-scaling, min/max limits).
 */
class PlotterWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit PlotterWindow(QWidget *parent = nullptr);
    ~PlotterWindow();

protected:
    void resizeEvent(QResizeEvent* event) override;
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dropEvent(QDropEvent *event) override;

private slots:
    void onClearClicked();
    void onPauseToggled(bool checked);
    void onAutoScaleToggled(bool checked);
    void onManualScaleChanged();
    void onAddConstantClicked();
    void onUpdateRateChanged(int val);
    void onLineThicknessChanged(int val);
    void updatePlots();

private:
    QWidget* m_legendOverlay;
    QVBoxLayout* m_legendLayout;
    void updateLegendValues();
    void updateLegendPosition();

    void setupIvy();
    void setupUI();
    void setupMenu();
    void addPlotFromPayload(const QString& payload);
    void handleMessage(QString sender, const pprzlink::Message& msg);
    void onLegendRefreshTimeout();
    
    void addCurveToMenu(PlotConfig& config);
    void removeCurve(QLineSeries* series);
    void recalculateYBounds();
    QColor getNextSaturatedColor();
    
    QChart *m_chart;
    QValueAxis *m_axisX;
    QValueAxis *m_axisY;
    qint64 m_startTime;
    
    pprzlink::MessageDictionary* m_dict;
    pprzlink::IvyQtLink* m_link;
    
    QList<PlotConfig> m_activePlots;
    
    int m_colorIndex = 0;
    double m_minY;
    double m_maxY;
    bool m_paused;
    bool m_autoScale;
    
    QCheckBox* m_cbAutoScale;
    QLineEdit* m_edtMinY;
    QLineEdit* m_edtMaxY;
    QSlider* m_slTimeWindow;
    QLineEdit* m_edtConstant;
    QSlider* m_slUpdateRate;
    QLineEdit* m_edtScaleNext;
    QSpinBox* m_spnLineThickness;
    QTimer* m_updateTimer;
    QMenu* m_curvesMenu;
    QTimer* m_legendUpdateTimer;
    bool m_legendNeedsRefresh;
    std::recursive_mutex m_plotMutex;
};

/**
 * @brief Safely extracts a native double value from a generic pprzlink FieldValue.
 * @param value The strongly-typed variant value transmitted over the Ivy bus.
 * @return The converted 64-bit float, or a quiet NaN if the layout is an array or unparseable string.
 * @details Ivy telemetry packages data in multiple raw binary types. This helper cleanly cascades 
 *          downward through the supported type taxonomy to normalize inputs onto a plotting-friendly 
 *          1D numerical axis.
 */
static double fieldValueAsDouble(const pprzlink::FieldValue &value, int arrayIndex = -1)
{
    const auto &type = value.getType();
    if (type.isArray()) {
        if (arrayIndex >= 0) {
            try {
                switch (type.getBaseType()) {
                    case pprzlink::BaseType::CHAR: { std::vector<char> v; value.getValue(v); if (arrayIndex < v.size()) return static_cast<double>(v[arrayIndex]); } break;
                    case pprzlink::BaseType::INT8: { std::vector<int8_t> v; value.getValue(v); if (arrayIndex < v.size()) return static_cast<double>(v[arrayIndex]); } break;
                    case pprzlink::BaseType::INT16: { std::vector<int16_t> v; value.getValue(v); if (arrayIndex < v.size()) return static_cast<double>(v[arrayIndex]); } break;
                    case pprzlink::BaseType::INT32: { std::vector<int32_t> v; value.getValue(v); if (arrayIndex < v.size()) return static_cast<double>(v[arrayIndex]); } break;
                    case pprzlink::BaseType::UINT8: { std::vector<uint8_t> v; value.getValue(v); if (arrayIndex < v.size()) return static_cast<double>(v[arrayIndex]); } break;
                    case pprzlink::BaseType::UINT16: { std::vector<uint16_t> v; value.getValue(v); if (arrayIndex < v.size()) return static_cast<double>(v[arrayIndex]); } break;
                    case pprzlink::BaseType::UINT32: { std::vector<uint32_t> v; value.getValue(v); if (arrayIndex < v.size()) return static_cast<double>(v[arrayIndex]); } break;
                    case pprzlink::BaseType::FLOAT: { std::vector<float> v; value.getValue(v); if (arrayIndex < v.size()) return static_cast<double>(v[arrayIndex]); } break;
                    case pprzlink::BaseType::DOUBLE: { std::vector<double> v; value.getValue(v); if (arrayIndex < v.size()) return static_cast<double>(v[arrayIndex]); } break;
                    default: return std::numeric_limits<double>::quiet_NaN();
                }
            } catch (...) {
            }
        }
        return std::numeric_limits<double>::quiet_NaN();
    }
    switch (type.getBaseType()) {
    case pprzlink::BaseType::CHAR: {
        char v;
        value.getValue(v);
        return static_cast<double>(v);
    }
    case pprzlink::BaseType::INT8: {
        int8_t v;
        value.getValue(v);
        return static_cast<double>(v);
    }
    case pprzlink::BaseType::INT16: {
        int16_t v;
        value.getValue(v);
        return static_cast<double>(v);
    }
    case pprzlink::BaseType::INT32: {
        int32_t v;
        value.getValue(v);
        return static_cast<double>(v);
    }
    case pprzlink::BaseType::UINT8: {
        uint8_t v;
        value.getValue(v);
        return static_cast<double>(v);
    }
    case pprzlink::BaseType::UINT16: {
        uint16_t v;
        value.getValue(v);
        return static_cast<double>(v);
    }
    case pprzlink::BaseType::UINT32: {
        uint32_t v;
        value.getValue(v);
        return static_cast<double>(v);
    }
    case pprzlink::BaseType::FLOAT: {
        float v;
        value.getValue(v);
        return static_cast<double>(v);
    }
    case pprzlink::BaseType::DOUBLE: {
        double v;
        value.getValue(v);
        return v;
    }
    case pprzlink::BaseType::STRING: {
        QString s;
        value.getValue(s);
        bool ok = false;
        double d = s.toDouble(&ok);
        return ok ? d : std::numeric_limits<double>::quiet_NaN();
    }
    default:
        return std::numeric_limits<double>::quiet_NaN();
    }
}

/**
 * @brief Retrieves the next uniformly-distributed vibrant color for a curve.
 * @return A unique QColor guaranteed to remain legible against standard IDE themes.
 * @details By traversing the Hue spectrum based on the golden angle (approx. 137.5 degrees), 
 *          we mathematically guarantee maximum perceptual spacing between sequentially 
 *          spawned line colors.
 */
QColor PlotterWindow::getNextSaturatedColor() {
    double h = std::fmod(m_colorIndex * 137.508, 360.0);
    m_colorIndex++;
    // Hue varies, Saturation = 1.0 (no white/gray, min channel is 0), Value = 1.0 (no dark colors, max channel is 255)
    return QColor::fromHsvF(h / 360.0, 1.0, 1.0);
}

/**
 * @brief Constructs the Plotter Window and instantiates all visual layouts.
 * @param parent Optional parent widget (usually null for root windows).
 * @details Establishes zero-margin frameless QChart setups, registers memory 
 *          handling attributes like `WA_DeleteOnClose` to prevent leaks upon 
 *          user dismissal, and wires up the UI actions.
 */
PlotterWindow::PlotterWindow(QWidget *parent) : QMainWindow(parent), m_minY(std::numeric_limits<double>::infinity()), m_maxY(-std::numeric_limits<double>::infinity()), m_paused(false), m_autoScale(true), m_legendNeedsRefresh(false) {
    m_legendOverlay = nullptr;
    m_legendLayout = nullptr;
    setAcceptDrops(true);
    setAttribute(Qt::WA_DeleteOnClose);
    setWindowTitle("Plotter");
    resize(300, 400);

    m_chart = new QChart();
    m_chart->setTitle("Drag & Drop messages here");
    m_chart->legend()->setVisible(true);
    m_chart->legend()->setAlignment(Qt::AlignRight);
    m_chart->legend()->detachFromChart();
    m_chart->legend()->setBackgroundVisible(true);
    m_chart->setAnimationOptions(QChart::NoAnimation);
    m_chart->setBackgroundRoundness(0);
    m_chart->setMargins(QMargins(0, 0, 0, 0));
    m_chart->layout()->setContentsMargins(0, 0, 0, 0);
    m_chart->setBackgroundPen(QPen(Qt::NoPen));

    m_axisX = new QValueAxis();
    m_axisX->setTitleText("Time (s)");
    m_axisX->setTitleVisible(false); // Hidden by default
    // m_axisX->setLabelFormat("%gs"); previous if you want to
    m_axisX->setLabelFormat("%.1fs");
    m_chart->addAxis(m_axisX, Qt::AlignBottom);

    m_axisY = new QValueAxis();
    m_axisY->setTitleText("Value");
    m_axisY->setTitleVisible(false); // Hidden by default
    m_chart->addAxis(m_axisY, Qt::AlignLeft);

    setupUI();
    setupMenu();

    m_startTime = QDateTime::currentMSecsSinceEpoch();

    setupIvy();
}

/**
 * @brief Destructor. Guarantees clean teardown of network handlers.
 * @details Ivy loops typically spawn background worker threads; explicitly 
 *          calling `m_link->stop()` prevents segmentation faults upon app exit.
 */
PlotterWindow::~PlotterWindow() {
    if (m_link) {
        m_link->stop();
        m_link = nullptr;
    }
    if (m_dict) {
        delete m_dict;
        m_dict = nullptr;
    }
}

/**
 * @brief Bootstraps the local Paparazzi network hooks for telemetry binding.
 * @details Finds the system configuration directory referencing `messages.xml` 
 *          to understand incoming binary protocol shapes cleanly at runtime.
 */
void PlotterWindow::setupIvy() {
    QString phome = qgetenv("PAPARAZZI_HOME");
    if (phome.isEmpty()) phome = QString("/home/%1/paparazzi").arg(qgetenv("USER"));
    QString xmlPath = phome + "/var/messages.xml";

    if (!QFile::exists(xmlPath)) {
        qWarning() << "Plotter: message dictionary not found at" << xmlPath << ". Ivy telemetry will be disabled.";
        m_dict = nullptr;
        m_link = nullptr;
        return;
    }

    try {
        m_dict = new pprzlink::MessageDictionary(xmlPath);
        m_link = new pprzlink::IvyQtLink(*m_dict, "plotter", this);
        m_link->start("127.255.255.255:2010");
    } catch (const std::exception &e) {
        qWarning() << "Plotter: failed to initialize Ivy link or message dictionary:" << e.what();
        delete m_link;
        m_link = nullptr;
        delete m_dict;
        m_dict = nullptr;
    } catch (...) {
        qWarning() << "Plotter: unknown failure during Ivy initialization.";
        delete m_link;
        m_link = nullptr;
        delete m_dict;
        m_dict = nullptr;
    }
}

/**
 * @brief Constructs the application's widgets, layouts, menus, and timers dynamically.
 * @details This separates graphical state binding away from pure telemetry handling.
 *          Timers are generated here regulating UI FPS (default ~60Hz base).
 */
void PlotterWindow::setupUI() {
    QWidget *mainWidget = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(mainWidget);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);

    QWidget *toolbarWidget = new QWidget();
    QHBoxLayout *toolbarLayout = new QHBoxLayout(toolbarWidget);
    toolbarLayout->setContentsMargins(2, 2, 2, 2); // keep some margin for the toolbar

    m_cbAutoScale = new QCheckBox("Auto Scale");
    m_cbAutoScale->setChecked(true);

    m_edtMinY = new QLineEdit();
    m_edtMaxY = new QLineEdit();
    m_edtMinY->setMaximumWidth(60);
    m_edtMaxY->setMaximumWidth(60);
    m_edtMinY->setEnabled(false);
    m_edtMaxY->setEnabled(false);

    m_slTimeWindow = new QSlider(Qt::Horizontal);
    m_slTimeWindow->setToolTip("Time Window (s)");
    m_slTimeWindow->setRange(5, 1000); // 5 to 1000 (0.05s to 10.0s)
    m_slTimeWindow->setValue(1000); // Default to 10s


    QLabel *lblConst = new QLabel("Constant");
    m_edtConstant = new QLineEdit();
    m_edtConstant->setMaximumWidth(50);

    QLabel *lblScaleNext = new QLabel("Scale next by");
    m_edtScaleNext = new QLineEdit("1.0");
    m_edtScaleNext->setMaximumWidth(50);
    m_edtScaleNext->setToolTip("Scale next curve (e.g. 0.0174 to convert deg in rad, 57.3 to convert rad in deg)");

    m_slUpdateRate = new QSlider(Qt::Horizontal);
    m_slUpdateRate->setToolTip("Update Rate (ms)");
    m_slUpdateRate->setRange(10, 1000); // 10ms to 1000ms
    m_slUpdateRate->setValue(16); // Default to 16ms (~60Hz)

    m_spnLineThickness = new QSpinBox();
    m_spnLineThickness->setToolTip("Line Thickness (px)");
    m_spnLineThickness->setRange(1, 10);
    m_spnLineThickness->setValue(1);
    m_spnLineThickness->hide();

    m_updateTimer = new QTimer(this);
    m_updateTimer->start(m_slUpdateRate->value());

    m_legendUpdateTimer = new QTimer(this);
    m_legendUpdateTimer->setInterval(200);
    connect(m_legendUpdateTimer, &QTimer::timeout, this, &PlotterWindow::onLegendRefreshTimeout);
    m_legendUpdateTimer->start();

    toolbarLayout->addWidget(m_cbAutoScale);
    toolbarLayout->addWidget(new QLabel("Min"));
    toolbarLayout->addWidget(m_edtMinY);
    toolbarLayout->addWidget(new QLabel("Max"));
    toolbarLayout->addWidget(m_edtMaxY);

    QLabel *lblTimeWindowVal = new QLabel(QString("%1").arg(m_slTimeWindow->value() / 100.0, 0, 'f', 2));
    lblTimeWindowVal->setAlignment(Qt::AlignCenter);
    QVBoxLayout *vboxTime = new QVBoxLayout();
    vboxTime->addWidget(lblTimeWindowVal);
    vboxTime->addWidget(m_slTimeWindow);
    vboxTime->setContentsMargins(0, 0, 0, 0);
    QWidget *wTime = new QWidget();
    wTime->setLayout(vboxTime);

    QLabel *lblUpdateRateVal = new QLabel(QString("%1").arg(m_slUpdateRate->value()));
    lblUpdateRateVal->setAlignment(Qt::AlignCenter);
    QVBoxLayout *vboxRate = new QVBoxLayout();
    vboxRate->addWidget(lblUpdateRateVal);
    vboxRate->addWidget(m_slUpdateRate);
    vboxRate->setContentsMargins(0, 0, 0, 0);
    QWidget *wRate = new QWidget();
    wRate->setLayout(vboxRate);

    connect(m_slTimeWindow, &QSlider::valueChanged, lblTimeWindowVal, [lblTimeWindowVal](int val) {
        lblTimeWindowVal->setText(QString("%1").arg(val / 100.0, 0, 'f', 2));
    });
    connect(m_slUpdateRate, &QSlider::valueChanged, lblUpdateRateVal, [lblUpdateRateVal](int val) {
        lblUpdateRateVal->setText(QString("%1").arg(val));
    });

    toolbarLayout->addWidget(wTime, 1);
    toolbarLayout->addWidget(wRate, 1);
    toolbarLayout->addWidget(lblConst);
    toolbarLayout->addWidget(m_edtConstant);
    toolbarLayout->addWidget(lblScaleNext);
    toolbarLayout->addWidget(m_edtScaleNext);
    QLabel* lblLineThickness = new QLabel("Line:");
    lblLineThickness->hide();
    toolbarLayout->addWidget(lblLineThickness);
    toolbarLayout->addWidget(m_spnLineThickness);


    QChartView *chartView = new QChartView(m_chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setAcceptDrops(false); // Let drops fall through
    chartView->setContentsMargins(0, 0, 0, 0);
    chartView->setFrameShape(QFrame::NoFrame);

    mainLayout->addWidget(toolbarWidget);
    mainLayout->addWidget(chartView);
    setCentralWidget(mainWidget);

    m_legendOverlay = new QWidget(chartView);
    m_legendLayout = new QVBoxLayout(m_legendOverlay);
    m_legendLayout->setContentsMargins(0, 0, 0, 0);
    m_legendLayout->setSpacing(0);

    connect(m_cbAutoScale, &QCheckBox::toggled, this, &PlotterWindow::onAutoScaleToggled);
    connect(m_edtMinY, &QLineEdit::editingFinished, this, &PlotterWindow::onManualScaleChanged);
    connect(m_edtMaxY, &QLineEdit::editingFinished, this, &PlotterWindow::onManualScaleChanged);
    connect(m_edtConstant, &QLineEdit::editingFinished, this, &PlotterWindow::onAddConstantClicked);
    connect(m_slUpdateRate, &QSlider::valueChanged, this, &PlotterWindow::onUpdateRateChanged);
    connect(m_spnLineThickness, QOverload<int>::of(&QSpinBox::valueChanged), this, &PlotterWindow::onLineThicknessChanged);
    connect(m_updateTimer, &QTimer::timeout, this, &PlotterWindow::updatePlots);
}

/**
 * @brief Periodically syncs statistical computations directly to UI display labels.
 * @details Re-walking plot arrays repeatedly is intense; batching them via a gentle 
 *          0.2s timer enables deep math analysis (StdDev, Average) without choking 
 *          the fast-rendering path.
 */
void PlotterWindow::onLegendRefreshTimeout()
{
    if (!m_legendNeedsRefresh) {
        return;
    }
    m_legendNeedsRefresh = false;
    updateLegendValues();

    std::lock_guard<std::recursive_mutex> lock(m_plotMutex);
    // Compute average and standard deviation for each curve
    for (const auto& plot : m_activePlots) {
        if (!plot.series || (!plot.avgAction && !plot.stdevAction)) continue;
        int n = plot.series->count();
        if (n < 1) {
            if (plot.avgAction) plot.avgAction->setText(tr("Average: N/A"));
            if (plot.stdevAction) plot.stdevAction->setText(tr("Stdev: N/A"));
            continue;
        }
        double sum = 0.0;
        double sum_sq = 0.0;
        for (int i = 0; i < n; ++i) {
            double y = plot.series->at(i).y();
            sum += y;
            sum_sq += y * y;
        }
        double fn = static_cast<double>(n);
        double avg = sum / fn;
        
        if (plot.avgAction) {
            plot.avgAction->setText(QString("Average: %1").arg(avg, 0, 'f', 6));
        }

        if (plot.stdevAction) {
            if (n < 2) {
                plot.stdevAction->setText(tr("Stdev: N/A"));
            } else {
                double variance = (sum_sq - fn * avg * avg) / fn;
                double stdev = (variance > 0.0) ? std::sqrt(variance) : 0.0;
                plot.stdevAction->setText(QString("Stdev: %1").arg(stdev, 0, 'f', 6));
            }
        }
    }
}

/**
 * @brief Obliterates all existing series data, purging memory completely.
 * @details Resets internal tracking extrema and the unified time origin safely.
 */
void PlotterWindow::onClearClicked() {
    std::lock_guard<std::recursive_mutex> lock(m_plotMutex);
    for (auto& plot : m_activePlots) {
        if (plot.series) {
            m_chart->removeSeries(plot.series);
            delete plot.series;
            plot.series = nullptr;
        }
    }
    m_activePlots.clear();
    if (m_curvesMenu) {
        m_curvesMenu->clear();
    }
    m_chart->setTitle("Drag & Drop fields here");
    m_minY = std::numeric_limits<double>::infinity();
    m_maxY = -std::numeric_limits<double>::infinity();
    m_startTime = QDateTime::currentMSecsSinceEpoch(); // reset time origin
    m_legendNeedsRefresh = true;
    // remove constants too if any
}

/**
 * @brief Freezes background updates gracefully. Buffer continues, but UI slumbers.
 */
void PlotterWindow::onPauseToggled(bool checked) {
    m_paused = checked;
}

/**
 * @brief Flips mode constraints when toggling automated y-axis boundaries.
 */
void PlotterWindow::onAutoScaleToggled(bool checked) {
    m_autoScale = checked;
    m_edtMinY->setEnabled(!checked);
    m_edtMaxY->setEnabled(!checked);
    if (!checked) {
        onManualScaleChanged();
    } else {
        recalculateYBounds();
    }
}

/**
 * @brief Manual override parsing strictly for user text entries dictating min/max ranges.
 */
void PlotterWindow::onManualScaleChanged() {
    if (!m_autoScale) {
        bool okMin = false, okMax = false;
        double minY = m_edtMinY->text().toDouble(&okMin);
        double maxY = m_edtMaxY->text().toDouble(&okMax);
        if (okMin && okMax && minY < maxY) {
            m_axisY->setRange(minY, maxY);
        }
    }
}

/**
 * @brief Recalculates mathematical bounds precisely across all visual layers (DRY implementation).
 * @details Because series logic actively drops historical buffers natively as sliding Windows 
 *          pass by, computing bounds sequentially against only the surviving active frame guarantees
 *          clean structural resizing across multi-curves seamlessly without duplicating array traversal.
 */
void PlotterWindow::recalculateYBounds() {
    if (!m_autoScale) return;
    
    std::lock_guard<std::recursive_mutex> lock(m_plotMutex);
    m_minY = std::numeric_limits<double>::infinity();
    m_maxY = -std::numeric_limits<double>::infinity();
    bool hasPoints = false;
    
    for (const auto& plot : std::as_const(m_activePlots)) {
        if (!plot.series) continue;
        int count = plot.series->count();
        for (int i = 0; i < count; ++i) {
            const QPointF pt = plot.series->at(i);
            if (pt.y() < m_minY) m_minY = pt.y();
            if (pt.y() > m_maxY) m_maxY = pt.y();
            hasPoints = true;
        }
    }
    
    if (hasPoints && m_minY <= m_maxY) {
        double margin = (m_maxY - m_minY) * 0.1;
        if (margin == 0.0) margin = 1.0;
        m_axisY->setRange(m_minY - margin, m_maxY + margin);
        m_edtMinY->setText(QString::number(m_minY - margin, 'f', 2));
        m_edtMaxY->setText(QString::number(m_maxY + margin, 'f', 2));
    }
}

/**
 * @brief Injects a static infinite-length visual baseline dynamically.
 * @details Synthesizes a faux PlotConfig that bypasses network hooks but renders evenly 
 *          across the whole epoch timeline acting as a visual ruler.
 */
void PlotterWindow::onAddConstantClicked() {
    bool ok = false;
    double val = m_edtConstant->text().toDouble(&ok);
    if (!ok || !std::isfinite(val)) return;
    
    std::lock_guard<std::recursive_mutex> lock(m_plotMutex);
    
    // Create a new constant series
    PlotConfig cfg;
    cfg.senderName = "sys";
    cfg.className = "const";
    cfg.msgName = "const";
    cfg.fieldName = QString("C=%1").arg(val);
    cfg.coef = 1.0;
    
    cfg.series = new QLineSeries();
    cfg.series->setName(cfg.fieldName);
    
    // Assign black color for constant lines
    QPen pen1 = cfg.series->pen();
    pen1.setColor(Qt::black);
    pen1.setWidth(m_spnLineThickness->value());
    cfg.series->setPen(pen1);
    m_chart->addSeries(cfg.series);
    
    // We add points initially, and the rest will be updated in handleMessage
    cfg.series->append(0, val);
    cfg.series->append(m_slTimeWindow->value() / 100.0, val);
    
    cfg.series->attachAxis(m_axisX);
    cfg.series->attachAxis(m_axisY);
    
    m_activePlots.append(cfg);
    
    addCurveToMenu(m_activePlots.last());
    QTimer::singleShot(10, this, &PlotterWindow::updateLegendPosition);
}

/**
 * @brief Dynamic slider hook propagating rendering speeds natively down to QTimer intervals.
 */
void PlotterWindow::onUpdateRateChanged(int val) {
    const int interval = std::max(10, val);
    m_updateTimer->setInterval(interval);
}

/**
 * @brief Permits receiving raw text drops from other X11/Wayland Desktop apps.
 */
void PlotterWindow::dragEnterEvent(QDragEnterEvent *event) {
    if (event->mimeData()->hasText()) {
        event->acceptProposedAction();
    }
}

/**
 * @brief Orchestrates raw dropped strings into formal telemetry series requests.
 */
#include <QFile>
#include <QTimer>

class DelayedDropWatcher : public QObject {
    Q_OBJECT
    QString m_filePath;
    QTimer *m_timer;
    int m_attempts;
public:
    DelayedDropWatcher(const QString& path, QObject* parent) : QObject(parent), m_filePath(path), m_attempts(0) {
        m_timer = new QTimer(this);
        connect(m_timer, &QTimer::timeout, this, &DelayedDropWatcher::checkFile);
        m_timer->start(100);
    }
signals:
    void payloadsReady(const QString& text);
private slots:
    void checkFile() {
        m_attempts++;
        QFile f(m_filePath);
        if (f.exists()) {
            if (f.open(QIODevice::ReadOnly | QIODevice::Text)) {
                QString content = f.readAll();
                f.close();
                f.remove();
                if (!content.isEmpty()) {
                    emit payloadsReady(content);
                }
                m_timer->stop();
                deleteLater();
            }
        } else if (m_attempts > 600) { // 60 seconds timeout
            m_timer->stop();
            deleteLater();
        }
    }
};

void PlotterWindow::dropEvent(QDropEvent *event) {
    if (event->mimeData()->hasText()) {
        QString payloadText = event->mimeData()->text();
        if (payloadText.startsWith("delayed_array:")) {
            QString filePath = payloadText.mid(14);
            DelayedDropWatcher* watcher = new DelayedDropWatcher(filePath, this);
            connect(watcher, &DelayedDropWatcher::payloadsReady, this, [this](const QString& content) {
                QStringList payloads = content.split('\n', Qt::SkipEmptyParts);
                for (const QString& payload : payloads) {
                    this->addPlotFromPayload(payload);
                }
            });
            event->acceptProposedAction();
            return;
        }
        
        // payloadText can contain multiple payloads separated by newline
        QStringList payloads = payloadText.split('\n', Qt::SkipEmptyParts);
        for (const QString& payload : payloads) {
            addPlotFromPayload(payload);
        }
        event->acceptProposedAction();
    }
}

/**
 * @brief Translates structured Paparazzi textual signatures into hard-linked data curves.
 * @param payload E.g. "senderName:className:msgName:fieldName:optional_coef".
 * @details Establishes a formal Ivy-bus lambda subscription parsing the specific 
 *          index directly matching the field name required natively upon connection.
 */
void PlotterWindow::addPlotFromPayload(const QString& payload) {
    // payload format: m_senderName + ":" + m_className + ":" + msgName + ":" + fieldName + ":" + coef;
    QStringList parts = payload.split(":");
    if (parts.size() >= 4) {
        PlotConfig cfg;
        cfg.senderName = parts[0];
        cfg.className = parts[1];
        cfg.msgName = parts[2];
        
        QString fieldStr = parts[3];
        int bracketIndex = fieldStr.indexOf('[');
        if (bracketIndex != -1 && fieldStr.endsWith(']')) {
            cfg.fieldName = fieldStr.left(bracketIndex);
            cfg.arrayIndex = fieldStr.mid(bracketIndex + 1, fieldStr.length() - bracketIndex - 2).toInt();
        } else {
            cfg.fieldName = fieldStr;
            cfg.arrayIndex = -1;
        }
        
        bool ok = false;
        double scaleNext = m_edtScaleNext->text().toDouble(&ok);
        if (!ok || !std::isfinite(scaleNext)) scaleNext = 1.0;
        double coef = 1.0;
        if (parts.size() >= 5) {
            bool okCoef = false;
            coef = parts[4].toDouble(&okCoef);
            if (!okCoef || !std::isfinite(coef)) coef = 1.0;
        }
        cfg.coef = coef * scaleNext;
        if (cfg.coef == 0.0 || !std::isfinite(cfg.coef)) cfg.coef = 1.0;

        // Check if already plotted
        for (const auto& existing : m_activePlots) {
            if (existing.senderName == cfg.senderName &&
                existing.msgName == cfg.msgName &&
                existing.fieldName == cfg.fieldName &&
                existing.arrayIndex == cfg.arrayIndex) {
                return; // already plotting
            }
        }

        cfg.series = new QLineSeries();
        QString prefix = (cfg.senderName.isEmpty() || cfg.senderName == "all") ? "" : cfg.senderName + ":";
        QString classPrefix = cfg.className.isEmpty() ? "" : cfg.className + ":";
        QString arraySuffix = (cfg.arrayIndex >= 0) ? QString("[%1]").arg(cfg.arrayIndex) : "";
        cfg.series->setName(QString("%1%2%3:%4%5").arg(prefix).arg(classPrefix).arg(cfg.msgName).arg(cfg.fieldName).arg(arraySuffix));
        
        // Assign custom distinct saturated color
        QPen pen2 = cfg.series->pen();
        pen2.setColor(getNextSaturatedColor());
        pen2.setWidth(m_spnLineThickness->value());
        cfg.series->setPen(pen2);
        m_chart->addSeries(cfg.series);
        cfg.series->attachAxis(m_axisX);
        cfg.series->attachAxis(m_axisY);

        int fieldIndex = -1;
        bool alreadyBound = false;
        std::vector<pprzlink::MessageDefinition> defs;
        if (m_dict) {
            defs = m_dict->getMsgsForClass(cfg.className);
        }
        for (const auto& existing : m_activePlots) {
            if (existing.msgName == cfg.msgName && existing.className == cfg.className) {
                alreadyBound = true;
                break;
            }
        }
        for (const auto& def : defs) {
            if (def.getName() == cfg.msgName) {
                for (int k = 0; k < (int)def.getNbFields(); ++k) {
                    if (def.getField(k).getName() == cfg.fieldName) {
                        fieldIndex = k;
                        break;
                    }
                }
                if (!alreadyBound && m_link) {
                    //qDebug() << "Binding message:" << cfg.msgName;
                    m_link->BindMessage(def, this, [this](QString sender, pprzlink::Message msg) {
                        this->handleMessage(sender, msg);
                    });
                }
                break;
            }
        }
        cfg.fieldIndex = fieldIndex;

        {
            std::lock_guard<std::recursive_mutex> lock(m_plotMutex);
            m_activePlots.append(cfg);
            m_chart->setTitle("");
            
            addCurveToMenu(m_activePlots.last());
        }
        m_legendNeedsRefresh = true;
    QTimer::singleShot(10, this, &PlotterWindow::updateLegendPosition);
    }
}

/**
 * @brief Bootstraps standard menubar hooks supporting application-level suspension/quit calls.
 */
void PlotterWindow::setupMenu() {
    QMenu* plotMenu = menuBar()->addMenu(tr("&Plot"));
    
    QAction* newAction = plotMenu->addAction(tr("New"));
    newAction->setShortcut(QKeySequence("Ctrl+N"));
    connect(newAction, &QAction::triggered, this, []() {
        PlotterWindow* newWindow = new PlotterWindow();
        newWindow->show();
    });

    QAction* resetAction = plotMenu->addAction(tr("Reset"));
    resetAction->setShortcut(QKeySequence("Ctrl+L"));
    connect(resetAction, &QAction::triggered, this, &PlotterWindow::onClearClicked);

    QAction* suspendAction = plotMenu->addAction(tr("Suspend"));
    suspendAction->setShortcut(QKeySequence("Ctrl+S"));
    connect(suspendAction, &QAction::triggered, this, [this]() { m_paused = true; });

    QAction* stopAction = plotMenu->addAction(tr("Stop"));
    stopAction->setShortcut(QKeySequence("Ctrl+C"));
    connect(stopAction, &QAction::triggered, this, [this]() { m_paused = true; });

    QAction* restartAction = plotMenu->addAction(tr("Restart"));
    restartAction->setShortcut(QKeySequence("Ctrl+X"));
    connect(restartAction, &QAction::triggered, this, [this]() { m_paused = false; });

    plotMenu->addSeparator();

    QAction* closeAction = plotMenu->addAction(tr("Close"));
    closeAction->setShortcut(QKeySequence("Ctrl+W"));
    connect(closeAction, &QAction::triggered, this, &QWidget::close);

    QAction* quitAction = plotMenu->addAction(tr("Quit"));
    quitAction->setShortcut(QKeySequence("Ctrl+Q"));
    connect(quitAction, &QAction::triggered, qApp, &QApplication::quit);

    m_curvesMenu = menuBar()->addMenu(tr("&Curves"));
}

/**
 * @brief Thread-safe ingestion queue receiving highly asynchronous Ivy bus telemetry packages.
 * @param sender Originating entity emitting the message format.
 * @param msg Validated and natively inflated binary structure definition mapping payload contents.
 * @details Appends numerical coordinates natively formatted into an internal memory buffer. 
 *          We explicitly restrict raw redraw actions (`series->append`) here, delegating drawing 
 *          solely to the main GUI event loop syncing logic securely to 60fps refresh limits.
 */
void PlotterWindow::handleMessage(QString sender, const pprzlink::Message& msg) {
    if (m_paused) return; // Skip updating data while paused

    QString sId = sender;
    if (sId.isEmpty()) {
        const auto& senderV = msg.getSenderId();
        if (std::holds_alternative<QString>(senderV)) sId = std::get<QString>(senderV);
        else sId = QString::number(std::get<uint8_t>(senderV));
    }
    if (sId.isEmpty()) sId = "ground";

    QString msgName = msg.getDefinition().getName();
    double currentTime = (QDateTime::currentMSecsSinceEpoch() - m_startTime) / 1000.0;
    
    std::lock_guard<std::recursive_mutex> lock(m_plotMutex);
    
    for (auto& plot : m_activePlots) {
        if (!plot.series) continue;
        if (plot.msgName == msgName && (plot.senderName == sId || plot.senderName == "all")) {
            const auto& def = msg.getDefinition();
            int fieldIndex = plot.fieldIndex;
            if (fieldIndex < 0 || fieldIndex >= (int)def.getNbFields()) {
                // fallback path for legacy configs or missing cached index
                for (int i = 0; i < (int)def.getNbFields(); ++i) {
                    if (def.getField(i).getName() == plot.fieldName) {
                        fieldIndex = i;
                        plot.fieldIndex = i;
                        break;
                    }
                }
            }
            if (fieldIndex < 0 || fieldIndex >= (int)def.getNbFields()) {
                continue;
            }
            try {
                const auto& rv = msg.getRawValue(fieldIndex);
                if (rv.getType().isArray() && plot.arrayIndex == -1) {
                    try {
                        int arrSize = 0;
                        switch (rv.getType().getBaseType()) {
                            case pprzlink::BaseType::CHAR: { std::vector<char> v; rv.getValue(v); arrSize = v.size(); } break;
                            case pprzlink::BaseType::INT8: { std::vector<int8_t> v; rv.getValue(v); arrSize = v.size(); } break;
                            case pprzlink::BaseType::INT16: { std::vector<int16_t> v; rv.getValue(v); arrSize = v.size(); } break;
                            case pprzlink::BaseType::INT32: { std::vector<int32_t> v; rv.getValue(v); arrSize = v.size(); } break;
                            case pprzlink::BaseType::UINT8: { std::vector<uint8_t> v; rv.getValue(v); arrSize = v.size(); } break;
                            case pprzlink::BaseType::UINT16: { std::vector<uint16_t> v; rv.getValue(v); arrSize = v.size(); } break;
                            case pprzlink::BaseType::UINT32: { std::vector<uint32_t> v; rv.getValue(v); arrSize = v.size(); } break;
                            case pprzlink::BaseType::FLOAT: { std::vector<float> v; rv.getValue(v); arrSize = v.size(); } break;
                            case pprzlink::BaseType::DOUBLE: { std::vector<double> v; rv.getValue(v); arrSize = v.size(); } break;
                            default: break;
                        }
                        if (arrSize > 0) {
                            for (int k = 0; k < arrSize; ++k) {
                                QString subPayload = QString("%1:%2:%3:%4[%5]:%6")
                                    .arg(plot.senderName)
                                    .arg(plot.className)
                                    .arg(plot.msgName)
                                    .arg(plot.fieldName)
                                    .arg(k)
                                    .arg(plot.coef);
                                QMetaObject::invokeMethod(this, [this, subPayload]() {
                                    this->addPlotFromPayload(subPayload);
                                }, Qt::QueuedConnection);
                            }
                            QMetaObject::invokeMethod(this, [this, series = plot.series]() {
                                this->removeCurve(series);
                            }, Qt::QueuedConnection);
                        }
                    } catch (...) {}
                    continue;
                }

                double val = fieldValueAsDouble(rv, plot.arrayIndex);
                if (!std::isfinite(val)) continue;
                val *= plot.coef;

                if (plot.discrete) {
                    double lastY = val;
                    bool hasLastY = false;
                    if (!plot.buffer.isEmpty()) {
                        lastY = plot.buffer.last().y();
                        hasLastY = true;
                    } else if (plot.series && plot.series->count() > 0) {
                        lastY = plot.series->at(plot.series->count() - 1).y();
                        hasLastY = true;
                    }
                    // Optimize step-functions: only inject the right-angle corner if the state actually changed.
                    if (hasLastY && lastY != val) {
                        plot.buffer.append(QPointF(currentTime, lastY));
                    }
                }

                plot.buffer.append(QPointF(currentTime, val));
            } catch (const std::exception& e) {
                qWarning() << "Exception in handleMessage:" << e.what();
            } catch (...) {
                qWarning() << "Unknown exception in handleMessage.";
            }
        }
    }
}

/**
 * @brief Core 60Hz rendering pass draining back-buffers flushing into native widget views.
 * @details Modifies visual ranges by stripping natively expired history points mathematically off 
 *          the time window scale (X-axis). Performs localized Y bounds expansion directly as 
 *          arrays stream safely across active timeframes. Employs `std::isfinite` to guard 
 *          against QChart canvas corruptions safely.
 */
void PlotterWindow::updatePlots() {
    if (m_paused) return;

    double currentTime = (QDateTime::currentMSecsSinceEpoch() - m_startTime) / 1000.0;
    double windowSize = m_slTimeWindow->value() / 100.0;
    bool needsAxisUpdate = false;
    bool needsFullRecalc = false;

    std::lock_guard<std::recursive_mutex> lock(m_plotMutex);

    for (auto& plot : m_activePlots) {
        if (!plot.series) continue;
        if (plot.className == "const") {
            bool ok = false;
            double val = plot.fieldName.section('=', 1).toDouble(&ok);
            if (!ok || !std::isfinite(val)) continue;
            double startX = std::max(0.0, currentTime - windowSize);
            plot.series->replace(
                QList<QPointF>() << QPointF(startX, val) << QPointF(std::max(windowSize, currentTime), val)
            );
            continue;
        }

        if (!plot.buffer.isEmpty()) {
            plot.series->append(plot.buffer);
            for (const QPointF& pt : std::as_const(plot.buffer)) {
                if (pt.y() < m_minY) { m_minY = pt.y(); needsAxisUpdate = true; }
                if (pt.y() > m_maxY) { m_maxY = pt.y(); needsAxisUpdate = true; }
            }
            plot.buffer.clear();
        }

        int pointsToRemove = 0;
        int count = plot.series->count();
        double cutoffTime = currentTime - windowSize;
        while (pointsToRemove < count && plot.series->at(pointsToRemove).x() < cutoffTime) {
            double ptY = plot.series->at(pointsToRemove).y();
            // If the point we're dropping defined the bounding box, we must shrink/re-evaluate the whole box natively.
            if (m_autoScale && (ptY <= m_minY || ptY >= m_maxY)) {
                needsFullRecalc = true;
            }
            pointsToRemove++;
        }
        if (pointsToRemove > 0) {
            plot.series->removePoints(0, pointsToRemove);
        }
    }

    if (currentTime > windowSize) {
        m_axisX->setRange(currentTime - windowSize, currentTime);
    } else {
        m_axisX->setRange(0, windowSize);
    }

    if (m_autoScale) {
        if (needsFullRecalc) {
            recalculateYBounds();
        } else if (needsAxisUpdate && m_minY <= m_maxY) {
            double margin = (m_maxY - m_minY) * 0.1;
            if (margin == 0) margin = 1.0;
            m_axisY->setRange(m_minY - margin, m_maxY + margin);
            m_edtMinY->setText(QString::number(m_minY - margin, 'f', 2));
            m_edtMaxY->setText(QString::number(m_maxY + margin, 'f', 2));
        }
    }
    m_legendNeedsRefresh = true;
}

/**
 * @brief Automates drop-down bindings allocating math operators / deletion tools onto curves.
 * @param cfg Passed dynamically to tether UI state triggers directly towards struct instances.
 */
void PlotterWindow::addCurveToMenu(PlotConfig& cfg) {
    if (!m_curvesMenu || !cfg.series) return;

    QPixmap pixmap(16, 16);
    pixmap.fill(cfg.series->color());
    QIcon icon(pixmap);

    QMenu* curveMenu = m_curvesMenu->addMenu(icon, cfg.series->name());

    QAction* avgAction = curveMenu->addAction(tr("Average: N/A"));
    avgAction->setEnabled(false);
    cfg.avgAction = avgAction;

    QAction* stdevAction = curveMenu->addAction(tr("Stdev: N/A"));
    stdevAction->setEnabled(false);
    cfg.stdevAction = stdevAction;
    
    QAction* deleteAction = curveMenu->addAction(tr("Delete"));
    QLineSeries* targetSeries = cfg.series;
    connect(deleteAction, &QAction::triggered, this, [this, targetSeries, curveMenu]() {
        removeCurve(targetSeries);
        delete curveMenu;
    });

    QAction* discreteAction = curveMenu->addAction(tr("Discrete"));
    discreteAction->setCheckable(true);
    discreteAction->setChecked(cfg.discrete);
    connect(discreteAction, &QAction::toggled, this, [this, targetSeries](bool checked) {
        std::lock_guard<std::recursive_mutex> lock(m_plotMutex);
        for (auto& plot : m_activePlots) {
            if (plot.series == targetSeries) {
                plot.discrete = checked;
                break;
            }
        }
    });
}

/**
 * @brief Obliterates a selected line trajectory correctly de-registering GUI and heap ties.
 * @param series Targets exactly which curve UI element triggered the destruction hook.
 */
void PlotterWindow::removeCurve(QLineSeries* series) {
    if (!series) return;
    
    std::lock_guard<std::recursive_mutex> lock(m_plotMutex);
    
    for (int i = 0; i < m_activePlots.size(); ++i) {
        if (m_activePlots[i].series == series) {
            m_chart->removeSeries(series);
            m_activePlots[i].series = nullptr;
            m_activePlots.removeAt(i);
            delete series;
            series = nullptr;
            
            recalculateYBounds();
            
            QTimer::singleShot(10, this, &PlotterWindow::updateLegendPosition);
            m_legendNeedsRefresh = true;
            break;
        }
    }
}

/**
 * @brief Automatically repaints floating labels mirroring internal QLegend positions correctly.
 * @details Re-assembles bespoke text overlays simulating natively docked legends securely handling 
 *          font heights automatically against active chart bounding rectangles.
 */
void PlotterWindow::updateLegendPosition() {
    if (!m_chart || !m_legendOverlay || !m_legendLayout) return;
    
    m_chart->legend()->hide();
    
    QLayoutItem *child;
    while ((child = m_legendLayout->takeAt(0)) != nullptr) {
        if (child->widget()) delete child->widget();
        delete child;
    }
    
    auto seriesList = m_chart->series();
    if (seriesList.isEmpty()) {
        m_legendOverlay->hide();
        return;
    }
    
    m_legendOverlay->show();
    
    for (auto* s : seriesList) {
        QLineSeries* ls = qobject_cast<QLineSeries*>(s);
        if (ls) {
            QWidget* rowWidget = new QWidget;
            QHBoxLayout* rowLayout = new QHBoxLayout(rowWidget);
            rowLayout->setContentsMargins(4, 2, 4, 2);
            rowLayout->setSpacing(5);
            
            QLabel* colorBox = new QLabel;
            QString colorMsg = ls->pen().color().name();
            colorBox->setStyleSheet(QString("background-color: %1; border: none;").arg(colorMsg));
            
            double latestVal = 0.0;
            if (ls->count() > 0) {
                latestVal = ls->at(ls->count() - 1).y();
            }
            QLabel* textLbl = new QLabel(QString("%1 : %2").arg(ls->name()).arg(latestVal, 0, 'f', 4));
            textLbl->setStyleSheet("color: black; border: none; background: transparent;");
            textLbl->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            
            int textHeight = textLbl->fontMetrics().height();
            // Optional: You can reduce it slightly if the font height includes big ascender/descender margins, 
            // e.g. int boxSize = textHeight * 0.8; but textHeight directly is a safe square.
            int boxSize = textHeight;
            colorBox->setFixedSize(boxSize, boxSize);
            
            rowLayout->addWidget(textLbl, 1);
            rowLayout->addWidget(colorBox);
            m_legendLayout->addWidget(rowWidget);
        }
    }
    
    m_legendOverlay->adjustSize();
    if (QWidget* parent = m_legendOverlay->parentWidget()) {
        const int margin = 15;
        int x = std::max(0, parent->width() - m_legendOverlay->width() - margin);
        m_legendOverlay->move(x, margin);
    }
}

/**
 * @brief Traverses active frames to cleanly recompute label readouts asynchronously.
 */
void PlotterWindow::updateLegendValues() {
    if (!m_legendOverlay || !m_legendLayout) return;
    
    auto seriesList = m_chart->series();
    if (m_legendLayout->count() != seriesList.size()) {
        updateLegendPosition();
        return;
    }
    
    for (int i = 0; i < seriesList.size(); ++i) {
        QLineSeries* ls = qobject_cast<QLineSeries*>(seriesList[i]);
        if (ls) {
            double latestVal = 0.0;
            if (ls->count() > 0) {
                latestVal = ls->at(ls->count() - 1).y();
            }
            
            QLayoutItem* item = m_legendLayout->itemAt(i);
            if (item) {
                QWidget* rowWidget = item->widget();
                if (rowWidget) {
                    QHBoxLayout* rowLayout = qobject_cast<QHBoxLayout*>(rowWidget->layout());
                    if (rowLayout && rowLayout->count() >= 2) {
                        QLabel* lbl = qobject_cast<QLabel*>(rowLayout->itemAt(0)->widget());
                        if (lbl) {
                            lbl->setText(QString("%1 : %2").arg(ls->name()).arg(latestVal, 0, 'f', 4));
                        }
                    }
                }
            }
        }
    }
    m_legendOverlay->adjustSize();
    if (QWidget* parent = m_legendOverlay->parentWidget()) {
        const int margin = 15;
        int x = std::max(0, parent->width() - m_legendOverlay->width() - margin);
        m_legendOverlay->move(x, margin);
    }
}

/**
 * @brief Ensures overlay layout components reposition perfectly matching arbitrary desktop reframes.
 */
void PlotterWindow::resizeEvent(QResizeEvent *event) {
    QMainWindow::resizeEvent(event);
    updateLegendPosition();
}

/**
 * @brief Interactively bolsters or weakens global plotting pixel strokes dynamically.
 */
void PlotterWindow::onLineThicknessChanged(int val) {
    std::lock_guard<std::recursive_mutex> lock(m_plotMutex);
    for (auto& plot : m_activePlots) {
        if (!plot.series) continue;
        QPen p = plot.series->pen();
        p.setWidth(val);
        plot.series->setPen(p);
    }
}

/**
 * @class EditorLighteningStyle
 * @brief Enhances text input legibility specifically for deep dark GTK environments trivially.
 * @details Instead of manipulating deeply entrenched CSS strings and hard-breaking system logic, 
 *          we exploit dynamic `QProxyStyle` interception resolving color mismatches automatically.
 */
class EditorLighteningStyle : public QProxyStyle {
public:
    // Inherit constructors from QProxyStyle
    using QProxyStyle::QProxyStyle; 

    // The polish function is called automatically for every widget 
    // right before it is displayed.
    void polish(QWidget *widget) override {
        // Always call the base class implementation first
        QProxyStyle::polish(widget); 

        // Check if the current widget is an edit field or a spinbox
        if (qobject_cast<QLineEdit*>(widget) ||
            qobject_cast<QTextEdit*>(widget) ||
            qobject_cast<QPlainTextEdit*>(widget) ||
            qobject_cast<QAbstractSpinBox*>(widget)) {
            
            // It's a match! Grab this specific widget's palette
            QPalette customPalette = widget->palette();
            
            // Change the Base color to a lighter dark-mode gray
            customPalette.setColor(QPalette::Base, QColor("#3a3a3a"));
            
            // Apply it ONLY to this specific widget
            widget->setPalette(customPalette);
        }
    }
};

/**
 * @brief Formal execution entry point instantiating process rules and UI execution contexts.
 */
int main(int argc, char *argv[]) 
{
    // Set metadata BEFORE application instantiation to prevent XDG portal double-registration 
    // root cause ("Connection already associated with an application ID").
    QCoreApplication::setApplicationVersion("1.0");
    //QCoreApplication::setOrganizationName("paparazzi"); // only for settings, not really relevant here
    // Follow XDG spec for desktop integration and use a fixed name to ensure the .desktop file is correctly associated with the app
    QGuiApplication::setDesktopFileName(QStringLiteral("paparazzi_plotter"));
    QCoreApplication::setApplicationName(QStringLiteral("Real-time Plotter"));

    QApplication app(argc, argv);

    // Apply the custom proxy style to the application.
    // We pass app.style() so it inherits all the default OS/Wayland drawing 
    // behavior, simply layering our palette override on top.
    app.setStyle(new EditorLighteningStyle(app.style()));

    //app.setApplicationDisplayName(QStringLiteral("Real-time Plotter"));

    QString iconPath = ":/penguin_icon_rtp.png";
    QIcon icon(iconPath);
    installLinuxDesktopIntegration(app.desktopFileName(), "Paparazzi Real-Time Plotter", "Real-time plotter for telemetry messages", iconPath, "paparazzi-plotter");

    app.setWindowIcon(icon);

    PlotterWindow window;
    window.setWindowIcon(icon);
    window.show();

    return app.exec();
}

#include "plotter.moc"
