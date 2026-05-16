#include "PlotterWindow.h"
#include <QLineSeries>
#include <QChart>
#include <QChartView>
#include <QValueAxis>
#include <QMimeData>
#include <sstream>
#include <iostream>

#include "pprzlinkQt/Message.h"
#include "pprzlinkQt/MessageDictionary.h"
#include "pprzlinkQt/IvyQtLink.h"
#include <QDebug>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QTimer>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QApplication>
#include <QKeySequence>


PlotterWindow::PlotterWindow(QWidget *parent) : QMainWindow(parent), m_minY(1e9), m_maxY(-1e9), m_paused(false), m_autoScale(true) {
    setAcceptDrops(true);
    setWindowTitle("Real-Time Plotter");

    m_chart = new QChart();
    m_chart->setTitle("Drag & Drop fields here");
    m_chart->legend()->setVisible(true);
    m_chart->setAnimationOptions(QChart::NoAnimation);

    m_axisX = new QValueAxis();
    m_axisX->setTitleText("Time (s)");
    m_chart->addAxis(m_axisX, Qt::AlignBottom);

    m_axisY = new QValueAxis();
    m_axisY->setTitleText("Value");
    m_chart->addAxis(m_axisY, Qt::AlignLeft);

    setupUI();
    setupMenu();

    m_startTime = QDateTime::currentMSecsSinceEpoch();

    setupIvy();
}

PlotterWindow::~PlotterWindow() {
    if (m_link) {
        m_link->stop();
        delete m_link;
    }
    delete m_dict;
}

void PlotterWindow::setupIvy() {
    QString phome = qgetenv("PAPARAZZI_HOME");
    if (phome.isEmpty()) phome = QString("/home/%1/paparazzi").arg(qgetenv("USER"));
    QString xmlPath = phome + "/var/messages.xml";

    m_dict = new pprzlink::MessageDictionary(xmlPath);
    m_link = new pprzlink::IvyQtLink(*m_dict, "plotter_qt", this);
    m_link->start("127.255.255.255:2010");
}

void PlotterWindow::setupUI() {
    QWidget *mainWidget = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(mainWidget);

    QWidget *toolbarWidget = new QWidget();
    QHBoxLayout *toolbarLayout = new QHBoxLayout(toolbarWidget);

    QPushButton *btnClear = new QPushButton("Clear");
    QPushButton *btnPause = new QPushButton("Pause");
    btnPause->setCheckable(true);

    m_cbAutoScale = new QCheckBox("Auto Scale");
    m_cbAutoScale->setChecked(true);

    m_spnMinY = new QDoubleSpinBox();
    m_spnMaxY = new QDoubleSpinBox();
    m_spnMinY->setRange(-1000000, 1000000);
    m_spnMaxY->setRange(-1000000, 1000000);
    m_spnMinY->setEnabled(false);
    m_spnMaxY->setEnabled(false);

    QLabel *lblTime = new QLabel("Time Window (s):");
    m_spnTimeWindow = new QDoubleSpinBox();
    m_spnTimeWindow->setRange(1, 1000);
    m_spnTimeWindow->setValue(10.0);

    QLabel *lblConst = new QLabel("Constant:");
    m_spnConstant = new QDoubleSpinBox();
    m_spnConstant->setRange(-1000000, 1000000);
    QPushButton *btnAddConst = new QPushButton("Add");

    QLabel *lblUpdate = new QLabel("Update Rate (Hz):");
    m_spnUpdateRate = new QDoubleSpinBox();
    m_spnUpdateRate->setRange(1, 100); // 1Hz to 100Hz
    m_spnUpdateRate->setValue(20.0); // Default to 20Hz (0.05s) - 10x faster than plotter.ml's 0.5s

    m_updateTimer = new QTimer(this);
    m_updateTimer->start(1000 / m_spnUpdateRate->value());

    toolbarLayout->addWidget(btnClear);
    toolbarLayout->addWidget(btnPause);
    toolbarLayout->addWidget(m_cbAutoScale);
    toolbarLayout->addWidget(new QLabel("Min:"));
    toolbarLayout->addWidget(m_spnMinY);
    toolbarLayout->addWidget(new QLabel("Max:"));
    toolbarLayout->addWidget(m_spnMaxY);
    toolbarLayout->addWidget(lblTime);
    toolbarLayout->addWidget(m_spnTimeWindow);
    toolbarLayout->addWidget(lblConst);
    toolbarLayout->addWidget(m_spnConstant);
    toolbarLayout->addWidget(btnAddConst);
    toolbarLayout->addWidget(lblUpdate);
    toolbarLayout->addWidget(m_spnUpdateRate);
    toolbarLayout->addStretch();

    QChartView *chartView = new QChartView(m_chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setAcceptDrops(false); // Let drops fall through

    mainLayout->addWidget(toolbarWidget);
    mainLayout->addWidget(chartView);
    setCentralWidget(mainWidget);

    connect(btnClear, &QPushButton::clicked, this, &PlotterWindow::onClearClicked);
    connect(btnPause, &QPushButton::toggled, this, &PlotterWindow::onPauseToggled);
    connect(m_cbAutoScale, &QCheckBox::toggled, this, &PlotterWindow::onAutoScaleToggled);
    connect(m_spnMinY, &QDoubleSpinBox::editingFinished, this, &PlotterWindow::onManualScaleChanged);
    connect(m_spnMaxY, &QDoubleSpinBox::editingFinished, this, &PlotterWindow::onManualScaleChanged);
    connect(btnAddConst, &QPushButton::clicked, this, &PlotterWindow::onAddConstantClicked);
    connect(m_spnUpdateRate, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &PlotterWindow::onUpdateRateChanged);
    connect(m_updateTimer, &QTimer::timeout, this, &PlotterWindow::updatePlots);
}

void PlotterWindow::onClearClicked() {
    for (auto& plot : m_activePlots) {
        m_chart->removeSeries(plot.series);
        delete plot.series;
    }
    m_activePlots.clear();
    m_minY = 1e9;
    m_maxY = -1e9;
    m_startTime = QDateTime::currentMSecsSinceEpoch(); // reset time origin
    // remove constants too if any
}

void PlotterWindow::onPauseToggled(bool checked) {
    m_paused = checked;
}

void PlotterWindow::onAutoScaleToggled(bool checked) {
    m_autoScale = checked;
    m_spnMinY->setEnabled(!checked);
    m_spnMaxY->setEnabled(!checked);
    if (!checked) {
        onManualScaleChanged();
    }
}

void PlotterWindow::onManualScaleChanged() {
    if (!m_autoScale) {
        m_axisY->setRange(m_spnMinY->value(), m_spnMaxY->value());
    }
}

void PlotterWindow::onAddConstantClicked() {
    double val = m_spnConstant->value();
    
    // Create a new constant series
    PlotConfig cfg;
    cfg.senderName = "sys";
    cfg.className = "const";
    cfg.msgName = "const";
    cfg.fieldName = QString("C=%1").arg(val);
    cfg.coef = 1.0;
    
    cfg.series = new QLineSeries();
    cfg.series->setName(cfg.fieldName);
    
    // We add points initially, and the rest will be updated in handleMessage
    cfg.series->append(0, val);
    cfg.series->append(m_spnTimeWindow->value(), val);
    
    m_chart->addSeries(cfg.series);
    cfg.series->attachAxis(m_axisX);
    cfg.series->attachAxis(m_axisY);
    
    m_activePlots.append(cfg);
}

void PlotterWindow::onUpdateRateChanged(double val) {
    if (val > 0) {
        m_updateTimer->setInterval(1000.0 / val);
    }
}

void PlotterWindow::dragEnterEvent(QDragEnterEvent *event) {
    if (event->mimeData()->hasText()) {
        event->acceptProposedAction();
    }
}

void PlotterWindow::dropEvent(QDropEvent *event) {
    if (event->mimeData()->hasText()) {
        QString payload = event->mimeData()->text();
        qDebug() << "Dropped payload:" << payload;
        addPlotFromPayload(payload);
        event->acceptProposedAction();
    }
}

void PlotterWindow::addPlotFromPayload(const QString& payload) {
    // payload format: m_senderName + ":" + m_className + ":" + msgName + ":" + fieldName + ":" + coef;
    QStringList parts = payload.split(":");
    if (parts.size() >= 4) {
        PlotConfig cfg;
        cfg.senderName = parts[0];
        cfg.className = parts[1];
        cfg.msgName = parts[2];
        cfg.fieldName = parts[3];
        cfg.coef = (parts.size() >= 5) ? parts[4].toDouble() : 1.0;
        if (cfg.coef == 0.0) cfg.coef = 1.0;

        // Check if already plotted
        for (const auto& existing : m_activePlots) {
            if (existing.senderName == cfg.senderName &&
                existing.msgName == cfg.msgName &&
                existing.fieldName == cfg.fieldName) {
                return; // already plotting
            }
        }

        cfg.series = new QLineSeries();
        cfg.series->setName(QString("%1:%2").arg(cfg.msgName).arg(cfg.fieldName));
        
        m_chart->addSeries(cfg.series);
        cfg.series->attachAxis(m_axisX);
        cfg.series->attachAxis(m_axisY);

        // Bind message if not already bound
        bool alreadyBound = false;
        for (const auto& existing : m_activePlots) {
            if (existing.msgName == cfg.msgName && existing.className == cfg.className) {
                alreadyBound = true;
                break;
            }
        }
        
        m_activePlots.append(cfg);

        if (!alreadyBound) {
            const auto msgs = m_dict->getMsgsForClass(cfg.className);
            for (const auto& def : msgs) {
                if (def.getName() == cfg.msgName) {
                    qDebug() << "Binding message:" << cfg.msgName;
                    m_link->BindMessage(def, this, [this](QString sender, pprzlink::Message msg) {
                        this->handleMessage(sender, msg);
                    });
                    break;
                }
            }
        }
    }
}



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
    
    for (auto& plot : m_activePlots) {
        if (plot.msgName == msgName && (plot.senderName == sId || plot.senderName == "all")) {
            // Find field index
            const auto& def = msg.getDefinition();
            for (int i = 0; i < (int)def.getNbFields(); ++i) {
                if (def.getField(i).getName() == plot.fieldName) {
                    try {
                        const auto& rv = msg.getRawValue(i);
                        std::stringstream ss;
                        ss << rv;
                        double val = QString::fromStdString(ss.str()).toDouble();
                        val *= plot.coef;
                        plot.buffer.append(QPointF(currentTime, val));
                    } catch(...) {}
                    break;
                }
            }
        }
    }
}

void PlotterWindow::updatePlots() {
    if (m_paused) return;

    double currentTime = (QDateTime::currentMSecsSinceEpoch() - m_startTime) / 1000.0;
    double windowSize = m_spnTimeWindow->value();
    bool needsAxisUpdate = false;

    for (auto& plot : m_activePlots) {
        if (plot.className == "const") {
            double val = plot.fieldName.section('=', 1).toDouble();
            double startX = std::max(0.0, currentTime - windowSize);
            plot.series->replace(
                QList<QPointF>() << QPointF(startX, val) << QPointF(std::max(windowSize, currentTime), val)
            );
            continue;
        }

        if (plot.buffer.isEmpty()) continue;

        plot.series->append(plot.buffer);
        
        if (m_autoScale) {
            for (const QPointF& pt : qAsConst(plot.buffer)) {
                if (pt.y() < m_minY) { m_minY = pt.y(); needsAxisUpdate = true; }
                if (pt.y() > m_maxY) { m_maxY = pt.y(); needsAxisUpdate = true; }
            }
        }
        
        plot.buffer.clear();

        int pointsToRemove = 0;
        while (pointsToRemove < plot.series->count() && plot.series->at(pointsToRemove).x() < currentTime - windowSize) {
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

    if (needsAxisUpdate && m_autoScale) {
        double margin = (m_maxY - m_minY) * 0.1;
        if (margin == 0) margin = 1.0;
        m_axisY->setRange(m_minY - margin, m_maxY + margin);
    }
}
