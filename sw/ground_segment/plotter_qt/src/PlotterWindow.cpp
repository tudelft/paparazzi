#include "PlotterWindow.h"
#include <QColor>
#include <cmath>

static int g_colorIndex = 0;
static QColor getNextSaturatedColor() {
    double h = std::fmod(g_colorIndex * 137.508, 360.0);
    g_colorIndex++;
    return QColor::fromHsvF(h / 360.0, 0.9, 0.9);
}


#include <QGraphicsLayout>
#include <QGraphicsScene>
#include <QLineSeries>
#include <QLegendMarker>
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
#include <QLineEdit>
#include <QLabel>
#include <QTimer>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QApplication>
#include <QKeySequence>
#include <QSlider>


PlotterWindow::PlotterWindow(QWidget *parent) : QMainWindow(parent), m_minY(1e9), m_maxY(-1e9), m_paused(false), m_autoScale(true) {
    m_legendOverlay = nullptr;
    m_legendLayout = nullptr;
    setAcceptDrops(true);
    setWindowTitle("Plotter");

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
    m_axisX->setLabelFormat("%gs");
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
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);

    QWidget *toolbarWidget = new QWidget();
    QHBoxLayout *toolbarLayout = new QHBoxLayout(toolbarWidget);
    toolbarLayout->setContentsMargins(2, 2, 2, 2); // keep some margin for the toolbar

    m_cbAutoScale = new QCheckBox("Auto Scale");
    m_cbAutoScale->setChecked(true);

    m_edtMinY = new QLineEdit();
    m_edtMaxY = new QLineEdit();
    m_edtMinY->setEnabled(false);
    m_edtMaxY->setEnabled(false);

    m_slTimeWindow = new QSlider(Qt::Horizontal);
    m_slTimeWindow->setToolTip("Time Window (s)");
    m_slTimeWindow->setRange(1, 1000);
    m_slTimeWindow->setValue(10);


    QLabel *lblConst = new QLabel("Constant:");
    m_edtConstant = new QLineEdit();

    QLabel *lblScaleNext = new QLabel("Scale next by:");
    m_edtScaleNext = new QLineEdit("1.0");
    m_edtScaleNext->setToolTip("Scale next curve (e.g. 0.0174 to convert deg in rad, 57.3 to convert rad in deg)");


    m_slUpdateRate = new QSlider(Qt::Horizontal);
    m_slUpdateRate->setToolTip("Update Rate (ms)");
    m_slUpdateRate->setRange(10, 1000); // 10ms to 1000ms
    m_slUpdateRate->setValue(50); // Default to 50ms (20Hz)

    m_spnLineThickness = new QSpinBox();
    m_spnLineThickness->setToolTip("Line Thickness (px)");
    m_spnLineThickness->setRange(1, 10);
    m_spnLineThickness->setValue(1);
    m_spnLineThickness->hide();

    m_updateTimer = new QTimer(this);
    m_updateTimer->start(m_slUpdateRate->value());

    toolbarLayout->addWidget(m_cbAutoScale);
    toolbarLayout->addWidget(new QLabel("Min:"));
    toolbarLayout->addWidget(m_edtMinY);
    toolbarLayout->addWidget(new QLabel("Max:"));
    toolbarLayout->addWidget(m_edtMaxY);
    toolbarLayout->addWidget(m_slTimeWindow);
    toolbarLayout->addWidget(m_slUpdateRate);
    toolbarLayout->addWidget(lblConst);
    toolbarLayout->addWidget(m_edtConstant);
    toolbarLayout->addWidget(lblScaleNext);
    toolbarLayout->addWidget(m_edtScaleNext);
    QLabel* lblLineThickness = new QLabel("Line:");
    lblLineThickness->hide();
    toolbarLayout->addWidget(lblLineThickness);
    toolbarLayout->addWidget(m_spnLineThickness);
    toolbarLayout->addStretch();


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

void PlotterWindow::onClearClicked() {
    for (auto& plot : m_activePlots) {
        m_chart->removeSeries(plot.series);
        delete plot.series;
    }
    m_activePlots.clear();
    m_chart->setTitle("Drag & Drop fields here");
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
    m_edtMinY->setEnabled(!checked);
    m_edtMaxY->setEnabled(!checked);
    if (!checked) {
        onManualScaleChanged();
    } else {
        if (m_minY <= m_maxY) {
            double margin = (m_maxY - m_minY) * 0.1;
            if (margin == 0) margin = 1.0;
            m_axisY->setRange(m_minY - margin, m_maxY + margin);
            m_edtMinY->setText(QString::number(m_minY - margin, 'f', 2));
            m_edtMaxY->setText(QString::number(m_maxY + margin, 'f', 2));
        }
    }
}

void PlotterWindow::onManualScaleChanged() {
    if (!m_autoScale) {
        m_axisY->setRange(m_edtMinY->text().toDouble(), m_edtMaxY->text().toDouble());
    }
}

void PlotterWindow::onAddConstantClicked() {
    bool ok;
    double val = m_edtConstant->text().toDouble(&ok);
    if (!ok) return;
    
    // Create a new constant series
    PlotConfig cfg;
    cfg.senderName = "sys";
    cfg.className = "const";
    cfg.msgName = "const";
    cfg.fieldName = QString("C=%1").arg(val);
    cfg.coef = 1.0;
    
    cfg.series = new QLineSeries();
    cfg.series->setName(cfg.fieldName);
    
    // Assign custom distinct saturated color
    QPen pen1 = cfg.series->pen();
    pen1.setColor(getNextSaturatedColor());
    pen1.setWidth(m_spnLineThickness->value());
    cfg.series->setPen(pen1);
    m_chart->addSeries(cfg.series);
    
    // We add points initially, and the rest will be updated in handleMessage
    cfg.series->append(0, val);
    cfg.series->append(m_slTimeWindow->value(), val);
    
    cfg.series->attachAxis(m_axisX);
    cfg.series->attachAxis(m_axisY);
    
    m_activePlots.append(cfg);
    
    addCurveToMenu(m_activePlots.last());
    QTimer::singleShot(10, this, &PlotterWindow::updateLegendPosition);
}

void PlotterWindow::onUpdateRateChanged(int val) {
    if (val > 0) {
        m_updateTimer->setInterval(val);
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
        
        bool ok = false;
        double scaleNext = m_edtScaleNext->text().toDouble(&ok);
        if (!ok) scaleNext = 1.0;
        
        cfg.coef = ((parts.size() >= 5) ? parts[4].toDouble() : 1.0) * scaleNext;

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
        QString prefix = (cfg.senderName.isEmpty() || cfg.senderName == "all") ? "" : cfg.senderName + ":";
        QString classPrefix = cfg.className.isEmpty() ? "" : cfg.className + ":";
        cfg.series->setName(QString("%1%2%3:%4").arg(prefix).arg(classPrefix).arg(cfg.msgName).arg(cfg.fieldName));
        
        // Assign custom distinct saturated color
        QPen pen2 = cfg.series->pen();
        pen2.setColor(getNextSaturatedColor());
        pen2.setWidth(m_spnLineThickness->value());
        cfg.series->setPen(pen2);
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
        m_chart->setTitle("");
        
        addCurveToMenu(m_activePlots.last());
    QTimer::singleShot(10, this, &PlotterWindow::updateLegendPosition);

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

                        if (plot.discrete) {
                            double lastY = val;
                            bool hasLastY = false;
                            if (!plot.buffer.isEmpty()) {
                                lastY = plot.buffer.last().y();
                                hasLastY = true;
                            } else if (plot.series->count() > 0) {
                                lastY = plot.series->at(plot.series->count() - 1).y();
                                hasLastY = true;
                            }
                            if (hasLastY) {
                                plot.buffer.append(QPointF(currentTime, lastY));
                            }
                        }

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
    double windowSize = m_slTimeWindow->value();
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
        
        for (const QPointF& pt : qAsConst(plot.buffer)) {
            if (pt.y() < m_minY) { m_minY = pt.y(); needsAxisUpdate = true; }
            if (pt.y() > m_maxY) { m_maxY = pt.y(); needsAxisUpdate = true; }
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
        m_edtMinY->setText(QString::number(m_minY - margin, 'f', 2));
        m_edtMaxY->setText(QString::number(m_maxY + margin, 'f', 2));
    }
    updateLegendValues();
}

void PlotterWindow::addCurveToMenu(PlotConfig& cfg) {
    if (!m_curvesMenu) return;

    QPixmap pixmap(16, 16);
    pixmap.fill(cfg.series->color());
    QIcon icon(pixmap);

    QMenu* curveMenu = m_curvesMenu->addMenu(icon, cfg.series->name());
    
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
        for (auto& plot : m_activePlots) {
            if (plot.series == targetSeries) {
                plot.discrete = checked;
                break;
            }
        }
    });
}

void PlotterWindow::removeCurve(QLineSeries* series) {
    if (!series) return;
    
    for (int i = 0; i < m_activePlots.size(); ++i) {
        if (m_activePlots[i].series == series) {
            m_chart->removeSeries(series);
            m_activePlots.removeAt(i);
            delete series;
            
            // Recalculate min/max if autoscale is on
            if (m_autoScale) {
                m_minY = 1e9;
                m_maxY = -1e9;
                bool hasPoints = false;
                for (const auto& plot : qAsConst(m_activePlots)) {
                    for (int j = 0; j < plot.series->count(); ++j) {
                        double y = plot.series->at(j).y();
                        if (y < m_minY) m_minY = y;
                        if (y > m_maxY) m_maxY = y;
                        hasPoints = true;
                    }
                }
                if (hasPoints && m_minY <= m_maxY) {
                    double margin = (m_maxY - m_minY) * 0.1;
                    if (margin == 0) margin = 1.0;
                    m_axisY->setRange(m_minY - margin, m_maxY + margin);
                    m_edtMinY->setText(QString::number(m_minY - margin, 'f', 2));
                    m_edtMaxY->setText(QString::number(m_maxY + margin, 'f', 2));
                }
            }
            
            QTimer::singleShot(10, this, &PlotterWindow::updateLegendPosition);
            break;
        }
    }
}













void PlotterWindow::updateLegendPosition() {
    if (!m_chart || !m_legendOverlay) return;
    
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
            rowLayout->setContentsMargins(5, 2, 5, 2);
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
    QChartView* view = qobject_cast<QChartView*>(m_legendOverlay->parentWidget());
    if (view) {
        int x = view->width() - m_legendOverlay->width() - 15;
        int y = 15;
        m_legendOverlay->move(x, y);
    }
}

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
    QChartView* view = qobject_cast<QChartView*>(m_legendOverlay->parentWidget());
    if (view) {
        int x = view->width() - m_legendOverlay->width() - 15;
        int y = 15;
        m_legendOverlay->move(x, y);
    }
}

void PlotterWindow::resizeEvent(QResizeEvent *event) {
    QMainWindow::resizeEvent(event);
    updateLegendPosition();
}

void PlotterWindow::onLineThicknessChanged(int val) {
    for (auto& plot : m_activePlots) {
        QPen p = plot.series->pen();
        p.setWidth(val);
        plot.series->setPen(p);
    }
}
