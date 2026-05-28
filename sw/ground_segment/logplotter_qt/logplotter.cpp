#include <QPair>
#include <QProcess>
#include <QPainter>
#include <QMessageBox>
#include <QProgressDialog>
#include <QApplication>
#include <QMainWindow>
#include <QGraphicsLayout>
#include <QChart>
#include <QChartView>
#include <QLineSeries>
#include <QValueAxis>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QFileDialog>
#include <QSplitter>
#include <QDebug>
#include <QShortcut>
#include <QWheelEvent>
#include <QFile>
#include <QTextStream>
#include <QRegularExpression>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QDialog>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QDialogButtonBox>
#include <QDir>
#include <QCheckBox>
#include <QLineEdit>
#include <QSlider>
#include <QLabel>
#include <QSpinBox>
#include <QTimer>
#include <QFileInfo>

#include <QProxyStyle>
#include <QCommandLineParser>
#include <QCommandLineOption>
#include <QPalette>
#include <QColor>

// Include the widgets you want to target for color changes
#include <QLineEdit>
#include <QTextEdit>
#include <QPlainTextEdit>
#include <QAbstractSpinBox> // Covers QSpinBox and QDoubleSpinBox

#include <unistd.h>
#include <fcntl.h>
#include <functional>
#include "../linux_desktop_utils.h"
#include "shared_plot.h"

#include "pprzlinkQt/MessageDictionary.h"//TODO: should not be needed
#include "pprzlinkQt/MessageDefinition.h"//TODO: should not be needed

// Helper class to temporarily suppress stderr warnings (like GTK Wayland criticals) during native dialogs.
//
// WHY WE NEED THIS: 
// When Qt runs natively on Wayland and opens a native file dialog (QFileDialog), 
// the underlying GTK portal throws verbose `Gdk-CRITICAL` assertion errors into 
// the console because it misunderstands the Wayland window handles provided by Qt.
// 
// WHY THIS APPROACH:
// Instead of hacking global environment variables (like forcing GDK_BACKEND=x11) 
// which could break Wayland integration elsewhere, we simply use this RAII class 
// to mute 'stderr' (routing it to /dev/null) precisely while the dialog is open. 
// This keeps our Qt/Wayland environment perfectly clean while hiding the GTK spam.
class StderrBlocker {
    int oldStderr;
    int devNull;
public:
    StderrBlocker() {
        fflush(stderr);
        oldStderr = dup(STDERR_FILENO);
        devNull = open("/dev/null", O_WRONLY);
        if (devNull >= 0) dup2(devNull, STDERR_FILENO);
    }
    ~StderrBlocker() {
        fflush(stderr);
        if (devNull >= 0) {
            dup2(oldStderr, STDERR_FILENO);
            close(devNull);
        }
        close(oldStderr);
    }
};

// The default background of the edit fields in a Dark theme is too dark, almost not visible it is an entry box
// We tweak those fields to be a lighter gray just for better visibility in dark mode, without affecting the rest of the UI which is already dark themed and looks fine
class EditorLighteningStyle : public QProxyStyle { //TODO: Move to common header linux_desktop_utils.h since we like this elsewhere also
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
            
            // Change the Base color to your lighter dark-mode gray
            customPalette.setColor(QPalette::Base, QColor("#3a3a3a"));
            
            // Apply it ONLY to this specific widget
            widget->setPalette(customPalette);
        }
    }
};

class ChartViewFilter : public QObject {
    QChart* m_chart;
    std::function<void()> m_onZoom;
public:
    ChartViewFilter(QChart* chart, std::function<void()> onZoom, QObject* parent = nullptr) : QObject(parent), m_chart(chart), m_onZoom(onZoom) {}
    bool eventFilter(QObject *obj, QEvent *event) override {
        if (event->type() == QEvent::Wheel) {
            QWheelEvent *wheelEvent = static_cast<QWheelEvent*>(event);
            qreal factor = wheelEvent->angleDelta().y() > 0 ? 1.2 : 1.0 / 1.2;
            m_chart->zoom(factor);
            if (m_onZoom) m_onZoom();
            return true;
        } else if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::RightButton) {
                m_chart->zoomReset();
                return true;
            }
        } else if (event->type() == QEvent::MouseButtonRelease) {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::LeftButton) {
                if (m_onZoom) m_onZoom();
            }
        }
        return QObject::eventFilter(obj, event);
    }
};

class LogPlotterWindow : public QMainWindow {
    Q_OBJECT

public:
    LogPlotterWindow(QWidget *parent = nullptr) : QMainWindow(parent) {
        setWindowTitle("Log Plotter");
        resize(900, 300);
        QString xmlPath = QDir::homePath() + "/paparazzi/var/messages.xml";
        try { m_dict = new pprzlink::MessageDictionary(xmlPath); } catch (...) { m_dict = nullptr; }
        setupUI();
    }

private slots:
    void saveScreenshot() {
        if (!m_chartView) return;
        
        QString defaultPath = QDir::homePath() + "/paparazzi/var/logs/screenshot.png";
        if (!m_currentLogFile.isEmpty()) {
            QFileInfo fi(m_currentLogFile);
            defaultPath = fi.path() + "/" + fi.completeBaseName() + ".png";
        }

        QString fileName;
        {
            StderrBlocker blocker;
            QFileDialog dialog(this, tr("Save Screenshot"), defaultPath);
            dialog.setAcceptMode(QFileDialog::AcceptSave);
            dialog.setNameFilters({
                tr("PNG Image (*.png)"),
                tr("JPEG Image (*.jpg)"),
                tr("WebP Image (*.webp)"),
                tr("BMP Image (*.bmp)")
            });
            dialog.setDefaultSuffix("png");
            
            // Updates the default suffix whenever a new filter is selected from the combobox
            connect(&dialog, &QFileDialog::filterSelected, &dialog, [&dialog](const QString& filter) {
                if (filter.contains("*.png")) dialog.setDefaultSuffix("png");
                else if (filter.contains("*.jpg")) dialog.setDefaultSuffix("jpg");
                else if (filter.contains("*.webp")) dialog.setDefaultSuffix("webp");
                else if (filter.contains("*.bmp")) dialog.setDefaultSuffix("bmp");
            });

            if (dialog.exec() == QDialog::Accepted) {
                fileName = dialog.selectedFiles().first();
            }
        }

        if (!fileName.isEmpty()) {
            QPixmap pixmap(m_chartView->size() * 2);
            pixmap.fill(Qt::transparent);
            QPainter painter(&pixmap);
            painter.setRenderHint(QPainter::Antialiasing);
            painter.setRenderHint(QPainter::TextAntialiasing);
            m_chartView->scene()->render(&painter, QRectF(pixmap.rect()), m_chartView->sceneRect());
            painter.end();
            if (!pixmap.save(fileName)) {
                QMessageBox::warning(this, tr("Error"), tr("Failed to save screenshot to %1").arg(fileName));
            } else {
                //qDebug() << "Screenshot saved to" << fileName;
            }
        }
    }

    void onAutoScaleToggled(bool checked) {
        m_edtMinY->setEnabled(!checked);
        m_edtMaxY->setEnabled(!checked);
        if (checked) {
            autoRescaleAxes();
        } else {
            onManualScaleChanged();
        }
    }

    void onManualScaleChanged() {
        if (!m_cbAutoScale->isChecked()) {
            bool okMin = false, okMax = false;
            double minY = m_edtMinY->text().toDouble(&okMin);
            double maxY = m_edtMaxY->text().toDouble(&okMax);
            if (okMin && okMax && minY < maxY) {
                m_axisY->setRange(minY, maxY);
            }
        }
    }

    void onAddConstantClicked() {
        bool ok = false;
        double val = m_edtConstant->text().toDouble(&ok);
        if (!ok || !std::isfinite(val)) return;

        QLineSeries* series = new QLineSeries();
        series->setName(QString("C=%1").arg(val));

        QPen pen = series->pen();
        pen.setColor(Qt::black);
        pen.setWidth(m_spnLineThickness->value());
        series->setPen(pen);

        double minX = 0;
        double maxX = 100;
        if (m_chart->series().count() > 0) {
            auto axes = m_chart->axes(Qt::Horizontal);
            if (!axes.isEmpty()) {
                QValueAxis *axisX = qobject_cast<QValueAxis*>(axes.first());
                if (axisX) {
                    minX = axisX->min();
                    maxX = axisX->max();
                }
            } else {
                auto existing = m_chart->series().first();
                QLineSeries *ls = qobject_cast<QLineSeries*>(existing);
                if (ls && ls->count() > 0) {
                    minX = ls->at(0).x();
                    maxX = ls->at(ls->count() - 1).x();
                }
            }
        }

        series->append(minX, val);
        series->append(maxX, val);

        m_chart->addSeries(series);
        series->attachAxis(m_axisX);
        series->attachAxis(m_axisY);
        autoRescaleAxes();

        if (m_curvesMenu) {
            QPixmap pixmap(16, 16);
            pixmap.fill(pen.color());
            QIcon icon(pixmap);
            QString title = QString("C=%1").arg(val);
            QAction* deleteAction = m_curvesMenu->addAction(icon, title);
            deleteAction->setToolTip(tr("Delete constant curve"));
            deleteAction->setStatusTip(tr("Delete constant curve"));

            QLineSeries* targetSeries = series;
            connect(deleteAction, &QAction::triggered, this, [this, targetSeries, deleteAction]() {
                m_chart->removeSeries(targetSeries);
                delete targetSeries;
                deleteAction->deleteLater();
                autoRescaleAxes();
                if (m_legendManager) m_legendManager->updateLegendPosition();
                m_chartView->viewport()->update();
            });
        }
        
        QTimer::singleShot(15, this, [this]() {
            if (m_legendManager) m_legendManager->updateLegendPosition();
            m_chartView->viewport()->update();
        });
    }

        void exportFig() {
        QString fileName;
        {
            StderrBlocker blocker;
            fileName = QFileDialog::getSaveFileName(this, tr("Export FIG"), "", tr("FIG Files (*.fig)"));
        }
        if (fileName.isEmpty()) return;

        QFile file(fileName);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(this, tr("Error"), tr("Cannot write to file %1").arg(fileName));
            return;
        }

        QTextStream out(&file);
        out << "#FIG 3.2\n";
        out << "Landscape\n";
        out << "Center\n";
        out << "Metric\n";
        out << "A4\n";
        out << "100.00\n";
        out << "Single\n";
        out << "-2\n";
        out << "1200 2\n";

        double figWidth = 10000.0;
        double figHeight = 6000.0;
        double figOffsetX = 1000.0;
        double figOffsetY = 1000.0;

        double minX = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        double minY = std::numeric_limits<double>::max();
        double maxY = std::numeric_limits<double>::lowest();

        QList<QAbstractSeries*> seriesList = m_chart->series();
        for (auto series : seriesList) {
            QLineSeries* lineSeries = qobject_cast<QLineSeries*>(series);
            if (lineSeries) {
                for (const QPointF& pt : lineSeries->points()) {
                    if (pt.x() < minX) minX = pt.x();
                    if (pt.x() > maxX) maxX = pt.x();
                    if (pt.y() < minY) minY = pt.y();
                    if (pt.y() > maxY) maxY = pt.y();
                }
            }
        }

        if (minX >= maxX) { minX = 0; maxX = 1; }
        if (minY >= maxY) { minY = 0; maxY = 1; }

        double scaleX = figWidth / (maxX - minX);
        double scaleY = figHeight / (maxY - minY);

        // draw axes box
        out << "2 1 0 1 0 0 51 -1 -1 0.000 0 0 -1 0 0 5\n";
        out << "\t " << (int)figOffsetX << " " << (int)figOffsetY 
            << " " << (int)(figOffsetX + figWidth) << " " << (int)figOffsetY 
            << " " << (int)(figOffsetX + figWidth) << " " << (int)(figOffsetY + figHeight) 
            << " " << (int)figOffsetX << " " << (int)(figOffsetY + figHeight) 
            << " " << (int)figOffsetX << " " << (int)figOffsetY << "\n";

        int colorIndex = 1;
        for (auto series : seriesList) {
            QLineSeries* lineSeries = qobject_cast<QLineSeries*>(series);
            if (lineSeries && lineSeries->count() > 0) {
                int npoints = lineSeries->count();
                out << "2 1 0 1 " << (colorIndex % 32) << " 0 50 -1 -1 0.000 0 0 -1 0 0 " << npoints << "\n\t";
                int count = 0;
                for (const QPointF& pt : lineSeries->points()) {
                    int fx = static_cast<int>(figOffsetX + (pt.x() - minX) * scaleX);
                    int fy = static_cast<int>(figOffsetY + figHeight - (pt.y() - minY) * scaleY);
                    out << " " << fx << " " << fy;
                    if (++count >= 10) {
                        out << "\n\t";
                        count = 0;
                    }
                }
                out << "\n";
                colorIndex++;
            }
        }
    }

    void closeLogFile() {
        if (m_chart) {
            m_chart->removeAllSeries();
        }
        if (m_curvesMenu) {
            m_curvesMenu->clear();
        }
        for (QMenu* menu : m_logMenus) {
            if (menu) {
                delete menu;
            }
        }
        m_logMenus.clear();
        m_currentLogFile.clear();

        if (m_axisX) m_axisX->hide();
        if (m_axisY) m_axisY->hide();

        if (m_legendManager) m_legendManager->updateLegendPosition();
        if (m_chartView && m_chartView->viewport()) m_chartView->viewport()->update();
    }

    void openLogFile() {
        QString fileName;
        {
            StderrBlocker blocker;
            fileName = QFileDialog::getOpenFileName(this, "Open Paparazzi Log", QDir::homePath() + "/paparazzi/var/logs", "Log Files (*.log);;All Files (*)");
        }
        if (!fileName.isEmpty()) {
            loadLogFile(fileName);
        }
    }

protected:
    void resizeEvent(QResizeEvent *event) override {
        QMainWindow::resizeEvent(event);
        if (m_legendManager) {
            m_legendManager->triggerRelayout();
        }
    }

private:
    QChart *m_chart;
    QChartView *m_chartView;
    QValueAxis *m_axisX;
    QValueAxis *m_axisY;
    QString m_currentLogFile;
    QString m_originallyLoadedFile;
    ChartLegendManager* m_legendManager;
    pprzlink::MessageDictionary *m_dict;
    QMenu *m_curvesMenu;
    QCheckBox* m_cbAutoScale;
    QLineEdit* m_edtMinY;
    QLineEdit* m_edtMaxY;
    QLineEdit* m_edtConstant;
    QLineEdit* m_edtScaleNext;
    QSpinBox* m_spnLineThickness;
    QTimer* m_updateTimer;
    QList<QMenu*> m_logMenus;

    void setupMenu() {
        QMenu* fileMenu = menuBar()->addMenu(tr("&File"));
        
        QAction* openAction = fileMenu->addAction(tr("Open Log"));
        openAction->setShortcut(QKeySequence("Ctrl+O"));
        connect(openAction, &QAction::triggered, this, &LogPlotterWindow::openLogFile);

        QAction* newAction = fileMenu->addAction(tr("New"));
        newAction->setShortcut(QKeySequence("Ctrl+N"));
        connect(newAction, &QAction::triggered, this, [this]() {
            QStringList args;
            if (!m_originallyLoadedFile.isEmpty()) {
                args << m_originallyLoadedFile;
            }
            
            // Start detached process for maximum robustness and memory isolation.
            // This ensures huge log files don't share identical process memory or block the current UI thread.
            bool processStarted = QProcess::startDetached(QCoreApplication::applicationFilePath(), args);
            
            if (!processStarted) {
                // Elegant fallback to in-process spawn if binary launching is unexpectedly restricted.
                LogPlotterWindow* newWindow = new LogPlotterWindow();
                if (!m_originallyLoadedFile.isEmpty()) {
                    newWindow->loadLogFile(m_originallyLoadedFile);
                }
                newWindow->show();
            }
        });

        QAction* exportFigAction = fileMenu->addAction(tr("Export Fig"));
        exportFigAction->setShortcut(QKeySequence("Ctrl+E"));
        connect(exportFigAction, &QAction::triggered, this, &LogPlotterWindow::exportFig);

        QAction* saveAction = fileMenu->addAction(tr("Save screenshot"));
        saveAction->setShortcut(QKeySequence("Ctrl+S"));
        connect(saveAction, &QAction::triggered, this, &LogPlotterWindow::saveScreenshot);

        QAction* closeAction = fileMenu->addAction(tr("Close"));
        closeAction->setShortcut(QKeySequence("Ctrl+W"));
        connect(closeAction, &QAction::triggered, this, &LogPlotterWindow::closeLogFile);

        fileMenu->addSeparator();

        QAction* quitAction = fileMenu->addAction(tr("Quit"));
        quitAction->setShortcut(QKeySequence("Ctrl+Q"));
        connect(quitAction, &QAction::triggered, qApp, &QApplication::quit);

        m_curvesMenu = menuBar()->addMenu(tr("&Curves"));
        m_curvesMenu->setToolTipsVisible(true);
    }

    void setupUI() {
        setupMenu();
        
        QWidget *mainWidget = new QWidget(this);
        setCentralWidget(mainWidget);
        QVBoxLayout *mainLayout = new QVBoxLayout(mainWidget);
        mainLayout->setContentsMargins(0, 0, 0, 0);
        mainLayout->setSpacing(0);


        
        QWidget *toolbarWidget = new QWidget();
        QHBoxLayout *toolbarLayout = new QHBoxLayout(toolbarWidget);
        toolbarLayout->setContentsMargins(2, 2, 2, 2);

        m_cbAutoScale = new QCheckBox("Auto Scale");
        m_cbAutoScale->setChecked(true);

        m_edtMinY = new QLineEdit();
        m_edtMaxY = new QLineEdit();
        m_edtMinY->setMaximumWidth(90);
        m_edtMaxY->setMaximumWidth(90);
        m_edtMinY->setEnabled(false);
        m_edtMaxY->setEnabled(false);

        QLabel *lblConst = new QLabel("Constant");
        m_edtConstant = new QLineEdit();
        m_edtConstant->setMaximumWidth(75);

        QLabel *lblScaleNext = new QLabel("Scale next by");
        m_edtScaleNext = new QLineEdit("1.0");
        m_edtScaleNext->setMaximumWidth(75);

        m_spnLineThickness = new QSpinBox();
        m_spnLineThickness->setToolTip("Line Thickness (px)");
        m_spnLineThickness->setRange(1, 10);
        m_spnLineThickness->setValue(1);
        m_spnLineThickness->hide();

        m_updateTimer = new QTimer(this);

        toolbarLayout->addWidget(m_cbAutoScale);
        connect(m_cbAutoScale, &QCheckBox::toggled, this, &LogPlotterWindow::onAutoScaleToggled);
        connect(m_edtMinY, &QLineEdit::editingFinished, this, &LogPlotterWindow::onManualScaleChanged);
        connect(m_edtMaxY, &QLineEdit::editingFinished, this, &LogPlotterWindow::onManualScaleChanged);
        toolbarLayout->addWidget(new QLabel("Min"));
        toolbarLayout->addWidget(m_edtMinY);
        toolbarLayout->addWidget(new QLabel("Max"));
        toolbarLayout->addWidget(m_edtMaxY);

        toolbarLayout->addWidget(lblConst);
        toolbarLayout->addWidget(m_edtConstant);
        connect(m_edtConstant, &QLineEdit::editingFinished, this, &LogPlotterWindow::onAddConstantClicked);
        toolbarLayout->addWidget(lblScaleNext);
        toolbarLayout->addWidget(m_edtScaleNext);
        QLabel* lblLineThickness = new QLabel("Line:");
        lblLineThickness->hide();
        toolbarLayout->addWidget(lblLineThickness);
        toolbarLayout->addWidget(m_spnLineThickness);
        toolbarLayout->addStretch();

        m_chartView = new QChartView();
        m_chartView->setContentsMargins(0, 0, 0, 0);
        m_chartView->setFrameShape(QFrame::NoFrame);
        m_chartView->setRubberBand(QChartView::RectangleRubberBand);

        // Zooming and Panning Shortcuts
        QShortcut *zoomInSc = new QShortcut(QKeySequence(Qt::Key_Plus), this);
        connect(zoomInSc, &QShortcut::activated, this, [this]() { if(m_chart) { m_chart->zoomIn(); m_cbAutoScale->setChecked(false); } });
        QShortcut *zoomOutSc = new QShortcut(QKeySequence(Qt::Key_Minus), this);
        connect(zoomOutSc, &QShortcut::activated, this, [this]() { if(m_chart) { m_chart->zoomOut(); m_cbAutoScale->setChecked(false); } });
        QShortcut *zoomResetSc = new QShortcut(QKeySequence(Qt::Key_0), this);
        connect(zoomResetSc, &QShortcut::activated, this, [this]() { if(m_chart) m_chart->zoomReset(); });

        QShortcut *panLeftSc = new QShortcut(QKeySequence(Qt::Key_Left), this);
        connect(panLeftSc, &QShortcut::activated, this, [this]() { if(m_chart) { m_chart->scroll(-50, 0); m_cbAutoScale->setChecked(false); } });
        QShortcut *panRightSc = new QShortcut(QKeySequence(Qt::Key_Right), this);
        connect(panRightSc, &QShortcut::activated, this, [this]() { if(m_chart) { m_chart->scroll(50, 0); m_cbAutoScale->setChecked(false); } });
        QShortcut *panUpSc = new QShortcut(QKeySequence(Qt::Key_Up), this);
        connect(panUpSc, &QShortcut::activated, this, [this]() { if(m_chart) { m_chart->scroll(0, 50); m_cbAutoScale->setChecked(false); } });
        QShortcut *panDownSc = new QShortcut(QKeySequence(Qt::Key_Down), this);
        connect(panDownSc, &QShortcut::activated, this, [this]() { if(m_chart) { m_chart->scroll(0, -50); m_cbAutoScale->setChecked(false); } });
        
        m_chart = new QChart();
        m_chart->setMargins(QMargins(0, 0, 0, 0));
        m_chart->layout()->setContentsMargins(0, 0, 0, 0);
        m_chart->setBackgroundRoundness(0);
        m_chart->setBackgroundPen(QPen(Qt::NoPen));
        m_chartView->setChart(m_chart);
        m_chartView->setRenderHint(QPainter::Antialiasing);
        m_chartView->viewport()->installEventFilter(new ChartViewFilter(m_chart, [this](){ m_cbAutoScale->setChecked(false); }, m_chartView));
        
        m_axisX = new QValueAxis();
        m_axisX->setLabelFormat("%gs");
        m_axisY = new QValueAxis();
        
        m_axisX->hide();
        m_axisY->hide();

        m_chart->addAxis(m_axisX, Qt::AlignBottom);
        m_chart->addAxis(m_axisY, Qt::AlignLeft);
        
        mainLayout->addWidget(toolbarWidget);
        mainLayout->addWidget(m_chartView);
        
        m_legendManager = new ChartLegendManager(m_chart, m_chartView);
    }

    void autoRescaleAxes() {
        double calcMinX = 1e9, calcMaxX = -1e9;
        double calcMinY = 1e9, calcMaxY = -1e9;
        bool hasData = false;

        for (auto* s : m_chart->series()) {
            QLineSeries* ls = qobject_cast<QLineSeries*>(s);
            if (ls && ls->count() > 0) {
                hasData = true;
                const auto& pts = ls->points();
                for (const QPointF& p : pts) {
                    if (p.x() < calcMinX) calcMinX = p.x();
                    if (p.x() > calcMaxX) calcMaxX = p.x();
                    if (p.y() < calcMinY) calcMinY = p.y();
                    if (p.y() > calcMaxY) calcMaxY = p.y();
                }
            }
        }
        if (hasData) {
            if (calcMinX == calcMaxX) { calcMinX -= 1; calcMaxX += 1; }
            if (calcMinY == calcMaxY) { calcMinY -= 1; calcMaxY += 1; }
            double marginY = (calcMaxY - calcMinY) * 0.05;
            
            m_axisX->setRange(calcMinX, calcMaxX);
            
            if (m_cbAutoScale->isChecked()) {
                m_axisY->setRange(calcMinY - marginY, calcMaxY + marginY);
                m_edtMinY->setText(QString::number(calcMinY - marginY, 'f', 2));
                m_edtMaxY->setText(QString::number(calcMaxY + marginY, 'f', 2));
            } else {
                bool okMin = false, okMax = false;
                double minY = m_edtMinY->text().toDouble(&okMin);
                double maxY = m_edtMaxY->text().toDouble(&okMax);
                if (okMin && okMax && minY < maxY) {
                    m_axisY->setRange(minY, maxY);
                }
            }
            
            m_axisX->show();
            m_axisY->show();
        } else {
            m_axisX->hide();
            m_axisY->hide();
        }
    }

    void addCurve(const QString& acId, const QString& msgName, const QString& fieldName, int fieldIndex) {
        if (m_currentLogFile.isEmpty()) return;
        QFile file(m_currentLogFile);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;

        bool okScale = false;
        double scale = m_edtScaleNext->text().toDouble(&okScale);
        if (!okScale) scale = 1.0;

        bool okTranspose = false;
        double transpose = m_edtConstant->text().toDouble(&okTranspose);
        if (!okTranspose) transpose = 0.0;

        QFileInfo fi(m_currentLogFile);
        QString logName = fi.baseName();
        QString logNameCondensed = logName;
        logNameCondensed.replace("__", "#TEMP#");
        logNameCondensed.replace("_", "");
        logNameCondensed.replace("#TEMP#", "_");

        QString curveTitle = logName + ":" + acId + ":" + msgName + ":" + fieldName + ":" + QString::number(scale) + "+" + QString::number(transpose);
        QString curveTitleCondensed = logNameCondensed + ":" + acId + ":" + msgName + ":" + fieldName + ":" + QString::number(scale) + "+" + QString::number(transpose);

        QLineSeries *series = new QLineSeries();
        series->setName(curveTitle);

        QTextStream in(&file);
        QRegularExpression re("\\s+");
        while (!in.atEnd()) {
            QString line = in.readLine();
            QStringList parts = line.split(re, Qt::SkipEmptyParts);
            if (parts.size() > 3 + fieldIndex) {
                if (parts[1] == acId && parts[2] == msgName) {
                    bool okTime, okVal;
                    double t = parts[0].toDouble(&okTime);
                    double v = parts[3 + fieldIndex].toDouble(&okVal);
                    if (okTime && okVal) {
                        series->append(t, v * scale + transpose);
                    }
                }
            }
        }
        
        if (series->count() > 0) {
            QPen pen = series->pen();
            pen.setColor(getNextSaturatedColor());
            pen.setWidth(m_spnLineThickness->value());
            series->setPen(pen);
            
            m_chart->addSeries(series);
            series->attachAxis(m_axisX);
            series->attachAxis(m_axisY);
            autoRescaleAxes();
            
            if (m_curvesMenu) {
                QPixmap pixmap(16, 16);
                pixmap.fill(pen.color());
                QIcon icon(pixmap);
                QAction* deleteAction = m_curvesMenu->addAction(icon, curveTitleCondensed);
                deleteAction->setToolTip(tr("Delete curve"));
                deleteAction->setStatusTip(tr("Delete curve"));
                
                QLineSeries* targetSeries = series;
                connect(deleteAction, &QAction::triggered, this, [this, targetSeries, deleteAction]() {
                    m_chart->removeSeries(targetSeries);
                    delete targetSeries;
                    deleteAction->deleteLater();
                    autoRescaleAxes();
                    if (m_legendManager) m_legendManager->updateLegendPosition();
                    m_chartView->viewport()->update();
                });
            }
            
            QTimer::singleShot(15, this, [this]() {
                if (m_legendManager) m_legendManager->updateLegendPosition();
                m_chartView->viewport()->update();
            });
        } else {
            delete series;
        }
    }

public:
    void loadLogFile(const QString &fileName) {
        QApplication::setOverrideCursor(Qt::WaitCursor);
        
        m_originallyLoadedFile = fileName;
        QString dataFileName = fileName;
        QMap<QString, QString> acIdToName;
        QMap<QString, QStringList> dictFields;

        if (fileName.endsWith(".log", Qt::CaseInsensitive)) {
            QFile logFile(fileName);
            if (logFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
                QString content = logFile.readAll();
                
                QRegularExpression reDataFile("<configuration[^>]*data_file=\"([^\"]+)\"");
                QRegularExpressionMatch matchDataFile = reDataFile.match(content);
                if (matchDataFile.hasMatch()) {
                    QString dFile = matchDataFile.captured(1);
                    QFileInfo fi(fileName);
                    dataFileName = fi.absolutePath() + "/" + dFile;
                }

                QRegularExpression reAircraft("<aircraft([^>]+)>");
                QRegularExpressionMatchIterator itAc = reAircraft.globalMatch(content);
                while (itAc.hasNext()) {
                    QString attrs = itAc.next().captured(1);
                    QRegularExpression reName("name=\"([^\"]+)\"");
                    QRegularExpression reId("ac_id=\"([^\"]+)\"");
                    QString acName = reName.match(attrs).captured(1);
                    QString acId = reId.match(attrs).captured(1);
                    if (!acId.isEmpty()) {
                        acIdToName[acId] = acName;
                    }
                }

                int protoStart = content.indexOf("<protocol>");
                int protoEnd = content.indexOf("</protocol>", protoStart);
                if (protoStart != -1 && protoEnd != -1) {
                    //printf("Found protocol from %d to %d\n", protoStart, protoEnd); fflush(stdout);//Enable for Debug only
                    QString protocolXml = content.mid(protoStart, protoEnd - protoStart + 11);
                    QXmlStreamReader xml(protocolXml);
                    while (!xml.atEnd() && !xml.hasError()) {
                        QXmlStreamReader::TokenType token = xml.readNext();
                        if (token == QXmlStreamReader::StartElement) {
                            if (xml.name().toString() == "message" && xml.attributes().hasAttribute("NAME")) {
                                QString msgName = xml.attributes().value("NAME").toString();
                                QStringList fields;
                                while (!(xml.tokenType() == QXmlStreamReader::EndElement && xml.name().toString() == "message") && !xml.atEnd()) {
                                    xml.readNext();
                                    if (xml.tokenType() == QXmlStreamReader::StartElement && xml.name().toString() == "field") {
                                        if (xml.attributes().hasAttribute("NAME")) {
                                            fields.append(xml.attributes().value("NAME").toString());
                                        }
                                    }
                                }
                                dictFields[msgName] = fields;
                            //printf("Extracted msg: %s with %lld fields\n", msgName.toStdString().c_str(), fields.size()); fflush(stdout);//Enable for Debug only
                            }
                        }
                    }
                }
            }
        }

        if (!dataFileName.isEmpty() && m_currentLogFile == dataFileName) {
            while (QApplication::overrideCursor()) QApplication::restoreOverrideCursor();
            return;
        }

        m_currentLogFile = dataFileName;
        QFile file(dataFileName);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            while (QApplication::overrideCursor()) QApplication::restoreOverrideCursor();
            QMessageBox::warning(this, "Error", "Cannot open file " + dataFileName);
            return;
        }

        qint64 totalSize = file.size();
        QProgressDialog progress(tr("Parsing log file..."), tr("Cancel"), 0, totalSize > 0 ? totalSize : 1, this);
        progress.setWindowModality(Qt::WindowModal);
        progress.setMinimumDuration(200);

        QSet<QPair<QString, QString>> acMsgPairs;
        
        // Fast parsing of the data file
        const int CHUNK_SIZE = 1024 * 1024; //Adjust chunk size as you deem fit for performance/memory balance
        QByteArray buffer;
        while (!file.atEnd()) {
            if (progress.wasCanceled()) {
                m_currentLogFile.clear();
                while (QApplication::overrideCursor()) QApplication::restoreOverrideCursor();
                return;
            }
            buffer.append(file.read(CHUNK_SIZE));
            progress.setValue(file.pos());
            QCoreApplication::processEvents();
            
            int lineStart = 0;
            int nlIdx = 0;
            
            while ((nlIdx = buffer.indexOf('\n', lineStart)) != -1) {
                int lineLen = nlIdx - lineStart;
                if (lineLen > 0) {
                    const char* lineData = buffer.constData() + lineStart;
                    
                    int s1 = -1, len1 = 0;
                    int s2 = -1, len2 = 0;
                    int s3 = -1, len3 = 0;
                    
                    for (int i = 0; i < lineLen; ++i) {
                        if (lineData[i] != ' ' && lineData[i] != '\t' && lineData[i] != '\r') {
                            if (s1 == -1) { s1 = i; }
                            else if (len1 > 0 && s2 == -1) { s2 = i; }
                            else if (len2 > 0 && s3 == -1) { s3 = i; }
                        } else {
                            if (s1 != -1 && s2 == -1) { len1 = i - s1; }
                            else if (s2 != -1 && s3 == -1) { len2 = i - s2; }
                            else if (s3 != -1 && len3 == 0) { len3 = i - s3; break; }
                        }
                    }
                    if (s3 != -1 && len3 == 0) {
                         len3 = lineLen - s3;
                    }
                    
                    if (s2 != -1 && len2 > 0 && s3 != -1 && len3 > 0) {
                        QString acId = QString::fromUtf8(lineData + s2, len2);
                        QString msgName = QString::fromUtf8(lineData + s3, len3);
                        acMsgPairs.insert(qMakePair(acId, msgName));
                    }
                }
                lineStart = nlIdx + 1;
            }
            buffer.remove(0, lineStart);
        }

        QFileInfo fi(fileName);
        QString logName = fi.baseName();
        
        QMap<QString, QSet<QString>> acToMsgs;
        for (const auto& pair : acMsgPairs) {
            acToMsgs[pair.first].insert(pair.second);
        }

        for (auto it = acToMsgs.begin(); it != acToMsgs.end(); ++it) {
            QString acId = it.key();
            //QString acNameDisplay = acIdToName.value(acId, "AC_" + acId);
            //QString menuTitle = logName + ":" + acNameDisplay + " (" + acId + ")";
            QString menuTitle = logName + ":" + acId ;
            QMenu* acMenu = menuBar()->addMenu(menuTitle);
            m_logMenus.append(acMenu);
            
            QStringList msgs = it.value().values();
            msgs.sort(); // Sorting messages alphabetically
            
            for (const QString& msgName : msgs) {
                
                if (dictFields.contains(msgName)) {
                    //printf("Found MSG in dict: %s\n", msgName.toStdString().c_str()); fflush(stdout);//Enable for Debug only
                    QMenu* msgMenu = acMenu->addMenu(msgName);
                    const QStringList& fields = dictFields.value(msgName);
                    for (int i = 0; i < fields.size(); ++i) {
                        QString fieldName = fields.at(i);
                                                QAction* fieldAction = msgMenu->addAction(fieldName);
                        connect(fieldAction, &QAction::triggered, this, [this, acId, msgName, fieldName, i]() {
                            this->addCurve(acId, msgName, fieldName, i);
                        });
                    }
                } else {
                    acMenu->addAction(msgName);
                }
            }

            acMenu->addSeparator();

            QAction* exportKmlAction = acMenu->addAction("Export KML");
            connect(exportKmlAction, &QAction::triggered, this, [this, acId, logName, dictFields]() {
                QString defaultName = QFileInfo(m_currentLogFile).path() + "/" + logName + "_" + acId + ".kml";
                
                QString fileName;
                {
                    StderrBlocker blocker;
                    fileName = QFileDialog::getSaveFileName(this, tr("Export KML"), defaultName, tr("KML Files (*.kml)"));
                }
                if (fileName.isEmpty()) return;
                
                QFile file(m_currentLogFile);
                if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
                    QMessageBox::warning(this, "Export KML", "Cannot open data file.");
                    return;
                }
                
                int latIdx = -1, lonIdx = -1, altIdx = -1;
                int utmEastIdx = -1, utmNorthIdx = -1, utmZoneIdx = -1;
                QString targetMsg;
                bool isUtm = false;
                double latScale = 1.0, lonScale = 1.0, altScale = 1.0;
                
                if (dictFields.contains("GPS")) {
                    targetMsg = "GPS";
                    utmEastIdx = dictFields["GPS"].indexOf("utm_east");
                    utmNorthIdx = dictFields["GPS"].indexOf("utm_north");
                    utmZoneIdx = dictFields["GPS"].indexOf("utm_zone");
                    altIdx = dictFields["GPS"].indexOf("alt");
                    altScale = 1e-3;
                    isUtm = true;
                } else if (dictFields.contains("GPS_INT")) {
                    targetMsg = "GPS_INT";
                    latIdx = dictFields["GPS_INT"].indexOf("lat");
                    lonIdx = dictFields["GPS_INT"].indexOf("lon");
                    altIdx = dictFields["GPS_INT"].indexOf("hmsl");
                    if (altIdx == -1) altIdx = dictFields["GPS_INT"].indexOf("alt");
                    latScale = 1e-7; lonScale = 1e-7; altScale = 1e-3;
                } else if (dictFields.contains("MINIMAL_COM")) {
                    targetMsg = "MINIMAL_COM";
                    latIdx = dictFields["MINIMAL_COM"].indexOf("lat");
                    lonIdx = dictFields["MINIMAL_COM"].indexOf("lon");
                    altIdx = dictFields["MINIMAL_COM"].indexOf("hmsl");
                    if (altIdx == -1) altIdx = dictFields["MINIMAL_COM"].indexOf("alt");
                } else {
                    // Try to dynamically figure it out from file scanning if not correctly in dict
                    QFile checkFile(m_currentLogFile);
                    if (checkFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
                        QTextStream checkIn(&checkFile);
                        QRegularExpression checkRe("\\s+");
                        while (!checkIn.atEnd()) {
                            QString line = checkIn.readLine();
                            QStringList parts = line.split(checkRe, Qt::SkipEmptyParts);
                            if (parts.size() > 3 && parts[1] == acId) {
                                if (parts[2] == "GPS") { targetMsg = "GPS"; break; }
                                else if (parts[2] == "GPS_INT") { targetMsg = "GPS_INT"; break; }
                                else if (parts[2] == "MINIMAL_COM") { targetMsg = "MINIMAL_COM"; break; }
                            }
                        }
                    }
                    if (targetMsg == "GPS" && dictFields.contains("GPS")) {
                        utmEastIdx = dictFields["GPS"].indexOf("utm_east");
                        utmNorthIdx = dictFields["GPS"].indexOf("utm_north");
                        utmZoneIdx = dictFields["GPS"].indexOf("utm_zone");
                        altIdx = dictFields["GPS"].indexOf("alt");
                        altScale = 1e-3;
                        isUtm = true;
                    }
                }
                
                if (targetMsg.isEmpty() || (!isUtm && (latIdx == -1 || lonIdx == -1)) || (isUtm && (utmEastIdx == -1 || utmNorthIdx == -1 || utmZoneIdx == -1))) {
                    QMessageBox::warning(this, "Export KML", "Could not find valid GPS coordinates in the log for this AC.");
                    return;
                }
                
                QTextStream in(&file);
                QRegularExpression re("\\s+");
                QString kmlCoords;
                
                auto utm2deg = [](double x, double y, int zone, double& lat, double& lon) {
                    double a = 6378137.0;
                    double eccSquared = 0.00669438000426224;
                    double k0 = 0.9996;
                    double eccPrimeSquared = eccSquared / (1.0 - eccSquared);
                    x = x / (k0 * a);
                    y = y / k0;
                    double m = y / a;
                    double mu = m / (1.0 - eccSquared / 4.0 - 3.0 * eccSquared * eccSquared / 64.0 - 5.0 * pow(eccSquared, 3) / 256.0);
                    double e1 = (1.0 - sqrt(1.0 - eccSquared)) / (1.0 + sqrt(1.0 - eccSquared));
                    
                    double phi1Rad = mu + (3.0 * e1 / 2.0 - 27.0 * pow(e1, 3) / 32.0) * sin(2.0 * mu)
                                     + (21.0 * e1 * e1 / 16.0 - 55.0 * pow(e1, 4) / 32.0) * sin(4.0 * mu)
                                     + (151.0 * pow(e1, 3) / 96.0) * sin(6.0 * mu);
                    double N1 = a / sqrt(1.0 - eccSquared * pow(sin(phi1Rad), 2));
                    double T1 = pow(tan(phi1Rad), 2);
                    double C1 = eccPrimeSquared * pow(cos(phi1Rad), 2);
                    double R1 = a * (1.0 - eccSquared) / pow(1.0 - eccSquared * pow(sin(phi1Rad), 2), 1.5);
                    double D = x;
                    double LongOrigin = (zone - 1) * 6 - 180 + 3;
                    
                    lat = phi1Rad - (N1 * tan(phi1Rad) / R1) * (D * D / 2.0 - (5.0 + 3.0 * T1 + 10.0 * C1 - 4.0 * C1 * C1 - 9.0 * eccPrimeSquared) * D * D * D * D / 24.0
                          + (61.0 + 90.0 * T1 + 298.0 * C1 + 45.0 * T1 * T1 - 252.0 * eccPrimeSquared - 3.0 * C1 * C1) * pow(D, 6) / 720.0);
                    lat = lat * 180.0 / M_PI;
                    lon = (D - (1.0 + 2.0 * T1 + C1) * pow(D, 3) / 6.0 + (5.0 - 2.0 * C1 + 28.0 * T1 - 3.0 * C1 * C1 + 8.0 * eccPrimeSquared + 24.0 * T1 * T1)
                          * pow(D, 5) / 120.0) / cos(phi1Rad);
                    lon = LongOrigin + lon * 180.0 / M_PI;
                };

                while (!in.atEnd()) {
                    QString line = in.readLine();
                    QStringList parts = line.split(re, Qt::SkipEmptyParts);
                    if (parts.size() > 3 && parts[1] == acId && parts[2] == targetMsg) {
                        if (!isUtm && parts.size() > 3 + std::max({latIdx, lonIdx, altIdx})) {
                            double lat = parts[3 + latIdx].toDouble() * latScale;
                            double lon = parts[3 + lonIdx].toDouble() * lonScale;
                            double alt = altIdx != -1 ? parts[3 + altIdx].toDouble() * altScale : 0.0;
                            kmlCoords += QString::number(lon, 'f', 6) + "," + QString::number(lat, 'f', 6) + "," + QString::number(alt, 'f', 6) + " ";
                        } else if (isUtm && parts.size() > 3 + std::max({utmEastIdx, utmNorthIdx, utmZoneIdx, altIdx})) {
                            double utmEast = parts[3 + utmEastIdx].toDouble() / 100.0;
                            double utmNorth = parts[3 + utmNorthIdx].toDouble() / 100.0;
                            int utmZone = parts[3 + utmZoneIdx].toInt();
                            double alt = altIdx != -1 ? parts[3 + altIdx].toDouble() * altScale : 0.0;
                            
                            if (utmZone > 0 && alt > 0) {
                                double lat, lon;
                                utm2deg(utmEast, utmNorth, utmZone, lat, lon);
                                kmlCoords += QString::number(lon, 'f', 6) + "," + QString::number(lat, 'f', 6) + "," + QString::number(alt, 'f', 6) + " ";
                            }
                        }
                    }
                }

                QFile kmlFile(fileName);
                if (kmlFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                    // QColor c = getNextSaturatedColor();
                    // QString kmlColor = QString("%1%2%3%4")
                    //     .arg(c.alpha(), 2, 16, QLatin1Char('0'))
                    //     .arg(c.blue(), 2, 16, QLatin1Char('0'))
                    //     .arg(c.green(), 2, 16, QLatin1Char('0'))
                    //     .arg(c.red(), 2, 16, QLatin1Char('0'));

                    QTextStream out(&kmlFile);
                    out << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
                    out << "  <Document>\n";
                    out << "    <name>" << logName << "_" << acId << "</name>\n";
                    out << "    <Placemark>\n";
                    out << "      <name>" << logName << "</name>\n";
                    out << "      <Style>\n";
                    out << "        <LineStyle>\n";
                    //out << "          <color>" << kmlColor << "</color>\n";//New option to you gusto
                    out << "          <color>ff0000ff</color>\n"; // Red color like in original code
                    out << "          <width>2</width>\n";
                    out << "        </LineStyle>\n";
                    out << "      </Style>\n";
                    out << "      <LineString>\n";
                    out << "        <altitudeMode>absolute</altitudeMode>\n";
                    out << "        <coordinates>\n";
                    out << "          " << kmlCoords << "\n";
                    out << "        </coordinates>\n";
                    out << "      </LineString>\n";
                    out << "    </Placemark>\n";
                    out << "  </Document>\n";
                    out << "</kml>\n";

                }

            });

            QAction* exportCsvAction = acMenu->addAction("Export CSV");
            connect(exportCsvAction, &QAction::triggered, this, [this, acId, logName, dictFields, msgs]() {
                QDialog dialog(this);
                dialog.setWindowTitle(tr("Export CSV - %1").arg(acId));
                dialog.resize(500, 600);
                QVBoxLayout* layout = new QVBoxLayout(&dialog);
                
                QTreeWidget* tree = new QTreeWidget(&dialog);
                tree->setHeaderLabel("Messages and Fields");
                layout->addWidget(tree);
                
                for (const QString& msgName : msgs) {
                    if (dictFields.contains(msgName)) {
                        QTreeWidgetItem* msgItem = new QTreeWidgetItem(tree);
                        msgItem->setText(0, msgName);
                        msgItem->setFlags(msgItem->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsAutoTristate);
                        msgItem->setCheckState(0, Qt::Unchecked);
                        
                        const QStringList& fields = dictFields.value(msgName);
                        for (const QString& fieldName : fields) {
                            QTreeWidgetItem* fieldItem = new QTreeWidgetItem(msgItem);
                            fieldItem->setText(0, fieldName);
                            fieldItem->setFlags(fieldItem->flags() | Qt::ItemIsUserCheckable);
                            fieldItem->setCheckState(0, Qt::Unchecked);
                        }
                    }
                }
                
                QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
                buttonBox->button(QDialogButtonBox::Ok)->setText("Export");
                layout->addWidget(buttonBox);
                connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
                connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
                
                if (dialog.exec() == QDialog::Accepted) {
                    QString defaultName = QFileInfo(m_currentLogFile).path() + "/" + logName + "_" + acId + "_export.csv";
                    QString outFileName;
                    {
                        StderrBlocker blocker;
                        outFileName = QFileDialog::getSaveFileName(this, tr("Save CSV"), defaultName, tr("CSV Files (*.csv)"));
                    }
                    if (outFileName.isEmpty()) return;
                    
                    QMap<QString, QList<int>> selectedFields;
                    QMap<QString, QStringList> selectedFieldNames;
                    
                    int totalCols = 0;
                    QStringList headerCols;
                    headerCols << "Time";
                    
                    for (int i = 0; i < tree->topLevelItemCount(); ++i) {
                        QTreeWidgetItem* msgItem = tree->topLevelItem(i);
                        QString msgName = msgItem->text(0);
                        for (int j = 0; j < msgItem->childCount(); ++j) {
                            QTreeWidgetItem* fieldItem = msgItem->child(j);
                            if (fieldItem->checkState(0) == Qt::Checked) {
                                selectedFields[msgName].append(j);
                                QString cName = msgName + "." + fieldItem->text(0);
                                selectedFieldNames[msgName].append(cName);
                                headerCols << cName;
                                totalCols++;
                            }
                        }
                    }
                    if (totalCols == 0) {
                        QMessageBox::information(this, tr("Export CSV"), tr("No fields selected for export."));
                        return;
                    }

                    QFile inFile(m_currentLogFile);
                    if (!inFile.open(QIODevice::ReadOnly)) {
                        QMessageBox::warning(this, "Export CSV", "Cannot open data file.");
                        return;
                    }

                    QFile outFile(outFileName);
                    if (!outFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                        QMessageBox::warning(this, "Export CSV", "Cannot write to CSV file.");
                        return;
                    }

                    QTextStream out(&outFile);
                    out << headerCols.join(",") << "\n";
                    
                    QProgressDialog progress(tr("Exporting CSV..."), tr("Cancel"), 0, inFile.size(), this);
                    progress.setWindowModality(Qt::WindowModal);
                    progress.setMinimumDuration(100);
                    
                    const int CHUNK_SIZE = 1048576; // 1MB chunks
                    QByteArray buffer;
                    qint64 processedBytes = 0;
                    
                    // Pre-convert acId to UTF-8 to speed up comparison
                    QByteArray acIdBytes = acId.toUtf8();
                    const char* targetAcId = acIdBytes.constData();
                    int targetAcIdLen = acIdBytes.length();

                    while (!inFile.atEnd()) {
                        buffer.append(inFile.read(CHUNK_SIZE));
                        if (progress.wasCanceled()) break;
                        processedBytes = inFile.pos();
                        progress.setValue(processedBytes);
                        QCoreApplication::processEvents();

                        int lineStart = 0;
                        while (true) {
                            int nlIdx = buffer.indexOf('\n', lineStart);
                            if (nlIdx == -1) break;

                            int lineLen = nlIdx - lineStart;
                            if (lineLen > 0 && buffer.at(nlIdx - 1) == '\r') {
                                lineLen--;
                            }

                            if (lineLen > 0) {
                                const char* lineData = buffer.constData() + lineStart;
                                int s1 = -1, len1 = 0; // timestamp
                                int s2 = -1, len2 = 0; // ac_id
                                int s3 = -1, len3 = 0; // msg_name
                                int dataStart = -1;

                                for (int i = 0; i < lineLen; ++i) {
                                    if (lineData[i] != ' ' && lineData[i] != '\t' && lineData[i] != '\r') {
                                        if (s1 == -1) { s1 = i; }
                                        else if (len1 > 0 && s2 == -1) { s2 = i; }
                                        else if (len2 > 0 && s3 == -1) { s3 = i; }
                                        else if (len3 > 0 && dataStart == -1) { dataStart = i; break; }
                                    } else {
                                        if (s1 != -1 && s2 == -1) { len1 = i - s1; }
                                        else if (s2 != -1 && s3 == -1) { len2 = i - s2; }
                                        else if (s3 != -1 && len3 == 0) { len3 = i - s3; }
                                    }
                                }
                                if (s3 != -1 && len3 == 0) {
                                     len3 = lineLen - s3;
                                }

                                if (s2 != -1 && len2 > 0 && s3 != -1 && len3 > 0) {
                                    if (len2 == targetAcIdLen && qstrncmp(lineData + s2, targetAcId, len2) == 0) {
                                        QString msgName = QString::fromUtf8(lineData + s3, len3);
                                        if (selectedFields.contains(msgName)) {
                                            QString timeStr = QString::fromUtf8(lineData + s1, len1);
                                            QStringList values;
                                            if (dataStart != -1) {
                                                QString dataPart = QString::fromUtf8(lineData + dataStart, lineLen - dataStart);
                                                values = dataPart.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
                                            }
                                            
                                            QStringList row;
                                            row << timeStr;
                                            
                                            for (int i = 1; i < headerCols.size(); ++i) {
                                                const QString& col = headerCols[i];
                                                if (col.startsWith(msgName + ".")) {
                                                    int fIdx = selectedFieldNames[msgName].indexOf(col);
                                                    if (fIdx != -1 && fIdx < selectedFields[msgName].size()) {
                                                        int paramIdx = selectedFields[msgName][fIdx];
                                                        if (paramIdx < values.size()) {
                                                            row << values[paramIdx].trimmed();
                                                        } else {
                                                            row << "";
                                                        }
                                                    } else {
                                                        row << "";
                                                    }
                                                } else {
                                                    row << "";
                                                }
                                            }
                                            out << row.join(",") << "\n";
                                        }
                                    }
                                }
                            }
                            lineStart = nlIdx + 1;
                        }
                        buffer.remove(0, lineStart);
                    }
                    
                    progress.setValue(inFile.size());
                    QMessageBox::information(this, tr("Export CSV"), tr("CSV Export Complete!"));
                }
            });
        }
        
        while (QApplication::overrideCursor()) {
            QApplication::restoreOverrideCursor();
        }
    }
};

int main(int argc, char *argv[]) {

    // Set metadata BEFORE application instantiation to prevent XDG portal double-registration 
    // root cause ("Connection already associated with an application ID").
    QCoreApplication::setApplicationVersion("1.0");
    //QCoreApplication::setOrganizationName("paparazzi"); // only for settings, not really relevant here
    // Follow XDG spec for desktop integration and use a fixed name to ensure the .desktop file is correctly associated with the app
    QGuiApplication::setDesktopFileName(QStringLiteral("paparazzi_logplotter"));
    QCoreApplication::setApplicationName(QStringLiteral("Paparazzi log plotter"));

    // Suppress Wayland text input garbage warnings, ybe Mutter dev get their act together one day and fix this upstream, but until then, this is the cleanest solution to avoid spamming the console with GTK criticals when opening native dialogs on Wayland.
    qputenv("QT_LOGGING_RULES", "qt.qpa.wayland.textinput=false");

    QApplication app(argc, argv);

    //app.setApplicationDisplayName(QStringLiteral("Log Plotter"));

    QString iconPath = ":/penguin_icon_log.png";
    QIcon icon(iconPath);
    installLinuxDesktopIntegration(app.desktopFileName(), "Paparazzi log plotter", "Log plotter for telemetry messages", iconPath, "paparazzi-logplotter");

    // Apply the custom proxy style to the application.
    // We pass app.style() so it inherits all the default OS/Wayland drawing 
    // behavior, simply layering our palette override on top.
    app.setStyle(new EditorLighteningStyle(app.style()));

    app.setWindowIcon(icon);

    QCommandLineParser parser;
    parser.setApplicationDescription("Paparazzi Log Plotter");
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption exportCsvOption(QStringList() << "export_csv", "Export in CSV in batch mode according to saved preferences.");
    parser.addOption(exportCsvOption);

    QCommandLineOption verboseOption(QStringList() << "v" << "verbose", "Verbose mode.");
    parser.addOption(verboseOption);

    parser.addPositionalArgument("logs", "Log files to open.", "[log files...]");

    // Parse the command line arguments
    parser.process(app);

    bool exportCsv = parser.isSet(exportCsvOption);
    bool verbose = parser.isSet(verboseOption);
    QStringList logFiles = parser.positionalArguments();

    if (exportCsv) {
        qWarning() << "Batch CSV export via CLI is not currently supported in the Qt C++ backend. Please use the GUI Export CSV feature.";
    }

    if (verbose) {
        qDebug() << "Verbose mode enabled.";
        qDebug() << "Log files to process:" << logFiles;
    }

    if (logFiles.isEmpty()) {
        LogPlotterWindow* window = new LogPlotterWindow();
        window->setWindowIcon(icon);
        window->setAttribute(Qt::WA_DeleteOnClose);
        window->show();
    } else {
        for (const QString& argFile : logFiles) {
            // Ignore arguments that sneak through Qt arg parsing like wayland parameters just in case
            if (argFile == "--platform") continue;
            
            LogPlotterWindow* window = new LogPlotterWindow();
            window->setWindowIcon(icon);
            window->setAttribute(Qt::WA_DeleteOnClose);
            window->loadLogFile(argFile);
            
            if (!exportCsv) {
                window->show();
            }
        }
    }

    if (exportCsv) {
        // Mock OCaml behavior where CSV export skips GUI loop execution
        app.processEvents();
        return 0;
    }

    return app.exec();

}

#include "logplotter.moc"
