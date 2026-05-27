#include <QPair>
#include <QMessageBox>
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
#include <QFile>
#include <QTextStream>
#include <QRegularExpression>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QDir>
#include <QCheckBox>
#include <QLineEdit>
#include <QSlider>
#include <QLabel>
#include <QSpinBox>
#include <QTimer>
#include <QFileInfo>
#include <unistd.h>
#include <fcntl.h>
#include "../linux_desktop_utils.h"

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
#include "pprzlinkQt/MessageDictionary.h"
#include "pprzlinkQt/MessageDefinition.h"
#include "shared_plot.h"

class LogPlotterWindow : public QMainWindow {
    Q_OBJECT

public:
    LogPlotterWindow(QWidget *parent = nullptr) : QMainWindow(parent) {
        setWindowTitle("Log Plotter");
        resize(600, 300);
        QString xmlPath = QDir::homePath() + "/paparazzi/var/messages.xml";
        try { m_dict = new pprzlink::MessageDictionary(xmlPath); } catch (...) { m_dict = nullptr; }
        setupUI();
    }

private slots:
    void saveScreenshot() {
        if (!m_chartView) return;
        QString fileName;
        {
            StderrBlocker blocker;
            fileName = QFileDialog::getSaveFileName(this, tr("Save Screenshot"), QDir::homePath() + "/paparazzi/var/logs", tr("Images (*.png *.webp *.jpg *.bmp)"));
        }
        if (!fileName.isEmpty()) {
            QPixmap pixmap = m_chartView->grab();
            if (!pixmap.save(fileName)) {
                QMessageBox::warning(this, tr("Error"), tr("Failed to save screenshot to %1").arg(fileName));
            } else {
                qDebug() << "Screenshot saved to" << fileName;
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

void openLogFile() {
        QString fileName;
        {
            StderrBlocker blocker;
            fileName = QFileDialog::getOpenFileName(this, "Open Paparazzi Log", QDir::homePath() + "/paparazzi/var/logs", "Data Files (*.data);;All Files (*)");
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

    void setupMenu() {
        QMenu* fileMenu = menuBar()->addMenu(tr("&File"));
        
        QAction* openAction = fileMenu->addAction(tr("Open Log"));
        openAction->setShortcut(QKeySequence("Ctrl+O"));
        connect(openAction, &QAction::triggered, this, &LogPlotterWindow::openLogFile);

        QAction* newAction = fileMenu->addAction(tr("New"));
        newAction->setShortcut(QKeySequence("Ctrl+N"));
        connect(newAction, &QAction::triggered, this, []() {
            LogPlotterWindow* newWindow = new LogPlotterWindow();
            newWindow->show();
        });

        QAction* exportFigAction = fileMenu->addAction(tr("Export Fig"));
        connect(exportFigAction, &QAction::triggered, this, &LogPlotterWindow::exportFig);

        QAction* saveAction = fileMenu->addAction(tr("Save screenshot"));
        saveAction->setShortcut(QKeySequence("Ctrl+S"));
        connect(saveAction, &QAction::triggered, this, &LogPlotterWindow::saveScreenshot);

        QAction* exportAction = fileMenu->addAction(tr("Export fig"));
        exportAction->setShortcut(QKeySequence("Ctrl+X"));

        fileMenu->addSeparator();

        QAction* closeAction = fileMenu->addAction(tr("Close"));
        closeAction->setShortcut(QKeySequence("Ctrl+W"));
        connect(closeAction, &QAction::triggered, this, &QWidget::close);

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
        m_edtMinY->setMaximumWidth(60);
        m_edtMaxY->setMaximumWidth(60);
        m_edtMinY->setEnabled(false);
        m_edtMaxY->setEnabled(false);

        QLabel *lblConst = new QLabel("Constant");
        m_edtConstant = new QLineEdit();
        m_edtConstant->setMaximumWidth(50);

        QLabel *lblScaleNext = new QLabel("Scale next by");
        m_edtScaleNext = new QLineEdit("1.0");
        m_edtScaleNext->setMaximumWidth(50);

        m_spnLineThickness = new QSpinBox();
        m_spnLineThickness->setToolTip("Line Thickness (px)");
        m_spnLineThickness->setRange(1, 10);
        m_spnLineThickness->setValue(1);
        m_spnLineThickness->hide();

        m_updateTimer = new QTimer(this);

        toolbarLayout->addWidget(m_cbAutoScale);
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

        m_chartView = new QChartView();
        m_chartView->setContentsMargins(0, 0, 0, 0);
        m_chartView->setFrameShape(QFrame::NoFrame);
        
        m_chart = new QChart();
        m_chart->setMargins(QMargins(0, 0, 0, 0));
        m_chart->layout()->setContentsMargins(0, 0, 0, 0);
        m_chart->setBackgroundRoundness(0);
        m_chart->setBackgroundPen(QPen(Qt::NoPen));
        m_chartView->setChart(m_chart);
        m_chartView->setRenderHint(QPainter::Antialiasing);
        
        m_axisX = new QValueAxis();
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
            m_axisY->setRange(calcMinY - marginY, calcMaxY + marginY);
            m_axisX->show();
            m_axisY->show();
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
        logName.replace("__", "#TEMP#");
        logName.replace("_", "");
        logName.replace("#TEMP#", "_");

        QString curveTitle = logName + ":" + acId + ":" + msgName + ":" + fieldName + ":" + QString::number(scale) + "," + QString::number(transpose);

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
                QAction* deleteAction = m_curvesMenu->addAction(icon, curveTitle);
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

    void loadLogFile(const QString &fileName) {
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
                            }
                        }
                    }
                }
            }
        }

        m_currentLogFile = dataFileName;
        QFile file(dataFileName);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QMessageBox::warning(this, "Error", "Cannot open file " + dataFileName);
            return;
        }

        QSet<QPair<QString, QString>> acMsgPairs;
        
        // Fast parsing of the data file
        const int CHUNK_SIZE = 1024 * 1024;
        QByteArray buffer;
        while (!file.atEnd()) {
            buffer.append(file.read(CHUNK_SIZE));
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
            QString acNameDisplay = acIdToName.value(acId, "AC_" + acId);
            QString menuTitle = logName + ":" + acNameDisplay + " (" + acId + ")";
            QMenu* acMenu = menuBar()->addMenu(menuTitle);
            
            QStringList msgs = it.value().values();
            msgs.sort(); // Sorting messages alphabetically
            
            for (const QString& msgName : msgs) {
                if (dictFields.contains(msgName)) {
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
        }
    }
};

int main(int argc, char *argv[]) {
    // Suppress Wayland text input harmless warnings

    // Set metadata BEFORE application instantiation to prevent XDG portal double-registration 
    // root cause ("Connection already associated with an application ID").
    QCoreApplication::setApplicationVersion("1.0");
    //QCoreApplication::setOrganizationName("paparazzi"); // only for settings, not really relevant here
    // Follow XDG spec for desktop integration and use a fixed name to ensure the .desktop file is correctly associated with the app
    QGuiApplication::setDesktopFileName(QStringLiteral("paparazzi_logplotter"));
    QCoreApplication::setApplicationName(QStringLiteral("Paparazzi log plotter"));

    // Suppress Wayland text input harmless warnings
    qputenv("QT_LOGGING_RULES", "qt.qpa.wayland.textinput=false");

    QApplication app(argc, argv);

    //app.setApplicationDisplayName(QStringLiteral("Log Plotter"));

    QString iconPath = ":/penguin_icon_log.png";
    QIcon icon(iconPath);
    installLinuxDesktopIntegration(app.desktopFileName(), "Paparazzi log plotter", "Log plotter for telemetry messages", iconPath, "paparazzi-logplotter");

    app.setWindowIcon(icon);

    LogPlotterWindow window;
    window.setWindowIcon(icon);
    window.show();

    return app.exec();

}

#include "logplotter.moc"
