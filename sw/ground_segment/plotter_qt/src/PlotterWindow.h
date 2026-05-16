#ifndef PLOTTERWINDOW_H
#define PLOTTERWINDOW_H

#include <QMainWindow>
#include <QMap>
#include <QDateTime>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QList>

class QLineSeries;
class QChart;
class QChartView;
class QValueAxis;
class QCheckBox;
class QDoubleSpinBox;
class QPushButton;

namespace pprzlink {
    class MessageDictionary;
    class IvyQtLink;
    class Message;
}

struct PlotConfig {
    QString senderName;
    QString className;
    QString msgName;
    QString fieldName;
    double coef;
    QLineSeries* series;
    QList<QPointF> buffer;
};

class PlotterWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit PlotterWindow(QWidget *parent = nullptr);
    ~PlotterWindow();

protected:
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dropEvent(QDropEvent *event) override;

private slots:
    void onClearClicked();
    void onPauseToggled(bool checked);
    void onAutoScaleToggled(bool checked);
    void onManualScaleChanged();
    void onAddConstantClicked();
    void onUpdateRateChanged(double val);
    void updatePlots();

private:
    void setupIvy();
    void setupUI();
    void setupMenu();
    void addPlotFromPayload(const QString& payload);
    void handleMessage(QString sender, const pprzlink::Message& msg);
    
    QChart *m_chart;
    QValueAxis *m_axisX;
    QValueAxis *m_axisY;
    qint64 m_startTime;
    
    pprzlink::MessageDictionary* m_dict;
    pprzlink::IvyQtLink* m_link;
    
    QList<PlotConfig> m_activePlots;
    
    double m_minY;
    double m_maxY;
    bool m_paused;
    bool m_autoScale;
    
    QCheckBox* m_cbAutoScale;
    QDoubleSpinBox* m_spnMinY;
    QDoubleSpinBox* m_spnMaxY;
    QDoubleSpinBox* m_spnTimeWindow;
    QDoubleSpinBox* m_spnConstant;
    QDoubleSpinBox* m_spnUpdateRate;
    QTimer* m_updateTimer;
    class QMenu* m_curvesMenu;
};

#endif // PLOTTERWINDOW_H
