#ifndef PLOTTER_H
#define PLOTTER_H

#include <QMainWindow>
#include <QList>
#include <QPointF>
#include <QString>

class QLineSeries;
class QChart;
class QChartView;
class QValueAxis;
class QCheckBox;
class QLineEdit;
class QSlider;
class QMenu;
class QTimer;
class QWidget;

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
    int fieldIndex = -1;
    double coef = 1.0;
    QLineSeries* series = nullptr;
    QList<QPointF> buffer;
    bool discrete = false;
};

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
    void onLegendRefreshTimeout();

private:
    QWidget* m_legendOverlay;
    class QVBoxLayout* m_legendLayout;
    void updateLegendValues();
    void updateLegendPosition();
    bool m_legendNeedsRefresh = false;
    QTimer* m_legendUpdateTimer = nullptr;

    void setupIvy();
    void setupUI();
    void setupMenu();
    void addPlotFromPayload(const QString& payload);
    void handleMessage(QString sender, const pprzlink::Message& msg);
    
    void addCurveToMenu(PlotConfig& config);
    void removeCurve(QLineSeries* series);
    
    QChart *m_chart = nullptr;
    QValueAxis *m_axisX = nullptr;
    QValueAxis *m_axisY = nullptr;
    qint64 m_startTime = 0;
    
    pprzlink::MessageDictionary* m_dict = nullptr;
    pprzlink::IvyQtLink* m_link = nullptr;
    
    QList<PlotConfig> m_activePlots;
    
    double m_minY = 1e9;
    double m_maxY = -1e9;
    bool m_paused = false;
    bool m_autoScale = true;
    
    QCheckBox* m_cbAutoScale = nullptr;
    class QLineEdit* m_edtMinY = nullptr;
    class QLineEdit* m_edtMaxY = nullptr;
    class QSlider* m_slTimeWindow = nullptr;
    class QLineEdit* m_edtConstant = nullptr;
    class QSlider* m_slUpdateRate = nullptr;
    class QLineEdit* m_edtScaleNext = nullptr;
    class QSpinBox* m_spnLineThickness = nullptr;
    QTimer* m_updateTimer = nullptr;
    class QMenu* m_curvesMenu = nullptr;
};

#endif // PLOTTER_H
