#pragma once
#include <QColor>
#include <cmath>
#include <QLineSeries>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QAction>
#include <QMenu>
#include <QChart>
#include <QChartView>
#include <QTimer>
#include <QWidget>
#include <QEvent>
#include <QCoreApplication>

static inline int g_colorIndex = 0;
static inline QColor getNextSaturatedColor() {
    double h = std::fmod(g_colorIndex * 137.508, 360.0);
    g_colorIndex++;
    return QColor::fromHsvF(h / 360.0, 1.0, 1.0);
}

class ChartLegendManager : public QObject {
public:
    ChartLegendManager(QChart* chart, QWidget* overlayParent) 
        : QObject(overlayParent), m_chart(chart) {
        
        m_legendOverlay = new QWidget(overlayParent);
        m_legendOverlay->setObjectName("LegendOverlay");
        m_legendOverlay->setAttribute(Qt::WA_TransparentForMouseEvents);
        m_legendOverlay->setStyleSheet("#LegendOverlay { background-color: rgba(255, 255, 255, 220); border: none; border-radius: 4px; } * { color: black; }");
        
        m_legendLayout = new QVBoxLayout(m_legendOverlay);
        m_legendLayout->setContentsMargins(0, 0, 0, 0);
        m_legendLayout->setSpacing(0);
        m_legendOverlay->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        m_legendOverlay->hide();

        m_legendUpdateTimer = new QTimer(this);
        m_legendUpdateTimer->setInterval(200);
        connect(m_legendUpdateTimer, &QTimer::timeout, this, &ChartLegendManager::onLegendRefreshTimeout);
        m_legendUpdateTimer->start();
        
        overlayParent->installEventFilter(this);
    }
    
    QWidget* overlay() const { return m_legendOverlay; }
    void requestRefresh() { m_legendNeedsRefresh = true; }
    
    void updateLegendPosition() {
        if (!m_chart || !m_legendOverlay || !m_legendLayout) return;
        m_chart->legend()->hide();
        
        m_legendOverlay->hide();
        
        QLayoutItem* item;
        while ((item = m_legendLayout->takeAt(0)) != nullptr) {
            if (QWidget* widget = item->widget()) {
                widget->hide();
                widget->deleteLater();
            }
            delete item;
        }
        
        
        auto seriesList = m_chart->series();
        if (seriesList.isEmpty()) {
            return;
        }
        
        for (auto* s : seriesList) {
            QLineSeries* ls = qobject_cast<QLineSeries*>(s);
            if (ls) {
                QWidget* rowWidget = new QWidget();
                rowWidget->setStyleSheet("background: transparent; border: none;");
                QHBoxLayout* rowLayout = new QHBoxLayout(rowWidget);
                rowLayout->setContentsMargins(4, 2, 4, 2);
                rowLayout->setSpacing(5);
                
                QLabel* colorBox = new QLabel();
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
                int boxSize = textHeight;
                colorBox->setFixedSize(boxSize, boxSize);
                
                rowLayout->addWidget(textLbl, 1);
                rowLayout->addWidget(colorBox, 0);
                
                m_legendLayout->addWidget(rowWidget);
                rowWidget->show();
            }
        }
        
        triggerRelayout();
    }
    
    void triggerRelayout() {
        if (m_legendOverlay && m_legendOverlay->parentWidget()) {
            if (m_legendLayout->count() == 0) {
                m_legendOverlay->hide();
                return;
            }
            
            m_legendOverlay->show();

            m_legendLayout->invalidate();
            m_legendLayout->activate();
            m_legendOverlay->adjustSize();
            
            int pWidth = m_legendOverlay->parentWidget()->width();
            int wWidth = m_legendOverlay->width();
            m_legendOverlay->move(std::max(0, pWidth - wWidth - 15), 15);
            
            m_legendOverlay->raise();
            m_legendOverlay->repaint();
        }
    }

protected:
    bool eventFilter(QObject *obj, QEvent *event) override {
        if (event->type() == QEvent::Resize && m_legendOverlay && obj == m_legendOverlay->parentWidget()) {
            triggerRelayout();
        }
        return QObject::eventFilter(obj, event);
    }

private:
    void onLegendRefreshTimeout() {
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
                if (item && item->widget()) {
                    QWidget* rowWidget = item->widget();
                    QHBoxLayout* l = qobject_cast<QHBoxLayout*>(rowWidget->layout());
                    if (l && l->count() >= 2) {
                        QLabel* lbl = qobject_cast<QLabel*>(l->itemAt(0)->widget());
                        if (lbl) {
                            lbl->setText(QString("%1 : %2").arg(ls->name()).arg(latestVal, 0, 'f', 4));
                        }
                    }
                }
            }
        }
    }

    QChart* m_chart;
    QWidget* m_legendOverlay;
    QVBoxLayout* m_legendLayout;
    QTimer* m_legendUpdateTimer;
    bool m_legendNeedsRefresh = true;
};
