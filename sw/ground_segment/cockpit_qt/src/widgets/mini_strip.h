#ifndef MINISTRIP_H
#define MINISTRIP_H

#include <QtWidgets>
#include <QGridLayout>
#include <QHBoxLayout>
#include "aircraft_watcher.h"

class MiniStrip : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(int size READ getSizeProperty WRITE setSizeProperty DESIGNABLE true)
public:
    explicit MiniStrip(QString ac_id, QWidget *parent = nullptr);
    void setSizeProperty(int s) {icons_size = QSize(s, s);}
    int getSizeProperty() {return icons_size.width();}

protected:
    bool eventFilter(QObject *obj, QEvent *ev) override;


signals:
    void updated();
private:

    static const QMap<QString, QString> mode_colors;

    enum SpeedMode {
        GROUND_SPEED,
        AIR_SPEED,
        VERTICAL_SPEED,
    };

    enum TimeMode {
        FLIGHT_TIME,
        BLOCK_TIME,
        STAGE_TIME,
    };

    void handle_bat_status(AircraftWatcher::BatStatus);

    void updateData();

    void updateFlightTime(uint32_t flight_time, uint32_t block_time, uint32_t stage_time);
    void updateAp(QString ap_mode);
    void updateGps(QString gps_mode);
    void updateImu(QString state_filter_mode);
    void updateCurrentBlock(uint8_t cur_block);

    QString ac_id;
    QSize icons_size;

    QToolButton* time_button;
    QLabel* flight_time_label;
    TimeMode time_mode;
    QIcon time_flight;
    QIcon time_block;
    QIcon time_stage;

    QToolButton* alt_button;
    QIcon agl_icon;
    QIcon msl_icon;
    QLabel* alt_label;
    bool alt_mode;

    QToolButton* speed_button;
    QIcon ground_speed_icon;
    QIcon air_speed_icon;
    QIcon vertical_speed_icon;
    QLabel* speed_label;
    SpeedMode speed_mode;

    QLabel* block_label;
    QLabel* block_icon;

    QLabel* bat_icon;
    QIcon bat_ok, bat_low, bat_critic, bat_catastrophic;
    QLabel* bat_label;
    int bat_nb_cells;

    QLabel* throttle_icon;
    QLabel* throttle_label;
    QIcon throttle_on;
    QIcon throttle_killed;

    QPushButton* ap_mode_button;

    QLabel* link_icon;
    QIcon link_ok;
    QIcon link_warning;
    QIcon link_lost;

    QLabel* gps_icon;
    QIcon gps_ok;
    QIcon gps_lost;

    QLabel* rc_icon;
    QIcon rc_ok;
    QIcon rc_lost;

    QLabel* imu_icon;
    QIcon imu_ok;
    QIcon imu_lost;

};

#endif // MINISTRIP_H
