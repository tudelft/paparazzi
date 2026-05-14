#ifndef STRIP_H
#define STRIP_H

#include <QtWidgets>
#include <QMouseEvent>
#include "pprz_dispatcher.h"
#include "graphlabel.h"
#include "jaugelabel.h"
#include "colorlabel.h"

class Strip : public QWidget
{
    Q_OBJECT
public:
    explicit Strip(QString ac_id, QWidget *parent = nullptr, bool full = false);
    ~Strip();

    void setCompact(bool);
    void addWidget(QWidget* w, QString group = "");

    static Strip* getStrip(QString ac_id);

signals:

public slots:

protected:
    void paintEvent(QPaintEvent*) override;
    virtual void mousePressEvent(QMouseEvent *e) override;
    virtual void mouseReleaseEvent(QMouseEvent *e) override;
    bool eventFilter(QObject *watched, QEvent *event) override;

private:
    void build_full_strip();
    void build_short_strip();

    void updateEngineStatus();
    void updateApStatus();
    void updateAltTargetDiff();
    void updateFlightParams();
    void updateTelemetryStatus();
    void updateFBW();

    QString _ac_id;
    QColor ac_color;

    QWidget* full_strip;
    QWidget* short_strip;
    QHBoxLayout* layout_user;
    QMap<QString, QVBoxLayout*> user_group_layouts;

    GraphLabel* full_bat_graph;
    GraphLabel* full_alt_graph;
    JaugeLabel* full_throttle_label;
    JaugeLabel* full_speed_label;
    ColorLabel* full_link_label;
    ColorLabel* full_ap_mode_label;
    ColorLabel* full_fbw_mode_label;
    ColorLabel* full_gps_mode_label;
    QLabel*     full_flight_time_label;
    QLabel*     full_block_name_label;
    QLabel*     full_stage_name_label;
    QLabel*     full_alt_value_label;
    QLabel*     full_target_diff_label;
    QLabel*     full_block_time_label;
    QLabel*     full_stage_time_label;
    QLabel*     full_eta_time_label;
    QLabel*     full_apt_label;
    QLabel*     full_apt_value_label;

    QPushButton* btn_launch;
    QPushButton* btn_kill;
    QPushButton* btn_resurrect;
    QPushButton* btn_up;
    QPushButton* btn_down;
    QPushButton* btn_up_up;
    QPushButton* btn_left;
    QPushButton* btn_center;
    QPushButton* btn_right;
    QPushButton* btn_mark;
    
    JaugeLabel* short_jl_bat;
    QLabel*     short_flight_time_label;
    QLabel*     short_speed_label;
    QLabel*     short_alt_label;
    QLabel*     short_vspeed_indicator;
    QLabel*     short_vspeed_label;
    QLabel*     short_target_label;
    ColorLabel* short_ap_mode_label;
    ColorLabel* short_fbw_mode_label;
    ColorLabel* short_gps_mode_label;

    static QMap<QString, Strip*> strips;
};

#endif // STRIP_H


/* --- HISTORICAL OCAML COMMENTS: strip.mli ---
 * * Add a user widget in the low row of the strip
 ------------------------------------------
 * * [connect_apt get_ac_unix_time send_value]
 ------------------------------------------
 * * [add config params]
 ------------------------------------------
*/
