#ifndef GPS_CLASSIC_VIEWER_H
#define GPS_CLASSIC_VIEWER_H

#include <QtWidgets>

class GPSClassicViewer : public QWidget
{
    Q_OBJECT
public:
    explicit GPSClassicViewer(QString ac_id, QWidget *parent = nullptr);

    QSize sizeHint() const;
    QSize minimumSizeHint() const;

protected:
    void paintEvent(QPaintEvent *event);

public:
    virtual void mousePressEvent(QMouseEvent *event);

signals:

public slots:

private:

    struct SvInfo {
        int id;
        int flags;
        int cno;
        int age;
    };

    std::tuple<uint16_t, QList<struct SvInfo>> getData();

    static const int SV_WIDTH = 30;

    QString ac_id;

    QSize minSize;

    bool reduced;
};

class Setting;

class GPSViewerTab : public QWidget
{
    Q_OBJECT
public:
    explicit GPSViewerTab(QString ac_id, QWidget *parent = nullptr);

private:
    QString ac_id;
    GPSClassicViewer* viewer;
    Setting* gps_reset_setting;
};

#endif // GPS_CLASSIC_VIEWER_H
