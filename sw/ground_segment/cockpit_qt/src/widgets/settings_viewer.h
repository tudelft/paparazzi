#ifndef SETTINGS_VIEWER_H
#define SETTINGS_VIEWER_H

#include <QWidget>
#include <QtWidgets>
#include <QBoxLayout>
#include "configurable.h"
#include <setting_menu.h>
#include <memory>
#include "pprz_dispatcher.h"
#include <QMouseEvent>

class SettingsViewer : public QWidget
{
    Q_OBJECT
public:
    explicit SettingsViewer(QString ac_id, QWidget *parent = nullptr);

signals:

public slots:

protected:
    bool eventFilter(QObject *obj, QEvent *ev) override;

private:
    void init(QString ac_id);
    void create_widgets(SettingMenu* setting_menu, QList<SettingMenu*> stack);
    void populate_search_results(QString searched);
    void restore_searched_items();
    QWidget* makeSettingWidget(Setting* setting, QWidget* parent);
    void updateSettings(QString id, Setting*, float);

    QString ac_id;
    QVBoxLayout* main_layout;
    QHBoxLayout* search_layout;
    QHBoxLayout* path_save_layout;
    QHBoxLayout* path_layout;
    QStackedWidget* path;
    QScrollArea* scroll;
    QStackedWidget* scroll_content;
    QVBoxLayout* content_layout;

    QToolButton* button_home;
    QToolButton* button_save;

    QLineEdit* search_bar;

    QList<QWidget*> path_widgets;

    QMap<SettingMenu*, int> widgets_indexes;
    QMap<SettingMenu*, int> path_indexes;

    int last_widget_index;
    int last_path_index;

    QMap<Setting*, QWidget*> setting_widgets;
    QMap<SettingMenu*, QWidget*> setting_menu_widgets;

    int search_result_index;
    int search_path_index;

    struct PositionHistory {
        QWidget* widget;
        QBoxLayout* layout;
        int index;
    };

    QList<PositionHistory> pos_hist;


    std::map<Setting*, std::function<void(double)>> label_setters;
    std::map<Setting*, std::function<void(double)>> setters;
    std::map<Setting*, bool> initialized;
    QList<QShortcut*> shortcuts;

};

class SettingSaver: public QDialog
{
    Q_OBJECT
public:
    explicit SettingSaver(QString ac_id, QWidget *parent = nullptr);

signals:

public slots:

private:
    QString ac_id;

};

#endif // SETTINGS_VIEWER_H
