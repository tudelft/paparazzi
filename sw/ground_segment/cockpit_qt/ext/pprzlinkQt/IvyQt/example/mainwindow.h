#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <IvyQt/ivyqt.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void subscribe_regex();

private:
    Ui::MainWindow *ui;

    IvyQt* ivyqt;
};

#endif // MAINWINDOW_H
