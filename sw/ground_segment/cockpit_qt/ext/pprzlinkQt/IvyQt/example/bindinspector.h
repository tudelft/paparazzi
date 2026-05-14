#ifndef BINDINSPECTOR_H
#define BINDINSPECTOR_H

#include <QWidget>

namespace Ui {
class BindInspector;
}

class BindInspector : public QWidget
{
    Q_OBJECT

public:
    explicit BindInspector(QWidget *parent = nullptr);
    ~BindInspector();

private:
    Ui::BindInspector *ui;

    friend class MainWindow;
};

#endif // BINDINSPECTOR_H
