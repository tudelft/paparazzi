#ifndef ALERTVIEWER_H
#define ALERTVIEWER_H

#include <QWidget>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QDateTime>

class AlertViewer : public QWidget
{
    Q_OBJECT
public:
    explicit AlertViewer(QWidget *parent = nullptr);

public slots:
    void addMessage(const QString& text);

private:
    QTextEdit* view;
    QString lastText;
};

#endif // ALERTVIEWER_H
