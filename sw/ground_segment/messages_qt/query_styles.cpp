#include <QApplication>
#include <QStyleFactory>
#include <QDebug>

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    qDebug() << QStyleFactory::keys();
    return 0;
}
