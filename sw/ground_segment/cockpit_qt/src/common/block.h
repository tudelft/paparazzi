#ifndef BLOCK_H
#define BLOCK_H

#include <map>
#include <QString>
#include <QtXml>

using namespace std;

struct Stage {
    QString instruction;
    map<QString, QString> attributes;
    uint8_t no;
};

class Block
{
public:
    Block(QDomElement blk);
    Block(QString name, uint8_t no);

    void setIcon(QString icon) {strip_icon = icon;}
    void setText(QString text) {strip_button_txt = text;}
    void setKey(QString key) {this->key = key;}
    void setGroup(QString group) {this->group = group;}

    QString getName() {return name;}
    uint8_t getNo() {return no;}
    QString getIcon() {return strip_icon;}
    QString getText() {return strip_button_txt ;}
    QString getKey() {return key;}
    QString getGroup() {return group;}
    QList<Stage>& getStages() {return stages;}

    friend ostream& operator<<(ostream& os, const Block& wp);

private:
    void parse_block_stages(QDomElement blk);

    QString name;
    uint8_t no;

    QString strip_icon;
    QString strip_button_txt;
    QString key;
    QString group;

    map<QString, QString> attributes;

    QList<Stage> stages;
};

#endif // BLOCK_H
