#include "block.h"
#include <iostream>
#include <assert.h>
#include <QtXml>

Block::Block(QString name, uint8_t no) :
    name(name), no(no)
{

}

Block::Block(QDomElement blk) {

    assert(blk.hasAttribute("name"));

    name = blk.attribute("name");
    no = blk.attribute("no", "0").toUInt();

    setIcon(blk.attribute("strip_icon", ""));
    setText(blk.attribute("strip_button", ""));
    setKey(blk.attribute("key", ""));
    setGroup(blk.attribute("group", ""));


    auto attr = blk.attributes();
    for(int i=0; i < attr.count(); i++) {
        auto att = attr.item(i).toAttr();
        attributes[att.name()] = att.value();
    }

    parse_block_stages(blk);
}

void Block::parse_block_stages(QDomElement blk) {
    for(auto stel=blk.firstChildElement();
        !stel.isNull();
        stel = stel.nextSiblingElement()) {
        Stage stage;
        stage.no = 255;
        stage.instruction = stel.tagName();

        auto attr = stel.attributes();
        for(int i=0; i < attr.count(); i++) {
            auto att = attr.item(i).toAttr();
            stage.attributes[att.name()] = att.value();
        }

        stage.no = stel.attribute("no", "0").toUInt();
        stages.push_back(stage);
    }
}


ostream& operator<<(ostream& os, const Block& bk) {

    os << "{" << bk.name.toStdString() << ", no: " << to_string(bk.no);

    if(bk.strip_icon != "") {
        os << " icon: " << bk.strip_icon.toStdString();
    }

    if(bk.strip_button_txt != "") {
        os << " text: " << bk.strip_button_txt.toStdString();
    }

    if(bk.key != "") {
        os << " key: " << bk.key.toStdString();
    }

    if(bk.group != "") {
        os << " group: " << bk.group.toStdString();
    }

    os << "}";

    return os;
}
