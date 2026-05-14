#include "papgetconfig.h"
#include "AircraftManager.h"

PapgetConfig::PapgetConfig(Papget::DataDef datadef, Papget::Params params, QWidget *parent) : QDialog(parent),
    datadef(datadef), current_index(0), params(params)
{
    setWindowTitle(datadef.msg_name + " " + datadef.field);
    auto lay = new QVBoxLayout(this);
    auto comboStyle = new QComboBox(this);
    lay->addWidget(comboStyle);
    auto stack = new QStackedWidget(this);
    lay->addWidget(stack);


    comboStyle->addItem("Text");
    QWidget* text_widget = new QWidget(this);
    auto cb = config_text(text_widget);
    callbacks.append(cb);
    stack->addWidget(text_widget);

    connect(comboStyle, qOverload<int>(&QComboBox::currentIndexChanged), this, [=](int index) {
        stack->setCurrentIndex(index);
        callbacks[index]();
    });


    auto buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    lay->addWidget(buttonBox);

    connect(buttonBox->button(QDialogButtonBox::Ok), &QPushButton::clicked, [=]() mutable {

        emit paramsChanged(this->params);
        accept();
    });

    connect(buttonBox->button(QDialogButtonBox::Cancel), &QPushButton::clicked, [=](){
        reject();
    });
}

std::function<void()> PapgetConfig::config_text(QWidget* w) {
    auto layout = new QVBoxLayout(w);

    auto size_layout = new QHBoxLayout();
    size_layout->addWidget(new QLabel("Size", w));
    auto size_sp = new QSpinBox(w);
    size_sp->setValue(params.fontSize);
    size_layout->addWidget(size_sp);
    layout->addLayout(size_layout);

    auto color_layout = new QHBoxLayout();
    color_layout->addWidget(new QLabel("Color", w));
    auto combo_color = new QComboBox(w);
    color_layout->addWidget(combo_color);
    QStringList colors = {"", "AC color", "black", "blue", "red", "green", "purple", "yellow", "white"};
    combo_color->addItems(colors);
    layout->addLayout(color_layout);

    connect(size_sp, qOverload<int>(&QSpinBox::valueChanged), [=](int val) {
        params.fontSize = val;
        emit paramsChanged(params);
    });

    connect(combo_color, &QComboBox::currentTextChanged, this, [=](QString text) {
        if(text == "AC color" || text == ""){
            auto ac = AircraftManager::get()->getAircraft(datadef.ac_id);
            params.color = ac->getColor();
        } else {
            params.color = text;
        }
        emit paramsChanged(params);
    });

    auto val = [=]() mutable {
        params.fontSize = size_sp->value();
        params.style = Papget::Style::TEXT;
    };

    return val;
}
