import sys

with open("src/MainWindow.cpp", "r") as f:
    content = f.read()

# Fix itemWidget transparency so the selection highlighting of Adwaita shows through
content = content.replace("QWidget* itemWidget = new QWidget();", "QWidget* itemWidget = new QWidget();\n        itemWidget->setStyleSheet(\"background: transparent;\");")

# Fix timeBox typography in dark themes to be readable (black over green), with nicer rounded borders mimicking Adwaita badges
content = content.replace("timeBox->setStyleSheet(\"QWidget { background-color: #00ff00; }\");", "timeBox->setStyleSheet(\".QWidget { background-color: #55dd55; border-radius: 4px; }\\nQLabel { color: #000; font-weight: bold; }\");")
content = content.replace("tracker.timeBox->setStyleSheet(\"QWidget { background-color: #00ff00; }\");", "tracker.timeBox->setStyleSheet(\".QWidget { background-color: #55dd55; border-radius: 4px; }\\nQLabel { color: #000; font-weight: bold; }\");")
content = content.replace("tracker.timeBox->setStyleSheet(\"QWidget { background-color: #008000; }\");", "tracker.timeBox->setStyleSheet(\".QWidget { background-color: #229922; border-radius: 4px; }\\nQLabel { color: #fff; font-weight: bold; }\");")

# Ensure the list widget itself has a proper modern feel by removing its generic focus rectangle and providing standard margin if any
content = content.replace("m_listWidget = new QListWidget(this);", "m_listWidget = new QListWidget(this);\n    m_listWidget->setFrameShape(QFrame::NoFrame);\n    m_listWidget->setAttribute(Qt::WA_MacShowFocusRect, false);")

with open("src/MainWindow.cpp", "w") as f:
    f.write(content)

