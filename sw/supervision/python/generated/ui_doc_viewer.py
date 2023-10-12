# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/ui_doc_viewer.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_DocPanel(object):
    def setupUi(self, DocPanel):
        DocPanel.setObjectName("DocPanel")
        DocPanel.resize(654, 658)
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(DocPanel)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.splitter_2 = QtWidgets.QSplitter(DocPanel)
        self.splitter_2.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_2.setObjectName("splitter_2")
        self.conf = QtWidgets.QWidget(self.splitter_2)
        self.conf.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.conf.setObjectName("conf")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.conf)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_4 = QtWidgets.QLabel(self.conf)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_4.sizePolicy().hasHeightForWidth())
        self.label_4.setSizePolicy(sizePolicy)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_2.addWidget(self.label_4)
        self.target_combo = QtWidgets.QComboBox(self.conf)
        self.target_combo.setObjectName("target_combo")
        self.horizontalLayout_2.addWidget(self.target_combo)
        self.verticalLayout_4.addLayout(self.horizontalLayout_2)
        self.searchLineEdit = QtWidgets.QLineEdit(self.conf)
        self.searchLineEdit.setClearButtonEnabled(True)
        self.searchLineEdit.setObjectName("searchLineEdit")
        self.verticalLayout_4.addWidget(self.searchLineEdit)
        self.verticalLayout.addLayout(self.verticalLayout_4)
        self.momo = QtWidgets.QWidget(self.conf)
        self.momo.setObjectName("momo")
        self.verticalLayout_8 = QtWidgets.QVBoxLayout(self.momo)
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.splitter = QtWidgets.QSplitter(self.momo)
        self.splitter.setOrientation(QtCore.Qt.Vertical)
        self.splitter.setObjectName("splitter")
        self.layoutWidget = QtWidgets.QWidget(self.splitter)
        self.layoutWidget.setObjectName("layoutWidget")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.layoutWidget)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label = QtWidgets.QLabel(self.layoutWidget)
        self.label.setObjectName("label")
        self.verticalLayout_3.addWidget(self.label)
        self.modules_list = QtWidgets.QListWidget(self.layoutWidget)
        self.modules_list.setObjectName("modules_list")
        self.verticalLayout_3.addWidget(self.modules_list)
        self.layoutWidget1 = QtWidgets.QWidget(self.splitter)
        self.layoutWidget1.setObjectName("layoutWidget1")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.layoutWidget1)
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.label_2 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_2.setObjectName("label_2")
        self.verticalLayout_6.addWidget(self.label_2)
        self.depends_modules_list = QtWidgets.QListWidget(self.layoutWidget1)
        self.depends_modules_list.setObjectName("depends_modules_list")
        self.verticalLayout_6.addWidget(self.depends_modules_list)
        self.layoutWidget2 = QtWidgets.QWidget(self.splitter)
        self.layoutWidget2.setObjectName("layoutWidget2")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.layoutWidget2)
        self.verticalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.label_3 = QtWidgets.QLabel(self.layoutWidget2)
        self.label_3.setObjectName("label_3")
        self.verticalLayout_7.addWidget(self.label_3)
        self.unloaded_modules_list = QtWidgets.QListWidget(self.layoutWidget2)
        self.unloaded_modules_list.setObjectName("unloaded_modules_list")
        self.verticalLayout_7.addWidget(self.unloaded_modules_list)
        self.verticalLayout_8.addWidget(self.splitter)
        self.verticalLayout.addWidget(self.momo)
        self.verticalLayout.setStretch(1, 1)
        self.layoutWidget3 = QtWidgets.QWidget(self.splitter_2)
        self.layoutWidget3.setObjectName("layoutWidget3")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.layoutWidget3)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.backButton = QtWidgets.QPushButton(self.layoutWidget3)
        self.backButton.setText("")
        icon = QtGui.QIcon.fromTheme("go-previous")
        self.backButton.setIcon(icon)
        self.backButton.setObjectName("backButton")
        self.horizontalLayout.addWidget(self.backButton)
        self.urlLineEdit = QtWidgets.QLineEdit(self.layoutWidget3)
        self.urlLineEdit.setObjectName("urlLineEdit")
        self.horizontalLayout.addWidget(self.urlLineEdit)
        self.open_browser_button = QtWidgets.QToolButton(self.layoutWidget3)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icons/icons/browser.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.open_browser_button.setIcon(icon)
        self.open_browser_button.setObjectName("open_browser_button")
        self.horizontalLayout.addWidget(self.open_browser_button)
        self.doc_source_combo = QtWidgets.QComboBox(self.layoutWidget3)
        self.doc_source_combo.setObjectName("doc_source_combo")
        self.doc_source_combo.addItem("")
        self.doc_source_combo.addItem("")
        self.horizontalLayout.addWidget(self.doc_source_combo)
        self.verticalLayout_2.addLayout(self.horizontalLayout)
        self.webView = QtWebKitWidgets.QWebView(self.layoutWidget3)
        self.webView.setUrl(QtCore.QUrl("about:blank"))
        self.webView.setObjectName("webView")
        self.verticalLayout_2.addWidget(self.webView)
        self.verticalLayout_5.addWidget(self.splitter_2)

        self.retranslateUi(DocPanel)
        QtCore.QMetaObject.connectSlotsByName(DocPanel)

    def retranslateUi(self, DocPanel):
        _translate = QtCore.QCoreApplication.translate
        DocPanel.setWindowTitle(_translate("DocPanel", "Form"))
        self.label_4.setText(_translate("DocPanel", "Target:"))
        self.searchLineEdit.setPlaceholderText(_translate("DocPanel", "search..."))
        self.label.setText(_translate("DocPanel", "Modules"))
        self.label_2.setText(_translate("DocPanel", "Modules Dependencies"))
        self.label_3.setText(_translate("DocPanel", "Modules Unloaded"))
        self.backButton.setToolTip(_translate("DocPanel", "Go back"))
        self.open_browser_button.setToolTip(_translate("DocPanel", "Open in Browser"))
        self.open_browser_button.setText(_translate("DocPanel", "..."))
        self.doc_source_combo.setToolTip(_translate("DocPanel", "source"))
        self.doc_source_combo.setItemText(0, _translate("DocPanel", "Internet"))
        self.doc_source_combo.setItemText(1, _translate("DocPanel", "Local"))
from PyQt5 import QtWebKitWidgets
from generated import resources_rc
