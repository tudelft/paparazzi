# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'rt_plotter.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_RT_Plotter(object):
    def setupUi(self, RT_Plotter):
        RT_Plotter.setObjectName("RT_Plotter")
        RT_Plotter.resize(910, 404)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(RT_Plotter.sizePolicy().hasHeightForWidth())
        RT_Plotter.setSizePolicy(sizePolicy)
        RT_Plotter.setAcceptDrops(False)
        self.verticalLayout = QtWidgets.QVBoxLayout(RT_Plotter)
        self.verticalLayout.setObjectName("verticalLayout")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.label_3 = QtWidgets.QLabel(RT_Plotter)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 0, 1, 1, 1)
        self.dt_label = QtWidgets.QLabel(RT_Plotter)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.dt_label.sizePolicy().hasHeightForWidth())
        self.dt_label.setSizePolicy(sizePolicy)
        self.dt_label.setObjectName("dt_label")
        self.gridLayout.addWidget(self.dt_label, 0, 2, 1, 1)
        self.size_label = QtWidgets.QLabel(RT_Plotter)
        self.size_label.setObjectName("size_label")
        self.gridLayout.addWidget(self.size_label, 0, 5, 1, 1)
        self.menu_button = QtWidgets.QToolButton(RT_Plotter)
        self.menu_button.setContextMenuPolicy(QtCore.Qt.ActionsContextMenu)
        icon = QtGui.QIcon.fromTheme("format-justify-fill")
        self.menu_button.setIcon(icon)
        self.menu_button.setObjectName("menu_button")
        self.gridLayout.addWidget(self.menu_button, 0, 13, 1, 1)
        self.label_5 = QtWidgets.QLabel(RT_Plotter)
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 0, 4, 1, 1)
        self.label_2 = QtWidgets.QLabel(RT_Plotter)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 0, 10, 1, 1)
        self.dt_slider = QtWidgets.QSlider(RT_Plotter)
        self.dt_slider.setMinimum(5)
        self.dt_slider.setMaximum(100)
        self.dt_slider.setSliderPosition(50)
        self.dt_slider.setOrientation(QtCore.Qt.Horizontal)
        self.dt_slider.setObjectName("dt_slider")
        self.gridLayout.addWidget(self.dt_slider, 0, 3, 1, 1)
        self.label = QtWidgets.QLabel(RT_Plotter)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 0, 7, 1, 1)
        self.size_slider = QtWidgets.QSlider(RT_Plotter)
        self.size_slider.setMinimum(10)
        self.size_slider.setMaximum(240)
        self.size_slider.setProperty("value", 30)
        self.size_slider.setSliderPosition(30)
        self.size_slider.setOrientation(QtCore.Qt.Horizontal)
        self.size_slider.setObjectName("size_slider")
        self.gridLayout.addWidget(self.size_slider, 0, 6, 1, 1)
        self.scale_spin = QtWidgets.QDoubleSpinBox(RT_Plotter)
        self.scale_spin.setMinimum(-9999.0)
        self.scale_spin.setMaximum(9999.99)
        self.scale_spin.setProperty("value", 1.0)
        self.scale_spin.setObjectName("scale_spin")
        self.gridLayout.addWidget(self.scale_spin, 0, 11, 1, 1)
        self.autoscale = QtWidgets.QPushButton(RT_Plotter)
        self.autoscale.setObjectName("autoscale")
        self.gridLayout.addWidget(self.autoscale, 0, 0, 1, 1)
        self.constant_input = QtWidgets.QLineEdit(RT_Plotter)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.constant_input.sizePolicy().hasHeightForWidth())
        self.constant_input.setSizePolicy(sizePolicy)
        self.constant_input.setMinimumSize(QtCore.QSize(45, 0))
        self.constant_input.setObjectName("constant_input")
        self.gridLayout.addWidget(self.constant_input, 0, 8, 1, 1)
        self.offset_spin = QtWidgets.QDoubleSpinBox(RT_Plotter)
        self.offset_spin.setMinimum(-9999.0)
        self.offset_spin.setMaximum(9999.99)
        self.offset_spin.setObjectName("offset_spin")
        self.gridLayout.addWidget(self.offset_spin, 0, 12, 1, 1)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout.addItem(spacerItem, 0, 9, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout)
        self.action_new_plot = QtWidgets.QAction(RT_Plotter)
        self.action_new_plot.setObjectName("action_new_plot")
        self.action_reset = QtWidgets.QAction(RT_Plotter)
        self.action_reset.setObjectName("action_reset")
        self.action_suspend = QtWidgets.QAction(RT_Plotter)
        self.action_suspend.setObjectName("action_suspend")
        self.action_restart = QtWidgets.QAction(RT_Plotter)
        self.action_restart.setObjectName("action_restart")
        self.action_close = QtWidgets.QAction(RT_Plotter)
        self.action_close.setObjectName("action_close")
        self.action_quit = QtWidgets.QAction(RT_Plotter)
        self.action_quit.setObjectName("action_quit")
        self.action_dark_background = QtWidgets.QAction(RT_Plotter)
        self.action_dark_background.setCheckable(True)
        self.action_dark_background.setObjectName("action_dark_background")

        self.retranslateUi(RT_Plotter)
        QtCore.QMetaObject.connectSlotsByName(RT_Plotter)

    def retranslateUi(self, RT_Plotter):
        _translate = QtCore.QCoreApplication.translate
        RT_Plotter.setWindowTitle(_translate("RT_Plotter", "Real-time plotter"))
        self.label_3.setText(_translate("RT_Plotter", "update"))
        self.dt_label.setText(_translate("RT_Plotter", "0.50 s"))
        self.size_label.setText(_translate("RT_Plotter", "30 s"))
        self.menu_button.setText(_translate("RT_Plotter", "..."))
        self.label_5.setText(_translate("RT_Plotter", "size"))
        self.label_2.setText(_translate("RT_Plotter", "scale next by"))
        self.dt_slider.setToolTip(_translate("RT_Plotter", "update time (s)"))
        self.label.setText(_translate("RT_Plotter", "constant"))
        self.size_slider.setToolTip(_translate("RT_Plotter", "display time interval"))
        self.scale_spin.setToolTip(_translate("RT_Plotter", "scaling factor"))
        self.autoscale.setToolTip(_translate("RT_Plotter", "restart autoscale on Y axis"))
        self.autoscale.setText(_translate("RT_Plotter", "auto scale"))
        self.constant_input.setToolTip(_translate("RT_Plotter", "enter a number and press enter to draw a line"))
        self.offset_spin.setToolTip(_translate("RT_Plotter", "offset"))
        self.action_new_plot.setText(_translate("RT_Plotter", "New plot"))
        self.action_new_plot.setShortcut(_translate("RT_Plotter", "Ctrl+N"))
        self.action_reset.setText(_translate("RT_Plotter", "Reset"))
        self.action_reset.setToolTip(_translate("RT_Plotter", "reset data for all plots"))
        self.action_reset.setShortcut(_translate("RT_Plotter", "Ctrl+L"))
        self.action_suspend.setText(_translate("RT_Plotter", "Suspend"))
        self.action_suspend.setToolTip(_translate("RT_Plotter", "freeze plotter view"))
        self.action_suspend.setShortcut(_translate("RT_Plotter", "Ctrl+S"))
        self.action_restart.setText(_translate("RT_Plotter", "Restart"))
        self.action_restart.setToolTip(_translate("RT_Plotter", "restart suspended plotter"))
        self.action_restart.setShortcut(_translate("RT_Plotter", "Ctrl+X"))
        self.action_close.setText(_translate("RT_Plotter", "Close"))
        self.action_close.setToolTip(_translate("RT_Plotter", "close plotter window"))
        self.action_close.setShortcut(_translate("RT_Plotter", "Ctrl+W"))
        self.action_quit.setText(_translate("RT_Plotter", "Quit"))
        self.action_quit.setToolTip(_translate("RT_Plotter", "quit realtime plotter (close all windows)"))
        self.action_quit.setShortcut(_translate("RT_Plotter", "Ctrl+Q"))
        self.action_dark_background.setText(_translate("RT_Plotter", "Dark background"))
        self.action_dark_background.setToolTip(_translate("RT_Plotter", "change background from white to black"))
        self.action_dark_background.setShortcut(_translate("RT_Plotter", "Ctrl+B"))
