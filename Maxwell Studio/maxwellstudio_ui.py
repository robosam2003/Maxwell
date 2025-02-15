# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'maxwellstudio.ui'
##
## Created by: Qt User Interface Compiler version 6.8.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QFrame, QGridLayout, QHBoxLayout,
    QLabel, QMainWindow, QSizePolicy, QTabWidget,
    QWidget)

from pyqtgraph import PlotWidget
from rotorview import RotorView

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1262, 806)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.gridLayout_2 = QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tab = QWidget()
        self.tab.setObjectName(u"tab")
        self.gridLayout_3 = QGridLayout(self.tab)
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.current_plots = PlotWidget(self.tab)
        self.current_plots.setObjectName(u"current_plots")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.current_plots.sizePolicy().hasHeightForWidth())
        self.current_plots.setSizePolicy(sizePolicy)
        self.current_plots.setMinimumSize(QSize(0, 0))

        self.gridLayout_3.addWidget(self.current_plots, 1, 1, 1, 1)

        self.voltage_plots = PlotWidget(self.tab)
        self.voltage_plots.setObjectName(u"voltage_plots")
        sizePolicy.setHeightForWidth(self.voltage_plots.sizePolicy().hasHeightForWidth())
        self.voltage_plots.setSizePolicy(sizePolicy)

        self.gridLayout_3.addWidget(self.voltage_plots, 2, 1, 1, 1)

        self.rotorview = RotorView(self.tab)
        self.rotorview.setObjectName(u"rotorview")

        self.gridLayout_3.addWidget(self.rotorview, 1, 2, 2, 1)

        self.frame = QFrame(self.tab)
        self.frame.setObjectName(u"frame")
        self.frame.setMaximumSize(QSize(16777215, 100))
        self.frame.setFrameShape(QFrame.Box)
        self.frame.setFrameShadow(QFrame.Plain)
        self.frame.setLineWidth(1)
        self.horizontalLayout_2 = QHBoxLayout(self.frame)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.label_2 = QLabel(self.frame)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setMaximumSize(QSize(200, 16777215))
        palette = QPalette()
        brush = QBrush(QColor(53, 132, 228, 255))
        brush.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.WindowText, brush)
        palette.setBrush(QPalette.Inactive, QPalette.WindowText, brush)
        brush1 = QBrush(QColor(190, 190, 190, 255))
        brush1.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Disabled, QPalette.WindowText, brush1)
        self.label_2.setPalette(palette)
        font = QFont()
        font.setFamilies([u"Ubuntu Sans Mono"])
        font.setPointSize(30)
        font.setBold(True)
        self.label_2.setFont(font)

        self.horizontalLayout.addWidget(self.label_2)

        self.label = QLabel(self.frame)
        self.label.setObjectName(u"label")
        font1 = QFont()
        font1.setPointSize(30)
        font1.setBold(True)
        self.label.setFont(font1)
        self.label.setAlignment(Qt.AlignCenter)

        self.horizontalLayout.addWidget(self.label)


        self.horizontalLayout_2.addLayout(self.horizontalLayout)


        self.gridLayout_3.addWidget(self.frame, 0, 0, 1, 3)

        self.fault_codes_label = QLabel(self.tab)
        self.fault_codes_label.setObjectName(u"fault_codes_label")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.fault_codes_label.sizePolicy().hasHeightForWidth())
        self.fault_codes_label.setSizePolicy(sizePolicy1)
        font2 = QFont()
        font2.setPointSize(13)
        font2.setBold(True)
        self.fault_codes_label.setFont(font2)

        self.gridLayout_3.addWidget(self.fault_codes_label, 3, 1, 1, 2)

        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QWidget()
        self.tab_2.setObjectName(u"tab_2")
        self.tabWidget.addTab(self.tab_2, "")

        self.gridLayout_2.addWidget(self.tabWidget, 0, 0, 2, 1)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        self.tabWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"ROBOSAM", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Maxwell Studio", None))
        self.fault_codes_label.setText(QCoreApplication.translate("MainWindow", u"Fault Codes:\n"
"\n"
"", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), QCoreApplication.translate("MainWindow", u"Tab 1", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), QCoreApplication.translate("MainWindow", u"Tab 2", None))
    # retranslateUi

