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
from PySide6.QtWidgets import (QApplication, QComboBox, QFrame, QGridLayout,
    QHBoxLayout, QLabel, QMainWindow, QSizePolicy,
    QTabWidget, QWidget)

from pyqtgraph import PlotWidget

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
        self.plot3_combobox = QComboBox(self.tab)
        self.plot3_combobox.setObjectName(u"plot3_combobox")

        self.gridLayout_3.addWidget(self.plot3_combobox, 4, 1, 1, 1)

        self.titleFrame = QFrame(self.tab)
        self.titleFrame.setObjectName(u"titleFrame")
        self.titleFrame.setMaximumSize(QSize(16777215, 100))
        self.titleFrame.setFrameShape(QFrame.Box)
        self.titleFrame.setFrameShadow(QFrame.Plain)
        self.titleFrame.setLineWidth(1)
        self.horizontalLayout_2 = QHBoxLayout(self.titleFrame)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.label_2 = QLabel(self.titleFrame)
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

        self.label = QLabel(self.titleFrame)
        self.label.setObjectName(u"label")
        font1 = QFont()
        font1.setPointSize(30)
        font1.setBold(True)
        self.label.setFont(font1)
        self.label.setAlignment(Qt.AlignCenter)

        self.horizontalLayout.addWidget(self.label)


        self.horizontalLayout_2.addLayout(self.horizontalLayout)


        self.gridLayout_3.addWidget(self.titleFrame, 0, 0, 1, 4)

        self.plot2_combobox = QComboBox(self.tab)
        self.plot2_combobox.setObjectName(u"plot2_combobox")

        self.gridLayout_3.addWidget(self.plot2_combobox, 2, 2, 1, 1)

        self.plot4 = PlotWidget(self.tab)
        self.plot4.setObjectName(u"plot4")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.plot4.sizePolicy().hasHeightForWidth())
        self.plot4.setSizePolicy(sizePolicy)

        self.gridLayout_3.addWidget(self.plot4, 3, 2, 1, 1)

        self.plot4_combobox = QComboBox(self.tab)
        self.plot4_combobox.setObjectName(u"plot4_combobox")

        self.gridLayout_3.addWidget(self.plot4_combobox, 4, 2, 1, 1)

        self.plot3 = PlotWidget(self.tab)
        self.plot3.setObjectName(u"plot3")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.plot3.sizePolicy().hasHeightForWidth())
        self.plot3.setSizePolicy(sizePolicy1)

        self.gridLayout_3.addWidget(self.plot3, 3, 1, 1, 1)

        self.plot2 = PlotWidget(self.tab)
        self.plot2.setObjectName(u"plot2")
        sizePolicy.setHeightForWidth(self.plot2.sizePolicy().hasHeightForWidth())
        self.plot2.setSizePolicy(sizePolicy)

        self.gridLayout_3.addWidget(self.plot2, 1, 2, 1, 1)

        self.plot1 = PlotWidget(self.tab)
        self.plot1.setObjectName(u"plot1")
        sizePolicy1.setHeightForWidth(self.plot1.sizePolicy().hasHeightForWidth())
        self.plot1.setSizePolicy(sizePolicy1)
        self.plot1.setMinimumSize(QSize(0, 0))

        self.gridLayout_3.addWidget(self.plot1, 1, 1, 1, 1)

        self.plot1_combobox = QComboBox(self.tab)
        self.plot1_combobox.setObjectName(u"plot1_combobox")

        self.gridLayout_3.addWidget(self.plot1_combobox, 2, 1, 1, 1)

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
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), QCoreApplication.translate("MainWindow", u"Tab 1", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), QCoreApplication.translate("MainWindow", u"Tab 2", None))
    # retranslateUi

