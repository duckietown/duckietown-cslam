# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'diagnostics_ui.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(670, 547)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.tabWidget = QtGui.QTabWidget(self.centralwidget)
        self.tabWidget.setGeometry(QtCore.QRect(12, 19, 651, 471))
        self.tabWidget.setObjectName(_fromUtf8("tabWidget"))
        self.tab = QtGui.QWidget()
        self.tab.setObjectName(_fromUtf8("tab"))
        self.table_apriltags = QtGui.QTableWidget(self.tab)
        self.table_apriltags.setGeometry(QtCore.QRect(10, 10, 631, 421))
        self.table_apriltags.setObjectName(_fromUtf8("table_apriltags"))
        self.table_apriltags.setColumnCount(5)
        self.table_apriltags.setRowCount(0)
        item = QtGui.QTableWidgetItem()
        self.table_apriltags.setHorizontalHeaderItem(0, item)
        item = QtGui.QTableWidgetItem()
        self.table_apriltags.setHorizontalHeaderItem(1, item)
        item = QtGui.QTableWidgetItem()
        self.table_apriltags.setHorizontalHeaderItem(2, item)
        item = QtGui.QTableWidgetItem()
        self.table_apriltags.setHorizontalHeaderItem(3, item)
        item = QtGui.QTableWidgetItem()
        self.table_apriltags.setHorizontalHeaderItem(4, item)
        self.tabWidget.addTab(self.tab, _fromUtf8(""))
        self.tab_2 = QtGui.QWidget()
        self.tab_2.setObjectName(_fromUtf8("tab_2"))
        self.table_odometry = QtGui.QTableWidget(self.tab_2)
        self.table_odometry.setGeometry(QtCore.QRect(10, 10, 631, 421))
        self.table_odometry.setObjectName(_fromUtf8("table_odometry"))
        self.table_odometry.setColumnCount(5)
        self.table_odometry.setRowCount(0)
        item = QtGui.QTableWidgetItem()
        self.table_odometry.setHorizontalHeaderItem(0, item)
        item = QtGui.QTableWidgetItem()
        self.table_odometry.setHorizontalHeaderItem(1, item)
        item = QtGui.QTableWidgetItem()
        self.table_odometry.setHorizontalHeaderItem(2, item)
        item = QtGui.QTableWidgetItem()
        self.table_odometry.setHorizontalHeaderItem(3, item)
        item = QtGui.QTableWidgetItem()
        self.table_odometry.setHorizontalHeaderItem(4, item)
        self.tabWidget.addTab(self.tab_2, _fromUtf8(""))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 670, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(1)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "CSLAM Diagnostics", None))
        item = self.table_apriltags.horizontalHeaderItem(0)
        item.setText(_translate("MainWindow", "Device", None))
        item = self.table_apriltags.horizontalHeaderItem(1)
        item.setText(_translate("MainWindow", "Last Msg", None))
        item = self.table_apriltags.horizontalHeaderItem(2)
        item.setText(_translate("MainWindow", "Diff", None))
        item = self.table_apriltags.horizontalHeaderItem(3)
        item.setText(_translate("MainWindow", "First Msg", None))
        item = self.table_apriltags.horizontalHeaderItem(4)
        item.setText(_translate("MainWindow", "Status", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "Apriltags", None))
        item = self.table_odometry.horizontalHeaderItem(0)
        item.setText(_translate("MainWindow", "Device", None))
        item = self.table_odometry.horizontalHeaderItem(1)
        item.setText(_translate("MainWindow", "Last Msg", None))
        item = self.table_odometry.horizontalHeaderItem(2)
        item.setText(_translate("MainWindow", "Diff", None))
        item = self.table_odometry.horizontalHeaderItem(3)
        item.setText(_translate("MainWindow", "First Msg", None))
        item = self.table_odometry.horizontalHeaderItem(4)
        item.setText(_translate("MainWindow", "Status", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "Odometry", None))

