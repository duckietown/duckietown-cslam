#!/usr/bin/env python
from cslam_diagnostics.diagnostics_ui import Ui_MainWindow
import sys
import rospy
from PyQt4 import QtGui,QtCore
import signal
import os
from geometry_msgs.msg import TransformStamped

class DiagnosticsPanel(QtGui.QMainWindow):

    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.devices = {}

        rospy.Subscriber('/poses_acquisition/poses', TransformStamped, self.poses_callback)
    
        header = self.ui.tableWidget.horizontalHeader()
        header.setResizeMode(0, QtGui.QHeaderView.Stretch)
        header.setResizeMode(1, QtGui.QHeaderView.ResizeToContents)
        header.setResizeMode(2, QtGui.QHeaderView.ResizeToContents)

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(30)


    def poses_callback(self, msg):
        this_device = msg.header.frame_id
        if this_device not in self.devices:
            self.devices[this_device] = {}
            self.devices[this_device]['first'] = msg.header.stamp.secs + \
                msg.header.stamp.nsecs * 10**-9

        self.devices[this_device]['last'] = msg.header.stamp.secs + \
                msg.header.stamp.nsecs * 10**-9
        print msg

    def update(self):
        while self.ui.tableWidget.rowCount() < len(self.devices.keys()):
            self.ui.tableWidget.insertRow(0)

        for itr, device_id in enumerate(sorted(self.devices.keys())):
            
            time_now = rospy.get_time()
            diff = time_now - self.devices[device_id]['last']

            self.ui.tableWidget.setItem(itr , 0, \
                QtGui.QTableWidgetItem(device_id))
            
            self.ui.tableWidget.setItem(itr , 1, \
                QtGui.QTableWidgetItem(str(self.devices[device_id]['last'])))            
            
            self.ui.tableWidget.setItem(itr , 2, \
                QtGui.QTableWidgetItem(str(diff)))
            
            self.ui.tableWidget.setItem(itr , 3, \
                QtGui.QTableWidgetItem(str(self.devices[device_id]['first'])))
            
            status_val = 'OK'
            color = QtGui.QColor(50,255,50)

            if diff > 5 and diff <= 10:
                status_val = 'DELAYED'
                color = QtGui.QColor(255,255,50)

            elif diff > 10 and diff <= 20:
                status_val = 'V. DELAYED'
                color = QtGui.QColor(255,180,50)

            elif diff > 20:
                status_val = 'ERROR'
                color = QtGui.QColor(255,50,50)

            self.ui.tableWidget.setItem(itr , 4, \
                QtGui.QTableWidgetItem(status_val))

            self.ui.tableWidget.item(itr, 4).setBackground(color)
            

def main():
    rospy.init_node("cslam_debugger")
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtGui.QApplication(sys.argv)
    diag = DiagnosticsPanel()
    diag.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
