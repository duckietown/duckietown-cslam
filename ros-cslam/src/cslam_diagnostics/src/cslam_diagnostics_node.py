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

        self.at_devices = {}
        self.odom_devices = {}

        rospy.Subscriber('/poses_acquisition/poses', TransformStamped, self.poses_callback)
        rospy.Subscriber('/poses_acquisition/odometry', TransformStamped, self.poses_callback)
    

        self.setup_table(self.ui.table_apriltags)
        self.setup_table(self.ui.table_odometry)
        
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(30)

    def setup_table(self, table):
        header = table.horizontalHeader()
        table.setColumnWidth(0,150)        
        table.setColumnWidth(1,121)
        table.setColumnWidth(2,121)
        table.setColumnWidth(3,120)

    def poses_callback(self, msg):
        this_device = msg.header.frame_id
        child_device = msg.child_frame_id
        
        if child_device == this_device:
            if this_device not in self.odom_devices:
                self.odom_devices[this_device] = {}
                self.odom_devices[this_device]['first'] = msg.header.stamp.secs + \
                    msg.header.stamp.nsecs * 10**-9
                self.odom_devices[this_device]['timestamps'] = []
            
            self.odom_devices[this_device]['timestamps'].append(rospy.get_time())
            self.odom_devices[this_device]['last'] = msg.header.stamp.secs + \
                    msg.header.stamp.nsecs * 10**-9

        else:
            if this_device not in self.at_devices:
                self.at_devices[this_device] = {}
                self.at_devices[this_device]['first'] = msg.header.stamp.secs + \
                    msg.header.stamp.nsecs * 10**-9
                self.at_devices[this_device]['timestamps'] = []

            self.at_devices[this_device]['timestamps'].append(rospy.get_time())
            self.at_devices[this_device]['last'] = msg.header.stamp.secs + \
                    msg.header.stamp.nsecs * 10**-9

    def update_table(self,table,devices):
        while table.rowCount() < len(devices.keys()):
            table.insertRow(0)

        for itr, device_id in enumerate(sorted(devices.keys())):
            
            time_now = rospy.get_time()
            diff = time_now - devices[device_id]['last']

            table.setItem(itr , 0, \
                QtGui.QTableWidgetItem(device_id))
            
            table.setItem(itr , 1, \
                QtGui.QTableWidgetItem(str(devices[device_id]['last'])))            
            
            table.setItem(itr , 2, \
                QtGui.QTableWidgetItem(str(diff)))
            
            trimmed_timestamps = [t for t in devices[device_id]['timestamps'] if rospy.get_time() - t <= 5]
            table.setItem(itr , 3, \
                QtGui.QTableWidgetItem(str(len(trimmed_timestamps))))
            devices[device_id]['timestamps'] = trimmed_timestamps
            
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

            table.setItem(itr , 4, \
                QtGui.QTableWidgetItem(status_val))

            table.item(itr, 4).setBackground(color)

    def update(self):
        self.update_table(self.ui.table_apriltags,self.at_devices)
        self.update_table(self.ui.table_odometry,self.odom_devices)

            

def main():
    rospy.init_node("cslam_diagnostics")
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtGui.QApplication(sys.argv)
    diag = DiagnosticsPanel()
    diag.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
