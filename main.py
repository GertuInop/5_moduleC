from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import Qt
from motion.core import RobotControl, LedLamp, Waypoint
from motion.robot_control import InterpreterStates
import sys, os, math, design, time, datetime

class MainWindow(QtWidgets.QMainWindow, design.Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.robot = RobotControl()
        self.lamp = LedLamp()

        self.logs = []
        self.e_logs = []
        self.log_model = QtCore.QStringListModel()
        self.lvLogs.setModel(self.log_model)

        self.add_log('Приложение запущено')

    def add_log(self, msg: str):
        time = datetime.datetime.now().strftime('%H:%M:%S')
        log = f'[{time}] - {msg}'
        self.logs.append(log)
        self.log_model.setStringList(self.logs)

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()