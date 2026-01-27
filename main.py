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

        self.gripper = 0
        self.onOff = False
        self.connect = False
        self.conveyer = False
        self.pause = False
        self.cartActive = False
        self.jointActive = False

        self.labels = [self.lWait, self.lWork, self.lPause, self.lStop]
        self.sliders = [self.s1, self.s2, self.s3, self.s4, self.s5, self.s6]

        self.btnOnOff.clicked.connect(self.on_off)
        self.btnPause.clicked.connect(self.set_pause)
        self.btnStop.clicked.connect(self.set_stop)
        self.btnStartConveyer.clicked.connect(self.start_conveyer)
        self.btnToStart.clicked.connect(self.to_start)
        self.btnManualCart.clicked.connect(self.m_cart)
        self.btnManualJoint.clicked.connect(self.m_joint)
        self.btnGripperOnOff.clicked.connect(self.gripper_on_off)
        self.btnSaveStats.clicked.connect(self.save_stats)
        self.btnSaveLogs.clicked.connect(self.save_logs)
        self.btnSaveELogs.clicked.connect(self.save_e_logs)

        self.sldLinerTrack.valueChanged.connect(self.liner_track)

        self.s1.sliderReleased.connect(self.sld_to_zero)
        self.s2.sliderReleased.connect(self.sld_to_zero)
        self.s3.sliderReleased.connect(self.sld_to_zero)
        self.s4.sliderReleased.connect(self.sld_to_zero)
        self.s5.sliderReleased.connect(self.sld_to_zero)
        self.s6.sliderReleased.connect(self.sld_to_zero)

        self.sb11.valueChanged.connect(self.save_stats_1)
        self.sb21.valueChanged.connect(self.save_stats_1)
        self.sb12.valueChanged.connect(self.save_stats_2)
        self.sb22.valueChanged.connect(self.save_stats_2)
        self.sb13.valueChanged.connect(self.save_stats_3)
        self.sb23.valueChanged.connect(self.save_stats_3)

        self.logs = []
        self.e_logs = []
        self.log_model = QtCore.QStringListModel()
        self.lvLogs.setModel(self.log_model)

        self.pose = QtCore.QTimer()
        self.pose.timeout.connect(self.update_pose)

        self.motors = QtCore.QTimer()
        self.motors.timeout.connect(self.update_motors)

        self.cartTimer = QtCore.QTimer()
        self.cartTimer.timeout.connect(self.update_cart)
        self.jointTimer = QtCore.QTimer()
        self.jointTimer.timeout.connect(self.update_joint)

        self.savePoseLogsTime = QtCore.QTimer()
        self.savePoseLogsTime.timeout.connect(self.save_pose_to_log)

        self.add_log('Приложение запущено')

    def on_off(self):
        self.robot.connect()
        if not self.onOff:
            self.robot.engage()
            self.set_lamp_code('0100')
            # self.pose.start(500)
            # self.motors.start(500)
            self.savePoseLogsTime.start(10000)
            self.onOff = True
            self.sldLinerTrack.setEnabled(True)
            self.btnOnOff.setText('Off')
            self.add_log('Робот запущен')
        else:
            self.robot.moveToInitialPose()
            self.pose.stop()
            self.motors.stop()
            self.savePoseLogsTime.stop()
            QtWidgets.QMessageBox.about(self, 'Info', 'Робот завершает работу, подождите некоторое время')
            time.sleep(5)
            self.robot.disengage()
            self.set_lamp_code('0000')
            self.onOff = False
            self.btnOnOff.setText('On')
            self.sldLinerTrack.setEnabled(False)
            self.add_log('Робот выключен')
    
    def set_pause(self):
        if self.onOff:
            if not self.pause:
                self.robot.pause()
                self.set_lamp_code('0010')
                self.pause = True
                self.btnPause.setText('Activate')
                self.add_log('Робот поставлен на паузу')
            else:
                self.robot.play()
                self.set_lamp_code('0100')
                self.pause = False
                self.btnPause.setText('Pause')
                self.add_log('Робот возобнавлён к работе')
        else:
            return QtWidgets.QMessageBox.warning(self, 'Error', 'Робот не запущен')
    
    def set_stop(self):
        if self.onOff:
            self.robot.addConveyerState(0)
            self.robot.conveyer_stop()
            self.pose.stop()
            self.motors.stop()
            self.savePoseLogsTime.stop()
            self.robot.stop()
            self.set_lamp_code('0001')
            self.btnPause.setText('Pause')
            self.btnOnOff.setText('On')
            self.onOff = False
            self.pause = False
            self.conveyer = False
            self.add_e_log('Робот аварийно остановлен')
        else:
            return QtWidgets.QMessageBox.warning(self, 'Error', 'Робот не запущен')
    
    def start_conveyer(self):
        if self.onOff:
            if not self.conveyer:
                self.robot.conveyer_start()
                self.robot.addConveyerState(1)
                self.conveyer = True
                self.btnStartConveyer.setText('Stop Conveyer')
                self.add_log('Конвейер запущен')
            else:
                self.robot.conveyer_stop()
                self.robot.addConveyerState(0)
                self.conveyer = False
                self.btnStartConveyer.setText('Start Conveyer')
                self.add_log('Конвейер остановлен')
        else:
            return QtWidgets.QMessageBox.warning(self, 'Error', 'Робот не запущен')
    
    def to_start(self):
        if self.onOff:
            self.robot.moveToInitialPose()
            self.add_log('Робот движется к началу')
        else:
            return QtWidgets.QMessageBox.warning(self, 'Error', 'Робот не запущен')
        
    def m_cart(self):
        if self.onOff:
            if not self.cartActive:
                self.robot.manualCartMode()
                # self.cart_connect()
                self.cartTimer.start(200)
                self.cartActive = True
                self.update_cart()
                for sld in self.sliders[:3]:
                    sld.setEnabled(True)
                self.btnManualJoint.setEnabled(False)
                self.btnManualCart.setText('Stop Cart')
                self.add_log('Ручной режим Декарта включён')
            else:
                # self.cart_disconnect()
                self.cartTimer.stop()
                self.cartActive = False
                self.update_cart()
                for sld in self.sliders:
                    sld.setEnabled(False)
                self.btnManualJoint.setEnabled(True)
                self.btnManualCart.setText('Manual Cart')
                self.add_log('Ручной режим Декарта выключён')
        else:
            return QtWidgets.QMessageBox.warning(self, 'Error', 'Робот не запущен')
    
    def m_joint(self):
        if self.onOff:
            if not self.jointActive:
                self.robot.manualJointMode()
                # self.joint_connect()
                self.jointTimer.start(200)
                self.jointActive = True
                self.update_joint()
                self.btnManualCart.setEnabled(False)
                self.btnManualJoint.setText('Stop Joint')
                for sld in self.sliders:
                    sld.setEnabled(True)
                self.add_log('Ручной режим Сочленений включён')
            else:
                # self.joint_disconnect()
                self.jointTimer.stop()
                self.jointActive = False
                self.update_joint()
                self.btnManualCart.setEnabled(True)
                self.btnManualJoint.setText('Manual Joint')
                for sld in self.sliders:
                    sld.setEnabled(False)
                self.add_log('Ручной режим Сочленений выключён')
        else:
            return QtWidgets.QMessageBox.warning(self, 'Error', 'Робот не запущен')
    
    def cart_connect(self):
        self.s1.valueChanged.connect(self.update_cart)
        self.s2.valueChanged.connect(self.update_cart)
        self.s3.valueChanged.connect(self.update_cart)

    def cart_disconnect(self):
        self.s1.valueChanged.disconnect(self.update_cart)
        self.s2.valueChanged.disconnect(self.update_cart)
        self.s3.valueChanged.disconnect(self.update_cart)

    def joint_connect(self):
        self.s1.valueChanged.connect(self.update_joint)
        self.s2.valueChanged.connect(self.update_joint)
        self.s3.valueChanged.connect(self.update_joint)
        self.s4.valueChanged.connect(self.update_joint)
        self.s5.valueChanged.connect(self.update_joint)
        self.s6.valueChanged.connect(self.update_joint)

    def joint_disconnect(self):
        self.s1.valueChanged.disconnect(self.update_joint)
        self.s2.valueChanged.disconnect(self.update_joint)
        self.s3.valueChanged.disconnect(self.update_joint)
        self.s4.valueChanged.disconnect(self.update_joint)
        self.s5.valueChanged.disconnect(self.update_joint)
        self.s6.valueChanged.disconnect(self.update_joint)

    def update_cart(self):
        v1 = self.s1.value() / 1000.0
        v2 = self.s2.value() / 1000.0
        v3 = self.s3.value() / 1000.0
        v = [v1, v2, v3, 0.0, 0.0, 0.0]
        self.robot.setCartesianVelocity(v)
        print(v)

    def update_joint(self):
        v1 = self.s1.value() / 1000.0
        v2 = self.s2.value() / 1000.0
        v3 = self.s3.value() / 1000.0
        v4 = self.s4.value() / 1000.0
        v5 = self.s5.value() / 1000.0
        v6 = self.s6.value() / 1000.0
        v = [v1, v2, v3, v4, v5, v6]
        self.robot.setJointVelocity(v)
        print(v)

    def gripper_on_off(self):
        if self.onOff:
            if self.gripper == 0:
                self.robot.addToolState(1)
                self.gripper = 1
                self.btnGripperOnOff.setText('Off')
                self.add_log('Схват активен')
            elif self.gripper == 1:
                self.robot.addToolState(0)
                self.gripper = 0
                self.btnGripperOnOff.setText('On')
                self.add_log('Схват выключен')
        else:
            return QtWidgets.QMessageBox.warning(self, 'Error', 'Робот не запущен')

    def liner_track(self, val):
        pos = val / 10.0
        try:
            self.robot.addLinearTrackMove(pos)
            self.add_log(f'Линейный трек равен: {pos:.1}')
        except:
            return QtWidgets.QMessageBox.warning(self, 'Error', 'Ошибка изменения линейного трека')
        
    def update_pose(self):
        pose = self.robot.getToolPosition()

        self.twActualToolPose.setItem(0, 0, QtWidgets.QTableWidgetItem(round(pose[0], 3)))
        self.twActualToolPose.setItem(0, 1, QtWidgets.QTableWidgetItem(round(pose[1], 3)))
        self.twActualToolPose.setItem(0, 2, QtWidgets.QTableWidgetItem(round(pose[2], 3)))
        self.twActualToolPose.setItem(0, 3, QtWidgets.QTableWidgetItem(self.gripper))

    def save_pose_to_log(self):
        pose = self.robot.getToolPosition()
        self.add_log(f'Текущая позиция инструмента: [{pose[0]}, {pose[1]}, {pose[2]}]')

    def update_motors(self):
        ticks = self.robot.getMotorPositionTick()
        radians = self.robot.getMotorPositionRadians()
        degrees = [math.degrees(r) for r in radians]
        temperature = self.robot.getActualTemperature()

        for i in range(6):
            self.twMotorsStates.setItem(0, i, QtWidgets.QTableWidgetItem(round(ticks[i], 3)))
        for i in range(6):
            self.twMotorsStates.setItem(1, i, QtWidgets.QTableWidgetItem(round(radians[i], 3)))
        for i in range(6):
            self.twMotorsStates.setItem(2, i, QtWidgets.QTableWidgetItem(round(degrees[i], 3)))
        for i in range(6):
            self.twMotorsStates.setItem(3, i, QtWidgets.QTableWidgetItem(round(temperature[i], 3)))

    def save_logs(self):
        filename = self.leLogsPath.text().strip()
        if not filename:
            filename, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, 'Select folder and name your file', QtCore.QDir.homePath(), 'Text Files (*.txt);; All Files (*)'
            )
            if not filename:
                return QtWidgets.QMessageBox.warning(self, 'Error', 'Назовите свой файл')
        
        filename = filename if filename.endswith('.txt') else f'{filename}.txt'
        self.leLogsPath.setText(filename)

        try:
            with open(filename, 'a', encoding='UTF-8') as f:
                f.write('\n'.join(self.logs) + '\n')
            self.add_log(f'Логи сохранены по пути {filename}')
        except:
            QtWidgets.QMessageBox.warning(self, 'Error', 'Ошибка сохранения логов')

    def save_e_logs(self):
        filename = self.leELogsPath.text().strip()
        if not filename:
            filename, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, 'Select folder and name your file', QtCore.QDir.homePath(), 'Text Files (*.txt);; All Files (*)'
            )
            if not filename:
                return QtWidgets.QMessageBox.warning(self, 'Error', 'Назовите свой файл')
        
        filename = filename if filename.endswith('.txt') else f'{filename}.txt'
        self.leELogsPath.setText(filename)

        try:
            with open(filename, 'a', encoding='UTF-8') as f:
                f.write('\n'.join(self.e_logs) + '\n')
            self.add_log(f'Аварийные логи сохранены по пути {filename}')
        except:
            QtWidgets.QMessageBox.warning(self, 'Error', 'Ошибка сохранения аварийных логов')
        
    def save_stats(self):
        filename = self.leStatsPath.text().strip()
        if not filename:
            filename, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, 'Select folder and name your file', QtCore.QDir.homePath(), 'Excel Files (*.xlsx);; All Files (*)'
            )
            if not filename:
                return QtWidgets.QMessageBox.warning(self, 'Error', 'Назовите свой файл')
        
        filename = filename if filename.endswith('.xlsx') else f'{filename}.xlsx'
        self.leStatsPath.setText(filename)

        rows = [
            [self.sb11.value(), self.sb21.value(), self.leTime31.text().strip()],
            [self.sb12.value(), self.sb22.value(), self.leTime32.text().strip()],
            [self.sb13.value(), self.sb23.value(), self.leTime33.text().strip()]
        ]

        try:
            from openpyxl import Workbook
            wb = Workbook(); ws = wb.active
            for r in rows: ws.append(r)
            wb.save(filename)
            self.add_log(f'Статистика сохранена по пути {filename}')
        except:
            QtWidgets.QMessageBox.warning(self, 'Error', 'Ошибка сохранения статистики')

    def save_stats_1(self):
        time = datetime.datetime.now().strftime('%H:%M:%S')
        self.leTime31.setText(time)
        self.save_stats()
    
    def save_stats_2(self):
        time = datetime.datetime.now().strftime('%H:%M:%S')
        self.leTime32.setText(time)
        self.save_stats()

    def save_stats_3(self):
        time = datetime.datetime.now().strftime('%H:%M:%S')
        self.leTime33.setText(time)
        self.save_stats()

    def set_lamp_code(self, code: str):
        self.lamp.setLamp(code)

        for el in self.labels: el.setStyleSheet('')

        match code:
            case '1000': self.lWait.setStyleSheet('background-color: blue')
            case '0100': self.lWork.setStyleSheet('background-color: green')
            case '0010': self.lPause.setStyleSheet('background-color: yellow')
            case '0001': self.lStop.setStyleSheet('background-color: red')

    def add_log(self, msg: str):
        time = datetime.datetime.now().strftime('%H:%M:%S')
        log = f'[{time}] - {msg}'
        self.logs.append(log)
        self.log_model.setStringList(self.logs)

    def add_e_log(self, msg: str):
        time = datetime.datetime.now().strftime('%H:%M:%S')
        log = f'[{time}] - {msg}'
        self.logs.append(log)
        self.e_logs.append(log)
        self.log_model.setStringList(self.logs)

    def sld_to_zero(self):
        for sld in self.sliders:
            sld.setValue(0)

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()