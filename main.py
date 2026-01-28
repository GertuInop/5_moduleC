from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import Qt
from motion.core import RobotControl, LedLamp, Waypoint
from motion.robot_control import InterpreterStates
import sys, os, math, design, time, datetime, cv2
import numpy as np
from ultralytics import YOLO

class MainWindow(QtWidgets.QMainWindow, design.Ui_MainWindow):
    DEFAULT_RX = math.pi / 2
    DEFAULT_RY = 0
    DEFAULT_RZ = math.pi
    
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.robot = RobotControl()
        self.lamp = LedLamp('192.168.2.101')

        self.gripper = 0
        self.onOff = False
        self.connect = False
        self.conveyer = False
        self.pause = False
        self.cartActive = False
        self.jointActive = False

        self.labels = [self.lWait, self.lWork, self.lPause, self.lStop]
        self.sliders = [self.s1, self.s2, self.s3, self.s4, self.s5, self.s6]

        self.start_pos = [0.85, -0.19, 0.76]

        self.takeCell = [1.07, -0.54, -0.24]
        self.takeTrack = 0.0

        self.cells = {
            1: [1.05, 0.08, -0.11], 2: [1.05, 0.26, -0.11]
        }
        self.cellTracks = {1:1,2:1}

        self.rejectCell = [1.05, -0.17, -0.109]
        self.rejectTrack = 0.0

        self.allowed = {'Box1': [1,2], 'Box2': [], 'Box3': []}
        self.occupied = {i: False for i in range(1,3)}
        self.start_remove_slot = None

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

        self.btnAddObject.clicked.connect(self.add_from_cb)
        self.btnDeleteLast.clicked.connect(self.delete_last)
        self.btnClearSession.clicked.connect(self.clear_list)
        self.btnRunSession.clicked.connect(self.run_session)
        self.btnDownloadSession.clicked.connect(self.download_session)
        self.btnLoadSession.clicked.connect(self.load_session)
        self.btnSlotsClear.clicked.connect(self.clear_slots)

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

        self.session = []
        self.session_model = QtCore.QStringListModel()
        self.lvSession.setModel(self.session_model)

        self.busy = False
        self.moves_enabled = False

        self.cls2cat = {
            "": "Box1",
            "": "Box1",
            "": "Reject"
        }

        self.btnCamera.clicked.connect(self.in_next_update)
        self.btnVideo.clicked.connect(self.in_next_update)
        self.btnDetectObjects.clicked.connect(self.in_next_update)

        # self.yolo = YOLO('best.pt')
        self.cap = None
        self.videoActive = False
        self.cameraActive = False
        self.det_objects = False
        self.det_was_active = False
        self.people_present = False
        self._seen, self._ttl = {}, 2000

        self.frame_timer = QtCore.QTimer(self)
        self.frame_timer.timeout.connect(self.tick)

        self.add_log('Приложение запущено')

    def on_off(self):
        self.robot.connect()
        if not self.onOff:
            self.robot.engage()
            self.set_lamp_code('0100')
            self.pose.start(500)
            self.motors.start(500)
            self.savePoseLogsTime.start(30000)
            self.onOff = True
            self.sldLinerTrack.setEnabled(True)
            self.btnOnOff.setText('Off')
            self.add_log('Робот запущен')
        else:
            self.robot.moveToInitialPose()
            self.pose.stop()
            self.motors.stop()
            self.jointTimer.stop()
            self.cartTimer.stop()
            self.savePoseLogsTime.stop()
            QtWidgets.QMessageBox.about(self, 'Info', 'Робот завершает работу, подождите некоторое время')
            time.sleep(5)
            self.robot.disengage()
            self.set_lamp_code('1000')
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
                self.cartTimer.start(150)
                self.cartActive = True
                self.update_cart()
                for sld in self.sliders[:3]:
                    sld.setEnabled(True)
                self.btnManualJoint.setEnabled(False)
                self.btnManualCart.setText('Stop Cart')
                self.add_log('Ручной режим Декарта включён')
            else:
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
                self.jointTimer.start(25)
                self.jointActive = True
                self.update_joint()
                self.btnManualCart.setEnabled(False)
                self.btnManualJoint.setText('Stop Joint')
                for sld in self.sliders:
                    sld.setEnabled(True)
                self.add_log('Ручной режим Сочленений включён')
            else:
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

    def update_cart(self):
        v1 = self.s1.value() / 100.0
        v2 = self.s2.value() / 100.0
        v3 = self.s3.value() / 100.0
        v = [v1, v2, v3, 0.0, 0.0, 0.0]
        self.robot.setCartesianVelocity(v)

    def update_joint(self):
        v1 = self.s1.value() / 100.0
        v2 = self.s2.value() / 100.0
        v3 = self.s3.value() / 100.0
        v4 = self.s4.value() / 100.0
        v5 = self.s5.value() / 100.0
        v6 = self.s6.value() / 100.0
        v = [v1, v2, v3, v4, v5, v6]
        self.robot.setJointVelocity(v)

    def gripper_on_off(self):
        if self.onOff:
            if self.gripper == 0:
                self.robot.toolON()
                self.gripper = 1
                self.btnGripperOnOff.setText('Off')
                self.add_log('Схват активен')
            elif self.gripper == 1:
                self.robot.toolOFF()
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

        self.twActualToolPose.setItem(0, 0, QtWidgets.QTableWidgetItem(str(round(pose[0], 3))))
        self.twActualToolPose.setItem(0, 1, QtWidgets.QTableWidgetItem(str(round(pose[1], 3))))
        self.twActualToolPose.setItem(0, 2, QtWidgets.QTableWidgetItem(str(round(pose[2], 3))))
        self.twActualToolPose.setItem(0, 3, QtWidgets.QTableWidgetItem(str(self.gripper)))

    def save_pose_to_log(self):
        pose = self.robot.getToolPosition()
        self.add_log(f'Текущая позиция инструмента: [{round(pose[0], 3)}, {round(pose[1], 3)}, {round(pose[2], 3)}]')

    def update_motors(self):
        ticks = self.robot.getMotorPositionTick()
        radians = self.robot.getMotorPositionRadians()
        degrees = [math.degrees(r) for r in radians]
        temperature = self.robot.getActualTemperature()

        for i in range(6):
            self.twMotorsStates.setItem(0, i, QtWidgets.QTableWidgetItem(str(round(ticks[i], 3))))
        for i in range(6):
            self.twMotorsStates.setItem(1, i, QtWidgets.QTableWidgetItem(str(round(radians[i], 3))))
        for i in range(6):
            self.twMotorsStates.setItem(2, i, QtWidgets.QTableWidgetItem(str(round(degrees[i], 3))))
        for i in range(6):
            self.twMotorsStates.setItem(3, i, QtWidgets.QTableWidgetItem(str(round(temperature[i], 3))))

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
        self.lvLogs
        self.log_model.setStringList(self.logs)

    def sld_to_zero(self):
        for sld in self.sliders:
            sld.setValue(0)

    def refresh_session(self):
        self.session_model.setStringList([f'{i+1}. {x}' for i, x in enumerate(self.session)])

    def add_from_cb(self):
        if not hasattr(self, 'cbObjects'): return
        cat = self.cbObjects.currentText().strip()
        if cat in ['Box1', 'Box2', 'Box3', 'Reject']:
            self.session.append(cat)
            self.refresh_session()
            self.add_log(f'Добавен объект {cat}')
    
    def delete_last(self):
        rem = self.session.pop()
        self.refresh_session()
        self.add_log(f'Удалён объект: {rem}')
    
    def clear_list(self):
        self.session.clear()
        self.refresh_session()
        self.add_log('Список очищен')

    def chose_slot(self, cat):
        for i in self.allowed.get(cat, []):
            if not self.occupied[i]:
                self.occupied[i] = True
                return i
        return None

    def pick_and_place(self, dst_pos, dst_track):
        up = 0.20
        rx, ry, rz = getattr(self, 'DEFAULT_RX', 0.0), getattr(self, 'DEFAULT_RY', 0.0), getattr(self, 'DEFAULT_RZ', 0.0)

        self.robot.addMoveToPointL([Waypoint([0.0, 0.0, 0.0, rx, ry, rz])])
        self.robot.addLinearTrackMove(self.takeTrack)
        self.robot.addMoveToPointL([Waypoint([*self.takeCell[:2], self.takeCell[2]+up, rx, ry, rz])])
        self.robot.addMoveToPointL([Waypoint([*self.takeCell, rx, ry, rz])])
        self.robot.addToolState(1)
        self.robot.addMoveToPointL([Waypoint([*self.takeCell[:2], self.takeCell[2]+up, rx, ry, rz])])

        self.robot.addLinearTrackMove(dst_track)
        self.robot.addMoveToPointL([Waypoint([*dst_pos[:2], dst_pos[2]+up, rx, ry, rz])])
        self.robot.addMoveToPointL([Waypoint([*dst_pos, rx, ry, rz])])
        self.robot.addToolState(0)
        self.robot.addMoveToPointL([Waypoint([*dst_pos[:2], dst_pos[2]+up, rx, ry, rz])])
    
    def run_session(self):
        if not self.session:
            QtWidgets.QMessageBox.warning(self, 'Error', 'Очередь отсутствует')
            return
        self.add_log('Очередь перемещения началась')
        try:
            isWasInfo = False
            for cat in self.session:
                self.add_log(f'Начинается перемещение в коробку {cat}')
                if cat == 'Reject':
                    self.pick_and_place(self.rejectCell, self.rejectTrack)
                else:
                    slot = self.chose_slot(cat)
                    if slot is None:
                        if not isWasInfo:
                            QtWidgets.QMessageBox.information(self, 'Info', 'Требуется освобождение слотов')
                            isWasInfo = True
                            self.set_lamp_code('0001')
                        self.add_log(f'Требуется освобождение слотов для {cat}')
                        continue
                    self.pick_and_place(self.cells[slot], self.cellTracks[slot])
                while self.robot.getActualStateOut() != 200:
                # while self.robot.getActualStateOut() == 200:
                    time.sleep(0.1)
                self.add_log(f'Перемещение в коробку {cat} завершено')
                self.robot.play()
            self.robot.moveToInitialPose()
            self.add_log('Очередь перемещения завершена')
        except:
            return QtWidgets.QMessageBox.warning(self, 'Error', 'Ошибка запуска очереди')
    
    def clear_slots(self):
        for i in range(len(self.occupied) + 1):
            self.occupied[i] = False
        self.set_lamp_code('0100')
        self.add_log('Слоты очищены')

    def download_session(self):
        filename = self.leSessionPath.text().strip()
        if not filename:
            filename, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, 'Select folder and name your file', QtCore.QDir.homePath(), 'Text Files (*.txt);; All Files (*)'
            )
            if not filename:
                return QtWidgets.QMessageBox.warning(self, 'Error', 'Назовите свой файл')
        
        filename = filename if filename.endswith('.txt') else f'{filename}.txt'
        self.leSessionPath.setText(filename)

        try:
            with open(filename, 'a', encoding='UTF-8') as f:
                f.write('\n'.join(self.session) + '\n')
            self.add_log(f'Очередь сохранена по пути {filename}')
        except:
            QtWidgets.QMessageBox.warning(self, 'Error', 'Ошибка сохранения очереди')

    def load_session(self):
        filename = self.leLoadSessionPath.text().strip()
        if not filename:
            filename, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, 'Select file', QtCore.QDir.homePath(), 'Text Files (*.txt);; All Files (*)'
            )
            if not filename:
                return QtWidgets.QMessageBox.warning(self, 'Error', 'Нужно выбрать файл')
        
        filename = filename if filename.endswith('.txt') else f'{filename}.txt'
        self.leLoadSessionPath.setText(filename)

        try:
            with open(filename, encoding='UTF-8') as f:
                oke = []
                for l in f:
                    line = l.replace('\n', '')
                    if line in ['Box1', 'Box2', 'Box3', 'Reject']:
                        self.session.append(line)
                        self.refresh_session()
                    else:
                        return QtWidgets.QMessageBox.warning(self, 'Error', 'В очереди есть несуществующие объекты')
                self.add_log('Очередь загружена')
        except:
            QtWidgets.QMessageBox.warning(self, 'Error', 'Ошибка загрузки очереди')

    def Camera(self):
        if not self.cameraActive:
            if self.cap:
                try: self.cap.release()
                except: pass
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened(): self.add_log('Не удалось открыть камеру 0'); return
            self.videoActive, self.cameraActive = False, True
            if not self.frame_timer.isActive(): self.frame_timer.start(33)
            self.add_log('Камера запущена')
        else:
            if self.frame_timer.isActive(): self.frame_timer.stop()
            if self.cap:
                try: self.cap.release()
                except: pass
            self.cap = None
            self.videoActive = self.cameraActive = False
            if hasattr(self, 'lCamera1'): self.lCamera1.clear()
            if hasattr(self, 'lCamera2'): self.lCamera2.clear()
            self.add_log('Камера остановлена')

    def Video(self):
        if not self.videoActive:
            src = '1.webm'
            if self.cap:
                try: self.cap.release()
                except: pass
            self.cap = cv2.VideoCapture(src)
            if not self.cap.isOpened(): self.add_log(f'Не удалось открыть видео {src}'); return
            self.videoActive, self.cameraActive = True, False
            if not self.frame_timer.isActive(): self.frame_timer.start(33)
            self.add_log(f'Видео {src} запущено')
        else:
            if self.frame_timer.isActive(): self.frame_timer.stop()
            if self.cap:
                try: self.cap.release()
                except: pass
            self.cap = None
            self.videoActive = self.cameraActive = False
            if hasattr(self, 'lCamera1'): self.lCamera1.clear()
            if hasattr(self, 'lCamera2'): self.lCamera2.clear()
            self.add_log('Видео остановлено')

    def toggle_objects(self):
        self.det_objects = not self.det_objects
        if hasattr(self, 'btnDetectObjects'):
            self.btnDetectObjects.setText("Stop Detection" if self.det_objects else "Detect Objects")
        self.add_log("Детекция объектов: ON" if self.det_objects else "Детекция объектов: OFF")

    def _dedup(self, name, cx, cy):
        k, now = (name, cx//10, cy//10), int(time.time()*1000)
        if now - self._seen.get(k, 0) < self._ttl: return False
        self._seen[k] = now; return True

    def _ok(self, name: str) -> bool:
        name = name.lower()
        mapping = {
            '': getattr(self, 'cbBox1', None),
            '': getattr(self, 'cbBox2', None),
            '': getattr(self, 'cbReject', None)
        }
        w = mapping.get(name)
        return bool(w and w.isChecked())

    def to_label(self, label: QtWidgets.QLabel, bgr):
        if label is None: return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w, c = rgb.shape
        qimg = Qt.QImage(rgb.data, w, h, w * c, Qt.QImage.Format_RGB888)
        label.setPixmap(Qt.QPixmap.fromImage(qimg).scaled(label.width(), label.height(), QtCore.Qt.KeepAspectRatio))

    def tick(self):
        if not self.cap:
            return

        ok, frame = self.cap.read()
        if not ok:
            if self.videoActive:
                self.add_log("Конец видео")
            if self.frame_timer.isActive():
                self.frame_timer.stop()
            if self.cap:
                try: self.cap.release()
                except: pass
            self.cap = None
            self.videoActive = self.cameraActive = False
            if hasattr(self, 'lCamera1'): self.to_label(self.lCamera1, img1)
            if hasattr(self, 'lCamera2'): self.to_label(self.lCamera2, img2)
            return

        img2 = frame
        img1 = frame.copy()

        if self.det_objects:
            try:
                r = self.yolo(frame, verbose=False, device='cpu')[0]

                if hasattr(r, 'boxes') and r.boxes is not None:
                    keep = [self._ok(str(self.yolo.names[int(b.cls[0])]).lower()) for b in r.boxes]
                    if any(keep):
                        import numpy as np
                        r.boxes = r.boxes[np.array(keep, dtype=bool)]
                    else:
                        r.boxes = r.boxes[:0]

                    for b in r.boxes:
                        cls_id = int(b.cls[0])
                        name = str(self.yolo.names[cls_id]).lower()
                        x1, y1, x2, y2 = map(int, b.xyxy[0])
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        if self._dedup(name, cx, cy):
                            self.add_log(f"Detected {name}")
                            if self.moves_enabled and not self.people_present:
                                self.run_session_yolo(name)
                            else:
                                self.add_log("Перемещение отключено" if not self.moves_enabled else "Ожидание: человек в зоне")

                img1 = r.plot()
            except Exception as e:
                self.add_log(f"YOLO error: {e}")

        if hasattr(self, 'lCamera1'): self.to_label(self.lCamera1, img1)
        if hasattr(self, 'lCamera2'): self.to_label(self.lCamera2, img2)

    def in_next_update(self):
        return QtWidgets.QMessageBox.about(self, 'Info', 'Этот функционал будет добавлен в следующих обновлениях!')

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()