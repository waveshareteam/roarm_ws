#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QImage, QPainter, QColor, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

_MODULE_DIR = Path(__file__).resolve().parent
CONFIG_DIR = _MODULE_DIR.parent / "config"
CONFIG_PATH = CONFIG_DIR / "lab_tool_colors.json"

DEFAULT_PROFILES = {
    "red": {"lower": [0, 160, 0], "upper": [255, 255, 255]},
    "blue": {"lower": [0, 0, 0], "upper": [255, 110, 130]},
    "green": {"lower": [0, 0, 130], "upper": [255, 110, 255]},
}

def load_profiles() -> dict:
    if CONFIG_PATH.is_file():
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    return dict(DEFAULT_PROFILES)


def save_profiles(data: dict) -> None:
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def np_to_qpixmap(bgr: np.ndarray) -> QPixmap:
    if bgr is None or bgr.size == 0:
        return QPixmap()
    rgb = np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    h, w, ch = rgb.shape
    qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
    return QPixmap.fromImage(qimg.copy())


class AbGamutWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self._l = 128

    def set_l(self, l_val: int):
        self._l = l_val
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, Qt.black)
        step = 8
        for a in range(0, 256, step):
            for b in range(0, 256, step):
                lab = np.uint8([[[self._l, a, b]]])
                bgr = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)[0, 0]
                c = QColor(int(bgr[2]), int(bgr[1]), int(bgr[0]))
                p.fillRect(
                    int(a / 255 * (w - 20)) + 10,
                    int((255 - b) / 255 * (h - 20)) + 10,
                    max(2, w // 32),
                    max(2, h // 32),
                    c,
                )
        p.setPen(Qt.white)
        p.drawText(10, h - 10, "a*  →")
        p.drawText(10, 20, "b*  ↑")


class ChannelRow(QWidget):
    valueChanged = pyqtSignal()

    def __init__(self, name, vmin, vmax, lo, hi, parent=None):
        super().__init__(parent)
        self._block = False

        self.lbl = QLabel(name)
        self.lbl.setFixedWidth(24)
        self.lbl.setStyleSheet("font-weight:bold; color:#147ec1;")

        self.btn_m_lo = QPushButton("-")
        self.btn_p_lo = QPushButton("+")
        self.btn_m_hi = QPushButton("-")
        self.btn_p_hi = QPushButton("+")
        for b in (self.btn_m_lo, self.btn_p_lo, self.btn_m_hi, self.btn_p_hi):
            b.setFixedSize(28, 28)
            b.setStyleSheet(
                "QPushButton{background:#147ec1;color:#111;border:none;border-radius:4px;}"
                "QPushButton:hover{background:#1453c1;}"
            )

        self.s_lo = QSlider(Qt.Horizontal)
        self.s_hi = QSlider(Qt.Horizontal)
        for s in (self.s_lo, self.s_hi):
            s.setRange(vmin, vmax)
            s.setStyleSheet(
                "QSlider::groove:horizontal{height:6px;background:#444;border-radius:3px;}"
                "QSlider::handle:horizontal{width:14px;margin:-5px 0;"
                "background:#147ec1;border-radius:7px;}"
            )
        self.s_lo.setValue(lo)
        self.s_hi.setValue(hi)

        self.sp_lo = QSpinBox()
        self.sp_hi = QSpinBox()
        for sp, v in ((self.sp_lo, lo), (self.sp_hi, hi)):
            sp.setRange(vmin, vmax)
            sp.setValue(v)
            sp.setFixedWidth(48)
            sp.setStyleSheet("background:#333;color:#eee;border:1px solid #555;")

        grid = QGridLayout(self)
        grid.setContentsMargins(0, 4, 0, 4)
        grid.addWidget(self.lbl, 0, 0)
        grid.addWidget(self.btn_m_lo, 0, 1)
        grid.addWidget(self.s_lo, 0, 2)
        grid.addWidget(self.btn_p_lo, 0, 3)
        grid.addWidget(self.sp_lo, 0, 4)
        grid.addWidget(self.btn_m_hi, 1, 1)
        grid.addWidget(self.s_hi, 1, 2)
        grid.addWidget(self.btn_p_hi, 1, 3)
        grid.addWidget(self.sp_hi, 1, 4)

        self.s_lo.valueChanged.connect(self._on_lo)
        self.s_hi.valueChanged.connect(self._on_hi)
        self.sp_lo.valueChanged.connect(self._on_lo)
        self.sp_hi.valueChanged.connect(self._on_hi)
        self.btn_m_lo.clicked.connect(lambda: self._nudge(self.s_lo, -1))
        self.btn_p_lo.clicked.connect(lambda: self._nudge(self.s_lo, 1))
        self.btn_m_hi.clicked.connect(lambda: self._nudge(self.s_hi, -1))
        self.btn_p_hi.clicked.connect(lambda: self._nudge(self.s_hi, 1))

    def _nudge(self, slider, d):
        slider.setValue(max(slider.minimum(), min(slider.maximum(), slider.value() + d)))

    def _on_lo(self, v):
        if self._block:
            return
        self._block = True
        if v > self.s_hi.value():
            self.s_hi.setValue(v)
        self.sp_lo.setValue(self.s_lo.value())
        self.sp_hi.setValue(self.s_hi.value())
        self._block = False
        self.valueChanged.emit()

    def _on_hi(self, v):
        if self._block:
            return
        self._block = True
        if v < self.s_lo.value():
            self.s_lo.setValue(v)
        self.sp_lo.setValue(self.s_lo.value())
        self.sp_hi.setValue(self.s_hi.value())
        self._block = False
        self.valueChanged.emit()

    def values(self):
        return self.s_lo.value(), self.s_hi.value()

    def set_values(self, lo, hi):
        self._block = True
        self.s_lo.setValue(lo)
        self.s_hi.setValue(hi)
        self.sp_lo.setValue(lo)
        self.sp_hi.setValue(hi)
        self._block = False


class LabToolWindow(QMainWindow):
    TEXT = {
        "zh": {
            "mask": "Mask",
            "orig": "Original",
            "lab_desc": "L* 表示亮度；\nA* 表示绿-红分量；\nB* 表示蓝-黄分量。",
            "color_list": "颜色列表",
            "add": "添加",
            "delete": "删除",
            "save": "保存",
            "quit": "退出",
            "coverage": "覆盖率",
        },
        "en": {
            "mask": "Mask",
            "orig": "Original",
            "lab_desc": "L* lightness;\nA* green-red;\nB* blue-yellow.",
            "color_list": "Color list",
            "add": "Add",
            "delete": "Delete",
            "save": "Save",
            "quit": "Quit",
            "coverage": "Coverage",
        },
    }

    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.lang = "zh"
        self.profiles = load_profiles()

        self.setWindowTitle("LAB_Tool 1.0")
        self.setMinimumSize(1100, 720)
        self.setStyleSheet(
            "QMainWindow{background:#2b2b2b;}"
            "QLabel{color:#ddd;}"
            "QGroupBox{color:#ccc;border:1px solid #555;margin-top:8px;padding-top:8px;}"
            "QComboBox{background:#3a3a3a;color:#eee;padding:6px 8px;"
            "  border:1px solid #666;border-radius:4px;min-height:24px;}"
            "QComboBox:hover{background:#454545;border:1px solid #147ec1;}"
            "QComboBox::drop-down{border:none;width:24px;}"
            "QComboBox::down-arrow{image:none;border-left:5px solid transparent;"
            "  border-right:5px solid transparent;border-top:6px solid #147ec1;}"
            "QComboBox QAbstractItemView{background:#2d2d2d;color:#eee;"
            "  selection-background-color:#147ec1;selection-color:#fff;"
            "  border:1px solid #666;outline:0;padding:4px;}"
            "QComboBox QAbstractItemView::item{min-height:28px;padding-left:8px;}"
            "QComboBox QAbstractItemView::item:hover{background:#454545;}"
            "QRadioButton{color:#ffffff;}"
        )

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(8)
        root.setContentsMargins(10, 10, 10, 10)

        top = QHBoxLayout()
        self.box_mask = self._image_panel("Mask")
        self.box_orig = self._image_panel("Original")
        top.addWidget(self.box_mask, 1)
        top.addWidget(self.box_orig, 1)
        root.addLayout(top, 3)

        bottom = QHBoxLayout()
        left = QVBoxLayout()
        self.row_l = ChannelRow("L", 0, 255, 0, 90)
        self.row_a = ChannelRow("A", 0, 255, 0, 255)
        self.row_b = ChannelRow("B", 0, 255, 0, 255)
        for row in (self.row_l, self.row_a, self.row_b):
            row.valueChanged.connect(self._on_range_changed)
            left.addWidget(row)
        self.lbl_desc = QLabel()
        self.lbl_desc.setStyleSheet("color:#aaa;font-size:12px;margin-top:8px;")
        left.addWidget(self.lbl_desc)
        left.addStretch()
        bottom.addLayout(left, 2)

        mid = QVBoxLayout()
        self.gamut = AbGamutWidget()
        mid.addWidget(self.gamut, 1)
        bottom.addLayout(mid, 1)

        right = QVBoxLayout()
        self.lbl_list = QLabel()
        self.combo = QComboBox()
        self._refresh_combo()
        self.combo.currentTextChanged.connect(self._load_profile)

        btn_style = (
            "QPushButton{background:#147ec1;color:#111;font-weight:bold;"
            "padding:10px;border:none;border-radius:4px;min-width:80px;}"
            "QPushButton:hover{background:#1453c1;}"
        )
        gray_style = (
            "QPushButton{background:#888;color:#111;font-weight:bold;"
            "padding:12px;border:none;border-radius:4px;}"
            "QPushButton:hover{background:#aaa;}"
        )
        self.btn_add = QPushButton()
        self.btn_del = QPushButton()
        self.btn_save = QPushButton()
        for b in (self.btn_add, self.btn_del, self.btn_save):
            b.setStyleSheet(btn_style)
        self.btn_add.clicked.connect(self._add_profile)
        self.btn_del.clicked.connect(self._delete_profile)
        self.btn_save.clicked.connect(self._save_profiles)

        lang_row = QHBoxLayout()
        self.rb_zh = QRadioButton("中文")
        self.rb_en = QRadioButton("English")
        self.rb_zh.setChecked(True)
        self.rb_zh.toggled.connect(lambda c: c and self._set_lang("zh"))
        self.rb_en.toggled.connect(lambda c: c and self._set_lang("en"))
        lang_row.addWidget(self.rb_zh)
        lang_row.addWidget(self.rb_en)

        self.btn_quit = QPushButton()
        self.btn_quit.setStyleSheet(gray_style)
        self.btn_quit.clicked.connect(self.close)

        right.addWidget(self.lbl_list)
        right.addWidget(self.combo)
        right.addWidget(self.btn_add)
        right.addWidget(self.btn_del)
        right.addWidget(self.btn_save)
        right.addLayout(lang_row)
        right.addStretch()
        right.addWidget(self.btn_quit)
        bottom.addLayout(right, 1)
        root.addLayout(bottom, 2)

        self.lbl_cov = QLabel()
        root.addWidget(self.lbl_cov)

        self._set_lang("zh")
        if self.combo.count():
            self._load_profile(self.combo.currentText())

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update_images)
        self._timer.start(33)

    def _image_panel(self, title):
        box = QGroupBox(title)
        lay = QVBoxLayout(box)
        lbl = QLabel()
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setMinimumSize(400, 300)
        lbl.setStyleSheet("background:#111;border:1px solid #444;")
        lay.addWidget(lbl)
        box._inner = lbl
        return box

    def _panel_label(self, box):
        return box._inner

    def _set_lang(self, lang):
        self.lang = lang
        t = self.TEXT[lang]
        self.box_mask.setTitle(t["mask"])
        self.box_orig.setTitle(t["orig"])
        self.lbl_desc.setText(t["lab_desc"])
        self.lbl_list.setText(t["color_list"])
        self.btn_add.setText(t["add"])
        self.btn_del.setText(t["delete"])
        self.btn_save.setText(t["save"])
        self.btn_quit.setText(t["quit"])

    def _refresh_combo(self):
        self.combo.blockSignals(True)
        self.combo.clear()
        self.combo.addItems(sorted(self.profiles.keys()))
        self.combo.blockSignals(False)

    def _current_lower_upper(self):
        lo = np.array([
            self.row_l.values()[0],
            self.row_a.values()[0],
            self.row_b.values()[0],
        ], dtype=np.uint8)
        hi = np.array([
            self.row_l.values()[1],
            self.row_a.values()[1],
            self.row_b.values()[1],
        ], dtype=np.uint8)
        return lo, hi

    def _load_profile(self, name):
        if not name or name not in self.profiles:
            return
        p = self.profiles[name]
        lo, hi = p["lower"], p["upper"]
        self.row_l.set_values(lo[0], hi[0])
        self.row_a.set_values(lo[1], hi[1])
        self.row_b.set_values(lo[2], hi[2])
        self.gamut.set_l((lo[0] + hi[0]) // 2)

    def _add_profile(self):
        name, ok = QInputDialog.getText(self, "Add", "Profile name:")
        if ok and name.strip():
            lo, hi = self._current_lower_upper()
            self.profiles[name.strip()] = {
                "lower": lo.tolist(),
                "upper": hi.tolist(),
            }
            self._refresh_combo()
            self.combo.setCurrentText(name.strip())

    def _delete_profile(self):
        name = self.combo.currentText()
        if name in self.profiles:
            del self.profiles[name]
            save_profiles(self.profiles)
            self._refresh_combo()

    def _save_profiles(self):
        name = self.combo.currentText()
        if name:
            lo, hi = self._current_lower_upper()
            self.profiles[name] = {"lower": lo.tolist(), "upper": hi.tolist()}
        save_profiles(self.profiles)
        lo, hi = self._current_lower_upper()
        self.node.get_logger().info(
            "Saved lower=%s upper=%s" % (lo.tolist(), hi.tolist())
        )
        print("lower:", lo.tolist())
        print("upper:", hi.tolist())
        QMessageBox.information(self, "Save", "已保存到\n%s" % CONFIG_PATH)

    def _on_range_changed(self):
        l0, l1 = self.row_l.values()
        self.gamut.set_l((l0 + l1) // 2)

    def _update_images(self):
        frame = self.node.frame
        if frame is None:
            return
        lo, hi = self._current_lower_upper()
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        mask = cv2.inRange(lab, lo, hi)
        cov = 100.0 * np.count_nonzero(mask) / max(mask.size, 1)
        mask_show = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        for lbl, img in (
            (self._panel_label(self.box_mask), mask_show),
            (self._panel_label(self.box_orig), frame),
        ):
            pix = np_to_qpixmap(img)
            if not pix.isNull():
                lbl.setPixmap(
                    pix.scaled(lbl.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
                )

        t = self.TEXT[self.lang]
        self.lbl_cov.setText(
            "%s: %.1f%%    L[%d,%d] a[%d,%d] b[%d,%d]"
            % (t["coverage"], cov, lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])
        )

    def closeEvent(self, event):
        lo, hi = self._current_lower_upper()
        print("lower:", lo.tolist())
        print("upper:", hi.tolist())
        event.accept()


class LabToolRosNode(Node):
    def __init__(self):
        super().__init__("lab_trackbar_node")
        self.declare_parameter("image_topic", "/image_raw")
        topic = self.get_parameter("image_topic").value
        self.bridge = CvBridge()
        self.frame = None
        self.create_subscription(Image, topic, self._cb, 10)

    def _cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")


def main(args=None):
    rclpy.init(args=args)
    node = LabToolRosNode()
    app = QApplication(sys.argv)
    win = LabToolWindow(node)
    win.show()

    spin = QTimer()
    spin.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    spin.start(5)

    if hasattr(app, "exec"):
        code = app.exec()
    else:
        code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)
