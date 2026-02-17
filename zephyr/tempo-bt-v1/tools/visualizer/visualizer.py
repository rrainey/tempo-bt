import sys
import os
import argparse
import serial
import serial.tools.list_ports
import numpy as np
from PyQt5 import QtWidgets, QtOpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import glutBitmapCharacter, GLUT_BITMAP_HELVETICA_18, glutInit
from PyQt5.QtCore import Qt
from PyQt5 import QtCore
import pywavefront
from pywavefront import visualization

# Tempo-BT USB identifiers (prj.conf)
TEMPO_VID = 0x1209
TEMPO_PID = 0x2026

# Load 3D model relative to script location
_script_dir = os.path.dirname(os.path.abspath(__file__))
obj = pywavefront.Wavefront(os.path.join(_script_dir, "objects", "tempo-bt.obj"))

glutInit(sys.argv)

# Quaternion to 4x4 rotation matrix (column-major for OpenGL)
def quaternion_to_matrix(w, x, y, z):

    # Extract the values from Q
    q0 = w
    q1 = x
    q2 = y
    q3 = z

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 4x4 rotation matrix
    matrix = [[r00, r10, r20, 0],
              [r01, r11, r21, 0],
              [r02, r12, r22, 0],
              [0,0,0,1]]
    return matrix

def nmea_checksum(payload):
    """Calculate NMEA checksum: XOR of all chars in payload."""
    cs = 0
    for ch in payload:
        cs ^= ord(ch)
    return cs

def nmea_validate(sentence):
    """Validate an NMEA sentence '$.....*HH'.

    Returns the payload (between $ and *) if checksum is valid, or None.
    """
    if not sentence.startswith('$') or '*' not in sentence:
        return None
    body, _, checksum_str = sentence.partition('*')
    payload = body[1:]  # strip leading $
    try:
        expected = int(checksum_str[:2], 16)
    except (ValueError, IndexError):
        return None
    if nmea_checksum(payload) != expected:
        return None
    return payload

class OpenGLWidget(QtOpenGL.QGLWidget):
    def __init__(self, parent=None):
        super(OpenGLWidget, self).__init__(parent)
        self.orientation = np.identity(4)
        self.viewpoint = (20, 0, 0)
        self.up = (0, 0, -1)
        self.accel = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)

        self.help_label = QtWidgets.QLabel(self)
        self.help_label.setText("Press F for forward\n      R for Right\n      Q to quit")
        self.help_label.setStyleSheet(
            "font-family: Courier; font-weight: bold; "
            "background-color: rgba(20, 20, 20, 150); color: white; "
            "font-size: 16px; padding: 35px;"
        )
        self.help_label.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        self.help_label.setFixedWidth(300)

        self.telemetry_label = QtWidgets.QLabel(self)
        self.telemetry_label.setText("Accel (m/s\u00b2): --\nGyro (rad/s): --")
        self.telemetry_label.setStyleSheet(
            "font-family: Courier; font-weight: bold; "
            "background-color: rgba(20, 20, 20, 150); color: #ddff55; "
            "font-size: 14px; padding: 10px;"
        )
        self.telemetry_label.setAlignment(Qt.AlignRight | Qt.AlignTop)
        self.telemetry_label.setFixedWidth(280)
        self.telemetry_label.setFixedHeight(200)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Anchor telemetry label to top-right corner
        self.telemetry_label.move(self.width() - self.telemetry_label.width() - 10, 10)

    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.0, 0.0, 0.0, 1.0)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        # Setup camera
        gluLookAt(self.viewpoint[0], self.viewpoint[1], self.viewpoint[2], 0, 0, 0, *self.up)

        # Apply the object's orientation
        glPushMatrix()

        glMultMatrixf(self.orientation)

        self.draw_axes()
        # Correct OBJ model alignment: rotate +90° around Z-axis
        glRotatef(90, 0, 0, 1)
        visualization.draw(obj)

        glPopMatrix()

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w / h, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def draw_axes(self):
        axes = [
            ((4, 0, 0), 'X', (1, 0, 0)),
            ((0, 4, 0), 'Y', (0, 1, 0)),
            ((0, 0, 4), 'Z', (0, 0, 1)),
        ]

        glLineWidth(4.0)
        for (dir_vec, label, color) in axes:
            glColor3f(*color)
            glBegin(GL_LINES)
            glVertex3f(0, 0, 0)
            glVertex3f(*dir_vec)
            glEnd()

            glRasterPos3f(*(np.array(dir_vec) * 1.1))
            for char in label:
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ord(char))

    def update_orientation(self, w, x, y, z):
        self.orientation = quaternion_to_matrix(w, x, y, z)
        self.update()

    def update_imu(self, ax, ay, az, gx, gy, gz):
        """Update displayed IMU telemetry values."""
        self.accel = (ax, ay, az)
        self.gyro = (gx, gy, gz)
        self.telemetry_label.setText(
            f"Accel (m/s\u00b2)\n"
            f"  X: {ax:+8.2f}\n"
            f"  Y: {ay:+8.2f}\n"
            f"  Z: {az:+8.2f}\n"
            f"Gyro (rad/s)\n"
            f"  X: {gx:+8.4f}\n"
            f"  Y: {gy:+8.4f}\n"
            f"  Z: {gz:+8.4f}"
        )
        self.update()

    def set_viewpoint(self, point, up):
        self.viewpoint = point
        self.up = up
        self.update()

class Window(QtWidgets.QMainWindow):
    def __init__(self, port=None):
        super(Window, self).__init__()
        self.opengl_widget = OpenGLWidget(self)
        self.setCentralWidget(self.opengl_widget)
        self.setWindowTitle("Tempo-BT Visualizer")

        # Auto-detect or use provided port
        if port is None:
            port = self._find_tempo_port()
        if port is None:
            print("ERROR: No Tempo-BT device found (VID=0x%04X, PID=0x%04X)." %
                  (TEMPO_VID, TEMPO_PID))
            print("       Connect Tempo-BT via USB or specify port with --port.")
            sys.exit(1)

        print(f"Connecting to Tempo-BT on {port}")
        self.serial_port = serial.Serial(port, 115200, timeout=0.1)
        self.read_data()

    @staticmethod
    def _find_tempo_port():
        """Find the first COM port matching Tempo-BT VID/PID."""
        for p in serial.tools.list_ports.comports():
            if p.vid == TEMPO_VID and p.pid == TEMPO_PID:
                return p.device
        return None

    def read_data(self):
        try:
            while self.serial_port.in_waiting:
                line = self.serial_port.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('$'):
                    self.process_line(line)
        except serial.SerialException:
            print("Serial connection lost.")
            self.close()
            return
        self.update()
        QtCore.QTimer.singleShot(10, self.read_data)

    def process_line(self, line):
        """Parse an NMEA sentence and dispatch to the appropriate handler."""
        payload = nmea_validate(line)
        if payload is None:
            return  # silently ignore invalid checksums

        fields = payload.split(',')
        sentence_type = fields[0]

        if sentence_type == 'PIM2' and len(fields) >= 6:
            # $PIM2,<timestamp_ms>,<qw>,<qx>,<qy>,<qz>*HH
            try:
                qw = float(fields[2])
                qx = float(fields[3])
                qy = float(fields[4])
                qz = float(fields[5])
                self.opengl_widget.update_orientation(qw, qx, qy, qz)
            except (ValueError, IndexError):
                pass

        elif sentence_type == 'PIMU' and len(fields) >= 8:
            # $PIMU,<timestamp_ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>*HH
            try:
                ax = float(fields[2])
                ay = float(fields[3])
                az = float(fields[4])
                gx = float(fields[5])
                gy = float(fields[6])
                gz = float(fields[7])
                self.opengl_widget.update_imu(ax, ay, az, gx, gy, gz)
            except (ValueError, IndexError):
                pass

        # All other sentence types silently ignored

    def keyPressEvent(self, event):
        key = event.text().upper()
        if key == 'Q':
            self.close()
        elif key == 'F':
            self.opengl_widget.set_viewpoint((20, 0, 0), (0, 0, -1))
        elif key == 'R':
            self.opengl_widget.set_viewpoint((0, 20, 0), (0, 0, -1))
        event.accept()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tempo-BT 3D Orientation Visualizer")
    parser.add_argument('--port', '-p', type=str, default=None,
                        help='COM port (e.g., COM5). Auto-detects Tempo-BT if omitted.')
    args, qt_args = parser.parse_known_args()

    app = QtWidgets.QApplication(qt_args)
    window = Window(port=args.port)
    window.resize(1200, 1200)
    window.show()
    sys.exit(app.exec_())
