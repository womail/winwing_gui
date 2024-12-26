#!/usr/bin/env python3
"""
WinWing FCU Controller GUI

This application provides a graphical interface for the WinWing FCU (Flight Control Unit) 
used with X-Plane flight simulator. It controls the WinWing Airbus FCU/EFIS panels.

Requirements:
- Python 3.6 or higher
- PyQt6
- libusb1 (Install via: apt install libusb1 on Debian/Ubuntu or brew install libusb on macOS)
- X-Plane 11/12 with Toliss Airbus aircraft

Installation:
1. Install required packages:
   pip install PyQt6 pyusb

2. Place these files in the same directory:
   - winwing_gui.py
   - winwing_fcu.py
   - XPlaneUdp.py

3. Ensure proper USB permissions:
   Copy udev/71-winwing.rules to /etc/udev/rules.d/
   Then run: sudo udevadm control --reload-rules && sudo udevadm trigger

Usage:
1. Start X-Plane with a Toliss Airbus aircraft
2. Connect the WinWing FCU hardware
3. Run this application: python3 winwing_gui.py
4. Click "Start FCU Controller"
5. Use "Output: On/Off" to toggle debug output

The application will:
- Detect and initialize the WinWing FCU hardware
- Connect to X-Plane via UDP
- Handle all communication between X-Plane and the FCU
- Provide real-time status and debugging information

Author: schenlap (https://github.com/schenlap/winwing_fcu)
GUI Enhancement: Claude AI Assistant
"""

import sys
import io
from functools import partial
from PyQt6.QtWidgets import (QApplication, QMainWindow, QPushButton, 
                            QVBoxLayout, QWidget, QTextEdit, QLabel,
                            QHBoxLayout, QInputDialog, QMessageBox,
                            QSystemTrayIcon, QMenu)
from PyQt6.QtCore import QThread, pyqtSignal, Qt, QRect
from PyQt6.QtGui import QFont, QIcon, QPainter, QColor, QPen
import subprocess
import winwing_fcu
import hashlib
import requests
import json
from pathlib import Path
import time

class FCUWorker(QThread):
    output_received = pyqtSignal(str)
    status_update = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.running = True
        self.suppress_output = False
        
        # Pre-compile the strip check
        self.strip = str.strip
    
    def toggle_output(self, enabled):
        self.suppress_output = not enabled
    
    def stop(self):
        """Enhanced stop method"""
        self.running = False
        self.suppress_output = True
        winwing_fcu.running = False
        
        # Ensure stdout is restored
        if sys.stdout != sys.__stdout__:
            sys.stdout = sys.__stdout__
        
        self.status_update.emit("Stopping...")
    
    def run(self):
        class SignalEmitter:
            __slots__ = ('signal', 'worker', 'strip')  # Optimize memory usage
            
            def __init__(self, signal, worker):
                self.signal = signal
                self.worker = worker
                self.strip = str.strip  # Cache method lookup

            def write(self, text):
                try:
                    if self.strip(text) and not self.worker.suppress_output:
                        self.signal.emit(text)
                except Exception as e:
                    self.signal.emit(f"Debug - Write Error: {str(e)}")

            def flush(self):
                pass

        try:
            # Store original stdout
            original_stdout = sys.stdout
            
            # Set up signal emitter
            sys.stdout = SignalEmitter(self.output_received, self)
            
            self.status_update.emit("Starting FCU Controller...")
            self.output_received.emit("Debug - Initializing winwing_fcu...")
            winwing_fcu.running = True
            winwing_fcu.main()
        except Exception as e:
            if not self.suppress_output:
                self.output_received.emit(f"Debug - Detailed Error: {str(e)}")
                self.output_received.emit(f"Debug - Error Type: {type(e)}")
                import traceback
                self.output_received.emit(f"Debug - Traceback: {traceback.format_exc()}")
                self.status_update.emit("Error occurred!")
        finally:
            # Restore original stdout
            sys.stdout = original_stdout
            winwing_fcu.running = False
            if not self.suppress_output:
                self.status_update.emit("Stopped")

class DigitalDisplay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(100)
        self.speed = "---"
        self.heading = "---"
        self.altitude = "-----"
        self.vs = "----"  # Added V/S display
        self.setStyleSheet("background-color: #1a1a1a;")

    def update_values(self, speed, heading, altitude, vs="----"):  # Added vs parameter
        self.speed = str(speed).rjust(3, ' ')
        self.heading = str(heading).rjust(3, ' ')
        self.altitude = str(altitude).rjust(5, ' ')
        self.vs = str(vs).rjust(4, ' ')  # Format V/S
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Set up the amber/orange color similar to the FCU
        amber = QColor(255, 147, 0)
        painter.setPen(QPen(amber, 2))

        # Set font for the digital display
        font = painter.font()
        font.setFamily("Consolas")
        font.setPixelSize(32)
        font.setBold(True)
        painter.setFont(font)

        # Calculate positions
        width = self.width()
        height = self.height()
        
        # Draw speed (leftmost)
        speed_rect = QRect(0, 10, width//4, height-20)
        painter.drawText(speed_rect, Qt.AlignmentFlag.AlignCenter, self.speed)
        
        # Draw heading
        hdg_rect = QRect(width//4, 10, width//4, height-20)
        painter.drawText(hdg_rect, Qt.AlignmentFlag.AlignCenter, self.heading)
        
        # Draw altitude
        alt_rect = QRect(2*width//4, 10, width//4, height-20)
        painter.drawText(alt_rect, Qt.AlignmentFlag.AlignCenter, self.altitude)

        # Draw V/S (rightmost)
        vs_rect = QRect(3*width//4, 10, width//4, height-20)
        painter.drawText(vs_rect, Qt.AlignmentFlag.AlignCenter, self.vs)

        # Draw labels
        font.setPixelSize(14)
        painter.setFont(font)
        painter.drawText(QRect(0, height-20, width//4, 20), Qt.AlignmentFlag.AlignCenter, "SPD")
        painter.drawText(QRect(width//4, height-20, width//4, 20), Qt.AlignmentFlag.AlignCenter, "HDG")
        painter.drawText(QRect(2*width//4, height-20, width//4, 20), Qt.AlignmentFlag.AlignCenter, "ALT")
        painter.drawText(QRect(3*width//4, height-20, width//4, 20), Qt.AlignmentFlag.AlignCenter, "V/S")

class FCUButtonPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(50)
        self.button_states = {
            'LOC': False,
            'AP1': False,
            'AP2': False,
            'A/THR': False,
            'EXPED': False,
            'APPR': False
        }
        self.setStyleSheet("background-color: #1a1a1a;")

    def update_button_state(self, button_name, state):
        if button_name in self.button_states:
            self.button_states[button_name] = state
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Colors - adjusted for better visibility
        off_color = QColor(40, 40, 40)  # Dark gray for off state
        on_color = QColor(0, 255, 0)    # Bright green for on state
        text_color = QColor(255, 255, 255)  # White text for better contrast

        # Calculate button dimensions
        width = self.width()
        height = self.height()
        button_width = width // 6
        button_height = height - 10
        spacing = 5

        # Draw each button
        x = 0
        for button_name, is_on in self.button_states.items():
            # Draw button background
            button_rect = QRect(x + spacing, 5, button_width - 2*spacing, button_height)
            painter.setPen(Qt.PenStyle.NoPen)
            
            if is_on:
                # Add glow effect for illuminated buttons
                glow = QColor(0, 255, 0, 50)  # Semi-transparent green
                painter.setBrush(glow)
                painter.drawRoundedRect(button_rect.adjusted(-2, -2, 2, 2), 6, 6)
            
            # Draw main button
            painter.setBrush(on_color if is_on else off_color)
            painter.drawRoundedRect(button_rect, 5, 5)

            # Draw button text
            painter.setPen(text_color)
            font = painter.font()
            font.setPixelSize(14)
            font.setBold(True)
            painter.setFont(font)
            painter.drawText(button_rect, Qt.AlignmentFlag.AlignCenter, button_name)

            x += button_width

class UpdateChecker:
    """Checks for updates to WinWing FCU files on GitHub"""
    
    def __init__(self):
        self.base_url = "https://api.github.com/repos/schenlap/winwing_fcu/contents"
        self.local_files = [
            "winwing_fcu.py",
            "XPlaneUdp.py",
            "test_endpoint.py",
            "test_endpoint_in.py"
        ]
        self.cache_file = Path("update_cache.json")
        self.cache_timeout = 3600  # 1 hour in seconds

    def get_file_hash(self, filepath):
        """Calculate SHA-1 hash of local file"""
        try:
            with open(filepath, 'rb') as f:
                return hashlib.sha1(f.read()).hexdigest()
        except FileNotFoundError:
            return None

    def load_cache(self):
        """Load cached GitHub file data"""
        if self.cache_file.exists():
            try:
                with open(self.cache_file) as f:
                    cache = json.load(f)
                if time.time() - cache.get('timestamp', 0) < self.cache_timeout:
                    return cache.get('data')
            except:
                pass
        return None

    def save_cache(self, data):
        """Save GitHub file data to cache"""
        try:
            with open(self.cache_file, 'w') as f:
                json.dump({
                    'timestamp': time.time(),
                    'data': data
                }, f)
        except:
            pass

    def check_updates(self):
        """Check for file updates on GitHub"""
        updates = []
        
        # Try to load from cache first
        github_files = self.load_cache()
        
        # If no cache or expired, fetch from GitHub
        if not github_files:
            try:
                response = requests.get(self.base_url)
                response.raise_for_status()
                github_files = {f['name']: f['sha'] for f in response.json() 
                              if f['name'] in self.local_files}
                self.save_cache(github_files)
            except Exception as e:
                return f"Error checking for updates: {str(e)}"

        # Compare file hashes
        for filename in self.local_files:
            if filename in github_files:
                local_hash = self.get_file_hash(filename)
                if local_hash and local_hash != github_files[filename]:
                    updates.append(filename)
                elif not local_hash:
                    updates.append(f"{filename} (missing)")

        if updates:
            return f"Updates available for: {', '.join(updates)}"
        return "All files are up to date"

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("WinWing FCU Control")
        self.setMinimumSize(800, 600)
        
        # Set window icon
        self.setWindowIcon(QIcon("propeller_icon.png"))  # Ensure the path is correct
        
        # Initialize version
        self.version = 0.010
        
        # Initialize UI components
        self._init_ui()
        
        # Initialize worker
        self.worker = None
        
        # Initialize system tray
        self._init_system_tray()
        
        # Initialize status bar
        self.statusBar().showMessage(f"Version: {self.version:.3f}")

        # Apply stylesheet
        self._init_stylesheet()

        # Add cleanup handler
        app.aboutToQuit.connect(self.cleanup)

        # Initialize update checker
        self.update_checker = UpdateChecker()

    def increment_version(self):
        self.version += 0.001
        self.statusBar().showMessage(f"Version: {self.version:.3f}")

    # Call this method whenever you want to increment the version
    def some_method_that_changes(self):
        # ... your code ...
        self.increment_version()  # Increment version after a change

    def _init_stylesheet(self):
        # Move stylesheet to a separate method for better organization
        self.setStyleSheet("""
            QMainWindow { background-color: #f0f4f8; }
            QPushButton {
                background-color: #4a90e2;
                color: white;
                border: none;
                border-radius: 4px;
                padding: 12px 24px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #357abd;
            }
            QPushButton:pressed {
                background-color: #2d6da3;
            }
            QPushButton#stop_button {
                background-color: #e74c3c;
            }
            QPushButton#stop_button:hover {
                background-color: #c0392b;
            }
            QPushButton#stop_button:pressed {
                background-color: #a93226;
            }
            QPushButton:disabled {
                background-color: #bdc3c7;
            }
            QTextEdit {
                background-color: white;
                border: 1px solid #dce1e8;
                border-radius: 4px;
                padding: 8px;
                font-family: "Consolas", monospace;
                font-size: 13px;
            }
            QLabel {
                color: #2c3e50;
                font-size: 24px;
                font-weight: bold;
            }
            QLabel#status_label {
                color: #2c3e50;
                font-size: 14px;
                font-weight: normal;
                padding: 8px 16px;
                border-radius: 4px;
                background-color: #e8f0fe;
            }
            QLabel#status_label[status="running"] {
                background-color: #e3fcef;
                color: #0a7c42;
            }
            QLabel#status_label[status="error"] {
                background-color: #fee8e7;
                color: #c0392b;
            }
            QPushButton#output_button {
                background-color: #27ae60;
            }
            QPushButton#output_button:checked {
                background-color: #27ae60;
            }
            QPushButton#output_button:!checked {
                background-color: #95a5a6;
            }
            QPushButton#output_button:hover {
                background-color: #219a52;
            }
            QPushButton#output_button:!checked:hover {
                background-color: #7f8c8d;
            }
        """)

    def _init_ui(self):
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(15)

        # Add title
        title = QLabel("WinWing FCU Controller")
        layout.addWidget(title)

        # Add digital display
        self.digital_display = DigitalDisplay()
        layout.addWidget(self.digital_display)

        # Add FCU button panel after digital display
        self.button_panel = FCUButtonPanel()
        layout.addWidget(self.button_panel)

        # Create button layout
        button_layout = self._create_button_layout()
        layout.addLayout(button_layout)

        # Create output text box
        self.output_text = QTextEdit()
        self.output_text.setReadOnly(True)
        self.output_text.setPlaceholderText("Output will appear here...")
        self.output_text.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)  # Optimize for log output
        layout.addWidget(self.output_text)

    def _create_button_layout(self):
        button_layout = QHBoxLayout()
        
        # Create start button
        self.start_button = QPushButton("Start FCU Controller")
        self.start_button.clicked.connect(self.start_fcu)
        button_layout.addWidget(self.start_button)
        
        # Create output toggle button
        self.output_button = QPushButton("Output: On")
        self.output_button.setObjectName("output_button")
        self.output_button.setCheckable(True)
        self.output_button.setChecked(True)
        self.output_button.clicked.connect(self.toggle_output)
        self.output_button.setEnabled(False)
        button_layout.addWidget(self.output_button)
        
        # Add status label
        self.status_label = QLabel("Ready")
        self.status_label.setObjectName("status_label")
        button_layout.addWidget(self.status_label)
        
        # Add minimize to tray button
        self.minimize_button = QPushButton("Minimize to Tray")
        self.minimize_button.clicked.connect(self.hide)
        button_layout.addWidget(self.minimize_button)
        
        # Add exit button
        self.exit_button = QPushButton("Exit")
        self.exit_button.clicked.connect(self.shutdown_application)  # Connect to shutdown method
        button_layout.addWidget(self.exit_button)
        
        # Add check updates button
        self.update_button = QPushButton("Check Updates")
        self.update_button.clicked.connect(self.check_updates)
        button_layout.addWidget(self.update_button)
        
        button_layout.addStretch()
        return button_layout

    def toggle_output(self):
        if self.worker:
            enabled = self.output_button.isChecked()
            self.worker.toggle_output(enabled)
            self.output_button.setText(f"Output: {'On' if enabled else 'Off'}")

    def start_fcu(self):
        if self.worker is None or not self.worker.isRunning():
            self.start_button.setEnabled(False)
            self.output_button.setEnabled(True)
            self.start_button.setText("Running...")
            self.output_text.clear()
            
            # Create and start worker thread
            self.worker = FCUWorker()
            self.worker.output_received.connect(self.update_output)
            self.worker.status_update.connect(self.update_status)
            self.worker.finished.connect(self.on_worker_finished)
            self.worker.start()

    def update_output(self, text):
        try:
            if "cache: v:" in text:
                # Update displays
                if "sim/cockpit2/autopilot/airspeed_dial_kts_mach val:" in text:
                    speed = text.split("val:")[1].strip()
                    self.digital_display.speed = speed
                elif "sim/cockpit/autopilot/heading_mag val:" in text:
                    heading = text.split("val:")[1].strip()
                    self.digital_display.heading = heading
                elif "sim/cockpit/autopilot/altitude val:" in text:
                    altitude = text.split("val:")[1].strip()
                    self.digital_display.altitude = altitude
                elif "sim/cockpit/autopilot/vertical_velocity val:" in text:
                    vs = text.split("val:")[1].strip()
                    try:
                        vs_value = int(int(vs) / 100)
                        if vs_value < 0:
                            vs = str(abs(vs_value)).rjust(3, '0') + '↓'
                        else:
                            vs = str(vs_value).rjust(3, '0') + '↑'
                    except:
                        vs = "----"
                    self.digital_display.vs = vs
                    
                # Update button states based on LED states from FCU
                elif "cache: v:AirbusFBW/LOCilluminated val:" in text:
                    state = int(text.split("val:")[1].strip()) > 0
                    self.button_panel.update_button_state('LOC', state)
                elif "cache: v:AirbusFBW/AP1Engage val:" in text:
                    state = int(text.split("val:")[1].strip()) > 0
                    self.button_panel.update_button_state('AP1', state)
                elif "cache: v:AirbusFBW/AP2Engage val:" in text:
                    state = int(text.split("val:")[1].strip()) > 0
                    self.button_panel.update_button_state('AP2', state)
                elif "cache: v:AirbusFBW/ATHRengaged val:" in text:
                    state = int(text.split("val:")[1].strip()) > 0
                    self.button_panel.update_button_state('A/THR', state)
                elif "cache: v:AirbusFBW/EXPEDilluminated val:" in text:
                    state = int(text.split("val:")[1].strip()) > 0
                    self.button_panel.update_button_state('EXPED', state)
                elif "cache: v:AirbusFBW/APPRilluminated val:" in text:
                    state = int(text.split("val:")[1].strip()) > 0
                    self.button_panel.update_button_state('APPR', state)
                
                # Update the display
                self.digital_display.update_values(
                    self.digital_display.speed,
                    self.digital_display.heading,
                    self.digital_display.altitude,
                    self.digital_display.vs
                )

        except Exception as e:
            sys.stderr.write(f"Error processing update: {e}\n")
            sys.stderr.flush()
        
        # Only the text output is controlled by the output button
        if self.output_button.isChecked():
            self.output_text.append(text)
            self.output_text.verticalScrollBar().setValue(
                self.output_text.verticalScrollBar().maximum()
            )

    def update_status(self, status):
        self.status_label.setText(status)
        if "Error" in status:
            self.status_label.setProperty("status", "error")
        elif "Starting" in status or "Running" in status:
            self.status_label.setProperty("status", "running")
        else:
            self.status_label.setProperty("status", "")
        self.status_label.style().unpolish(self.status_label)
        self.status_label.style().polish(self.status_label)

    def on_worker_finished(self):
        self.start_button.setEnabled(True)
        self.output_button.setEnabled(False)
        self.output_button.setChecked(True)
        self.output_button.setText("Output: On")
        self.start_button.setText("Start FCU Controller")

    def get_sudo_password(self):
        password, ok = QInputDialog.getText(
            self, 
            'Sudo Authentication Required',
            'Please enter your sudo password:',
            QInputDialog.textEchoMode.Password
        )
        if ok and password:
            # Test the password
            test_process = subprocess.Popen(
                ['sudo', '-S', 'true'],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            try:
                _, stderr = test_process.communicate(input=password + '\n', timeout=5)
                if test_process.returncode == 0:
                    return password
                else:
                    QMessageBox.critical(self, "Error", "Incorrect password")
                    return None
            except:
                QMessageBox.critical(self, "Error", "Failed to verify password")
                return None
        return None

    def _init_system_tray(self):
        # Create system tray icon
        self.tray_icon = QSystemTrayIcon(self)
        self.tray_icon.setIcon(QIcon("propeller_icon.png"))  # Ensure the path is correct
        
        # Create tray menu
        tray_menu = QMenu()
        show_action = tray_menu.addAction("Show")
        show_action.triggered.connect(self.show)
        quit_action = tray_menu.addAction("Quit")
        quit_action.triggered.connect(QApplication.quit)
        
        self.tray_icon.setContextMenu(tray_menu)
        self.tray_icon.activated.connect(self._tray_icon_activated)
        self.tray_icon.show()
        
    def _tray_icon_activated(self, reason):
        if reason == QSystemTrayIcon.ActivationReason.Trigger:
            self.show()
            self.setWindowState(Qt.WindowState.WindowActive)
            
    def cleanup(self):
        """Cleanup resources before application exit"""
        if self.worker and self.worker.isRunning():
            # Stop the FCU worker
            winwing_fcu.running = False
            self.worker.stop()
            self.worker.wait()  # Wait for thread to finish
            
            # Restore stdout
            if sys.stdout != sys.__stdout__:
                sys.stdout = sys.__stdout__
            
            # Flush any pending I/O
            sys.stdout.flush()
            sys.stderr.flush()

    def shutdown_application(self):
        """Handle application shutdown"""
        self.cleanup()
        QApplication.quit()

    def check_updates(self):
        """Check for updates and show result"""
        self.update_button.setEnabled(False)
        self.update_button.setText("Checking...")
        
        class UpdateWorker(QThread):
            finished = pyqtSignal(str)
            
            def __init__(self, checker):
                super().__init__()
                self.checker = checker
                
            def run(self):
                try:
                    # Get file updates
                    response = requests.get(self.checker.base_url)
                    response.raise_for_status()
                    github_files = {f['name']: f['sha'] for f in response.json() 
                                  if f['name'] in self.checker.local_files}
                    
                    # Compare with local files
                    updates = []
                    for filename in self.checker.local_files:
                        if filename in github_files:
                            local_hash = self.checker.get_file_hash(filename)
                            if not local_hash:
                                updates.append(f"• {filename} (missing)")
                            elif local_hash != github_files[filename]:
                                updates.append(f"• {filename} (modified)")
                    
                    if updates:
                        msg = '<div style="font-size: 11px;">'
                        msg += "Updates available:<br><br>"
                        msg += "<br>".join(updates)
                        msg += "<br><br>Visit github.com/schenlap/winwing_fcu</div>"
                        self.finished.emit(msg)
                    else:
                        self.finished.emit('<div style="font-size: 11px;">All files are up to date</div>')
                        
                except Exception as e:
                    self.finished.emit(f'<div style="font-size: 11px;">Error checking updates: {str(e)}</div>')
        
        worker = UpdateWorker(self.update_checker)
        
        def on_finished(result):
            self.update_button.setEnabled(True)
            self.update_button.setText("Check Updates")
            
            if "Error" in result:
                QMessageBox.warning(self, "Update Check Failed", result)
            elif "Updates available" in result:
                QMessageBox.information(self, "Updates Available", result)
            else:
                QMessageBox.information(self, "No Updates", result)
            
            worker.deleteLater()
        
        worker.finished.connect(on_finished)
        worker.start()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setFont(QFont("Segoe UI", 10))
    
    # Create and show window
    window = MainWindow()
    window.show()
    
    # Start event loop
    sys.exit(app.exec()) 