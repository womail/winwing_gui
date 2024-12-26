# WinWing FCU Controller GUI

A modern graphical interface for controlling WinWing FCU/EFIS panels with X-Plane flight simulator.

## Features

- Clean, modern Qt-based interface
- Digital display showing:
  - Speed
  - Heading
  - Altitude
  - Vertical Speed
- System tray integration
- Real-time hardware status monitoring
- Automatic updates checking
- Dark mode support
- Error logging and debugging capabilities

## Installation

### Requirements

- Python 3.6 or higher
- X-Plane 11/12
- Toliss Airbus aircraft
- WinWing FCU/EFIS hardware

### Python Dependencies

pip install PyQt6 pyusb


## Requirements

### Software Requirements
- Python 3.6 or higher
- X-Plane 11/12
- Toliss Airbus aircraft
- WinWing FCU/EFIS hardware

### Python Dependencies
- PyQt6 (GUI framework)
- pyusb (USB communication)
- requests (Updates checking)

## Installation

1. Clone the repository:

winwing-fcu-gui/
├── winwing_gui.py # Main GUI application
├── winwing_fcu.py # FCU communication logic
├── requirements.txt # Python dependencies
├── LICENSE # MIT license
└── README.md # This file

### Contributing
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Original FCU Controller by schenlap
- PyQt6 framework
- WinWing hardware specifications

## Support

For bugs and feature requests, please [open an issue](https://github.com/yourusername/winwing-fcu-gui/issues) on GitHub.

## FAQ

**Q: The FCU is not detected**
A: Check USB permissions and ensure libusb is installed correctly.

**Q: X-Plane connection fails**
A: Verify X-Plane is running and UDP port 49000 is not blocked by firewall.

**Q: The display shows dashes (---)**
A: This indicates no data is being received from X-Plane. Check your network connection.