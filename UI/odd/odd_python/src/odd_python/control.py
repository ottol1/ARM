from typing import Tuple, List
from serial import Serial
import subprocess
import logging

class ODDRobot:
    def __init__(self, logger=logging.getLogger(__name__)) -> None:
        self.logger = logger
        self._init_variables()
        self.connect()
    
    
    def _init_variables(self) -> None:
        self.serial = None
        self.motor_commands: List[float] = [0.0, 0.0]  # L, R (rpm)
        self.motor_left_pid: List[float] = [0.5, 0.01, 0.01] # L (P I D)
        self.motor_right_pid: List[float] = [0.5, 0.01, 0.01] # R (P I D)
        self.encoders: List[int] = [0, 0]  # L, R (counts)
        self.rpm: List[float] = [0.0, 0.0]  # L, R (rpm)
        self.orientation: List[float] = [0.0, 0.0, 0.0]  # X Y Z (deg)
        self.acceleration: List[float] = [0.0, 0.0, 0.0]  # X Y Z (m/s^2)
        self.bump_sensors: List[bool] = [False, False, False, False]  # right, front, left, back
        self.battery_voltage: float = 0.0  # Volts
        self.temperature: int = 0  # Celsius
        
    
    def connect(self) -> None:
        port = "/dev/ttyACM0"
        grep = subprocess.run("udevadm info -q property /dev/ttyACM0 | grep 'ID_MODEL_ID='", shell=True, capture_output=True, text=True)
        model_id_split = grep.stdout.strip().split("=")
        if len(model_id_split) != 2:
            return
        
        if model_id_split[1] == "ff48":
            port = "/dev/ttyACM1"
        
        self.close()
        self.serial: Serial = Serial(port, 115200, timeout=0.1)
        if self.serial.is_open:
            self.logger.info("Serial opened!")
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
        else:
            self.logger.error("Serial couldn't open!")
    
    
    def __del__(self) -> None:
        self.close()


    def close(self) -> None:
        self.motor_commands = [0.0, 0.0]
        self.communicate()
        if self.serial and self.serial.is_open:
            self.serial.close()
    
    
    def _read_msg_line(self, line: str) -> None:
        self.logger.info(line)
    
    
    def _read_data_line(self, line: str) -> None:
        if line[0] not in '1234567890-':
            _read_msg_line(line)
            return
        
        str_data: List[str] = line.split(",")
        
        if len(str_data) != 16:
            self.logger.error(f"Data had {len(str_data)} entries! It should have 16!")
            return
        else:
            self.logger.debug(line)
        
        try:
            self.rpm[0] = float(str_data[0])
            self.rpm[1] = float(str_data[1])
            self.encoders[0] = int(str_data[2])
            self.encoders[1] = int(str_data[3])
            self.orientation[0] = float(str_data[4])
            self.orientation[1] = float(str_data[5])
            self.orientation[2] = float(str_data[6])
            self.acceleration[0] = float(str_data[7])
            self.acceleration[1] = float(str_data[8])
            self.acceleration[2] = float(str_data[9])
            self.bump_sensors[0] = str_data[10] == "1"
            self.bump_sensors[1] = str_data[11] == "1"
            self.bump_sensors[2] = str_data[12] == "1"
            self.bump_sensors[3] = str_data[13] == "1"
            self.battery_voltage = float(str_data[14])
            self.temperature = int(str_data[15])
        except ValueError as e:
            self.logger.info(f"Could not read '{line}'")
        
    
    def communicate(self) -> None:
        if not self.serial or not self.serial.is_open:
            return
        
        data_line: str = self.serial.readline().decode("ascii")
        if len(data_line) > 0:
            self._read_data_line(data_line)
        else:
            self.logger.error("No serial data from the Arduino.")
        send = f"{self.motor_commands[0]},{self.motor_commands[1]},"
        send += ",".join(f"{x}" for x in (self.motor_left_pid + self.motor_right_pid))
        send += "\n"
        self.logger.debug(send)
        self.serial.write(send.encode("ascii"))
