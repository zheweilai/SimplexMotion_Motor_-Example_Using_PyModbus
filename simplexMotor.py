from pymodbus.client import ModbusSerialClient
import time
import reg_map
import logging
from typing import Union

# Default communication parameters
DEFAULT_PORT = 'COM3'
DEFAULT_BAUDRATE = 57600
DEFAULT_SLAVE_ID = 1

class SimplexMotor:
    # =========================================================
    #       Lifecycle Methods (Initialization & Connection)
    # =========================================================
    def __init__(self, port: str = DEFAULT_PORT, 
                 baudrate: int = DEFAULT_BAUDRATE, 
                 slave_id: int = DEFAULT_SLAVE_ID,
                 timeout: float = 1.0,
                 parity: str = 'E',      
                 stopbits: int = 1,
                 debug: bool = True):
        
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.timeout = timeout 
        self.parity = parity
        self.stopbits = stopbits

        self.cpr = 4096  # Default encoder resolution (counts per revolution)
        
        # Setup Logger
        # Naming convention: ClassName.SlaveID (e.g., SimplexMotor.1)
        self.logger_name = f"SimplexMotor.{self.slave_id}"
        self.logger = logging.getLogger(self.logger_name)
        
        # Set instance-level logging threshold
        if debug:
            self.logger.setLevel(logging.DEBUG)
        else:
            self.logger.setLevel(logging.INFO)

        self.client: ModbusSerialClient | None = None

    def connect(self):
        """
        Establish connection to the Modbus serial client.
        """
        self.logger.info(f"Connecting to Simplex Motor on {self.port} at {self.baudrate} bps ...")

        self.client = ModbusSerialClient(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            parity=self.parity,
            stopbits=self.stopbits,
            bytesize=8
        )

        ok = self.client.connect()
        self.cpr = self.get_counters_per_rev()
        if not ok:
            # Log error before raising exception to ensure audit trail
            self.logger.error(f"Can NOT connect to {self.port}.")
            raise RuntimeError(f"Can NOT connect to {self.port}.")

        self.logger.info("Connected.")

    def close(self):
        """
        Close the serial connection safely.
        """
        if self.client is not None:
            self.logger.info("Closing connection.")
            self.client.close()

    # =========================================================
    #       Getters Methods
    # =========================================================
    def get_torque_max(self) -> float:
        """
        Read maximum torque setting.
        Returns:
            float: Torque in Nm.
        """
        self._check_connection()

        resp = self.client.read_holding_registers(
            address=reg_map.MOTOR_TORQUE_MAX, 
            count=1,
            slave=self.slave_id
        )
        
        if resp.isError():
            self._handle_modbus_error(resp, "TorqueMax")

        raw = resp.registers[0]   # Unit: mNm (int16)
        torque_max = raw / 1000.0
        
        self._log_debug_reg("TorqueMax", raw, torque_max, "Nm")
        
        return torque_max

    def get_counters_per_rev(self) -> int:
        """
        Read MotorOptions to determine encoder resolution.
        Bit 12-15 of reg 212:
            0 -> 4096 cpr
            1 -> 8192 cpr
            2 -> 16384 cpr
        """
        self._check_connection()

        resp = self.client.read_holding_registers(
            address=reg_map.MOTOR_OPTIONS,
            count=1,
            slave=self.slave_id,
        )
        
        if resp.isError():
            self._handle_modbus_error(resp, "MotorOptions")

        raw = resp.registers[0]
        resolution_bits = (raw >> 12) & 0xF
        
        # Decode resolution bits
        if resolution_bits == 0:
            cpr = 4096
        elif resolution_bits == 1:
            cpr = 8192
        elif resolution_bits == 2:
            cpr = 16384
        else:
            self.logger.warning(f"Unknown resolution bits: {resolution_bits}, defaulting to 4096.")
            cpr = 4096

        self._log_debug_reg("MotorOptions", raw, cpr, "cpr")
        
        return cpr

    def get_position_counts(self) -> int:
        """
        Read raw position feedback in encoder counts (Int32).
        """
        self._check_connection()

        resp = self.client.read_holding_registers(
            address=reg_map.MOTOR_POSTITION, 
            count=2, 
            slave=self.slave_id
        )

        if resp.isError():
            self._handle_modbus_error(resp, "Position")

        msw = resp.registers[0]
        lsw = resp.registers[1]
        
        # Combine into 32-bit unsigned
        raw_uint32 = (msw << 16) | lsw
        
        # Convert to signed 32-bit integer
        if raw_uint32 & 0x80000000:
            counts = raw_uint32 - 0x100000000
        else:
            counts = raw_uint32
        
        # Renamed label to distinguish from 'get_position' (degrees)
        self._log_debug_reg("PosCounts", raw_uint32, counts, "cnts")
        a = counts
        a.to_bytes(4, byteorder='big', signed=True)
        print(a)
        return counts

    def get_position(self) -> float:
        """
        Calculate position in degrees based on counts and CPR.
        Returns:
            float: Position in degrees (+/-).
        """
        # Fetch raw counts
        counts = self.get_position_counts()          
        position_deg = counts * 360.0 / self.cpr                      

        self._log_debug_reg("Position", counts, position_deg, "deg")                            
        return position_deg
    
    def get_speed(self) -> float:
        """
        Read motor speed in deg/s.
        """
        self._check_connection()

        resp = self.client.read_holding_registers(
            address=reg_map.MOTOR_SPEED, 
            count=1, 
            slave=self.slave_id
        )

        if resp.isError():
            self._handle_modbus_error(resp, "Speed")

        raw = resp.registers[0]

        # Handle Signed Int16 conversion
        if raw & 0x8000:
            raw_signed = raw - 0x10000
        else:
            raw_signed = raw  
        
          
        speed_deg_s = raw_signed * 16.0 * 360.0 / self.cpr        

        self._log_debug_reg("Speed", raw, speed_deg_s, "deg/s")

        return speed_deg_s
    
    def get_PID_gains(self) -> tuple[float, float, float]:
        """
        Read PID gains (Kp, Ki, Kd).
        Returns:
            tuple: (Kp, Ki, Kd)
        """
        self._check_connection()

        resp = self.client.read_holding_registers(
            address=reg_map.REG_KP,
            count=3,
            slave=self.slave_id,
        )

        if resp.isError():
            self._handle_modbus_error(resp, "PID Gains")

        kp = resp.registers[0]
        ki = resp.registers[1]
        kd = resp.registers[2]


        self._log_debug_reg("Kp", kp, kp, "")
        self._log_debug_reg("Ki", ki, ki, "")
        self._log_debug_reg("Kd", kd, kd, "")

        return (kp, ki, kd)


    def get_mode(self) -> int:
        """
        Get current motor operation mode.
        """
        self._check_connection()

        resp = self.client.read_holding_registers(
            address=reg_map.MODE,
            count=1,
            slave=self.slave_id,
        )

        if resp.isError():
            self._handle_modbus_error(resp, "Mode")

        mode = resp.registers[0]
        self._log_debug_reg("Mode", mode, mode, "mode")

        return mode
    
    def get_ramp_speed_max(self) -> float:
        """
        Read maximum ramp speed in deg/s.
        """
        self._check_connection()

        resp = self.client.read_holding_registers(
            address=reg_map.RAMP_SPEED_MAX,
            count=1,
            slave=self.slave_id,
        )

        if resp.isError():
            self._handle_modbus_error(resp, "RampSpeedMax")

        raw = resp.registers[0]
        ramp_speed_max = raw * 16.0 * 360.0 / self.cpr

        self._log_debug_reg("RampSpeedMax", raw, ramp_speed_max, "deg/s")

        return ramp_speed_max
    
    def get_ramp_acc_dec_max(self) -> tuple[float, float]:
        """
        Read maximum ramp acceleration and deceleration in deg/s².
        Returns:
            tuple: (ramp_acc_max, ramp_dec_max)
        """
        self._check_connection()

        resp = self.client.read_holding_registers(
            address=reg_map.RAMP_ACC_MAX,
            count=2,
            slave=self.slave_id,
        )

        if resp.isError():
            self._handle_modbus_error(resp, "RampAccDecMax")

        raw_acc = resp.registers[0]
        raw_dec = resp.registers[1]

        ramp_acc_max = raw_acc * 256 * 360.0 / self.cpr
        ramp_dec_max = raw_dec * 256 * 360.0 / self.cpr

        self._log_debug_reg("RampAccMax", raw_acc, ramp_acc_max, "deg/s²")
        self._log_debug_reg("RampDecMax", raw_dec, ramp_dec_max, "deg/s²")

        return (ramp_acc_max, ramp_dec_max)

    # =========================================================
    #       Setters Methods
    # =========================================================
    def set_rev_resolution(self, mode: int):
        """
        Set encoder resolution mode.
        Args:
            mode: 0(4096), 1(8192), or 2(16384).
        """
        self._check_connection()

        if mode not in (0, 1, 2):
            self.logger.error(f"Invalid resolution mode: {mode}")
            raise ValueError("mode must be 0, 1, or 2")

        # 1. Read-Modify-Write: Read current value
        resp = self.client.read_holding_registers(
            address=reg_map.MOTOR_OPTIONS,
            count=1,
            slave=self.slave_id,
        )
        if resp.isError():
            self._handle_modbus_error(resp, "MotorOptions")

        options_old = resp.registers[0]

        # 2. Modify: Update only the relevant bits
        options_new = (options_old & 0x0FFF) | (mode << 12)

        # 3. Optimize: Skip write if value hasn't changed
        if options_old == options_new:
            self.logger.debug(f"MotorOptions already set to mode {mode}. Skipping write.")
            return

        # 4. Log the change before writing
        self._log_change_info("MotorOptions", options_old, options_new, f"Set Mode: {mode}")

        # 5. Write back to register
        wr = self.client.write_register(
            address=reg_map.MOTOR_OPTIONS,
            value=options_new,
            slave=self.slave_id,
        )
        if wr.isError():
            self.logger.error(f"Write MotorOptions failed: {wr}")
            raise RuntimeError(f"Write MotorOptions failed: {wr}")
        time.sleep(0.02)
        self.cpr = self.get_counters_per_rev()
    
    def set_target_position_counts(self, target_counts: int):
        """
        Set target position in encoder counts (Int32).
        Args:
            target_counts: Target position in counts.
        """
        self._check_connection()

        
        # Convert to unsigned 32-bit representation
        if target_counts < 0:
            target_uint32 = target_counts + 0x100000000
        else:
            target_uint32 = target_counts

        msw = (target_uint32 >> 16) & 0xFFFF
        lsw = target_uint32 & 0xFFFF

        # Log the intended change
        self._log_change_info("TargetPosCnts", 0, target_uint32, f"Set to {target_counts} cnts")

        # Write MSW and LSW to consecutive registers
        wr = self.client.write_registers(
            address=reg_map.TARGET_INPUT,
            values=[msw, lsw],
            slave=self.slave_id,
        )
        if wr.isError():
            self.logger.error(f"Write TargetPosition failed: {wr}")
            raise RuntimeError(f"Write TargetPosition failed: {wr}")
    
    def set_position(self, position_deg: float):
        """
        Set target position in degrees.
        Args:
            position_deg: Target position in degrees.
        """
          
        target_counts = int(round(position_deg * self.cpr / 360.0))

        self.set_target_position_counts(target_counts)
        

    def set_mode(self, mode: int):
        """
        Set motor operation mode.
        Args:
            mode: Mode value to set.
        """
        self._check_connection()

        # Log the intended change
        self._log_change_info("Mode", 0, mode, f"Set to {mode}")

        wr = self.client.write_register(
            address = reg_map.MODE,
            value = mode,
            slave = self.slave_id,
        )
        if wr.isError():
            self.logger.error(f"Write Mode failed: {wr}")
            raise RuntimeError(f"Write Mode failed: {wr}")
        
    def set_PID_gains(self, kp: int, ki: int, kd: int):
        """
        Set PID gains (Kp, Ki, Kd).
        Note: According to the manual, these are raw int16 values, not floats.
        """
        self._check_connection()

        
        for name, val in [("Kp", kp), ("Ki", ki), ("Kd", kd)]:
            if not (-32768 <= val <= 32767):
                raise ValueError(f"{name} value {val} is out of int16 range!")

        self._log_change_info("PID_Gains", 0, 0, f"Set Kp={kp}, Ki={ki}, Kd={kd}")

       
        wr = self.client.write_registers(
            address=reg_map.REG_KP,
            values=[kp, ki, kd],
            slave=self.slave_id,
        )
        
        if wr.isError():
            self.logger.error(f"Write PID Gains failed: {wr}")
            raise RuntimeError(f"Write PID Gains failed: {wr}")
        
    def set_ramp_speed_max(self, ramp_speed_deg_s: int):
        """
        Set maximum ramp speed in deg/s.
        Args:
            ramp_speed_deg_s: Ramp speed in deg/s.
        """
        self._check_connection()

        # Convert to raw register value
        raw = int(round(ramp_speed_deg_s * self.cpr / (16.0 * 360.0)))

        self._log_change_info("RampSpeedMax", 0, raw, f"Set to {ramp_speed_deg_s} deg/s")

        wr = self.client.write_register(
            address=reg_map.RAMP_SPEED_MAX,
            value=raw,
            slave=self.slave_id,
        )
        if wr.isError():
            self.logger.error(f"Write RampSpeedMax failed: {wr}")
            raise RuntimeError(f"Write RampSpeedMax failed: {wr}")
        
    def set_ramp_acc_dec_max(self, ramp_acc_deg_s2: int, ramp_dec_deg_s2: int):
        """
        Set maximum ramp acceleration and deceleration in deg/s².
        Args:
            ramp_acc_deg_s2: Ramp acceleration in deg/s².
            ramp_dec_deg_s2: Ramp deceleration in deg/s².
        """
        self._check_connection()

        # Convert to raw register values
        raw_acc = int(round(ramp_acc_deg_s2 * self.cpr / (256.0 * 360.0)))
        raw_dec = int(round(ramp_dec_deg_s2 * self.cpr / (256.0 * 360.0)))

        self._log_change_info("RampAccMax", 0, raw_acc, f"Set to {ramp_acc_deg_s2} deg/s²")
        self._log_change_info("RampDecMax", 0, raw_dec, f"Set to {ramp_dec_deg_s2} deg/s²")

        wr = self.client.write_registers(
            address=reg_map.RAMP_ACC_MAX,
            values=[raw_acc, raw_dec],
            slave=self.slave_id,
        )
        if wr.isError():
            self.logger.error(f"Write RampAccDecMax failed: {wr}")
            raise RuntimeError(f"Write RampAccDecMax failed: {wr}")


    def beep(self, duration_ms: int = 500):
        """
        Make the motor emit a beep sound for a specified duration.
        Args:
            duration_ms: Duration of the beep in milliseconds.
            amplitude: Sound volume (TargetInput) Usually using 100
        """
        self._check_connection()
        amplitude = 100
        # Log
        self._log_change_info("Beep", 0, amplitude, f"Beeping for {duration_ms} ms")

        
        self.client.write_registers(
            address=reg_map.TARGET_INPUT, 
            values=[0, amplitude],           
            slave=self.slave_id
        )

        
        self.set_mode(60) 

        
        time.sleep(duration_ms / 1000.0)

    
        self.set_mode(0) 
        
      


    # =========================================================
    #       Internal Helpers (Private Methods)
    # =========================================================

    def _check_connection(self):
        """Ensure client is connected before performing operations."""
        if self.client is None:
            self.logger.error("Client not connected.")
            raise RuntimeError("Client not connected.")

    def _handle_modbus_error(self, resp, context: str):
        """Standardized error handling for Modbus responses."""
        self.logger.error(f"Failed to read {context}: {resp}")
        raise RuntimeError(f"Failed to read {context}: {resp}")

    def _log_change_info(self, name: str, old_hex: int, new_hex: int, note: str = ""):
        """
        Helper method to log state changes at INFO level.
        Format: [Name] Change: 0xOld -> 0xNew | Note
        """
        # Adjusted width to accommodate longer names like 'MotorOptions'
        NAME_WIDTH = 12
        
        hex_width = 8 if (old_hex > 0xFFFF or new_hex > 0xFFFF) else 4
        
        msg = (f"[{name:<{NAME_WIDTH}}] "
               f"Change: 0x{old_hex:0{hex_width}X} -> 0x{new_hex:0{hex_width}X} | {note}")
        
        self.logger.info(msg)

    def _log_debug_reg(self, name: str, raw: int, val: Union[float, int], unit: str = ""):
        """
        Helper method to format and log register values at DEBUG level.
        Format: [Name] Raw: 0xHex -> Val: Value Unit
        """
        NAME_WIDTH = 12
        
        hex_width = 8 if raw > 0xFFFF else 4
        
        if isinstance(val, float):
            val_str = f"{val:>.3f}"
        else:
            val_str = f"{val:>}"
            
        msg = f"[{name:<{NAME_WIDTH}}] Raw: 0x{raw:0{hex_width}X} -> Val: {val_str:>10} {unit}"
        
        self.logger.debug(msg)
    
