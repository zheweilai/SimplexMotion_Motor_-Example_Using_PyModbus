from pymodbus.client import ModbusSerialClient
import time
import reg_map
import logging

# Some default parameters depending on user setup
DEFAULT_PORT = 'COM3'
DEFAULT_BAUDRATE = 57600
DEFAULT_SLAVE_ID = 1

  

class SimplexMotor:
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
        self.debug = debug
        self.client: ModbusSerialClient | None = None

    def connect(self):
        if self.debug:
            print(f"[SimplexMotor] Connecting on {self.port} ...")

        self.client = ModbusSerialClient(
            port = self.port,
            baudrate = self.baudrate,
            timeout = self.timeout,
            parity = self.parity,
            stopbits = self.stopbits,
            bytesize = 8
        )

        ok = self.client.connect()
        if not ok:
            raise RuntimeError(f"Can NOT connect to {self.port}.")

        if self.debug:
            print("[SimplexMotor] Connected.")

    def get_torque_max(self) -> float:
        """
        Return maximum torque (Nm).
        """
        if self.client is None:
            raise RuntimeError("Client not connected.")

        resp = self.client.read_holding_registers(
            address = reg_map.MOTOR_TORQUE_MAX, 
            count = 1,
            slave = self.slave_id
        )
        if resp.isError():
            raise RuntimeError(f"Failed to read TorqueMax: {resp}")

        raw = resp.registers[0]   # [int16] in mNm
        torque_max = raw / 1000.0

        return torque_max

    def close(self):
        if self.client is not None:
            if self.debug:
                print("[SimplexMotor] Closing connection ...")
            self.client.close()



    def get_counters_per_rev(self) -> int:
        """
        According to MotorOptions (reg 212) → bit 12-15:
            0 → 4096 counts/rev
            1 → 8192 counts/rev
            2 → 16384 counts/rev
        """
        if self.client is None:
            raise RuntimeError("Client not connected.")

        resp = self.client.read_holding_registers(
            address = reg_map.MOTOR_OPTIONS,
            count = 1,
            slave = self.slave_id,
        )
        if resp.isError():
            raise RuntimeError(f"Failed to read MotorOptions: {resp}")

        options = resp.registers[0]

        resolution_bits = (options >> 12) & 0xF

        if resolution_bits == 0:
            return 4096
        elif resolution_bits == 1:
            return 8192
        elif resolution_bits == 2:
            return 16384
        else:
            print(f"Warning: Unknown resolution bits: {resolution_bits}, defaulting to 4096.")
            return 4096


    def get_position_counts(self) -> int:
        """
        Read position feedback in counts. (int32)
        """
        resp = self.client.read_holding_registers(address = reg_map.MOTOR_POSTITION, count = 2, slave = self.slave_id)
        msw = resp.registers[0]
        lsw = resp.registers[1]
        raw = msw << 16 | lsw
        print(f"Register Value: 0x{raw:08X}")
        if raw & 0x80000000:
            raw -= 0x100000000
        
        return raw

    def get_position(self) -> float:
        """
        Read position in degrees with +/- sign.
        """
        counts = self.get_position_counts()            
        cpr = float(self._get_counters_per_rev())
        ang = counts * 360.0 / cpr                      
                                    
        return ang
    
    def get_speed(self) -> float:
        """
        Read motor speed in deg/s with +/- sign.
        """
        resp = self.client.read_holding_registers(address=reg_map.MOTOR_SPEED, count = 1, slave = self.slave_id)      
        print(f"Raw Speed Register: 0x{resp.registers[0]:04X}")
        raw = resp.registers[0] & 0xFFFF
        if raw & 0x8000:
            raw -= 0x10000

        cpr = float(self.get_counters_per_rev())  
        speed_dps = raw * 16.0 * 360.0 / cpr        

        return speed_dps
    
    def set_position_resolution(self, mode: int):
        """
        set the counters per revolution.
        mode = 0 → 4096 cpr
        mode = 1 → 8192 cpr
        mode = 2 → 16384 cpr
        """
        if mode not in (0, 1, 2):
            raise ValueError("mode must be 0, 1, or 2")

        resp = self.client.read_holding_registers(
            address = reg_map.MOTOR_OPTIONS,
            count = 1,
            slave = self.slave_id,
        )
        if resp.isError():
            raise RuntimeError(f"Read MotorOptions failed: {resp}")

        options = resp.registers[0]

        options_new = (options & 0x0FFF) | (mode << 12)
        print(f"Setting MotorOptions from 0x{options:04X} to 0x{options_new:04X}")

        wr = self.client.write_register(
            address = reg_map.MOTOR_OPTIONS,
            value = options_new,
            slave = self.slave_id,
        )
        if wr.isError():
            raise RuntimeError(f"Write MotorOptions failed: {wr}")
        
    def get_mode(self) -> int:
        """
        Get current motor mode.
        """
        resp = self.client.read_holding_registers(
            address = reg_map.MODE,
            count = 1,
            slave = self.slave_id,
        )
        if resp.isError():
            raise RuntimeError(f"Read Mode failed: {resp}")

        mode = resp.registers[0]
        return mode
    
    
if __name__ == "__main__":
    motor = SimplexMotor(debug=True)
    try:
        motor.connect()
        
          
        while True:
            position = motor.get_position()
            print(f"Position: {position:.2f} deg")
            time.sleep(1)
    finally:
        motor.close()
