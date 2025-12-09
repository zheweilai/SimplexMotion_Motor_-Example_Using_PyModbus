import time
from simplexMotor import SimplexMotor
import logging
import reg_map as regmap

logging.basicConfig(
    level=logging.DEBUG, 
    format='%(asctime)s - %(name)s - [%(levelname)s] - %(message)s', 
    datefmt='%H:%M:%S'
)
logging.getLogger("pymodbus").setLevel(logging.WARNING)

motor = SimplexMotor(debug=True)
try:
    motor.connect()
  
    motor.set_mode(regmap.MODE_POSITION_RAMP)
    mode = motor.get_mode()
    motor.get_position_counts()

    motor.set_target_position_counts(500)
    time.sleep(10) 
    motor.get_position_counts()
    motor.set_target_position_counts(-500)
    time.sleep(1) 
    motor.get_position_counts()
    motor.set_mode(0)
finally:
    motor.close()