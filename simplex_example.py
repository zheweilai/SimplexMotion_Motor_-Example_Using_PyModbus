import time
from simplexMotor import SimplexMotor
import logging
import reg_map as regmap

logging.basicConfig(
    level=logging.DEBUG, 
    format='%(asctime)s - %(name)s - [%(levelname)s] - %(message)s', 
    datefmt='%H:%M:%S'
)
logging.getLogger("pymodbus").setLevel(logging.DEBUG)

motor = SimplexMotor(debug=True)
try:
    motor.connect() 
    motor.set_mode(regmap.MODE_RESET)
    motor.beep(duration_ms=1000)
    
finally:
    motor.close()