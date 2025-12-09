import time
from simplexMotor import SimplexMotor
import logging
import reg_map as regmap


logging.basicConfig(
    level=logging.INFO, 
    format='%(asctime)s - %(name)s - [%(levelname)s] - %(message)s', 
    datefmt='%H:%M:%S'
)

logging.getLogger("pymodbus").setLevel(logging.WARNING)

motor = SimplexMotor(debug=True)
try:
    motor.connect() 

    motor.set_position(0) 

   
    print("Enabling Motor...")
    motor.set_mode(regmap.MODE_POSITION_RAMP)
    
    while True:
        try:
            str_in = input("Enter target position (degrees) or 'q' to quit: ")
            if str_in.lower() == 'q':
                break
                
            target_pos = float(str_in)
            
            
            motor.set_position(target_pos)
            time.sleep(0.5)
            motor.get_position()
            
        except ValueError:
            print("Invalid number!")

finally:
    print("Disabling motor...")
    motor.set_mode(regmap.MODE_RESET) 
    motor.close()