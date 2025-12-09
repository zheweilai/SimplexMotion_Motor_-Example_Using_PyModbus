# Register map for Simplex Motor Controller (SC series)

# ==========================================
#  Register Addresses 
# ==========================================
MOTOR_POSTITION             = 199        
MOTOR_SPEED                 = 201
MOTOR_TORQUE                = 202
MOTOR_TORQUE_MAX            = 203
MOTOR_OPTIONS               = 211
REG_KP                      = 299
REG_KI                      = 300
REG_KD                      = 301
RAMP_SPEED_MAX              = 350
RAMP_ACC_MAX                = 352
RAMP_DEC_MAX                = 353
MODE                        = 399
TARGET_INPUT                = 449

# ==========================================
#  Register Values 
# ==========================================
# Mode Register (400) options
MODE_OFF                    =   0           # Stop / Disable
MODE_RESET                  =   1
MODE_PWM                    =  10           # Open Loop
MODE_POSITION               =  20           # Closed Loop Position Control, no ramp [WARNING!] Too large target position changes can cause damage
MODE_POSITION_RAMP          =  21
MODE_ROTARY                 =  30