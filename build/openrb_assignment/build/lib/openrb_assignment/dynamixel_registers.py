import numpy as np


class Reg:
    EEPROM_REGISTERS = []
    RAM_REGISTERS = []
    NAME_TO_REG = {}
    ADDR_TO_REG = {}
    READ_ONLY_REG = set()
    RW_REG = set()

    def __init__(self, name, addr, length=0, access='R', initial=None,
                 conv_factor=1, unsupported_motors=None):
        """
        Initialize a Dynamixel register object. The control table can be obtained e.g. from https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/
        Args:
            name (str): The name identifier for the register
            addr (int): The memory address of the register
            length (int, optional): The length of the register data in bytes.
            access (str, optional): Access permissions for the register ('R' for read-only, 'RW' for read-write).
            initial (optional): Initial value for the register.
            conv_factor (int, optional): Scaling factor for register value calculations.
            unsupported_motors (str | Sequence[str] | None, optional):
                Regex pattern or list of patterns for motor models that do not
                support this register.  Important for RW registers since if data
                is written to a group of addresses including unsupported
                registers, the whole write will fail.
        """
        self.name = name
        self.addr = addr
        self.len = length
        self.access = access
        self.initial = initial
        self.conv_factor = conv_factor

        patterns = unsupported_motors or []
        if isinstance(patterns, str):
            patterns = [patterns]
        self.unsupported_motors = patterns

        Reg.NAME_TO_REG[name] = self
        Reg.ADDR_TO_REG[addr] = self

        if self.addr < ADDR_CTRL_TABLE_START:
            Reg.EEPROM_REGISTERS.append(self)
        else:
            Reg.RAM_REGISTERS.append(self)
            if access == 'R':
                Reg.READ_ONLY_REG.add(self)
            elif access == 'RW':
                Reg.RW_REG.add(self)


# Protocol version
PROTOCOL_VERSION = 2.0

ADDR_CTRL_TABLE_START = 64
ADDR_CTRL_TABLE_END = 227

DEFAULT_POS_SCALE = 2.0 * np.pi / 4096  # 0.088 degrees
# See http://emanual.robotis.com/docs/en/dxl/x/xh430-v210/#goal-velocity
DEFAULT_VEL_SCALE = 0.229 * 2.0 * np.pi / 60.0  # 0.229 rpm
DEFAULT_CUR_SCALE = 1.34
DEFAULT_VOLT_SCALE = 0.1

# Control Table Address - EEPROM Area (Persists after reboot)
EEREG_MODEL_NUMBER    = Reg("MODEL_NUMBER", 0, 2, 'R', 1080)
EEREG_MODEL_INFO      = Reg("MODEL_INFO", 2, 4, 'R')
EEREG_FIRMWARE_VER    = Reg("FIRMWARE_VER", 6, 1, 'R')
EEREG_ID              = Reg("ID", 7, 1, 'RW', 1)
EEREG_BAUD_RATE       = Reg("BAUD_RATE", 8, 1, 'RW', 1)
EEREG_RETURN_DELAY    = Reg("RETURN_DELAY", 9, 1, 'RW', 250)
EEREG_DRIVE_MODE      = Reg("DRIVE_MODE", 10, 1, 'RW', 0)
EEREG_OPERATING_MODE  = Reg("OPERATING_MODE", 11, 1, 'RW', 3)
EEREG_SHADOW_ID       = Reg("SHADOW_ID", 12, 1, 'RW', 255)
EEREG_PROTOCOL_TYPE   = Reg("PROTOCOL_TYPE", 13, 1, 'RW', 2)
EEREG_HOMING_OFFSET   = Reg("HOMING_OFFSET", 20, 4, 'RW', 0)
EEREG_MOVING_THRESH   = Reg("MOVING_THRESH", 24, 4, 'RW', 10)
EEREG_TEMP_LIMIT      = Reg("TEMP_LIMIT", 31, 1, 'RW', 80)
EEREG_MAX_VOLTAGE     = Reg("MAX_VOLTAGE", 32, 2, 'RW', 160)
EEREG_MIN_VOLTAGE     = Reg("MIN_VOLTAGE", 34, 2, 'RW', 60)
EEREG_PWM_LIMIT       = Reg("PWM_LIMIT", 36, 2, 'RW', 885)
EEREG_VEL_LIMIT       = Reg("VEL_LIMIT", 44, 4, 'RW', 306)
EEREG_MAX_POSITION    = Reg("MAX_POSITION", 48, 4, 'RW', 4095)
EEREG_MIN_POSITION    = Reg("MIN_POSITION", 52, 4, 'RW', 0)
EEREG_STARTUP_CONFIG  = Reg("STARTUP_CONFIG", 60, 1, 'RW', 0)
EEREG_SHUTDOWN        = Reg("SHUTDOWN", 63, 1, 'RW', 52)

# Control Table Address
REG_TORQUE_ENABLE    = Reg("torque_enable", 64, 1, 'RW', 0)
REG_LED              = Reg("led", 65, 1, 'RW', 0)
REG_STATUS_RETURN    = Reg("STATUS_RETURN", 68, 1, 'RW', 2)
REG_REGISTERED_INST  = Reg("REGISTERED_INST", 69, 1, 'R', 0)
REG_HARDWARE_ERROR   = Reg("hwerror", 70, 1, 'R', 0)

# PID Gains
REG_VEL_I_GAIN      = Reg("velIgain", 76, 2, 'RW', 1920)
REG_VEL_P_GAIN      = Reg("velPgain", 78, 2, 'RW', 100)
REG_POS_D_GAIN      = Reg("posDgain", 80, 2, 'RW', 0)
REG_POS_I_GAIN      = Reg("posIgain", 82, 2, 'RW', 0)
REG_POS_P_GAIN      = Reg("posPgain", 84, 2, 'RW', 700)

# Feedforward Gains
REG_FF_2ND_GAIN     = Reg("ffgain1st", 88, 2, 'RW', 0)
REG_FF_1ST_GAIN     = Reg("ffgain2nd", 90, 2, 'RW', 0)

# Movement
REG_BUS_WATCHDOG    = Reg("BUS_WATCHDOG", 98, 1, 'RW', 0)
REG_GOAL_PWM        = Reg("gpwm", 100, 2, 'RW', 0)
REG_GOAL_CURR       = Reg("gcurr", 102, 2, 'RW', 0, DEFAULT_CUR_SCALE,
                         unsupported_motors=["xc430*"])
REG_GOAL_VEL        = Reg("gvel", 104, 4, 'RW', 0, DEFAULT_VEL_SCALE)
REG_PROF_ACCEL      = Reg("profacc", 108, 4, 'RW', 0)
REG_PROF_VEL        = Reg("profvel", 112, 4, 'RW', 0, DEFAULT_VEL_SCALE)
REG_GOAL_POS        = Reg("gpos", 116, 4, 'RW', 0, DEFAULT_POS_SCALE)

# Status
REG_REALTIME_TICK   = Reg("tick", 120, 2, 'R')
REG_MOVING          = Reg("moving", 122, 1, 'R', 0)
REG_MOVING_STATUS   = Reg("moving_status", 123, 1, 'R', 0)
REG_PRESENT_PWM     = Reg("ppwm", 124, 2, 'R', 0)
REG_PRESENT_CURR    = Reg("pcurr", 126, 2, 'R', 0, DEFAULT_CUR_SCALE,
                         unsupported_motors=["xc430*"])
REG_PRESENT_VEL     = Reg("pvel", 128, 4, 'R', 0, DEFAULT_VEL_SCALE)
REG_PRESENT_POS     = Reg("ppos", 132, 4, 'R', 0, DEFAULT_POS_SCALE)
REG_VEL_TRAJECTORY  = Reg("veltraj", 136, 4, 'R', 0, DEFAULT_VEL_SCALE)
REG_POS_TRAJECTORY  = Reg("postraj", 140, 4, 'R', 0, DEFAULT_POS_SCALE)
REG_PRESENT_VOLTAGE = Reg("pvolt", 144, 2, 'R', 0, DEFAULT_VOLT_SCALE)
REG_PRESENT_TEMP    = Reg("ptemp", 146, 1, 'R', 0, 1)
REG_BACKUP_READY    = Reg("BACKUP_READY", 147, 1, 'R')

SPL_REG_PRESENT_CURR_VEL_POS = Reg("pcurr_vel_pos", 126, 10, 'R')
SPL_REG_INDIRECT_ADDR = Reg("INDIRECT_ADDR", 168, 40, 'RW')
# hardcode this in dynamixel_client.py since it's different for each motor type
# SPL_REG_INDIRECT_DATA = Reg("INDIRECT_DATA", 208, 20, 'RW')
SPL_REG_CONTROL_TABLE = Reg("CONTROL_TABLE", ADDR_CTRL_TABLE_START, REG_PRESENT_TEMP.addr-ADDR_CTRL_TABLE_START+1, 'RW')