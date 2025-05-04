
#--------------------------------------------nau7802용(시작)
from machine import I2C, Pin, SoftI2C
from time import sleep_ms, ticks_ms
from array import array

class NAU7802:
    # I2C address
    NAU7802_ADDR = 0x2a
    # Registers
    PU_CTRL = 0x00  # Power-Up Control RW
    CTRL1 = 0x01  # Control 1 RW
    CTRL2 = 0x02  # Control 2 RW 
    ADCO_B2 = 0x12  # ADC_OUT[23:16] ROnly
    ADCO_B1 = 0x13  # ADC_OUT[16: 8] ROnly
    ADCO_B0 = 0x14  # ADC_OUT[ 7: 0] ROnly
    ADC = 0x15 #OTP_B1 과 공동 사용
    PGA = 0x1B # PGA Registers: 0x15 용도 결정
    PGA_PWR = 0x1C #POWER CONTROL Register
    # Other constants
    NAU7802_LDO_3V3 = 0b100
    NAU7802_PU_CTRL_AVDDS = 7
    
    # DEFINE I2C DEVICE BITS
    PGA_PWR_PGA_CAP_EN = 7 #Enable PGA
    CTRL2_CALS = 2
    PU_CTRL_RR = 0
    PU_CTRL_PUD = 1
    PU_CTRL_PUA = 2
    PU_CTRL_PUR = 3
    PU_CTRL_CR = 5
    # Calibration state
    CAL_SUCCESS = 0
    CAL_IN_PROGRESS = 1
    CAL_FAILURE = 2
    
    #함수들-------------------------------------
    def __init__(self, i2c, addr=0x2a, active_channels=1):
        self.i2c = i2c
        self.addr = addr
        self.active_channels = active_channels
        self.zero_offset = 0
        self.calibration_factor = 1
        self.buf = bytearray(4)
        self.max_arr = 1000
        self.data_arr_i = array('i', [0] * self.max_arr)
        self.offset = 0.0
        self.a_sparkfun_500g = 0.000325360053483067 #gain: 128
        self.reset()
        sleep_ms(500)
        #초기화 시작
        self.power_up()
        self.set_ldo_3v3()
        #self.set_gain_1()
        self.set_gain_128()
        self.set_sample_rate_80SPS()
        self.set_ADC_register()
        self.set_bit(self.PGA_PWR_PGA_CAP_EN, self.PGA_PWR)
        self.calibrate_afe()        
        
    def reg_write(self, reg, data):
        msg = bytearray()
        msg.append(data)
        self.i2c.writeto_mem(self.addr, reg, msg)
    def reg_read(self, reg, nbytes=1):
        if nbytes < 1:
            return bytearray()
        data = self.i2c.readfrom_mem(self.addr, reg, nbytes)
        return data    
    def set_bit(self, bit_number, register_address):
        value = self.reg_read(register_address, nbytes=1)[0]
        value |= (1 << bit_number)
        self.reg_write(register_address, value)
    def clear_bit(self, bit_number, register_address):
        value = self.reg_read(register_address, nbytes=1)[0]
        value &= ~(1 << bit_number)
        self.reg_write(register_address, value)
    def get_bit(self, bit_number, register_address):
        value = self.reg_read(register_address, nbytes=1)[0]
        return bool(value & (1 << bit_number))
    def reset(self):
        self.set_bit(self.PU_CTRL_RR, self.PU_CTRL)
        sleep_ms(1)
        self.clear_bit(self.PU_CTRL_RR, self.PU_CTRL)        
    def set_ldo_3v3(self):
        value = self.reg_read(self.CTRL1, nbytes=1)[0]
        value &= 0b11000111 #VLDO 4.5(default)
        value |= 0b00100000 #VLDO 3V3
        self.reg_write(self.CTRL1, value)
        self.set_bit(self.NAU7802_PU_CTRL_AVDDS, self.PU_CTRL)
    def set_gain_128(self):
        value = self.reg_read(self.CTRL1, nbytes=1)[0]
        value &= 0b11111000 #GAINS x1(default)
        value |= 0b00000111 #GAINS x128
        self.reg_write(self.CTRL1, value)
    def set_sample_rate_80SPS(self):
        value = self.reg_read(self.CTRL2, nbytes=1)[0]
        value &= 0b10001111
        value |= 0b00110000
        self.reg_write(self.CTRL2, value)
    def set_ADC_register(self):
        value = self.reg_read(self.PGA, nbytes=1)[0]
        value &= 0b01111111 #Read REG0x15 will read ADC registers
        self.reg_write(self.PGA, value)        
        value = self.reg_read(self.ADC, nbytes=1)[0]
        value &= 0b11001111 #REG_CPHS[1:0]
        value |= 0b00110000 #trned off, high('1') state
        self.reg_write(self.ADC, value)
    def begin_calibrate_afe(self):
        self.set_bit(self.CTRL2_CALS, self.CTRL2)
    def wait_for_calibrate_afe(self, timeout_ms=0):
        #timeout_ms=0 : 시간제한 없음
        begin = ticks_ms()
        cal_ready = self.cal_afe_status()
        while cal_ready == self.CAL_IN_PROGRESS:
            if timeout_ms > 0 and (ticks_ms() - begin) > timeout_ms:
                break
            sleep_ms(1)
            cal_ready = self.cal_afe_status()
        if cal_ready == self.CAL_SUCCESS:
            return True
        return False
    def cal_afe_status(self): #afe: Analog Front End
        if self.get_bit(2, self.CTRL2):
            return self.CAL_IN_PROGRESS
        if self.get_bit(3, self.CTRL2):
            return self.CAL_FAILURE
        return self.CAL_SUCCESS    
    def calibrate_afe(self):
        self.begin_calibrate_afe()
        return self.wait_for_calibrate_afe(1000)    
    def available(self):
        return self.get_bit(self.PU_CTRL_CR, self.PU_CTRL)    
    def get_reading(self):
        raw_data = self.reg_read(self.ADCO_B2, nbytes=3)
        value = (raw_data[0] << 16) | (raw_data[1] << 8) | (raw_data[2])
        
        if value > ((1 << 23) - 1):
            value -= (1 << 24)        
        return value
    def get_reading_adv(self, times=100):
        if (times > self.max_arr) or (times < 20):
            return 0.0
        i = 0
        while True:
            if self.available():
                self.data_arr_i[i] = self.get_reading()
                i += 1
            if i == times:
                break
        new_arr = self.data_arr_i[0:times]
        sorted_arr = sorted(new_arr)
        remove_top = 5
        remove_bottom = 5 
        num_eliminate = remove_top + remove_bottom
        if num_eliminate >= times:
            return 0.0
        new_arr = sorted_arr[remove_bottom:(times-remove_top)]
        return sum(new_arr)/(times-num_eliminate)        
    def power_up(self):
        self.set_bit(self.PU_CTRL_PUD, self.PU_CTRL) #Digital Power ON, first
        self.set_bit(self.PU_CTRL_PUA, self.PU_CTRL) #Analog Power ON, second
        counter = 0
        while True:
            if self.get_bit(self.PU_CTRL_PUR, self.PU_CTRL):
                break
            sleep_ms(1)
            counter += 1
            if counter > 100:
                return False
        return True          
#--------------------------------------------nau7802용(끝)
i2c = SoftI2C(scl=Pin(9), sda=Pin(8), freq=400000) 
nau7802 = NAU7802(i2c=i2c)
nau7802.offset = nau7802.get_reading_adv(times=300)
while True:
    if nau7802.available():
        print("{:.2f}".format((nau7802.get_reading_adv(times=300) - nau7802.offset)*nau7802.a_sparkfun_500g))
