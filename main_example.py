from machine import Pin, I2C, UART
import utime
import struct
import q4
import onefilter

uart0=UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

class MPU6050Data:
    def __init__(self):
        self.Gx=0
        self.Gy=0
        self.Gz=0
        self.Temperature=0
        self.Gyrox=0
        self.Gyroy=0
        self.Gyroz=0

class MPU6050:
    AccelerationFactor= 2.0/32768.0;   #assuming +/- 16G
    GyroFactor=500.0 / 32768.0;         #assuming 500 degree / sec

    # Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
    TemperatureGain = 1.0 / 340.0
    TemperatureOffset = 36.53

    MPU6050_RA_SMPLRT_DIV = 0x19
    MPU6050_RA_ACCEL_CONFIG = 0x1C
    MPU6050_RA_GYRO_CONFIG = 0x1B
    MPU6050_RA_PWR_MGMT_1 = 0x6B
    MPU6050_RA_PWR_MGMT_2 = 0x6C
    MPU6050_RA_INT_ENABLE = 0x38
    MPU6050_RA_INT_STATUS = 0x3A
    MPU6050_RA_FIFO_EN = 0x23
    MPU6050_RA_USER_CTRL = 0x6A
    MPU6050_RA_FIFO_COUNTH = 0x72
    MPU6050_RA_FIFO_R_W = 0x74
    MPU6050_RA_ACCEL_XOUT_H = 0x3B

    def reg_writeByte(self,reg_addr,value): # 写操作
        self.i2c.writeto_mem(self.MPU6050_ADDRESS,reg_addr,bytes((value,)))

    def reg_read(self,reg_addr, count):
        return self.i2c.readfrom_mem(self.MPU6050_ADDRESS,reg_addr,count)

    def __init__(self, bus=1, address=0x68, scl=Pin(15), sda=Pin(14), freq=100000):
        self.i2c = I2C(bus,scl=scl,sda=sda,freq=freq)
        self.MPU6050_ADDRESS = address
        self.setSampleRate(100) # 采样速率100Hz
        self.setGResolution(2) # 加速度=-4g
        self.setGyroResolution(250) # 角速度+-500deg/s
        self.reg_writeByte(self.MPU6050_RA_PWR_MGMT_1, 0b00000010) # 使用陀螺仪y轴作为时钟参考
        #Controls frequency of wakeups in accel low power mode plus the sensor standby modes
        self.reg_writeByte(self.MPU6050_RA_PWR_MGMT_2, 0x00) # 低电量模式
        # self.reg_writeByte(self.MPU6050_RA_INT_ENABLE, 0x01)
        # print(self.readStatus())
        self.fifoCount =0

    def readData(self):
        #read accelerometers , temperature and gyro
        GData = self.reg_read(self.MPU6050_RA_ACCEL_XOUT_H,14)
        #convert list of 14 values bytes into MPU6050Data struct in engineering units
        return self.convertData(GData)

    def convertData(self,ListData):
        ShortData = struct.unpack(">hhhhhhh", bytearray(ListData))
        #lets create the Data Class
        AccData = MPU6050Data()

        # first 3 short value are Accelerometer

        AccData.Gx = ShortData[0] * self.AccelerationFactor
        AccData.Gy = ShortData[1] * self.AccelerationFactor
        AccData.Gz = ShortData[2] * self.AccelerationFactor

        #temperature
        AccData.Temperature = ShortData[3] * self.TemperatureGain + self.TemperatureOffset

        #and the 3 last ar'e the gyro data

        AccData.Gyrox = ShortData[4] * self.GyroFactor
        AccData.Gyroy = ShortData[5] * self.GyroFactor
        AccData.Gyroz = ShortData[6] * self.GyroFactor

        return AccData

    def setSampleRate(self, Rate): # 设置采样速率，采样率=陀螺仪输出速率/（1+寄存器值）
        SampleReg =  int(( 8000 / Rate) -1)
        self.SampleRate = 8000.0 / (SampleReg + 1.0)
        self.reg_writeByte(self.MPU6050_RA_SMPLRT_DIV,SampleReg)

    def setGResolution(self, value): # G resolution 2,4,8 or 16G
        self.reg_writeByte(self.MPU6050_RA_ACCEL_CONFIG,{2 : 0 , 4 : 8 , 8 : 16 , 16 : 24}[value])
        self.AccelerationFactor= value/32768.0;
        
    def setGyroResolution(self, value): # G resolution +-250,500,1000,2000 deg/s
        self.reg_writeByte(self.MPU6050_RA_GYRO_CONFIG,{250 : 0 , 500 : 8 , 1000 : 16 , 2000 : 24}[value])
        self.GyroFactor= value/32768.0;

    def readStatus(self):
        return  self.reg_read(self.MPU6050_RA_INT_STATUS,1)

LED = Pin(25,Pin.OUT)
Yaw = 0
lasttime = 0
if __name__ == '__main__':
    mpu = MPU6050()
    while True:
        thistims = utime.ticks_ms()
        dt = (thistims-lasttime)/1000
        # print("running time:", dt)
        g=mpu.readData()
        utime.sleep_ms(25)

        r=onefilter.one_filter(g.Gyrox/131,g.Gyroy/131,g.Gyroz/131,g.Gx,g.Gy,g.Gz)
        uart0.write("{:.1f},{:.1f},{:.1f}\n".format(r[0],r[1],0))
        # print("{:.1f} {:.1f}".format(r[2],r[3]))
        
        # q=q4.IMUupdate(g.Gyrox/131*0.0174533,g.Gyroy/131*0.0174533,g.Gyroz/131*0.0174533,g.Gx,g.Gy,g.Gz) # 角度换弧度, 
        # uart0.write("{:.1f},{:.1f},{:.1f}\n".format(q[0],q[1],0))
        # print("{:.1f},{:.1f},{:.1f}\n".format(q[0],q[1],q[2]))

        lasttime = thistims