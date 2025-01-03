import serial
import time

class SerialConnection():
    '''
    This class is used to control the motor using serial communication.
    id: 1 for x-axis motor, 2 for y-axis motor, 3 for z-axis motor
    port: hardware port number, modify according to actual situation
    baudrate: baud rate
    parity: parity bit
    stopbits: stop bit
    bytesize: data bit
    timeout: timeout time
    '''
    def __init__(
        self,
        id,                                # o
        port='/dev/ttyUSB1',               # 硬件端口号，根据实际情况修改
        baudrate=19200,                    # 波特率
        parity=serial.PARITY_EVEN,         # 校验位
        stopbits=serial.STOPBITS_ONE,      # 停止位
        bytesize=serial.EIGHTBITS,         # 数据位
        timeout=1,                         # 超时时间
    ):
        
        self.id = id
        self.id_hex = hex(id)[2:].upper().zfill(2)
        self.port = port
        self.baudrate = baudrate
        self.parity = parity
        self.stopbits = stopbits
        self.bytesize = bytesize
        self.timeout = timeout
        self.ser = None
        
        self.velocities = [0] * 16
        self.velocities_updata = False
        
        self.accelerationtimes = [0] * 16
        self.accelerationtimes_updata = False
        
        self.waittimes = [0] * 16
        self.waittimes_updata = False
    
    def connect(self):
        '''
        Connect to the serial port
        rtscts: enable hardware flow control
        '''
        self.ser = serial.Serial(
            self.port,
            self.baudrate,
            parity=self.parity,
            stopbits=self.stopbits,
            bytesize=self.bytesize,
            timeout=self.timeout,
            rtscts=True             
        )
        if self.ser.isOpen():
            print("Serial port opened successfully")
        else:
            print("Error opening serial port")

    def write_hex(self, data, wait=0.25):
        self.ser.write(bytes.fromhex(data))
        time.sleep(wait)
        ret = self.read_hex()
        success = False
        data_hex = bytes.fromhex(data).hex().upper()
        if data_hex[2:4] == '06':
            if ret == bytes.fromhex(data).hex().upper():
                success = True
            else:
                success = False
        elif data_hex[2:4] == '03':
            num = int(data_hex[8:12], 16) * 4
            if ret == self.crc16(self.id_hex + '03' + (hex(int(data_hex[8:12], 16)*2)[2:].upper().zfill(2)) + ret[6:6+num]):
                success = True
            else:
                success = False
        elif data_hex[2:4] == '10':
            num = int(data_hex[8:12]) * 4
            if ret == self.crc16(self.id_hex + '10' + data_hex[4:12]):
                success = True
            else:
                success = False
            
        return success, ret

    def read_hex(self):
        return self.ser.read_all().hex().upper()
    
    def close(self):
        self.ser.close()
        if not self.ser.isOpen():
            print("Serial port closed successfully")
        else:
            print("Error closing serial port")
            
    def set_position_mode(self):
        self.write_hex(self.crc16(self.id_hex + '06 17 02 00 1C'), 0.05)  #vdi2多段位置使能
        self.write_hex(self.crc16(self.id_hex + '06 17 00 00 01'), 0.05)  #vd1伺服使能
        self.write_hex(self.crc16(self.id_hex + '06 02 00 00 01'), 0.05)  #控制模式为位置模式
        self.write_hex(self.crc16(self.id_hex + '06 05 00 00 02'), 0.05)  #多段位置指令给定
        self.write_hex(self.crc16(self.id_hex + '06 11 00 00 03'), 0.05)  #单次运行
        self.write_hex(self.crc16(self.id_hex + '06 11 04 00 01'), 0.05)  #绝对值运行
        # print("Position mode set successfully")

    def set_velocity_mode(self):
        self.write_hex(self.crc16(self.id_hex + '06 02 00 00 00'), 0.05)  #控制模式为速度模式
        self.write_hex(self.crc16(self.id_hex + '06 17 00 00 01'), 0.05)  #vd1为伺服使能
        # print("Velocity mode set successfully")

    def set_init_mode(self):
        self.write_hex(self.crc16(self.id_hex + '06 02 00 00 01'), 0.05)  #控制模式为位置模式
        self.write_hex(self.crc16(self.id_hex + '06 17 00 00 01'), 0.05)  #vd1为伺服使能
        self.write_hex(self.crc16(self.id_hex + '06 05 1E 00 01'), 0.05)  #di控制原点功能
        self.write_hex(self.crc16(self.id_hex + '06 05 1F 00 07'), 0.05)  #正向限位为原点
        self.write_hex(self.crc16(self.id_hex + '06 17 04 00 20'), 0.05)  #vdi3为原点使能
        self.write_hex(self.crc16(self.id_hex + '06 03 10 00 00'), 0.05)  #di8无功能
        self.write_hex(self.crc16(self.id_hex + '06 11 04 00 01'), 0.05)  #绝对值运行
        # print("Init mode set successfully")
    
    def set_positions_value(self, positions):
        if len(positions) > 16 or len(positions) < 1:
            print("Error: positions should be a list with 1 to 16 elements")
            return

        num = len(positions)
        num_hex = hex(num)[2:].upper().zfill(4)
        data = self.id_hex + '06 11 01' + num_hex
        # self.write_hex(self.crc16(data), 0.05)
    
        wait = 0
        for i in range(num):
            position = (positions[i]) / 0.08 * 10000
            position = int(position)
            d = position - (self.get_current_position() / 0.08 * 10000)
            if position >= 0:
                position_temp_hex = hex(int(position))[2:].upper().zfill(8)
                position_hex = position_temp_hex[4:8] + position_temp_hex[0:4]
            else:
                # 取补码
                position_temp_hex = hex(int(position + 2**32) % 2**32)[2:].upper().zfill(8)
                position_hex = position_temp_hex[4:8] + position_temp_hex[0:4]
                
            position_address_hex = hex(i * 5 + 12)[2:].upper().zfill(2)
            data = self.id_hex + '10 11' + position_address_hex + '00 02 04' + position_hex
            data = self.crc16(data)
            self.write_hex(data, 0.05)
            # print("Position set to {} successfully".format(positions[i]))
            
            if not self.velocities_updata:
                self.get_velocities_value()
            if not self.accelerationtimes_updata:
                self.get_accelerationtimes_value()
            if not self.waittimes_updata:
                self.get_waittimes_value()
            
            velocity_value = self.velocities[i]
            acceleration_value = self.accelerationtimes[i]
            waittime_value = self.waittimes[i]
            
            wait = wait + (abs(d) / 10000) / velocity_value * 60 + acceleration_value / 1000 * 2 + waittime_value / 1000
            # print("Wait time: {}".format(wait))
                    
        return wait
    
    # 设置多段位置模式下的速度
    def set_velocities_value(self, velocities):
        if len(velocities) > 16 or len(velocities) < 1:
            print("Error: velocities should be a list with 1 to 16 elements")
            return
        num = len(velocities)
        for i in range(num):
            if velocities[i] > 0:
                velocity_hex = hex(int(velocities[i]))[2:].upper().zfill(4)
            else:
                print("Error: velocity should be a positive number")
                return
            address_hex = hex(i * 5 + 14)[2:].upper().zfill(2)
            data = self.id_hex + '06 11' + address_hex + velocity_hex
            data = self.crc16(data)
            self.write_hex(data, 0.05)
            self.velocities[i] = velocities[i]
            self.velocities_updata = True
    
    def set_accelerationtimes_value(self, accelerationtimes):
        if len(accelerationtimes) > 16 or len(accelerationtimes) < 1:
            print("Error: accelerationtimes should be a list with 1 to 16 elements")
            return
        num = len(accelerationtimes)
        for i in range(num):
            if accelerationtimes[i] >= 0:
                accelerationtime_hex = hex(int(accelerationtimes[i]))[2:].upper().zfill(4)
            else:
                print("Error: accelerationtime should be a non-negative number")
                return
            address_hex = hex(i * 5 + 15)[2:].upper().zfill(2)
            data = self.id_hex + '06 11' + address_hex + accelerationtime_hex
            data = self.crc16(data)
            self.write_hex(data, 0.05)
            self.accelerationtimes[i] = accelerationtimes[i]
            self.accelerationtimes_updata = True
    
    def set_waittimes_value(self, waittimes):
        if len(waittimes) > 16 or len(waittimes) < 1:
            print("Error: waittimes should be a list with 1 to 16 elements")
            return
        num = len(waittimes)
        for i in range(num):
            if waittimes[i] >= 0:
                waittime_hex = hex(int(waittimes[i]))[2:].upper().zfill(4)
            else:
                print("Error: waittime should be a non-negative number")
                return
            address_hex = hex(i * 5 + 16)[2:].upper().zfill(2)
            data = self.id_hex + '06 11' + address_hex + waittime_hex
            data = self.crc16(data)
            self.write_hex(data, 0.05)
            self.waittimes[i] = waittimes[i]
            self.waittimes_updata = True
    
    # 速度模式下的速度设置
    def set_velocity_value(self, velocity, runtime):
        if velocity >= 0:
            velocity_hex = hex(int(velocity))[2:].upper().zfill(4)
        else:
            # 取补码
            velocity_hex = hex(int(velocity + 2**16) % 2**16)[2:].upper().zfill(4)
        data = self.id_hex + '06 06 03' + velocity_hex
        data = self.crc16(data)
        self.write_hex(data, 0.05)
        time.sleep(runtime)
    
    def set_power_value(self, power):
        if power >= 0:
            power_hex = hex(int(power*100))[2:].upper().zfill(4)
        else:
            print("Error: power should be a non-negative number")
            return
        data = self.id_hex + '06 08 0F' + power_hex
        print(data)
        data = self.crc16(data)
        
        self.write_hex(data, 0.05)
    
    #获取位置模式下的多段速度
    def get_velocities_value(self):
        for i in range(16):
            address_hex = hex(i * 5 + 14)[2:].upper().zfill(2)
            data = self.id_hex + '03 11' + address_hex + '00 01'
            data = self.crc16(data)
            _, ret = self.write_hex(data, 0.05)
            velocity_value = int(ret[6:10], 16)
            self.velocities[i] = velocity_value
        self.velocities_updata = True
        return self.velocities
    
    def get_accelerationtimes_value(self):
        for i in range(16):
            address_hex = hex(i * 5 + 15)[2:].upper().zfill(2)
            data = self.id_hex + '03 11' + address_hex + '00 01'
            data = self.crc16(data)
            _, ret = self.write_hex(data, 0.05)
            accelerationtime_value = int(ret[6:10], 16)
            self.accelerationtimes[i] = accelerationtime_value
        self.accelerationtimes_updata = True
        return self.accelerationtimes
    
    def get_waittimes_value(self):
        for i in range(16):
            address_hex = hex(i * 5 + 16)[2:].upper().zfill(2)
            data = self.id_hex + '03 11' + address_hex + '00 01'
            data = self.crc16(data)
            _, ret = self.write_hex(data, 0.05)
            waittime_value = int(ret[6:10], 16)
            self.waittimes[i] = waittime_value
        self.waittimes_updata = True
        return self.waittimes
    
    def get_current_position(self):
        _, ret = self.write_hex(self.crc16(self.id_hex + '03 0B 07 00 02'), 0.05)
        ret = ret[10:14] + ret[6:10]
        position = int(ret, 16)
        if position >= 2**30:
            # 取补码
            position = int(ret, 16) - 2**32
        position = float(position)
        # print("Current position: {}".format(position))
        position = position / 10000 * 0.08
        return position
    
    def motor_enable(self):
        success, _ = self.write_hex(self.crc16(self.id_hex + '06 31 00 00 01'))
        if success:
            print("Motor {} enabled successfully".format(self.id))
        else:
            print("Error: motor {} enable failed".format(self.id))

    def motor_disable(self):
        success, _ = self.write_hex(self.crc16(self.id_hex + '06 31 00 00 00'))
        if success:
            print("Motor {} disabled successfully".format(self.id))
        else:
            print("Error: motor {} disable failed".format(self.id))
    
    def motor_run_init(self):
        success, _ = self.write_hex(self.crc16(self.id_hex + '06 31 00 00 05'))
        # if success:
        #     print("Motor {} running in init mode successfully".format(self.id))
        # else:
        #     print("Error: motor {} run in init mode failed".format(self.id))

    def motor_run_position(self):
        success, _ = self.write_hex(self.crc16(self.id_hex + '06 31 00 00 03'))
        # if success:
        #     # print("Motor {} running in position mode successfully".format(self.id))
        # else:
        #     print("Error: motor {} run in position mode failed".format(self.id))
    
    def motor_stop_position(self):
        success, _ = self.write_hex(self.crc16(self.id_hex + '06 31 00 00 01'))
        # if success:
        #     # print("Motor {} stopped in position mode successfully".format(self.id))
        # else:
        #     print("Error: motor {} stop in position mode failed".format(self.id))
            
    def set_id(self, id):
        if id < 1 or id > 3:
            print("Error: id should be 1, 2 or 3")
            return
        self.id = id
        self.id_hex = hex(id)[2:].upper().zfill(2)
    
    def clean_error(self):
        self.write_hex(self.crc16(self.id_hex + '06 0D 01 00 00'))
        self.write_hex(self.crc16(self.id_hex + '06 0D 01 00 01')) 
    
    def crc16(self, data):
        poly = 0xA001
        init_value = 0xFFFF
        data_hex = bytes.fromhex(data)
        crc = init_value
        for byte in data_hex:
            crc ^= byte  # 按字节异或
            for _ in range(8):  # 每字节处理 8 位
                if crc & 0x0001:
                    crc = (crc >> 1) ^ poly
                else:
                    crc >>= 1
        crc = hex(crc)[2:].upper().zfill(4)
        res = data + crc[2:4] + crc[0:2]
        return res

def motors_enable(x, y, z):
    x.motor_enable()
    y.motor_enable()
    z.motor_enable()

def motors_disable(x, y, z):
    x.motor_disable()
    y.motor_disable()
    z.motor_disable()

def motors_init_mode(x, y, z):
    x.set_init_mode()
    y.set_init_mode()
    z.set_init_mode()

def motors_position_mode(x, y, z):
    x.set_position_mode()
    y.set_position_mode()
    z.set_position_mode()

def motors_velocity_mode(x, y, z):
    x.set_velocity_mode()
    y.set_velocity_mode()
    z.set_velocity_mode()

def motors_init(x, y, z, wait_time=5):
    x.motor_run_init()
    y.motor_run_init()
    z.motor_run_init()
    time.sleep(wait_time)
    
def motors_move(x, y, z, position):
    wait_x = x.set_positions_value([position[0]])
    wait_y = y.set_positions_value([position[1]])
    wait_z = z.set_positions_value([position[2]])
    wait = max(wait_x, wait_y, wait_z)
    
    x.motor_run_position()
    y.motor_run_position()
    z.motor_run_position()
    # print("Motor move to position {} successfully, wait time: {}s".format(position, wait))
    
    time.sleep(wait)
    motors_stop(x, y, z)

def motors_stop(x, y, z):
    x.motor_stop_position()
    y.motor_stop_position()
    z.motor_stop_position()

def motors_position(x, y, z):
    position = [x.get_current_position(), y.get_current_position(), z.get_current_position()]
    return position

# if __name__ == '__main__':
#     motor_x = SerialConnection(id=1)
#     motor_x.connect()
#     motor_y = SerialConnection(id=2)
#     motor_y.connect()
#     motor_z = SerialConnection(id=3)
#     motor_z.connect()
#     # motor_z.clean_error()
#     # motor_z.write_hex(motor_z.crc16(motor_z.id_hex + '06 04 0A 00 01'))
    
#     motors_init_mode(motor_x, motor_y, motor_z)
#     motors_enable(motor_x, motor_y, motor_z)
#     motors_init(motor_x, motor_y, motor_z, wait_time=5)
#     print(motors_position(motor_x, motor_y, motor_z))
    
#     count = 0
#     position = [0.1, 0.1, 0.1]
#     while True:
#         user_input = input("motors are ready, input q to quit, or input 1 to conutinue: ")
#         if user_input == '1':
            
#             motors_move(motor_x, motor_y, motor_z, position)
#             motors_move(motor_x, motor_y, motor_z, [0.3, 0.2, 0.15])
#             motors_move(motor_x, motor_y, motor_z, [0.1, 0.1, 0.1])
            
#         elif user_input == 'q':
#             break
    
#     motors_disable(motor_x, motor_y, motor_z)
    
#     motor_x.close()
#     motor_y.close()
#     motor_z.close()
    
