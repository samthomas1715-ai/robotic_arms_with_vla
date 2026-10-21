from dynamixel_sdk import *
import config as con

class Dynamixelline:
    def __init__(self, port_, baudrate):
        self.port = PortHandler(port_)
        self.port.openPort()
        self.port.setBaudRate(baudrate)
        self.packethandler = PacketHandler(con.PROTOCOL_VERSION)
    def closeport(self):
        self.port.closePort()


class DynamixelController:

    def __init__(self,  motor_ids, line):

        self.porthandler = line.port
        self.packethandler = line.packethandler
        self.motor_ids = motor_ids

       
    # def connect(self):
    #     if self.porthandler.openPort():
    #         print(f"Connected to {self.port}")
    #     else:
    #         raise Exception(f"Failed to open {self.port}")

    #     if self.porthandler.setBaudRate(con.BAUDRATE):
    #         print(f"Baudrate set to {con.BAUDRATE}")
    #     else:
    #         raise Exception("Failed to set baudrate")


    def disconnect(self):
        self.porthandler.closePort()
        print(f"Disconnected from {self.port}")


#if no arguement passed for id then 
#torque for whole daisy chain will be enabled
    def tq_enb(self,dx_id=None):
        if dx_id == None:
            for id in self.motor_ids:
                self.packethandler.write1ByteTxRx(
                self.porthandler,
                id,
                con.ADDR_TORQUE,
                con.TORQUE_ENABLE)
        else:
            self.packethandler.write1ByteTxRx(
                self.porthandler,
                dx_id,
                con.ADDR_TORQUE,
                con.TORQUE_ENABLE)
            
    def tq_disb(self,dx_id=None):
        if dx_id == None:
            for id in self.motor_ids:
                self.packethandler.write1ByteTxRx(
                self.porthandler,
                id,
                con.ADDR_TORQUE,
                con.TORQUE_DISABLE)
        else:
            self.packethandler.write1ByteTxRx(
                self.porthandler,
                dx_id,
                con.ADDR_TORQUE,
                con.TORQUE_DISABLE)   
    
    
    def result_all(self):
        for id in self.motor_ids:
            pos,res,err = self.packethandler.read2ByteTxRx(self.porthandler,id)
        if res != COMM_SUCCESS:
           print(self.packethandler.getTxRxResult(res))
           return None
        if err != 0:
           print(self.packethandler.getRxPacketError(err))
           return None
        return pos
    
    def get_pos(self, dx_id=None):
        if dx_id == None:
            positions = {}
            for id in self.motor_ids:
                
                pose, res, err = self.packethandler.read2ByteTxRx(
                    self.porthandler,
                    id,
                    con.ADDR_PRESENT_POSITION,
                )
                if res != COMM_SUCCESS or err != 0:
                    return None
                positions[id] = pose
            return positions
        else:
             position, res, err = self.packethandler.read2ByteTxRx(
            self.porthandler,
            dx_id,
            con.ADDR_PRESENT_POSITION
        )
        return position
    
    def move_to_pos(self, pos, dx_id):
        pos = max(0, min(1023, int(pos))

        res, err = self.packethandler.write2ByteTxRx(
            self.porthandler,
            dx_id,
            con.ADDR_GOAL_POSITION,
            pos
        )
    
    def move_all_to_pos(self,positions):
        for id,pose in positions.items():
            res, err = self.packethandler.write2ByteTxRx(
            self.porthandler,
            id,
            con.ADDR_GOAL_POSITION,
            pose
        )

    def set_speed(self, speed, dx_id=None):

        if dx_id is None:
            for motor_id in self.motor_ids:
                self.packethandler.write2ByteTxRx(
                    self.porthandler,
                    motor_id,
                    con.ADDR_MOVING_SPEED,
                    speed
                )    
        else:
            self.packethandler.write2ByteTxRx(
                self.porthandler,
                dx_id,
                con.ADDR_MOVING_SPEED,
                speed
            )
            
    def get_temp(self,dx_id):
        temp, _, _ = self.packethandler.read1ByteTxRx(self.porthandler,dx_id,con.ADDR_PRESENT_TEMP)
        return temp

    def degrees_to_position(self, deg):
        deg = max(0, min(300, deg))
        return int(deg*1023/300)
