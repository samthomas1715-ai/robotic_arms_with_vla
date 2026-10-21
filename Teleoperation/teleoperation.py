from dynamixel_functions import  Dynamixelline,DynamixelController
import config as con
import time
import calib as cb
from dynamixel_sdk import GroupSyncWrite, DXL_LOBYTE, DXL_HIBYTE

def teleoperation():

    leader_line = Dynamixelline(con.LEADER_PORT, con.BAUDRATE)
    follower_line = Dynamixelline(con.FOLLOWER_PORT, con.BAUDRATE)

    leader = DynamixelController(con.LEADER_IDS, leader_line)
    follower = DynamixelController(con.FOLLOWER_IDS, follower_line)

    leader.tq_disb()
    follower.tq_enb()
    sync_write = GroupSyncWrite(follower_line.port, follower_line.packethandler, con.ADDR_GOAL_POSITION, 2)

    try:
        while True:
            for leader_id, follower_id in zip(con.LEADER_IDS, con.FOLLOWER_IDS):
                position = leader.get_pos(leader_id)
                fposi = follower.get_pos(follower_id)
                if position is None or fposi is None:
                    continue
                if abs(position - fposi) > 900:
                    break

                param_goal_position = [DXL_LOBYTE(position), DXL_HIBYTE(position)]
                sync_write.addParam(follower_id, param_goal_position)
            if abs(position - fposi) > 900:
                print("Follower motor is too far from leader. Stopping teleoperation.")
                break
            sync_write.txPacket()
            sync_write.clearParam()
            

    except KeyboardInterrupt:
        print("Teleoperation stopped.")

    finally:
        follower.tq_disb()
        leader_line.closeport()
        follower_line.closeport()

teleoperation()
