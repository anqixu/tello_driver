"""License.

Copyright 2018 PingguSoft <pinggusoft@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import datetime
import math
import pprint
import socket
import threading
import time
import traceback

from bytebuffer import ByteBuffer

class Tello(object):
# TELLO COMMAND
    TELLO_CMD_CONN                      = 1         
    TELLO_CMD_CONN_ACK                  = 2
    TELLO_CMD_SSID                      = 17     # pt48
    TELLO_CMD_SET_SSID                  = 18     # pt68
    TELLO_CMD_SSID_PASS                 = 19     # pt48
    TELLO_CMD_SET_SSID_PASS             = 20     # pt68
    TELLO_CMD_REGION                    = 21     # pt48
    TELLO_CMD_SET_REGION                = 22     # pt68
    TELLO_CMD_REQ_VIDEO_SPS_PPS         = 37     # pt60
    TELLO_CMD_TAKE_PICTURE              = 48     # pt68
    TELLO_CMD_SWITCH_PICTURE_VIDEO      = 49     # pt68
    TELLO_CMD_START_RECORDING           = 50     # pt68
    TELLO_CMD_SET_EV                    = 52     # pt48
    TELLO_CMD_DATE_TIME                 = 70     # pt50
    TELLO_CMD_STICK                     = 80     # pt60
    TELLO_CMD_LOG_HEADER_WRITE          = 4176   # pt50
    TELLO_CMD_LOG_DATA_WRITE            = 4177   # RX_O
    TELLO_CMD_LOG_CONFIGURATION         = 4178   # pt50
    TELLO_CMD_WIFI_SIGNAL               = 26     # RX_O
    TELLO_CMD_VIDEO_BIT_RATE            = 40     # pt48
    TELLO_CMD_LIGHT_STRENGTH            = 53     # RX_O
    TELLO_CMD_VERSION_STRING            = 69     # pt48
    TELLO_CMD_ACTIVATION_TIME           = 71     # pt48
    TELLO_CMD_LOADER_VERSION            = 73     # pt48
    TELLO_CMD_STATUS                    = 86     # RX_O pt88
    TELLO_CMD_ALT_LIMIT                 = 4182   # pt48
    TELLO_CMD_LOW_BATT_THRESHOLD        = 4183   # pt48
    TELLO_CMD_ATT_ANGLE                 = 4185   # pt48
    TELLO_CMD_SET_JPEG_QUALITY          = 55     # pt68
    TELLO_CMD_TAKEOFF                   = 84     # pt68
    TELLO_CMD_LANDING                   = 85     # pt68
    TELLO_CMD_SET_ALT_LIMIT             = 88     # pt68
    TELLO_CMD_FLIP                      = 92     # pt70
    TELLO_CMD_THROW_FLY                 = 93     # pt48
    TELLO_CMD_PALM_LANDING              = 94     # pt48
    TELLO_CMD_PLANE_CALIBRATION         = 4180   # pt68
    TELLO_CMD_SET_LOW_BATTERY_THRESHOLD = 4181   # pt68
    TELLO_CMD_SET_ATTITUDE_ANGLE        = 4184   # pt68
    TELLO_CMD_ERROR1                    = 67     # RX_O
    TELLO_CMD_ERROR2                    = 68     # RX_O
    TELLO_CMD_FILE_SIZE                 = 98     # pt50
    TELLO_CMD_FILE_DATA                 = 99     # pt50
    TELLO_CMD_FILE_COMPLETE             = 100    # pt48
    TELLO_CMD_HANDLE_IMU_ANGLE          = 90     # pt48
    TELLO_CMD_SET_VIDEO_BIT_RATE        = 32     # pt68
    TELLO_CMD_SET_DYN_ADJ_RATE          = 33     # pt68
    TELLO_CMD_SET_EIS                   = 36     # pt68
    TELLO_CMD_SMART_VIDEO_START         = 128    # pt68
    TELLO_CMD_SMART_VIDEO_STATUS        = 129    # pt50
    TELLO_CMD_BOUNCE                    = 4179   # pt68
    
# Smart Video     
    TELLO_SMART_VIDEO_STOP              = 0x00
    TELLO_SMART_VIDEO_START             = 0x01
    TELLO_SMART_VIDEO_360               = 0x01
    TELLO_SMART_VIDEO_CIRCLE            = 0x02
    TELLO_SMART_VIDEO_UP_OUT            = 0x03

# PORT
    TELLO_PORT_VIDEO                    = 6037

# CRC TABLES
    TBL_CRC16 = [
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
    ]

    TBL_CRC8 = [
        0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
        0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
        0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
        0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
        0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
        0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
        0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
        0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
        0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
        0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
        0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
        0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
        0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
        0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
        0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
        0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
    ]

    FLIP_FRONT = 0
    FLIP_LEFT = 1
    FLIP_BACK = 2
    FLIP_RIGHT = 3
    FLIP_FRONT_LEFT = 4
    FLIP_BACK_LEFT = 5
    FLIP_BACK_RIGHT = 6
    FLIP_FRONT_RIGHT = 7
    
    NEW_ALT_LIMIT_M = 30

    RC_VAL_MIN = 364
    RC_VAL_MAX = 1684
    RC_RANGE = (RC_VAL_MAX-RC_VAL_MIN)

    @staticmethod
    def ratio2rcval(r):
        v = int((r+1.0)/2*Tello.RC_RANGE + Tello.RC_VAL_MIN)
        if v < Tello.RC_VAL_MIN: v = Tello.RC_VAL_MIN
        elif v > Tello.RC_VAL_MAX: v = Tello.RC_VAL_MAX
        return v

    def __init__(self, tello_ip = '192.168.10.1', portCmd = 8889):
        self.pill2kill = threading.Event()
        self.threadCmdRX = None
        self.threadVideoRX = None
        self.connected = False

        self.sockCmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addrCmd = (tello_ip, portCmd)
        self.sockCmd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sockCmd.settimeout(.5)
        self.threadCmdRX = threading.Thread(target=self._threadCmdRX, args=(self.pill2kill, "task"))
        self.threadCmdRX.start()

        #self.threadVideoRX = threading.Thread(target=self._threadVideoRX, args=(self.pill2kill, "task")) # TODO: deal with video
        #self.threadVideoRX.start()
        self.seqID = 0
        self.stickData = 0
        self.rcCtr = 0
        self.connected = False
        self._sendCmd(0x00, self.TELLO_CMD_CONN, None)

    def __del__(self):
        self.stop()

    def process_sensor_update(self, sensor_telem_dict):
        pass

    def stop(self):
        self.pill2kill.set()
        if self.threadCmdRX is not None:
            self.threadCmdRX.join()
        if self.threadVideoRX is not None:
            self.threadVideoRX.join()
        self.sockCmd.close()

    def setCmdVel(self, roll_ratio, pitch_ratio, yaw_ratio, vspeed_ratio, fast_mode=False):
        fast_bit = int(fast_mode)
        roll_val = Tello.ratio2rcval(-roll_ratio)
        pitch_val = Tello.ratio2rcval(pitch_ratio)
        yaw_val = Tello.ratio2rcval(-yaw_ratio)
        vspeed_val = Tello.ratio2rcval(vspeed_ratio)
        self.stickData = (fast_bit << 44) | (yaw_val << 33) | (vspeed_val << 22) | (pitch_val << 11) | (roll_val)
        success = self._sendCmd(0x60, self.TELLO_CMD_STICK, None)
        self.rcCtr = self.rcCtr + 1
        return success

    def takeOff(self):
        return self._sendCmd(0x68, self.TELLO_CMD_TAKEOFF, None)
        
    def throwTakeOff(self):
        return self._sendCmd(0x68, self.TELLO_CMD_THROW_FLY, None)
        
    def land(self, stop_land=False):
        return self._sendCmd(0x68, self.TELLO_CMD_LANDING, bytearray((int(stop_land),)))

    def palmLand(self):
        return self._sendCmd(0x68, self.TELLO_CMD_PALM_LANDING, bytearray('\x00'))
    
    def flattrim(self):
        return self._sendCmd(0x68, self.TELLO_CMD_PLANE_CALIBRATION, None)

    def flip(self, flip_dir):
        if flip_dir < 0 or flip_dir > 7: # See FLIP_XYZ macros
            return False
        arg = bytearray((int(flip_dir),))
        print('flip arg: %s' % str(arg)) # TODO: remove after debug
        return self._sendCmd(0x68, self.TELLO_CMD_FLIP, arg)

    def setSmartVideoShot(self, mode, isStart):
        data = self.TELLO_SMART_VIDEO_START if isStart == True else self.TELLO_SMART_VIDEO_STOP
        data = data | (mode << 2)
        return self._sendCmd(0x68, self.TELLO_CMD_SMART_VIDEO_START, bytearray((data,)))
        
    def bounce(self, isStart): # Fly up and down slowly
        data = 0x30 if isStart == True else 0x31
        return self._sendCmd(0x68, self.TELLO_CMD_BOUNCE, bytearray([data]));

###############################################################################
# utility functions
###############################################################################
    def _calcCRC16(self, buf, size):
        i = 0
        seed = 0x3692
        while size > 0:
            seed = self.TBL_CRC16[(seed ^ buf[i]) & 0xff] ^ (seed >> 8)
            i = i + 1
            size = size - 1

        return seed

    def _calcCRC8(self, buf, size):
        i = 0
        seed = 0x77
        while size > 0:
            seed = self.TBL_CRC8[(seed ^ buf[i]) & 0xff]
            i = i + 1
            size = size - 1

        return seed

    def _buildPacket(self, pacType, cmdID, seqID, data):
        size = len(data) if data != None else 0
        size = 11 + size

        bb = ByteBuffer.allocate(size)
        bb.clear()
        bb.put_ULInt8(0xCC)
        bb.put_ULInt16(size << 3)
        crc8 = self._calcCRC8(bb.get_array(), 3)
        bb.put_ULInt8(crc8)
        bb.put_ULInt8(pacType)
        bb.put_ULInt16(cmdID)
        bb.put_ULInt16(seqID)
        if data:
            bb.put(data)
        crc16 = self._calcCRC16(bb.get_array(), size - 2);
        bb.put_ULInt16(crc16)
        bb.flip()
        return bb

    def _parsePacket(self, buf):
        cmdID    = None

        if len(buf) >= 11:
            bb = ByteBuffer.wrap(buf)
            mark = bb.get_ULInt8()
            if mark == 0xCC: # Binary stream for CMD_...
                size = bb.get_ULInt16() >> 3
                if size > len(buf):
                    print('packet/read size mismatch: %d / %d' % (size, len(buf)))
                    return None

                crc8 = bb.get_ULInt8()
                calcCRC8 = self._calcCRC8(buf, 3)
                if crc8 != calcCRC8:
                    print('wrong CRC8: %02x / %02x' % (crc8, calcCRC8))
                    return None

                pacType  = bb.get_ULInt8()
                cmdID    = bb.get_ULInt16()
                seqID    = bb.get_ULInt16()

                if cmdID == Tello.TELLO_CMD_STATUS:
                    # References:
                    # https://github.com/Kragrathea/TelloLib/blob/master/TelloLib/Tello.cs#L993
                    # https://github.com/hybridgroup/gobot/blob/master/platforms/dji/tello/driver.go#L563

                    status_telem = dict()
                    status_telem['Height'] = bb.get_SLInt16()
                    status_telem['NorthSpeed'] = bb.get_SLInt16()
                    status_telem['EastSpeed'] = bb.get_SLInt16()
                    status_telem['HorizontalSpeed'] = math.sqrt(status_telem['NorthSpeed']*status_telem['NorthSpeed'] + status_telem['EastSpeed']*status_telem['EastSpeed'])
                    status_telem['VerticalSpeed'] = bb.get_SLInt16()
                    status_telem['FlyTime'] = bb.get_ULInt16()

                    flags = bb.get_ULInt8()
                    status_telem['IMUState'] = (((flags >> 0) & 0x1) == 1)
                    status_telem['PressureState'] = (((flags >> 1) & 0x1) == 1)
                    status_telem['DownVisualState'] = (((flags >> 2) & 0x1) == 1)
                    status_telem['PowerState'] = (((flags >> 3) & 0x1) == 1)
                    status_telem['BatteryState'] = (((flags >> 4) & 0x1) == 1)
                    status_telem['GravityState'] = (((flags >> 5) & 0x1) == 1)
                    status_telem['WindState'] = (((flags >> 7) & 0x1) == 1)

                    status_telem['IMUCalibrationState'] = bb.get_ULInt8()
                    status_telem['BatteryPercentage'] = bb.get_ULInt8()
                    status_telem['DroneFlyTimeLeft'] = bb.get_ULInt16()
                    status_telem['DroneBatteryLeft'] = bb.get_ULInt16()

                    flags = bb.get_ULInt8()
                    status_telem['IsFlying'] = (((flags >> 0) & 0x1) == 1)
                    status_telem['IsOnGround'] = (((flags >> 1) & 0x1) == 1)
                    status_telem['IsEmOpen'] = (((flags >> 2) & 0x1) == 1)
                    status_telem['IsDroneHover'] = (((flags >> 3) & 0x1) == 1)
                    status_telem['IsOutageRecording'] = (((flags >> 4) & 0x1) == 1)
                    status_telem['IsBatteryLow'] = (((flags >> 5) & 0x1) == 1)
                    status_telem['IsBatteryLower'] = (((flags >> 6) & 0x1) == 1)
                    status_telem['IsFactoryMode'] = (((flags >> 7) & 0x1) == 1)

                    status_telem['FlyMode'] = bb.get_ULInt8()
                    status_telem['ThrowFlyTimer'] = bb.get_ULInt8()
                    status_telem['CameraState'] = bb.get_ULInt8()

                    status_telem['ElectricalMachineryState'] = bb.get_ULInt8()

                    flags = bb.get_ULInt8()
                    status_telem['FrontIn'] = (((flags >> 0) & 0x1) == 1)
                    status_telem['FrontOut'] = (((flags >> 1) & 0x1) == 1)
                    status_telem['FrontLSC'] = (((flags >> 2) & 0x1) == 1)

                    status_telem['TemperatureHeight'] = bb.get_ULInt8()

                    self.process_sensor_update(status_telem)

                    """
                    elif cmdID == Tello.TELLO_CMD_LOG_HEADER_WRITE:
                        # TODO: send ack

                    elif cmdID == Tello.TELLO_CMD_LOG_DATA_WRITE:
                        # TODO: parse vel, IMU quat, ...
                    """
                else:
                    pass


                bb.set_position(size - 2)
                crc16    = bb.get_ULInt16()
                calcCRC16=self._calcCRC16(buf, size - 2);
                if crc16 != calcCRC16:
                    print('wrong CRC16 {%04x / %04x}' % (crc16, calcCRC16))
                    return None
            elif mark == 0x63: # Connection acknowledgement
                ack = ByteBuffer.allocate(11)
                ack.put_bytes('conn_ack:'.encode())
                ack.put_ULInt16(self.TELLO_PORT_VIDEO)
                ack.flip()
                if ack.get_array() == buf:
                    cmdID = self.TELLO_CMD_CONN_ACK
                else:
                    print('wrong video port !!')
            else:
                print('unrecognized mark !! %02x' % mark)
        elif buf != None:
            print('wrong packet length=%d, 1st byte=%02x' % (len(buf), buf[0]))

        return cmdID

    def _sendCmd(self, pacType, cmdID, data):
        bb      = None
        payload = None
        out     = None
        seq = 0
        len = 0

        if cmdID == self.TELLO_CMD_CONN:
            out = ByteBuffer.allocate(11)
            out.clear()
            out.put_bytes('conn_req:'.encode())
            out.put_ULInt16(self.TELLO_PORT_VIDEO);
            self.seqID = self.seqID + 1
        elif cmdID == self.TELLO_CMD_STICK:
            now = datetime.datetime.now()
            bb = ByteBuffer.allocate(11)
            bb.clear()
            
            # put 64bit stick data
            pp = ByteBuffer.allocate(8)
            pp.put_ULInt64(self.stickData) # TODO: can probably do this better as more directly
            pp.flip()
            
            # get 48bit data only
            bb.put(pp.get_array(), 0, 6);
            bb.put_ULInt8(now.hour)
            bb.put_ULInt8(now.minute)
            bb.put_ULInt8(now.second)
            bb.put_ULInt16(now.microsecond & 0xffff)
            seq = 0
        elif cmdID == self.TELLO_CMD_DATE_TIME:
            seq = self.seqID
            now = datetime.datetime.now()
            bb = ByteBuffer.allocate(15)
            bb.clear()
            bb.put_ULInt8(0x00)
            bb.put_ULInt16(now.year)
            bb.put_ULInt16(now.month)
            bb.put_ULInt16(now.day)
            bb.put_ULInt16(now.hour)
            bb.put_ULInt16(now.minute)
            bb.put_ULInt16(now.second)
            bb.put_ULInt16(now.microsecond & 0xffff)
            self.seqID = self.seqID + 1
        elif cmdID == self.TELLO_CMD_REQ_VIDEO_SPS_PPS:
            seq = 0
        else:
            seq = self.seqID
            self.seqID = self.seqID + 1

        if bb != None:
            payload = bb.get_array()
        else:
            payload = data

        if out == None:
            out = self._buildPacket(pacType, cmdID, seq, payload)

        try:
            self.sockCmd.sendto(out.get_array(), self.addrCmd)
        except socket.timeout as e:
            return False
        return True


###############################################################################
# CommandRX Thread
###############################################################################
    def _threadCmdRX(self, stop_event, arg):
        #print('_threadCmdRX started !!!')
        statusCtr = 0
        data = bytearray(1024)
        payload = None
        
        while not stop_event.is_set():
            try:
                size, addr = self.sockCmd.recvfrom_into(data)
            except socket.timeout as e:
                time.sleep(.5)
                continue
            except socket.error as e:
                print(e)
                continue
            else:
                cmdID   = self._parsePacket(data[:size])
                if cmdID is None:
                    continue
                payload = ByteBuffer.wrap(data[9:size-1])
               
                if cmdID == self.TELLO_CMD_CONN_ACK:
                    print('connection successful !')
                    self.connected = True
                
                elif cmdID == self.TELLO_CMD_DATE_TIME:
                    self._sendCmd(0x50, cmdID, None)
                
                elif cmdID == self.TELLO_CMD_STATUS:
                    if statusCtr == 3:
                        self._sendCmd(0x60, self.TELLO_CMD_REQ_VIDEO_SPS_PPS, None)
                        self._sendCmd(0x48, self.TELLO_CMD_VERSION_STRING, None)
                        self._sendCmd(0x48, self.TELLO_CMD_SET_VIDEO_BIT_RATE, None)
                        self._sendCmd(0x48, self.TELLO_CMD_ALT_LIMIT, None)
                        self._sendCmd(0x48, self.TELLO_CMD_LOW_BATT_THRESHOLD, None)
                        self._sendCmd(0x48, self.TELLO_CMD_ATT_ANGLE, None)
                        self._sendCmd(0x48, self.TELLO_CMD_REGION, None)
                        self._sendCmd(0x48, self.TELLO_CMD_SET_EV, bytearray([0x00]));
                    statusCtr = statusCtr + 1
                
                elif cmdID == self.TELLO_CMD_VERSION_STRING:
                    if size >= 42:
                        print('Version:' + data[10:30].decode())
                
                elif cmdID == self.TELLO_CMD_SMART_VIDEO_START:
                    if payload.get_remaining() > 0:
                        print('smart video start')
                        
                elif cmdID == self.TELLO_CMD_ALT_LIMIT:
                    if payload.get_remaining() > 0:
                        payload.get_ULInt8()                    # 0x00
                        height = payload.get_ULInt16()
                        print('alt limit : %2d meter' % height)
                        
                        if height != self.NEW_ALT_LIMIT_M:
                            print('set new alt limit : %2d meter' % self.NEW_ALT_LIMIT_M)
                            self._sendCmd(0x68, self.TELLO_CMD_SET_ALT_LIMIT, bytearray([self.NEW_ALT_LIMIT_M & 0xff, (self.NEW_ALT_LIMIT_M >> 8) & 0xff]))
               
                elif cmdID == self.TELLO_CMD_SMART_VIDEO_STATUS:
                    if payload.get_remaining() > 0:
                        resp = payload.get_ULInt8()
                        dummy = resp & 0x07
                        start = (resp >> 3) & 0x03
                        mode  = (resp >> 5) & 0x07
                        print('smart video status - mode: %d, start: %d' % (mode, start))
                        self._sendCmd(0x50, self.TELLO_CMD_SMART_VIDEO_STATUS, bytearray([0x00]))
                #else:
                    # print(','.join(hex(ord(i)) for i in data))
        #print('_threadCmdRX terminated !!!')


###############################################################################
# VideoRX Thread
###############################################################################
    def _threadVideoRX(self, stop_event, arg):
        #print('_threadVideoRX started !!!')

        sockVideo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        addrVideo = ('192.168.10.2', self.TELLO_PORT_VIDEO)
        sockVideo.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sockVideo.settimeout(.5)
        sockVideo.bind(addrVideo)
        
        data = bytearray(4096)
        fileVideo = open('video.h264', 'wb')
        isSPSRcvd = False
        
        while not stop_event.is_set():
            try:
                size, addr = sockVideo.recvfrom_into(data)
            except socket.timeout as e:
                time.sleep(.5)
                continue
            except socket.error as e:
                print(e)
                break
            else:
                if size > 6 and data[2] == 0x00 and data[3] == 0x00 and data[4] == 0x00 and data[5] == 0x01:
                    nal_type = data[6] & 0x1f
                    #print('NAL=', nal_type)
                    if nal_type == 7:
                        isSPSRcvd = True

                # drop 2 bytes
                if isSPSRcvd:
                    fileVideo.write(data[2:size]) # TODO: try decoding this

        sockVideo.close()
        fileVideo.close()
        #print('_threadVideoRX terminated !!!')


###############################################################################
# timerTask
###############################################################################
    def _timerTask(self, arg): # TODO: no longer needed
        self._sendCmd(0x60, self.TELLO_CMD_STICK, None)
        self.rcCtr = self.rcCtr + 1

        #every 1sec
        if self.rcCtr % 50 == 0:
            self._sendCmd(0x60, self.TELLO_CMD_REQ_VIDEO_SPS_PPS, None)
