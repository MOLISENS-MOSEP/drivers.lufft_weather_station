#!/usr/bin/python3
# https://github.com/Tasm-Devil/lufft-python

import time
import struct

class UMBError(BaseException):
    pass

class WS_UMB:
    """
    This is a simple driver for communicating to Weatherstations
    made by the German company Lufft. It implements their UMB-Protocol.
    You just need a USB-to-RS485 dongle and connect it to your PWS 
    according to the wiring diagram you find in the manual.
    Downsides: This class does not replace the UMB-config-tool, because
    its not able to set the config values in your PWS at the moment.
    
    Attributes
    ----------
    device : string
        Serial port. Default is 'COM3'
    
    baudrate : integer
        The default baud rate is 19200
        
    Methods
    -------
    onlineDataQuery(channel, receiver_id=1):
        Use this method to request a value from one channel.
        It will return a (value, status) tuple.
        Status number 0 means everything is ok.
        If you have more than one PWS on the BUS, use receiver_id to
        distinguish between them.

    onlineDataQueryMulti(channels, receiver_id=1):
        Use this method to request  values from multiple channels.
        It will return two lists: values, statuses
        Status number 0 means everything is ok.

     onlineDataQueryMultiOneCall(channels, receiver_id=1):
        Use this method to request values from multiple channels in a one call.
        It will return two lists: values, statuses
        Status number 0 means everything is ok.

    checkStatus(status):
        Call this to check what status number means.
    
    Usage
    -----
    1. In your python-script example 1: 
        from WS_UMB import WS_UMB
        
        with WS_UMB() as umb:
            value, status = umb.onlineDataQuery(SomeChannelNumber)
            if status != 0:
                print(umb.checkStatus(status))
            else:
                print(value)

     2. In your python-script example 2: 
        from WS_UMB import WS_UMB
        
        with WS_UMB() as umb:
            channels = [113, 700]
            value, status = umb.onlineDataQueryMulti(channels)
            if status != 0:
                print(umb.checkStatus(status))
            else:
                print(value)

     3. In your python-script example 3: 
        from WS_UMB import WS_UMB
        
        with WS_UMB() as umb:
            channels = [113, 700]
            value, status = umb.onlineDataQueryMultiOneCall(channels)
            if status != 0:
                print(umb.checkStatus(status))
            else:
                print(value)
    
    4. As a standalone program:
        ./WS_UMB.py 100 111 200 300 460 580
    """

    def __init__(self, device='COM3', baudrate=19200):
        self.device = device
        self.baudrate = baudrate
    
    def __enter__(self): # throws a SerialException if it cannot connect to device
        import serial
        self.serial = serial.Serial(self.device, baudrate = self.baudrate, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS, interCharTimeout=1)
        return self
    
    def __exit__(self, exception_type, exception_value, traceback):
        self.serial.close()
    
    def readFromSerial(self, timeout=1):
        timeout_count = 0
        data = b''
        while True:
            if self.serial.inWaiting() > 0:
                new_data = self.serial.read(1)
                data = data + new_data
                timeout_count = 0
            else:
                timeout_count += 1
                if timeout is not None and timeout_count >= 10 * timeout:
                    break
                time.sleep(0.01)
        return data

    def calc_next_crc_byte(self, crc_buff, nextbyte):
        for i in range (8):
            if( (crc_buff & 0x0001) ^ (nextbyte & 0x01) ):
                x16 = 0x8408;
            else:
                x16 = 0x0000;
            crc_buff = crc_buff >> 1;
            crc_buff ^= x16;
            nextbyte = nextbyte >> 1;
        return(crc_buff);
    
    def calc_crc16(self, data):
        crc = 0xFFFF;
        for byte in data:
            crc = self.calc_next_crc_byte(crc, byte);
        return crc

    def send_request_one_call_multi(self, receiver_id, command, command_version, channels):
        
        SOH, STX, ETX, EOT= b'\x01', b'\x02', b'\x03', b'\x04'
        VERSION = b'\x10'

        TO = int(receiver_id).to_bytes(1,'little')
        TO_CLASS = b'\x70'
        FROM = int(1).to_bytes(1,'little')
        FROM_CLASS = b'\xF0'
        
        LEN = 3
        channel_len = len(channels)
        NUMBER = int(channel_len).to_bytes(1,'little')

        FULLPAYLOAD = NUMBER
        for channel in channels:
            as_byte = int(channel).to_bytes(2,'little')
            FULLPAYLOAD += as_byte
            for byte in as_byte:
                LEN += 1

        LEN = int(LEN).to_bytes(1,'little')
        
        COMMAND = int(command).to_bytes(1,'little')
        COMMAND_VERSION = int(command_version).to_bytes(1,'little')

        # Assemble transmit-frame
        tx_frame = SOH + VERSION + TO + TO_CLASS + FROM + FROM_CLASS + LEN + STX + COMMAND + COMMAND_VERSION + FULLPAYLOAD + ETX

        # calculate checksum for trasmit-frame and concatenate
        tx_frame += self.calc_crc16(tx_frame).to_bytes(2, 'little') + EOT

        # Write transmit-frame to serial
        self.serial.write(tx_frame)
        #print([hex(c) for c in tx_frame])
        
        ### < --- --- > ###
        
        # Read frame from serial
        rx_frame = self.readFromSerial()
        #print("one call response: " + str(rx_frame))
        #print([hex(c) for c in rx_frame])
        
        # compare checksum field to calculated checksum
        cs_calculated = self.calc_crc16(rx_frame[:-3]).to_bytes(2, 'little')
        cs_received = rx_frame[-3:-1]
        if (cs_calculated != cs_received):
            raise UMBError("RX-Error! Checksum test failed. Calculated Checksum: " + str(cs_calculated) + "| Received Checksum: " + str(cs_received))
        
        # Check the length of the frame
        length = int.from_bytes(rx_frame[6:7], byteorder='little')
        if (rx_frame[8+length:9+length] != ETX):
            raise UMBError("RX-Error! Length of Payload is not valid. length-field says: " + str(length))
        
        # Check if all frame field are valid
        if (rx_frame[0:1] != SOH):
            raise UMBError("RX-Error! No Start-of-frame Character")
        if (rx_frame[1:2] != VERSION):
            raise UMBError("RX-Error! Wrong Version Number")
        if (rx_frame[2:4] != (FROM + FROM_CLASS)):
            raise UMBError("RX-Error! Wrong Destination ID")
        if (rx_frame[4:6] != (TO + TO_CLASS)):
            raise UMBError("RX-Error! Wrong Source ID")
        if (rx_frame[7:8] != STX):
            raise UMBError("RX-Error! Missing STX field")
        if (rx_frame[8:9] != COMMAND):
            raise UMBError("RX-Error! Wrong Command Number")
        if (rx_frame[9:10] != COMMAND_VERSION):
            raise UMBError("RX-Error! Wrong Command Version Number")
            
        values = []
        statuses = []
        index = 16
        parse_index = 17

        for i in range(len(channels)):
            #status = int.from_bytes(rx_frame[10:11], byteorder='little')
            status = int.from_bytes(rx_frame[index - 3: (index + 1) - 3], byteorder='little')
            type_of_value = int.from_bytes(rx_frame[index:index + 1], byteorder='little')
            sub_len = int.from_bytes(rx_frame[index - 4: (index + 1) - 4], byteorder='little')
            #print(sub_len)
            #print(status)

            index += sub_len + 1
            
            value = 0
            if type_of_value == 16:     # UNSIGNED_CHAR
                value = struct.unpack('<B', rx_frame[parse_index:parse_index + 1])[0]
            elif type_of_value == 17:   # SIGNED_CHAR
                value = struct.unpack('<b', rx_frame[parse_index:parse_index + 1])[0]
            elif type_of_value == 18:   # UNSIGNED_SHORT
                value = struct.unpack('<H', rx_frame[parse_index:parse_index + 2])[0]
            elif type_of_value == 19:   # SIGNED_SHORT
                value = struct.unpack('<h', rx_frame[parse_index:parse_index + 2])[0]
            elif type_of_value == 20:   # UNSIGNED_LONG
                value = struct.unpack('<L', rx_frame[parse_index:parse_index + 4])[0]
            elif type_of_value == 21:   # SIGNED_LONG
                value = struct.unpack('<l', rx_frame[parse_index:parse_index + 4])[0]
            elif type_of_value == 22:   # FLOAT
                value = struct.unpack('<f', rx_frame[parse_index:parse_index + 4])[0]
            elif type_of_value == 23:   # DOUBLE
                value = struct.unpack('<d', rx_frame[parse_index:parse_index + 8])[0]

            parse_index += sub_len + 1
            values.append(value)
            statuses.append(status)

        return values, statuses
    
    def send_request(self, receiver_id, command, command_version, payload):
        
        SOH, STX, ETX, EOT= b'\x01', b'\x02', b'\x03', b'\x04'
        VERSION = b'\x10'
        TO = int(receiver_id).to_bytes(1,'little')
        TO_CLASS = b'\x70'
        FROM = int(1).to_bytes(1,'little')
        FROM_CLASS = b'\xF0'
        
        LEN = 2
        for payload_byte in payload:
            LEN += 1
        LEN = int(LEN).to_bytes(1,'little')
        
        COMMAND = int(command).to_bytes(1,'little')
        COMMAND_VERSION = int(command_version).to_bytes(1,'little')
        
        # Assemble transmit-frame
        tx_frame = SOH + VERSION + TO + TO_CLASS + FROM + FROM_CLASS + LEN + STX + COMMAND + COMMAND_VERSION + payload + ETX
        # calculate checksum for trasmit-frame and concatenate
        tx_frame += self.calc_crc16(tx_frame).to_bytes(2, 'little') + EOT
        
        # Write transmit-frame to serial
        self.serial.write(tx_frame)
        #print([hex(c) for c in tx_frame])
        
        ### < --- --- > ###
        
        # Read frame from serial
        rx_frame = self.readFromSerial()
        #print("single channel response: " + str(rx_frame))
        #print([hex(c) for c in rx_frame])
        
        # compare checksum field to calculated checksum
        cs_calculated = self.calc_crc16(rx_frame[:-3]).to_bytes(2, 'little')
        cs_received = rx_frame[-3:-1]
        if (cs_calculated != cs_received):
            raise UMBError("RX-Error! Checksum test failed. Calculated Checksum: " + str(cs_calculated) + "| Received Checksum: " + str(cs_received))
        
        # Check the length of the frame
        length = int.from_bytes(rx_frame[6:7], byteorder='little')
        if (rx_frame[8+length:9+length] != ETX):
            raise UMBError("RX-Error! Length of Payload is not valid. length-field says: " + str(length))
        
        # Check if all frame field are valid
        if (rx_frame[0:1] != SOH):
            raise UMBError("RX-Error! No Start-of-frame Character")
        if (rx_frame[1:2] != VERSION):
            raise UMBError("RX-Error! Wrong Version Number")
        if (rx_frame[2:4] != (FROM + FROM_CLASS)):
            raise UMBError("RX-Error! Wrong Destination ID")
        if (rx_frame[4:6] != (TO + TO_CLASS)):
            raise UMBError("RX-Error! Wrong Source ID")
        if (rx_frame[7:8] != STX):
            raise UMBError("RX-Error! Missing STX field")
        if (rx_frame[8:9] != COMMAND):
            raise UMBError("RX-Error! Wrong Command Number")
        if (rx_frame[9:10] != COMMAND_VERSION):
            raise UMBError("RX-Error! Wrong Command Version Number")
            
        status = int.from_bytes(rx_frame[10:11], byteorder='little')
        type_of_value = int.from_bytes(rx_frame[13:14], byteorder='little')     
        value = 0
        #print("work: type_of_value: " + str(type_of_value))
        #print("work: status: " + str(status))
        
        if type_of_value == 16:     # UNSIGNED_CHAR
            value = struct.unpack('<B', rx_frame[14:15])[0]
        elif type_of_value == 17:   # SIGNED_CHAR
            value = struct.unpack('<b', rx_frame[14:15])[0]
        elif type_of_value == 18:   # UNSIGNED_SHORT
            value = struct.unpack('<H', rx_frame[14:16])[0]
        elif type_of_value == 19:   # SIGNED_SHORT
            value = struct.unpack('<h', rx_frame[14:16])[0]
        elif type_of_value == 20:   # UNSIGNED_LONG
            value = struct.unpack('<L', rx_frame[14:18])[0]
        elif type_of_value == 21:   # SIGNED_LONG
            value = struct.unpack('<l', rx_frame[14:18])[0]
        elif type_of_value == 22:   # FLOAT
            value = struct.unpack('<f', rx_frame[14:18])[0]
        elif type_of_value == 23:   # DOUBLE
            value = struct.unpack('<d', rx_frame[14:22])[0]
        
        return (value, status)
    
    def checkStatus(self, status):
        '''Check status code'''
        if status == 0:
            return ("Status: Command successful; no error; all OK")
        elif status == 16:
            return ("Status: Unknown command; not supported by this device")
        elif status == 17:
            return ("Status: Invalid parameter")
        elif status == 18:
            return ("Status: Invalid header version")
        elif status == 19:
            return ("Status: Invalid version of the command")
        elif status == 20:
            return ("Status: Invalid password for command")
        elif status == 32:
            return ("Status: Read error")
        elif status == 33:
            return ("Status: Write errorr")
        elif status == 34:
            return ("Status: Length too great; max. permissible length is designated in <maxlength>")
        elif status == 35:
            return ("Status: Invalid address / storage location")
        elif status == 36:
            return ("Status: Invalid channel")
        elif status == 37:
            return ("Status: Command not possible in this mode")
        elif status == 38:
            return ("Status: Unknown calibration command")
        elif status == 39:
            return ("Status: Calibration error")
        elif status == 40:
            return ("Status: Device not ready; e.g. initialization / calibrationrunning")
        elif status == 41:
            return ("Status: Under-voltage")
        elif status == 42:
            return ("Status: Hardware error")
        elif status == 43:
            return ("Status: Measurement error")
        elif status == 44:
            return ("Status: Error on device initialization")
        elif status == 45:
            return ("Status: Error in operating system")
        elif status == 48:
            return ("Status: Configuration error, default configuration was loaded")
        elif status == 49:
            return ("Status: Calibration error / the calibration is invalid, measurement not possible")
        elif status == 50:
            return ("Status: CRC error on loading configuration; defaultconfiguration was loaded")
        elif status == 51:
            return ("Status: CRC error on loading calibration; measurement not possible")
        elif status == 52:
            return ("Status: Calibration Step 1")
        elif status == 53:
            return ("Status: Calibration OK")
        elif status == 54:
            return ("Status: Channel deactivated")
    
    def onlineDataQuery(self, channel, receiver_id=1):
        '''Query data from one channel'''
        return self.send_request(receiver_id, 35, 16, int(channel).to_bytes(2,'little'))

    def onlineDataQueryMulti(self, channels, receiver_id=1):
        '''Query data from multiple channels'''
        values = []
        statuses = []

        for channel in channels:
            responses, responses2 = self.send_request(receiver_id, 35, 16, int(channel).to_bytes(2,'little'))
            values.append(responses)
            statuses.append(responses2)

        return values, statuses

    def onlineDataQueryMultiOneCall(self, channels, receiver_id=1):
        '''Query data from multiple channels in one call'''
        return self.send_request_one_call_multi(receiver_id, 47, 16, channels)

#dummy class for testing
class WS_UMB_dummy:
    def __init__(self):
        pass
    def __enter__(self):
        return self
    def __exit__(self, exception_type, exception_value, traceback):
        pass
    def onlineDataQuery (self, channel, receiver_id=1):
        return float(channel), 0
    def checkStatus(self, status):
        return ("Status: unbekanntes Kommando; wird von diesen Gerät nicht unterstützt")
    def close(self):
        pass

import sys
import json

if __name__ == "__main__":
    with WS_UMB() as umb:
    #with WS_UMB_dummy() as umb:
        mydict = {}  
        for channel in sys.argv[1:]:
            if 100 <= int(channel) <= 29999:
                value, status = umb.onlineDataQuery(channel)
                if status == 0:
                    mydict[channel] = value
                else:
                    sys.stderr.write("On channel " + str(channel) + " got bad " + umb.checkStatus(status) + "\n")
    print (json.dumps(mydict, separators=(',', ': ')))
