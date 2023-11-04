"""
Author       : Hanqing Qi, Karen Li
Date         : 2023-11-03 19:16:19
LastEditors  : Hanqing Qi
LastEditTime : 2023-11-04 13:30:11
FilePath     : /Bicopter-Vision-Control/Blob Detection & Tracking V2/lib/Ibus.py
Description  : The iBus library for the Bicopter Vision Control project.
"""

from pyb import UART

# Macros
IBUS_MSG_LEN = 32 # The length of the iBus message
IBUS_MSG_HEADER = [0x20, 0x40] # The header of the iBus message
NICLA_TGT = 0x11 # Flag to set Nicla in target mode
NICLA_GAL = 0x22 # Flag to set Nicla in goal mode

class iBus:
    def __init__(self, pinset:str="LP1", baudrate:int=115200, timeout:int=2000)->None:
        """
        @description: Initialize the iBus object.
        @param       {*} self: 
        @param       {str} pinset: The set of RX and TX pins for the UART (Should not be changed)
        @param       {int} baudrate: The baudrate of the UART (Default: 115200)
        @param       {int} timeout: The timeout time for the UART (Default: 2000)
        @return      {*} None
        """
        # Initialize the UART
        self.uart = UART(pinset, baudrate, timeout_char=timeout) # (TX, RX) = (P1, P0) = (PB14, PB15)
        # Flush the buffer
        self._flush_buffer()

    def _pack_msg(self, raw_msg:list)->bytearray:
        """
        @description: Pack the raw_msg into a iBus message.
        @param       {*} self: 
        @param       {list} raw_msg: The raw message to be packed
        @return      {bytearray} The packed iBus message
        """
        msg = bytearray(IBUS_MSG_LEN)
        msg[0] = IBUS_MSG_HEADER[0]
        msg[1] = IBUS_MSG_HEADER[1]
        # Check the raw_msg length
        if len(raw_msg) > 14:
            raise ValueError("The length of the raw_msg is too long!")
        for i in range(len(raw_msg)):
            raw_byte_tuple = bytearray(raw_msg[i].to_bytes(2, 'little')) # Convert the int to a byte tuple
            msg[2*i+2] = raw_byte_tuple[0]
            msg[2*i+3] = raw_byte_tuple[1]

        # Calculate the checksum
        checksum = self._checksum(msg[2:])
        msg[30] = checksum[0]
        msg[31] = checksum[1]
        return msg

    def _checksum(self, msg:bytearray)->tuple:
        """
        @description: Calculate the checksum of the message.
        @param       {*} self:
        @param       {bytearray} msg: The message to be calculated
        @return      {tuple} The two bytes of the checksum
        """
        sum = 0
        for b in msg:
            sum += b
        checksum = 0xFFFF - sum
        chA = checksum >> 8
        chB = checksum & 0xFF
        return (chA, chB)
    
    def _flush_buffer(self)->None:
        """
        @description: Flush the buffer of the UART.
        @param       {*} self: 
        @return      {*} None
        """
        self.uart.read(self.uart.any())

    def send(self, raw_msg:list=[0,0,0,0])->None:
        """
        @description: Send the raw_msg to the receiver.
        @param       {*} self: 
        @param       {list} raw_msg: The raw message to be sent
        @return      {*} None
        """
        # Flush the buffer
        self._flush_buffer()
        # Pack the message
        msg = self._pack_msg(raw_msg)
        self.uart.write(msg)
        
    def receive(self)->str:
        """
        @description: Receive the message from the UART.
        @param       {*} self: -
        @return      {str} The flag to set Nicla in target mode or goal mode
        """
        if self.uart.any():
            msg = self.uart.read(32)
            if msg == NICLA_TGT:
                return "T"
            elif msg == NICLA_GAL:
                return "G"
            else: 
                return "N" # Receive malformed message

if __name__ == "__main__":
    # Initialize the iBus class with default parameters
    ibus = iBus()
    # Define a test raw message list
    test_raw_msg = [121, 1117, 211, 1104]
    # Test sending a message
    print("Testing sending function...")
    ibus.send(test_raw_msg)
    print("Message sent.")