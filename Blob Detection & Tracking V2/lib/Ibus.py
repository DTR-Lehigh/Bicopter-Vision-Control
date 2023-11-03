"""
Author       : Hanqing Qi
Date         : 2023-11-03 19:16:19
LastEditors  : Hanqing Qi
LastEditTime : 2023-11-03 19:48:10
FilePath     : /Bicopter-Vision-Control/Blob Detection & Tracking V2/lib/Ibus.py
Description  : The Ibus library for the Bicopter Vision Control project.
"""

from pyb import UART

# Macros
IBUS_MSG_LEN = 32 # The length of the Ibus message
IBUS_MSG_HEADER = [0x20, 0x40] # The header of the Ibus message
NICLA_TGT = 0x11 # Flag to set Nicla in target mode
NICLA_GAL = 0x22 # Flag to set Nicla in goal mode

class Ibus:
    def __init__(self, pinset:str="LP1", baudrate:int=115200, timeout_char:int=2000)->None:
        """
        @description: Initialize the Ibus object.
        @param       {*} self: 
        @param       {str} pinset: The set of RX and TX pins for the UART (Should not be changed)
        @param       {int} baudrate: The baudrate of the UART (Default: 115200)
        @param       {int} timeout_char: The timeout time for the UART (Default: 2000)
        @return      {*} None
        """
        # Initialize the UART
        self.uart = UART(pinset, baudrate, timeout_char) # (TX, RX) = (P1, P0) = (PB14, PB15)

    def _pack_msg(self, raw_msg:list)->bytearray:
        """
        @description: Pack the raw_msg into a Ibus message.
        @param       {*} self: 
        @param       {list} raw_msg: The raw message to be packed
        @return      {bytearray} The packed Ibus message
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
        checksum = 0
        for char in msg:
            checksum ^= ord(char) # XOR
        # Convert the checksum to a byte tuple
        ch1 = checksum & 0xFF # The lower 8 bits
        ch2 = (checksum >> 8) & 0xFF # The higher 8 bits
        return (ch1, ch2)

    def send(self, raw_msg:list=[0,0,0,0])->None:
        """
        @description: Send the raw_msg to the receiver.
        @param       {*} self: 
        @param       {list} raw_msg: The raw message to be sent
        @return      {*} None
        """
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
