#!/usr/bin/env python3

from pymodbus.client import ModbusSerialClient as ModbusClient

class rs_485:
    def __init__(self, port, stopbits, bytesize, parity, baudrate, timeout):
        self.method = "rtu"
        self.port = port
        self.stopbits = stopbits
        self.bytesize = bytesize
        self.parity = parity
        self.baudrate = baudrate
        self.timeout = timeout
    def connect_(self):
        client = ModbusClient(
            method = self.method,
            port = self.port,
            stopbits=self.stopbits,
            bytesize=self.bytesize,
            parity=self.parity,
            baudrate=self.baudrate,
            timeout = self.timeout,
        )
        return client
    def write_value_(self, address, value, id):
        self.result_ = self.connect_().write_registers(
            address=address,
            values=value,
            slave =id,
            skip_encode = True,
        )
        return self.result_
    def read_speed_value_(self, address, count, id):
        self.speed_rep = self.connect_().read_holding_registers(
            address=address,
            count = count,
            slave = id,
        )
        return self.speed_rep.registers[1]
        