import asyncio
import nest_asyncio
from bleak import BleakClient
import struct
import time


# Adresse obtenue avec Bleak scan
# MAC adress du looger 1 : utiliser bluesee
address = "8AECE6F8-FE3F-41F6-A6AE-2DCAC51C1B95"

# MAC adress du looger 2r
#address = "B8EEE7CE-860B-48F5-8385-2DFC55EC4F4E"

LOGGER_READ_UUID = "19b10002-e8f2-537e-4f6c-d104768a1214"
LOGGER_WRITE_UUID = "19b10003-e8f2-537e-4f6c-d104768a1214"

null = b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'

def convert_bytes_float(binary_data):
    FLOAT = 'f'
    #binary_data = bytes.fromhex(s_hex[2:])
    fmt = '<' + FLOAT * (len(binary_data) // struct.calcsize(FLOAT))
    if (len(binary_data)) <= (struct.calcsize(FLOAT)):
            numbers = 0
    else:
            numbers = struct.unpack(fmt, binary_data)
    return numbers

async def run(address, loop):
    client = BleakClient(address, loop=loop) 
    
    try:
        await client.connect()
        
        while True:
            value = await client.read_gatt_char(LOGGER_READ_UUID) 
           
            print('Command Logger:')
            mot = input()
            
            if mot == 'read':
                    if (value == null):
                        print(value)
                    else:
                        data = convert_bytes_float(value)
                        print("Heading: {:.2f}, Pitch: {:.2f}, Roll: {:.2f}".format(data[2],data[1],data[0]))
                
            if mot == 'stop':
                break 
            
            
            if mot == 'on':
                write_value = bytes('A', 'utf-8')
                await client.write_gatt_char(LOGGER_WRITE_UUID, write_value)
                
            if mot == 'off':
                write_value = bytes('Z', 'utf-8')
                await client.write_gatt_char(LOGGER_WRITE_UUID, write_value)
            
            
    except Exception as e:
        print(e) 
        
    finally:
        await client.disconnect()

nest_asyncio.apply()
loop = asyncio.get_event_loop()
loop.run_until_complete(run(address, loop))