# USART DFU Protocol 1.0

(Copyright 2021 BG5HHP)

#### Note: Byte order is Big-Endian(Network Order) eg: 0x12345678

## 1. Initial Packet
Host sends the initial packet to device to start the flash operation.  

Type = 0x00, Len = 12, Value = block_type(8)=0(flash)/1(eeprom), block_size8=128, block_count16/total_size32/crc32  

### 1.1 Host sends initial packet
```
           HEAD | LEN (15)  | TYPE | DAT_TYPE | BLK_SIZE | DATA_SIZE           | DATA_CRC            | CRC32
[Host] --> 0xFE | 0x00 0x00 | 0x00 | 0x00     | 0x00     | 0x00 0x00 0x00 0x00 | 0x00 0x00 0x00 0x00 | 0x00 0x00 0x00 0x00
```

### 1.2 Device respond with same type, with 1 byte (ack0/nak1/error2) value and crc32
```
           HEAD | LEN (06)  | TYPE | ACK  | CRC32
[Dev]  --> 0xFE | 0x00 0x00 | 0x00 | 0x00 | 0x00 0x00 0x00 0x00
```

## 2. Write Data Packet
Host sends the firmware data to write into the device flash. The start address is defined by the bootloader.  
The host proceed only get ack(0), retry n-times for the nak(1), and abort for the error(2)  

Type = 0x01, Len = ??, Value = block_data/crc32  

### 2.1 Host Sends Firmware Data to Device
```
           HEAD | LEN (133) | TYPE | BLK_DATA(128) | CRC32
[Host] --> 0xFE | 0x00 0x00 | 0x01 | 0x00 ... 0x00 | 0x00 0x00 0x00 0x00
```

### 2.2 Device respond with ack(0)/nak(1)/err(2)
Only initial packet is received that the write command could proceed, otherwise error(2) returns.
```
           HEAD | LEN (06)  | TYPE | ACK  | CRC32
[Dev]  --> 0xFE | 0x00 0x00 | 0x01 | 0x00 | 0x00 0x00 0x00 0x00
```

## 3. Verify Written Data
Host sends data verification command to device and the device returns the firmware crc32 code  
Type = 0x02

### 3.1 Host Requests Data Verification
```
           HEAD | LEN (06)  | TYPE | DAT_TYPE | CRC32
[Host] --> 0xFE | 0x00 0x00 | 0x02 | 0x00     | 0x00 0x00 0x00 0x00

```

### 3.2 Device Respond with 128bit hash or crc32 of the flashed data
```
           HEAD | LEN (10+) | TYPE | ACK  | CRC32 or HASH       | CRC32
[Dev]  --> 0xFE | 0x00 0x00 | 0x02 | 0x00 | 0x00 0x00 0x00 0x00 | 0x00 0x00 0x00 0x00
```


## 4. Read Data
type = 0x03


## 4. Command Packet
Type = 0x80 ~ 0xFF (bit7 = 1)

### 4.1 GetInfo Command
Type = 0x80  

### 4.2 Reboot Command
Type = 0x81  


