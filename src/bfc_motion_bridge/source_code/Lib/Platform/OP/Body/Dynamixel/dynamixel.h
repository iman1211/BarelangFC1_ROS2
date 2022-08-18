#ifndef __DYNAMIXEL_H
#define __DYNAMIXEL_H

#ifdef __cplusplus
extern "C" {
#endif

#define DYNAMIXEL_PACKET_HEADER (255)
#define DYNAMIXEL_PACKET_HEADER3 (253)
#define DYNAMIXEL_PARAMETER_MAX (250)
#define DYNAMIXEL_BROADCAST_ID (254)
#define DYNAMIXEL_RESERVED (0)

#define INST_PING (1)
#define INST_READ (2)
#define INST_WRITE (3)
#define INST_REG_WRITE (4)
#define INST_ACTION (5)
#define INST_RESET (6)
#define INST_SYNC_WRITE (131)
#define INST_BULK_READ (146)

#define ERRBIT_VOLTAGE          (1)
#define ERRBIT_ANGLE            (2)
#define ERRBIT_OVERHEAT         (4)
#define ERRBIT_RANGE            (8)
#define ERRBIT_CHECKSUM         (16)
#define ERRBIT_OVERLOAD         (32)
#define ERRBIT_INSTRUCTION      (64)

typedef unsigned char uchar;
typedef unsigned short int  uint16_t;
typedef unsigned int        uint32_t;
// typedef unsigned int        uint64_t;

#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uchar)(((uint32_t)(a)) & 0xff)) | ((uint16_t)((uchar)(((uint32_t)(b)) & 0xff))) << 8))
#define DXL_LOBYTE(w)       ((uchar)(((uint32_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uchar)((((uint32_t)(w)) >> 8) & 0xff))
#define DXL_LOWORD(l)       ((uint16_t)(((uint32_t)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((uint16_t)((((uint32_t)(l)) >> 16) & 0xffff))

typedef struct DynamixelPacket {
  uchar header1;
  uchar header2;
  uchar header3; //protocol 2.0
  uchar reserved; //protocol 2.0
  uchar id;
  uchar length_low; // length does not include first 4 bytes
  uchar length_high; //protocol 2.0
  uchar instruction; // or error for status packets
  uchar parameter[DYNAMIXEL_PARAMETER_MAX]; // reserve for maximum packet size
  uchar crc_low; //protocol 2.0
  uchar crc_high; //protocol 2.0
  
} DynamixelPacket;

  DynamixelPacket *dynamixel_instruction(uchar id,
					 uchar inst,
					 uchar *parameter,
					 uchar nparameter);
  DynamixelPacket *dynamixel_instruction_read_data(uchar id,
						   uchar address, uchar n);
  DynamixelPacket *dynamixel_instruction_write_data(uchar id,
						    uchar address,
						    uchar data[], uchar n);
  DynamixelPacket *dynamixel_instruction_reg_write(uchar id,
						   uchar address,
						   uchar data[], uchar n);
  DynamixelPacket *dynamixel_instruction_action();
  DynamixelPacket *dynamixel_instruction_ping(int id);
  DynamixelPacket *dynamixel_instruction_reset(int id);
  DynamixelPacket *dynamixel_instruction_sync_write(uchar address,
						    uchar len,
						    uchar data[], uchar n);


  //added for bulk read
  DynamixelPacket *dynamixel_instruction_bulk_read_data(
	uchar id_cm730, uchar id[], uchar address, uchar len, uchar n);
  

  int dynamixel_input(DynamixelPacket *pkt, uchar c, int n);
  
#ifdef __cplusplus
}
#endif

#endif // __DYNAMIXEL_H
