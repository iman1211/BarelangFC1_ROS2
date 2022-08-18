/*
  Lua module to provide process dynamixel packets
*/

#include "dynamixel.h"
#include <stdio.h>
#include <iostream>   
#include <stdlib.h>
#include <cstdint>
#include <cmath>
#include <bitset>
using std::cout;
using std::endl;
using std::cin;
using std::hex;
using std::dec;
using std::bitset;

#ifdef __cplusplus
extern "C"
{
#endif
  #include "lua.h"
  #include "lualib.h"
  #include "lauxlib.h"
#ifdef __cplusplus
}
#endif

static int lua_pushpacket(lua_State *L, DynamixelPacket *p) {
  if (p != NULL) {
    int nlen = p->length_low + 7;
    lua_pushlstring(L, (char *)p, nlen);
    return 1;
  }
  return 0;
}

static int lua_dynamixel_instruction_ping(lua_State *L) {
  int id = luaL_checkint(L, 1);
  // std::cout << "\n" << "/////////////id : " << id << "////////////\n";
  DynamixelPacket *p = dynamixel_instruction_ping(id);
  //printf("%d\n",sizeof(p));
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_read_data(lua_State *L) {
  int id = luaL_checkint(L, 1);
  unsigned char addr = luaL_checkint(L, 2);
  unsigned char len = luaL_optinteger(L, 3, 1);
  // printf("id_read = %d, addr_read = %hu, len_read = % hu\n",id,addr,len);
  DynamixelPacket *p = dynamixel_instruction_read_data
    (id, addr, len);
  return lua_pushpacket(L, p);
}

//ADDED for bulk read
static int lua_dynamixel_instruction_bulk_read_data(lua_State *L) {
  uchar id_cm730 = luaL_checkint(L, 1);
  size_t nstr;
  const char *str = luaL_checklstring(L, 2, &nstr);
  // printf("str[0] = %hu\n",str[0]);
  uchar addr = luaL_checkint(L, 3);
  uchar len = luaL_checkint(L, 4);
  DynamixelPacket *p = dynamixel_instruction_bulk_read_data
    (id_cm730, (uchar *) str, addr, len, nstr);
  return lua_pushpacket(L, p);
}


static int lua_dynamixel_instruction_write_data(lua_State *L) {
  uchar id = luaL_checkint(L, 1);
  uchar addr = luaL_checkint(L, 2);
  size_t nstr;
  const char *str = luaL_checklstring(L, 3, &nstr);
  DynamixelPacket *p = dynamixel_instruction_write_data
    (id, addr, (uchar *)str, nstr);
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_write_byte(lua_State *L) {
  uchar id = luaL_checkint(L, 1);
  uchar addr = luaL_checkint(L, 2);
  uchar byte = luaL_checkint(L, 3);
  DynamixelPacket *p = dynamixel_instruction_write_data
    (id, addr, &byte, 1);
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_write_word(lua_State *L) {
  uchar id = luaL_checkint(L, 1);
  uchar addr = luaL_checkint(L, 2);
  unsigned short word = luaL_checkint(L, 3);
  uchar byte[4];
  byte[0] = DXL_LOBYTE(DXL_LOWORD(word)); //(word & 0x00FF); 
  byte[1] = DXL_HIBYTE(DXL_LOWORD(word)); //(word & 0xFF00) >> 8;
  byte[2] = DXL_LOBYTE(DXL_HIWORD(word)); //(word & 0x00FF); 
  byte[3] = DXL_HIBYTE(DXL_HIWORD(word)); //(word & 0xFF00) >> 8;
  for (size_t i = 0; i < sizeof(byte); i++)
  {
    // printf("byte[%d] = %hu\n",i,byte[i]);
  }
  DynamixelPacket *p = dynamixel_instruction_write_data
    (id, addr, byte, 4);
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_sync_write(lua_State *L) {
  uchar addr = luaL_checkint(L, 1);
  uchar len = luaL_checkint(L, 2);
  size_t nstr;
  const char *str = luaL_checklstring(L, 3, &nstr);
  // printf("strs %hu\n",str[0]);
  DynamixelPacket *p = dynamixel_instruction_sync_write
    (addr, len, (uchar *)str, nstr);
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_input(lua_State *L) {
  size_t nstr;
  const char *str = luaL_checklstring(L, 1, &nstr);
  int nPacket = luaL_optinteger(L, 2, 1)-1;
  DynamixelPacket pkt;
  int ret = 0;
  if (str) {
    //printf("lua_dxl_input\n");
    ///printf("npacket %d\n", nPacket);
    for (int i = 0; i < nstr; i++) {
      // printf("status [%d] = %hu\n", i, str[i]);
      nPacket = dynamixel_input(&pkt, str[i], nPacket);
      if (nPacket < 0) {
	ret += lua_pushpacket(L, &pkt);
      }
    }
  }
  return ret;
}

static int lua_dynamixel_byte_to_word(lua_State *L) {
  int n = lua_gettop(L);
  int ret = 0;
  for (int i = 1; i < n; i += 2) {
    unsigned short byteLow = luaL_checkint(L, i);
    unsigned short byteHigh = luaL_checkint(L, i+1);
    unsigned short word = (byteHigh << 8) + byteLow;
    lua_pushnumber(L, word);
    ret++;
  }
  return ret;
}

static int lua_dynamixel_word_to_byte(lua_State *L) {
  int n = lua_gettop(L);
  int ret = 0;
  // printf("zzzz = %d\n",n);
  for (int i = 1; i <= n; i++) {
    unsigned short word = luaL_checkint(L, i);
    unsigned short byteLow = word & 0x00FF; // DXL_LOBYTE(DXL_LOWORD(word));
    lua_pushnumber(L, byteLow);
    ret++;
    unsigned short byteHigh = (word & 0xFF00)>>8; // DXL_HIBYTE(DXL_LOWORD(word));
    lua_pushnumber(L, byteHigh);
    ret++;
  }
  return ret;
}

static int lua_dynamixel_get_low_byte(lua_State *L) {
  int ret = 0;
  uint16_t word = luaL_checkint(L, 1);
  // printf("word = %hu\n",word);
  uchar byteLow = word&(0xff); 
  printf("byte_low = %hu\n",byteLow);
  lua_pushnumber(L, byteLow);
  return ret++;
}

static int lua_dynamixel_get_high_byte(lua_State *L) {
  int ret = 0;
  uint16_t word = luaL_checkint(L, 1);
  // printf("word = %hu\n",word);
  uchar byteHigh = (word>>8) & 0xff;
  printf("byte_high = %hu\n",byteHigh);
  lua_pushnumber(L, byteHigh);
  return ret++;
}

static int twos_complement(lua_State *L)
{
  const int smallInt = luaL_checkint(L,1);
  printf("aaaaaaaaaaaaa%d",smallInt);
  const int negative = (smallInt & (1 << 17)) != 0;
  int nativeInt;

  if (negative)
    nativeInt = smallInt | ~((1 << 18) - 1);
  else
    nativeInt = smallInt;
  return nativeInt;
}

static int ConvertTwosComplementByteToInteger(lua_State *L)
{
  unsigned short int rawValue = luaL_checkint(L,1);
  printf("ss = %hu\n",rawValue);
  // If a positive value, return it
  if ((rawValue & 0x80) == 0)
  {
    return rawValue;
    lua_pushnumber(L, rawValue);
  } else {
  // Otherwise perform the 2's complement math on the value
    return (unsigned short int)(~(rawValue - 0x01)) * -1;
    lua_pushnumber(L, rawValue);
    printf("ss = %hu\n",rawValue);
  }
}

static const struct luaL_reg dynamixelpacket_functions[] = {
  {"input", lua_dynamixel_input},
  {"ping", lua_dynamixel_instruction_ping},
  {"write_data", lua_dynamixel_instruction_write_data},
  {"write_byte", lua_dynamixel_instruction_write_byte},
  {"write_word", lua_dynamixel_instruction_write_word},
  {"sync_write", lua_dynamixel_instruction_sync_write},
  {"read_data", lua_dynamixel_instruction_read_data},
  {"bulk_read_data", lua_dynamixel_instruction_bulk_read_data},
  {"word_to_byte", lua_dynamixel_word_to_byte},
  {"byte_to_word", lua_dynamixel_byte_to_word},
  {"get_low_byte", lua_dynamixel_get_low_byte},
  {"get_high_byte", lua_dynamixel_get_high_byte},
  {"twos_complement", twos_complement},
  {"convert", ConvertTwosComplementByteToInteger},
  {NULL, NULL}
};

static const struct luaL_reg dynamixelpacket_methods[] = {
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_DynamixelPacket (lua_State *L) {
  luaL_newmetatable(L, "dynamixelpacket_mt");

  // OO access: mt.__index = mt
  // Not compatible with array access
  lua_pushvalue(L, -1);
  lua_setfield(L, -2, "__index");

  luaL_register(L, NULL, dynamixelpacket_methods);
  luaL_register(L, "DynamixelPacket", dynamixelpacket_functions);

  return 1;
}
