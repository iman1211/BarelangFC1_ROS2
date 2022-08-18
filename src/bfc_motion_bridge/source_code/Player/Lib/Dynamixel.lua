module(..., package.seeall);
require('DynamixelPacket');
require('unix');
require('stty');

fd = -1;
baudDefault = 2000000;

function open(ttyname, ttybaud)
   if (not ttyname) then
      local ttys = unix.readdir("/dev");
      for i=1,#ttys do
	 if (string.find(ttys[i], "tty.usb") or
	  string.find(ttys[i], "ttyUSB")) then
	    ttyname = "/dev/"..ttys[i];
	    break;
	 end
      end
   end
   assert(ttyname, "Dynamixel tty not found");

   print(string.format("Opening Dynamixel tty: %s\n", ttyname));

   if (fd >= 0) then unix.close(fd); end
   fd = unix.open(ttyname, unix.O_RDWR+unix.O_NOCTTY+unix.O_NONBLOCK);
   assert(fd >= 0, "Could not open port");

   ttybaud = ttybaud or baudDefault;
   -- Setup serial port parameters
   stty.raw(fd);
   stty.serial(fd);
   stty.speed(fd, ttybaud);

   return fd;
end

function close()
   if (fd >= 0) then
      unix.close(fd);
   end
   fd = -1;
end

function reset()
   print("Reseting Dynamixel tty");
   close();
   unix.usleep(100000);
   open();
end

function parse_status_packet(pkt)
   local t = {};
   -- print("parsing\n")
   t.id = pkt:byte(5);
   t.length_low = pkt:byte(6);
   t.length_high = pkt:byte(7);
   t.inst = pkt:byte(8)
   t.error = pkt:byte(9);
   t.parameter = {pkt:byte(10,t.length_low+3)};
   t.crc_low = pkt:byte(t.length_low+6);
   t.crc_high = pkt:byte(t.length_low+7);
   -- local ket = "crc_rx_low = ";
   -- local ket1 = "crc_rx_high = ";
   -- print(ket..(tostring(pkt:byte(t.length_low+6))))
   -- print(ket1..(tostring(pkt:byte(t.length_low+7))))
   return t;
end

function get_status(timeout)
   timeout = timeout or 0.010;
   local t0 = unix.time();
   local str = "";
   while (unix.time()-t0 < timeout) do
      local s = unix.read(fd);
      if (type(s) == "string") then
	      str = str..s;
         local ket = "Jumlah data terima = ";
         -- print(ket..(#str))
	      pkt = DynamixelPacket.input(str);

	      if (pkt) then
	         local status = parse_status_packet(pkt);
	         -- print(string.format("Status: id=%d error=%d",status.id,status.error));
	         return status;
	      end
      end
      unix.usleep(100);
   end
   return nil;
end

function send_ping(id)
   local inst = DynamixelPacket.ping(id);
   return unix.write(fd, inst);
end

function ping_probe(twait)
   twait = twait or 0.010;
   for id = 0,253 do
      send_ping(id);
      local status = get_status(twait);
      if (status) then
	   print(string.format("Ping: Dynamixel ID %d", status.id));
      end
   end
end

function set_delay(id, value)
   local addr = 9;  -- Return Delay address
   local inst = DynamixelPacket.write_byte(id, addr, value);
   return unix.write(fd, inst);
end

function set_id(idOld, idNew)
   local addr = 7;  -- ID
   local inst = DynamixelPacket.write_byte(idOld, addr, idNew);
   return unix.write(fd, inst);
end

function set_led(id, value)
   local addr = 65;  -- Led
   local inst = DynamixelPacket.write_byte(id, addr, value);
   return unix.write(fd, inst);
end

function set_torque_enable(id, value)
   -- print(value);
   local addr = 64;  -- Torque enable address
   local inst = DynamixelPacket.write_byte(id, addr, value);
   return unix.write(fd, inst);
end

function imu_calibration(id, value)
   -- print(value);
   local addr = 50;  -- imu_control address
   local inst = DynamixelPacket.write_byte(id, addr, value);
   return unix.write(fd, inst);
end

function dynamixel_power(id, value)
   -- print(value);
   local addr = 24;  -- dynamixel_power address
   local inst = DynamixelPacket.write_byte(id, addr, value);
   return unix.write(fd, inst);
end

function set_velocity(id, value)
   local addr = 44; -- Moving speed address
   local inst = DynamixelPacket.write_word(id, addr, value);
   return unix.write(fd, inst);
end

function set_hardness(id, value)
   local addr = 36;  -- Torque limit address
   local inst = DynamixelPacket.write_word(id, addr, value);
   return unix.write(fd, inst)
end

function set_command(id, value)
   local addr = 116;  -- Goal position address
   local inst = DynamixelPacket.write_word(id, addr, value);
   return unix.write(fd, inst)
end

function get_led(id)
   local twait = 0.100;
   local addr = 65;  -- Led address
   local inst = DynamixelPacket.read_data(id, addr, 4);
   unix.read(fd); -- clear old status packets
   unix.write(fd, inst)
   local status = get_status(twait);
   if (status) then
      return status.parameter[1];
   end
end

function get_delay(id)
   local twait = 0.100;
   local addr = 9;  -- Return delay address
   local inst = DynamixelPacket.read_data(id, addr, 4);
   unix.read(fd); -- clear old status packets
   unix.write(fd, inst)
   local status = get_status(twait);
   if (status) then
      return status.parameter[1];
   end
end

function get_torque_enable(id)
   local twait = 0.100;
   local addr = 64;  -- Torque enable address
   local inst = DynamixelPacket.read_data(id, addr, 4);
   unix.read(fd); -- clear old status packets
   unix.write(fd, inst)
   local status = get_status(twait);
   if (status) then
      return status.parameter[1];
   end
end

function get_position(id)
   local twait = 0.100;
   local addr = 132;  -- Present position address
   local inst = DynamixelPacket.read_data(id, addr, 4);
   unix.read(fd); -- clear old status packets
   unix.write(fd, inst)
   local status = get_status(twait);
   if (status) then
      -- print(string.format("Dynamixel ID %d Position %d%d", status.id,status.parameter[2],status.parameter[1]));
      return DynamixelPacket.byte_to_word(unpack(status.parameter,1,2));
   end
end

function get_command(id)
   local twait = 0.100;
   local addr = 116;  -- Goal position address
   local inst = DynamixelPacket.read_data(id, addr, 4);
   unix.read(fd); -- clear old status packets
   unix.write(fd, inst)
   local status = get_status(twait);
   if (status) then
      return DynamixelPacket.byte_to_word(unpack(status.parameter,1,2));
   end
end

function get_velocity(id)
   local twait = 0.100;
   local addr = 44; -- Moving speed address
   -- local inst = DynamixelPacket.read_data(id, addr, 4);
   unix.read(fd); -- clear old status packets
   unix.write(fd, inst)
   local status = get_status(twait);
   if (status) then
      return DynamixelPacket.byte_to_word(unpack(status.parameter,1,2));
   end
end

function get_hardness(id)
   local twait = 0.100;
   local addr = 36;  -- Torque limit address
   --local inst = DynamixelPacket.read_data(id, addr, 4);
   unix.read(fd); -- clear old status packets
   unix.write(fd, inst)
   local status = get_status(twait);
   if (status) then
      return DynamixelPacket.byte_to_word(unpack(status.parameter,1,2));
   end
end

function get_battery(id)
   local twait = 0.100;
   local addr = 144;  -- Present voltage address
   local inst = DynamixelPacket.read_data(id, addr, 4);
   unix.read(fd); -- clear old status packets
   unix.write(fd, inst)
   local status = get_status(twait);
   if (status) then
      return status.parameter[1];
   end
end

function get_temperature(id)
   local twait = 0.100;
   local addr = 146;  -- Present Temperature
   local inst = DynamixelPacket.read_data(id, addr, 4);
   unix.read(fd); -- clear old status packets
   unix.write(fd, inst)
   local status = get_status(twait);
   if (status) then
      return status.parameter[1];
   end
end

function read_data(id, addr, len, twait)
   twait = twait or 0.100;
   len  = len or 2;
   local inst = DynamixelPacket.read_data(id, addr, len);
   unix.read(fd); -- clear old status packets
   unix.write(fd, inst)
   local status = get_status(twait);
   if (status) then
      return status.parameter;
   end
end

function bulk_read_data(id_cm730, ids, addr, len, twait)
   twait = twait or 0.100;
   len  = len or 2;
   local inst = DynamixelPacket.bulk_read_data(
	id_cm730,string.char(unpack(ids)), addr, len, #ids);
   unix.read(fd); -- clear old status packets
   unix.write(fd, inst)
   local status = get_status(twait);
   if (status) then
      return status.parameter;
   end
end

function sync_write_byte(ids, addr, data)
   local nid = #ids;
   local len = 1;
   local t = {};
   local n = 1;
   for i = 1,nid do
      t[n] = ids[i];
      t[n+1] =  data[i];
      n = n + len + 1;
   end
   local inst = DynamixelPacket.sync_write(addr, len,
					   string.char(unpack(t)));
   unix.write(fd, inst);
end

function sync_write_word(ids, addr, data)
   local nid = #ids;
   local len = 4;
   local t = {};
   local n = 1;
   for i = 1,nid do
      t[n] = ids[i];
      t[n+1] = DynamixelPacket.word_to_byte(data[i] % 2^8);
      t[n+2] = DynamixelPacket.word_to_byte(math.floor((data[i] % 2^16) / 2^8));
      t[n+3] = DynamixelPacket.word_to_byte(math.floor((data[i] % 2^24) / 2^16));
      t[n+4] = DynamixelPacket.word_to_byte((math.floor(data[i] / 2^24))); 
      n = n + len + 1;
   end
   local inst = DynamixelPacket.sync_write(addr, len,
					   string.char(unpack(t)));
   unix.write(fd, inst);
end
