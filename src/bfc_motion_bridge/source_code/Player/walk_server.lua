module(... or '', package.seeall)

-- Get Platform for package path
cwd = '.';
local platform = os.getenv('PLATFORM') or '';
if (string.find(platform,'webots')) then cwd = cwd .. '/Player';
end

-- Get Computer for Lib suffix
local computer = os.getenv('COMPUTER') or '';
if (string.find(computer, 'Darwin')) then
  -- MacOS X uses .dylib:
  package.cpath = cwd .. '/Lib/?.dylib;' .. package.cpath;
else
  package.cpath = cwd .. '/Lib/?.so;' .. package.cpath;
end

package.path = cwd .. '/?.lua;' .. package.path;
package.path = cwd .. '/Util/?.lua;' .. package.path;
package.path = cwd .. '/Config/?.lua;' .. package.path;
package.path = cwd .. '/Lib/?.lua;' .. package.path;
package.path = cwd .. '/Dev/?.lua;' .. package.path;
package.path = cwd .. '/Motion/?.lua;' .. package.path;
package.path = cwd .. '/Motion/keyframes/?.lua;' .. package.path;
package.path = cwd .. '/Motion/Walk/?.lua;' .. package.path;
package.path = cwd .. '/Vision/?.lua;' .. package.path;
package.path = cwd .. '/World/?.lua;' .. package.path;

require('unix')
require('Config')
require('shm')
require('vector')
require('mcm')
require('Speak')
require('getch')
require('Body')
require('Motion')
require('dive')

require('grip')

-------------- UDP COMMUNICATION FOR BODY KINEMATIC ----------
local socket_body = require "socket"
local udp_body = socket.udp()
udp_body:settimeout(0)
udp_body:setsockname('*', 5000)
local data_body, msg_or_ip_body, port_or_nil_body
-------------------------------------------------------

--------------- UDP COMMUNICATION FOR HEAD MOVEMENT --------------
local socket_head = require "socket"
local udp_head = socket.udp()
udp_head:settimeout(0)
udp_head:setsockname('*', 5001)
local data_head, msg_or_ip_head, port_or_nil_head
---------------------------------------------------


Motion.entry();
darwin = false;
webots = false;

SensorCM=shm.open('dcmSensor');

-- Enable OP specific 
if(Config.platform.name == 'OP') then
  darwin = true;
--SJ: OP specific initialization posing (to prevent twisting)
--  Body.set_body_hardness(0.3);
--  Body.set_actuator_command(Config.stance.initangle)
end

--TODO: enable new nao specific
newnao = false; --Turn this on for new naos (run main code outside naoqi)
newnao = true;

getch.enableblock(1);
unix.usleep(1E6*1.0);
Body.set_body_hardness(0);

--This is robot specific 
webots = false;
init = false;
calibrating = false;
ready = false;
if( webots or darwin) then
  ready = true;
end

initToggle = true;
targetvel=vector.zeros(3);
button_pressed = {0,0};


function string:split(inSplitPattern, outResults)
   if not outResults then
      outResults = { }
   end
   local theStart = 1
   local theSplitStart, theSplitEnd = string.find( self, inSplitPattern, theStart )
   while theSplitStart do
      table.insert( outResults, string.sub( self, theStart, theSplitStart-1 ) )
      theStart = theSplitEnd + 1
      theSplitStart, theSplitEnd = string.find( self, inSplitPattern, theStart )
   end
   table.insert( outResults, string.sub( self, theStart ) )
   return outResults
end
-- deklarasi variable tunning
local regres = true;
local errorWalkX = -0.006999; -- sesuai dengan error Walk di configure.ini -- -0.0059990001666389
local errorWalkA = -0.005; -- sesuai dengan error Walk di configure.ini 
local parsing;
local inputStart = errorWalkX;
local inputEnd = walk.velLimitX[2];
local outputStart = 0.035;
local outputEnd = 0.08;
local Output;
local Error = 0;
local Perror;
local dataAkhir;
local walkKine = errorWalkX;
local actual_walk = walk.get_velocity();


local Count = 0;
local ErrorStep;
local PerrorStep;
local dataAkhirStep;
local walkKineStep;
local selisih = 0;
local selisihA = 0;
local fallCheck = Motion.fall;
local posHead = vector.new({0,-120})*math.pi/180;
local stateFallCheck = 'done';

os.execute("screen -d player");
function process_keyinput()
	---------Start Program Kirim Data Sensor--------
	local CMAcc=SensorCM:get('imuAcc');
	local CMGyro=SensorCM:get('imuGyr');
	-- local CMAngle=vector.new(SensorCM:get('imuAngle'))*180/math.pi;
	local CMAngle=SensorCM:get('imuAngle');
	local CMStrategy=SensorCM:get('strategy')
	local ServPos=vector.new(SensorCM:get('position'))*180/math.pi;

	--[[ outfile3 = io.open("SensorNote","w");
	outfile3:write(string.format("%.2f;%.2f;%.2f",unpack(CMAcc)) , ";" , string.format("%.2f;%.2f;%.2f",unpack(CMGyro)) , ";" , string.format("%.2f;%.2f;%.2f",unpack(CMAngle)), ";" , string.format("%d;%d",unpack(CMStrategy)))
	outfile3:close() ]]

	outfile4 = io.open("PosNote","w");
	outfile4:write(string.format("%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f ",unpack(ServPos)))
	outfile4:close()
	---------End Program Kirim Data Sensor--------

	data_body, msg_or_ip_body, port_or_nil_body = udp_body:receivefrom() -- tampung data dari socket
	--start proses input motion and walk
	if data_body ~= nil then
		if data_body then
			local byte;
			parsing = data_body:split(",") -- pemisahan data dari header "motion" = motion , "walk" = velocity
			if parsing[1] == "motion" then 
				byte=string.byte(parsing[2]);
				-- print(byte)
				if byte==string.byte("1") then	
					kick.set_kick("kickOffLeftSyntethic");
					Motion.event("kick");
					walk.initStep = 2;
				elseif byte==string.byte("2") then	
					kick.set_kick("kickOffRightSyntethic");
					Motion.event("kick");
					walk.initStep = 1;
				elseif byte==string.byte("3") then	
					kick.set_kick("kickSideLeft90");
					Motion.event("kick");
					walk.initStep = 2;
				elseif byte==string.byte("4") then	
					kick.set_kick("kickSideRight90");
					Motion.event("kick");
					walk.initStep = 1;
				elseif byte==string.byte("5") then
					walk.hardnessSupport    = 1; --.75
					walk.hardnessSwing      = 1; --.5
					walk.hardnessArm        = .3; --.3
					walk.doWalkKickLeft();
					walk.initStep = 2;
				elseif byte==string.byte("6") then
					walk.hardnessSupport    = 1; --.75
					walk.hardnessSwing      = 1; --.5
					walk.hardnessArm        = .3; --.3
					walk.doWalkKickRight();
					walk.initStep = 1;
				elseif byte==string.byte("7") then  
					Motion.event("sit");
				elseif byte==string.byte("8") then	
					Motion.event("standup");
					print("standup")
				elseif byte==string.byte("9") then	
					Motion.event("walk");
					walk.start();
					print("walk")
				elseif byte==string.byte("0") then	
					if walk.active then walk.stop(); end
					print("stop")
					end
				else -- program input velocity
					local hasilParsing = data_body:split(",") --process pemisahan data velocity dengan header dan pemisahan sumbu X,Y,A
					-- print(hasilParsing)
					if hasilParsing[1]=="walk" then
						print(hasilParsing[2],hasilParsing[3],hasilParsing[4])
						walkX = actual_walk[1];
						walkY = actual_walk[2];
						walkA = actual_walk[3];
						selisih = walkX - 0.0;
--						selisihA = tonumber(hasilParsing[2]) - walkX;
						--print("actual walk	: ",actual_walk[1]);
						--print("hasil parsing	: ",hasilParsing[2]);
						--print("walk kine	: ",walkKine);
						--print("stepHeight	: ",walk.stepHeight);
						-----------Start Program Decelerasi "P" controller-----------
						if tonumber(hasilParsing[2]) == errorWalkX and dataAkhir ~= 0 then
						        walk.tStep = 0.28;
						        walk.tZmp = 0.120;
							if selisih > 0 then
									Error =  actual_walk[1] + errorWalkX;
							elseif selisih < 0 then
									Error =  actual_walk[1] - errorWalkX;		
							end
							Perror = Error * 0.40;
							dataAkhir = actual_walk[1] - Perror;
							if dataAkhir > 0.08 then
								dataAkhir = 0.08;
							elseif dataAkhir < -0.03 then
								dataAkhir = -0.03; end
							--print("Decel")
						-----------Start Program Accelerasi "P" controller-----------
						
						elseif walkX <= 0.02 and tonumber(hasilParsing[2]) > 0.06 then
						        walk.tStep = 0.28;
						        walk.tZmp = 0.120;
							if walkKine < 0.02 then
							        walk.tStep = 0.28;
							        walk.tZmp = 0.120;
							else						
							        walk.tStep = 0.30;
							        walk.tZmp = 0.140;
							end
--							walk.tStep = 0.26;
							Error =  tonumber(hasilParsing[2]) - actual_walk[1];
							Perror = Error * 0.1;
							dataAkhir = actual_walk[1] + Perror;
							if dataAkhir > 0.08 then
								dataAkhir = 0.08;
							elseif dataAkhir < -0.05 then
								dataAkhir = -0.05;  end
							--print("Accel")
						
						else
							dataAkhir = tonumber(hasilParsing[2]);
						end
						walkKine = tonumber(dataAkhir);

						if tonumber(walkKine) < errorWalkX then
							walk.hardnessSwing = 0.51;
						else
							walk.hardnessSwing = 0.59;
						end

						walk.set_velocity(tonumber(walkKine),tonumber(hasilParsing[3]),tonumber(hasilParsing[4]));
						Output = outputStart + ((outputEnd - outputStart) / (inputEnd - inputStart)) * (tonumber(walkKine) - inputStart); -- proses regresi stepHeight
						if walkKine > 0.02 then
						        walk.tStep = 0.30;
						        walk.tZmp = 0.140;
						else
						        walk.tStep = 0.28;
						        walk.tZmp = 0.120;
						end
						if walkY ~= 0 or walkA < 0 or walkA > errorWalkA then --jika erroWalkA bernilai positif
						--if walkY ~= 0 or walkA > 0 or walkA < errorWalkA then --jika errorWalkA bernilai negatif
							--walkKine = tonumber(walkKine) + 0.014;
							walk.stepHeight = 0.08;
						else
							if walkX < -0.008 then
								walk.stepHeight = 0.08;
							else 
								if (regres == false) then -- mode stepHeight tanpa regresi
									if walkX > -0.005 and walkX < 0.005 then 
										walk.stepHeight = 0.03;
										--print("0");
									elseif walkX > 0.005 and walkX < 0.015 then 
										walk.stepHeight = 0.036;
										--print("1");
									elseif walkX > 0.015 and walkX < 0.025 then 
										walk.stepHeight = 0.042;
										--print("2");
									elseif walkX > 0.025 and walkX < 0.035 then 
										walk.stepHeight = 0.048;
										--print("3");
									elseif walkX > 0.035 and walkX < 0.045 then 
										walk.stepHeight = 0.054;
										--print("4");
									elseif walkX > 0.045 and walkX < 0.055 then 
										walk.stepHeight = 0.060;
										--print("5");
									elseif walkX > 0.055 and walkX < 0.065 then 
										walk.stepHeight = 0.067;
										--print("6");
									elseif walkX > 0.065 and walkX < 0.075 then 
										walk.stepHeight = 0.073;
										--print("7");
									elseif walkX > 0.075 and walkX < 0.085 then 
										walk.stepHeight = 0.080;
										--print("8");
									end 
								else -- mode stepHeight regresi
									if walkKine > 0.2 then
										walk.stepHeight = 0.08;
--										walk.tStep = 26;
									else
										walk.stepHeight = Output;
--										walk.tStep = 28;
									end
								end
							end
						end
					end
				end
			end
	elseif data_body == nil then 
		data_body = "walk,0.00,0.00,0.00";
	--   print(data_body)
	else
		data_body = "walk,0.00,0.00,0.00";
	--   print("null");
	end	

	data_head, msg_or_ip_head, port_or_nil_head = udp_head:receivefrom() -- tampung data gerak kepala dari socket
	--start proses input head movement
	--print(stateFallCheck);
	if data_head then
		-- print(data_head)
		if (stateFallCheck == nil) then
		   stateFallCheck = 'fail';
		end
		local head_angle = data_head:split(",") --pemisahan data antara tilt dan pan (angguk dan geleng)
--			if (fallCheck == 1 and stateFallCheck == 'fail') then
--				Body.set_head_command(posHead);
				--print("fallCheck");
--			else
				Body.set_head_command({tonumber(head_angle[1]),tonumber(head_angle[2])});
				print(head_angle[1],head_angle[2])
				--print("Main");
--			end
	elseif msg_or_ip_head ~= 'timeout' then
	end	
end

-- main loop
count = 0;
lcount = 0;
tUpdate = unix.time();

function update()
  count = count + 1;
  if (not init)  then
    if (calibrating) then
      if (Body.calibrate(count)) then
        Speak.talk('Calibration done');
        calibrating = false;
        ready = true;
      end
    elseif (ready) then
      init = true;
    else
      if (count % 20 == 0) then
-- start calibrating w/o waiting
--        if (Body.get_change_state() == 1) then
          Speak.talk('Calibrating');
          calibrating = true;
--        end
      end
      -- toggle state indicator
      if (count % 100 == 0) then
        initToggle = not initToggle;
        if (initToggle) then
          Body.set_indicator_state({1,1,1}); 
        else
          Body.set_indicator_state({0,0,0});
        end
      end
    end
  else
    -- update state machines 
--    barelangfc_tunning();
    stateFallCheck = Motion.fallCheckState;
    fallCheck = Motion.fall;
    actual_walk = walk.get_velocity();
    process_keyinput();
    Motion.update();
    Body.update();
  end
  local dcount = 50;
  if (count % 50 == 0) then
--    print('fps: '..(50 / (unix.time() - tUpdate)));
    tUpdate = unix.time();
    -- update battery indicator
    Body.set_indicator_batteryLevel(Body.get_battery_level());
  end
  
  -- check if the last update completed without errors
  lcount = lcount + 1;
  if (count ~= lcount) then
    --print('count: '..count)
    --print('lcount: '..lcount)
    Speak.talk('missed cycle');
    lcount = count;
  end

  --Stop walking if button is pressed and the released
  if (Body.get_change_state() == 1) then
    button_pressed[1]=1;
  else
    if button_pressed[1]==1 then
      Motion.event("sit");
    end
    button_pressed[1]=0;
  end
end

-- if using Webots simulator just run update
if (webots) then
  while (true) do
    -- update motion process
    update();
    io.stdout:flush();
  end
end

--Now both nao and darwin runs this separately
if (darwin) or (newnao) then
  local tDelay = 0.005 * 1E6; -- Loop every 5ms
  while 1 do
    update();
    unix.usleep(tDelay);
  end
end
