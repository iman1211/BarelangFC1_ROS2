module(..., package.seeall);

require('Body')
require('walk')

t0 = 0;
timeout = Config.falling_timeout or 0.3;


--qLArmFront = vector.new({45,9,-135})*math.pi/180;
--qRArmFall = vector.new({45,-9,-135})*math.pi/180;
qLArmFall = vector.new({90,55,0})*math.pi/180;
qRArmFall = vector.new({90,-55,0})*math.pi/180;
qLLegFrontBack = vector.new({0,0,0,0,0,0})*math.pi/180;
qRLegFrontBack = vector.new({0,0,0,0,0,0})*math.pi/180;
qLLegFallRight = vector.new({0,20,0,0,0,0})*math.pi/180;
qRLegFallLeft = vector.new({0,-20,0,0,0,0})*math.pi/180;
headPos = vector.new({0,-120})*math.pi/180;
limitYmin = -25; -- -10
limitYmax = 40; -- 35
limitXmin = -35; -- -35
limitXmax = 35; -- 35

function entry()
  print(_NAME.." entry");

  -- relax all the joints while falling
--  Body.set_body_hardness(0);
  Body.set_body_hardness(0.7);


  --Ukemi motion (safe fall)
  local AngleY = Body.get_sensor_imuAngle(2);
  local AngleX = Body.get_sensor_imuAngle(1);
  imuAngleY = AngleY;
  imuAngleX = AngleX;
  if (imuAngleY > limitYmax or imuAngleY < limitYmin or imuAngleX > limitXmax or imuAngleX < limitXmin ) then --falling 
    --Body.set_larm_hardness({0.6,0,0.6});
    --Body.set_rarm_hardness({0.6,0,0.6});
    Body.set_head_hardness({0.7,0.7});
    Body.set_head_command(headPos);
    Body.set_larm_command(qLArmFall);
    Body.set_rarm_command(qRArmFall);
    --Body.set_lleg_command(qLLegFrontBack);
    --Body.set_rleg_command(qRLegFrontBack);
    if (imuAngleX > limitXmax) then
      print("FALL RIGHT")
      Body.set_head_command(headPos);
      Body.set_lleg_command(qLLegFallRight);
      Body.set_rleg_command(qRLegFrontBack);
    elseif (imuAngleX < limitXmin) then
      print("FALL LEFT")
      Body.set_head_command(headPos);
      Body.set_lleg_command(qLLegFrontBack);
      Body.set_rleg_command(qRLegFallLeft);
    elseif (imuAngleY > limitYmax) then
      print("FALL FRONT")
      Body.set_head_command(headPos);
      Body.set_lleg_command(qLLegFrontBack);
      Body.set_rleg_command(qRLegFrontBack);
    elseif (imuAngleY < limitYmin) then
      print("FALL BACK")
      Body.set_head_command(headPos);
      Body.set_lleg_command(qLLegFrontBack);
      Body.set_rleg_command(qRLegFrontBack);
    end
--  if (imuAngleY > 0) then --Front falling 
--print("UKEMI FRONT")
--    Body.set_larm_hardness({0.6,0,0.6});
--    Body.set_rarm_hardness({0.6,0,0.6});
--    Body.set_larm_command(qLArmFront);
--    Body.set_rarm_command(qRArmFront);
  else
  end

--[[
  --Ukemi motion (safe fall)
  local imuAngleY = Body.get_sensor_imuAngle(2);
  if (imuAngleY > 0) then --Front falling 
print("UKEMI FRONT")
    Body.set_larm_hardness({0.6,0,0.6});
    Body.set_rarm_hardness({0.6,0,0.6});
    Body.set_larm_command(qLArmFront);
    Body.set_rarm_command(qRArmFront);
  else
  end
--]]

  t0 = Body.get_time();
  Body.set_syncread_enable(1); --OP specific
  walk.stance_reset();--reset current stance
end

function update()
  local t = Body.get_time();
  -- set the robots command joint angles to thier current positions
  --  this is needed to that when the hardness is re-enabled
  if (t-t0 > timeout) then
    return "done"
  end
end

function exit()
  local qSensor = Body.get_sensor_position();
  Body.set_actuator_command(qSensor);
end
