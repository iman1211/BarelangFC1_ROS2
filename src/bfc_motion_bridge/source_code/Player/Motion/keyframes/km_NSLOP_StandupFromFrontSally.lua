local mot={};

mot.servos={
1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,};
mot.keyframes={  

--1
--Rotate arm
{
angles=vector.new({
0,0,
90,90,-22,
1,-3,-15,24,-60,0,
-1,3,-15,24,-60,0,
90,-90,-22
})*math.pi/180,
duration = 0.3;
},

--2
--arm up
{
angles=vector.new({
0,0,
-70,90,-22,
1,-3,-15,24,-60,0,
-1,3,-15,24,-60,0,
-70,-90,-22
})*math.pi/180,
duration = 0.9;--.9
},

--3
--arm close
{
angles=vector.new({
0,0,
-70,5,-22,

--1,-3,-15,24,-60,0,
---1,3,-15,24,-60,0,
0,0,-19,70,-80,0,  -- -95
0,0,-19,70,-80,0,  -- -95
-70,-5,-22
})*math.pi/180,
duration = 0.9; --.3
},

--4
--arm pull
{
angles=vector.new({
0,0,
50,5,-55,
0,0,-50,100,-80,0,
0,0,-50,100,-80,0,
50,-5,-55
})*math.pi/180,
duration = 0.9;--.7
},

--5
--Arm push
{
angles=vector.new({
0,0,
35,0,-0,
0,3,-110,110,-70,-3,
0,-3,-110,110,-70,3,
35,-0,-0

})*math.pi/180,
duration = 0.9;--.2
},

--6
{
angles=vector.new({
0,0,
35,0,-0,
0,3,-110,130,-60,-3,
0,-3,-110,130,-60,3,
35,-0,-0
})*math.pi/180,
duration = 0.9;--.2
},

--7
--This is the final pose of bodySit
{
angles=vector.new({
0,0,
90,5,-90,
0,3,-60,110,-59,-3,
0,-3,-60,110,-59,3,
90,-5,-90,

})*math.pi/180,
duration = 0.9; --.3
},

--7
--This is the final pose of bodySit
{
angles=vector.new({
0,0,
90,10,-90,
0,3,-60,110,-59,-3,
0,-3,-60,110,-59,3,
90,-10,-90,

})*math.pi/180,
duration = 0.9;--.3
},

--8
--This is the final pose of bodySit
{
angles=vector.new({
0,0,
90,15,-90,
0,3,-46,70,-35,-3,
0,-3,-46,70,-35,3,
90,-15,-90,

})*math.pi/180,
duration = 0.9;--.5
},

};

return mot;

