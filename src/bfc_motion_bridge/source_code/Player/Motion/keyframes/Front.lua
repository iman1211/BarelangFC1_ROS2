local mot={};

mot.servos={
1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,};
mot.keyframes={  

--0
--for set
{
angles=vector.new({
0,-120,
140,60,-140,
0,0,-19,70,-90,0,  -- -95
0,0,-19,70,-90,0,  -- -95
140,-60,-140
})*math.pi/180,
duration = 0.3;
},

--1
--Arm set
{
angles=vector.new({
0,-120,
120,60,-140,
0,0,-19,70,-90,0,  -- -95
0,0,-19,70,-90,0,  -- -95
120,-60,-140
})*math.pi/180,
duration = 0.5;
},

--2
--arm pull
{
angles=vector.new({
0,-120,
60,60,-80,
0,0,-50,100,-90,0,
0,0,-50,100,-90,0,
60,-60,-80
})*math.pi/180,
duration = 0.4;
},

--3
--arm push
{
angles=vector.new({
0,-120,
30,60,-20,
0,0,-127,120,-70,0,  -- -95
0,0,-127,120,-70,0,  -- -95
30,-60,-20
})*math.pi/180,
duration = 0.5;
},

--3.5
-- After Push
{
angles=vector.new({
0,-120,
35,60,-0,
0,0,-127,135,-60,0,  -- -95
0,0,-127,135,-60,0,  -- -95
35,-60,-0
})*math.pi/180,
duration = 0.5;
},


{
angles=vector.new({
0,-120,
35,60,-0,
0,0,-127,130,-45,0,  -- -95
0,0,-127,130,-45,0,  -- -95
35,-60,-0
})*math.pi/180,
duration = 0.2;
},


--3.55
-- Tegak
{
angles=vector.new({
0,-120,
90,60,-90,
0,0,-60,110,-60,0,  -- -95
0,0,-60,110,-60,0,  -- -95
90,-60,-90
})*math.pi/180,
duration = 0.5;
},

--4
--Penstabilan
{
angles=vector.new({
0,-120,
90,60,-90,
0,0,-60,110,-60,0,
0,0,-60,110,-60,0,
90,-60,-90
})*math.pi/180,
duration = 0.4;
},

--5
--Final Step
{
angles=vector.new({
0,-100,
90,60,-20,
0,0,-46,70,-35,0,
0,0,-46,70,-35,0,
90,-60,-30
})*math.pi/180,
duration = 0.3;
},
};

return mot;
