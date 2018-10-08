clf;
clc;
L1 = Link('d',0.053,'a',0,'alpha',pi/2,'offset', deg2rad(-85),'qlim',[deg2rad(-150),deg2rad(150)]);
L2 = Link('d',0,'a',0,'alpha',-pi/2,'offset', 0,'qlim',[deg2rad(-105),deg2rad(105)]);
L3 = Link('d',0.128,'a',0,'alpha',pi/2,'offset', 0,'qlim',[deg2rad(-150),deg2rad(150)]);
L4 = Link('d',0,'a',0.065,'alpha',-pi/2,'offset', pi/2,'qlim',[deg2rad(-105),deg2rad(105)]);
L5 = Link('d',0,'a',0.068,'alpha',pi/2,'offset', 0,'qlim',[deg2rad(-105),deg2rad(105)]);
L6 = Link('d',0,'a',0,'alpha',-pi/2,'offset', -pi/2,'qlim',[deg2rad(-105),deg2rad(105)]);
L7 = Link('d',0.17,'a',0,'alpha',0,'offset', -pi/2,'qlim',[deg2rad(-150),deg2rad(150)]);

cyton = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name','cyton');
cyton.base = transl(0,0,0);


workspace = [-0.5 0.5 -0.5 0.5 -0.5 0.5];
scale = 0.4;

q = zeros(1,7);
steps = 50;

cyton.plot(q,'workspace',workspace,'scale',scale,'trail','r-');

% r1 = [-pi,0,0,-pi/2,0,0,0];
% r2 = [pi,0,0,-pi/2,0,0,0];
% 
% trajA = trajectory(r1,r2,steps);

cyton.teach;
% cyton.animate(trajA);
q = cyton.getpos();
T = cyton.fkine(q);
RPY = tr2rpy(T,'deg');
