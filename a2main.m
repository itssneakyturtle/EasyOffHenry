classdef a2main < handle    
    properties (Constant)
        base = transl(0,0,0.1185);
        workspace = [-1.2 1.2 -1.2 1.2 -0.02 1];
    end
    
    properties
        cyton;
        controlBoardLoc;
        robotBaseLoc;
        cytonBaseLoc;
        safetyConeLoc;
        showerLoc;
        irSensorLoc;
        TLShowerLoc;
        TMShowerLoc;
        TRShowerLoc;
        BLShowerLoc;
        BMShowerLoc;
        BRShowerLoc;
        MLShowerLoc;
        MMShowerLoc;
        MRShowerLoc;
        waypoint;
    end
    
    methods
        function self = a2main()
            hold on;
            self.cyton = Cyton('cyton',self.base,self.workspace);
            %% Set up variables
            self.controlBoardLoc = self.base * transl([0.5 0.5 0.2]);
            self.robotBaseLoc = transl([0 0 0.078]);
            self.cytonBaseLoc = transl([0 0 0.028]);
            self.showerLoc = self.base * transl([-0.3 0 0.15]);
            self.irSensorLoc = self.base * transl([-0.2 0 0.3]);
            
            %% Setup Environment
            loadPly('ply\control.ply', self.controlBoardLoc);
            loadPly('ply\RobotBase.ply',self.robotBaseLoc);
            loadPly('ply\cytonbase.ply',self.cytonBaseLoc);
%             loadPly('ply\safetycone.ply',self.safetyConeLoc);
            loadPly('ply\glass.ply',self.showerLoc);
            loadPly('ply\irsensor.ply',self.irSensorLoc);
            
            %% glass location
            self.TLShowerLoc = transl(-0.25,-0.1,0.35)*trotx(pi)*troty(-pi/2)*trotz(-pi/2);
            self.TMShowerLoc = transl(-0.25,0,0.35)*trotx(pi)*troty(-pi/2)*trotz(-pi/2);
            self.TRShowerLoc = transl(-0.25,0.1,0.35)*trotx(pi)*troty(-pi/2)*trotz(-pi/2);
            self.BLShowerLoc = transl(-0.25,-0.1,0.1)*trotx(pi)*troty(-pi/2)*trotz(-pi/2);
            self.BMShowerLoc = transl(-0.25,0,0.1)*trotx(pi)*troty(-pi/2)*trotz(-pi/2);
            self.BRShowerLoc = transl(-0.25,0.1,0.1)*trotx(pi)*troty(-pi/2)*trotz(-pi/2);
            self.MLShowerLoc = transl(-0.25,-0.1,0.225)*trotx(pi)*troty(-pi/2)*trotz(-pi/2);
            self.MMShowerLoc = transl(-0.25,0,0.225)*trotx(pi)*troty(-pi/2)*trotz(-pi/2);
            self.MRShowerLoc = transl(-0.25,0.1,0.225)*trotx(pi)*troty(-pi/2)*trotz(-pi/2);
            self.waypoint = transl(-0.1,0,0.3)*trotx(pi)*troty(-pi/2)*trotz(-pi/2);
            
        end
        %% Function to update all joint angles when sliders are used
        function updateJoint(self,joint,angle)
            qNew = self.cyton.model.getpos();
            qNew(1,joint) = deg2rad(angle);
            self.cyton.model.animate(qNew)
        end
        %% update xyz rpy
        function [xyz,rpy] = updateEndEffectorPos(self)
            q = self.cyton.model.getpos();
            xyz = self.cyton.model.fkine(q);
            rpy = tr2rpy(xyz,'deg');
        end
        %% Function to identify path for cleaning
        function Clean(self)
            p = self.getendpos(self.TLShowerLoc);
            self.MoveTo(p);
            p = self.getendpos(self.MLShowerLoc);
            self.MoveTo(p);
            p = self.getendpos(self.BLShowerLoc);
            self.MoveTo(p);
            p = self.getendpos(self.waypoint);
            self.MoveTo(p);
            p = self.getendpos(self.TMShowerLoc);
            self.MoveTo(p);
            p = self.getendpos(self.MMShowerLoc);
            self.MoveTo(p);
            p = self.getendpos(self.BMShowerLoc);
            self.MoveTo(p);
            p = self.getendpos(self.waypoint);
            self.MoveTo(p);
            p = self.getendpos(self.TRShowerLoc);
            self.MoveTo(p);
            p = self.getendpos(self.MRShowerLoc);
            self.MoveTo(p);
            p = self.getendpos(self.BRShowerLoc);
            self.MoveTo(p);
            p = self.getendpos(self.waypoint);
            self.MoveTo(p);

% left to right
%             p = self.getendpos(self.TRShowerLoc);
%             self.MoveTo(p);
% %             p = self.getendpos(self.waypoint);
% %             self.MoveTo(p);
%             p = self.getendpos(self.BLShowerLoc);
%             self.MoveTo(p);
%             p = self.getendpos(self.BRShowerLoc);
%             self.MoveTo(p);            
        end
        %% Function to move robot to desired location and candy if conditions are met
        function p = getendpos(self, pend)
             qGuess = [0,pi/4,pi/2,pi/4,0,pi,pi]; % Guessed joint positions for ikcon
            p = self.cyton.model.ikcon(pend,qGuess);
        end
        
        %% 
        function MoveTo(self, qend) 
            steps = 50;
            qcurrent = self.cyton.model.getpos(); % Get current joint angles
%             newQ = self.cyton.model.ikcon(qend); % Updated joint position for desired end effector location
            qMatrix = TrapProfile(qcurrent,qend,steps); % Use Trapezoidal Profile Method to obtain qMatrix
            for i = 1:1:steps
                self.cyton.model.animate(qMatrix(i,:));
                drawnow;
            end
        end
    end
end

