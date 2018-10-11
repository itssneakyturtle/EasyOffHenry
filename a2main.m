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
        T1ShowerLoc;
        T2ShowerLoc;
        T3ShowerLoc;
        T4ShowerLoc;
        T5ShowerLoc;
        M1ShowerLoc;
        M2ShowerLoc;
        M3ShowerLoc;
        M4ShowerLoc;
        M5ShowerLoc;
        waypoint;
        qMatrix;
        mybum;
        estopflag;
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
            loadPly('ply/control.ply', self.controlBoardLoc);
            loadPly('ply/RobotBase.ply',self.robotBaseLoc);
            loadPly('ply/cytonbase.ply',self.cytonBaseLoc);
%             loadPly('ply\safetycone.ply',self.safetyConeLoc);
            loadPly('ply/glass.ply',self.showerLoc);
            loadPly('ply/irsensor.ply',self.irSensorLoc);
            
            %% glass cleaning locations
            self.T1ShowerLoc = transl(-0.3,-0.1,0.35)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.T2ShowerLoc = transl(-0.3,-0.05,0.35)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.T3ShowerLoc = transl(-0.3,0,0.35)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.T4ShowerLoc = transl(-0.3,0.05,0.35)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.T5ShowerLoc = transl(-0.3,0.1,0.35)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            
            self.M1ShowerLoc = transl(-0.3,-0.1,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.M2ShowerLoc = transl(-0.3,-0.05,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.M3ShowerLoc = transl(-0.3,0,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.M4ShowerLoc = transl(-0.3,0.05,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.M5ShowerLoc = transl(-0.3,0.1,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            
            self.waypoint = transl(-0.1,-0.1,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            
%             self.T1ShowerLoc = transl(-0.1,-0.25,0.35)*trotx(pi)*troty(-pi/2)*trotz(-pi/2);
%             self.T2ShowerLoc = transl(-0.05,-0.3,0.35)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
%             self.T3ShowerLoc = transl(0,-0.3,0.35)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
%             self.T4ShowerLoc = transl(0.05,-0.3,0.35)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
%             self.T5ShowerLoc = transl(0.1,-0.25,0.35)*trotx(pi)*troty(-pi/2)*trotz(-pi/2);
%             
%             self.M1ShowerLoc = transl(-0.1,-0.25,0.3)*trotx(pi)*troty(-pi/2)*trotz(-pi/2);
%             self.M2ShowerLoc = transl(-0.05,-0.,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
%             self.M3ShowerLoc = transl(0,-0.3,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
%             self.M4ShowerLoc = transl(0.05,-0.3,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
%             self.M5ShowerLoc = transl(0.1,-0.25,0.3)*trotx(pi)*troty(-pi/2)*trotz(-pi/2);
%             
%             self.waypoint = transl(0,-0.1,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            
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
        function [a,b,c,d,e,f,g,h,i,j,k] = Clean(self)
            steps = 50;
            
            q = self.getendpos(self.T1ShowerLoc);
            a = self.MoveTo(q);
            
            [startTransl,endTransl,startRPY,endRPY] = self.updateLocs(self.T2ShowerLoc);
            b = self.RMRC(startTransl,endTransl,startRPY,endRPY,steps);
            
            [startTransl,endTransl,startRPY,endRPY] = self.updateLocs(self.T3ShowerLoc);
            c= self.RMRC(startTransl,endTransl,startRPY,endRPY,steps);
            
            [startTransl,endTransl,startRPY,endRPY] = self.updateLocs(self.T4ShowerLoc);
            d=self.RMRC(startTransl,endTransl,startRPY,endRPY,steps);
            
            [startTransl,endTransl,startRPY,endRPY] = self.updateLocs(self.T5ShowerLoc);
            e = self.RMRC(startTransl,endTransl,startRPY,endRPY,steps);
            
            p = self.getendpos(self.waypoint);
            f = self.MoveTo(p);
            
            [startTransl,endTransl,startRPY,endRPY] = self.updateLocs(self.M1ShowerLoc);
            g = self.RMRC(startTransl,endTransl,startRPY,endRPY,steps);
            
            [startTransl,endTransl,startRPY,endRPY] = self.updateLocs(self.M2ShowerLoc);
            h = self.RMRC(startTransl,endTransl,startRPY,endRPY,steps);
            
            [startTransl,endTransl,startRPY,endRPY] = self.updateLocs(self.M3ShowerLoc);
            i = self.RMRC(startTransl,endTransl,startRPY,endRPY,steps);
            
            [startTransl,endTransl,startRPY,endRPY] = self.updateLocs(self.M4ShowerLoc);
            j = self.RMRC(startTransl,endTransl,startRPY,endRPY,steps);
            
            [startTransl,endTransl,startRPY,endRPY] = self.updateLocs(self.M5ShowerLoc);
            k = self.RMRC(startTransl,endTransl,startRPY,endRPY,steps);
                      
        end
        %% Function to move robot to desired location and candy if conditions are met
        function p = getendpos(self, pend)
             qGuess = [0,pi/4,pi/2,pi/4,0,pi,pi]; % Guessed joint positions for ikcon
            p = self.cyton.model.ikcon(pend,qGuess);
        end
        %% 
        function qMatrix = MoveTo(self, qend) 
            steps = 50;
            qcurrent = self.cyton.model.getpos(); % Get current joint angles
            qMatrix = TrapProfile(qcurrent,qend,steps); % Use Trapezoidal Profile Method to obtain qMatrix
%             for i = 1:1:steps
%                 if self.estopflag ~= 0
%                     while self.estopflag ~= 0
%                         pause(0.1);
%                     end
%                 end
%                 self.cyton.model.animate(qMatrix(i,:));
%                 drawnow;
%             end
        end
        %% update next location for RMRC
        function [startTransl,endTransl,startRPY,endRPY] = updateLocs(self,nextLoc)
            qCurrent = self.cyton.model.getpos();
            startTransl = self.cyton.model.fkine(qCurrent);
            endTransl = nextLoc;
            startRPY = tr2rpy(startTransl(1:3,1:3));
            endRPY = tr2rpy(endTransl(1:3,1:3));
        end
        %% RMRC
         function qMatrix = RMRC(self,startTransl,endTransl,startRPY,endRPY,steps)
            startXYZ = startTransl(1:3,4)';
            endXYZ = endTransl(1:3,4)';

            t = 5;                              % Total time
            deltaT = t/steps;                   % Discrete time step
            delta = 2*pi/steps;                 % Small angle change
            epsilon = 0.05;                     % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1 0.1 0.1 0.1]);      % Weighting matrix for the velocity vector

            % Allocate array data
            m = zeros(steps,1);                 % Array for Measure of Manipulability
            qMatrix = zeros(steps,7);           % Array for joint angles
            qdot = zeros(steps,7);              % Array for joint velocities
            theta = zeros(3,steps);             % Array for roll-pitch-yaw angles
            x = zeros(3,steps);                 % Array for x-y-z trajectory
            positionError = zeros(3,steps);     % For plotting trajectory error
            angleError = zeros(3,steps);        % For plotting trajectory error

            % Trajectory setup
            s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
            for i=1:steps
                x(1,i) = (1-s(i))*startXYZ(1,1) + s(i)*endXYZ(1,1);     % X direction movement
                x(2,i) = (1-s(i))*startXYZ(1,2) + s(i)*endXYZ(1,2);     % Y direction movement
                x(3,i) = (1-s(i))*startXYZ(1,3) + s(i)*endXYZ(1,3);     % Z direction movement
                theta(1,i) = (1-s(i))*startRPY(1) + s(i)*endRPY(1);     % Roll angle 
                theta(2,i) = (1-s(i))*startRPY(2) + s(i)*endRPY(2);     % Pitch angle
                theta(3,i) = (1-s(i))*startRPY(3) + s(i)*endRPY(3);     % Yaw angle
            end

            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];   % Transformation of first point and angle
            q0 = zeros(1,7);
            qMatrix(1,:) = self.cyton.model.getpos();% self.model.ikcon(T,q0);                               % First waypoint
            % Track the trajectory with RMRC
            for i = 1:steps-1
                T = self.cyton.model.fkine(qMatrix(i,:));                     % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                           % Change to next position from current position
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                            % Calculate rotation matrix error
                S = Rdot*Ra';                                           % Skew symmetric equation rearranged
                linear_velocity = (1/deltaT)*deltaX;        
                angular_velocity = [S(3,2);S(1,3);S(2,1)];              % Extract velocities from skew symmetric
                deltaTheta = tr2rpy(Rd*Ra');                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];            % Calculate end-effector velocity to reach next waypoint.
                J = self.cyton.model.jacob0(qMatrix(i,:));                    % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));                                 % maniputability
                if m(i) < epsilon                                       % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-3;                   % Damping coefficient for Damped Least Squares
                else
                    lambda = 0;                                         % dont use Damped Least Squares
                end
                invJ = inv(J'*J + lambda *eye(7))*J';                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                               % Solve the RMRC equation
                for j = 1:7                                             % Loop through joints 1 to 7
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.cyton.model.qlim(j,1)       % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0;                                              % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.cyton.model.qlim(j,2)   % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0;                                              % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);       % Update next joint state based on joint velocities

                positionError(:,i) = deltaX;                            % For plotting
                angleError(:,i) = deltaTheta;                           % For plotting
            end
%             for i = 1:steps
%                 if self.estopflag ~= 0
%                     while self.estopflag ~= 0
%                         pause(0.1);
%                     end
%                 end
%                     self.cyton.model.animate(qMatrix(i,:));
%                     [a,b] = self.updateEndEffectorPos();
%                     
%                     drawnow;                    
%             end
            % Plotting movement
    %             figure(1)
    %             plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
    %             
    %             if nargin == 5                                  %if no part supplied, move robot only
    %                 for i = 1:steps
    %                     self.model.animate(qMatrix(i,:));
    %                     % add while STOP is pressed ....
    %                 end
    %             end
    %             
    %             
    %             %controls both robot and part
    %             if nargin == 6
    %                 for i = 1:steps
    %                     self.model.animate(qMatrix(i,:));
    %                     varargin{1}.model.base = self.model.fkine(qMatrix(i,:))*trotx(pi/2);
    %                     varargin{1}.model.animate(0);
    %                     % add while STOP is pressed....
    %                 end
    %             end
    %         end
         end
         function sim_cytonanimate(self,qMatrix)
             self.cyton.model.animate(qMatrix);
             drawnow;
         end
         
    end
end
