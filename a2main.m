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
        cagefrontLoc;
        cagebackLoc;
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
        waypoint1;
        waypoint2;
        waypoint3;
        qMatrix;
        mybum;
        estopflag;
        vertex;
        faces;
        faceNormals;
        prismCenterpnt;
    end
    
    methods
        function self = a2main()
            hold on;
            self.cyton = Cyton('cyton',self.base,self.workspace);
            
            % Set up variables
            self.controlBoardLoc = self.base * transl([1 0.5 0.075]);
            self.robotBaseLoc = transl([0 0 0.078]);
            self.cytonBaseLoc = transl([0 0 0.028]);
            self.showerLoc = self.base * transl([-0.3 0 0.15]);
            self.irSensorLoc = self.base * transl([-0.1 0.7 0])*trotz(pi/2);
            self.cagefrontLoc = self.base * transl([-0.2 -0.5 0.3]);
            self.cagebackLoc = self.base * transl([0 0.5 0.3]);
            
            % Setup Environment
            loadPly('ply/control.ply', self.controlBoardLoc);
            loadPly('ply/RobotBase.ply',self.robotBaseLoc);
            loadPly('ply/cytonbase.ply',self.cytonBaseLoc);
            loadPly('ply/glass.ply',self.showerLoc);
            loadPly('ply/IR.ply',self.irSensorLoc);
            loadPly('ply/cagefront.ply',self.cagefrontLoc);
            loadPly('ply/cageback.ply',self.cagebackLoc);
            
            % glass cleaning locations
            % top 
            self.T1ShowerLoc = transl(-0.3,-0.1,0.35)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.T2ShowerLoc = transl(-0.3,-0.05,0.35)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.T3ShowerLoc = transl(-0.3,0,0.35)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.T4ShowerLoc = transl(-0.3,0.05,0.35)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.T5ShowerLoc = transl(-0.3,0.1,0.35)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            % mid
            self.M1ShowerLoc = transl(-0.3,-0.1,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.M2ShowerLoc = transl(-0.3,-0.05,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.M3ShowerLoc = transl(-0.3,0,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.M4ShowerLoc = transl(-0.3,0.05,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.M5ShowerLoc = transl(-0.3,0.1,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            % waypoint
            self.waypoint1 = transl(-0.1,0.1,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.waypoint2 = transl(-0.1,0,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            self.waypoint3 = transl(-0.1,-0.1,0.3)*trotx(-pi/2)*troty(-pi/2)*trotz(-pi/2);
            
            % rectangular prism for collsion checking 
            self.prismCenterpnt = [-0.5,0,0.25];
            side = [0.4 0.6 0.5];
            plotOptions.plotFaces = true;
            [self.vertex,self.faces,self.faceNormals] = RectangularPrism(self.prismCenterpnt-side/2, self.prismCenterpnt+side/2,plotOptions);
            
        end

        %% Function to update all joint angles when sliders are used
        function qNew = updateJoint(self,joint,inputangle)
            qNew = self.cyton.model.getpos();
            qNew(1,joint) = deg2rad(inputangle);
%             self.cyton.model.animate(qNew)
        end
        %% update xyz rpy
        function [xyz,rpy] = updateEndEffectorPos(self)
            q = self.cyton.model.getpos();
            xyz = self.cyton.model.fkine(q);
            rpy = tr2rpy(xyz,'deg');
        end

        %% Function to move robot to desired location and candy if conditions are met
        function p = getendpos(self, pend)
             qGuess = [0,pi/4,pi/2,pi/4,0,pi,pi]; % Guessed joint positions for ikcon
            p = self.cyton.model.ikcon(pend,qGuess);
        end
        %% jtraj qMatrix movement setup
        function qMatrix = MoveTo(self, qend) 
            steps = 25;
            qcurrent = self.cyton.model.getpos(); % Get current joint angles
            qMatrix = TrapProfile(qcurrent,qend,steps); % Use Trapezoidal Profile Method to obtain qMatrix
        end
        %% update next location for RMRC
        function [startTransl,endTransl,startRPY,endRPY] = updateLocs(self,nextLoc)
            qCurrent = self.cyton.model.getpos();
            startTransl = self.cyton.model.fkine(qCurrent);
            endTransl = nextLoc;
            startRPY = tr2rpy(startTransl(1:3,1:3));
            endRPY = tr2rpy(endTransl(1:3,1:3));
        end
        %% RMRC qMatrix movement setup
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
         end
        %% animate cyton          
        function sim_cytonanimate(self,qMatrix)
            self.cyton.model.animate(qMatrix);
            drawnow;
        end
        %% get joint angle positions (rads)
        function qCurrent = getCurrentPos(self)
            qCurrent = self.cyton.model.getpos();
        end 
        %% fkine cyton
        function endPos = getfkine(self,q)
            endPos = self.cyton.model.fkine(q);
        end
        %% ikcon cyton
        function newq = getikcon(self,endPos)
            newq = self.cyton.model.ikcon(endPos);
        end 
%% add function for collision checking
%% determine joint locations/transforms
function [ transforms ] = GetLinkPoses(self,q)

links = self.cyton.link;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = self.cyton.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(self,intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(self,qMatrix,prismfaces,prismvertex,prismfaceNormals,returnOnceFound)
if nargin < 5
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = self.GetLinkPoses(qMatrix(qIndex,:));

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(prismfaces,1)
            vertOnPlane = prismvertex(prismfaces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(prismfaceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && self.IsIntersectionPointInsideTriangle(intersectP,prismvertex(prismfaces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Intersection');
                result = true;
%                 if returnOnceFound
%                     return
%                 end
            end
        end    
    end
end

% add waypoints by determining cartesian points that the end effector could
% follow


end

         
    end
end
