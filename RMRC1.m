        function RMRC(self,startTransl,endTransl,RPY,steps)
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
                theta(1,i) = RPY(1,1);                                  % Roll angle
                theta(2,i) = RPY(1,2);                                  % Pitch angle   
                theta(3,i) = RPY(1,3);                                  % Yaw angle
            end
            
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];   % Transformation of first point and angle
            q0 = zeros(1,7);
            qMatrix(1,:) = self.model.getpos();% self.model.ikcon(T,q0);                               % First waypoint
            % Track the trajectory with RMRC
            for i = 1:steps-1
                T = self.model.fkine(qMatrix(i,:));                     % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                           % Change to next position from current position
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                            % Calculate rotation matrix error
                S = Rdot*Ra';                                           % Skew symmetric equation rearranged
                linear_velocity = (1/deltaT)*deltaX;        
                angular_velocity = [S(3,2);S(1,3);S(2,1)];              % Extract velocities from skew symmetric
                deltaTheta = tr2rpy(Rd*Ra');                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];            % Calculate end-effector velocity to reach next waypoint.
                J = self.model.jacob0(qMatrix(i,:));                    % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));                                 % maniputability
                if m(i) < epsilon                                       % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-3;                   % Damping coefficient for Damped Least Squares
                else
                    lambda = 0;                                         % dont use Damped Least Squares
                end
                invJ = inv(J'*J + lambda *eye(7))*J';                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                               % Solve the RMRC equation
                for j = 1:7                                             % Loop through joints 1 to 7
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.model.qlim(j,1)       % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0;                                              % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.model.qlim(j,2)   % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0;                                              % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);       % Update next joint state based on joint velocities
                
                positionError(:,i) = deltaX;                            % For plotting
                angleError(:,i) = deltaTheta;                           % For plotting
            end
            
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
