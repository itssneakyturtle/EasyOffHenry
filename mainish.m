classdef mainish
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot
        things
    end
    
    methods
        function self = mainish()
        self.robot = Cyton();   
        end
        function updateJoint(self,joint,angle)
            qNew = self.robot.model.getpos();
            qNew(1,joint) = deg2rad(angle);
            self.robot.model.animate(qNew)
        end
    end
end

