classdef a2main < handle    
    properties (Constant)
        base = transl(0,0,0.1);
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
    end
    
    methods
        function self = a2main()
            hold on;
            self.cyton = Cyton('cyton',self.base,self.workspace);
            %% Set up variables
            self.controlBoardLoc = self.base * transl([0.5 0.5 0.2]);
            self.robotBaseLoc = self.base * transl([0 0 0.1]);
            self.cytonBaseLoc = self.base * transl([0 0 0.5]);
            self.showerLoc = self.base * transl([-0.1 0 0.3]);
            self.irSensorLoc = self.base * transl([-0.2 0 0.3]);
            
            %% Setup Environment
            loadPly('ply\control.ply', self.controlBoardLoc);
            loadPly('ply\RobotBase.ply',self.robotBaseLoc);
            loadPly('ply\cytonbase.ply',self.cytonBaseLoc);
%             loadPly('ply\safetycone.ply',self.safetyConeLoc);
            loadPly('ply\shower.ply',self.showerLoc);
            loadPly('ply\irsensor.ply',self.irSensorLoc);
            

        end
    end
end

