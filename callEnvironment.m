classdef callEnvironment < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    methods
    
    methods(Static)
        function self = callEnvironment()
            self.controlBoard();
            self.RobotBase();
            self.cytonBase();
            self.safetyCone();
            self.shower();
            self.irsensor();
            
        end
<<<<<<< HEAD
=======
<<<<<<< HEAD

=======
<<<<<<< HEAD
        %% 
>>>>>>> 222700ad29aee76d165e873bb0226096a63ed931
        function controlBoard(self)

        
>>>>>>> 984f052215e90675fd51ec8ad9ddc3adb9f6210c
        function controlBoard()
>>>>>>> 8ab4d9ffefdeed697f080cd14a92e252cef01882
            [f,v,data] = plyread('control.ply','tri');
            % Get vertex count
            VertexCount = size(v,1);
            % Move center point to origin
            midPoint = sum(v)/VertexCount;
            Verts = v - repmat(midPoint,VertexCount,1);
            % Create a transform to describe the location (at the origin, since it's centered
%             pose = eye(4);
            % model offsets
            zOffset = 0.2;
            yOffset = 0.5;
            xOffset = 0.5;
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Then plot the trisurf
            Mesh_h = trisurf(f,Verts(:,1)+xOffset,Verts(:,2)+yOffset, Verts(:,3)+zOffset ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
        %% 
        function RobotBase(self)
            [f,v,data] = plyread('RobotBase.ply','tri');
            % Get vertex count
            VertexCount = size(v,1);
            % Move center point to origin
            midPoint = sum(v)/VertexCount;
            Verts = v - repmat(midPoint,VertexCount,1);
            % Create a transform to describe the location (at the origin, since it's centered
%             pose = eye(4);
            % model offsets
            zOffset = 0.1;
            yOffset = 0;
            xOffset = 0;
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Then plot the trisurf
            Mesh_h = trisurf(f,Verts(:,1)+xOffset,Verts(:,2)+yOffset, Verts(:,3)+zOffset ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
        %% 
        function cytonBase(self)
            [f,v,data] = plyread('cytonbase.ply','tri');
            % Get vertex count
            VertexCount = size(v,1);
            % Move center point to origin
            midPoint = sum(v)/VertexCount;
            Verts = v - repmat(midPoint,VertexCount,1);
            % Create a transform to describe the location (at the origin, since it's centered
%             pose = eye(4);
            % model offsets
            zOffset = 0.05;
            yOffset = 0;
            xOffset = 0;
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Then plot the trisurf
            Mesh_h = trisurf(f,Verts(:,1)+xOffset,Verts(:,2)+yOffset, Verts(:,3)+zOffset ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
        %% 
        function safetyCone(self)
            [f,v,data] = plyread('safetycone.ply','tri');
            % Get vertex count
            VertexCount = size(v,1);
            % Move center point to origin
            midPoint = sum(v)/VertexCount;
            Verts = v - repmat(midPoint,VertexCount,1);
            % Create a transform to describe the location (at the origin, since it's centered
%             pose = eye(4);
            % model offsets
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            zOffset = 0;
            for xOffset = [-1, 1]
                for yOffset = [-1, 1]
                    % Then plot the trisurf with offset verticies
                    trisurf(f,Verts(:,1)+xOffset,Verts(:,2)+yOffset, Verts(:,3)+zOffset ...
                        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
                end
            end
        end
        %% 
        function shower(self)
            [f,v,data] = plyread('glass.ply','tri');
            % Get vertex count
            VertexCount = size(v,1);
            % Move center point to origin
            midPoint = sum(v)/VertexCount;
            Verts = v - repmat(midPoint,VertexCount,1);
            % Create a transform to describe the location (at the origin, since it's centered
%             pose = eye(4);
            % model offsets
            zOffset = 0.3;
            yOffset = 0;
            xOffset = -0.1;
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Then plot the trisurf
            Mesh_h = trisurf(f,Verts(:,1)+xOffset,Verts(:,2)+yOffset, Verts(:,3)+zOffset ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
        %% 
        function irsensor(self)
            [f,v,data] = plyread('irsensor.ply','tri');
            % Get vertex count
            VertexCount = size(v,1);
            % Move center point to origin
            midPoint = sum(v)/VertexCount;
            Verts = v - repmat(midPoint,VertexCount,1);
            % Create a transform to describe the location (at the origin, since it's centered
%             pose = eye(4);
            % model offsets
            zOffset = 0.3;
            yOffset = 0;
            xOffset = -0.2;
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Then plot the trisurf
            Mesh_h = trisurf(f,Verts(:,1)+xOffset,Verts(:,2)+yOffset, Verts(:,3)+zOffset ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
    end
end

