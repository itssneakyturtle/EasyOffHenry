classdef callEnvironment < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods(Static)
        %% 
        function self = callEnvironment()
            self.controlBoard();
            self.RobotBase();
            self.cytonBase();
            self.safetyCone();
            self.shower();
            self.irsensor();
            
        end

        function controlBoard()
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
        function RobotBase()
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
        function cytonBase()
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
        function safetyCone()
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
        function shower()
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
        function irsensor()
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

