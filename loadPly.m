function loadPly(name, transform) % Load ply models in desired location and orientation
[f,v,data] = plyread(name,'tri');

VertexCount = size(v,1);

midPoint = sum(v)/VertexCount;
Verts = v - repmat(midPoint,VertexCount,1);

vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

Mesh = trisurf(f,Verts(:,1),Verts(:,2), Verts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

newloc = [transform * [Verts,ones(VertexCount,1)]']'; 
Mesh.Vertices = newloc(:,1:3);

end
