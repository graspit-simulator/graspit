%function ok=vrmlgeneration(fileprefix)
%palmgeneration

  fileprefix='palm';

  pointFile=strcat(fileprefix,'Points.txt');
  [a,x,y,z]=textread(pointFile);
  disp(sprintf('Loaded point data from file %s.\n',pointFile))
  
  triangleFile=strcat(fileprefix,'Triangles.txt');
  [pt1, pt2, pt3]=textread(triangleFile);
  disp(sprintf('Loaded triangle data from file %s.\n',triangleFile))
  
  noOfPts=length(a);
  noOfFaces=length(pt1);
  
  disp(sprintf('%d points and %d faces have been defined.\n',noOfPts,noOfFaces))
  outputFile=strcat(fileprefix,'Body.iv');
  fid=fopen(outputFile,'w');
  fprintf(fid,'#Inventor V2.0 ascii\n#plastic\n#10\n\n');
  fprintf(fid,'Separator {\n Material {\n  ambientColor 0.5 0.5 0.5\n  diffuseColor 0.5 0.5 0.5\n  specularColor 0 0 0\n  transparency 0\n}\n');
  fprintf(fid,'  Separator {\n   Transform {\n    translation 0 0 0 \n    }\n');
  fprintf(fid,'  ShapeHints {\n   vertexOrdering CLOCKWISE\n   shapeType  SOLID\n  }\n\n  Coordinate3 {\n   point [\n\n');

  
  for i=1:noOfFaces
    ptA=pt1(i);
    ptB=pt2(i);
    ptC=pt3(i);
    fprintf(fid,sprintf('    # %d %d %d\n',ptA,ptB,ptC));
    fprintf(fid,sprintf('    %6.2f %6.2f %6.2f\n',x(ptA),y(ptA),z(ptA)));
    fprintf(fid,sprintf('    %6.2f %6.2f %6.2f\n',x(ptB),y(ptB),z(ptB)));
    fprintf(fid,sprintf('    %6.2f %6.2f %6.2f\n\n',x(ptC),y(ptC),z(ptC)));
  end

  fprintf(fid,'    ]\n  }\n  FaceSet {\n   numVertices [\n    ');
  for j=1:noOfFaces-1
    fprintf(fid,'3,');
  end
  fprintf(fid,'3\n    ]\n  }');
  fprintf(fid,' }\n}\n');
  fclose(fid);
