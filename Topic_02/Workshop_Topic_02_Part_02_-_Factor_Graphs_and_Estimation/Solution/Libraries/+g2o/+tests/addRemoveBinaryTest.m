% Binary edge test


import g2o.core.*;
import g2o.tests.*;

H = HyperGraph();

v1 = TestVertex();
v2 = TestVertex();

disp('v1.edges()=');
v1.edges()

disp('v2.edges()=');
v2.edges()

T2 = TestBinaryEdge();

T2.setVertex(1, v1);
T2.setVertex(2, v2);

H.addVertex(v1);
H.addVertex(v2);
H.addEdge(T2);