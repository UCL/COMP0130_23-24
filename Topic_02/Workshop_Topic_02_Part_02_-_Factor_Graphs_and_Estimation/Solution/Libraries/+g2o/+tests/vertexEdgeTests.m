% Vertex edge test

bv1 = g2o.BaseVertex();

be1 = g2o.BaseEdge();


be1.setVertex(1, bv1);


be2 = g2o.BaseEdge();
be2.setVertex(1, bv1);