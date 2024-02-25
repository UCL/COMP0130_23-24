% Vertex edge test

hg = g2o.HyperGraph();

bv1 = g2o.tests.TestVertex();
be1 = g2o.tests.TestEdge();

be1.setVertex(1, bv1);

hg.addVertex(bv1);
hg.addEdge(be1);
