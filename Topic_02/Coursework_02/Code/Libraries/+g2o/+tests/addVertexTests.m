% Add vertex test cases

graph = g2o.HyperGraph();

bv = g2o.BaseVertex();
bv.setId(0);

tv = g2o.tests.TestVertex();

graph.addVertex(bv);

% Fails because tv Id not set
try
    graph.addVertex(tv);
catch ME
    ME
end

% Set the id
tv.setId(0);

% Fails because of a duplicate Id
try
    graph.addVertex(tv);
catch ME
    ME
end

% Set the id
tv.setId(0);

% Fails because of a duplicate Id
try
    graph.addVertex(tv);
catch ME
    ME
end


% Set the id correctly
tv.setId(1);

% Fails because of a duplicate Id
try
    graph.addVertex(tv);
catch ME
    ME
end


    %graph.addVertex(graph);
