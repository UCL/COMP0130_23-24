% Script to set up the search path

if (exist('g2o.core.BaseVertex', 'class')  ~= 8)
    addpath(genpath([pwd, filesep, 'Libraries']));
end