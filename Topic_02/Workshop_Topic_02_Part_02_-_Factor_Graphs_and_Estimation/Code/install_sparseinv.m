% Installation script

% First make sure the search paths are set up
setup

% See if we need to build the sparseinv mex file, if so, set it up
if (exist('sparseinv_mex', 'file') == 3)
    fprintf('Found sparseinv mexfile at %s\n', which('sparseinv_mex'));
else
    disp('Building sparseinv mex file');
    sparseInvFile = which('sparseinv.m');
    [filepath,~,~] = fileparts(sparseInvFile);
    currentPwd = pwd;
    cd(filepath);
    sparseinv_install
    cd(currentPwd);
end