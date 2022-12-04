function options = chomp_optimset_init()
%CHOMP_OPTIMSET_INIT Get a default struct of chomp options

options = optimset();
options.MetricInverse = [];
options.Metric = [];
options.Eta = 1;
options.DecreaseStepSize = 0;
options.MaxIter = 100;
options.MinIter = 1;
options.TolFun = 1e-3;
options.TolCon = 1e-3;
options.ProjNewtStepSize = 0.05;
options.InequConstraintAlgorithm = 'qp_full'; %Choices {'proj_newton', 'qp_full',  'qp_sparse', 'qp_warmstart'}
options.FreeEndPoint = 0;
end