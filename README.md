# CHOMP guided by demonstrations

*Note:*  
The codes in `chomp/` are written refering to [CMU AIRLAB's chomp toolbox](https://theairlab.org/research/2016/11/01/opensourceplanning/).  
However, there are some problems w.r.t the `projected newton method` proposed in CMU's chomp toolbox. The algorithm is actually different from its [corresponding article](https://epubs.siam.org/doi/abs/10.1137/08073812X).  

**3D traj tracking demo requires:**  
- Peter Coke rvctools
- Coppeliasim 4.4

## Some Test Logs:
### 1. `chomp_2DPoint2_fixedbound`
Use trapezoidal velocity profile trajectory as initial condition.  

*Note: velocity boundary condition cannot be held by partitioned 2-order smooth matrix. (use full smooth matrix instead)*  

**Parameter tuning**: Tune the smooth cost parameter to make the smooth cost at the same **order of magnitude** as the obstacle cost, holding the order of magnitude **around 10**. 
- fine parameters: $\lambda_1=10^{-7}, \lambda_2=100, \eta=10^7, N=200$

### 2. `chomp_2DPoint3_fixedbound`
Same as above.
- fine parameters: $\lambda_1=10^{-7}, \lambda_2=200, \eta=1.5\times 10^{10}, N=200$

### 3. `chomp_2DPoint1_box_inequ`
- No need to use sparse QP for such small scale problem.
- Did not see significant computational advance of QP active-set warmstart for small number of constraints.

### 4. `chomp_2DPoint3_fixedbound2`
As shown in `chomp_2DPoint3_fixedbound.m`, both velocity and acceleration boundary conditions of the deformed trajectory can be held by using **3-order forward differetial matrix** as the metric.  
This demo shows the combinition of using 3-order forward differetial metric and 1-order smoothness cost function (equivalent to **shortest path**)