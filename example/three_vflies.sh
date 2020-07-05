rosrun multi_drone_platform add_drone <<!

NEW
vflie_00
1.5
1

NEW
vflie_01
1.5
2.5

NEW 
vflie_02
2.5
0.5

END
!

rosrun multi_drone_platform all_debug_windows expanded& #command line options are 'expanded' or 'compressed'

~/MATLAB/bin/matlab -nodesktop -nosplash -r "addpath('~/catkin_ws/src/multi_drone_platform/matlab'); run('~/catkin_ws/src/multi_drone_platform/matlab/scripts/mdp_gen_graphing.m');quit();"
