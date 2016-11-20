NOTE:
in the default configuration we are using 'gpu_ray' sensor.
if you want to use 'ray' sensor (processing on the cpu instead of on gpu) 
you will need to do the following steps:
1. there is sdf file named 'model_cpu.sdf' change his name to 'model.sdf' (of course you will need to give to existing 'model.sdf' a diffrent name ).
2. in the '/src/ibeo.cpp' there is a Define that call 'USE_GPU', mark it.
3. recompile the component.
