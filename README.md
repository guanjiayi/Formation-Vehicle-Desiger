# Formation-Vehicle-Desiger

# decription:
the program is use to control the following vehicle to following the head vehicle of the formation vehicle project.


version decription:
   1. VF_Following_20200811, which is a frame without any control model;
   2. VF_Following_20200816, which is a whole project of group vehicle control, but it base on a very sample contorl model. and it is the fisrt version program.
   3. VF_Following_20200830, which is a whole project but it out of test in real vehicle, so there are some mistake in this program. 
   4. VF_Following_20200905_n which is the second version program  and it add some control model base on the first version program.
   conclusion : so you can use VF_Following_20200816 and VF_20200905_n grogram control the following vehicle
   
   
# use guide：
   step 1: cd ~    
   step 2: cd VF_Following_20200905_n/            # 进入该工程目录下;
   
   step 3: rm -rf build                           # 第一次运行时，需要把 build/文件家删掉，不然之前的编译的文件会提示链接不对;
   
   step 4: mkdir build                            # 创建一个build/文件夹;
   
   step 5: cd build/                              # 进入创建的build/文件夹下;
   
   step 6：cmake ..                               # 通过cmake生成makefile;
   
   step 7: make                                   # 根据makefile直接用make编译;
   
   step 8: ./FV_Folloing                          # 直接执行编译后生成的可执行文件;
   
   

# contact:
  email: guan_jiayi@126.com; 1224415674@qq.com; gunajiayi@tongji.edu.cn
