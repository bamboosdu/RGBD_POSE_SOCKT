20180314 push to github: interface of move to views;
//修改传pose和rgbd的顺序；
//五个、多个机器人基本完成；
//设置socket禁用nagle算法；
//设置参数控制每次规划的路程跑多少比例，同时相应地也修改total_distance的计算；
//kinect frustum near far, before change code to multirobots;
//fix bug "dont move and dont turn", corresponds to MultiScan version 20180111;
//when dont move and rbt pose == task pose, change "scan direction is from rbt pose to task pose" to "turn PI/3 to scan around", corresponds to MultiScan version 20180110, lunch;
//set objective scan direction, corresponds to MultiScan version 20180109;
//modified a lot of codes, corresponds to MultiScan version 20180109;
//对应20171230晚上的更新，不过好像还是上次的版本，没什么改动；
//除了图优化，其他基本完成；
//机器人离散的旋转和移动统一用变量times衡量；
//与MultiScan修改depth为RGBD后的程序匹配；
