% codeDeinitialize 函数介绍：代码反初始化，自动从工作路径中删除当前目录及子目录
% 
%==========================================================================
% 创建目的： 进行多个实验时，当前工作路径中可能与其他地方有同名函数名，这些函数可
%           能存在版本不一样，出现调用混轮，从而导致bug发生。因此在这里整合代码，
%           在每次实验完成时从工作路径中删除当前路径，确保该次实验使用的函数不会
%           对其他实验的函数产生影响。
%
%==========================================================================
%                              函数用法
% 无输入输出， 直接调用函数名即可 codeDeinitialize() 或 codeDeinitialize
%==========================================================================
%                           当前版本： v1.0      
%                              作者：吕恒
%==========================================================================
%  更新日志：
%       【v1.0】 2022/1/21  初步实现功能
%==========================================================================
%                              用法示例
% 示例1： codeDeinitialize();
% 示例2： codeDeinitialize;
%==========================================================================
%                              具体实现
%==========================================================================

if(exist('CURRENTPATH','var'))
    rmpath(genpath(CURRENTPATH));  % 从工作路径中删除当前路径
else
    fprintf(2,'警告：程序中途存在clear，CURRENTPATH变量被清除， 目前只会从工作路径中移除当前文件夹路径\n');
    rmpath(genpath(pwd));  % 从工作路径中删除当前路径
end