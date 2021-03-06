% codeInitialize 函数介绍：代码初始化，清空一切变量，显示的窗口，显示的命令行内
%                         容，自动添加当前目录及子目录为工作路径
% 
%==========================================================================
% 创建目的： 每次新建一个项目时，都要手动添加工作路径或addpath添加，比较麻烦，这
%           里便对这些功能进行整合，达到初始化效果。
%
%==========================================================================
%                              函数用法
% 无输入输出， 直接调用函数名即可 codeInitialize() 或 codeInitialize
%==========================================================================
%                           当前版本： v1.0      
%                              作者：吕恒
%==========================================================================
%  更新日志：
%       【v1.0】 2022/1/21  初步实现功能
%==========================================================================
%                              用法示例
% 示例1： codeInitialize();
% 示例2： codeInitialize;
%==========================================================================
%                              具体实现
%==========================================================================

clc                             % 清除所有命令行文本
clear                           % 清除所有变量
close all                       % 关闭所有额外窗口
CURRENTPATH = pwd;              % 获取当前目录路径
addpath(genpath(CURRENTPATH));  % 添加当前目录到工作路径