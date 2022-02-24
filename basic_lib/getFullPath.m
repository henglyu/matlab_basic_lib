function fullpath = getFullPath(inputstr)
% getFullPath函数介绍：用于获取可供matlab识别的完整路径
% 
%==========================================================================
% 创建目的： matlab目录提取感觉不方便，也没有像C语言可以使用../../../data来获取
%           不同层级目录的路径，因此在此处进行基础库建设。
%
%==========================================================================
%                              函数用法
% 输出：
%   fullpath    返回matlab标准的完整路径
% 输入：
%   inputstr    输入的路径字符串，用\\或\或//或/或\和/的任意组合来分割不同层级的目录
%
%   【其他tips】
%    1. .. 可以用于表示当前目录或返回上层目录，具体见下方介绍
%       ../           表示当前目录
%       ../../        表示上一级目录
%       ../../../     表示上上一级目录
%       依次类推
%
%    2. 无输入，或输入为空，或输入为..，或输入为../时，表示获取当前文件夹路径。
%
%    3. 
%
%
%==========================================================================
%                           当前版本： v1.1      
%                              作者：吕恒
%==========================================================================
%  更新日志：
%
%       【v1.1】 2022/1/21  新增输入为空或无输入的情况，此时效果为获取当前路径
%       【v1.0】 2022/1/20  初步实现功能
%==========================================================================
%                              用法示例
% 假设当前程序所在目录为 D:\MATLAB2020a\lvheng_basic_lib
% 以及目前的层级结构为 D:\MATLAB2020a
%                           |--- toolbox
%                           |--- help
%                           |--- lvheng_basic_lib
%                                       |--- test1
%                                       |--- test2
%                           |--- ... (其他文件夹)
%
%
% 示例1：
%   fullpath = getFullPath('test1')
%  ===》 返回： fullpath = 'D:\MATLAB2020a\lvheng_basic_lib\test1'
% 
%
% 示例2：
%   fullpath = getFullPath('..\test1')
%  ===》 返回： fullpath = 'D:\MATLAB2020a\lvheng_basic_lib\test1'
%
%
% 示例3：
%   fullpath = getFullPath('..\..\test1')
% ===》 返回：
%   警告（也仅仅是警告）：可能不存在路径D:\MATLAB2020a\test1
%   fullpath = 'D:\MATLAB2020a\test1'
%
%
% 示例4：
%   fullpath = getFullPath('..\..\toolbox')
%  ===》 返回： fullpath = 'D:\MATLAB2020a\toolbox'
%
%
% 示例5：
%   fullpath = getFullPath('..\..\toolbox')
%  ===》 返回： fullpath = 'D:\MATLAB2020a\toolbox'
%
%
% NOTE： 上述..\..\中的\可以由任意个\或/组合而成, 但是..不支持多个点组合，目前
%        只支持2个点的规范使用，即 ..
%
% 示例6：
%   fullpath = getFullPath('..\\\..//\/\\toolbox')
%  ===》 返回： fullpath = 'D:\MATLAB2020a\toolbox'
%
%
% 示例7：
%   fullpath = getFullPath('....\toolbox')
%  ===》 返回： 
%             ../           表示当前目录
%             ../../        表示上一级目录
%             ../../../     表示上上一级目录
%             依次类推
%             错误使用 getFullPath (line 89)
%             非法路径名: 超过2个..的连用， ..的用法参考上面内容
%
% 示例8：
%   fullpath = getFullPath()
%  ===》 返回： fullpath = 'D:\MATLAB2020a\lvheng_basic_lib'
%
%
% 示例9：
%   fullpath = getFullPath([])
%  ===》 返回： fullpath = 'D:\MATLAB2020a\lvheng_basic_lib'
%
%
% 示例10：
%   fullpath = getFullPath('..')
%  ===》 返回： fullpath = 'D:\MATLAB2020a\lvheng_basic_lib'
%==========================================================================
%                               具体实现
%==========================================================================

if(nargin<1 || isempty(inputstr)) % 【v1.1新增】 无输入或空输入时效果等价与pwd
    fullpath = pwd; % 无输入时，表示获取当前文件夹路径
else
    % 获取返回上一层目录的次数
    locs = strfind(inputstr, '..');
    if(~isempty(locs))
        if(numel(locs) == 1 || min(diff(locs))>1)
            backNumber = numel(locs)-1;
        else
            disp('../           表示当前目录');
            disp('../../        表示上一级目录');
            disp('../../../     表示上上一级目录');
            disp('依次类推');
            error('非法路径名: 超过2个..的连用， ..的用法参考上面内容');
        end


        % 获取省略路径
        currentFolder = pwd;  % 获取当前文件夹路径
        if(backNumber==0)
            hidePath = currentFolder;
        else
            index_dir=strfind(currentFolder,'\'); % 寻找层级下标
            level = numel(index_dir)-backNumber+1; % 确定目录层级
            if(level<1)
                fprintf(2,'警告（也仅仅是警告）：已到根目录，无法获取更上一层目录\n');
                level = 1;
            end
            hidePath=currentFolder(1:index_dir(level)-1);
        end

        % 获取非省略路径
        relativePath = inputstr(locs(end)+2:end); % 获取相对路径
        relativePath(relativePath == '/') = '\'; % 转化 / 为 \ 
        relativePath(strfind(relativePath, '\\')) = []; % 转化 \\ 为 \


        % 组合路径，获取完整路径
        fullpath = [hidePath, relativePath];
    else
        fullpath = inputstr;
    end

    % 删除不规范的\ 
    fullpath(fullpath == '/') = '\'; % 转化 / 为 \ 
    while(fullpath(1) == '\')
        fullpath(1) = [];
    end
    while(fullpath(end) == '\')
        fullpath(end) = [];
    end
end

% 目录存在性检测，并给予提醒
if(exist(fullpath,'file'))
    if(nargin > 0 && ~isempty(inputstr) && isempty(locs))
        fullpath = fullfile(pwd,fullpath);
    end
else
    fprintf(2,'警告（也仅仅是警告）：可能不存在路径%s\n',fullpath);
end

end % function end