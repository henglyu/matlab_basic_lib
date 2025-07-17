function hotplot(X, qlevel)
    % 1. 获取信号
    [N,M] = size(X);
    if(N == 0)
        return;
    end
    
    % 2. 确定幅值等级
    minVal = min(X(:));
    maxVal = max(X(:));
    levels = minVal:qlevel:maxVal;      % 所有可能出现的整数幅值
    nLevel = numel(levels);

    % 3. 逐采样点做直方图，统计重复次数
    counts = zeros(nLevel, M);   % 行：幅值等级，列：采样点
    for m = 1:M
        % histcounts 默认返回落在各 bin 的计数
        c = histcounts(X(:,m), [levels Inf]);  % 多一个右边界
        counts(:,m) = c(1:end);              % 去掉最后一个 bin
    end

    % 4. 画热图
    imagesc(1:M, levels, counts);   % 横轴：采样点，纵轴：幅值
    axis xy
    colorbar
    xlabel('采样点')
    ylabel('幅值等级')
    title('信号叠加热图（颜色=重复次数）');
    colormap hot  
%     caxis([0, 1000])
%     ylim([-1000,1000]);
end