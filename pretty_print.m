function pretty_print(A, rowNames, colNames)
% This function displays a matrix in a pretty way with row and column numbers, and delimiters for rows and columns
    [rows, cols] = size(A);
    if nargin < 2 || isempty(rowNames)
        rowNames = arrayfun(@(x) num2str(x), 1:rows, 'UniformOutput', false);
    end
    if nargin < 3 || isempty(colNames)
        colNames = arrayfun(@(x) num2str(x), 1:cols, 'UniformOutput', false);
    end
    if ischar(rowNames)
        rowNames = cellstr(rowNames);
    end
    if ischar(colNames)
        colNames = cellstr(colNames);
    end
    maxColNameLength = max(cellfun(@length, colNames)); % calculate the maximum length of the column names
    fprintf('%*s', maxColNameLength + 7, ''); % print an empty space to align the column numbers with the column names
    for j = 1:cols
        fprintf('%*s', maxColNameLength + 3, colNames{j});
    end
    fprintf('\n');
    fprintf('%*s', maxColNameLength + 4, ''); % print an empty space to align the delimiter with the column names
    for j = 1:cols
        fprintf('----------');
    end
    fprintf('\n');
    for i = 1:rows
        fprintf('%*s |', maxColNameLength + 1, rowNames{i});
        for j = 1:cols
            fprintf('%*.3f |', maxColNameLength + 3, A(i,j));
        end
        fprintf('\n');
        fprintf('%*s |', maxColNameLength + 4, ''); % print an empty space to align the delimiter with the column names
        for j = 1:cols
            fprintf('----------');
        end
        fprintf('\n');
    end
end
