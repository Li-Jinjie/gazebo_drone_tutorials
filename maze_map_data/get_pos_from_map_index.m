
function pos = get_pos_from_map_index(i,j)
% i Ϊ ����
% j Ϊ ����
% Map(i, j)>0 ��ʾ (i, j)���ǿ�ͨ��
if (i < 1) || (i > 400)|| (j < 1) || (j > 400)
    disp("EEROR: index is out of range.");
else
definition = 0.1;
x = -1 * definition * (i-1) + 20.0;
y = -1 * definition * (j-1) + 20.0;
pos = [x, y];
end
end