function map_index = get_index_from_pos(x,y)
% i Ϊ ����
% j Ϊ ����
% Map(i, j)>0 ��ʾ (i, j)���ǿ�ͨ��
if (x <-19.9) || (x > 20.0)|| (y <-19.9) || (y > 20.0)
    disp("EEROR: index is out of range.");
else
definition = 0.1;
i = floor((20 - x) / definition) + 1 ;
j = floor((20 - y) / definition) + 1 ;
map_index = [i, j];
end
end