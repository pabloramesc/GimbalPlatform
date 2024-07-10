clear

s1 = serial('COM15', 'BaudRate', 115200);

fopen(s1);
for k=1:120*100
    str = fscanf(s1);
    data(k,:)=str2num(str);
    if(~mod(k,100))
        k
    end
end
fclose(s1);
delete(s1);
save('datasets/platform/test.mat')