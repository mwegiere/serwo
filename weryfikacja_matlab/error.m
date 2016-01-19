liczbaZdjec = 71000;
A = zeros(liczbaZdjec,1);
fileID = fopen('robot_error.txt','r');
formatSpec = '%f';

arrayCounter = 1;

while (arrayCounter<liczbaZdjec+1) 
    A(arrayCounter) = fscanf(fileID,formatSpec,1);
    arrayCounter = arrayCounter + 1;
    
end
fclose(fileID);
plot(A(39000:60000));
%plot(A);
