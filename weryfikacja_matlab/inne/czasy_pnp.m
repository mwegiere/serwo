liczbaZdjec = 100;
A = zeros(liczbaZdjec,1);
fileID = fopen('szachownica_czas_pojedyncze/recognition_time.txt','r');
formatSpec = '%i';

arrayCounter = 1;

while (arrayCounter<liczbaZdjec+1) 
    A(arrayCounter) = fscanf(fileID,formatSpec,1);
    arrayCounter = arrayCounter + 1;
    
end
fclose(fileID);
arrayCounter
A
Mean = mean(A)
Std = std(A)
