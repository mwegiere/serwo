function [] = wykresStabilnosci( file1, file2, destFile1, destFile2, liczbaProb, nazwaWzorca, nr_kata )
%obliczanie stabilności estymacji i pozycji i orientacji dla wybranego kąta (nr_kata) dla liczby próbek liczbaProb
%Wejścia
    %file1 - ścieżka - przekształceine T_DC - wzorzez->kamera
    %file2 - ścieżka - przekształceine T_CW - kamera->świat
    %destFile1 - ścieżka - wykres dla odległości
    %destFile2 - ścieżka - wykres dla kątów
    %nazwaWzorca - nazwa wzorca np. diody, szachownica
    
    %liczbaProb - liczba próbek dla każdego kąta
    %nr_kata - wartość aktualnego kąta
T_DC_Array = zeros(4,4,liczbaProb);
fileID = fopen(file1,'r');
formatSpec = '%f';
sizeA = [4 4];

arrayCounter = 1;

while (arrayCounter<liczbaProb+1) 
    A = fscanf(fileID,formatSpec,sizeA);
    T_DC_Array(:,:,arrayCounter)= A'
    arrayCounter = arrayCounter + 1;
    
end
fclose(fileID);

T_CW_Array = zeros(4,4,liczbaProb);
fileID = fopen(file2,'r');
formatSpec = '%f';
sizeA = [4 4];

arrayCounter = 1;

while (arrayCounter<liczbaProb+1) 
    A = fscanf(fileID,formatSpec,sizeA);
    T_CW_Array(:,:,arrayCounter)= A';
    arrayCounter = arrayCounter + 1;
    
end
fclose(fileID);

diody_przesuniecie_x = zeros(1,liczbaProb);
diody_przesuniecie_y = zeros(1,liczbaProb);
diody_przesuniecie_z = zeros(1,liczbaProb);
diody_kat_a = zeros(1,liczbaProb);
diody_kat_b = zeros(1,liczbaProb);
diody_kat_c = zeros(1,liczbaProb);

T_DW_Array = zeros(4,4,liczbaProb);

arrayCounter=1;
while (arrayCounter<liczbaProb+1) 
    T_CW = T_CW_Array(:,:,nr_kata);
    T_WC=(T_CW)^(-1);
    T_DC = T_DC_Array(:,:,arrayCounter);
    T_CD = (T_DC)^(-1);
    if (T_DC == eye(4,4))
        T_DW_Array(:,:,arrayCounter) = 0;
        diody_przesuniecie_x(1,arrayCounter) = 0;
        diody_przesuniecie_y(1,arrayCounter) = 0;
        diody_przesuniecie_z(1,arrayCounter) = 0;
        diody_kat_a(1,arrayCounter) = 0;
        diody_kat_b(1,arrayCounter) = 0;
        diody_kat_c(1,arrayCounter) = 0;
        arrayCounter = arrayCounter + 1;  
    else
        T_DW_Array(:,:,arrayCounter) = T_WC*T_DC;
        [a,b,c]=GetEulerAngles(T_DW_Array(:,:,arrayCounter));   
        diody_przesuniecie_x(1,arrayCounter) = T_DW_Array(1,4,arrayCounter);
        diody_przesuniecie_y(1,arrayCounter) = T_DW_Array(2,4,arrayCounter);
        diody_przesuniecie_z(1,arrayCounter) = T_DW_Array(3,4,arrayCounter);
        diody_kat_a(1,arrayCounter) = a;
        diody_kat_b(1,arrayCounter) = b;
        diody_kat_c(1,arrayCounter) = c;
        arrayCounter = arrayCounter + 1;  
    end

end

S_diody_przesuniecie_x = std(diody_przesuniecie_x(1,przedzial))
M_diody_przesuniecie_x = mean(diody_przesuniecie_x(1,przedzial))

S_diody_przesuniecie_y = std(diody_przesuniecie_y(1,przedzial))
M_diody_przesuniecie_y = mean(diody_przesuniecie_y(1,przedzial))

S_diody_przesuniecie_z = std(diody_przesuniecie_z(1,przedzial))
M_diody_przesuniecie_z = mean(diody_przesuniecie_z(1,przedzial))

S_diody_kat_a = std(diody_kat_a(1,przedzial))
M_diody_kat_a = mean(diody_kat_a(1,przedzial))

S_diody_kat_b = std(diody_kat_b(1,przedzial))
M_diody_kat_b = mean(diody_kat_b(1,przedzial))

S_diody_kat_c = std(diody_kat_c(1,przedzial))
M_diody_kat_c = mean(diody_kat_c(1,przedzial))


max_diody_przesuniecie_x = max(diody_przesuniecie_x(1,przedzial))
min_diody_przesuniecie_x = min(diody_przesuniecie_x(1,przedzial))


end


