function [] = wykresStabilnosci( file1, file2, destFile1, destFile2, liczbaKatow, nazwaWzorca )
%Wejścia
    %file1 - ścieżka - przekształceine T_DC - wzorzez->kamera
    %file2 - ścieżka - przekształceine T_CW - kamera->świat
    %destFile1 - ścieżka - wykres dla odległości
    %destFile2 - ścieżka - wykres dla kątów
    %liczbaKatow - liczba próbek zdjęć
    %nazwaWzorca - nazwa wzorca np. diody, szachownica
T_DC_Array = zeros(4,4,liczbaKatow);
fileID = fopen(file1,'r');
formatSpec = '%f';
sizeA = [4 4];

arrayCounter = 1;

while (arrayCounter<liczbaKatow+1) 
    A = fscanf(fileID,formatSpec,sizeA);
    T_DC_Array(:,:,arrayCounter)= A'
    arrayCounter = arrayCounter + 1;
    
end
fclose(fileID);

T_CW_Array = zeros(4,4,liczbaKatow);
fileID = fopen(file2,'r');
formatSpec = '%f';
sizeA = [4 4];

arrayCounter = 1;

while (arrayCounter<liczbaKatow+1) 
    A = fscanf(fileID,formatSpec,sizeA);
    T_CW_Array(:,:,arrayCounter)= A';
    arrayCounter = arrayCounter + 1;
    
end
fclose(fileID);

diody_przesuniecie_x = zeros(1,liczbaKatow);
diody_przesuniecie_y = zeros(1,liczbaKatow);
diody_przesuniecie_z = zeros(1,liczbaKatow);
diody_kat_a = zeros(1,liczbaKatow);
diody_kat_b = zeros(1,liczbaKatow);
diody_kat_c = zeros(1,liczbaKatow);

T_DW_Array = zeros(4,4,liczbaKatow);

arrayCounter=1;
while (arrayCounter<liczbaKatow+1) 
    T_CW = T_CW_Array(:,:,arrayCounter);
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

skala = 1:liczbaKatow

figure(1);
clf(1);
hold on
s1 = 'Stabilność estymacji pozycji wzorca (';
s = strcat(s1,nazwaWzorca);
s2 = ') w globalnym, nieruchomym układzie współrzędnych, podczas obserwacji wzorca pod różnymi kątami';
s = strcat(s,s2);
title(s,'FontSize',15);
xlabel('Kąt odchylenia osi optycznej kamery od osi prostopadłej do wzorca (stopnie)','FontSize',15); % x-axis label
ylabel('Odległość układu wzorca od układu bazowego (metry)','FontSize',15); % y-axis label
plot (skala, diody_przesuniecie_x, '.b');
plot (skala, diody_przesuniecie_y, '.c');
plot (skala, diody_przesuniecie_z, '.m');
legend('odległość w osi x','odległość w osi y', 'odległość w osi z');
% fig.PaperUnits = 'inches';
% fig1.PaperPosition = [0 0 150 300];
% fig1.PaperPositionMode = 'manual';
%set(fig1, 'Position', [0 0 2000 1000])
%fig1.PaperPositionMode = 'auto';
%print(destFile1,'-dpdf','-r0')
hold off

figure(2);
clf(2);
hold on
s1 = 'Stabilność estymacji rotacji wzorca (';
s = strcat(s1,nazwaWzorca);
s2 = ') w globalnym, nieruchomym układzie współrzędnych, podczas obserwacji wzorca pod różnymi kątami';
s = strcat(s,s2);
title(s,'FontSize',15);
xlabel('Kąt odchylenia osi optycznej kamery od osi prostopadłej do wzorca (stopnie)','FontSize',15); % x-axis label
ylabel('Kąt pomiędzy układem wzorca a układem bazowym (radiany)','FontSize',15); % y-axis label
plot (skala, diody_kat_a, '*b');
plot (skala, diody_kat_b, '*c');
plot (skala, diody_kat_c, '*m');
legend('kąt w osi x','kąt w osi y', 'kąt w osi z');
%print(destFile2,'-dpng')
hold off

if(strcmp(nazwaWzorca,'diody'))
    przedzial = 25:65;
end
if(strcmp(nazwaWzorca,'szachownica'))
   przedzial = 15:50;
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

end


