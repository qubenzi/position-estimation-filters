clc;
clear all;
close all;

% ile probek z DATA chcesz wyswietlic
n = 100000;

DATA2 = load('DATA.mat');
DATA = cell2mat(struct2cell(DATA2));
DATA2 = DATA(1:n,:);
g = 9.81;
acceltetta = -asin(DATA2(:,4)./g);
accelphi   = atan2(DATA2(:,5),DATA2(:,6));
dt = 1/214;
gyrotetta = zeros(size(acceltetta));
gyrophi   = zeros(size(acceltetta));

for i=1:length(acceltetta)
    if i == 1
        gyrotetta(1) = acceltetta(1);
        gyrophi(1) = accelphi(1);
    else
        gyrotetta(i) = gyrotetta(i-1) + DATA2(i,2) * dt;
        gyrophi(i)   = gyrophi(i-1)   + DATA2(i,1) * dt;
    end
end

T = 0:dt:(length(acceltetta)*dt-dt);    
est_tetta = zeros(size(acceltetta));
est_phi   = zeros(size(acceltetta)); 


est_tetta(1) = acceltetta(1);
est_phi(1) = accelphi(1);

% kowariancja szumu -> zakozenie jest, ze znamy te wartosci
%  ich wymiar to n x n, gdzie n to ilosc uzywanych czujnikow  
R_tetta = [var(acceltetta)^2 0; 0 var(gyrotetta)^2];	
R_phi   = [var(accelphi  )^2 0; 0 var(gyrophi  )^2];

%%%%%%%% macierz kowariancji bledu estymacji
P_tetta = 1;
P_phi   = 1;

% Macierz kolumnowa, ilosc elementow jest rowna ilosci czujnikow, z ktorych pobieramy dane
H = [1; 1];

for i=1:length(acceltetta)
    %%%%%%%% predykcja -> obiekt się nie porusza nowe polozenie = stare polozenie
    if i == 1
    else
        est_tetta(i) = est_tetta(i-1);
        est_phi(i) = est_phi(i-1);
    end
    %%%%%%%% innowacja
    % e - macierz kolumnowa, zalezna od ilosci czujnikow, kazdy wiersz dla innego czujnika

    e_tetta = [acceltetta(i); gyrotetta(i)] - H*est_tetta(i); 
    e_phi   = [accelphi(i);   gyrophi(i)]   - H*est_phi(i);

    %%%%%%%% Kowariancja innowacji

    S_tetta = H * P_tetta * H' + R_tetta;
    S_phi   = H * P_phi   * H' + R_phi;

    %%%%%%%% Wzmocnienia K 
    % obliczenie jaki wpływ na estymacje położenia ma wynik modelu i pomiary z czujnikow 

    K_tetta = P_tetta * H' * (S_tetta)^(-1);
    K_phi   = P_phi   * H' * (S_phi  )^(-1);

    %%%%%%%% aktualizacja polozenia

    est_tetta(i) = est_tetta(i) + K_tetta * e_tetta;
    est_phi(i)   = est_phi(i)   + K_phi   * e_phi;

    %%%%%%%% aktualizacja macierzy kowariancji bledu estymacji

    P_tetta = (1 - K_tetta * H) * P_tetta;
    P_phi   = (1 - K_phi   * H) * P_phi;
end

subplot(211)
plot(T,est_tetta);
hold on;
plot(T,acceltetta);
plot(T,gyrotetta);
legend('po filtracji','akcelerometr','zyroskop');
ylabel('kąt[rad]');
xlabel('czas[s]');
title('\theta')
grid on

subplot(212)
plot(T,est_phi)
hold on
plot(T,accelphi);
plot(T,gyrophi);
legend('po filtracji','akcelerometr','zyroskop');
ylabel('kąt[rad]');
xlabel('czas[s]');
title('\phi');
