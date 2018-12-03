%wykorzystano biblioteke stworzona przez Magwicka do obliczen zwiazanych z
%quaternionami. Pliki znajduja sie w folderze quaternion_library
%Wiecej inofrmacji na http://www.x-io.co.uk/node/8#quaternions

addpath('quaternion_library');      % dodanie biblioteki
close all; 
clear;


filename = './navchip/navchip.csv'; %odczytanie danych
samples = 6000; %ilosc probek do odczytania z pliku csv (caly plik ma ponad 200mb)
D_nav = csvread(filename, 1, 1, [1, 1, samples, 6]);
dt = 0.004637307785202; %s
T = 0:dt:dt*(samples-1);
g = 9.81; %m/s2
bias_correction = 1; %mozna ustawic 1, wtedy bedzie odjeta srednia uchybow na kazdej z osi zyroskopu

quaternion = zeros(length(T), 4); %zdefiniowanie tablicy quaternionow do przechowywania orientacji obiektu
q0 = [0 0 1 0];

Kp = 2; %wspolczynniki oraz wektor calki dla regulatora PI 
Ki = 0;
integral=[0 0 0];

Gyr  = D_nav(:, 1:3); %odczytanie danych z zyroskopu, format osi: xyz
if(bias_correction==1)
    Gyr = Gyr - mean(Gyr);
end
Acc = D_nav(:, 4:6); %odczytanie danych z akcelerometru, format osi: xyz
  
%wyznaczenie katow pitch i roll na podstawie danych z samego akcelerometru
pitchAcc = atan2(-Acc(:, 2), sqrt( Acc(:,1).^2 + Acc(:, 3).^2 ) ); %rad
rollAcc = atan2(-Acc(:, 1), sqrt( Acc(:, 2).^2 + Acc(:, 3).^2 )); %rad
%wyswietlanie danych z czujnikow
figure('Name', 'Dane z sensorow');
axis(1) = subplot(2,1,1);
hold on;
plot(T, rad2deg(Gyr(:,1)));
plot(T, rad2deg(Gyr(:,2)));
plot(T, rad2deg(Gyr(:,3)));
legend('X', 'Y', 'Z');
xlabel('czas (s)');
ylabel('predkosc katowa \omega (deg/s)');
title('Zyroskop');
grid on;
hold off;
axis(2) = subplot(2,1,2);
hold on;
plot(T, Acc(:,1));
plot(T, Acc(:,2));
plot(T, Acc(:,3));
legend('X', 'Y', 'Z');
xlabel('czas (s)');
ylabel('przyspieszenie (m/s^2)');
title('Akcelerometr');
hold off;
linkaxes(axis, 'x'); 
grid on;

  
  
  for i=1:length(T)
      %filtr mahony'ego
      if(i==1)
         q = q0;
      else
      q = quaternion(i-1, :);
      end
      
      %normowanie danych z akcelerometru
      if norm(Acc(i, 1:3))==0
      %    return
      end
      
      Acc_norm = Acc(i, 1:3)/norm(Acc(i, 1:3));
      
      %ostatnia estymacja wektora grawitacji
     
      vect_grav = [2*q(2)*q(4) - 2*q(1)*q(3), 
                   2*q(1)*q(2) + 2*q(3)*q(4),
                   q(1)^2-q(2)^2-q(3)^2+q(4)^2]; 
               
       
      %blad na podstawie iloczynu wektorowego   
               
      e_cross = cross(Acc_norm, vect_grav);
      integral = integral + Ki*e_cross*dt;
      
      %obliczanie d omega
      Gyr_fb = Gyr(i, 1:3) + Kp*e_cross + integral;
      
      %obliczanie jak zmieni sie quaternion
      Sw = [0 Gyr_fb];
      dQuat = 0.5 * quaternProd(q, Sw);
      %calkowanie, nowa estymacja orientacji
      q = q + dQuat * dt;
      q = q/norm(q);
      
      quaternion(i, :) = q;
    
  end
  
 %przeksztalcenie wyniki do katow eulera w celu wizualizacji
 euler = rad2deg(quatern2euler(quaternConj(quaternion)));
 
 %upewniam sie ze 0 stopni nie zostalo zamienione na 180 stopni
 for idx = 1:numel(euler)
    element = euler(idx);
    if(euler(idx)>120)
         euler(idx)=euler(idx)-180;
    elseif(euler(idx)<-120)
         euler(idx)=euler(idx)+180;
    end
end
 
 %wyswietlanei wynikow
 figure('Name', 'Katy');
 axis(1) = subplot(3,1,1);
 hold on;
 plot(T, rad2deg(pitchAcc));
 plot(T,euler(:, 1));
 legend('\phi (akcelerometr)', '\phi (filtracja)');
 title('Kat \phi');
 grid on;
 hold off;
 
 axis(2) = subplot(3, 1, 2);
 hold on;
 plot(T, rad2deg(rollAcc));
 plot(T, euler(:, 2));
 grid on;
 legend('\theta (akcelerometr)', '\theta (filtracja)');
 title('Kat \theta');
 hold off;
 
 axis(3) = subplot(3, 1, 3);
 hold on;
 plot(T, euler(:, 3));
 grid on;
 legend('\psi (filtracja)');
 title('Kat \psi');
  
 grid on;
 hold off