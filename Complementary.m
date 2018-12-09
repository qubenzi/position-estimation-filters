% Aby ponownie przeliczyć macierze kątów wg. akcelerometru i żyroskopu
% należy usunąć macierz DATA bądź Accelerometr
% Aby ponownie przeliczyć macierz kątów po filtracji należy usunąć macierz
% fi
% zmienna ver służy do przełączania źródła danych (1 dla Magwicka, 0 dla
% Navchip)
clear all % zakomentuj jeśli nie chcesz przeładowania danych
T = [0.1 0.1];
ver = 1;
% T = p*dt/(1-p)
g = 10;
ex = exist('DATA','var');
if ex==0 && ver==0
    load('DATA.mat')    
    dt = 4*60*60/length(DATA);
    fiAcc = [atan2(DATA(:,5),DATA(:,6)) asin(-DATA(:,4)/g)]*180/pi;
    fiGyr = zeros(length(DATA),2);
    fiGyr(1,:) = dt*DATA(1,1:2);
    for i=2:length(DATA)
        fiGyr(i,:) = fiGyr(i-1,:) +dt*DATA(i,1:2);
    end
    len = length(DATA);
    time = dt:dt:4*60*60;
end

ex = exist('Accelerometer','var');
if ex==0 && ver==1
    load('RevolvingIMU.mat')
    dt = time(2:end)-time(1:end-1);
    fiAcc = [atan2(Accelerometer(:,2),Accelerometer(:,3)) asin(-Accelerometer(:,1)/g)]*180/pi;
    fiGyr = zeros(length(Gyroscope),2);
    fiGyr(1,:) = dt(1)*Gyroscope(1,1:2);
    for i=2:length(Gyroscope)
        fiGyr(i,:) = fiGyr(i-1,:) +dt(i-1)*Gyroscope(i,1:2);
    end
    len = length(Accelerometer);
end

ex = exist('fi','var');
if ex==0
    alfa = T./(mean(dt) +T)
    fi = zeros(len,2);
    fi(1,:) = (1-alfa).*fiAcc(1,:) +alfa.*fiGyr(1,:);
    for i=2:len
        fi(i,:) = alfa.*fi(i-1,:) + (1-alfa).*fiAcc(i,:) +alfa.*(fiGyr(i,:)-fiGyr(i-1,:));
    end
end

figure
subplot(2,1,1)
title('Położenie obiektu obrót względem osi X')
hold on
plot(time,fiGyr(:,1))
plot(time,fiAcc(:,1))
plot(time,fi(:,1))
legend('kąt wg. żyroskopu','kąt wg. akcelerometru i magnetometru','kąt po filtracji')
subplot(2,1,2)
title('Położenie obiektu obrót względem osi Y')
hold on
plot(time,fiGyr(:,2))
plot(time,fiAcc(:,2))
plot(time,fi(:,2))
