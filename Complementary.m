% Aby ponownie przeliczyć macierze kątów wg. akcelerometru i żyroskopu
% należy usunąć macierz M
% Aby ponownie przeliczyć macierz kątów po filtracji należy usunąć macierz
% fi
% Aby zmodyfikować parametry filtru komplementarnego należy dla każdej 
% osi [x y z] zmodyfikować zawartość wektora T zawierającego stałe czasowe 
% dla poszczególnych osi.
% uwaga: z powodu braku magnetometru nie jest możliwym ustalenie kąta
% obrotu względem osi pionowej(z), ponieważ nie wiadomo gdzie się znajduje
% północ. Dlatego w celu pokazania działania filtru dla tej osi
% potraktowaną ją jak oś x.

T = [0.01 0.01 0.01];
% T = p*dt/(1-p)
g = 10;
ex = exist('M','var');
if ex==0
    M = csvread('navchip.csv',1,1);    
    dt = 4*60*60/length(M);
    fiAcc = [atan2(M(:,5),M(:,6)) asin(-M(:,4)/g) atan2(M(:,4),M(:,5))]*180/pi;
    fiGyr = zeros(length(M),3);
    fiGyr(1,:) = dt*M(1,1:3);
    for i=2:length(M)
        fiGyr(i,:) = fiGyr(i-1,:) +dt*M(i,1:3);
    end
end

ex = exist('fi','var');
if ex==0
    len = length(M);
    alfa = T./(dt +T)
    fi = zeros(len,3);
    fi(1,:) = (1-alfa).*fiAcc(1,:) +alfa.*fiGyr(1,:);
    for i=2:len
        fi(i,:) = alfa.*fi(i-1,:) + (1-alfa).*fiAcc(i,:) +alfa.*(fiGyr(i,:)-fiGyr(i-1,:));
    end
end

figure
subplot(3,1,1)
title('Położenie obiektu obrót względem osi X')
hold on
plot(dt:dt:4*60*60,fiGyr(:,1))
plot(dt:dt:4*60*60,fiAcc(:,1))
plot(dt:dt:4*60*60,fi(:,1))
legend('kąt wg. żyroskopu','kąt wg. akcelerometru','kąt po filtracji')
subplot(3,1,2)
title('Położenie obiektu obrót względem osi Y')
hold on
plot(dt:dt:4*60*60,fiGyr(:,2))
plot(dt:dt:4*60*60,fiAcc(:,2))
plot(dt:dt:4*60*60,fi(:,2))
subplot(3,1,3)
title('Położenie obiektu obrót względem osi Z (Traktowany jak oś X)')
hold on
plot(dt:dt:4*60*60,fiGyr(:,3))
plot(dt:dt:4*60*60,fiAcc(:,3))
plot(dt:dt:4*60*60,fi(:,3))

% figure
% title('Położenie po filtracji')
% subplot(3,1,1)
% plot(dt:dt:4*60*60,fi(:,1))
% subplot(3,1,2)
% plot(dt:dt:4*60*60,fi(:,2))
% subplot(3,1,3)
% plot(dt:dt:4*60*60,fi(:,3))
% 
% figure
% title('Położenie według akcelerometru')
% subplot(3,1,1)
% plot(dt:dt:4*60*60,fiAcc(:,1))
% subplot(3,1,2)
% plot(dt:dt:4*60*60,fiAcc(:,2))
% subplot(3,1,3)
% plot(dt:dt:4*60*60,fiAcc(:,3))
% 
% figure
% title('Położenie według żyroskopu')
% subplot(3,1,1)
% plot(dt:dt:4*60*60,fiGyr(:,1))
% subplot(3,1,2)
% plot(dt:dt:4*60*60,fiGyr(:,2))
% subplot(3,1,3)
% plot(dt:dt:4*60*60,fiGyr(:,3))
