T = [0.002 0.002 0.01];
ex = exist('M','var');
if ex==0
    M = csvread('navchip.csv',1,1);    
    dt = 4*60*60/length(M);
    fiAcc = [atan2(M(:,5),M(:,6)) atan2(M(:,6),M(:,4)) atan2(M(:,4),M(:,5))]*180/pi;
    fiGyr = zeros(length(M),3);
    fiGyr(1) = dt*M(1,1);
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
subplot(3,1,1)
plot(dt:dt:4*60*60,fi(:,1))
subplot(3,1,2)
plot(dt:dt:4*60*60,fi(:,2))
subplot(3,1,3)
plot(dt:dt:4*60*60,fi(:,3))