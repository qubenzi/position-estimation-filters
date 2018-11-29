%T = [0.01 0.01 0.01];


filename = './navchip/navchip.csv';

samples = 4000;
D_nav = csvread(filename, 1, 1, [1, 1, samples, 6]);
dt = 0.004637307785202;
T = 0:dt:dt*(samples-1);

%quaternion for orientation
quaternion = zeros(length(T), 4);
q0 = [0 0 1 0];

Kp = 6;
Ki = 4;
integral=[0 0 0];

%dt = 4*60*60/length(M);
   % fiAcc = [atan2(M(:,5),M(:,6)) asin(-M(:,4)/g) atan2(M(:,4),M(:,5))]*180/pi;
   % fiGyr = zeros(length(M),3);
  %  fiGyr(1,:) = dt*M(1,1:3);
  %  for i=2:length(M)
  %      fiGyr(i,:) = fiGyr(i-1,:) +dt*M(i,1:3);
  %  end
  
  Gyr  = D_nav(:, 1:3); %xyz
  Acc = D_nav(:, 4:6); %xyz
  
  pitchAcc = atan2(Acc(:, 1), ( Acc(:, 1).^2 + Acc(:, 3).^2 ) ); %rad
  rollAcc = atan2(Acc(:, 2), ( Acc(:, 2).^2 + Acc(:, 3).^2 )); %rad
  
  
  
  for i=1:length(T)
      %Update
      if(i==1)
         q = q0;
      else
      q = quaternion(i-1, :);
      end
      %norm curr m\acc
      if norm(Acc(i, 1:3))==0
      %    return
      end
      
      Acc_norm = Acc(i, 1:3)/norm(Acc(i, 1:3));
      
      %estimated gravity vector(from previous q)
     
      vect_grav = [2*q(2)*q(4) - 2*q(1)*q(3), 
                   2*q(1)*q(2) + 2*q(3)*q(4),
                   q(1)^2-q(2)^2-q(3)^2+q(4)^2]; %still dont understand that part :(
               
       
         
               
      e_cross = cross(Acc_norm, vect_grav);
      integral = integral + Ki*e_cross*dt;
      
      %feedback
      Gyr_fb = Gyr(i, 1:3) + Kp*e_cross + integral;
      
      %RATE OF CHANGE
      Sw = [0 Gyr_fb];
      dQuat = 0.5 * quaternProd(q, Sw);
      %NEW QUAT, INTEGRATION
      q = q + dQuat * dt;
      q = q/norm(q);
      
      quaternion(i, :) = q;
    
  end
  
 euler = quatern2euler(quaternion);
  
  figure();
  plot(T, rad2deg(pitchAcc));
  hold on;
  
  plot(T,rad2deg(euler(:, 1)));
  grid on;
  
  figure();
  plot(T, rad2deg(rollAcc));
  
  hold on;
  
  plot(T, rad2deg(euler(:, 2)));
  grid on;
  
  figure()
  plot(T, euler(:, 3));
  
  grid on;
 