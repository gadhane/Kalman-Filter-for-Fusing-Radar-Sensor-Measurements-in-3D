close all;
clear all;
dt=0.1;
u=2.55;
duration=10;
CM_IDX=[];
Q=[2; 2; 2;1;1; 0];
Q_estimate=Q;
Accel_Noise_Mag=0.05;
tkn_x=1;%measurement noise in x 
tkn_y=1;%measurement noise in y
tkn_z=1;%measurement noise in z
tkn_vx=1;%measurement noise of velocity in x
tkn_vy=2;%measurement noise of velocity in y
tkn_vz=1;%Measurement noise of Velocity in Z
measurement_noise_mag=10;

Ez=[tkn_x^2 0 0 0 0 0;0 tkn_y^2 0 0 0 0;0 0 tkn_z^2 0 0 0;0 0 0 tkn_vx^2 0 0;0 0 0 0 tkn_vy^2 0;0 0 0 0 0 tkn_vz^2];
Ex=Accel_Noise_Mag^2*[dt^4/4 0 0 dt^3 0 0;0 dt^4/4 0 0 dt^3/2 0;0 0 dt^4/4 0 0 dt^2;dt^3/2 0 0 dt^2 0 0;0 dt^3/2 0 0 dt^2 0;0 0 dt^3/2 0 0 dt^2];% process error covariance
P=Ex;

A=[1 0 0 dt 0 0;0 1 0 0 dt 0;0 0 1 0 0 dt;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1];% transition matrix
B=[dt^2/2;dt^2/2;0;dt;dt;0];%control matrix in this case accelaration
C=[1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1];%measurement matrix
%Initial variables
Q_Loc=[];
Q_Loc_x=[];
Q_Loc_y=[];
Q_Loc_z=[];
Vel_measured=[];
Q_Measure=[];
Q_Loc_MeasureX=[];
Q_Loc_MeasureY=[];
Q_Loc_MeasureZ=[];

figure(1);clf
for t=0:dt:duration
    targetAccel_noise=Accel_Noise_Mag*[(dt^2/2)*randn;(dt^2/2)*randn;0;dt*randn;dt*randn;0];
    Q=A*Q+B*u+targetAccel_noise;
    measurement_noise=measurement_noise_mag*randn;
    Measurement=C*Q+measurement_noise;
    Q_Loc=[Q_Loc Q(1:3)];
    Q_Loc_x=[Q_Loc_x;Q(1)];
    Q_Loc_y=[Q_Loc_y;Q(2)];
    Q_Loc_z=[Q_Loc_z;Q(3)];
    Q_Measure=[Q_Measure Measurement];
    Q_Loc_MeasureX=[Q_Loc_MeasureX;Measurement(1)];
    Q_Loc_MeasureY=[Q_Loc_MeasureY;Measurement(2)];
    Q_Loc_MeasureZ=[Q_Loc_MeasureZ;Measurement(3)];
    Vel_measured=[Vel_measured Measurement(4:6)];
    
    plot3(Q_Loc_MeasureX,Q_Loc_MeasureY,Q_Loc_MeasureZ,'ro');
    plot3(Q_Loc_x,Q_Loc_y,Q_Loc_z,'b*');
    hold on
    grid on
    %pause
end

%Estimated variables
Q_Loc_estimate=[];
Q_Loc_estimateX=[];
Q_Loc_estimateY=[];
Q_Loc_estimateZ=[];
Vel_estimate=[];
Q=[0; 0; 0;0; 0;0];
P_estimate=P;
Predic_state=[];
Predic_var=[];

for t=1:length(Q_Loc)
    Q_estimate=A*Q_estimate+B*u;
    Predic_state=[Predic_state Q_estimate];
    P=A*P*A'+Ex;
    Predic_var=[Predic_var P];
    K=P*C'*inv(C*P*C'+Ez);
    if ~isnan(Q_Measure(:,t))
     Q_estimate=Q_estimate+K*(Q_Measure(:,t)-C*Q_estimate);
    end
    P=(eye(6)-K*C)*P;
    Q_Loc_estimate=[Q_Loc_estimate Q_estimate(1:3)];
    Q_Loc_estimateX=[Q_Loc_estimateX;Q_estimate(1)];
    Q_Loc_estimateY=[Q_Loc_estimateY;Q_estimate(2)];
    Q_Loc_estimateZ=[Q_Loc_estimateZ;Q_estimate(3)];
    Vel_estimate=[Vel_estimate Q_estimate(4:6)];
    [r,a,e] = Estimate(Q_estimate(1),Q_estimate(2),Q_estimate(3));
        %if (t>1&&t<length(Q_Loc))
        %r(t)=r;
        %v=(r(t)-r(t-1))./dt;
        %end
        %
        %lqe();kalman filter estimator built in matlab function
        %lqr built in function used to calculate kalman gain
    hold on
    grid on
end
figure(2);clf
tt=0:dt:duration;
plot(tt,Q_Loc_y,'-r.',tt,Q_Loc_MeasureY,'-k.',tt,Q_Loc_estimateY,'-g.');
figure(3);clf
plot3(Q_Loc_x,Q_Loc_y,Q_Loc_z,'g+');
hold on
plot3(Q_Loc_MeasureX,Q_Loc_MeasureY,Q_Loc_MeasureZ,'b*');

hold on
plot3(Q_Loc_estimateX,Q_Loc_estimateY,Q_Loc_estimateZ,'ro');

hold on
legend('Actual location','Mesaured location','Estimated location');
grid on