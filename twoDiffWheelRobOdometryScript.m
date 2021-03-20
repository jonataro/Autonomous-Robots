%modelo cinemático de robot diferencial de dos ruedas
%Robotica autónoma 
%Máster sistemas y control UNED&UCM
%Curso 2020-2021
%Autor: Jonatan Rodriguez jonataro@ucm.es
clc
close all
clear all
%Simulation Parameters
    Radius=3/pi;%Wheels radius
    PulsePerRev=32;%pulses of incremental encoder per wheel revolution
    WheelDistance=6/pi;% linear distance beween two wheels
%pulse signals received by incremental encoder in the wheels
    NR=[0,5,6,5,0, 4, 4,0,5,6,5,4,2,2,2,0];
    NL=[0,5,6,5,0,-4,-4,0,6,5,5,6,6,6,4,0];
    TimeCycle=0.1;
    SimTime=(length(NR)-1)*TimeCycle;
    Time=0:TimeCycle:(SimTime);
%execution speed optmization by declaring array before to assign values
%in execution time
    %DR=zeros(1,length(NR));
    %DL=zeros(1,length(NR));
    %D=zeros(1,length(NR));
    %X=zeros(1,length(NR));
    %Y=zeros(1,length(NR));
    %Angle=zeros(1,length(NR));
%t=0  initial robot pose
    X(1)=3;
    Y(1)=5;
    Angle(1)=0;
    
%Simulation    
    for i=2:length(NR)
        %every wheeel displacement
            DR(i)=2*pi*Radius*NR(i)/PulsePerRev;
            DL(i)=2*pi*Radius*NL(i)/PulsePerRev;
        %absolut displacement
            D(i)=(DR(i)+DL(i))/2;
        %angular displacement
            RO(i)=(DR(i)-DL(i))/WheelDistance;
        %pose calculation
            Angle(i)=Angle(i-1)+RO(i);
            X(i)=X(i-1)+D(i)*cos(Angle(i));
            Y(i)=Y(i-1)+D(i)*sin(Angle(i));
        % orientation normalized vector calculatiom
            normX=X(i-1)+0.25*cos(Angle(i));
            normY=Y(i-1)+0.25*sin(Angle(i));
        %plot orientation vectors
            hold on
            plot([X(i-1),normX],[Y(i-1),normY],'-X','MarkerIndices',2,'LineWidth',3)
    end
        %merge all time sample pose information into a single array
            pose=[X;Y;Angle];
        %plot the trajectory
            plot(X,Y,'b-o','LineWidth',0.5)

