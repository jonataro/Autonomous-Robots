% Cálculo de los tiempos de vuelo teóricos de balizas en un sistema de
% posicionamiento indoor por trilateración
%Robotica autónoma 
%Máster sistemas y control UNED&UCM
%Curso 2020-2021
%Autor: Jonatan Rodriguez jonataro@ucm.es
clear all
close all

%beacon positions
    b1 = [0,  2, 0.5];
    b2 = [4,  0,   2];
    b3 = [6,  4,   2];
    h  = 0.5;
% %speed of sound m/s
     v=340;
% %Return time of signals each beacons
%     t1 = 1.584e-02;
%     t2 = 1.149e-2;
%     t3 = 2.412e-2;
% %estimated distance from the emitter to to each beacon
%     r1 = t1*v/2;
%     r2 = t2*v/2;
%     r3 = t3*v/2;

    z=0.5; %plane   = z == h;
    x=1;
    y=1;

%sphere = (x-a)^2 + (y-b)^2 + (z-c)^2 == r^2;
%     sphere1 = (x-b1(1))^2 + (y-b1(2))^2 + (z-b1(3))^2 == r1^2;
%     sphere2 = (x-b2(1))^2 + (y-b2(2))^2 + (z-b2(3))^2 == r2^2;
%     sphere3 = (x-b3(1))^2 + (y-b3(2))^2 + (z-b3(3))^2 == r3^2;
       
rad1 = sqrt ((x-b1(1))^2 + (y-b1(2))^2 + (z-b1(3))^2);
rad2 = sqrt ((x-b2(1))^2 + (y-b2(2))^2 + (z-b2(3))^2);
rad3 = sqrt ((x-b3(1))^2 + (y-b3(2))^2 + (z-b3(3))^2);

t1=rad1*2/v;
t2=rad2*2/v;
t3=rad3*2/v;
 
 %visualization    
    hold on;
    %suelo
       [GroundX,GroundY] = meshgrid(0:6:6,0:4:4);
       GroundZ=0*GroundX;
       mesh(GroundX,GroundY,GroundZ,'Facecolor','none','EdgeColor','black');
       xlim([0 6]) 
       ylim([0 4])
       zlim([0 2])  
   %plano de altura de la baila del robot
        RobotZ = 0.5 *ones(size(GroundX));
        mesh(GroundX,GroundY,RobotZ,'Facecolor','none','EdgeColor','magenta');
       
    %balizas
        plot3 (b1(1),(b1(2)),(b1(3)),'o','color','blue')
        plot3 (b2(1),(b2(2)),(b2(3)),'o','color','red')
        plot3 (b3(1),(b3(2)),(b3(3)),'o','color','green')
        
    %"soportes" de las balizas
        plot3 ([b1(1) b1(1)],[(b1(2)) (b1(2))],[(b1(3)) 0],'color','blue');
        plot3 ([b2(1) b2(1)],[(b2(2)) (b2(2))],[(b2(3)) 0],'color','red');
        plot3 ([b3(1) b3(1)],[(b3(2)) (b3(2))],[(b3(3)) 0],'color','green');
        
    %señal de cada baliza
       rad_b1 = sqrt( (b1(1)-x)^2 + (b1(2)-y)^2 + (z-z)^2 );
       rad_b2 = sqrt( (b2(1)-x)^2 + (b2(2)-y)^2 + (z-z)^2 );
       rad_b3 = sqrt( (b3(1)-x)^2 + (b3(2)-y)^2 + (z-z)^2 );
       plotCircle3D ([b1(1) b1(2) h],[0 0 0],rad_b1,'blue');
       plotCircle3D([b2(1) b2(2) h],[0 0 0],rad_b2,'red');
       plotCircle3D([b3(1) b3(2) h],[0 0 0],rad_b3,'green');
         
  %posición del robot
        %Vectores que apuntan a la posicion
        
        plot3 ([b1(1) x],[(b1(2)) y],[(b1(3)) z],'color','blue','Linewidth',1);
        plot3 ([b2(1) x],[(b2(2)) y],[(b2(3)) z],'color','red','Linewidth',1);
        plot3 ([b3(1) x],[(b3(2)) y],[(b3(3)) z],'color','green','Linewidth',1);
        plot3 (x,y,z,'*','color','black')
        plot3 ([x x],[y y],[0 z],'color','black','Linewidth',3);
        
function plotCircle3D(center,normal,radius,color)
    theta=0:0.01:2*pi;
    v=null(normal);
    points=repmat(center',1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
    plot3(points(1,:),points(2,:),points(3,:),'color',color);
end
  
     

