%Robotica autónoma 
%Máster sistemas y control UNED&UCM
%Curso 2020-2021
%Autor: Jonatan Rodriguez jonataro@ucm.es
%posicionamiento de un robot a partir de las señales de tres balizas
%ultrasonicas

clear all
close all

%beacon positions
    b1 = [0,2,0.5];
    b2 = [4,0,2];
    b3 = [6,4,2];
    h  = 0.5;
%speed of sound m/s
    v=340;
%Return time of signals each beacons
    t1 = 1.584e-02;
    t2 = 1.149e-2;
    t3 = 2.412e-2;
%times for (0, 0, 0.5) position
    %t1= 0.008318903308077;
    %t2= 0.020588235294118;
    %t3= 0.035416454643507;
% these times values are extracted from inverese logic script that extract 
%times form kwnon position:
%https://github.com/jonataro/Autonomous-Robots/blob/739f134d1a137202fdace76c77df992e6b9635fd/inverse_trilateration.m

%estimated distance from the emitter to to each beacon
    r1 = t1*v/2;
    r2 = t2*v/2;
    r3 = t3*v/2;
%mobile robot beacon distance to the floor
    z=0.5; %plane   = z == h; 
%Solución del sistema de ecuaciones no lineales
%Función Anónima hay que meter las ecuaciones en un vector y las varibales
%a resolver seran parte de este x=sys(1), y=sys(1), z=sys(1)
% en este caso z es constante se podria quitar una variable del sistema
% se conserva su forma por si quisiera dar un uso mas gneral
F = @(sys)[
    (sys(1)-b1(1))^2+(sys(2)-b1(2))^2+(sys(3)-b1(3))^2-r1^2;%sphere1 = (x-b1(1))^2 + (y-b1(2))^2 + (z-b1(3))^2 == r1^2;
    (sys(1)-b2(1))^2+(sys(2)-b2(2))^2+(sys(3)-b2(3))^2-r2^2;%sphere2 = (x-b2(1))^2 + (y-b2(2))^2 + (z-b2(3))^2 == r2^2;
    (sys(1)-b3(1))^2+(sys(2)-b3(2))^2+(sys(3)-b3(3))^2-r3^2;%sphere3 = (x-b3(1))^2 + (y-b3(2))^2 + (z-b3(3))^2 == r3^2;
    sys(3)-h % plane = z==h
];
sys0 = [0;0;0]; %Vector inicial para la la solucion del problema
sys = fsolve(F,sys0);  
 %visualization    
    hold on;
    %suelo
       [GroundX,GroundY] = meshgrid(0:6,0:4);
       GroundZ=0*GroundX;
       mesh(GroundX,GroundY,GroundZ,'Facecolor','none','EdgeColor','black')
       
    %balizas
        plot3 (b1(1),(b1(2)),(b1(3)),'o','color','blue')
        plot3 (b2(1),(b2(2)),(b2(3)),'o','color','red')
        plot3 (b3(1),(b3(2)),(b3(3)),'o','color','green')
        
    %"soportes" de las balizas
        plot3 ([b1(1) b1(1)],[(b1(2)) (b1(2))],[(b1(3)) 0],'color','blue');
        plot3 ([b2(1) b2(1)],[(b2(2)) (b2(2))],[(b2(3)) 0],'color','red');
        plot3 ([b3(1) b3(1)],[(b3(2)) (b3(2))],[(b3(3)) 0],'color','green');
        
    %esferas que comproenden las regiones posiblles donde se encuentra el robot:
        % mallado base de esfera unitaria 20 caras
            [X,Y,Z] = sphere(120);
        %esfera  baliza 1
            r1 = v*t1/2;
            X1 = X * r1;
            Y1 = Y * r1;
            Z1 = Z * r1;
            mesh(X1+b1(1),Y1+ b1(2),Z1+b1(3),'Facecolor','none','EdgeColor','b');
            
        %esfera  baliza 2   
            r2 = v*t2/2;
            X2 = X * r2;
            Y2 = Y * r2;
            Z2 = Z * r2;
            mesh(X2+b2(1),Y2+b2(2),Z2+b2(3),'Facecolor','none','EdgeColor','r');
        
        %esfera  baliza 3     
            r3 = v*t3/2;
            X3 = X * r3;
            Y3 = Y * r3;
            Z3 = Z * r3;
            mesh(X3+b3(1),Y3+b3(2),Z3+b3(3),'Facecolor','none','EdgeColor','g');
        
         %plano de corte en z(la altura de baliza es constante  
            RobotZ = 0.5 *ones(size(GroundX));
            mesh(GroundX,GroundY,RobotZ,'Facecolor','none','EdgeColor','magenta');
            
         %limitacion de los ejes de visualización para emular los limites
         %del almacen
            xlim([0 6]) 
            ylim([0 4])
            zlim([0 2])



