%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function animate(positions,angles,l)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
axle_x = [-l/2 0 0;
           l/2 0 0];
axle_y = [0 -l/2 0;
          0  l/2 0];
      
dmax = max([max(positions),l]);
r = 0.1*l; %radius of propellers
ang = linspace(0,2*pi);
x_circle = r*cos(ang);
y_circle = r*sin(ang);
z_circle = zeros(1,length(ang));
propeller = [x_circle',y_circle',z_circle'];
[p1,q1] = size(propeller);
[p2,q2] = size(axle_x);
[mm,nn] = size(angles);
for ii=1:mm
    x = positions(ii,1);
    y = positions(ii,2);
    z = positions(ii,3);
    phi = angles(ii,1); 
    theta = angles(ii,2);
    psi = angles(ii,3);
    R = get_rotation(phi,theta,psi);
    
    for i=1:p2
        r_body = axle_x(i,:)';
        r_world = R*r_body;
        new_axle_x(i,:) = r_world';
    end
    new_axle_x = [x y z] +new_axle_x;
    
    for i=1:p2
        r_body = axle_y(i,:)';
        r_world = R*r_body;
        new_axle_y(i,:) = r_world';
    end
    new_axle_y = [x y z] +new_axle_y;
    
    for i=1:p1
        r_body = propeller(i,:)';
        r_world = R*r_body;
        new_propeller(i,:) = r_world'; 
    end
    new_propeller1 = new_axle_x(1,:) + new_propeller;
    new_propeller3 = new_axle_x(2,:) + new_propeller;
    new_propeller2 = new_axle_y(1,:) + new_propeller;
    new_propeller4 = new_axle_y(2,:) + new_propeller;
     line(new_axle_x(:,1),new_axle_x(:,2),new_axle_x(:,3),'Linewidth',2); hold on;
     line(new_axle_y(:,1),new_axle_y(:,2),new_axle_y(:,3),'Linewidth',2);
    patch(new_propeller1(:,1),new_propeller1(:,2),new_propeller1(:,3),'r');
    patch(new_propeller2(:,1),new_propeller2(:,2),new_propeller2(:,3),'g');
    patch(new_propeller3(:,1),new_propeller3(:,2),new_propeller3(:,3),'b');
    patch(new_propeller4(:,1),new_propeller4(:,2),new_propeller4(:,3),'c');
    axis(1.2*[-dmax dmax -dmax dmax -dmax dmax]);
    xlabel('x'); ylabel('y'); zlabel('z');
    %view(0,90)
    view(3)
    pause(0.01)
    if (ii~=mm)
        clf
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function R = get_rotation(phi,theta,psi)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% uses 3-2-1 euler angles
R_x = [1    0       0; ...
       0  cos(phi) -sin(phi); ...
       0  sin(phi) cos(phi)];
   
R_y = [cos(theta)  0   sin(theta); ...
       0           1         0; ...
      -sin(theta) 0   cos(theta)]; 
   
R_z = [cos(psi) -sin(psi)  0; ...
       sin(psi)  cos(psi)  0; ...
       0           0       1];  
R = R_z*R_y*R_x;