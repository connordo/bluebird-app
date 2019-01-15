clear
clc
%% Initial Parameters
% These initial parameters represent data that will be readily accessible
% on the MAV. Pixel Array, Field of View angle are specifications of the
% camera, and the normalized line of sight vector (l_cusp_c)  will be
% determined by either the autonomous detection program, or the manual
% interface clients.

% The X and Y pixels denote the location of the target in the image plane
% and are centered about the center of the image, with +x extending to the
% 'right' and +y extending 'down'
x_pixel = 0;
y_pixel = 0;

M = 256; %square pixel array (width and height if square)
Ex = 40; %x_pixel/M; %X pixels, normalized
Ey = 50; %y_pixel/M; %Y pixels, normalized
fov_ang = 200;%sym('fov_ang'); %field of View angle --> A6000 83* - 32* (in radians)
f = M/(2*tan(fov_ang/2)); %focal length  in units of pixels

l_cusp_c = 1/sqrt(Ex^2 + Ey^2 + f^2) * [Ex;Ey;f];

roll = 0; %sym('roll'); %Radians
pitch = 0; %sym('pitch'); %RadiansP
yaw = 0; %sym('yaw'); %Radians

alpha_az = 0;%sym('az'); %Azmuth Angle: Should equal yaw.
alpha_el = 0;%sym('el'); %Elevation Angle: "Pitch" of the gimbal

k_i = [0;0;1];

LatGroundStation = 40.174470;
LonGroundStation = -111.654497;
CoorGroundStation = [LatGroundStation, LonGroundStation];

LatMav = 40.175606;
LonMav = -111.652994;
CoorMav = [LatMav, LonMav];

LatMav2 = 40.172524;
LonMav2 = -111.657491;
CoorMav2 = [LatMav2, LonMav2];

MavLocationData = GPStoMeters(LatGroundStation, LonGroundStation, LatMav2, LonMav2);

%P vector should be in Meters
Pn = MavLocationData(1);
Pe = MavLocationData(2);
Pd = -60;%sym('Pd');
h = -Pd;%;sym('h');

%% Rotation Matrices
% The location of the target is defined in the camera coordinate frame
% system, and must be transformed into the inertial frame to provide GPS
% coordinates. This transformation is done using three rotation matrices:
% Moving Camera frame to Gimbal coordinates; Gimbal coordinates to body
% frame; body frame to inertial frame

% R_b_to_i
% Found on page 15 of Small Unmanned Aircraft

R_v_to_b = [cos(pitch)*cos(yaw) cos(pitch)*sin(yaw) -sin(pitch); ...
    sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw) sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw) sin(roll)*cos(pitch); ...
    cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw) cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw) cos(roll)*cos(pitch)];

R_b_to_v = R_v_to_b'; %R_v_b is a translation of R_i_b. It may work, but I may need one more step.
R_b_to_i = R_b_to_v;

% ------------------------------------------------------------------------
% R_g_to_b
% Found on page 227 of Small Unmanned Aircraft
R_b_to_g1 = [cos(alpha_az) sin(alpha_az) 0;...
    -sin(alpha_az) cos(alpha_az) 0;...
    0 0 1];

R_g1_to_g = [cos(alpha_el) 0 -sin(alpha_el);...
    0 1 0;...
    sin(alpha_el) 0 cos(alpha_el)];

R_b_to_g = R_g1_to_g*R_b_to_g1;

R_g_to_b = R_b_to_g';

% ------------------------------------------------------------------------
% R_c_to_g
% Found on page 227 of Small Unmanned Aircraft
R_g_to_c = [0 1 0;...
    0 0 1;...
    1 0 0];
R_c_to_g = R_g_to_c';

% For simplicity, the three Rotation matrices are combined into one below
RbiRbgRcg = R_b_to_i * R_g_to_b * R_c_to_g;

%% Geolocation Algorithm
% The algorithm uses the line of sight vector in the camera frame and
% rotates it into the inertial frame. There, it is multiplied by a scaler
% (height) and divided by the projection of the transformed line of sight
% vector onto the unit vector k_i in the inertial frame
P_i_mav = [Pn;Pe;Pd];
P_i_obj = P_i_mav + h*(RbiRbgRcg*l_cusp_c)/(dot(k_i,(RbiRbgRcg*l_cusp_c)));

%% Publish
disp("Coordinates of the MAV");
disp(P_i_mav);

disp("Coordinates of the Target");
disp(P_i_obj);

TargetCoordinates = MeterstoGPS(LatGroundStation, LonGroundStation, P_i_obj(1), P_i_obj(2));
disp("Target Coordinates ");
disp(TargetCoordinates(1)+", "+TargetCoordinates(2));