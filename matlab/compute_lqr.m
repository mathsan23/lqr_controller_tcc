%% LQR implementation

% ROSPlane loop time
Ts = 0.01;

% [Ad_lat,Bd_lat] = ssdata(c2d(ss(A_lat, B_lat, [],[]), Ts, 'zoh'));
% 
% qv=1; qp=1; qr=1; qphi=1; qpsi=1;
% Q_lat = diag([qv qp qr qphi qpsi]);      % 5x5
% R_lat = diag([1 1]);                     % for [delta_a; delta_r]
% 
% Klat = dlqr(Ad_lat, Bd_lat, Q_lat, R_lat); 
% 
% [Ad_lon,Bd_lon] = ssdata(c2d(ss(A_lon, B_lon, [],[]), Ts, 'zoh'));
% 
% qu=1; qw=1; qq=1; qtheta=1; qh=1;
% Q_lon = diag([qu qw qq qtheta qh]);       % 5x5
% R_lon = diag([1 1]);                      % for [delta_e; delta_t]
% 
% Klon = dlqr(Ad_lon, Bd_lon, Q_lon, R_lon); 


% Augment the state with the integral of the error of course angle psi
H_lat = [0 0 0 0 1];
 
Alat_aug = [A_lat zeros(5,1); H_lat 0];
Blat_aug = [B_lat; 0 0];

[Ad_lat,Bd_lat] = ssdata(c2d(ss(Alat_aug, Blat_aug, [],[]), Ts, 'zoh'));

% State order: [v p r phi chi Ichi], input: delta_a
% Normal
Qn = diag([1.0, 1.2, 1.8, 20.0, 24.0, 2.0]);
Rn = 50.0;
K_normal = dlqr(Ad_lat, Bd_lat, Qn, Rn);

% Slow
Qs = diag([0.3, 0.3, 0.5,  6.0,  8.0, 0.5]);
Rs = 100.0;
K_slow = dlqr(Ad_lat, Bd_lat, Qs, Rs);

% Fast (extra emphasis on phi/rates/chi)
Qf = diag([4.0, 6.0, 8.0, 80.0, 60.0, 6.0]);
Rf = 5.0;
K_fast = dlqr(Ad_lat, Bd_lat, Qf, Rf);

% Discrete LQR on the augmented model
Klat = K_slow;


% Declare trim Variables
u_star = x_trim(4);
v_star = x_trim(5);
w_star = x_trim(6);
Va_star = sqrt(u_star^2 + v_star^2 + w_star^2);

% H_lon per book (rows: [h; Va])
H_lon = [0, 0, 0, 0, 1;
          u_star/Va_star, w_star/Va_star, 0, 0, 0 ];

% Augmented continuous-time system
Alon_aug = [A_lon,             zeros(5,2);
            H_lon,             zeros(2,2)];
Blon_aug = [B_lon;
         zeros(2,2)];

% Discretize (ZOH)
[Ad_lon,Bd_lon] = ssdata(c2d(ss(Alon_aug, Blon_aug, [], []), Ts, 'zoh'));


% Weights (start from Bryson-like, then tune)
% States order in augmented system: [u w q theta h  I_h  I_Va]
qu    = 0.16;   % u      (m/s)
qw    = 0.6;    % w      (m/s)
qq    = 2.0;    % q      (rad/s)
qth   = 80.0;   % theta  (rad)
qh    = 0.05;   % h      (m)
qI_h  = 3.0;    % integral of h error
qI_Va = 4.0;    % integral of Va error

Q_lon = diag([qu, qw, qq, qth, qh, qI_h, qI_Va]);

% Inputs [delta_e, delta_t]
Re = 12.0;      % elevator effort penalty
Rt = 15.0;      % throttle effort penalty
R_lon = diag([Re, Rt]);

% Discrete LQR on augmented system
Klon = dlqr(Ad_lon, Bd_lon, Q_lon, R_lon);   % K is 2x7

% Helper to print a line 
printLine = @(name, val) fprintf('  params_.declare_double("%s", % .4f);\n', name, val);

% Helper to print a line with value and units
printDecl = @(name, val, unit) fprintf('  params_.declare_double("%s", % .4f); // %s\n', name, val, unit);

% Actuators block
fprintf('  // Actuators Trim\n');
printLine('lqr_trim_de', u_trim(1));
printLine('lqr_trim_dt', u_trim(4)); 
printLine('lqr_trim_da', u_trim(2)); 
printLine('lqr_trim_dr', u_trim(3));
fprintf('\n');

% Longitudinal block
fprintf('  // Longitudinal Trim States LQR (u, w, q, theta, h)\n');
printDecl('lqr_trim_u',      x_trim(4),     'm/s');
printDecl('lqr_trim_w',      x_trim(6),     'm/s');
printDecl('lqr_trim_q',      x_trim(11),    'rad/s');
printDecl('lqr_trim_theta',  x_trim(8),     'rad');
printDecl('lqr_trim_h',      x_trim(3),     'm');
fprintf('\n');

% Lateral block
fprintf('  // Lateral Trim States LQR (v, p, r, phi, chi)\n');
printDecl('lqr_trim_v',      x_trim(5),   'm/s');
printDecl('lqr_trim_p',      x_trim(10),  'rad/s');
printDecl('lqr_trim_r',      x_trim(12),  'rad/s');
printDecl('lqr_trim_phi',    x_trim(7),   'rad');
printDecl('lqr_trim_chi',    x_trim(9),   'rad');
fprintf('\n');

% Longitudinal gains (elevator = first row, throttle = second row) 
fprintf('  // Longitudinal K gains for elevator de and throtle dt\n'); 
printLine('lqr_ke_u', Klon(1,1)); printLine('lqr_ke_w', Klon(1,2)); 
printLine('lqr_ke_q', Klon(1,3)); printLine('lqr_ke_theta', Klon(1,4)); 
printLine('lqr_ke_h', Klon(1,5)); printLine('lqr_ke_Ih', Klon(1,6));
printLine('lqr_ke_IVa', Klon(1,7)); fprintf('\n'); 
printLine('lqr_kt_u', Klon(2,1)); printLine('lqr_kt_w', Klon(2,2)); 
printLine('lqr_kt_q', Klon(2,3)); printLine('lqr_kt_theta', Klon(2,4)); 
printLine('lqr_kt_h', Klon(2,5)); printLine('lqr_kt_Ih', Klon(2,6));
printLine('lqr_kt_IVa', Klon(2,7)); fprintf('\n');

% Lateral gains (aileron = first row, rudder = second row) 
fprintf('  // Lateral K gains for aileron ka and rudder kr\n'); 
printLine('lqr_ka_v', Klat(1,1)); printLine('lqr_ka_p', Klat(1,2)); 
printLine('lqr_ka_r', Klat(1,3)); printLine('lqr_ka_phi', Klat(1,4)); 
printLine('lqr_ka_chi', Klat(1,5)); printLine('lqr_ka_Ichi', Klat(1,6)); 
% fprintf('\n'); 
% printLine('lqr_kr_v', Klat(2,1)); printLine('lqr_kr_p', Klat(2,2)); 
% printLine('lqr_kr_r', Klat(2,3)); printLine('lqr_kr_phi', Klat(2,4)); 
% printLine('lqr_kr_chi', Klat(2,5));