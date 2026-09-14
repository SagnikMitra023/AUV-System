%% =========================================================================
%  HOUSEKEEPING
% =========================================================================
close all; clear; clc;

%% =========================================================================
%  1. CONFIGURATION (TUNED FOR RAPID VELOCITY TRACKING)
% =========================================================================
params = struct();
params.Q_pos        = 1e-4;          
params.Q_vel        = 1e-2;           % Increased to allow state velocity to change fast
params.Q_vert       = 1e-6;          
params.R_dvl        = 1e-5;           % Trust DVL heavily to eliminate lag
params.R_gps_h      = 0.10;           
params.R_gps_v      = 0.10;
params.lpf_cutoff   = 4.0;            % Increased from 1.2Hz to 4.0Hz to capture real accelerations
params.max_vel      = 12.0;           
params.max_pos      = 10000;
params.heading_alpha = 0.95;          
params.gps_course_threshold = 0.5;    
params.turn_detection_threshold = 0.3; 
SAMPLE_PERIOD_IMU  = 0.01;

%% =========================================================================
%  2. LOAD DATA
% =========================================================================
data = readtable("C:\Users\ASUS\Downloads\20210428_1_0.csv");
N = height(data);

% FIX: Replaced typo 'mox' with the correct MATLAB function 'any'
if any(strcmpi(data.Properties.VariableNames, 'af'))
    acc = [data.af, data.al, data.au];
else
    acc = [data.ax, data.ay, data.az];
end

gyr = [data.wx, data.wy, data.wz];
dvl_vel = [data.vf, data.vl, data.vu];
yaw_raw   = data.yaw;      
pitch_raw = data.pitch;
roll_raw  = data.roll;
lat = data.lat_gps;
lon = data.lon_gps;
alt = data.alt_gps;
vn = data.vn;   ve = data.ve;
gps_speed = sqrt(vn.^2 + ve.^2);
gps_course = atan2(ve, vn);
samplePeriod = SAMPLE_PERIOD_IMU;

%% =========================================================================
%  3. GPS TO ENU
% =========================================================================
gt_pos = zeros(N, 3);
lat0 = lat(1); lon0 = lon(1); alt0 = alt(1);
a = 6378137; f = 1/298.257223563; e2 = f*(2-f);
N_radius = a / sqrt(1 - e2*sin(lat0*pi/180)^2);
R_e = N_radius * cos(lat0*pi/180);
for i = 1:N
    gt_pos(i,1) = (lon(i) - lon0) * R_e * pi/180;
    gt_pos(i,2) = (lat(i) - lat0) * a * pi/180;
    gt_pos(i,3) = alt(i) - alt0;
end

gt_pos_smooth = movmean(gt_pos, [5 0], 1);
gt_vel = zeros(N,3);
for i = 2:N
    gt_vel(i,:) = (gt_pos_smooth(i,:) - gt_pos_smooth(i-1,:)) / samplePeriod;
end
gt_vel(1,:) = gt_vel(2,:);

%% =========================================================================
%  4. HEADING OFFSET CALIBRATION
% =========================================================================
moving_mask = gps_speed > 2.0 & ~isnan(gps_course);
if sum(moving_mask) > 100
    yaw_diff = gps_course(moving_mask) - yaw_raw(moving_mask);
    yaw_diff = atan2(sin(yaw_diff), cos(yaw_diff));
    yaw_diff_clean = yaw_diff(abs(yaw_diff - median(yaw_diff)) < 3*std(yaw_diff));
    heading_offset = median(yaw_diff_clean);
    fprintf('Heading offset: %.2f deg\n', heading_offset*180/pi);
else
    heading_offset = 0;
    fprintf('Using heading offset = 0.\n');
end
heading = yaw_raw(1) + heading_offset;
heading = atan2(sin(heading), cos(heading)); 

gravity_bias = [0; 0; -9.81];

%% =========================================================================
%  5. EKF INITIALIZATION
% =========================================================================
x = zeros(6,1);
x(1:3) = gt_pos(1,:)';

% Initial velocity heading-only conversion
R_yaw_init = [cos(heading), -sin(heading), 0; sin(heading), cos(heading), 0; 0, 0, 1];
x(4:6) = R_yaw_init * [dvl_vel(1,1); dvl_vel(1,2); dvl_vel(1,3)];

P = eye(6) * 0.1;
Q = diag([params.Q_pos, params.Q_pos, params.Q_vert, ...
          params.Q_vel, params.Q_vel, params.Q_vel]);
      
R_dvl_mat = diag([params.R_dvl, params.R_dvl, params.R_dvl]); 
R_gps_horiz = diag([params.R_gps_h, params.R_gps_h]);
R_gps_vert = params.R_gps_v;

ekfState = zeros(N, 6);
alpha_lpf = (2*pi*samplePeriod*params.lpf_cutoff) / (2*pi*samplePeriod*params.lpf_cutoff + 1);
linAcc_filtered = [0; 0; 0];
heading_history = zeros(N,1);
gps_update_counter = 0;
dvl_update_counter = 0;

%% =========================================================================
%  6. MAIN LOOP
% =========================================================================
fprintf('Running Rapid-Tracking AUV EKF Filter...\n');
for i = 1:N
    dt = samplePeriod;
    
    if i > 1
        cr = cos(-roll_raw(i-1)); sr = sin(-roll_raw(i-1));
        cp = cos(-pitch_raw(i-1));
        if abs(cp) > 1e-4
            yaw_rate_enu = (sr/cp)*gyr(i-1,2) + (cr/cp)*gyr(i-1,3);
        else
            yaw_rate_enu = gyr(i-1,3);
        end
        
        heading_pred = heading + yaw_rate_enu * dt;
        heading_pred = atan2(sin(heading_pred), cos(heading_pred));
        
        if gps_speed(i) > params.gps_course_threshold && ~isnan(gps_course(i))
            if abs(yaw_rate_enu) > params.turn_detection_threshold
                alpha_turn = 0.99; 
            else
                alpha_turn = params.heading_alpha;
            end
            heading = alpha_turn * heading_pred + (1 - alpha_turn) * gps_course(i);
        else
            heading = heading_pred;
        end
        heading = atan2(sin(heading), cos(heading));
    end
    heading_history(i) = heading;
    
    % Heading-only rotation matrix for Earth-parallel sensors
    R_yaw = [cos(heading), -sin(heading), 0; 
             sin(heading),  cos(heading), 0; 
             0,             0,            1];
         
    R_full = auv2enu(heading, pitch_raw(i), roll_raw(i));
    
    % Transform accelerations
    acc_col = [acc(i,1); acc(i,2); acc(i,3)];
    acc_enu = R_full * acc_col;
    linAcc_raw = acc_enu - gravity_bias;  
    linAcc_filtered = alpha_lpf * linAcc_raw + (1 - alpha_lpf) * linAcc_filtered;
    
    % --- PREDICTION ---
    F = [eye(3), dt*eye(3); zeros(3), eye(3)];
    B = [0.5*dt^2*eye(3); dt*eye(3)];
    x = F * x + B * linAcc_filtered;
    P = F * P * F' + Q;
    
    % --- DVL UPDATE (Using Yaw-Only because columns are already Earth-Parallel) ---
    if ~any(isnan(dvl_vel(i,:)))
        dvl_col = [dvl_vel(i,1); dvl_vel(i,2); dvl_vel(i,3)];
        z_dvl_enu = R_yaw * dvl_col; 
        
        H_dvl = [zeros(3), eye(3)];
        y_dvl = z_dvl_enu - H_dvl * x;
        
        S_dvl = H_dvl * P * H_dvl' + R_dvl_mat; 
        K_dvl = P * H_dvl' / S_dvl;
        x = x + K_dvl * y_dvl;
        P = (eye(6) - K_dvl * H_dvl) * P;
        dvl_update_counter = dvl_update_counter + 1;
    end
    
    % --- GPS POSITION UPDATE ---
    if i > 1 && any(gt_pos(i,1:2) ~= gt_pos(i-1,1:2))
        H_gps_h = [1 0 0 0 0 0; 0 1 0 0 0 0];
        z_h = [gt_pos(i,1); gt_pos(i,2)];
        y_h = z_h - H_gps_h * x;
        S_h = H_gps_h * P * H_gps_h' + R_gps_horiz;
        K_h = P * H_gps_h' / S_h;
        x = x + K_h * y_h;
        P = (eye(6) - K_h * H_gps_h) * P;
        
        H_gps_v = [0 0 1 0 0 0];
        z_v = gt_pos(i,3);
        y_v = z_v - H_gps_v * x;
        S_v = H_gps_v * P * H_gps_v' + R_gps_vert;
        K_v = P * H_gps_v' / S_v;
        x = x + K_v * y_v;
        P = (eye(6) - K_v * H_gps_v) * P;
        
        gps_update_counter = gps_update_counter + 1;
    end
    
    vel_mag = norm(x(4:6));
    if vel_mag > params.max_vel, x(4:6) = x(4:6) * (params.max_vel / vel_mag); end
    for dim = 1:3
        if abs(x(dim)) > params.max_pos, x(dim) = sign(x(dim)) * params.max_pos; end
    end
    
    ekfState(i,:) = x';
end

%% =========================================================================
%  7. METRICS & VISUALIZATION
% =========================================================================
pos_est = ekfState(:,1:3);
vel_est = ekfState(:,4:6);
errors = sqrt(sum((gt_pos - pos_est).^2, 2));
rms_error = sqrt(mean(errors.^2));
max_error = max(errors);
final_drift = errors(end);
total_dist = sum(sqrt(sum(diff(gt_pos).^2, 2)));

fprintf('\n=============== FINAL PERFORMANCE ===============\n');
fprintf('RMS Error:      %.3f m\n', rms_error);
fprintf('Max Error:      %.3f m\n', max_error);
fprintf('Final Drift:    %.3f m\n', final_drift);
fprintf('Drift %% of dist: %.1f %%\n', (final_drift/total_dist)*100);
fprintf('=================================================\n');

figure('Name', 'Final Optimized Result', 'Position', [100, 100, 1400, 800]);
subplot(2,2,[1,3]);
plot3(pos_est(:,1), pos_est(:,2), pos_est(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(gt_pos(:,1), gt_pos(:,2), gt_pos(:,3), 'k--', 'LineWidth', 1.5);
grid on; axis equal; view(3);
xlabel('East (m)'); ylabel('North (m)'); zlabel('Altitude (m)');
title('Optimized AUV Trajectory Tracking');
legend('EKF Track', 'GPS Reference', 'Location', 'best');

subplot(2,2,2);
speed_est = sqrt(sum(vel_est.^2, 2));
plot((1:N)*samplePeriod, speed_est, 'r-', 'LineWidth', 1.5); hold on;
plot((1:N)*samplePeriod, gps_speed, 'k--', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('Speed (m/s)');
legend('EKF Speed', 'GPS Speed');
title('Velocity Tracking Response');

subplot(2,2,4);
plot((1:N)*samplePeriod, errors, 'g-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Position Error (m)');
title(sprintf('RMS: %.2f m, Max Error Optimized', rms_error));

fprintf('✅ System Normalized.\n');

%% =========================================================================
%  ROTATION LOCAL FUNCTION
% =========================================================================
function R = auv2enu(yaw, pitch, roll)
    p = -pitch; 
    r = -roll;
    Rz = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
    Rx = [1, 0, 0; 0, cos(p), -sin(p); 0, sin(p), cos(p)];
    Ry = [cos(r), 0, sin(r); 0, 1, 0; -sin(r), 0, cos(r)];
    R = Rz * Rx * Ry; 
end