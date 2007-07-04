clear();

getf('rotations.sci');
getf('imu.sci');
getf('ekf.sci');
getf('tilt_utils.sci');

use_sim = 1;

if (use_sim),
  getf('quadrotor.sci');
  true_euler0 = [ 0.01; 0.25; 0.5]; 
  dt =  0.015625;
  [time, true_rates, true_eulers] = quadrotor_gen_roll_step(true_euler0, dt);
  [accel, mag, gyro] = imu_sim(time, true_rates, true_eulers);
else
  //filename = "../data/log_ahrs_test";
  filename = "../data/log_ahrs_bug";
  //filename = "../data/log_ahrs_roll";
  //filename = "../data/log_ahrs_yaw_pitched";
  [time, accel, mag, gyro] = imu_read_log(filename);
end


dt = time(2) - time(1);
[X0, P0] = tilt_init(150, accel, gyro);

X=[X0];
P=[P0];
M=[X0(1)];

for i=1:length(time)-1
  
  true_rate = gyro(1, i) - X(2,i);
  X0dot = [ true_rate
            0         ];
  // Jacobian of Xdot wrt X
  F = [ 0 -1
        0  0 ];
  // Process covariance noise    
  Q = [ 1e-5  0         
        0  8e-3 ];
  X0 = X(:, i);
  P0 = tilt_get_P(P, i);
  [Xpred, Ppred] = ekf_predict_continuous(X0, X0dot, dt, P0, F, Q);
//  [Xpred, Ppred] = ekf_predict_discrete(X0, X0dot, dt, P0, F, Q);
  
//  X = [X Xpred];
//  P = [P Ppred];
  
  measure = phi_of_accel(accel(:,i+1));
  M = [M measure];
  err = measure - Xpred(1);
  // Jacobian of measurement wrt X
  H = [ 1 0 ];
  // Measurement covariance noise
  R = 0.3;
  [Xup, Pup] = ekf_update(Xpred, Ppred, H, R, err);
  
  X = [X Xup];
  P = [P Pup];
  
end

tilt_display(time, X, P, M);






















