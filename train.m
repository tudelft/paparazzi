function train()

MAX_SAMPLES = 25000;
start_sample = 1;

BIAS = true;
PRIOR = true;
TEST_SET = false;
test_ratio = 0.30;

weights = true; % whether we have onboard weights we want to compare to the current learning:
if(weights)
    w = load('Weights_00000.dat');
end

rand('seed', 1)

% structure A:
% 1) height
% 2) gain
% 3) cov div
% 4-end) textons
A = load('Training_set_00000.dat');
% A = load('Training_set_landing_mat_oscillate_different_heights.dat');
n_samples = size(A,1);
if(TEST_SET)
    n_training = round((1-test_ratio) * n_samples);
    n_test = n_samples - n_training;
    start_ind = floor(rand(1) * (n_samples - n_test));
    A_test = A(start_ind:start_ind+n_test, :);
    A = [A(1:start_ind-1, :); A(start_ind+n_test+1:end, :)];
    % A_test = A(n_training+1:end, :);
    % A = A(1:n_training, :);
end

% **************************************
% learn a mapping from features to gain:
% **************************************

A = A(start_sample:min([MAX_SAMPLES, size(A,1)]), :);
b = A(:,2); % targets
f = A(:,4:end); % features
if(BIAS)
    AA = [f, ones(size(A,1),1)];
else
    AA = f;
end
if(~PRIOR)
    x = AA \ b;
else
   alpha = 1; %10;
   x = inv(AA' * AA + alpha * eye(size(AA, 2))) * AA' * b; 
end

% store the resulting weights:
fid = fopen('Weights_MATLAB.dat', 'w');
for i = 1:length(x)-1
    fprintf(fid, '%f ', x(i));
end
fprintf(fid, '%f', x(end));
fclose(fid);

% evaluate the resulting estimates and compare them with the weights
% learned onboard:
y = AA * x; % x is the weights
height_gain_estimate = y;
fprintf('MAE on training set = %f\n', mean(abs(y-b)));
if(weights)
    Z = AA * w';
end
figure(); plot(y); hold on; plot(b);
if(weights)
    plot(Z);
    title('Height on training set')
    legend({'height gain estimate', 'height gain', 'onboard gain estimate'});
else
    title('Height on training set')
    legend({'height gain estimate', 'height gain'});
end


figure(); plot(smooth(y, 20)); hold on; plot(b);
if(weights)
    plot(smooth(Z, 20));
    legend({'height gain estimate', 'height gain', 'onboard gain estimate'});
    title('Smoothed Height')
else
    legend({'height gain estimate', 'height gain'});
    title('Smoothed Height')
end

figure();
bar(x, 'FaceColor', [1 0 0]); % hold on; bar(w);
title('Weights learned in MATLAB');

figure();
plot(A(:,1)); hold on; plot(A(:,2));
legend({'Height', 'Gain'});
title('Height and adaptive gain in the data set')

if(TEST_SET)
    % MAE on test set:
    f_test = A_test(:,4:end);
    if(BIAS)
        AA_test = [f_test, ones(size(A_test,1),1)];
    else
        AA_test = f_test;
    end
    y_test = AA_test * x;
    b_test = A_test(:, 2);
    fprintf('MAE on test set = %f\n', mean(abs(y_test-b_test)));
    if(weights)
        Z_test = AA_test * w';
    end
    figure(); plot(y_test); hold on; plot(b_test);
    if(weights)
        plot(Z_test);
        title('Height on test set')
        legend({'height gain estimate', 'height gain', 'onboard gain estimate'});
    else
        title('Height on test set')
        legend({'height gain estimate', 'height gain'});
    end
end

% ***********************************
% now learn a function to learn sonar 
% ***********************************

b = A(:,1);
f = A(:,4:end);
if(BIAS)
    AA = [f, ones(size(A,1),1)];
else
    AA = f;
end
x = AA \ b;
y = AA * x;
figure(); plot(smooth(y, 20)); hold on; plot(b);

b = A(:,1);
f = height_gain_estimate;
AA = [f, ones(size(A,1),1)];
x = AA \ b;
y = AA * x;
plot(smooth(y,20));
title('Sonar height')
legend({'estimate trained with sonar', 'sonar', 'scaled gain estimate'});

figure();
plot(A(:, 3));
title('Cov div');

entr = getEntropies(f);
p_peak = zeros(1, size(f,2));
p_peak(1) = 1;
min_entr = getEntropy(p_peak);
p_uniform = ones(1, size(f,2)) ./ size(f,2);
max_entr = getEntropy(p_uniform);
figure();
histogram(entr, 30);
title(['Entropies: min = ' num2str(min_entr) ', max = ' num2str(max_entr)]);

function entropies = getEntropies(f)
n_el = size(f,1);
entropies = zeros(n_el,1);
for el = 1:n_el
   entropies(el) = getEntropy(f(el, :));
end