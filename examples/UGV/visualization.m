load('data.csv');

% Plot positions
figure;
title('Position Estimate');
plot(data(:,[1,5,9]),data(:,[2,6,10]));
legend({'True Position', 'Prediction only', 'UKF Estimate'});
xlabel('x');
ylabel('y');

% Estimation error (euclidian distance)
figure;
title('Estimate errors from true position (Euclidian distance)');
ekf_error = sqrt((data(:,1)-data(:,7)).^2 + (data(:,2)-data(:,8)).^2);
ukf_error = sqrt((data(:,1)-data(:,10)).^2 + (data(:,2)-data(:,11)).^2);
plot([ekf_error, ukf_error]);
legend({'EKF Error', 'UKF Error'});
xlabel('Iteration')
ylabel('Error')
