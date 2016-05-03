logName = '../../logs/testlog.txt';

logFile = fopen(logName, 'r');
titles = fscanf(logFile, '%s %s %s\n', [3 1]);
values = fscanf(logFile, '%f %f %f\n', [3 Inf]);

t = values(1,:);
roll = values(2,:);
rollRate = values(3,:);

plot(t, roll, t, rollRate)