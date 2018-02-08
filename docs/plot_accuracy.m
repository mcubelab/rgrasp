data_double = [1
0
0
0
1
0
0
0
1
0
0
1
1
1
1
0
0
1
0
0
0
0
0
0
1
1
0
0
0
0
0
0
0
0
0
0
1
1
1
1
0
0
0
0
1
0
1
0
1
0
1
1
0
0
0
0
0
0
1
1
1
0
0
0
1
0
1
0
1
0
0
0
0
0
0
0
0
0
1
1
0
0
0
0
0
0
1
0
0
1
0
0
0
1
0
0
0
0
0
0
0
0
0
0
1
0
0
1
0
0
0
0
0
0
1
0
1
1
0
0
0
0
0
1
0
0
0
0
0
0
0
0
0
1
1
0
0
1
0
0
0
0
1
0
0
0
1
1
1
0
1
1
1
1
0
1
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
1
1
1
0
1
0
1
0
0
0
0
0
0
0
1
0
0
0
1
0
1
0
0
1
0
0
0
0
0
1
0
1
0
1
0
1
1
0
0
0
0
0
0
0
0
0
0
1
1
1
0
0
0
0
1
0
0
0
1
0
0
0
0
1
0
0
0
0
0
1
0
0
0
1
0
0
0
1
0
0
0
0
0
1
0
0
0
1
0
1
0
1
0
0
0
0
0
0
1
0
];
data_policy = [1
1
0
0
1
1
1
1
1
1
0
1
1
1
1
1
0
0
0
0
1
0
1
0
1
1
1
0
1
0
1
0
0
0
0
1
0
0
0
0
0
1
1
1
1
1
1
0
0
0
0
1
1
1
1
0
0
1
0
0
0
1
1
0
0
0
];
data = [1
0
1
1
1
0
1
1
1
1
1
0
1
1
0
0
1
1
1
1
0
0
1
1
1
0
0
1
1
1
1
1
0
1
1
1
0
1
1
0
1
0
0
1
0
1
0
1
0
0
1
1
1
0
1
1
1
1
1
1
0
0
0
0
1
0
0
1
0
0
1
1
0
1
0
1
1
1
1
1
1
1
0
1
1
1
0
0
0
1
1
1
0
1
1
1
0
1
0
0
0
0
0
0
1
0
1
1
0
0
1
0
1
1
1
0
0
1
1
1
1
0
1
0
1
0
1
0
0
0
1
0
1
1
0
0
1
0
0
0
0
1
1
1
0
1
1
0
0
0
1
1
1
1
0
1
1
0
1
1
1
1
1
1
0
0
1
0
0
1
0
0
0
1
1
1
0
0
0
1
0
0
1
1
];

success = cumsum(data);
success_double = cumsum(data_double);
success_policy = cumsum(data_policy);


figure;
% plot(success'./(1:length(data)));
hold on;
plot(success_double'./(1:length(data_double)));
hold on;
plot(success_policy'./(1:length(data_policy)));

