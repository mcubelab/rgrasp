data_basic = [
0
1
0
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
0
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
0
1
0
1
0
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
1
1
0
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
1
1
0
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
1
1
0
0
0
1
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
0
1
1
1
0
1
1



]; 

data_smirror = [
    
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
1
0
1
1
1
1
1
1
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
1
1
0
1
0
1
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
0
0
1
1
0
1
1
1
1
0

];

success_basic = cumsum(data_basic);
success_smirror = cumsum(data_smirror);

figure;hold on;
plot(success_basic'./(1:length(data_basic)), 'b');
plot(success_smirror'./(1:length(data_smirror)), 'g');

% 95% invertal of confidence
[phat,pci] = binofit(sum(data_basic),length(data_basic));
pci2(1) = phat - sqrt(phat*(1-phat)/length(data_basic))*normcdf(1-0.05/2);
pci2(2) = phat + sqrt(phat*(1-phat)/length(data_basic))*normcdf(1-0.95/2);
%errorbar(length(data_basic),phat,phat-pci(1),pci(2)-phat, 'b');
errorbar(length(data_basic),phat,phat-pci2(1),pci2(2)-phat, 'b')
[phat,pci] = binofit(sum(data_smirror),length(data_smirror));
pci2(1) = phat - sqrt(phat*(1-phat)/length(data_smirror))*normcdf(1-0.05/2);
pci2(2) = phat + sqrt(phat*(1-phat)/length(data_smirror))*normcdf(1-0.95/2);
% errorbar(length(data_smirror),phat,phat-pci(1),pci(2)-phat, 'g')
errorbar(length(data_smirror),phat,phat-pci2(1),pci2(2)-phat, 'g')

legend('Basic', 'Smirror')
