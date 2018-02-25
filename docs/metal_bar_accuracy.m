data_double = [];
data_policy = [
];
data_centroid = [

];


data_basic = [
0
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
1
1
0
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
0
0
0
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


]; 

data_raw_image = [
]; 

data_smirror = [1
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
1
1


];
data_policy_no_smirror = [];
% data_basic = data_basic(1:66);
success_basic = cumsum(data_basic);
success_centroid = cumsum(data_centroid);
success_double = cumsum(data_double);
success_policy = cumsum(data_policy);
success_smirror = cumsum(data_smirror);
success_policy_no_smirror = cumsum(data_policy_no_smirror);
success_raw_image = cumsum(data_raw_image);

figure;hold on;

plot(success_basic'./(1:length(data_basic)), 'c');
plot(success_double'./(1:length(data_double)), 'g');
plot(success_centroid'./(1:length(data_centroid)), 'r');
plot(success_policy_no_smirror'./(1:length(data_policy_no_smirror)), 'k');
plot(success_policy'./(1:length(data_policy)), 'b');
plot(success_smirror'./(1:length(data_smirror)), 'c');
plot(success_raw_image'./(1:length(data_raw_image)), 'm');

% 95% invertal of confidence
[phat,pci] = binofit(sum(data_basic),length(data_basic));
pci2(1) = phat - sqrt(phat*(1-phat)/length(data_basic))*normcdf(1-0.05/2);
pci2(2) = phat + sqrt(phat*(1-phat)/length(data_basic))*normcdf(1-0.95/2);
%errorbar(length(data_basic),phat,phat-pci(1),pci(2)-phat, 'c');
errorbar(length(data_basic),phat,phat-pci2(1),pci2(2)-phat, 'c');
[phat,pci] = binofit(sum(data_double),length(data_double));
pci2(1) = phat - sqrt(phat*(1-phat)/length(data_double))*normcdf(1-0.05/2);
pci2(2) = phat + sqrt(phat*(1-phat)/length(data_double))*normcdf(1-0.95/2);
%errorbar(length(data_double),phat,phat-pci(1),pci(2)-phat, 'g');
errorbar(length(data_double),phat,phat-pci2(1),pci2(2)-phat, 'g');
[phat,pci] = binofit(sum(data_centroid),length(data_centroid));
pci2(1) = phat - sqrt(phat*(1-phat)/length(data_centroid))*normcdf(1-0.05/2);
pci2(2) = phat + sqrt(phat*(1-phat)/length(data_centroid))*normcdf(1-0.95/2);
%errorbar(length(data_centroid),phat,phat-pci(1),pci(2)-phat, 'r');
errorbar(length(data_centroid),phat,phat-pci2(1),pci2(2)-phat, 'r');
[phat,pci] = binofit(sum(data_policy_no_smirror),length(data_policy_no_smirror));
pci2(1) = phat - sqrt(phat*(1-phat)/length(data_policy_no_smirror))*normcdf(1-0.05/2);
pci2(2) = phat + sqrt(phat*(1-phat)/length(data_policy_no_smirror))*normcdf(1-0.95/2);
%errorbar(length(data_policy_no_smirror),phat,phat-pci(1),pci(2)-phat)
errorbar(length(data_policy_no_smirror),phat,phat-pci2(1),pci2(2)-phat, 'k')
[phat,pci] = binofit(sum(data_policy),length(data_policy));
pci2(1) = phat - sqrt(phat*(1-phat)/length(data_policy))*normcdf(1-0.05/2);
pci2(2) = phat + sqrt(phat*(1-phat)/length(data_policy))*normcdf(1-0.95/2);
%errorbar(length(data_policy),phat,phat-pci(1),pci(2)-phat, 'b')
errorbar(length(data_policy),phat,phat-pci2(1),pci2(2)-phat, 'b')
[phat,pci] = binofit(sum(data_raw_image),length(data_raw_image));
pci2(1) = phat - sqrt(phat*(1-phat)/length(data_raw_image))*normcdf(1-0.05/2);
pci2(2) = phat + sqrt(phat*(1-phat)/length(data_raw_image))*normcdf(1-0.95/2);
% errorbar(length(data_raw_image),phat,phat-pci(1),pci(2)-phat, 'm')
errorbar(length(data_raw_image),phat,phat-pci2(1),pci2(2)-phat, 'm')
[phat,pci] = binofit(sum(data_smirror),length(data_smirror));
pci2(1) = phat - sqrt(phat*(1-phat)/length(data_smirror))*normcdf(1-0.05/2);
pci2(2) = phat + sqrt(phat*(1-phat)/length(data_smirror))*normcdf(1-0.95/2);
% errorbar(length(data_smirror),phat,phat-pci(1),pci(2)-phat, 'c')
errorbar(length(data_smirror),phat,phat-pci2(1),pci2(2)-phat, 'c')

legend('Basic', 'Double','Centroid','No smirror', 'Smirror','Raw image + Smirror')
