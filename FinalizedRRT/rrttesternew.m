clear
close all
tries = 100;
load('MANUSFile.mat','MANUS');
load('oldPtCloudData.mat', 'ptCloud');
 tobj = RRTClass(MANUS,ptCloud);
for i = 1:tries
    tic
    %tobj = RRTClass(MANUS,ptCloud);
    [~,success(i)] = tobj.findRoute();
    t(i) = toc;
    obj(i) = tobj;
    
end
t = t';
avg = mean(t);
min = min(t);
max = max(t);
time_Return = [t,success'];
success(success == 0) = [];
total = size(success);
success(success == 3) = [];
percentsuccess = (size(success)/total)*100;
disp(['Success Rate = ',num2str(percentsuccess),'%']);
disp(['Total Time = ',num2str(sum(t)),'seconds']);
disp(['Minimum time = ',num2str(min),' seconds']);
disp(['Average time = ',num2str(avg),' seconds']);
disp(['Maximum time = ',num2str(max),' seconds']);