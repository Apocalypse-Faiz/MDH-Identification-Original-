function outputDir = run_original_simulation(seed,runName)
% Execute byte-identical copies of the original source, then export artifacts.
% Diagnostics below do NOT participate in the original parameter fitting.
if nargin<1,seed=42;end
assert(isscalar(seed) && isfinite(seed) && seed>=0 && seed==floor(seed) && seed<=2^32-1,'Invalid seed.');
if nargin<2,runName=sprintf('seed%d_%s',seed,datestr(now,'yyyymmdd_HHMMSS'));end
assert(~isempty(regexp(runName,'^[A-Za-z0-9_-]+$','once')),'Use a simple run-directory name.');
repo=fileparts(fileparts(mfilename('fullpath')));
outputDir=fullfile(repo,'results',runName);
assert(~isfolder(outputDir),'Run directory already exists; choose a new name.');
assert(~isempty(which('Link')) && ~isempty(which('SerialLink')), ...
    'Install/configure Peter Corke Robotics Toolbox for MATLAB first.');
folders={'execution','original_outputs','data','diagnostics','figures'};
for i=1:numel(folders),mkdir(fullfile(outputDir,folders{i}));end
executionDir=fullfile(outputDir,'execution');
copyfile(fullfile(repo,'main.m'),fullfile(executionDir,'main.m'));
copyfile(fullfile(repo,'transformation_mdh.m'),fullfile(executionDir,'transformation_mdh.m'));
oldFolder=pwd;oldPath=path;
oldVisible=get(groot,'defaultFigureVisible');
oldAxesFont=get(groot,'defaultAxesFontName');oldTextFont=get(groot,'defaultTextFontName');
cleanup=onCleanup(@()restore_session(oldFolder,oldPath,oldVisible,oldAxesFont,oldTextFont));
set(groot,'defaultFigureVisible','off','defaultAxesFontName','Microsoft YaHei','defaultTextFontName','Microsoft YaHei');
diary(fullfile(outputDir,'console.log'));
fprintf('Original prototype; seed=%d; MATLAB=%s\n',seed,version);
cd(executionDir);
fprintf('Executing source snapshot: %s\n',which('main'));
evalin('base',sprintf('rng(%d,''twister''); main;',seed));
names={'Alpha_nominal','A_nominal','D_nominal','Theta_nominal', ...
    'delta_params_true','delta_params','alpha_identified','a_identified','d_identified','theta_identified', ...
    'Q','X_measured','Q_test','X_test_true','X_test_identified','position_errors','nominal_errors', ...
    'error_history','lambda_history','actual_iterations','N','N_test','max_iterations','tolerance'};
s=struct();
for i=1:numel(names),s.(names{i})=evalin('base',names{i});end
save(fullfile(outputDir,'data','workspace.mat'),'-struct','s');
movefile(fullfile(executionDir,'mdh_identification_results.mat'),fullfile(outputDir,'original_outputs','mdh_identification_results.mat'));
movefile(fullfile(executionDir,'identification_results.txt'),fullfile(outputDir,'original_outputs','identification_results.txt'));
figures=findall(groot,'Type','figure');[~,order]=sort([figures.Number]);figures=figures(order);
assert(numel(figures)==2,'Expected the robot visualization and original summary.');
for i=1:numel(figures)
    drawnow;
    export_figure(figures(i),fullfile(outputDir,'figures',sprintf('original_figure_%02d',i)));
end
p0=[s.Alpha_nominal,s.A_nominal,s.D_nominal,s.Theta_nominal];
pt=p0+s.delta_params_true;pf=p0+s.delta_params;
[nominalTrain,~]=positions(p0,s.Q,[0,0,0]);
[fittedTrain,~]=positions(pf,s.Q,[0,0,0]);
[~,trueR]=positions(pt,s.Q_test,[0,0,0]);
[~,fittedR]=positions(pf,s.Q_test,[0,0,0]);
orientationDeg=rotation_distances(trueR,fittedR)*180/pi;
J0=diagnostic_jacobian(p0,s.Q,[0,0,0]);Jt=diagnostic_jacobian(pt,s.Q,[0,0,0]);
singular0=svd(J0,'econ');singularT=svd(Jt,'econ');
scales=[ones(1,6)*pi/1800,ones(1,12)*0.001,ones(1,6)*pi/1800];
scaled0=svd(J0.*scales,'econ');scaledT=svd(Jt.*scales,'econ');
[trueTrain,~]=positions(pt,s.Q,[0,0,0]);
p6=pt;p6(24)=p6(24)+5*pi/180;
[changed6,changed6R]=positions(p6,s.Q_test,[0,0,0]);
[trueTest,trueTestR]=positions(pt,s.Q_test,[0,0,0]);
pd=pt;pd(14:15)=pd(14:15)+0.003;
changedD=positions(pd,s.Q,[0,0,0]);
Joff=diagnostic_jacobian(p0,s.Q,[0.1,0,0]);
robot=evalin('base','robot');rtbPose=robot.fkine(s.Q(1,:));
m=struct('seed',seed,'matlab_version',version,'train_count',s.N,'test_count',s.N_test, ...
    'iterations',s.actual_iterations,'measurement_noise_enabled',false, ...
    'initial_train_rms_mm',s.error_history(1)*1000,'final_train_rms_mm',s.error_history(end)*1000, ...
    'recomputed_train_rms_mm',sqrt(mean(sum((fittedTrain-s.X_measured).^2,2)))*1000, ...
    'nominal_test_mean_mm',mean(s.nominal_errors)*1000,'nominal_test_rms_mm',sqrt(mean(s.nominal_errors.^2))*1000, ...
    'identified_test_mean_mm',mean(s.position_errors)*1000,'identified_test_rms_mm',sqrt(mean(s.position_errors.^2))*1000, ...
    'identified_test_max_mm',max(s.position_errors)*1000,'identified_test_std_mm',std(s.position_errors)*1000, ...
    'mean_position_improvement_percent',100*(1-mean(s.position_errors)/mean(s.nominal_errors)), ...
    'diagnostic_orientation_mean_deg',mean(orientationDeg),'diagnostic_orientation_max_deg',max(orientationDeg), ...
    'position_J_rank_absolute_tolerance',1e-10,'nominal_position_J_rank',sum(singular0>1e-10), ...
    'truth_position_J_rank',sum(singularT>1e-10),'theta6_position_column_norm',norm(Jt(:,24)), ...
    'nominal_zero_only_rank',rank(J0(:,19:24),1e-10),'off_axis_zero_only_rank',rank(Joff(:,19:24),1e-10), ...
    'theta6_plus5deg_max_position_change_mm',max(vecnorm(changed6-trueTest,2,2))*1000, ...
    'theta6_plus5deg_max_orientation_change_deg',max(rotation_distances(trueTestR,changed6R))*180/pi, ...
    'd2_d3_both_plus3mm_max_position_change_mm',max(vecnorm(changedD-trueTrain,2,2))*1000, ...
    'rtb_vs_custom_nominal_first_pose_difference_m',norm(rtbPose.t'-nominalTrain(1,:)));
write_json(fullfile(outputDir,'metrics.json'),m);
jointNames=cellstr(compose('q%d_rad',1:6));
trainTable=array2table([s.Q,s.X_measured],'VariableNames',[jointNames,{'x_m','y_m','z_m'}]);
writetable(trainTable,fullfile(outputDir,'data','train_samples.csv'));
testTable=array2table([s.Q_test,s.X_test_true,s.X_test_identified,s.nominal_errors,s.position_errors,orientationDeg], ...
    'VariableNames',[jointNames,{'true_x_m','true_y_m','true_z_m','identified_x_m','identified_y_m','identified_z_m', ...
    'nominal_position_error_m','identified_position_error_m','diagnostic_orientation_error_deg'}]);
writetable(testTable,fullfile(outputDir,'data','test_samples.csv'));
parameterNames=[compose("alpha%d",(1:6)');compose("a%d",(1:6)');compose("d%d",(1:6)');compose("theta%d",(1:6)')];
units=[repmat("deg",6,1);repmat("mm",12,1);repmat("deg",6,1)];
unitScale=[ones(6,1)*180/pi;ones(12,1)*1000;ones(6,1)*180/pi];
parameterTable=table(parameterNames,units,s.delta_params_true(:).*unitScale,s.delta_params(:).*unitScale, ...
    (s.delta_params(:)-s.delta_params_true(:)).*unitScale, ...
    'VariableNames',{'parameter','unit','true_correction','identified_correction','identified_minus_true'});
writetable(parameterTable,fullfile(outputDir,'data','parameter_comparison.csv'));
historyTable=table((0:s.actual_iterations)',s.error_history*1000,[NaN;s.lambda_history], ...
    'VariableNames',{'iteration','position_rms_mm','lambda_recorded_after_iteration'});
writetable(historyTable,fullfile(outputDir,'data','iteration_history.csv'));
spectrum=table((1:24)',singular0,singularT,scaled0,scaledT, ...
    'VariableNames',{'index','nominal_raw','truth_raw','nominal_scaled','truth_scaled'});
writetable(spectrum,fullfile(outputDir,'diagnostics','position_jacobian_spectrum.csv'));
save(fullfile(outputDir,'diagnostics','posthoc_diagnostics.mat'),'J0','Jt','Joff','singular0','singularT','scaled0','scaledT','scales','orientationDeg','m');
render_original_diagnostics(outputDir);
fprintf('\nEXPORTED: %s\nTraining RMS: %.12g mm; test mean: %.12g mm; test max: %.12g mm\n', ...
    outputDir,m.final_train_rms_mm,m.identified_test_mean_mm,m.identified_test_max_mm);
fprintf('Diagnostic rank: nominal %d/24, truth %d/24. Orientation max: %.9g deg.\n', ...
    m.nominal_position_J_rank,m.truth_position_J_rank,m.diagnostic_orientation_max_deg);
diary off;
end

function [X,R]=positions(p,Q,tool)
N=size(Q,1);X=zeros(N,3,'like',p);R=zeros(3,3,N,'like',p);
for i=1:N
    T=eye(4);
    for j=1:6,T=T*transformation_mdh(p(j),p(6+j),p(12+j),p(18+j),1,Q(i,j));end
    x=T*[tool(:);1];X(i,:)=x(1:3).';R(:,:,i)=T(1:3,1:3);
end
end

function J=diagnostic_jacobian(p,Q,tool)
% Complex-step differentiation avoids real subtraction for this analytic FK.
% It is an independent diagnostic; main.m still uses its original 1e-8 FD.
h=1e-20;J=zeros(3*size(Q,1),24);
for k=1:24
    perturbed=complex(p);perturbed(k)=perturbed(k)+1i*h;
    X=positions(perturbed,Q,tool);J(:,k)=reshape(imag(X)',[],1)/h;
end
end

function angles=rotation_distances(A,B)
angles=zeros(size(A,3),1);
for i=1:size(A,3)
    R=A(:,:,i)'*B(:,:,i);
    skew=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]/2;
    angles(i)=atan2(norm(skew),(trace(R)-1)/2);
end
end

function export_figure(f,base)
drawnow;exportgraphics(f,[base,'.png'],'Resolution',160);savefig(f,[base,'.fig']);
end

function write_json(file,data)
fid=fopen(file,'w','n','UTF-8');assert(fid>=0,'Cannot write JSON.');
cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(data,'PrettyPrint',true));
end

function restore_session(folder,previousPath,visible,axesFont,textFont)
diary off;cd(folder);path(previousPath);
set(groot,'defaultFigureVisible',visible,'defaultAxesFontName',axesFont,'defaultTextFontName',textFont);
end
