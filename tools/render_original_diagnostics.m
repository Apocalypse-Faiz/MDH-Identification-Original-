function render_original_diagnostics(outputDir)
% Redraw supplementary figures from saved data; no optimization is performed.
s=load(fullfile(outputDir,'data','workspace.mat'));
d=load(fullfile(outputDir,'diagnostics','posthoc_diagnostics.mat'));
unitScale=[ones(6,1)*180/pi;ones(12,1)*1000;ones(6,1)*180/pi];
f=figure('Name','Parameter families in physical units','Color','w','Position',[100,100,1400,850],'Visible','off');
groups={1:6,7:12,13:18,19:24};labels={'alpha corrections (deg)','a corrections (mm)','d corrections (mm)','theta offsets (deg)'};
for i=1:4
    subplot(2,2,i);ids=groups{i};bar(1:6,[s.delta_params_true(ids)'.*unitScale(ids),s.delta_params(ids)'.*unitScale(ids)]);
    grid on;title(labels{i});xlabel('Joint / MDH row');legend('True','Identified','Location','best');
end
export_figure(f,fullfile(outputDir,'figures','parameter_families'));
f=figure('Name','Post-hoc diagnostics','Color','w','Position',[100,100,1400,850],'Visible','off');
subplot(2,2,1);semilogy(1:s.N_test,max(s.nominal_errors*1000,realmin),'o-');hold on;
semilogy(1:s.N_test,max(s.position_errors*1000,realmin),'o-');grid on;xlabel('Test sample');ylabel('Position error (mm)');legend('Nominal','Identified');
subplot(2,2,2);semilogy(0:s.actual_iterations,max(s.error_history*1000,realmin),'o-');grid on;xlabel('Iteration');ylabel('Training position RMS (mm)');
subplot(2,2,3);plot(1:s.N_test,d.orientationDeg,'o-');grid on;xlabel('Test sample');ylabel('Orientation difference (deg)');
ylim([0,max(0.05,1.2*max(d.orientationDeg))]);title('Post-hoc only; not optimized');
subplot(2,2,4);floorValue=1e-18;
semilogy(1:24,max(d.scaled0,floorValue),'o-');hold on;semilogy(1:24,max(d.scaledT,floorValue),'o-');
grid on;xlabel('Singular-value index');ylabel('Position response (m)');
ylim([floorValue/10,10*max([d.scaled0;d.scaledT])]);
title('Scaled J, values below 1e-18 shown at floor');legend('Nominal','Synthetic truth','Location','best');
export_figure(f,fullfile(outputDir,'figures','posthoc_diagnostics'));
end

function export_figure(f,base)
drawnow;exportgraphics(f,[base,'.png'],'Resolution',160);savefig(f,[base,'.fig']);
end
