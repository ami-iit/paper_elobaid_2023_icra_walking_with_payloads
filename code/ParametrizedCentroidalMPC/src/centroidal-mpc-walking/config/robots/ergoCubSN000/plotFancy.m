% plotting script

clc; clearvars;
close all;
% myData = readtable('example.txt');   
% myData1 = readtable('old_dataSet/Aug_19_FirstExperiment.txt');
myData = readtable('example.txt');  



% getting data from robot

dt                   = 0.01; % WB Block frequency
CoM                  = [myData.com_x myData.com_y  myData.com_z];
CoM_d                = [myData.des_com_x myData.des_com_y myData.des_com_z];
nominal_left_steps   = [myData.lf_des_x myData.lf_des_y myData.lf_des_z];
nominal_right_steps  = [myData.rf_des_x myData.rf_des_y myData.rf_des_z];
measured_left_steps  = [myData.lf_meas_x myData.lf_meas_y myData.lf_meas_z];
measured_right_steps = [myData.rf_meas_x myData.rf_meas_y myData.rf_meas_z];
lforce_1 = [myData.contact_0_corner_0_x myData.contact_0_corner_0_y myData.contact_0_corner_0_z];
lforce_2 = [myData.contact_0_corner_1_x myData.contact_0_corner_1_y myData.contact_0_corner_1_z];
lforce_3 = [myData.contact_0_corner_2_x myData.contact_0_corner_2_y myData.contact_0_corner_2_z];
lforce_4 = [myData.contact_0_corner_3_x myData.contact_0_corner_3_y myData.contact_0_corner_3_z];

rforce_1 = [myData.contact_1_corner_0_x myData.contact_1_corner_0_y myData.contact_1_corner_0_z];
rforce_2 = [myData.contact_1_corner_1_x myData.contact_1_corner_1_y myData.contact_1_corner_1_z];
rforce_3 = [myData.contact_1_corner_2_x myData.contact_1_corner_2_y myData.contact_1_corner_2_z];
rforce_4 = [myData.contact_1_corner_3_x myData.contact_1_corner_3_y myData.contact_1_corner_3_z];

lforce   = [lforce_1';lforce_2';lforce_3';lforce_4'];
rforce   = [rforce_1';rforce_2';rforce_3';rforce_4'];


lforce_z = [lforce(3,:) ; lforce(6,:); lforce(9,:) ;lforce(12,:)];
rforce_z = [rforce(3,:) ; rforce(6,:); rforce(9,:); rforce(12,:)];


computation_time = myData.computational_time*10^-3;
% computation_time_nonParametrized = myData1.computational_time*10^-3;


% %% PLOT FOOTPATHS
% % plot left footpath
% 
%  
% set(0,'DefaultLegendAutoUpdate','off')
% filename = 'centroidal_iCubGenova09.gif';
% h = figure('Renderer', 'painters');
% hold on; grid on;
% 
% xlim([-0.2  1.0]);
% ylim([-0.3  0.3]);
% 
% t     = 0:dt:length(CoM)*dt;
% 
% left_      = plot_footpath(nominal_left_steps(1,:)' , 0,'#3da4ab');
% right_     = plot_footpath(nominal_right_steps(1,:)' , 0,'#3da4ab');
% l_adapted_ = plot_footpath(measured_left_steps(1,:)' , 0,'#FFA500');
% r_adapted_ = plot_footpath(measured_right_steps(1,:)' , 0,'#FFA500');
% 
% anim = animatedline('Color', '#CB7266', 'LineWidth',2);
% frame = getframe(h);
% im = frame2im(frame);
% [imind,cm] = rgb2ind(im,256);
% 
% com = CoM;
% addpoints(anim, com(1, 1), com(1, 2));
% 
% 
% legend([left_, l_adapted_, anim], {'Nominal', 'Adapted', 'CoM'});
% drawnow;
% imwrite(imind,cm,filename,'gif', 'Loopcount',inf, 'DelayTime', 0.1);
% 
% 
% for i=1:length(CoM)
%     if (min(lforce_z(:,i)) > 1)
%         left_      = plot_footpath(nominal_left_steps(i,:)' , 0,'#3da4ab');
%         l_adapted_ = plot_footpath(measured_left_steps(i,:)' , 0,'#FFA500');
%     elseif (min(rforce_z(:,i)) > 1)
%         right_     = plot_footpath(nominal_right_steps(i,:)' , 0,'#3da4ab');
%         r_adapted_ = plot_footpath(measured_right_steps(i,:)' , 0,'#FFA500');
%     end
%     
%     addpoints(anim, com(i, 1), com(i, 2));
% 
%     drawnow;
%     frame = getframe(h);
%     im = frame2im(frame);
%     [imind,cm] = rgb2ind(im,256);
%        
%     imwrite(imind,cm,filename,'gif','WriteMode','append', 'DelayTime', 0.1);
% end


% pause(0.01)

       



%% 
% 
% foot_transform = hgtransform;
% foot_transform.Matrix = transforms(:,:,1);
% p = patch('XData',x,'YData',y,'FaceColor','#fe8a71','Parent',foot_transform);



t     = 0:dt:length(CoM)*dt;
% to    = 0:dt:(length(computation_time_nonParametrized)-1)*dt;
t(end) = [];

figure('name','CoM on the plane')
subplot(2,1,1);
hold on; grid on;
plot(t, CoM(:,1),  'LineWidth', 2);
plot(t, CoM_d(:,1),  'LineWidth', 2);
% plot(qas, qas_d, '--','color','#D95319', 'LineWidth', 2);
l = xlabel('Time',  'FontSize', 30); 
set(l,'Interpreter','Latex');
l = ylabel('$x$ component of $\mathcal{P}_{CoM}$', 'FontSize', 30); 
set(l,'Interpreter','Latex');
l = legend('Actual','Desired', 'FontSize', 30);
set(l,'Interpreter','Latex');


subplot(2,1,2);
hold on; grid on;
plot(t, CoM(:,2),  'LineWidth', 2);
plot(t, CoM_d(:,2),  'LineWidth', 2);
% plot(qas, qas_d, '--','color','#D95319', 'LineWidth', 2);
l = xlabel('Time',  'FontSize', 30); 
set(l,'Interpreter','Latex');
l = ylabel('$y$ component of $\mathcal{P}_{CoM}$', 'FontSize', 30); 
set(l,'Interpreter','Latex');
l = legend('Actual','Desired', 'FontSize', 30);
set(l,'Interpreter','Latex');



figure('name','CoM tracking error')

hold on; grid on;
plot(t, CoM(:,1) - CoM_d(:,1), 'LineWidth', 2);
plot(t, CoM(:,2) - CoM_d(:,2),  'LineWidth', 2);
l = xlabel('Time',  'FontSize', 30); 
set(l,'Interpreter','Latex');
l = ylabel('$\|\mathcal{P}_{CoM} - \mathcal{P}_{CoM_d}\|$', 'FontSize', 30); 
set(l,'Interpreter','Latex');
l = legend('$x$ component CoM tracking  error', '$y$ component CoM tracking  error','FontSize', 30);
set(l,'Interpreter','Latex');


figure('name','Contact location tracking error')
subplot(2,1,1);
hold on; grid on;
plot(t, measured_left_steps(:,1) - nominal_left_steps(:,1), 'LineWidth', 2);
plot(t, measured_left_steps(:,2) - nominal_left_steps(:,2), 'LineWidth', 2);
l = xlabel('Time',  'FontSize', 30); 
set(l,'Interpreter','Latex');
l = ylabel('$|\mathcal{P}_{C} - \mathcal{P}_{C_d}|$', 'FontSize', 30); 
set(l,'Interpreter','Latex');
l = legend('$x$ component CoM tracking  error', '$y$ component CoM tracking  error','FontSize', 30);
set(l,'Interpreter','Latex');

subplot(2,1,2);
hold on; grid on;
plot(t, measured_right_steps(:,1) - nominal_right_steps(:,1), 'LineWidth', 2);
plot(t, measured_right_steps(:,2) - nominal_right_steps(:,2), 'LineWidth', 2);
l = xlabel('Time',  'FontSize', 30); 
set(l,'Interpreter','Latex');
l = ylabel('$|\mathcal{P}_{C} - \mathcal{P}_{C_d}|$', 'FontSize', 30); 
set(l,'Interpreter','Latex');
l = legend('$x$ component CoM tracking  error', '$y$ component CoM tracking  error','FontSize', 30);
set(l,'Interpreter','Latex');


figure('name','Contact location tracking error')
subplot(2,1,1);
hold on; grid on;
plot(t, measured_left_steps(:,1) - nominal_left_steps(:,1), 'LineWidth', 2);
plot(t, measured_left_steps(:,2) - nominal_left_steps(:,2), 'LineWidth', 2);
l = xlabel('Time',  'FontSize', 30); 
set(l,'Interpreter','Latex');
l = ylabel('Left foot', 'FontSize', 30); 
set(l,'Interpreter','Latex');
l = legend('$x - x^n$  tracking error', '$y-y^n$  tracking  error','FontSize', 30);
set(l,'Interpreter','Latex');

subplot(2,1,2);
hold on; grid on;
plot(t, measured_right_steps(:,1) - nominal_right_steps(:,1), 'LineWidth', 2);
plot(t, measured_right_steps(:,2) - nominal_right_steps(:,2), 'LineWidth', 2);
l = xlabel('Time',  'FontSize', 30); 
set(l,'Interpreter','Latex');
l = ylabel('Right foot', 'FontSize', 30); 
set(l,'Interpreter','Latex');
l = legend('$x - x^n$  tracking error', '$y-y^n$  tracking  error','FontSize', 30);
set(l,'Interpreter','Latex');


figure('name','Computational time')

hold on; grid on;
plot(t, computation_time(1:length(computation_time)), 'LineWidth', 2);
% plot(to, computation_time_nonParametrized, 'r', 'LineWidth', 2);
plot(t, mean(computation_time)*ones(length(t)),'LineWidth', 2);
% plot(to, mean(computation_time_nonParametrized)*ones(length(to)), 'b--' ,'LineWidth', 2);
l = xlabel('Time',  'FontSize', 30); 
set(l,'Interpreter','Latex');
l = ylabel('Compuatational time [ms]', 'FontSize', 30); 
set(l,'Interpreter','Latex');
% l = legend('Parametrized MPC', 'Non parametrized', 'Average parametrized', 'Average non parametrized', 'FontSize', 30);
l = legend('Parametrized MPC', 'Average parametrized',  'FontSize', 30);
set(l,'Interpreter','Latex');
% 
function p = plot_footpath(pos, angle, color)

%% SET FOOT SIZE
x = [-0.08, -0.08, 0.08, 0.08];
y = [-0.03,  0.03, 0.03, -0.03];

%% PLOT FOOTPATHS
    % plot left footpath
    left_foot_transform = hgtransform;
    left_foot_transform.Matrix = makehgtform('translate', pos,...
                                             'zrotate', angle);
    p = patch('XData',x,'YData',y,'FaceColor',color,'Parent',left_foot_transform);
    alpha(p, 0.4);    
end

function plot_aesthetic(Title, Label_x, Label_y, Label_z, varargin)
% PLOT_AESTHETIC add Title, label and legends in a plot
%   PLOT_AESTHETIC(Title, Label_x, Label_y, Label_z, Legend_1, ..., Legend_n)
%   add title, labels and legends in a plot. LaTex syntax is allowed.

% set labels
if ~isempty(Label_x)
    x_label = xlabel(Label_x);
    set(x_label, 'Interpreter', 'latex', 'FontSize', 25);
end

if ~isempty(Label_y)
    y_label = ylabel(Label_y);
    set(y_label,'Interpreter','latex', 'FontSize', 25);
end

if ~isempty(Label_z)
    z_label = zlabel(Label_z);
    set(z_label,'Interpreter','latex');
    set(z_label,'FontSize', 20);
end

% set legend
if ~isempty(varargin)
    % get the legend object
    leg = get(legend(gca),'String');
    
    % if the legend does not exist create a new one
    
    for i = 1:length(varargin)
        varargin(i) = strrep(varargin(i),'_',' ');
    end
    
    if isempty(leg)
        new_legend = varargin;
    else
        old_legend = leg;
        % when a new plot is draw an automatic string is added to the
        % legend
        new_legend = [old_legend(1:end-1), varargin{:}];
    end
      % h = legend(varargin, 'Location', 'northoutside', 'Orientation','horizontal');
    h = legend(varargin, 'Location', 'best');
    set(h,'Interpreter','latex')
    set(h,'FontSize', 16);
end

% change linewidth
h = findobj(gcf,'type','line');
set(h,'linewidth',2)

% set the title
if ~isempty(Title)
    tit = title(Title);
    set(tit,'FontSize', 20);
    set(tit,'Interpreter','latex');
end

% change font size
set(gca,'FontSize', 18)

% set grid
grid on;
end

function [q] = plot_forces(foot_pos, forces)

R = eye(3);

[U,V] = meshgrid(linspace(-0.1, 0.1, 2),linspace(-0.05, 0.05, 2));
W = 0 * U;
POINT = zeros(size(U,1), size(U,2), 3);
F = zeros(size(U,1), size(U,2), 3);

k = 1;

for i = 1:size(U,1)
    for j = 1:size(U,2)
        F(i,j,:) = forces(:,k);
       
        
        point = foot_pos + R * [U(i,j); V(i,j); 0];
        POINT(i,j,1) = point(1);
        POINT(i,j,2) = point(2);
        POINT(i,j,3) = point(3);
        
        k= k+1;
    end
end
q = quiverC3D(POINT(:,:,1), POINT(:,:,2), POINT(:,:,3),...
    F(:,:,1), F(:,:,2), F(:,:,3));


end

function [ quivs ] = quiverC3D(x,y,z,u,v,w,maxNumArrows)
%quiverC3D creates a 3D quiver plot and adds a color coding. The color coding is
%given by the absolut values of the component vectors. Large values result in colors 
%from the upper end of the used colormap. Plotting parameters have to be changed within 
%the function in this version. Values equal to NaN or inf are set to zero.
%In case of complex arrays the absolute value is used.
% 
%   INPUT:
%       x - 2D matrix, x components of initial points
%       y - 2D matrix, y components of initial points
%       z - 2D matrix, z components of initial points
%       u - 2D matrix, x components of arrows
%       v - 2D matrix, y components of arrows
%       w - 2D matrix, z components of arrows
%       maxNumArrows - a positive integer (non-integer should work as well)
%           number limiting the maximum number of plotted arrows. Since vectors
%           of length zero aren't plotted and the matrices are sampled
%           uniformly, it's possible that fewer arrows will be visible in the
%           end. If maxNumArrows is not given its value is set to 1000.
% 
%   OUTPUT:
%       none
% 
%   WARNING!: Using large datasets in combination with choosing maxNumArrows 
%       very large might result in this script running forever.
% 
% --------------------------------------------------------------------------------------
% 
%   EXAMPLE:
%       [x,y] = meshgrid(linspace(0,10,100),linspace(0,10,100));
%       z = sin(x.^2 + y.^2);
%       u = exp(-0.2*(x-5).^2 - 0.2*(y-5).^2);
%       v = -u;
%       w = u.*v;
%       quiverC3D(x,y,z,u,v,w,500);
%   
% --------------------------------------------------------------------------------------
% 
%   See also: QUIVER3, LINESPEC, COLORMAP.
% 

%% prearrangements

narginchk(6,7);
n_in = nargin;

if ~isequal(size(x),size(y),size(z),size(u),size(v),size(w))
    error('X,Y,Z,U,V,W have to be matrices of the same size.');
end


%% assignments

% maximum number of arrows if necessary
if n_in == 6
    maxNumArrows = 1000;
end
% Line width
lw = 2.5;
% Maximum of arrow head sizeP
hs = 0.5;
% Colormap
colormap jet;

%% initialization
if numel(u) > maxNumArrows
    N = ceil(sqrt(numel(u)/maxNumArrows));
    
    x = x(1:N:end,1:N:end);
    y = y(1:N:end,1:N:end);
    z = z(1:N:end,1:N:end);
    u = u(1:N:end,1:N:end);
    v = v(1:N:end,1:N:end);
    w = w(1:N:end,1:N:end);
end

s = size(u);

%% taking care of possible issues

x = issues(x);
y = issues(y);
z = issues(z);
u = issues(u);
v = issues(v);
w = issues(w);

% VECTORS_NORMS = sqrt(u.^2 + v.^2 + w.^2);
% v_max_norm = max(VECTORS_NORMS);
% 
% if ~isequal(u,zeros(s)) 
%     u = u./v_max_norm;
% end
% if ~isequal(v,zeros(s))
%     v = v./v_max_norm;
% end
% if ~isequal(w,zeros(s))
%     w = w./v_max_norm;
% end



%% colormap definition
I = sqrt(u.^2 + v.^2 + w.^2);
Ic = round(I/max(max(I))*200);
Ic( Ic == 0) = 1;
C = colormap;
%% plotting
hold on;
quivs =[];
for n = 1:s(1)
    for m = 1:s(2)
       % if u(n,m) > 0 || v(n,m) > 0 
           quivs =[quivs,  quiver3(x(n,m),y(n,m),z(n,m),u(n,m),v(n,m),w(n,m),'Color',C(Ic(n,m),:),'LineWidth',lw,'maxheadsize',hs)];
       % end
    end
end


hold off;

end


function [ u ] = issues( u )
%issues avoids possible problems that might occur when trying to do a quiver
%plot of the given input data u.
%   
%   NaN and inf values are set to zero.
%   In case of complex numbers - replacement by absolute value
% 
%   INPUT:
%       u - scalar value or array of arbitrary dimensions
% 
%   OUTPUT:
%       u - scalar value or array of arbitrary dimensions
% 


narginchk(1,1);

try
    u(isnan(u)) = 0;
    u(isinf(u)) = 0;
    if ~isreal(u)
        u = abs(u);
    end
catch
    error('Unable to perform operations on input data.');
end


end