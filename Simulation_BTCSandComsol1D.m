% Script: BTCS vs COMSOL vs Experiment 
clc; 
clear; 
close all;

%% 1. Geometry and Mesh
th_paste  = 100e-6;    % Thickness of thermal paste [m]
th_pc_bot = 250e-6;    % Thickness of bottom PC [m]
th_fluid  = 500e-6;   % Thickness of fluid layer [m]
th_pc_top = 1e-3;      % Thickness of top PC [m]
L_total   = th_paste + th_pc_bot + th_fluid + th_pc_top;

Nx = 1000;
dx = L_total / (Nx - 1);
x  = linspace(0, L_total, Nx);

%% 2. Material Properties
mat.paste = [2500, 1000];   % [density, Cp]
mat.pc    = [1280, 1350];
mat.fluid = [1020, 4250];

k_paste = 0.50;    % [W/m·K]
k_pc    = 0.10;    % [W/m·K]
k_fluid = 0.58;    % [W/m·K]

% Identify layer boundaries
x_paste_end  = th_paste;
x_pc_bot_end = x_paste_end + th_pc_bot;
x_fluid_end  = x_pc_bot_end + th_fluid;

% Build layer map: 1=paste, 2=bottom PC, 3=fluid, 4=top PC
layermap = zeros(size(x));
for i = 1:Nx
    if x(i) <= x_paste_end
        layermap(i) = 1;
    elseif x(i) <= x_pc_bot_end
        layermap(i) = 2;
    elseif x(i) <= x_fluid_end
        layermap(i) = 3;
    else
        layermap(i) = 4;
    end
end

% Assign density and specific heat from layer map
rho = zeros(size(x));
cp  = zeros(size(x));
for i = 1:Nx
    switch layermap(i)
        case 1
            prop = mat.paste;
        case 2
            prop = mat.pc;
        case 3
            prop = mat.fluid;
        case 4
            prop = mat.pc;
    end
    rho(i) = prop(1);
    cp(i)  = prop(2);
end

%% 3. Load Data
% Skip header (8 lines) for simulation files
data_peltier      = dlmread('peltier_data.txt',      '', 8, 0);
data_fluid_comsol = dlmread('1d1.txt',        '', 8, 0);
data_exp          = load('fluid_exp.txt');

time           = data_peltier(:,1);        % [s]
T_bottom_vec   = data_peltier(:,2);        % [K]
time_comsol    = data_fluid_comsol(:,1);   % [s]
T_fluid_comsol = data_fluid_comsol(:,2);   % [K]
time_exp       = data_exp(:,1);            % [s]
T_fluid_exp    = data_exp(:,2);            % [K]

% Convert experiment data to Kelvin if needed
if max(T_fluid_exp) < 200
    T_fluid_exp = T_fluid_exp + 273.15;
end

nt = length(time);

%% 4. Initial Condition
T = 273.15 + 25 * ones(Nx,1);  % Uniform 25°C [K]
% Set fluid region initial temperature to 34°C [K]
fluid_idx = find(x >= x_pc_bot_end & x <= x_fluid_end);
T(fluid_idx) = 273.15 + 34;

T_record = zeros(Nx, nt);

%% 5. Fluid Sampling Location (0.59 mm below top of fluid)
top_of_fluid_x         = x(fluid_idx(end));
x_sample               = top_of_fluid_x - 0.59e-3;
[~, idx_fluid_sample]  = min(abs(x - x_sample));

%% 6. Boundary & Thermal Conductivity
h     = 25;                  % Convective coefficient [W/m²·K]
T_inf = 273.15 + 25;         % Ambient temperature [K]

k = zeros(Nx,1);
for i = 1:Nx
    switch layermap(i)
        case 1
            k(i) = k_paste;
        case 2
            k(i) = k_pc;
        case 3
            k(i) = k_fluid;
        case 4
            k(i) = k_pc;
    end
end

%% 7. BTCS Time-Stepping Simulation
for t = 1:nt
    if t == 1
        dt = time(1);
    else
        dt = time(t) - time(t-1);
        if dt <= 0
            dt = 1e-3;
        end
    end
    
    T_prev = T;
    A = sparse(Nx,Nx);
    b = zeros(Nx,1);
    
    % Internal nodes
    for i = 2:Nx-1
        r1 = k(i-1)/dx;
        r2 = k(i)  /dx;
        aW = r1;
        aP = -(r1 + r2 + rho(i)*cp(i)*dx/dt);
        aE = r2;
        A(i,i-1) = aW;
        A(i,i)   = aP;
        A(i,i+1) = aE;
        b(i)     = -rho(i)*cp(i)*dx/dt * T_prev(i);
    end
    
    % Bottom Dirichlet BC
    A(1,1) = 1;
    b(1)   = T_bottom_vec(t);
    
    % Top convective BC: -k dT/dx = h (T - T_inf)
    A(end,end-1) =  k(end)/dx;
    A(end,end)   = -k(end)/dx - h;
    b(end)       = -h * T_inf;
    
    % Solve linear system
    T = A \ b;
    T_record(:,t) = T;
end

%% 8. Extract Simulated Fluid Temperature
T_fluid_sim = T_record(idx_fluid_sample, :);

%% 9. Plot Results
figure; hold on;
plot(time,           T_bottom_vec   - 273.15, 'k--', 'LineWidth', 1.2);
plot(time,           T_fluid_sim    - 273.15, 'm-',  'LineWidth', 1.5);
plot(time_comsol,    T_fluid_comsol , 'b-',  'LineWidth', 1.5);
plot(time_exp,       T_fluid_exp    - 273.15, 'r-',  'LineWidth', 1.2);

xlabel('Time [s]');
ylabel('Temperature [°C]');
legend({ ...
    'Peltier (input BC)', ...
    'Fluid (BTCS)', ...
    'Fluid (Comsol-1D)', ...
    'Fluid (Experiment)'}, ...
    'Location', 'best');
title('Comparison of Simulated and Experimental Fluid Temperatures');
grid on;
hold off;