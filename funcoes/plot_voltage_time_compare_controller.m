function plot_voltage_time_compare_controller(prop, zn, pid, t, name)
%PLOT_VOLTAGE_CURRENT Plot Voltage x Current
%   Makes a Voltage X Current plot from the simulation output.
%   The image will be saved automatically if the folder is argument 
%   is given
    
    name = strcat('Comparação dos Controladores - ', name);

    figure;
    hold all;
    
    plot(t, prop);
    plot(t, zn);
    plot(t, pid);
    
    ylabel('Tensão [V]');
    xlabel('Tempo [s]');
    title(name);
    
    legend('Proporcional', 'Ziegler-Nichols', 'PID');
    
    hold off;
    saveas(gcf, strcat('imagens/', name, '.png'));
end

