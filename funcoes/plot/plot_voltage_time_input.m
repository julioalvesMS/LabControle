function plot_voltage_time_input(u, t, name)
%PLOT_VOLTAGE_CURRENT Plot Voltage x Current
%   Makes a Voltage X Current plot from the simulation output.
%   The image will be saved automatically if the folder is argument 
%   is given
    
    name = strcat(name, ' - Esforço de Controle');

    figure;
    
    plot(t, u);
    
    ylabel('Esforço de Controle [V]');
    xlabel('Tempo [s]');
    title(name);
    
    saveas(gcf, strcat('imagens/', name, '.png'));
end

