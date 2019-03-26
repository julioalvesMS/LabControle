function plot_voltage_time(y, t, name)
%PLOT_VOLTAGE_CURRENT Plot Voltage x Current
%   Makes a Voltage X Current plot from the simulation output.
%   The image will be saved automatically if the folder is argument 
%   is given
    
    name = strcat(name, ' - Saída de Tensão');

    figure;
    
    plot(t, y);
    
    ylabel('V [V]');
    xlabel('t [s]');
    title(name);
    
    saveas(gcf, strcat('imagens/', name, '.png'));
end

